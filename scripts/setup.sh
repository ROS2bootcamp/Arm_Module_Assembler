#!/bin/bash
# =============================================================================
# setup.sh — UR3 Pick & Place 통합 워크스페이스 초기 설정 (1회 실행)
#
# 사용법:
#   git clone https://github.com/ROS2bootcamp/Arm_Module_Assembler.git
#   cd Arm_Module_Assembler
#   bash scripts/setup.sh
#
# 수행 작업:
#   1. ROS 2 Humble 설치 확인
#   2. 시스템 의존성(apt) 설치
#   3. Python 패키지 설치
#   4. 통합 브랜치 레포 클론 / 업데이트
#   5. 워크스페이스 빌드
#   6. .env (Gemini API 키) 설정
#   7. ~/.bashrc 환경 변수 등록
# =============================================================================
set -euo pipefail

# ── 색상 출력 ─────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'
BOLD='\033[1m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${BLUE}${BOLD}═══ $* ═══${NC}"; }
log_ok()    { echo -e "${GREEN}${BOLD}  ✔ $*${NC}"; }

# ── 경로 설정 ─────────────────────────────────────────────────
# 스크립트 위치 기준으로 워크스페이스 루트 결정
# 구조: <WS>/Arm_Module_Assembler/scripts/setup.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"           # .../Arm_Module_Assembler
WS_DIR="$(dirname "$REPO_DIR")"               # 워크스페이스 루트
ENV_FILE="$WS_DIR/.env"

echo -e "\n${CYAN}${BOLD}UR3 Pick & Place — 통합 환경 설정${NC}"
echo -e "${CYAN}워크스페이스: ${BOLD}$WS_DIR${NC}\n"

# ── Step 1: ROS 2 Humble 확인 ─────────────────────────────────
log_step "Step 1: ROS 2 Humble 확인"
if [ ! -f /opt/ros/humble/setup.bash ]; then
    log_error "ROS 2 Humble이 설치되지 않았습니다."
    echo "  설치 가이드: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html"
    exit 1
fi
log_ok "ROS 2 Humble 확인됨"

# shellcheck source=/opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash

# ── Step 2: apt 패키지 설치 ────────────────────────────────────
log_step "Step 2: 시스템 패키지 설치 (apt)"

APT_PACKAGES=(
    # MoveIt2 + MTC
    ros-humble-moveit
    ros-humble-moveit-task-constructor-core
    ros-humble-moveit-task-constructor-demo
    ros-humble-moveit-task-constructor-capabilities
    # UR3 시뮬레이션
    ros-humble-ur-simulation-gz
    # Gazebo / ros_gz
    ros-humble-ros-gz-bridge
    ros-humble-ros-gz-sim
    ros-humble-gz-ros2-control
    # 로봇 그리퍼
    ros-humble-robotiq-description
    # 비전 / TF
    ros-humble-cv-bridge
    ros-humble-tf2-ros
    ros-humble-robot-state-publisher
    # colcon 빌드 도구
    python3-colcon-common-extensions
)

log_info "패키지 목록 업데이트..."
sudo apt-get update -qq

MISSING_PKGS=()
for pkg in "${APT_PACKAGES[@]}"; do
    if ! dpkg -l "$pkg" &>/dev/null; then
        MISSING_PKGS+=("$pkg")
    fi
done

if [ ${#MISSING_PKGS[@]} -eq 0 ]; then
    log_ok "모든 apt 패키지 이미 설치됨"
else
    log_info "설치 중: ${MISSING_PKGS[*]}"
    sudo apt-get install -y "${MISSING_PKGS[@]}"
    log_ok "apt 패키지 설치 완료"
fi

# ── Step 3: Python 패키지 설치 ────────────────────────────────
log_step "Step 3: Python 패키지 설치 (pip)"

PIP_PACKAGES=(
    "google-genai"
    "python-dotenv"
    "ultralytics"
)

MISSING_PIP=()
for pkg in "${PIP_PACKAGES[@]}"; do
    pkg_name="${pkg%[><=]*}"    # 버전 지정자 제거
    if ! pip3 show "$pkg_name" &>/dev/null; then
        MISSING_PIP+=("$pkg")
    fi
done

if [ ${#MISSING_PIP[@]} -eq 0 ]; then
    log_ok "모든 Python 패키지 이미 설치됨"
else
    log_info "설치 중: ${MISSING_PIP[*]}"
    pip3 install "${MISSING_PIP[@]}"
    log_ok "Python 패키지 설치 완료"
fi

# ── Step 4: 레포 클론 / 업데이트 ──────────────────────────────
log_step "Step 4: 통합 레포 클론 / 업데이트"

# clone_or_update <url> <branch> <target_dir>
clone_or_update() {
    local url="$1"
    local branch="$2"
    local target="$3"
    local name
    name="$(basename "$target")"

    if [ -d "$target/.git" ]; then
        log_info "$name — 이미 존재, 업데이트 중..."
        cd "$target"
        current_branch="$(git branch --show-current 2>/dev/null || echo '')"
        if [ "$current_branch" != "$branch" ]; then
            log_warn "  브랜치 전환: $current_branch → $branch"
            git fetch origin
            git checkout "$branch"
        fi
        git pull origin "$branch"
        cd "$WS_DIR"
    else
        log_info "$name — 클론 중 (브랜치: $branch)..."
        git clone -b "$branch" "$url" "$target"
    fi
    log_ok "$name @ $branch"
}

cd "$WS_DIR"
clone_or_update \
    "https://github.com/ROS2bootcamp/PANDA_ENV.git" \
    "fix/srdf-group-name" \
    "$WS_DIR/PANDA_ENV"

clone_or_update \
    "https://github.com/ROS2bootcamp/Moveit_module.git" \
    "feat/service-interface" \
    "$WS_DIR/Moveit_module"

clone_or_update \
    "https://github.com/ROS2bootcamp/LLM_Agent.git" \
    "fix/agent-frame-params" \
    "$WS_DIR/LLM_Agent"

clone_or_update \
    "https://github.com/ROS2bootcamp/ROBOT_VISION.git" \
    "feat/integration" \
    "$WS_DIR/ROBOT_VISION"

clone_or_update \
    "https://github.com/ROS2bootcamp/UR3_CONVENIENCE_ENV.git" \
    "feat/model-dirs-and-integrated-world" \
    "$WS_DIR/UR3_CONVENIENCE_ENV"

# ── Step 5: 워크스페이스 빌드 ─────────────────────────────────
log_step "Step 5: 워크스페이스 빌드"
cd "$WS_DIR"

log_info "llm_agent_msgs 먼저 빌드 (의존 패키지)..."
colcon build \
    --packages-select llm_agent_msgs \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | grep -E "Starting|Finished|Failed|ERROR|error:" || true

log_info "나머지 패키지 빌드..."
colcon build \
    --packages-select \
        ur3_mtc_pick_place \
        ur3_moveit_module \
        robot_vision \
        llm_agent \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | grep -E "Starting|Finished|Failed|ERROR|error:" || true

# 빌드 결과 확인
FAILED=$(colcon list --packages-select \
    llm_agent_msgs ur3_mtc_pick_place ur3_moveit_module robot_vision llm_agent \
    2>/dev/null | wc -l)
if [ -f "$WS_DIR/install/setup.bash" ]; then
    log_ok "빌드 완료"
    # shellcheck source=/dev/null
    source "$WS_DIR/install/setup.bash"
else
    log_error "빌드 실패 — install/setup.bash가 생성되지 않았습니다."
    echo "  다음 명령으로 상세 오류를 확인하세요:"
    echo "  cd $WS_DIR && colcon build --packages-select llm_agent_msgs --symlink-install"
    exit 1
fi

# ── Step 6: .env (API 키) 설정 ────────────────────────────────
log_step "Step 6: Gemini API 키 설정"

if [ -f "$ENV_FILE" ] && grep -q "GEMINI_API_KEY" "$ENV_FILE"; then
    log_ok ".env 파일 이미 존재 — 건너뜀"
    log_warn "  키 변경이 필요하면: nano $ENV_FILE"
else
    echo ""
    echo -e "${YELLOW}Gemini API 키를 입력하세요.${NC}"
    echo -e "  발급 주소: ${CYAN}https://aistudio.google.com/app/apikey${NC}"
    echo -e "  (입력 내용은 화면에 표시되지 않습니다)"
    echo ""
    read -rsp "  GEMINI_API_KEY= " api_key
    echo ""

    if [ -z "$api_key" ]; then
        log_warn "API 키를 입력하지 않았습니다. 나중에 수동으로 설정하세요:"
        echo "  echo 'GEMINI_API_KEY=<your_key>' > $ENV_FILE"
        # 빈 템플릿 생성
        echo "GEMINI_API_KEY=" > "$ENV_FILE"
    else
        echo "GEMINI_API_KEY=$api_key" > "$ENV_FILE"
        chmod 600 "$ENV_FILE"
        log_ok ".env 파일 생성: $ENV_FILE"
    fi
fi

# ── Step 7: ~/.bashrc 환경 변수 등록 ──────────────────────────
log_step "Step 7: 환경 변수 ~/.bashrc 등록"

BASHRC="$HOME/.bashrc"
MARKER="# ── UR3 Pick & Place Integration ──"

if grep -q "UR3_CONVENIENCE_ENV_PATH" "$BASHRC"; then
    log_ok "~/.bashrc 이미 설정됨 — 건너뜀"
else
    cat >> "$BASHRC" << EOF

$MARKER
source /opt/ros/humble/setup.bash
export UR3_CONVENIENCE_ENV_PATH=$WS_DIR/UR3_CONVENIENCE_ENV
source $WS_DIR/install/setup.bash 2>/dev/null || true
EOF
    log_ok "~/.bashrc 에 환경 변수 추가됨"
fi

# ── 완료 메시지 ───────────────────────────────────────────────
echo ""
echo -e "${GREEN}${BOLD}════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}${BOLD}  ✅  설정 완료!${NC}"
echo -e "${GREEN}${BOLD}════════════════════════════════════════════════════${NC}"
echo ""
echo -e "  워크스페이스: ${BOLD}$WS_DIR${NC}"
echo ""
echo -e "${BOLD}다음 단계 — 테스트 실행:${NC}"
echo ""
echo -e "  ${CYAN}터미널 1${NC} (시뮬레이션 전체 스택):"
echo -e "  ${YELLOW}  bash $SCRIPT_DIR/run_stack.sh${NC}"
echo ""
echo -e "  ${CYAN}터미널 2${NC} (LLM Agent — 스택 기동 후 실행):"
echo -e "  ${YELLOW}  bash $SCRIPT_DIR/run_agent.sh${NC}"
echo ""
echo -e "  새 터미널에서 변경 사항 적용: ${YELLOW}source ~/.bashrc${NC}"
echo ""
