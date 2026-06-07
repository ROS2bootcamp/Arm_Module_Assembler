#!/bin/bash
# =============================================================================
# setup.sh — UR3 Pick & Place 초기 설정 (1회 실행)
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
#   4. 워크스페이스 빌드 (모노레포 — src/ 내 전체 패키지)
#   5. .env (Gemini API 키) 설정
#   6. ~/.bashrc 환경 변수 등록
#
# 이전 버전의 "8개 레포 클론" 단계 불필요 — 모든 소스가 src/ 에 포함됨.
# =============================================================================
set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${BLUE}${BOLD}═══ $* ═══${NC}"; }
log_ok()    { echo -e "${GREEN}${BOLD}  ✔ $*${NC}"; }

# 워크스페이스 = Arm_Module_Assembler 자체 (src/ 가 내부에 있음)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"           # scripts/ 한 단계 위 = 레포 루트
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
source /opt/ros/humble/setup.bash

# ── Step 2: apt 패키지 설치 ────────────────────────────────────
log_step "Step 2: 시스템 패키지 설치 (apt)"

APT_PACKAGES=(
    ros-humble-moveit
    ros-humble-moveit-task-constructor-core
    ros-humble-moveit-task-constructor-demo
    ros-humble-moveit-task-constructor-capabilities
    ros-humble-ur-simulation-gz
    ros-humble-ros-gz-bridge
    ros-humble-ros-gz-sim
    ros-humble-gz-ros2-control
    ros-humble-robotiq-description
    ros-humble-cv-bridge
    ros-humble-tf2-ros
    ros-humble-robot-state-publisher
    python3-colcon-common-extensions
)

sudo apt-get update -qq

MISSING_PKGS=()
for pkg in "${APT_PACKAGES[@]}"; do
    dpkg -l "$pkg" &>/dev/null || MISSING_PKGS+=("$pkg")
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

PIP_PACKAGES=("google-genai" "python-dotenv" "ultralytics")
MISSING_PIP=()
for pkg in "${PIP_PACKAGES[@]}"; do
    pip3 show "${pkg%[><=]*}" &>/dev/null || MISSING_PIP+=("$pkg")
done

if [ ${#MISSING_PIP[@]} -eq 0 ]; then
    log_ok "모든 Python 패키지 이미 설치됨"
else
    log_info "설치 중: ${MISSING_PIP[*]}"
    pip3 install "${MISSING_PIP[@]}"
    log_ok "Python 패키지 설치 완료"
fi

# ── Step 4: 워크스페이스 빌드 ─────────────────────────────────
log_step "Step 4: 워크스페이스 빌드"
cd "$WS_DIR"

log_info "llm_agent_msgs 먼저 빌드 (의존 패키지)..."
colcon build \
    --packages-select llm_agent_msgs \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+ || {
    log_error "llm_agent_msgs 빌드 실패"; exit 1
}

log_info "나머지 패키지 빌드..."
colcon build \
    --packages-select \
        ur3_mtc_pick_place \
        ur3_moveit_module \
        robot_vision \
        llm_agent \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+ || {
    log_error "패키지 빌드 실패"
    echo "  상세 로그: $WS_DIR/log/latest_build/"
    exit 1
}

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    log_error "빌드 실패 — install/setup.bash 없음"
    exit 1
fi

source "$WS_DIR/install/setup.bash"
log_ok "빌드 완료"

# ── Step 5: .env (API 키) 설정 ────────────────────────────────
log_step "Step 5: Gemini API 키 설정"

if [ -f "$ENV_FILE" ] && grep -q "GEMINI_API_KEY=." "$ENV_FILE"; then
    log_ok ".env 파일 이미 존재 — 건너뜀"
    log_warn "  키 변경: nano $ENV_FILE"
else
    echo ""
    echo -e "${YELLOW}Gemini API 키를 입력하세요.${NC}"
    echo -e "  발급: ${CYAN}https://aistudio.google.com/app/apikey${NC}"
    echo -e "  (입력 내용은 화면에 표시되지 않습니다)"
    echo ""
    read -rsp "  GEMINI_API_KEY= " api_key
    echo ""

    if [ -z "$api_key" ]; then
        log_warn "API 키를 입력하지 않았습니다. 나중에 설정:"
        echo "  echo 'GEMINI_API_KEY=<your_key>' > $ENV_FILE"
        echo "GEMINI_API_KEY=" > "$ENV_FILE"
    else
        echo "GEMINI_API_KEY=$api_key" > "$ENV_FILE"
        chmod 600 "$ENV_FILE"
        log_ok ".env 생성: $ENV_FILE"
    fi
fi

# ── Step 6: ~/.bashrc 등록 ────────────────────────────────────
log_step "Step 6: ~/.bashrc 환경 변수 등록"

BASHRC="$HOME/.bashrc"
MARKER="# ── Arm_Module_Assembler (UR3 Pick & Place) ──"

if grep -q "Arm_Module_Assembler" "$BASHRC" 2>/dev/null; then
    log_ok "~/.bashrc 이미 설정됨 — 건너뜀"
else
    cat >> "$BASHRC" << EOF

$MARKER
source /opt/ros/humble/setup.bash
source $WS_DIR/install/setup.bash 2>/dev/null || true
EOF
    log_ok "~/.bashrc 업데이트됨"
fi

# ── 완료 ──────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}${BOLD}════════════════════════════════════════════${NC}"
echo -e "${GREEN}${BOLD}  ✅  설정 완료!${NC}"
echo -e "${GREEN}${BOLD}════════════════════════════════════════════${NC}"
echo ""
echo -e "  워크스페이스: ${BOLD}$WS_DIR${NC}"
echo ""
echo -e "${BOLD}테스트 실행:${NC}"
echo -e "  터미널 1: ${YELLOW}bash $SCRIPT_DIR/run_stack.sh${NC}"
echo -e "  터미널 2: ${YELLOW}bash $SCRIPT_DIR/run_agent.sh${NC}"
echo ""
echo -e "  새 터미널: ${YELLOW}source ~/.bashrc${NC}"
echo ""
