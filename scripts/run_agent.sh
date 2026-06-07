#!/bin/bash
# =============================================================================
# run_agent.sh — LLM Agent 실행 (터미널 2)
#
# run_stack.sh로 시뮬레이션 스택이 기동된 후 실행합니다.
# Gemini API 키는 WS/.env 또는 환경 변수 GEMINI_API_KEY로 설정합니다.
#
# 사용법:
#   bash scripts/run_agent.sh
# =============================================================================
set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
ENV_FILE="$WS_DIR/.env"

echo -e "\n${BLUE}${BOLD}UR3 Pick & Place — LLM Agent 실행${NC}\n"

# ── 환경 확인 ─────────────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
    log_error "ROS 2 Humble 없음"
    exit 1
fi

INSTALL_SETUP="$WS_DIR/install/setup.bash"
if [ ! -f "$INSTALL_SETUP" ]; then
    log_error "워크스페이스 빌드 필요: bash $SCRIPT_DIR/build.sh"
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$INSTALL_SETUP"

# ── API 키 확인 ────────────────────────────────────────────────
if [ -f "$ENV_FILE" ]; then
    # .env 파일 로드 (GEMINI_API_KEY= 만 있는 경우 경고)
    # shellcheck source=/dev/null
    set -o allexport; source "$ENV_FILE"; set +o allexport
fi

if [ -z "${GEMINI_API_KEY:-}" ]; then
    log_error "GEMINI_API_KEY가 설정되지 않았습니다."
    echo ""
    echo "  방법 1 — .env 파일:"
    echo "    echo 'GEMINI_API_KEY=<your_key>' > $ENV_FILE"
    echo ""
    echo "  방법 2 — 환경 변수:"
    echo "    export GEMINI_API_KEY=<your_key>"
    echo "    bash $0"
    exit 1
fi

# ── /moveit/execute 서비스 대기 ────────────────────────────────
log_info "Moveit_module 서비스 대기 중..."
TIMEOUT=60
elapsed=0
while ! ros2 service list 2>/dev/null | grep -q "/moveit/execute"; do
    if [ $elapsed -ge $TIMEOUT ]; then
        log_warn "타임아웃: /moveit/execute 서비스를 찾을 수 없습니다."
        echo "  run_stack.sh가 실행 중인지 확인하세요."
        echo "  그래도 계속 진행합니다..."
        break
    fi
    echo -ne "\r  /moveit/execute 대기 중... ${elapsed}s / ${TIMEOUT}s"
    sleep 2
    elapsed=$((elapsed + 2))
done
echo ""
log_info "/moveit/execute 서비스 감지 완료"

# ── 실행 ──────────────────────────────────────────────────────
echo ""
echo -e "${CYAN}대화 예시:${NC}"
echo -e "  > 콜라캔 집어줘"
echo -e "  > pick up the coke can"
echo -e "  > 선반에 있는 병 집어서 옮겨줘"
echo ""
echo -e "종료: ${BOLD}Ctrl+C${NC}  또는  ${BOLD}quit / exit${NC}"
echo ""

cd "$WS_DIR"   # .env 탐색 경로 (CWD 기준)
ros2 run llm_agent agent_node
