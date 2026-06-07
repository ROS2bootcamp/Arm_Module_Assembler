#!/bin/bash
# =============================================================================
# build.sh — 워크스페이스 재빌드
#
# 사용법:
#   bash scripts/build.sh           # 증분 빌드
#   bash scripts/build.sh --clean   # build/install/log 삭제 후 전체 빌드
# =============================================================================
set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; NC='\033[0m'; BOLD='\033[1m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()  { echo -e "\n${BLUE}${BOLD}═══ $* ═══${NC}"; }
log_ok()    { echo -e "${GREEN}${BOLD}  ✔ $*${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

PACKAGES=(
    llm_agent_msgs
    ur3_mtc_pick_place
    ur3_moveit_module
    robot_vision
    llm_agent
)

echo -e "\n${BLUE}${BOLD}UR3 Pick & Place — 워크스페이스 빌드${NC}"
echo -e "워크스페이스: ${BOLD}$WS_DIR${NC}\n"

# ── --clean 옵션 ──────────────────────────────────────────────
if [[ "${1:-}" == "--clean" ]]; then
    log_warn "--clean 옵션: build/ install/ log/ 삭제 후 전체 빌드"
    read -rp "  계속하시겠습니까? [y/N] " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"
        log_info "이전 빌드 삭제 완료"
    else
        echo "취소됨"
        exit 0
    fi
fi

# ── ROS 소스 ──────────────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
    log_error "ROS 2 Humble 없음: /opt/ros/humble/setup.bash"
    exit 1
fi
source /opt/ros/humble/setup.bash

cd "$WS_DIR"

# ── 빌드 ──────────────────────────────────────────────────────
log_step "1/2 llm_agent_msgs 빌드 (의존 선행)"
colcon build \
    --packages-select llm_agent_msgs \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers console_cohesion+ || {
    log_error "llm_agent_msgs 빌드 실패"
    exit 1
}
log_ok "llm_agent_msgs 빌드 완료"

log_step "2/2 나머지 패키지 빌드"
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
    echo ""
    echo "빌드 로그: $WS_DIR/log/latest_build/"
    exit 1
}

log_ok "모든 패키지 빌드 완료"
echo ""
echo "적용: source $WS_DIR/install/setup.bash"
