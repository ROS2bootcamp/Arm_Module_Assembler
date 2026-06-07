#!/bin/bash
# =============================================================================
# run_stack.sh — 시뮬레이션 전체 스택 기동 (터미널 1)
#
# 포함 컴포넌트:
#   Gazebo (편의점 선반 씬) + UR3 ros2_control + MoveGroup +
#   Moveit_module 서비스 + YOLO 노드 + ros_gz_bridge + RViz2
#
# 사용법:
#   bash scripts/run_stack.sh [옵션]
#
# 옵션:
#   --no-rviz     RViz2 없이 실행 (headless 환경)
#   --mock        Moveit_module mock 모드 (MoveIt 없이 서비스 핸드셰이크만 확인)
# =============================================================================
set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# ── 인수 파싱 ─────────────────────────────────────────────────
NO_RVIZ=false
MOCK=false
for arg in "$@"; do
    case $arg in
        --no-rviz) NO_RVIZ=true ;;
        --mock)    MOCK=true ;;
        --help|-h)
            echo "사용법: $0 [--no-rviz] [--mock]"
            exit 0 ;;
        *)
            log_warn "알 수 없는 인수: $arg";;
    esac
done

# ── 환경 확인 ─────────────────────────────────────────────────
echo -e "\n${BLUE}${BOLD}UR3 Pick & Place — 시뮬레이션 스택 기동${NC}\n"

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

# ── IGN_GAZEBO_RESOURCE_PATH 설정 ─────────────────────────────
UR3_ENV="${UR3_CONVENIENCE_ENV_PATH:-$WS_DIR/UR3_CONVENIENCE_ENV}"
if [ ! -d "$UR3_ENV/models" ]; then
    log_error "UR3_CONVENIENCE_ENV 없음: $UR3_ENV"
    echo "  UR3_CONVENIENCE_ENV_PATH를 확인하거나 setup.sh를 다시 실행하세요."
    exit 1
fi
export UR3_CONVENIENCE_ENV_PATH="$UR3_ENV"
log_info "IGN_GAZEBO_RESOURCE_PATH: $UR3_ENV/models"

# ── launch 인수 구성 ───────────────────────────────────────────
LAUNCH_ARGS=()
if $MOCK; then
    LAUNCH_ARGS+=("mock:=true")
    log_warn "MOCK 모드 — MoveIt 실제 실행 안 함"
fi

# ── 기동 ──────────────────────────────────────────────────────
echo ""
echo -e "${CYAN}기동 컴포넌트:${NC}"
echo -e "  • Ignition Gazebo (ur3_pick_place.sdf)"
echo -e "  • UR3 + Robotiq 2F-85 (ros2_control)"
echo -e "  • MoveGroup + RViz2$(if $NO_RVIZ; then echo ' (RViz 제외)'; fi)"
echo -e "  • Moveit_module (/moveit/execute 서비스)$(if $MOCK; then echo ' [MOCK]'; fi)"
echo -e "  • YOLO 탐지 노드"
echo -e "  • ros_gz_bridge (camera/clock)"
echo ""
echo -e "${YELLOW}스택이 완전히 기동될 때까지 약 20~30초 기다린 후${NC}"
echo -e "${YELLOW}터미널 2에서 run_agent.sh를 실행하세요.${NC}"
echo ""
echo -e "종료: ${BOLD}Ctrl+C${NC}"
echo ""

ros2 launch ur3_mtc_pick_place ur3_integrated.launch.py "${LAUNCH_ARGS[@]}"
