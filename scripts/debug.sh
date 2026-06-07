#!/bin/bash
# =============================================================================
# debug.sh — 테스트 디버깅 세션 런처
#
# run_stack.sh 와 run_agent.sh 가 실행 중인 상태에서 별도 터미널에 실행합니다.
# tmux 가 있으면 4분할 레이아웃으로 자동 구성합니다.
#
# 사용법:
#   bash scripts/debug.sh              # 비전 + 에이전트 통합 세션 (tmux 4분할)
#   bash scripts/debug.sh --vision     # 비전 모니터만
#   bash scripts/debug.sh --agent      # 에이전트 모니터만
#   bash scripts/debug.sh --save-log   # 로그 파일 저장 (logs/ 디렉터리)
#   bash scripts/debug.sh --record     # ros2bag 녹화 추가
#
# tmux 레이아웃:
#   ┌──────────────────────┬──────────────────────┐
#   │  debug_vision.py     │  debug_agent.py      │
#   │  (탐지 결과·Hz)       │  (P1~P4·서비스 응답)  │
#   ├──────────────────────┼──────────────────────┤
#   │  raw topic echo      │  자유 쉘             │
#   └──────────────────────┴──────────────────────┘
# =============================================================================
set -uo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$WS_DIR/logs"

# ── 인수 파싱 ─────────────────────────────────────────────────
MODE="all"
SAVE_LOG=false
RECORD=false

for arg in "$@"; do
    case $arg in
        --vision)   MODE="vision"  ;;
        --agent)    MODE="agent"   ;;
        --save-log) SAVE_LOG=true  ;;
        --record)   RECORD=true    ;;
        --help|-h)
            echo "사용법: $0 [--vision|--agent] [--save-log] [--record]"
            exit 0 ;;
        *)
            log_warn "알 수 없는 인수: $arg" ;;
    esac
done

# ── 환경 소스 ─────────────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
    log_error "ROS 2 Humble 없음"
    exit 1
fi
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash" 2>/dev/null || {
    log_error "워크스페이스 빌드 필요: bash $SCRIPT_DIR/build.sh"
    exit 1
}

mkdir -p "$LOG_DIR"

VISION_PY="$SCRIPT_DIR/debug_vision.py"
AGENT_PY="$SCRIPT_DIR/debug_agent.py"

for f in "$VISION_PY" "$AGENT_PY"; do
    if [ ! -f "$f" ]; then
        log_error "스크립트 없음: $f"
        exit 1
    fi
done

LOG_OPTS=""
if $SAVE_LOG; then
    LOG_OPTS="--save-log --log-dir $LOG_DIR"
    log_info "로그 저장 디렉터리: $LOG_DIR"
fi

# 각 pane 에서 실행할 ROS 환경 활성화 명령
ACTIVATE="source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash"

# ── tmux 없는 경우: 수동 실행 안내 ────────────────────────────
if ! command -v tmux &>/dev/null; then
    log_warn "tmux 미설치 — 아래 명령을 각 터미널에서 수동 실행하세요."
    echo ""
    echo -e "${BOLD}터미널 A${NC} — 비전 파이프라인 모니터:"
    echo -e "  ${CYAN}python3 $VISION_PY $LOG_OPTS${NC}"
    echo ""
    echo -e "${BOLD}터미널 B${NC} — 에이전트 / MoveIt 상태 모니터:"
    echo -e "  ${CYAN}python3 $AGENT_PY $LOG_OPTS${NC}"
    echo ""
    echo -e "${BOLD}터미널 C (선택)${NC} — 탐지 결과 원시 출력:"
    echo -e "  ${CYAN}ros2 topic echo /vision/detection_results${NC}"
    echo ""
    if $RECORD; then
        TS=$(date +%Y%m%d_%H%M%S)
        BAG="$LOG_DIR/bag_$TS"
        echo -e "${BOLD}터미널 D${NC} — ros2bag 녹화:"
        echo -e "  ${CYAN}ros2 bag record -o $BAG \\"
        echo -e "    /camera/image /camera/depth_image /camera/image/camera_info \\"
        echo -e "    /vision/detection_results /llm_agent/log /moveit_module/log /moveit_status${NC}"
        echo ""
    fi
    echo -e "${BOLD}추가 유용한 명령:${NC}"
    echo -e "  ros2 topic hz /camera/image          # 카메라 프레임 레이트"
    echo -e "  ros2 topic hz /vision/detection_results"
    echo -e "  ros2 run tf2_ros tf2_echo base_link camera_link  # TF 실시간"
    echo -e "  ros2 node info /yolo_detector_node   # 노드 정보"
    echo -e "  ros2 service call /moveit/execute ... # 서비스 직접 호출 테스트"
    exit 0
fi

# ── tmux 세션 생성 ─────────────────────────────────────────────
SESSION="ur3_debug"
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo ""
echo -e "${BLUE}${BOLD}UR3 Debug Session — tmux '$SESSION'${NC}"

case $MODE in
  vision)
    tmux new-session -d -s "$SESSION" -x 200 -y 50
    tmux send-keys -t "$SESSION" \
      "$ACTIVATE && python3 $VISION_PY $LOG_OPTS" Enter
    ;;

  agent)
    tmux new-session -d -s "$SESSION" -x 200 -y 50
    tmux send-keys -t "$SESSION" \
      "$ACTIVATE && python3 $AGENT_PY $LOG_OPTS" Enter
    ;;

  all)
    # 창 생성
    tmux new-session -d -s "$SESSION" -x 220 -y 60

    # 좌상 (pane 0): 비전 모니터
    tmux send-keys -t "${SESSION}:0.0" \
      "$ACTIVATE && python3 $VISION_PY $LOG_OPTS" Enter

    # 우상 (pane 1): 에이전트 모니터
    tmux split-window -h -t "${SESSION}:0"
    tmux send-keys -t "${SESSION}:0.1" \
      "$ACTIVATE && python3 $AGENT_PY $LOG_OPTS" Enter

    # 좌하 (pane 2): detection 원시 echo
    tmux select-pane -t "${SESSION}:0.0"
    tmux split-window -v -t "${SESSION}:0.0"
    tmux send-keys -t "${SESSION}:0.2" \
      "$ACTIVATE && ros2 topic echo /vision/detection_results" Enter

    # 우하 (pane 3): 자유 쉘
    tmux select-pane -t "${SESSION}:0.1"
    tmux split-window -v -t "${SESSION}:0.1"
    tmux send-keys -t "${SESSION}:0.3" \
      "$ACTIVATE && echo '── 자유 쉘 ──  ros2 명령 입력 가능'" Enter

    # ros2bag 녹화 (--record 플래그)
    if $RECORD; then
        TS=$(date +%Y%m%d_%H%M%S)
        BAG="$LOG_DIR/bag_$TS"
        # 우하 pane 에서 bag 시작 (sleep 3 으로 다른 노드 뜬 후 시작)
        tmux send-keys -t "${SESSION}:0.3" \
          "sleep 3 && ros2 bag record -o $BAG \
/camera/image /camera/depth_image /camera/image/camera_info \
/vision/detection_results /llm_agent/log /moveit_module/log /moveit_status" Enter
        log_info "bag 저장 예정: $BAG"
    fi

    # 포커스: 비전 모니터 (좌상)
    tmux select-pane -t "${SESSION}:0.0"
    ;;
esac

echo ""
log_info "tmux 세션 '$SESSION' 생성 완료"
echo ""
echo -e "  접속: ${CYAN}tmux attach -t $SESSION${NC}"
echo -e "  종료: ${CYAN}tmux kill-session -t $SESSION${NC}  (또는 세션 내 Ctrl+B, &)"
echo -e "  pane 전환: ${CYAN}Ctrl+B, 방향키${NC}"
echo ""

tmux attach -t "$SESSION"
