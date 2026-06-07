#!/bin/bash
# =============================================================================
# verify.sh — 통합 스택 동작 검증
#
# run_stack.sh 기동 후 별도 터미널에서 실행합니다.
# 각 컴포넌트의 토픽·서비스·TF가 정상인지 확인합니다.
#
# 사용법:
#   bash scripts/verify.sh
# =============================================================================
set -uo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'; BOLD='\033[1m'

PASS=0; FAIL=0

check_pass() { echo -e "${GREEN}  ✔${NC} $*"; ((PASS++)); }
check_fail() { echo -e "${RED}  ✘${NC} $*"; ((FAIL++)); }
check_warn() { echo -e "${YELLOW}  ⚠${NC} $*"; }
log_section() { echo -e "\n${BLUE}${BOLD}── $* ──${NC}"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash" 2>/dev/null || {
    echo -e "${RED}워크스페이스 빌드 필요: bash $SCRIPT_DIR/build.sh${NC}"
    exit 1
}

echo -e "\n${BLUE}${BOLD}UR3 Pick & Place — 통합 검증${NC}"
echo -e "(run_stack.sh가 실행 중이어야 합니다)\n"

# ── 1. 노드 확인 ──────────────────────────────────────────────
log_section "ROS 2 노드"
NODES=$(ros2 node list 2>/dev/null || echo "")

if echo "$NODES" | grep -q "move_group";      then check_pass "MoveGroup 노드"; else check_fail "MoveGroup 노드 없음"; fi
if echo "$NODES" | grep -q "yolo_detector";   then check_pass "YOLO 탐지 노드"; else check_fail "YOLO 탐지 노드 없음"; fi
if echo "$NODES" | grep -q "moveit_module";   then check_pass "Moveit_module 노드"; else check_fail "Moveit_module 노드 없음"; fi

# ── 2. 서비스 확인 ────────────────────────────────────────────
log_section "ROS 2 서비스"
SERVICES=$(ros2 service list 2>/dev/null || echo "")

if echo "$SERVICES" | grep -q "/moveit/execute"; then
    check_pass "/moveit/execute 서비스"
else
    check_fail "/moveit/execute 서비스 없음 — Moveit_module 확인 필요"
fi

# ── 3. 토픽 확인 ──────────────────────────────────────────────
log_section "카메라 토픽 (ros_gz_bridge)"
TOPICS=$(ros2 topic list 2>/dev/null || echo "")

for topic in "/camera/image" "/camera/depth_image" "/camera/image/camera_info"; do
    if echo "$TOPICS" | grep -q "$topic"; then
        hz=$(ros2 topic hz "$topic" --window 5 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "?")
        check_pass "$topic  (${hz} Hz)"
    else
        check_fail "$topic 없음"
    fi
done

log_section "YOLO 탐지 결과"
if echo "$TOPICS" | grep -q "/vision/detection_results"; then
    check_pass "/vision/detection_results 발행 중"
    echo ""
    echo -e "${CYAN}  최근 탐지 결과 (5초):${NC}"
    timeout 5 ros2 topic echo /vision/detection_results --once 2>/dev/null \
        | python3 -c "
import sys, json
for line in sys.stdin:
    line = line.strip()
    if line.startswith('data:'):
        raw = line[5:].strip().strip(\"'\")
        try:
            d = json.loads(raw)
            print(f\"    탐지 수: {d.get('num_detections',0)}\")
            for obj in d.get('objects',[]):
                pos = obj.get('position_3d_base_frame',{})
                print(f\"    class={obj['class_name']}  conf={obj['confidence']:.2f}  \"\
                      f\"base=[{pos.get('X','?'):.3f}, {pos.get('Y','?'):.3f}, {pos.get('Z','?'):.3f}]\")
        except:
            print('    (JSON 파싱 실패)')
" 2>/dev/null || echo -e "    ${YELLOW}(탐지 결과 없음 — 카메라 시야 확인 필요)${NC}"
else
    check_fail "/vision/detection_results 없음"
fi

# ── 4. TF 확인 ────────────────────────────────────────────────
log_section "TF 트리"
if ros2 run tf2_ros tf2_echo world base_link 2>/dev/null | grep -q "Translation"; then
    check_pass "world → base_link TF"
else
    check_warn "world → base_link TF 확인 불가 (정상일 수 있음)"
fi

TF_OUT=$(ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null | head -5 || echo "")
if echo "$TF_OUT" | grep -q "Translation"; then
    tx=$(echo "$TF_OUT" | grep "x:" | head -1 | awk '{print $2}')
    tz=$(echo "$TF_OUT" | grep "z:" | head -1 | awk '{print $2}')
    check_pass "base_link → camera_link TF  (x=$tx z=$tz)"
    if (( $(echo "$tz < 0.4" | bc -l 2>/dev/null || echo 0) )); then
        check_warn "  camera_link z가 예상보다 낮음 (기대: z≈0.5)"
    fi
else
    check_fail "base_link → camera_link TF 없음 — camera_link static TF 확인 필요"
fi

# ── 5. /clock (sim_time) 확인 ──────────────────────────────────
log_section "시뮬레이션 클록"
if echo "$TOPICS" | grep -q "/clock"; then
    check_pass "/clock 발행 중 (sim_time 동기화 중)"
else
    check_fail "/clock 없음 — ros_gz_bridge 확인 필요"
fi

# ── 결과 요약 ─────────────────────────────────────────────────
echo ""
echo -e "${BOLD}════════════════════════════════${NC}"
echo -e "  통과: ${GREEN}${BOLD}$PASS${NC}  실패: ${RED}${BOLD}$FAIL${NC}"
echo -e "${BOLD}════════════════════════════════${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}${BOLD}모든 체크 통과 — 에이전트 실행 준비 완료${NC}"
    echo -e "  bash $SCRIPT_DIR/run_agent.sh"
else
    echo -e "${YELLOW}일부 체크 실패 — 위 항목을 확인하세요.${NC}"
    echo -e "  트러블슈팅: README.md 9번 섹션 참조"
fi
echo ""
