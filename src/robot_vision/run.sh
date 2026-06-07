#!/bin/bash
set -e

# 스크립트가 위치한 폴더 경로 추출 (src/robot_vision)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$SCRIPT_DIR/../.."

# 설치 완료를 기록할 숨김 파일 이름
FLAG_FILE="$SCRIPT_DIR/.setup_completed"

# ==========================================
# 1단계: 초기 환경 셋업 (첫 실행 시에만 동작)
# ==========================================
if [ ! -f "$FLAG_FILE" ]; then
    echo "🌱 [1/4] 첫 실행을 감지했습니다. 의존성 패키지 설치를 시작합니다..."
    
    echo ">> ROS 2 필수 패키지 설치 (비밀번호 입력이 필요할 수 있습니다)"
    sudo apt-get update
    sudo apt-get install -y ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

    echo ">> 파이썬 AI 라이브러리 설치"
    pip install -r "$SCRIPT_DIR/requirements.txt"

    # 설치가 무사히 끝나면 플래그 파일 생성
    touch "$FLAG_FILE"
    echo "✅ 초기 환경 구축이 완료되었습니다!"
else
    echo "⚡ [1/4] 환경 구축이 이미 완료된 상태입니다. 설치를 건너뜁니다."
fi

# ==========================================
# 2단계: 빌드 및 실행 (항상 동작)
# ==========================================
cd "$WORKSPACE_DIR"

echo "🚀 [2/4] robot_vision 패키지 빌드를 시작합니다..."
colcon build --packages-select robot_vision

echo "⚙️ [3/4] ROS 2 환경 변수를 적용합니다..."
source install/setup.bash

echo "🤖 [4/4] 통합 Launch 파일을 실행합니다 (Gazebo + Bridge + YOLO)..."
ros2 launch robot_vision vision_bringup.launch.py
