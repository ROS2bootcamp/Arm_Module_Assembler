# Arm_Module_Assembler

> **UR3 robot arm × LLM** 제어 프로젝트의 **통합(Integration) 메인 레포**.  
> 각 팀이 개별 개발한 모듈(LLM Agent / YOLO Vision / MoveIt / 로봇모델·시뮬)을
> 하나의 Gazebo 편의점 선반 시나리오로 통합한 기준 레포입니다.

## 시스템 개요

CLI 자연어 명령 → LLM이 의도 파싱 → YOLO가 선반 위 콜라캔 탐지·3D좌표 변환 → MoveIt이 Pick & Place 계획·실행

```
[사용자 CLI] ──자연어──▶ llm_agent (LLM_Agent)
                             │ /moveit/execute (ROS2 srv)
                             ▼
                      ur3_moveit_module (Moveit_module)
                             │ MoveIt2 + MTC
                             ▼
                    Gazebo UR3 + Robotiq 2F-85
                             ▲
              /vision/detection_results (JSON String)
                             │
                    yolo_detector (ROBOT_VISION)
                             ▲
              /camera/image + /camera/depth_image
                             │
                  fixed_rgbd_camera (UR3_CONVENIENCE_ENV)
```

## 모듈 현황

| 모듈 | 레포 | 통합 브랜치 | 상태 |
|------|------|------------|------|
| LLM Agent | `ROS2bootcamp/LLM_Agent` | `fix/agent-frame-params` | ✅ 통합 완료 |
| YOLO Vision | `ROS2bootcamp/ROBOT_VISION` | `feat/integration` | ✅ 통합 완료 |
| MoveIt 실행 | `ROS2bootcamp/Moveit_module` | `feat/service-interface` | ✅ 통합 완료 |
| MoveIt 환경 | `ROS2bootcamp/PANDA_ENV` | `fix/srdf-group-name` | ✅ 통합 완료 |
| 테스트 환경 | `ROS2bootcamp/UR3_CONVENIENCE_ENV` | `feat/model-dirs-and-integrated-world` | ✅ 통합 완료 |

---

## 즉시 테스트 가이드

> **이 레포 하나로 모든 것이 포함됩니다.** `src/` 에 전체 ROS 2 패키지, `src/ur3_mtc_pick_place/models/` 에 Gazebo 모델이 내장되어 있습니다. 별도 레포 클론 불필요.

```bash
# 1. 클론
git clone https://github.com/ROS2bootcamp/Arm_Module_Assembler.git
cd Arm_Module_Assembler

# 2. 전체 환경 설정 (1회 — apt/pip 설치 + 빌드 + API 키)
bash scripts/setup.sh

# 3-A. 시뮬레이션 스택 기동 (터미널 1)
bash scripts/run_stack.sh

# 3-B. LLM Agent 실행 (터미널 2, 스택 기동 후)
bash scripts/run_agent.sh
```

---

### 1. 사전 요구사항

**OS / ROS 2 스택**

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt update
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-task-constructor-core \
  ros-humble-moveit-task-constructor-demo \
  ros-humble-moveit-task-constructor-capabilities \
  ros-humble-ur-simulation-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  ros-humble-robotiq-description \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher
```

**Python 패키지**

```bash
pip3 install google-genai python-dotenv ultralytics
```

**Gemini API 키** (https://aistudio.google.com 에서 발급)

---

### 2. 레포 구조

이 레포는 **모노레포 ROS 2 워크스페이스**입니다. 별도 레포 클론 없이 이 레포 하나로 완전한 시스템을 구성합니다.

```
Arm_Module_Assembler/         ← colcon 워크스페이스 루트
├── src/
│   ├── llm_agent_msgs/       LLM ↔ MoveIt 서비스 인터페이스
│   ├── llm_agent/            LLM 에이전트 (Gemini + P1~P4 Pick & Place)
│   ├── ur3_mtc_pick_place/   UR3 환경 패키지
│   │   ├── models/           Gazebo 모델 (선반·콜라캔·디스트랙터·카메라)
│   │   └── worlds/           통합 씬 SDF (ur3_pick_place.sdf)
│   ├── ur3_moveit_module/    MoveIt2 + MTC 실행 서비스
│   └── robot_vision/         YOLO v8 탐지 노드
├── scripts/                  자동화 스크립트
├── docs/                     설계 결정 문서
└── .env                      API 키 (gitignore, setup.sh 가 생성)
```

**Gazebo 씬 구성:**

| 오브젝트 | YOLO 클래스 | 역할 |
|---------|------------|------|
| Coke Can | `bottle` | **타겟** — 에이전트가 집는 대상 |
| Soccer Ball | `sports ball` | 디스트랙터 |
| Toaster | `toaster` | 디스트랙터 |

최종 디렉터리 구조:

```
$WS/
├── PANDA_ENV/          ← ur3_mtc_pick_place 패키지 (URDF·SRDF·launch)
├── Moveit_module/      ← ur3_moveit_module + llm_agent_msgs 패키지
├── LLM_Agent/          ← llm_agent 패키지
├── ROBOT_VISION/       ← robot_vision 패키지
└── UR3_CONVENIENCE_ENV/← Gazebo 모델·월드 (ROS 패키지 아님)
```

---

### 3. 빌드

```bash
cd $WS
source /opt/ros/humble/setup.bash

# llm_agent_msgs 먼저 빌드 (다른 패키지가 의존)
colcon build --packages-select llm_agent_msgs --symlink-install

# 나머지 패키지 빌드
colcon build --packages-select \
  ur3_mtc_pick_place \
  ur3_moveit_module \
  robot_vision \
  llm_agent \
  --symlink-install

source $WS/install/setup.bash
```

> **빌드 에러 발생 시 체크리스트**
> - `robotiq_description` 미설치: `sudo apt install ros-humble-robotiq-description`
> - 빈 폴더로 인한 CMake 오류: `mkdir -p` 로 누락 폴더 생성 후 재빌드
> - 캐시 문제: `rm -rf build/ install/ log/` 후 재빌드

---

### 4. 환경 설정

**4-1. Gemini API 키 설정**

```bash
# $WS 또는 agent_node 실행 디렉터리에 .env 파일 생성
cat > $WS/.env << 'EOF'
GEMINI_API_KEY=여기에_발급받은_API_키_입력
EOF
```

> `load_dotenv()`는 CWD 및 상위 디렉터리를 자동 탐색합니다.  
> 또는 환경 변수로 직접 설정: `export GEMINI_API_KEY=<키>`

**4-2. UR3_CONVENIENCE_ENV 경로 설정**

```bash
# 기본값: ~/WorkspaceMain/UR3_CONVENIENCE_ENV
# 워크스페이스 경로가 다를 경우 아래 줄을 ~/.bashrc 에 추가
export UR3_CONVENIENCE_ENV_PATH=$WS/UR3_CONVENIENCE_ENV
```

> launch 파일이 `IGN_GAZEBO_RESOURCE_PATH`를 자동으로 설정하므로  
> **`UR3_CONVENIENCE_ENV_PATH`만 지정하면** 나머지는 자동 처리됩니다.

---

### 5. 실행

> 각 터미널에서 반드시 `source $WS/install/setup.bash` 를 먼저 실행하세요.

**터미널 1 — 전체 시뮬레이션 스택 기동**

```bash
source /opt/ros/humble/setup.bash && source $WS/install/setup.bash
export UR3_CONVENIENCE_ENV_PATH=$WS/UR3_CONVENIENCE_ENV

ros2 launch ur3_mtc_pick_place ur3_integrated.launch.py
```

기동되는 컴포넌트:
- Ignition Gazebo (편의점 선반 + 콜라캔 + 고정 RGB-D 카메라)
- UR3 + Robotiq 2F-85 ros2_control
- MoveGroup (MoveIt2 / MTC)
- RViz2
- Moveit_module 서비스 서버 (`/moveit/execute`)
- YOLO 탐지 노드 (`/vision/detection_results`)
- ros_gz_bridge (카메라 이미지·depth·camera_info·clock)
- Static TF (`world→base_link`, `base_link→camera_link`)

**Gazebo가 완전히 기동된 후** 터미널 2를 실행합니다 (약 20~30초 대기).

**터미널 2 — LLM Agent 실행**

```bash
source /opt/ros/humble/setup.bash && source $WS/install/setup.bash
cd $WS   # .env 파일 위치
ros2 run llm_agent agent_node
```

**터미널 2 — 대화 예시**

```
> 콜라캔 집어줘
> pick up the coke can
> 선반에 있는 병 집어서 옮겨줘
```

---

### 6. 동작 확인

**YOLO 탐지 확인** (새 터미널)

```bash
source $WS/install/setup.bash
ros2 topic echo /vision/detection_results
```

정상 출력 예시:
```json
{
  "timestamp_ns": 1234567890,
  "num_detections": 1,
  "objects": [{
    "class_name": "bottle",
    "confidence": 0.87,
    "position_3d_base_frame": {"X": 0.55, "Y": 0.0, "Z": 0.42}
  }]
}
```

> `class_name: "bottle"` = 콜라캔 (YOLOv8n COCO 클래스 매핑)  
> `Z ≈ 0.42` = 선반(z=0.34m) 위 캔 중심 높이 (예상값)

**카메라 토픽 확인**

```bash
ros2 topic list | grep camera
# 기대 출력:
# /camera/image
# /camera/depth_image
# /camera/image/camera_info
```

**TF 트리 확인**

```bash
ros2 run tf2_tools view_frames
# 또는
ros2 run tf2_ros tf2_echo base_link camera_link
```

**MoveIt 서비스 확인**

```bash
ros2 service list | grep moveit
# /moveit/execute 가 있어야 함

# 수동 테스트 (scan 명령)
ros2 service call /moveit/execute llm_agent_msgs/srv/MoveItExecute \
  '{cmd: "scan", params_json: "{}"}'
```

---

### 7. 시나리오 흐름 (P1 ~ P4)

| 단계 | 에이전트 동작 | 관찰 포인트 |
|------|-------------|------------|
| **P1 Scan** | 로봇 팔이 scan waypoint 이동 | Gazebo에서 팔 움직임, YOLO가 `bottle` 탐지 |
| **P2 Detect** | YOLO 결과에서 콜라캔 좌표 추출 | `/vision/detection_results` 로그 확인 |
| **P3 Pick** | MoveIt이 선반 위 캔으로 접근·파지 | Gazebo에서 그리퍼 닫힘 |
| **P4 Place** | 지정 위치(x=0.4, y=-0.2)에 내려놓음 | Gazebo에서 그리퍼 열림·캔 배치 |

---

### 8. 시뮬레이션 환경 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| 선반 위치 | x=0.55, y=0.0, z=-0.05, yaw=90° | `convenience_shelf` 스폰 좌표 (README 기준) |
| 선반 표면 높이 | z = 0.34 m | 월드 좌표계 기준 |
| 콜라캔 link_z | 0.57 m | 캔 바닥이 선반 표면(z=0.34)에 닿는 위치 |
| 콜라캔 중심 높이 | z ≈ 0.42 m | YOLO 탐지 기대값 |
| 카메라 위치 | x=1.2, y=0.0, z=0.5 | 고정 RGB-D, -X 방향 향함 |
| 카메라 TF | roll=-π/2, pitch=0, yaw=+π/2 | `base_link → camera_link` |
| 배치 목표 | x=0.4, y=-0.2, z=0.08 | `targets.yaml` 기준 |

**카메라 TF 튜닝이 필요한 경우:**

```bash
ros2 launch ur3_mtc_pick_place ur3_integrated.launch.py \
  camera_x:=1.2 camera_z:=0.5 \
  camera_roll:=-1.5708 camera_pitch:=0.0 camera_yaw:=1.5708
```

---

### 9. 스크립트 목록

| 스크립트 | 용도 | 실행 횟수 |
|----------|------|----------|
| `scripts/setup.sh` | 전체 환경 설정 (apt/pip 설치 + 클론 + 빌드 + `.env` 생성) | 1회 |
| `scripts/build.sh` | 워크스페이스 재빌드 (`--clean` 옵션으로 전체 재빌드) | 코드 변경 시 |
| `scripts/run_stack.sh` | 시뮬레이션 전체 스택 기동 (터미널 1) | 매 테스트 |
| `scripts/run_agent.sh` | LLM Agent 실행 (터미널 2) | 매 테스트 |
| `scripts/verify.sh` | 노드·토픽·TF·서비스 동작 검증 | 스택 기동 후 확인 |
| `scripts/debug.sh` | 비전·에이전트 통합 디버그 세션 (tmux 4분할) | 문제 발생 시 |
| `scripts/debug_vision.py` | 탐지 결과·카메라 Hz 실시간 모니터 (단독 실행 가능) | 비전 디버깅 |
| `scripts/debug_agent.py` | P1~P4 단계·MoveIt 응답 실시간 모니터 (단독 실행 가능) | 에이전트 디버깅 |

#### 디버그 세션 실행 예시

```bash
# tmux 4분할 통합 세션 (비전 + 에이전트 + raw echo + 자유 쉘)
bash scripts/debug.sh

# 로그 파일 저장 + ros2bag 녹화
bash scripts/debug.sh --save-log --record

# 비전만 단독 모니터
python3 scripts/debug_vision.py --save-log

# 에이전트만 단독 모니터
python3 scripts/debug_agent.py --save-log
```

> 로그 파일은 `Arm_Module_Assembler/logs/` 에 `vision_YYYYMMDD_HHMMSS.log` /
> `agent_YYYYMMDD_HHMMSS.log` 형식으로 저장됩니다.

---

### 10. 트러블슈팅

**Gazebo 모델 로드 실패** (`model://convenience_shelf not found`)

```bash
# UR3_CONVENIENCE_ENV_PATH가 올바르게 설정되었는지 확인
echo $UR3_CONVENIENCE_ENV_PATH
ls $UR3_CONVENIENCE_ENV_PATH/models/
# convenience_shelf/, coke_can/, fixed_rgbd_camera/ 가 있어야 함
```

**YOLO 탐지 없음** (`num_detections: 0`)

1. 카메라 이미지 확인: `ros2 topic hz /camera/image` (30 Hz 기대)
2. DISPLAY가 없는 경우 headless 모드 확인: `export DISPLAY` 미설정 시 자동 headless
3. camera_info 수신 확인: `ros2 topic hz /camera/image/camera_info`
4. TF 확인: `ros2 run tf2_ros tf2_echo base_link camera_link`

**`/moveit/execute` 서비스 없음**

```bash
# Moveit_module이 제대로 기동되었는지 확인
ros2 node list | grep moveit_module
```

**MoveIt IK 실패** (`ur3_manipulator` 그룹 없음 오류)

SRDF가 `name:=ur` (not `ur3`) 으로 생성되어야 함:
```bash
# 현재 그룹명 확인
ros2 param get /move_group robot_description_semantic | grep "group name"
# <group name="ur_manipulator"> 가 있어야 함
```

**API 키 오류** (`GEMINI_API_KEY not set`)

```bash
ls -la $WS/.env        # .env 파일 존재 확인
cat $WS/.env           # GEMINI_API_KEY=... 내용 확인
# 또는 환경 변수 직접 설정:
export GEMINI_API_KEY=<발급한_키>
```

**빌드 에러 — `robotiq_description` 미설치**

```bash
sudo apt install ros-humble-robotiq-description
```

---

## 문서

| 문서 | 내용 |
|------|------|
| [docs/DECISIONS.md](docs/DECISIONS.md) | 모든 통합 결정사항(D1~D18), 레포별 수정 액션, 구현 상태 |
| [docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md) | 토픽/서비스/프레임/좌표 규약 단일 진실원천(SSOT) |
| [docs/MOVEIT_MODULE_INTEGRATION.md](docs/MOVEIT_MODULE_INTEGRATION.md) | Moveit_module 전송 서비스화 구조 정립 |
| [docs/INTEGRATION_ANALYSIS.md](docs/INTEGRATION_ANALYSIS.md) | 모듈별 상세 분석, 인터페이스, 통합 이슈 전수 |
| [docs/INTEGRATION_STRATEGY.md](docs/INTEGRATION_STRATEGY.md) | 단계별 통합 로드맵 + 목표 워크스페이스 구조 |

---

## 공통 스택

| 항목 | 버전 / 내용 |
|------|------------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Gazebo | Ignition Fortress |
| 로봇 | UR3 + Robotiq 2F-85 |
| 모션 계획 | MoveIt2 + MoveIt Task Constructor |
| 비전 | YOLOv8n (COCO) |
| LLM | Google Gemini `gemini-2.5-flash` |
| 물체 인식 클래스 | `bottle` = 콜라캔 |
