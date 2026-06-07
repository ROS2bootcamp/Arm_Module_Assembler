# 통합 분석 (Integration Analysis)

> 작성일 2026-06-07 · 기준 커밋: 각 서브 레포 최신(2026-06-06~07)
> 목적: 8개 레포 전수 분석 → 모듈별 인터페이스 정리 → 통합 지점별 이슈 도출.
> 확정된 결정과 규약은 [DECISIONS.md](DECISIONS.md), [INTERFACE_CONTRACT.md](INTERFACE_CONTRACT.md) 참조.

---

## 0. 레포 전수 현황

지정 5개 외에, 통합에 직결되는 3개(PANDA_ENV / GAZEBO_ENV / UR3_CONVENIENCE_ENV)를 추가 식별.

| 레포 | 역할 | 상태 | 비고 |
|------|------|------|------|
| **Arm_Module_Assembler** | 통합 메인 | 🔴 빈 레포(이 커밋이 최초) | 통합 산출물 집결지 |
| **LLM_Agent** | 시나리오 두뇌 | 🟢 구현·문서·mock 충실 | `main`+`design/resolve-open-issues` |
| **ROBOT_VISION** | YOLO 비전 | 🟡 노드 동작 / 로봇없는 월드 | Ignition + ros_gz_bridge |
| **Moveit_module** | (지정) MoveIt | 🔴 빈 레포 | 실구현은 PANDA_ENV |
| **PANDA_ENV** | MoveIt MTC 실구현 | 🟡 one-shot 스크립트 | `ur3_mtc_pick_place` + 통합 launch |
| **ur3-glapper** | UR3+그리퍼 URDF | 🟡 RViz 확인 단계 | Gazebo/control/MoveIt ⏳ |
| **GAZEBO_ENV** | UR3 Gazebo 환경 | 🔴 README 가이드만 | 공식 `ur-simulation-gz` |
| **UR3_CONVENIENCE_ENV** | 선반 picking 환경 | 🔴 README 1줄 | 미착수 |

> **핵심**: 통합 "본체"여야 할 Moveit_module·메인 레포가 비어 있고, 실제 MoveIt 로직은
> PANDA_ENV, 로봇 모델은 ur3-glapper/PANDA/공식패키지 3곳에 분산.
> 각 모듈은 개별 동작하나 **아직 한 번도 같은 Gazebo 세계에서 만난 적이 없음.**

---

## 1. LLM_Agent — 통합의 두뇌 (가장 성숙, 계약 정의자)

### 구조
- `AgentNode`(`agent_node.py`): `MultiThreadedExecutor` + `ThreadPoolExecutor(max_workers=1)`.
  Phase 루프를 워커 스레드에서 돌려 ROS2 spin 스레드를 막지 않음.
- 컴포넌트: `CLIReader`(stdin) · `YoloSubscriber`(deque 버퍼) · `TFTransformer`(base→world) ·
  `MoveItClient`(서비스 client) · `LLMClient`(Gemini) · `PhaseManager`(상태머신·retry).

### 4-Phase 시나리오
```
parse_command(LLM) → target_class_name
P1 scan 서비스 + YOLO 탐지(conf·distance, 30s×최대3회)
P2 최신프레임 base좌표 → base→world TF → config Object Spec 조립 → pick 서비스 → grip검증(YOLO ≤0.15m, 3회)
P3 lift 10cm 서비스 → YOLO 수집 → verify_pickup(LLM) → 실패시 release+home+P1재시작
P4 place → release → home → 세션 완료
```
- 공유 `session_retry_count`(기본 3) 초과 시 세션 종료(P1 타임아웃·P2 grip실패·P3 검증실패 공통).
- LLM 호출 **2곳만**: `parse_command`(진입), `verify_pickup`(P3). P2 파라미터는 config 결정론 조립.

### 인터페이스 (이 모듈이 계약을 선언함)
| 채널 | 방향 | 타입 |
|------|------|------|
| stdin | CLI→Agent | text |
| `/vision/detection_results` | YOLO→Agent | `std_msgs/String`(JSON) |
| `/tf`,`/tf_static` | TF→Agent | `tf2_msgs/TFMessage` (base_link→world) |
| `/moveit/execute` | Agent→MoveIt | `llm_agent_msgs/srv/MoveItExecute` (agent=client) |
| Gemini API | Agent→Google | HTTPS, `.env` `GEMINI_API_KEY` |
| `/llm_agent/phase`,`/llm_agent/log` | Agent→모니터 | `std_msgs/String` |

### 설정 요약 (`config/`)
- `agent.yaml`: 임계값·Gemini·moveit/yolo/tf 블록
- `objects.yaml`: `cup{cyl,[0.12,0.025]}`, `bottle{cyl,[0.20,0.033]}`, `default_spec{cyl,[0.10,0.03]}`
- `scan_waypoints.yaml`: P1 4개 waypoint(world frame), `targets.yaml`: place [0.4,-0.2,0.06]

> **주의**: `agent.yaml`의 `hand_frame`/`grasp_frame_transform`이 실구현과 불일치 → [DECISIONS D1·D2](DECISIONS.md).

---

## 2. ROBOT_VISION — YOLO 비전

### 동작 (`robot_vision/yolo_detector.py`)
`/camera/image` + `/camera/depth_image` 구독 → YOLOv8n(COCO) 추론 →
핀홀 역투영(`fx=fy=554.25, cx=320, cy=240`) → `camera_link→base_link` TF 변환 →
`/vision/detection_results`(`std_msgs/String` JSON) 발행.

### 출력 JSON (LLM_Agent 기대치와 **정확히 일치** ✅)
```json
{"timestamp_ns":..., "num_detections":1, "objects":[
  {"class_name":"cup","confidence":0.892,"center_2d":{"u":320,"v":240},
   "distance_m":0.452,
   "position_3d_camera_frame":{"X":..,"Y":..,"Z":..},
   "position_3d_base_frame":{"X":..,"Y":..,"Z":..}}]}
```
좌표키 대문자 X/Y/Z, 에이전트는 `position_3d_base_frame` 사용.

### 환경 (`worlds/camera_test.sdf`, `launch/vision_bringup.launch.py`)
- **로봇 없는 독립 월드**: 고정 카메라(`0 0 0.5`) + Stop Sign(3m) + Coke Can(1.5m).
- 카메라 FOV 1.047/640 → 내부파라미터 554.25와 정합(이 월드 한정).
- 브릿지는 image 2개만(`ros_gz_bridge`). TF·clock 브릿지 없음.

### 통합 관점 문제
- 이 월드엔 `base_link`가 없어 `camera_link→base_link` TF가 **항상 실패** →
  `position_3d_base_frame = {null,null,null}`. (통합 월드에서 해소 필요)
- 카메라가 로봇과 무관하게 떠 있음 → **통합 시 로봇/씬에 카메라 장착** 필요.

---

## 3. MoveIt 구현 — PANDA_ENV `ur3_mtc_pick_place`

> 이름은 "PANDA"지만 실제 패키지는 **UR3 MTC pick&place**. MOVEIT_INTERFACE.md가
> "서비스 서버로 개조하라"고 지목한 원본.

### `scripts/ur3_pick_place.py`
- MTC Task **하나**로 전체를 구성·실행하는 **one-shot 노드**:
  CurrentState→open→Connect→**[pick: approach·GenerateGraspPose·IK·allow-collision·close·attach·lift]**
  →Connect→**[place: lower·GeneratePlacePose·IK·open·detach·retreat]**→home.
- `task.plan()` 후 `task.execute(solutions[0])` 1회 실행. 서비스/단계분리/상태유지 없음.
- 충돌물체 spawn: `make_cylinder`가 `z + height/2` 보정 추가(에이전트도 보정 → **이중보정 위험**).

### `config/ur3_mtc_config.yaml`
- `arm_group_name: ur_manipulator`, `eef_name: robotiq_2f_85`, `hand_group_name: gripper`
- `hand_frame: robotiq_85_tcp`, `grasp_frame_transform: [0,0,0.0,π,0,0]`
- 명령형 6단계 없음(전 과정 일괄).

### `launch/ur3_mtc_demo.launch.py` — **사실상 통합 bringup 원형** ⭐
이미 다음을 한 번에 구성:
- URDF `ur3_with_gripper.urdf.xacro`(공식 `robotiq_description` + custom `robotiq_85_tcp` @base+0.13m)
- SRDF `ur3_with_gripper.srdf.xacro`(`ur_moveit_config` UR SRDF + gripper 그룹 추가)
- kinematics/OMPL ← `ur_moveit_config`, controllers ← `ur_simulation_gz`
- **Gazebo** ← `ur_simulation_gz/launch/ur_sim_control.launch.py`
- **`world→base_link` identity static TF** 명시 발행
- `move_group` + MTC `ExecuteTaskSolutionCapability`

→ 통합 로봇/시뮬/MoveIt 스택의 **정규 경로**로 채택(공식 `ur_simulation_gz` 기반). [DECISIONS D5].

### dead 파일
- `urdf/robotiq_2f_85.xacro`(TCP `robotiq_2f_85_tcp` 정의)는 **어디서도 include 안 됨** → 혼동 유발, 제거 권장. [DECISIONS D6]

---

## 4. 로봇 모델 / 시뮬 (ur3-glapper / GAZEBO_ENV / UR3_CONVENIENCE_ENV)

### ur3-glapper
- `world→(ur_robot, identity)`, 그리퍼 `robotiq_2f_85`@`tool0`. 공식 `ur_description`+`robotiq_description`, `sim_ignition=true`.
- **RViz 모델 확인까지만** — Gazebo spawn·ros2_control·MoveIt config·Pick&Place 모두 ⏳.
- CMakeLists가 미존재 디렉토리(`launch config worlds`)를 `install(DIRECTORY)` → **colcon 빌드 실패 소지**.
- PANDA의 `ur3_with_gripper.urdf.xacro`가 동일 그리퍼 + TCP + SRDF + sim까지 더 완성형 → **PANDA 경로로 흡수**.

### GAZEBO_ENV
- apt `ros-humble-ur-simulation-gz`로 `ur_sim_control` / `ur_sim_moveit` 실행 가이드.
- PANDA launch가 참조하는 `ur_simulation_gz`와 동일 → **이 경로가 채택된 Gazebo 소스.**

### UR3_CONVENIENCE_ENV
- README 1줄(편의점 선반 picking). 사실상 미착수 → 후속 단계의 씬(scene) 소스 후보.

---

## 5. 공통 스택 정합성

| 항목 | 값 | 정합 |
|------|-----|------|
| OS/ROS | Ubuntu 22.04 / Humble | ✅ 전 모듈 |
| 시뮬 | Ignition Gazebo Fortress + `ros_gz` | ✅ |
| 빌드 | ament_python(agent/vision) / ament_cmake(msgs/모델) | ✅ |
| 로봇 | UR3 + Robotiq 2F-85 | ✅ |
| world→base_link | identity (PANDA static TF + URDF로 확정) | ✅ |

---

## 6. 통합 지점별 이슈 (요약 — 상세·결정은 DECISIONS.md)

### 🔴 Blocker
- **B1 단일 Gazebo 월드 부재** — UR3+카메라+테이블+물체가 한 월드에 없음 → E2E 불가.
- **B2 MoveIt 서비스 서버 미구현** — 에이전트의 `/moveit/execute`(6 cmd) 수신자 없음.
- **B3 `base_link` TF 부재(비전)** — YOLO 좌표 null.
- **B4 메인 레포 빔** — 통합 워크스페이스/빌드/launch 없음.

### 🟠 인터페이스 불일치
- **I1 hand_frame**: agent `robotiq_2f_85_tcp` ↔ 실구현 `robotiq_85_tcp`.
- **I2 grasp_frame z**: agent `0.13` ↔ 실구현 `0.0`(TCP가 이미 +0.13m) → agent 13cm 오차.
- **I3 lift 분리**: agent 별도 `lift` ↔ 실구현 lift가 pick 내부.
- **I4 세션 상태유지**: lift/place가 직전 pick의 attached object 가정 ↔ 실구현 stateless.
- **I5 scan 명령**: agent waypoint scan ↔ 실구현 미존재.
- **I6 URDF 이중 TCP**: `robotiq_85_tcp`(active) vs `robotiq_2f_85_tcp`(dead).
- **I7 error_code**: agent `0=SUCCESS` ↔ MoveItErrorCodes `SUCCESS=1`.
- **I8 z 이중보정**: agent center 보정 + 서버 `make_cylinder` 재보정 → +height 오차.
- **I9 좌표 null 미처리**: YOLO TF 실패 시 좌표 null인데 agent `_match`가 통과시킴.

### 🟡 정합/튜닝
- 로봇 모델 소스 3종 단일화, 카메라 `camera_info` 브릿지, 객체 치수 SDF 실측,
  `/tf`·`/clock`·controllers 브릿지, Coke Can의 COCO 클래스 검증.
