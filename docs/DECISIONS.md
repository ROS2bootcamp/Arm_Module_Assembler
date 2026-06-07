# 통합 결정 사항 (Decisions & Resolutions)

> "통합 정리상의 모호성·불일치를 전부 해소" 결과. 각 항목 = **확정 결정 + 근거 + 레포별 수정 액션**.
> 코드 수정은 후속 작업이며, 여기서는 **무엇을 어떻게 맞출지**를 확정한다.
> 기준 ground truth: PANDA_ENV `ur3_with_gripper.urdf.xacro` / `.srdf.xacro` / `ur3_mtc_demo.launch.py`.

범례: ✅ ground truth로 확정 · 🔧 코드 수정 필요 · 📌 설계 결정

> **2026-06-07 갱신**: `Moveit_module`이 실제 구현으로 채워짐(`ur3_moveit_module`). 이에 따라
> MoveIt 서버는 더 이상 "PANDA 스크립트 개조"가 아니라 **Moveit_module 적응**으로 확정.
> 신규 D15(전송=서비스)·D16(모듈 파라미터) 추가, D3·D4·D7·D8은 Moveit_module이 이미 상당부분 충족.
> 상세 구조: [MOVEIT_MODULE_INTEGRATION.md](MOVEIT_MODULE_INTEGRATION.md).
>
> **2026-06-07 모델·프레임 단일화**: D15·D16 구현 완료(Moveit_module PR#1 `feat/service-interface`).
> LLM_Agent `agent.yaml` 프레임 정정 완료(PR `fix/agent-frame-params`). 신규 D17(SRDF name 인수 버그) 추가.
> PANDA_ENV·Moveit_module launch의 SRDF `name:=ur3` → `name:=ur` 정정 완료(각 PR).

---

## D1 — hand_frame(TCP) 명칭 통일 🔧✅
- **불일치**: LLM_Agent `robotiq_2f_85_tcp` ↔ PANDA active `robotiq_85_tcp`.
- **근거**: PANDA `ur3_with_gripper.urdf.xacro`가 공식 `robotiq_description`(링크 prefix `robotiq_85_*`)를 include하고
  `robotiq_85_tcp`(=`robotiq_85_base_link` +0.13m z)를 추가. SRDF/`ur3_mtc_config.yaml`도 `robotiq_85_tcp` 사용.
  `robotiq_2f_85_tcp`는 미사용 dead 파일에만 존재(D6).
- **결정**: 정규 TCP 프레임 = **`robotiq_85_tcp`**.
- **액션**: LLM_Agent `config/agent.yaml`의 `moveit.hand_frame`을 `robotiq_85_tcp`로 변경.
  `MOVEIT_INTERFACE.md`·`ARCHITECTURE.md`·`CONTEXT.md`의 `robotiq_2f_85_tcp` 표기도 갱신.

## D2 — grasp_frame_transform z 정정 🔧✅
- **불일치**: LLM_Agent `[0,0,0.13,π,0,0]` ↔ PANDA `[0,0,0.0,π,0,0]`.
- **근거**: `hand_frame=robotiq_85_tcp`가 **이미** base+0.13m(핑거팁 중점). MTC `setIKFrame(tf, hand_frame)`의
  `tf`는 TCP 기준 추가 오프셋이므로 z=0.0이어야 함. 0.13이면 IK 프레임이 13cm 더 전진 → 파지 13cm 미달.
- **결정**: `grasp_frame_transform = [0.0, 0.0, 0.0, 3.1416, 0.0, 0.0]`. (roll=π: 그리퍼가 물체 향해 하강)
- **액션**: LLM_Agent `agent.yaml`·문서의 z값 0.13→0.0.

## D3 — pick에서 lift 분리 🔧📌
- **불일치**: Agent는 `pick`(파지까지) 후 별도 `lift` 호출 ↔ PANDA는 lift가 pick 컨테이너 내부(stage 4-7).
- **결정**: 서버 `pick` 콜백은 **approach·grasp·IK·allow-collision·close·attach까지만** 수행, lift 제외.
- **액션**: MoveIt 서버 개조 시 pick MTC에서 lift stage 제거 → 별도 `lift` 명령으로 이관.

## D4 — 단계 간 세션 상태 유지 🔧📌
- **이슈**: `lift`/`place`는 직전 `pick`의 attached object와 PlanningScene을 알아야 함. MTC task는 plan 단위 stateless.
- **결정**: MoveIt 서버는 **세션 동안 PlanningScene/attached object 상태를 유지**한다(서버 멤버로 보존,
  명령 콜백 간 공유). pick에서 attach → lift/place가 동일 attach 상태에서 동작 → place에서 detach.
- **액션**: 서버를 stateful 노드로 설계(현재 세션의 object_name·attach 링크·collision 허용 목록 보존).

## D5 — 로봇 모델 / Gazebo 소스 단일화 📌✅
- **이슈**: 로봇 모델 3종(공식 `ur_simulation_gz` / ur3-glapper 커스텀 / PANDA 커스텀).
- **근거**: PANDA `ur3_mtc_demo.launch.py`가 이미 **공식 `ur_simulation_gz`**(Gazebo+ros2_control) +
  `ur3_with_gripper.urdf/srdf` + `ur_moveit_config`(kinematics/OMPL) + `world→base_link` static TF +
  MTC capability를 통합 구성. GAZEBO_ENV가 권장한 `ur-simulation-gz`와 동일.
- **결정**: 정규 스택 = **`ur_simulation_gz`(시뮬·컨트롤러) + PANDA의 `ur3_with_gripper` URDF/SRDF + `ur_moveit_config`**.
  PANDA의 demo launch를 통합 bringup의 출발점으로 삼는다.
- **액션**: ur3-glapper는 모델 단일화에 흡수(독립 빌드대상에서 제외). 통합 워크스페이스는 PANDA 경로 채택.

## D6 — dead 파일 제거 🔧✅
- **이슈**: PANDA `urdf/robotiq_2f_85.xacro`(`robotiq_2f_85_tcp` 정의)는 어디서도 include 안 됨(grep 확인) → 혼동 원인.
- **결정**: 제거(또는 통합 패키지로 이관하지 않음).
- **액션**: 통합 MoveIt 패키지 구성 시 미포함.

## D7 — error_code 규약 🔧📌
- **불일치**: Agent는 `0=SUCCESS` 가정 ↔ MoveItErrorCodes는 `SUCCESS=1`.
- **결정**: **서버가 정규화** — 성공 시 `error_code=0`, 실패 시 음수. `error_message`에 사람이 읽는 사유.
- **액션**: 서버 응답 매핑(`MoveItErrorCodes.SUCCESS → 0`, 그 외 → 음수).

## D8 — z 이중보정 방지 🔧📌
- **이슈**: Agent `_build_pick_params`가 바닥→중심 보정(+h/2) 후 `pose_world` 전달.
  PANDA `make_cylinder`가 다시 `+height/2` → 합계 +height 오차.
- **결정**: **모든 world 좌표 = 객체 중심**, 보정은 **Agent 전담**, 서버는 추가 보정 없음.
- **액션**: 서버 spawn에서 `+height/2` 제거. place의 `+object_height/2`도 제거(아래 D8b).

### D8b — place 좌표 의미 통일 🔧📌
- **이슈**: pick `pose_world`는 중심, place `target_pose_world`는 PANDA가 `+od/2` 보정 → 비대칭.
- **결정**: place `target_pose_world`도 **객체 중심**으로 통일. Agent가 place 시에도 동일 중심 보정 적용.
  (현 `targets.yaml` z=0.06은 0.12 실린더의 중심 높이와 일치 → 중심 해석으로 일관.)
- **액션**: Agent P4가 place 좌표에 pick과 동일한 중심 보정 적용(현재 raw 전달은 보강 필요).
  서버 place에서 `+od[0]/2` 제거, `place_surface_offset`만 여유로 가산.

## D9 — YOLO 좌표 null 처리 🔧📌
- **이슈**: TF 실패 시 `position_3d_base_frame` 키 값이 null인데, Agent `_match`는 `distance_m>0`만 보고 통과 →
  P2에서 `base['X']` None으로 크래시 가능.
- **결정**: ① YOLO는 TF 실패 객체를 objects[]에서 **제외** 발행. ② Agent `_match`도 좌표 null이면 후보 제외(방어).
- **액션**: ROBOT_VISION `yolo_detector.py`(X_base None이면 append 생략) + Agent `yolo_subscriber._match` 가드.

## D10 — scan 비동기 중단 📌✅
- **결정**: `scan`은 **blocking**으로 전 waypoint 순회. preempt(조기중단) 미도입.
- **근거**: Agent `_run_p1`은 buffer를 먼저 clear → scan 서비스 호출(블로킹) → 그동안 YOLO가 deque에 버퍼 →
  반환 후 `wait_for_detection`이 버퍼 검사. 즉 **스캔 도중 수집된 프레임으로 사후 탐지**하므로 preempt 불필요.
- **액션**: 서버 scan = 다중 waypoint MoveTo 순차. (별도 액션 전환 불필요)

## D11 — base_link ↔ world 변환 ✅
- **미결(CONTEXT #2) 해소**: **identity**. PANDA URDF `world_joint`(origin 0) + demo launch의
  `static_transform_publisher 0 0 0 0 0 0 world base_link`로 확정. Agent `TFTransformer`(base→world) 그대로 유효.

## D12 — 객체 클래스/치수 📌
- **결정**: 클래스명은 COCO(yolov8n) 기준 — `cup`, `bottle` 채택(objects.yaml과 일치).
  Coke Can 등 시뮬 모델이 COCO에서 어떤 class로 잡히는지 **통합 월드에서 실측 검증** 후 objects.yaml 확정.
  치수도 통합 Gazebo SDF 실측으로 확정(현재 값은 잠정).
- **액션**: 통합 월드 구동 후 YOLO 로그로 class/conf 확인 → objects.yaml·SDF 정합.

## D13 — 카메라 장착 & camera_info 📌🔧
- **이슈**: 카메라가 로봇과 무관한 독립 월드에 존재, 내부파라미터 하드코딩(554.25).
- **결정**: 통합 월드에서 카메라를 **씬/로봇에 장착**하고 `camera_link`를 TF 트리에 연결.
  내부파라미터는 `/camera/camera_info` 브릿지로 받아 사용(하드코딩 대체) 권장.
- **액션**: 통합 월드 SDF에 카메라 추가 + `ros_gz_bridge`에 camera_info·tf·clock 추가.

## D14 — 빌드 결함 🔧
- **이슈**: ur3-glapper CMakeLists가 미존재 `launch/config/worlds` install → 빌드 실패 소지.
- **결정**: D5로 ur3-glapper를 독립 빌드대상에서 제외하므로 영향 소거. (잔존 사용 시 install 라인 수정)

## D15 — Agent↔MoveIt 전송 = 서비스로 통일 ✅📌 (2026-06-07 확정·구현 완료)
- **불일치**: 신규 Moveit_module은 **토픽**(`/moveit_command`·`/moveit_status`, 평면 JSON),
  LLM_Agent 양 브랜치는 **서비스**(`/moveit/execute`, `MoveItExecute.srv`, `{cmd, params_json}`).
  모듈 설계서 §1.1의 "Agent=토픽/`send_and_wait`" 분석은 사실과 다름(LLM_Agent에 그 코드 없음).
- **결정(사용자 승인)**: **서비스로 통일**. Moveit_module이 `/moveit/execute` **서비스 서버**를 노출하도록
  I/O 계층만 교체. LLM_Agent·`MoveItExecute.srv`·`mock_moveit_server.py`는 무변경.
- **근거**: LLM_Agent가 계약 소유자·더 성숙·서비스 mock까지 보유. 모듈 핵심(`CommandRouter.handle`)은
  전송 비의존이라 변경 국소화(서비스 콜백 ~30줄).
- **액션**: `moveit_module_node.py` 서비스 서버화 + `llm_agent_msgs` 의존 추가. 페이로드 병합
  `{"cmd":req.cmd, **json.loads(req.params_json)}` → router. 토픽 모드는 단독 디버그용 선택 유지.
  상세 [MOVEIT_MODULE_INTEGRATION.md §3.1](MOVEIT_MODULE_INTEGRATION.md).
- **구현**: Moveit_module 브랜치 `feat/service-interface` PR#1. `CommandRouter.handle_request()` 추가.
  단위테스트 34개 통과, flake8 clean.

## D16 — Moveit_module 파라미터 정정 & 고정값 서버권위 ✅ (구현 완료)
- **이슈**: ① 모듈 기본 `hand_frame=robotiq_2f_85_tcp`인데 bringup URDF의 실제 TCP는 `robotiq_85_tcp`
  (없는 프레임 → IK 실패). ② `grasp_frame_transform` z=0.13인데 그 TCP는 이미 base+0.13m → 0.0이어야 함.
  ③ `handlers._get`이 **명령값을 config보다 우선** → 에이전트가 보내는 (현재 틀린) 고정값이 올바른 기본값을 덮어씀.
- **결정**: 모듈 `module_params.yaml`/`_PARAM_DEFAULTS`를 `hand_frame=robotiq_85_tcp`(D1),
  `grasp_frame_transform=[0,0,0.0,π,0,0]`(D2)로 정정. **로봇 고정값(hand_frame·grasp_frame_transform·
  group/eef명)은 서버 config를 권위로** 사용(명령 override 무시), 동적값만 명령에서 취함.
  추가로 LLM_Agent `agent.yaml`도 정정(D1·D2)하여 이중 안전.
- **액션**: 모듈 config·`handlers._get` 고정값 분기 + LLM_Agent agent.yaml 수정.
- **구현**: Moveit_module `feat/service-interface` 포함. LLM_Agent `fix/agent-frame-params` PR.

---

## D17 — SRDF `name` 인수 버그: `ur3_manipulator` → `ur_manipulator` ✅ (구현 완료)
- **불일치**: PANDA_ENV `ur3_mtc_demo.launch.py`·Moveit_module `moveit_module.launch.py`가
  SRDF xacro에 `name:=$(ur_type)` (`ur3`)을 전달 → 생성 그룹명 `ur3_manipulator`.
  그런데 `ur_moveit_config/kinematics.yaml` `ompl_planning.yaml`은 `ur_manipulator` 하드코딩.
  → **IK·플래닝 파이프라인이 그룹명 불일치로 완전히 동작 불가**.
- **근거**: `ur_moveit_config/srdf/ur_macro.srdf.xacro`의 컨벤션은 `name="ur"` (로봇 타입 무관 고정).
  `ur_simulation_gz/launch/ur_sim_moveit.launch.py`도 동일 컨벤션 사용.
  xacro 확장 검증: `name:=ur` → `<group name="ur_manipulator">`, `name:=ur3` → `<group name="ur3_manipulator">`.
- **결정**: SRDF xacro 호출 시 **`name:=ur` 하드코딩** (ur_type과 무관하게 고정).
- **액션**: PANDA_ENV `ur3_mtc_demo.launch.py` `" name:=", ur_type` → `" name:=ur "`.
  Moveit_module `moveit_module.launch.py` 동일 수정.
- **구현**: PANDA_ENV 브랜치 `fix/srdf-group-name`, Moveit_module `feat/service-interface`에 포함.

---

## 부록 A — LLM_Agent 측 수정 요약 (계약 정합)
| 파일 | 변경 | 상태 |
|------|------|------|
| `config/agent.yaml` | `hand_frame: robotiq_85_tcp` (D1), `grasp_frame_transform: [...,0.0,...]` z=0.0 (D2) | ✅ 완료 (PR `fix/agent-frame-params`) |
| `agent_node.py` `_run_p4` | place 좌표에 중심 보정 적용 (D8b) | 🔧 미완 |
| `yolo_subscriber.py` `_match` | 좌표 null 가드 (D9) | 🔧 미완 |
| `MOVEIT_INTERFACE.md`/`ARCHITECTURE.md`/`CONTEXT.md` | TCP명·grasp z 표기 갱신 | 🔧 미완 |

## 부록 B — MoveIt 서버 측 요구사항 요약 (산출물 = `Moveit_module/ur3_moveit_module`)
> 대부분 Moveit_module이 **이미 충족**. 남은 변경은 전송 서비스화(D15)·파라미터(D16)뿐.

| 항목 | 요구 | Moveit_module 현황 |
|------|------|------|
| 명령 분해(scan/pick/lift/place/release/home), lift 분리 (D3) | handlers.py | ✅ 충족 |
| 상태 유지(attach/detach) (D4) | `_held` + scene_manager | ✅ 충족 |
| 좌표 중심 사용, +h/2 비중복 (D8/D8b) | grasp_planner가 중심보정 1회 | ✅ 충족(에이전트와 중복 안 되게 D8 규약 준수 확인) |
| error_code 0=SUCCESS 정규화 (D7) | command_router `ERR_OK=0` | ✅ 충족 |
| scan 다중 waypoint, blocking (D10) | handlers.scan | ✅ 충족 |
| **전송 = /moveit/execute 서비스 (D15)** | 서비스 서버 구현 완료 | ✅ 완료 (PR#1) |
| **파라미터 정정 hand_frame/grasp z (D16)** | 파라미터 정정·ROBOT_FIXED_KEYS 추가 | ✅ 완료 (PR#1) |
| **SRDF name:=ur 고정 (D17)** | `moveit_module.launch.py` 수정 | ✅ 완료 (PR#1 포함) |
| 산출 위치 | `Moveit_module` 레포 `ur3_moveit_module` 패키지 | ✅ 확정 |

## 부록 C — ROBOT_VISION 측 요구사항 요약
| 항목 | 요구 |
|------|------|
| TF 실패 객체 제외 발행 (D9) | `position_3d_base_frame` null이면 objects[]에서 제외 |
| 통합 월드 연동 (D13) | 카메라 TF 연결, camera_info 사용 |
