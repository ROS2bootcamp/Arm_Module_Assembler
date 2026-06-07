# Moveit_module 기반 통합 구조 정립

> 작성일 2026-06-07 · `Moveit_module@5414350`(feat: UR3 MoveIt control module) 재확인 반영.
> 전제: Agent↔MoveIt **전송 방식 = 서비스로 통일**(Moveit_module가 서비스 서버로 적응, LLM_Agent 무변경).
> 관련: [DECISIONS.md](DECISIONS.md) D15·D16, [INTERFACE_CONTRACT.md](INTERFACE_CONTRACT.md) §4.

---

## 1. Moveit_module 재확인 요약

업데이트된 `Moveit_module/src/ur3_moveit_module`(ament_python)은 **내부 설계가 우수**하다.

| 컴포넌트 | 책임 | 통합 관점 |
|----------|------|----------|
| `moveit_module_node.py` | ROS2 I/O + 단일워커 순차처리 | 🔴 **여기만 교체**(토픽→서비스) |
| `command_router.py` | JSON→dispatch→status dict, 예외격리 | ✅ 전송계층과 분리 → 재사용 |
| `handlers.py` | scan/pick/lift/place/release/home (lift 분리 준수) | ✅ 재사용 |
| `arm_controller.py`/`gripper_controller.py`/`scene_manager.py` | MoveItPy·그리퍼·PlanningScene | ✅ 재사용 |
| `grasp_planner.py`/`geometry_utils.py` | 순수 기하(파지/배치 pose) | ✅ 재사용(파라미터만 정정) |
| `ur3_moveit_bringup.launch.py` | Gazebo+move_group, PANDA URDF/SRDF 재사용 | ✅ 통합 bringup |
| 단위테스트 29개 | 라우팅·기하·핸들러 | ✅ 유지 |

핵심: `CommandRouter.handle(json_str) -> status_dict` 가 **ROS 비의존**이라 I/O만 바꾸면 전부 살아남는다.

### 발견된 충돌 (재확인)
- **C1 전송 불일치(치명)**: 모듈=토픽(`/moveit_command`·`/moveit_status`, 평면 JSON) ↔
  LLM_Agent(양 브랜치)=서비스(`/moveit/execute`, `MoveItExecute.srv`, `{cmd, params_json}`). → **통신 불가.**
  모듈 설계서 §1.1의 "Agent가 토픽/`send_and_wait`" 분석은 사실과 다름(그 코드 없음).
- **C2 모듈 자기모순(파라미터)**: bringup이 쓰는 URDF는 TCP가 `robotiq_85_tcp`인데
  `module_params.yaml` 기본 `hand_frame=robotiq_2f_85_tcp`(URDF에 없음→IK 실패),
  `grasp_frame_transform` z=0.13(해당 TCP는 이미 base+0.13m → 0.0이어야 함).
- **C3 무해**: 그리퍼는 named-pose(open/close) 추상화 → `finger_joint -0.7` 주석은 영향 없음.

---

## 2. 확정 구조 (서비스 통일)

```
LLM_Agent (무변경)                         Moveit_module (I/O 교체)
  MoveItClient.call_and_wait(cmd, params)     MoveItExecute 서비스 서버 /moveit/execute
    └─ /moveit/execute (MoveItExecute.srv) ─────► callback:
         Request{cmd, params_json}                 flat = {"cmd":cmd, **json.loads(params_json)}
                                                    status = CommandRouter.handle(json.dumps(flat))
         Response{success, error_code, error_message} ◄── status 매핑
```

- **계약(SSOT)**: `llm_agent_msgs/srv/MoveItExecute` (LLM_Agent 소유, 변경 없음).
- 모듈은 `MoveItExecute.srv`를 사용하기 위해 **`llm_agent_msgs`에 의존**한다.
- 모듈 내부 `CommandRouter`/`handlers`는 그대로. 토픽 모드는 단독 디버그용으로 선택 유지 가능.

---

## 3. Moveit_module 변경 명세 (구현 항목)

### 3.1 전송: 서비스 서버화 [D15]
`moveit_module_node.py`:
- 구독/발행(`/moveit_command`,`/moveit_status`) → **서비스 서버 `/moveit/execute`(`MoveItExecute`)** 로 교체.
- 콜백(요지, ~30줄):
  ```python
  def _on_execute(self, req, resp):
      with self._lock:                       # 동시 모션 방지(순차 보장)
          params = json.loads(req.params_json or "{}")
          flat = {"cmd": req.cmd, **params}
          status = self._router.handle(json.dumps(flat))
      resp.success       = status["success"]
      resp.error_code    = status["error_code"]   # 이미 0=SUCCESS 정규화됨
      resp.error_message = status["error_message"]
      return resp
  ```
- `ReentrantCallbackGroup`+`MultiThreadedExecutor` 유지. 서비스는 **실행 완료 후 응답**(동기) → 에이전트의 `call_and_wait` 의미와 일치.
- `package.xml`/`setup.py`에 `llm_agent_msgs` 의존 추가. `mock:=true`는 서비스 뒤에서 그대로 동작.

### 3.2 파라미터 정정 [D16]
`module_params.yaml` + `_PARAM_DEFAULTS`:
- `hand_frame: robotiq_2f_85_tcp` → **`robotiq_85_tcp`** (bringup URDF 실제 프레임).
- `grasp_frame_transform: [0,0,0.13,π,0,0]` → **`[0,0,0.0,π,0,0]`** (TCP가 이미 핑거팁).

### 3.3 로봇 고정값 = 서버 권위 [D16]
- 현재 `handlers._get`은 **명령 JSON 값을 config보다 우선** → 에이전트가 보내는 (현재 잘못된)
  `hand_frame`/`grasp_frame_transform`가 모듈의 올바른 기본값을 덮어쓴다.
- **결정**: `hand_frame`, `grasp_frame_transform`, group/eef 명칭 등 **로봇 고정값은 서버 config를 권위로** 사용
  (명령 override 무시). 동적값(object pose/dimensions, waypoints, target_pose)만 명령에서 취한다.
- 보강: LLM_Agent `agent.yaml`도 올바른 값으로 정정(D1·D2)하여 양측 일치(이중 안전).

---

## 4. 통합 워크스페이스 구조 (`ros2_ws/src/`)

```
ros2_ws/src/
├── llm_agent_msgs/         # ← LLM_Agent : MoveItExecute.srv (계약 SSOT)
├── llm_agent/              # ← LLM_Agent : 오케스트레이터 (무변경)
├── robot_vision/           # ← ROBOT_VISION : YOLO 노드
├── ur3_moveit_module/      # ← Moveit_module : 모션 실행 + /moveit/execute 서비스 서버
└── ur3_mtc_pick_place/     # ← PANDA_ENV : URDF/SRDF 제공(bringup이 xacro 참조)

apt 의존: ur_simulation_gz · ur_moveit_config · ur_description · robotiq_description
          moveit · moveit-task-constructor-* · py-binding-tools
```
- 메인 레포는 위 서브패키지를 **vcs(`.repos`) 또는 submodule**로 모음. `ur3_moveit_module`이 `llm_agent_msgs`를 빌드 의존.
- ur3-glapper는 모델 단일화(PANDA URDF)로 흡수 → 독립 빌드대상 제외.

---

## 5. 실행 순서 (통합 E2E)

```bash
export ROS_DOMAIN_ID=212          # 전 터미널 공통(ur3-glapper 관례)

# T1) Gazebo + ros2_control + move_group (+RViz)
ros2 launch ur3_moveit_module ur3_moveit_bringup.launch.py

# T2) MoveIt 모듈 (서비스 서버 /moveit/execute)
ros2 launch ur3_moveit_module moveit_module.launch.py     # mock:=true 로 핸드셰이크만도 가능

# T3) YOLO 비전 (통합 월드에서 camera_link TF 연결 필요 — B3/D13)
ros2 launch robot_vision vision_bringup.launch.py

# T4) LLM 에이전트 (CLI 자연어 입력)
ros2 launch llm_agent agent.launch.py
```

---

## 6. 검증 단계

| 레벨 | 구성 | 통과 기준 |
|------|------|----------|
| L1 | 모듈 단위테스트 29개 | `pytest` 통과(전송 변경 후에도 router/handlers 무영향) |
| L2 | 서비스 핸드셰이크: `moveit_module(mock:=true)` + 실 LLM_Agent | 에이전트 `call_and_wait`가 각 cmd에 success 수신 |
| L3 | 실모션: bringup + 모듈(real) + mock YOLO | scan→pick→lift→place→home 실행 성공 |
| L4 | 통합 비전: + 실 YOLO(통합 월드) | `position_3d_base_frame` non-null·정확 |
| L5 | 전체 E2E | CLI "cup 집어" → Pick&Place 완료 |

> L2는 Gazebo 없이 가능 → 전송 서비스화의 1차 검증으로 즉시 활용.

---

## 7. 남은 환경 의존 이슈 (모듈 외부, 기존 Blocker 유지)
- **B1/B3/D13**: 통합 Gazebo 월드에 카메라 장착 + `camera_link` TF + `/tf`·`/clock`·camera_info 브릿지.
  (Moveit_module bringup은 로봇/move_group만, 카메라는 미포함 → 비전 통합 시 확장 필요.)
- **D14**: ur3-glapper CMakeLists install 결함 — 모델 단일화로 회피.
- 그리퍼 ros2_control 컨트롤러(Robotiq 2F-85) Gazebo 설정 확정(모듈 설계서 §6-2).
