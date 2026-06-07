# 인터페이스 계약 (SSOT — Single Source of Truth)

> 모든 모듈이 **이 문서의 값**에 맞춘다. 본 문서와 개별 레포 코드가 다르면 **이 문서가 우선**이며,
> 해당 레포를 수정한다. 결정 근거는 [DECISIONS.md](DECISIONS.md).
> 최종 확정: 2026-06-07 (PANDA_ENV URDF/SRDF/launch ground truth 기준).

---

## 1. 데이터 흐름 토폴로지

```
[CLI stdin] ─► LLM Agent ─(/moveit/execute srv)─► MoveIt 서버 ─► Gazebo(UR3+2F-85)
                  ▲  ▲                                   │
   (/vision/detection_results)                     (/tf, /joint_states, controllers)
                  │  └──────(/tf: world↔base_link↔camera)──────┘
            YOLO(ROBOT_VISION) ◄─(/camera/image,/camera/depth_image,/camera/camera_info)─ Gazebo
```

---

## 2. 좌표·프레임 규약 (확정)

| 항목 | 확정값 | 근거 |
|------|--------|------|
| world_frame | `world` | 전 모듈 합의 |
| robot base | `base_link` | UR description |
| **world → base_link** | **identity** (0,0,0,0,0,0) | PANDA URDF `world_joint` + static TF |
| camera frame | `camera_link` | YOLO TF 소스 프레임 |
| **arm group** | `ur_manipulator` | `ur_moveit_config` SRDF (`name:=ur` 고정 필요, [D17]) |
| **gripper group** | `gripper` | PANDA SRDF |
| **end-effector** | `robotiq_2f_85` | PANDA SRDF `end_effector` |
| **hand_frame (TCP)** | **`robotiq_85_tcp`** | PANDA active URDF (base+0.13m) |
| gripper open / close | `open`(0.0) / `close`(0.8 rad) | PANDA SRDF group_state |
| arm home | `home` | UR SRDF group_state |
| 단위 / 회전 | meter / radian, RPY = **extrinsic xyz** | scipy `from_euler("xyz")` |
| 포즈 표기 | `[x, y, z, roll, pitch, yaw]` | 전 인터페이스 공통 |

> ⚠️ **변경점(D1)**: LLM_Agent의 기존 `hand_frame=robotiq_2f_85_tcp`는 **`robotiq_85_tcp`로 수정**했다(완료).
>
> ⚠️ **변경점(D17)**: PANDA_ENV·Moveit_module launch의 SRDF xacro 호출 시 `name:=$(ur_type)`(`ur3`)이
> `ur3_manipulator`를 생성해 `ur_moveit_config` kinematics/OMPL과 불일치. **`name:=ur` 하드코딩**으로 수정 완료.

---

## 3. YOLO → Agent : `/vision/detection_results`

- 타입: `std_msgs/String` (UTF-8 JSON). 토픽: `/vision/detection_results`.
- 발행 주기: 탐지가 있는 프레임마다(`num_detections>0`).

```json
{
  "timestamp_ns": 1718001234567890,
  "num_detections": 1,
  "objects": [
    {
      "class_name": "cup",
      "confidence": 0.892,
      "center_2d": {"u": 320, "v": 240},
      "distance_m": 0.452,
      "position_3d_camera_frame": {"X": 0.12, "Y": -0.04, "Z": 0.45},
      "position_3d_base_frame":   {"X": 0.40, "Y": 0.10,  "Z": 0.06}
    }
  ]
}
```

규약:
- 좌표 키는 **대문자 X/Y/Z**. 단위 m. 타겟 좌표는 `position_3d_base_frame` 사용(YOLO가 camera→base TF 완료).
- `class_name`은 **COCO**(yolov8n) 클래스명. Agent `objects.yaml` 키와 일치해야 함(`cup`,`bottle` 등).
- **TF 변환 실패로 `position_3d_base_frame`이 null이면, YOLO는 그 객체를 objects[]에서 제외**한다.
  (Agent도 방어적으로 null 좌표를 거른다.) [D9]
- 다중 동일 class → Agent가 `selection_policy`(기본 `highest_conf`)로 1개 선택.

---

## 4. Agent → MoveIt : `/moveit/execute`

- 타입: `llm_agent_msgs/srv/MoveItExecute`. 역할: **Agent=client, MoveIt=server**.
- **서버 구현 = `Moveit_module/ur3_moveit_module`** (전송 서비스로 통일, [D15]).
- 동시성: 단일 세션 직렬·blocking. 서버는 **계획+실행 완료 후** 응답.

> **서버측 페이로드 처리 규약**: 서버는 `Request{cmd, params_json}`를
> `flat = {"cmd": cmd, **json.loads(params_json or "{}")}`로 병합한 뒤 내부 라우터에 넘긴다
> (Moveit_module `CommandRouter.handle`는 이 평면 dict를 기대). 응답은 라우터 status의
> `success`/`error_code`/`error_message`를 그대로 매핑. [D15]
>
> **로봇 고정값은 서버 권위**(`hand_frame`·`grasp_frame_transform`·group/eef명): 서버 config 값 사용,
> 명령에 실려와도 무시. 동적값(object pose/dims, waypoints, target_pose)만 명령에서 취함. [D16]

```
# Request
string cmd          # "scan"|"pick"|"lift"|"place"|"release"|"home"
string params_json  # 명령별 파라미터(JSON 직렬화 문자열)
---
# Response
bool   success       # 계획+실행 최종 성공
int32  error_code    # 0=SUCCESS, 음수=실패 (서버가 MoveItErrorCodes를 정규화) [D7]
string error_message # 실패 사유(성공 시 "")
```

타임아웃: Agent 기본 `10s`(`agent.yaml: moveit.service_timeout_sec`). 장시간 모션은 협의.

### 4.1 명령별 `params_json` 스키마 (확정)

`scan` — waypoint 순차 이동(blocking, preempt 없음). Agent는 스캔 중 버퍼된 YOLO 프레임으로 탐지. [D10]
```json
{ "waypoints": [[x,y,z,r,p,y], ...] }   // world frame EEF 목표 자세 목록
```

`pick` — 충돌물체 spawn → grasp 생성 → approach → close → attach (**lift 제외**). [D3]
```json
{
  "arm_group_name": "ur_manipulator",
  "eef_name": "robotiq_2f_85",
  "hand_group_name": "gripper",
  "hand_frame": "robotiq_85_tcp",
  "hand_open_pose": "open",
  "hand_close_pose": "close",
  "object": {
    "name": "target_object",
    "shape": "cylinder",                 // "cylinder" | "box"
    "dimensions": [0.12, 0.025],         // cyl=[height,radius], box=[x,y,z]
    "pose_world": [0.40, 0.10, 0.06, 0.0, 0.0, 0.0]   // ★ 객체 '중심' world 좌표
  },
  "grasp_frame_transform": [0.0, 0.0, 0.0, 3.1416, 0.0, 0.0],   // ★ z=0.0 (TCP 기준)
  "approach_object_min_dist": 0.08, "approach_object_max_dist": 0.15,
  "lift_object_min_dist": 0.05, "lift_object_max_dist": 0.15,
  "max_solutions": 10
}
```

`lift` — 현재 잡은 상태에서 직선 상승(검증용). 서버는 attach 상태 유지. [D4]
```json
{ "direction": "z+", "distance_m": 0.1, "frame": "world" }
```

`place` — 배치(GeneratePlacePose + place + detach).
```json
{ "object_name": "target_object",
  "target_pose_world": [0.4, -0.2, 0.06, 0.0, 0.0, 0.0],   // ★ 객체 '중심' world 좌표
  "place_surface_offset": 0.001 }
```

`release`
```json
{ "hand_open_pose": "open" }
```

`home`
```json
{ "arm_home_pose": "home" }
```

### 4.2 좌표 보정 규약 (이중보정 방지) [D8]
- `pose_world` / `target_pose_world` 의 z는 **항상 객체 기하 중심**.
- **바닥→중심 보정(+height/2 등)은 Agent가 전담**한다.
- **MoveIt 서버는 받은 좌표를 그대로 사용**하며 추가 z 보정을 하지 않는다.
  (즉 PANDA `make_cylinder`의 `+height/2`는 서버 개조 시 제거.)
- `place_surface_offset`만 서버가 +z 방향 여유로 적용 가능.

---

## 5. Gazebo / 시뮬 인터페이스 (통합 월드)

| 토픽/프레임 | 제공자 | 비고 |
|------|--------|------|
| `/camera/image`, `/camera/depth_image` | Gazebo(rgbd) → `ros_gz_bridge` | YOLO 입력 |
| `/camera/camera_info` | 〃 | 내부파라미터(하드코딩 554.25 대체 권장) |
| `/clock` | Gazebo | `use_sim_time:=true` 전 노드 공통 |
| `/tf`,`/tf_static` | robot_state_publisher + static TF | world↔base_link↔...↔camera_link |
| `/joint_states`, controllers | `ur_simulation_gz` ros2_control | MoveIt 실행 |

- 카메라는 **로봇/씬에 장착**되고 `camera_link`가 TF 트리에 연결되어야 함(현재 미연결). [B3]
- 내부파라미터는 카메라 SDF의 FOV/해상도와 일치해야 함(현 554.25 = FOV 1.047·640 한정).

---

## 6. 외부 의존

| 항목 | 값 |
|------|-----|
| LLM | Google Gemini `gemini-2.5-flash`, `.env` `GEMINI_API_KEY` |
| YOLO 모델 | `yolov8n.pt` (COCO 80 classes) |
| MoveIt | MoveIt2 + MoveIt Task Constructor (Humble) |
| 로봇/시뮬 패키지 | `ur_description`,`ur_moveit_config`,`robotiq_description`,`ur_simulation_gz` |
