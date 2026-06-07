# 통합 전략 (Integration Strategy)

> 기준: [INTERFACE_CONTRACT.md](INTERFACE_CONTRACT.md)(SSOT) · [DECISIONS.md](DECISIONS.md)(확정 결정).
> 원칙: LLM_Agent의 계약을 고정 기준으로, 나머지를 정합시켜 **하나의 Gazebo 월드에서 E2E**를 달성한다.

---

## 1. 목표 워크스페이스 구조

```
ros2_ws/src/                    # 통합 colcon 워크스페이스 (메인레포가 .repos로 수집)
├── llm_agent_msgs/             # ← LLM_Agent (계약 SSOT: MoveItExecute.srv)
├── llm_agent/                  # ← LLM_Agent (오케스트레이터, 무변경)
├── robot_vision/               # ← ROBOT_VISION (YOLO)
├── ur3_moveit_module/          # ← Moveit_module (모션 실행 + /moveit/execute 서비스 서버, D15/D16)
└── ur3_mtc_pick_place/         # ← PANDA_ENV (URDF/SRDF 제공; bringup이 xacro 참조, D5/D6)

apt: ur_simulation_gz · ur_moveit_config · ur_description · robotiq_description · moveit · mtc-*

Arm_Module_Assembler/           # 메인 레포 자체
├── docs/                       # 분석·계약·결정·전략·모듈통합 문서
├── ur3_llm.repos               # vcs: 위 서브패키지 버전 고정(또는 submodule)
└── bringup.launch.py           # 원클릭: bringup(sim+move_group) + moveit_module + vision + agent
```
- 서브 레포는 **`vcstool`(.repos)** 또는 git submodule로 추적(원본 레포 히스토리 보존).
- MoveIt 서버 산출물은 **`Moveit_module/ur3_moveit_module`로 확정**(별도 PANDA 개조 불필요).
- 통합 Gazebo 월드/카메라 장착은 `ur3_moveit_module` bringup을 확장하거나 별도 `*_sim_bringup` 패키지로 추가(B1/B3/D13).

---

## 2. 단계별 로드맵

### Phase A — 토대 구축 (Blocker B1·B3·B4 해소)
1. **로봇 모델/프레임 단일 확정** (D1·D5·D6)
   - PANDA `ur3_with_gripper` URDF/SRDF 채택, dead `robotiq_2f_85.xacro` 제외.
   - LLM_Agent `agent.yaml`: `hand_frame=robotiq_85_tcp`, `grasp_frame_transform` z=0.0 반영(D2).
2. **통합 Gazebo 월드** (B1·D13)
   - `ur_simulation_gz` 기반 UR3 + ros2_control + 테이블 + 물체(cup/bottle) + **로봇/씬 장착 RGB-D 카메라**.
   - PANDA `ur3_mtc_demo.launch.py`를 출발점으로 확장.
3. **TF/브릿지 정비** (B3·D13)
   - `/tf`,`/tf_static`,`/clock`,`/joint_states`,controllers,`/camera/image|depth_image|camera_info` 브릿지.
   - `camera_link`를 TF 트리에 연결 → YOLO `position_3d_base_frame` non-null 성립.

### Phase B — MoveIt 서비스화 (Blocker B2 핵심) — **대부분 Moveit_module이 충족**
> 명령분해·상태유지·좌표보정·error_code·scan은 `ur3_moveit_module`이 이미 구현. 남은 것은 전송+파라미터.
4. **전송 서비스화**(D15): `moveit_module_node.py`를 `/moveit/execute`(`MoveItExecute`) 서비스 서버로 교체
   (`{cmd, params_json}`→평면 dict→`CommandRouter.handle`), `llm_agent_msgs` 의존 추가. 상세 [MOVEIT_MODULE_INTEGRATION.md §3].
5. **파라미터 정정**(D16): 모듈 `hand_frame=robotiq_85_tcp`, `grasp_frame_transform` z=0.0,
   로봇 고정값 서버 권위화. LLM_Agent `agent.yaml`도 동일 정정(D1·D2).

### Phase C — 단계별 통합 검증 (기존 mock 적극 활용)
6. **서비스 핸드셰이크**: `moveit_module(mock:=true)` 서비스 + 실 LLM_Agent → 각 cmd success 수신(Gazebo 불필요).
7. **MoveIt 실모션**: bringup + 모듈(real) + `test/mock_yolo_publisher.py`로 P1~P4 흐름 확인.
8. **Vision 단독**: 통합 월드에서 `position_3d_base_frame` 정확도·non-null 확인 + 좌표 null 가드(D9).
9. **Gemini 검증**: `parse_command`/`verify_pickup` 실호출(`.env` 키).
10. **E2E**: CLI "cup 집어" → 전 시나리오. `test/test_flow_mock.py`를 회귀 기준으로 유지.
11. **클래스/치수 확정**(D12): YOLO 로그로 시뮬 물체의 COCO class 확인 → objects.yaml·SDF 정합.

### Phase D — 마감
12. waypoint 워크스페이스 튜닝, 원클릭 `bringup.launch.py`, 트러블슈팅 문서, (선택) UR3_CONVENIENCE_ENV 선반 씬 통합.

---

## 3. 통합 검증 매트릭스

| 레벨 | 구성 | 통과 기준 |
|------|------|----------|
| L0 단위 | PhaseManager 등 | `pytest test_phase_manager.py` |
| L1 Agent+mock | mock YOLO + mock MoveIt | `test_flow_mock.py` 전 시나리오 |
| L2 MoveIt 실서버 | mock YOLO + 실 `/moveit/execute` + Gazebo | pick→lift→place 실행 성공 |
| L3 Vision 실연동 | 통합 월드 + 실 YOLO | base_frame 좌표 오차 허용범위 |
| L4 E2E | CLI + 전 모듈 | 자연어→Pick&Place 완료 |

---

## 4. 우선순위 Top 3 (지금 당장)

1. **Moveit_module 전송 서비스화 + 파라미터 정정** (D15·D16·D1·D2) — 에이전트와 실제로 통신. mock 핸드셰이크로 즉시 검증 가능.
2. **로봇 모델/프레임명 단일 확정** (D1·D5·D6) — 모든 좌표·IK의 기준점.
3. **통합 Gazebo 월드 + 카메라 장착 + TF 브릿지** (B1·B3·D13) — E2E의 물리적 전제.

> (1)은 Gazebo 없이도 mock 모드로 검증 가능 → 가장 먼저 착수 권장. (2)는 config·URDF, (3)은 시뮬·런치.

---

## 5. 리스크 / 오픈 이슈

| 리스크 | 영향 | 완화 |
|--------|------|------|
| 단계분리 상태유지(attach/detach) | 중 | Moveit_module이 `_held`+scene_manager로 이미 구현; 세션 경계 검증만 |
| MoveItPy plan/execute 동기 응답 지연 | 중 | 서비스 timeout(agent 10s) 내 응답·필요시 협의, 단일 lock으로 직렬화 |
| YOLO COCO가 시뮬 물체 미검출/오검출 | 중 | 커스텀 학습 or 검출 잘되는 모델배치/조명 튜닝(D12) |
| depth-rgb 비동기(시간동기 없음) | 중 | `message_filters` 시간동기 도입 검토 |
| Cartesian approach/lift 계획 실패 | 중 | min/max dist·IK solutions·timeout 튜닝(PANDA README 트러블슈팅) |
| place 좌표 보정 비대칭 잔존 | 중 | D8b로 Agent·서버 양측 정합 적용 |
