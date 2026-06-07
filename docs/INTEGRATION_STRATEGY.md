# 통합 전략 (Integration Strategy)

> 기준: [INTERFACE_CONTRACT.md](INTERFACE_CONTRACT.md)(SSOT) · [DECISIONS.md](DECISIONS.md)(확정 결정).
> 원칙: LLM_Agent의 계약을 고정 기준으로, 나머지를 정합시켜 **하나의 Gazebo 월드에서 E2E**를 달성한다.

---

## 1. 목표 워크스페이스 구조

```
Arm_Module_Assembler/
├── docs/                       # 본 문서들 (분석·계약·결정·전략)
├── src/
│   ├── llm_agent_msgs/         # ← LLM_Agent (계약 SSOT: MoveItExecute.srv)
│   ├── llm_agent/              # ← LLM_Agent (오케스트레이터)
│   ├── robot_vision/           # ← ROBOT_VISION (YOLO)
│   ├── ur3_description/        # ← PANDA ur3_with_gripper URDF/SRDF (단일 로봇모델, D5/D6)
│   ├── ur3_moveit_server/      # ← PANDA ur3_pick_place.py → /moveit/execute 서비스 서버 (D3/D4/D7/D8)
│   └── ur3_sim_bringup/        # ← 통합 Gazebo 월드 + 카메라장착 + bridge + spawn (B1/B3/D13)
├── ur3_llm.repos               # vcs: 각 서브레포 버전 고정(또는 git submodule)
└── bringup.launch.py           # 원클릭: sim + moveit_server + vision + agent
```
- 서브 레포는 **`vcstool`(.repos)** 또는 git submodule로 추적(원본 레포 히스토리 보존).
- `ur3_moveit_server` 산출물은 빈 `Moveit_module` 레포에 두고 vcs로 끌어와도 됨.

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

### Phase B — MoveIt 서비스화 (Blocker B2 핵심)
4. `ur3_pick_place.py` → **`/moveit/execute` 6콜백 서비스 서버** 개조 (MOVEIT_INTERFACE.md §6 가이드)
   - `scan`(다중 MoveTo, D10) / `pick`(lift 제외, D3) / `lift`(MoveRelative, D4) / `place` / `release` / `home`.
   - **세션 상태유지**(D4): attached object·PlanningScene 보존.
   - **좌표 중심 사용·이중보정 제거**(D8/D8b), **error_code 0=SUCCESS 정규화**(D7).
   - 로봇 고정값은 서버 기본 파라미터, 요청값으로 override.

### Phase C — 단계별 통합 검증 (기존 mock 적극 활용)
5. **MoveIt 단독**: `test/mock_yolo_publisher.py` + 실서버로 P1~P4 흐름 확인.
6. **Vision 단독**: 통합 월드에서 `position_3d_base_frame` 정확도·non-null 확인 + 좌표 null 가드(D9).
7. **Gemini 검증**: `parse_command`/`verify_pickup` 실호출(`.env` 키).
8. **E2E**: CLI "cup 집어" → 전 시나리오. `test/test_flow_mock.py`를 회귀 기준으로 유지.
9. **클래스/치수 확정**(D12): YOLO 로그로 시뮬 물체의 COCO class 확인 → objects.yaml·SDF 정합.

### Phase D — 마감
10. waypoint 워크스페이스 튜닝, 원클릭 `bringup.launch.py`, 트러블슈팅 문서, (선택) UR3_CONVENIENCE_ENV 선반 씬 통합.

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

1. **로봇 모델/프레임명 단일 확정** (D1·D2·D5·D6) — 모든 좌표·IK의 기준점.
2. **통합 Gazebo 월드 + 카메라 장착 + TF 브릿지** (B1·B3·D13) — E2E의 물리적 전제.
3. **MoveIt 서비스 서버 개조** (B2·D3·D4·D7·D8) — 에이전트가 실제 호출할 대상.

> 셋은 병렬 가능: (1)은 설정·URDF, (2)는 시뮬·런치, (3)은 MTC 개조. (2)가 (3) 검증의 무대를 제공.

---

## 5. 리스크 / 오픈 이슈

| 리스크 | 영향 | 완화 |
|--------|------|------|
| MTC 단계분리 시 상태유지 복잡 | 높음 | attached object를 서버 멤버로 보존, place에서 명시 detach |
| YOLO COCO가 시뮬 물체 미검출/오검출 | 중 | 커스텀 학습 or 검출 잘되는 모델배치/조명 튜닝(D12) |
| depth-rgb 비동기(시간동기 없음) | 중 | `message_filters` 시간동기 도입 검토 |
| Cartesian approach/lift 계획 실패 | 중 | min/max dist·IK solutions·timeout 튜닝(PANDA README 트러블슈팅) |
| place 좌표 보정 비대칭 잔존 | 중 | D8b로 Agent·서버 양측 정합 적용 |
