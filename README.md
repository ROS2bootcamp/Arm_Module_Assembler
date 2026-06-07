# Arm_Module_Assembler

> **UR3 robot arm × LLM** 제어 프로젝트의 **통합(Integration) 메인 레포**.
> 각 팀이 개별 개발한 모듈(LLM Agent / YOLO Vision / MoveIt / 로봇모델·시뮬)을
> 하나의 Gazebo 시나리오로 통합하기 위한 기준 레포입니다.

## 프로젝트 목표

CLI 자연어 명령 → (LLM) 의도 파싱 → (YOLO) 물체 탐지·절대좌표 → (MoveIt) Pick & Place
동작 계획·실행까지, ROS 2 Humble + Ignition Gazebo 위에서 자동 수행한다.

```
[CLI 자연어] → LLM Agent ──/moveit/execute(srv)──→ MoveIt 서버 → Gazebo(UR3+2F-85)
                  ▲
                  └──/vision/detection_results(JSON)── YOLO(ROBOT_VISION)
```

## 구성 모듈 (서브 레포)

| 모듈 | 레포 | 역할 | 상태 |
|------|------|------|------|
| LLM Agent | `ROS2bootcamp/LLM_Agent` | 시나리오 오케스트레이터(P1~P4), 인터페이스 계약 정의자 | 🟢 성숙 |
| YOLO Vision | `ROS2bootcamp/ROBOT_VISION` | 물체 인식 + 3D 좌표 발행 | 🟡 노드 OK / 환경 분리 |
| MoveIt (구현) | `ROS2bootcamp/PANDA_ENV` (`ur3_mtc_pick_place`) | MTC Pick&Place + 통합 bringup launch | 🟡 one-shot |
| MoveIt (지정) | `ROS2bootcamp/Moveit_module` | (서비스 서버 산출물 예정 위치) | 🔴 빈 레포 |
| 로봇 모델 | `ROS2bootcamp/ur3-glapper` | UR3 + Robotiq 2F-85 URDF | 🟡 RViz 단계 |
| Gazebo 환경 | `ROS2bootcamp/GAZEBO_ENV` | 공식 `ur-simulation-gz` 기반 UR3 시뮬 | 🔴 가이드만 |
| 픽킹 환경 | `ROS2bootcamp/UR3_CONVENIENCE_ENV` | 편의점 선반 picking 환경 | 🔴 미착수 |

## 문서

| 문서 | 내용 |
|------|------|
| [docs/INTEGRATION_ANALYSIS.md](docs/INTEGRATION_ANALYSIS.md) | 모듈별 상세 분석, 인터페이스, 통합 이슈 전수 |
| [docs/INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md) | **단일 진실원천(SSOT)** — 토픽/서비스/프레임/좌표 규약 확정본 |
| [docs/DECISIONS.md](docs/DECISIONS.md) | 모든 모호성·불일치 → 확정 결정 + 레포별 수정 액션 |
| [docs/INTEGRATION_STRATEGY.md](docs/INTEGRATION_STRATEGY.md) | 단계별 통합 로드맵 + 목표 워크스페이스 구조 |

## 공통 스택 (전 모듈 합의)

- Ubuntu 22.04 / **ROS 2 Humble**
- **Ignition Gazebo (Fortress)** + `ros_gz`
- 로봇: **UR3 + Robotiq 2F-85**
- LLM: **Google Gemini** `gemini-2.5-flash`
