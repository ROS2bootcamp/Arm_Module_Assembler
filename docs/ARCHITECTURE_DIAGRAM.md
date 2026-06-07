# 시스템 아키텍처 다이어그램

---

## 1. 전체 시스템 구조

```mermaid
flowchart TB
    subgraph USER["👤 사용자"]
        CLI["CLI\n자연어 명령 입력"]
    end

    subgraph CLOUD["☁️ Cloud"]
        GEMINI["Google Gemini\ngemini-2.5-flash"]
    end

    subgraph AGENT_NODE["🤖 llm_agent 노드"]
        direction TB
        AGENT_CORE["AgentNode\n오케스트레이터"]
        PM["PhaseManager\nP1 → P2 → P3 → P4"]
        LLM_CLIENT["LLMClient\nJSON 구조화 출력"]
        YOLO_SUB["YoloSubscriber\n탐지 결과 버퍼"]
        MOVEIT_CLI["MoveItClient\nService 클라이언트"]
        TF_TRANS["TFTransformer\nbase_link ↔ world"]
    end

    subgraph MOVEIT_NODE["⚙️ ur3_moveit_module 노드"]
        direction TB
        MM["MoveItModuleNode\nService 서버"]
        ROUTER["CommandRouter"]
        HANDLERS["Handlers\nscan / pick / lift\nplace / release / home"]
        ARM["ArmController\nMoveItPy"]
        GRIP["GripperController"]
        SCENE["SceneManager\nPlanning Scene"]
    end

    subgraph VISION_NODE["👁️ robot_vision 노드"]
        YOLO["YoloDetector\nYOLOv8n COCO\ncamera→base_link TF 변환"]
    end

    subgraph GAZEBO["🌐 Gazebo 시뮬레이터"]
        UR3["UR3 로봇팔\nRobotiq 2F-85"]
        CAM["고정 RGB-D 카메라"]
        ENV["편의점 선반\n콜라캔 · 디스트랙터"]
    end

    CLI -->|자연어 명령| AGENT_CORE
    AGENT_CORE <--> PM
    AGENT_CORE -->|system_prompt + user_content| LLM_CLIENT
    LLM_CLIENT <-->|HTTPS JSON| GEMINI

    AGENT_CORE --> YOLO_SUB
    AGENT_CORE --> MOVEIT_CLI
    AGENT_CORE --> TF_TRANS

    MOVEIT_CLI -->|"/moveit/execute\nROS2 Service"| MM
    MM --> ROUTER --> HANDLERS
    HANDLERS --> ARM & GRIP & SCENE
    ARM & GRIP -->|MoveIt2 계획·실행| UR3

    CAM -->|"/camera/image\n/camera/depth_image\nROS2 Topic"| YOLO
    YOLO -->|"/vision/detection_results\nROS2 Topic  JSON String"| YOLO_SUB

    AGENT_CORE -->|"/llm_agent/phase\nROS2 Topic"| MON["📊 모니터링"]
    MM -->|"/moveit_status\nROS2 Topic"| MON
```

---

## 2. ROS 2 노드 · 토픽 · 서비스 통신 구조

```mermaid
graph LR
    subgraph NODES["ROS 2 노드"]
        A(["llm_agent"])
        B(["ur3_moveit_module"])
        C(["robot_vision"])
    end

    subgraph TOPICS["Topic  지속 스트림"]
        T1["/vision/detection_results\nstd_msgs/String  JSON"]
        T2["/llm_agent/phase\nstd_msgs/String"]
        T3["/llm_agent/log\nstd_msgs/String"]
        T4["/moveit_status\nstd_msgs/String  JSON"]
        T5["/moveit_module/log\nstd_msgs/String"]
        T6["/camera/image\nsensor_msgs/Image"]
        T7["/camera/depth_image\nsensor_msgs/Image"]
    end

    subgraph SERVICES["Service  단발 요청-응답"]
        S1["/moveit/execute\nllm_agent_msgs/MoveItExecute\ncmd + params_json → success/error"]
    end

    C -->|publish| T1
    T1 -->|subscribe| A

    A -->|publish| T2
    A -->|publish| T3
    B -->|publish| T4
    B -->|publish| T5

    A -->|client| S1
    S1 -->|server| B

    T6 & T7 -->|subscribe| C

    style T1 fill:#d4edda,stroke:#28a745
    style S1 fill:#cce5ff,stroke:#004085
    style T2 fill:#fff3cd,stroke:#856404
    style T4 fill:#fff3cd,stroke:#856404
```

---

## 3. Pick & Place 시퀀스 (P1 → P2 → P3 → P4)

```mermaid
sequenceDiagram
    actor User as 👤 사용자
    participant Agent as AgentNode
    participant LLM as Gemini LLM
    participant MoveIt as MoveItModule
    participant YOLO as YoloDetector

    User->>Agent: "콜라캔 집어줘"

    rect rgb(230, 240, 255)
        Note over Agent,LLM: 명령 파싱 (LLM 호출 ①)
        Agent->>LLM: parse_command<br/>system_prompt + "명령: 콜라캔 집어줘"
        LLM-->>Agent: {"target_class_name": "bottle"}
    end

    rect rgb(255, 245, 230)
        Note over Agent,YOLO: P1 — Scan
        Agent->>MoveIt: scan(waypoints=[...])
        MoveIt-->>Agent: success
        loop YOLO 탐지 대기 (timeout)
            YOLO-->>Agent: /vision/detection_results
        end
        Agent->>Agent: wait_for_detection("bottle", conf≥0.5)
        Note right of Agent: detection{confidence, position_3d_base_frame}
    end

    rect rgb(230, 255, 230)
        Note over Agent,MoveIt: P2 — Pick
        Agent->>Agent: TF base_link→world 변환
        Agent->>Agent: _build_pick_params()<br/>shape/dims from config + world pose
        Agent->>MoveIt: pick(object.pose_world, grasp_params)
        MoveIt-->>Agent: success
        Agent->>Agent: grip 검증<br/>wait_for_detection(max_distance_m)
        Note right of Agent: 그리퍼 근접 거리 이내 감지 확인
    end

    rect rgb(255, 230, 255)
        Note over Agent,LLM: P3 — Pickup 검증 (LLM 호출 ②)
        Agent->>MoveIt: lift(z+, 0.1m)
        MoveIt-->>Agent: success
        YOLO-->>Agent: recent(10 frames)
        Agent->>LLM: verify_pickup<br/>system_prompt + target + YOLO frames JSON
        LLM-->>Agent: {"pickup_success": true, "reason": "..."}
    end

    rect rgb(255, 240, 230)
        Note over Agent,MoveIt: P4 — Place
        Agent->>MoveIt: place(target_pose_world=[0.4, -0.2, 0.08, ...])
        MoveIt-->>Agent: success
        Agent->>MoveIt: release()
        MoveIt-->>Agent: success
        Agent->>MoveIt: home()
        MoveIt-->>Agent: success
    end

    Agent->>User: "세션 완료: Pick & Place 성공"
```

---

## 4. LLM 예외 처리 및 세션 재시도 흐름

```mermaid
flowchart TD
    START(["사용자 명령 수신"]) --> PARSE

    subgraph LLM_GUARD["LLM 방어 로직 (llm_client.py)"]
        PARSE["LLM 호출\nparse_command"]
        RETRY_CHECK{재시도 횟수\n< max_retry?}
        BACKOFF["지수 백오프 대기\n1s → 2s → 4s"]
        SCHEMA_CHECK{"required 키\n모두 존재?"}
        FAIL_LLM(["RuntimeError\n세션 종료"])

        PARSE --> JSON_PARSE{"JSON 파싱\n성공?"}
        JSON_PARSE -->|JSONDecodeError| RETRY_CHECK
        JSON_PARSE -->|성공| SCHEMA_CHECK
        SCHEMA_CHECK -->|누락 키 있음| RETRY_CHECK
        SCHEMA_CHECK -->|정상| TARGET["target 추출 완료"]
        RETRY_CHECK -->|Yes| BACKOFF --> PARSE
        RETRY_CHECK -->|No| FAIL_LLM
    end

    TARGET --> P1

    subgraph SESSION["세션 재시도 로직 (phase_manager.py / agent_node.py)"]
        P1["P1 — Scan"] --> P1_OK{탐지 성공?}
        P1_OK -->|Yes| P2["P2 — Pick"]
        P1_OK -->|No  timeout| RETRY_CNT1{session_retry\n< 3?}
        RETRY_CNT1 -->|Yes| P1
        RETRY_CNT1 -->|No| FAIL_P1(["P1 최종 실패"])

        P2 --> P2_OK{grip 확인?}
        P2_OK -->|Yes| P3["P3 — Pickup 검증\nLLM 호출 ②"]
        P2_OK -->|No| FAIL_P2(["P2 최종 실패"])

        P3 --> P3_OK{pickup_success?}
        P3_OK -->|Yes| P4["P4 — Place"]
        P3_OK -->|No| RETRY_CNT2{session_retry\n< 3?}
        RETRY_CNT2 -->|Yes| RELEASE["release + home"] --> P1
        RETRY_CNT2 -->|No| FAIL_P3(["P3 최종 실패"])

        P4 --> P4_OK{place 성공?}
        P4_OK -->|Yes| DONE(["세션 완료 ✓"])
        P4_OK -->|No| FAIL_P4(["P4 실패\nhome 복귀만 실행\nrelease 생략"])
    end

    subgraph EXCEPT["최상위 예외 격리 (agent_node.py)"]
        EXC["except Exception\n세션 종료 로그"]
        FIN["finally:\nbusy=False\nreset_session()"]
        EXC --> FIN
        FAIL_LLM & FAIL_P1 & FAIL_P2 & FAIL_P3 & FAIL_P4 --> EXC
    end
```

---

## 다이어그램 범례

| 색상 / 기호 | 의미 |
|---|---|
| 초록 배경 (Topic) | 지속 스트림, 구독-발행 |
| 파랑 배경 (Service) | 단발 요청-응답, 블로킹 |
| 노랑 배경 (Topic) | 모니터링용 상태 발행 |
| `LLM 호출 ①` | `parse_command` — 자연어 → target class |
| `LLM 호출 ②` | `verify_pickup` — YOLO 프레임 → 파지 성공 여부 |
