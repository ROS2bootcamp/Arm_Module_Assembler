# ROBOT_VISION
- 해당 패키지는 Gazebo 시뮬레이션 환경에서 YOLOv8과 RGB-D 카메라를 이용해 객체를 인식하고, 3D 좌표를 계산해 퍼블리시하는 ROS 2 노드임.
- 비전 파트와 로봇 제어(MoveIt 2) 파트를 완전히 분리하기 위해, 인식된 객체의 위치 데이터를 JSON 형태로 쏴주는 역할만 담당함.

## 개발 환경 및 의존성
* OS / ROS 버전: Ubuntu 22.04.5 / ROS 2 Humble
* 시뮬레이터: Gazebo Ignition

### Python 라이브러리 설치
YOLOv8 모델 구동을 위해 다음 패키지들이 필요함
```bash
pip install -r requirements.txt
```
### ROS2 패키지 설치
cv_bridge 및 tf2 연산에 필요한 패키지를 설치함
```bash
sudo apt-get install ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```
## 빌드 방법
본인의 ROS2 워크스페이스 src 폴더 내에 클론 후에 빌드

```bash
cd ~/your_workspace/src
git clone https://github.com/ROS2bootcamp/ROBOT_VISION.git
cd ~/your_workspace
colcon build --packages-select robot_vision
source install/setup.bash
```

## 실행 방법
단일 쉘 스크립트 `run.sh` 를 통해 빌드->환경셋업->시뮬레이터 실행->브릿지 연결->비전노드 구동이 한번에 진행됨

```bash
cd robot_vision
./run.sh
```
> [!NOTE]
> 주의: 실행 후 Gazebo 시뮬레이터 창이 뜨면 하단의 재생 버튼을 눌러야 시뮬레이션 시간이 흐르고 센서 데이터가 전송됨

## 데이터 포맷
해당 노드는 `/vision/detection_results` 토픽으로 데이터를 퍼블리시함.
제어 스크립트에서 이 토픽(메시지 타입: `std_msgs/msg/String`)을 Subscribe 한 뒤, `json.loads(msg.data)`로 파싱해서 사용하면 됨.
```JSON
{
  "timestamp_ns": 1718001234567890,
  "num_detections": 1,
  "objects": [
    {
      "class_name": "stop sign",
      "confidence": 0.892,
      "center_2d": {"u": 320, "v": 240},
      "distance_m": 3.0,
      "position_3d_camera_frame": {
        "X": 0.0,
        "Y": 0.0,
        "Z": 3.0
      },
      "position_3d_base_frame": {
        "X": 0.0,
        "Y": 3.0,
        "Z": 0.5
      }
    }
  ]
}
```

> [!NOTE]
> 참고사항: 로봇 팔 조작을 위한 타겟 좌표는 `position_3d_base_frame` 값을 사용하면 됨. (TF2 변환이 적용된 절대 좌표값임).

