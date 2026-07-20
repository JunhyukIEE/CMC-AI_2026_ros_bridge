# UDP Camera Receiver for ROS2

MORAI 시뮬레이터에서 UDP로 전송되는 카메라 이미지를 ROS2 토픽으로 변환하는 패키지입니다.

## 기능

- UDP 패킷을 수신하여 ROS2 Image 메시지로 변환
- 여러 대의 카메라 동시 지원
- YAML 파일을 통한 간편한 설정
- 패킷 재조합 및 프레임 복원
- 디버그 모드 지원

## 패키지 구조

```
udp_camera_receiver/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── udp_camera_receiver/
│       └── udp_camera_receiver.hpp
├── src/
│   ├── udp_camera_receiver.cpp
│   └── udp_camera_receiver_node.cpp
├── config/
│   └── param.yaml
└── launch/
    └── udp_camera_receiver.launch.py
```

## 빌드

```bash
cd ~/ros_ws
colcon build --packages-select udp_camera_receiver
source install/setup.bash
```

## 실행

### 기본 실행
```bash
ros2 launch udp_camera_receiver udp_camera_receiver.launch.py
```

### 디버그 모드로 실행
```bash
ros2 launch udp_camera_receiver udp_camera_receiver.launch.py
```
(config/param.yaml에서 `debug_mode: true`로 설정)

## 설정

`config/param.yaml` 파일을 수정하여 카메라 설정을 변경할 수 있습니다.

### 카메라 설정 예시

```yaml
/**:
  ros__parameters:
    # 카메라 개수
    num_cameras: 2

    # 카메라 0 설정
    camera_0:
      name: "front_camera"
      ip: "192.168.0.37"
      port: 9090
      topic_name: "/camera/front/image_raw"
      width: 640
      height: 480
      channels: 3

    # 카메라 1 설정
    camera_1:
      name: "rear_camera"
      ip: "192.168.0.37"
      port: 9091
      topic_name: "/camera/rear/image_raw"
      width: 640
      height: 480
      channels: 3
```

### 파라미터 설명

- `debug_mode`: 디버그 메시지 출력 여부
- `max_buffered_frames`: 최대 버퍼링할 프레임 수
- `frame_timeout_sec`: 프레임 타임아웃 시간 (초)
- `num_cameras`: 카메라 개수
- `camera_X.name`: 카메라 이름
- `camera_X.ip`: UDP 수신 IP 주소
- `camera_X.port`: UDP 수신 포트
- `camera_X.topic_name`: ROS2 토픽 이름
- `camera_X.width`: 이미지 너비
- `camera_X.height`: 이미지 높이
- `camera_X.channels`: 채널 수 (3 for RGB)

## 토픽

### Published Topics

- `/camera/front/image_raw` (sensor_msgs/Image): 카메라 이미지
  - 설정 파일의 `topic_name`에 따라 달라집니다.

## UDP 패킷 구조

MORAI 시뮬레이터에서 전송하는 UDP 패킷 구조:

```cpp
struct PacketHeader {
    uint32_t magic_number;      // 매직 넘버
    uint32_t frame_number;      // 프레임 번호
    uint32_t packet_number;     // 현재 패킷 번호
    uint32_t total_packets;     // 총 패킷 수
    uint32_t width;             // 이미지 너비
    uint32_t height;            // 이미지 높이
    uint32_t channels;          // 채널 수
    uint64_t timestamp;         // 타임스탬프
};
```

## 트러블슈팅

### 1. "Address already in use" 에러

포트가 이미 사용 중입니다. 다른 프로그램이 해당 포트를 사용하고 있는지 확인하세요.

```bash
sudo netstat -tulpn | grep :9090
```

### 2. 이미지가 수신되지 않음

- MORAI 시뮬레이터가 올바른 IP와 포트로 데이터를 전송하는지 확인
- tcpdump로 UDP 패킷 수신 확인:
  ```bash
  sudo tcpdump -i lo udp port 9090 -c 10
  ```
- config/param.yaml의 IP 주소와 포트 번호 확인

### 3. 이미지가 깨짐

- 패킷 헤더 구조가 예상과 다를 수 있습니다.
- `debug_mode: true`로 설정하여 패킷 정보 확인
- 해상도 설정 확인 (width, height)

## 이미지 확인

### rqt_image_view 사용
```bash
ros2 run rqt_image_view rqt_image_view
```

### rviz2 사용
```bash
rviz2
```
Add → By topic → /camera/front/image_raw → Image

### topic echo
```bash
ros2 topic echo /camera/front/image_raw
ros2 topic hz /camera/front/image_raw
```

## 라이선스

MIT
