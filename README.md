# 🤖 TetriSpace — AI 기반 자율 창고 정리 시스템

> **Doosan E0509 협동 로봇 + Gemini Vision AI + ArUco Marker 인식**을 결합하여  
> 큐브를 자율적으로 인식하고 지정된 위치에 정렬하는 Pick & Place 자동화 시스템

---

## 📌 프로젝트 개요 (Project Overview)

### 배경 및 목적

현대 물류·제조 현장에서는 다종다양한 물품을 빠르고 정확하게 제자리에 배치하는 **창고 정리 자동화**에 대한 수요가 급증하고 있습니다.  
TetriSpace는 이 문제를 해결하기 위해

- **Gemini Robotics Vision API**로 색상별 큐브의 위치를 자연어 없이 픽셀 → 3D 좌표로 즉시 변환하고
- **ArUco 마커 기반 정밀 포즈 추정**으로 픽/플레이스 타겟 위치를 신뢰성 있게 계산하며
- **Doosan E0509 협동 로봇**이 계획된 경로를 따라 물체를 집어 정확한 위치에 내려놓는

End-to-End 자율 창고 정리 파이프라인을 구현한 ROS 2 기반 프로젝트입니다.

---

## 🛠️ 기술 스택 및 요구 사항 (Tech Stack & Prerequisites)

### 소프트웨어

| 구분 | 기술 |
|------|------|
| **Robot Framework** | ROS 2 Humble |
| **Robot SDK** | Doosan Robotics ROS 2 Package (`doosan-robot2`) |
| **Vision - 마커 인식** | OpenCV (`opencv-contrib-python`) · `cv2.aruco` |
| **Vision - 큐브 감지** | Google Gemini Robotics API (`gemini-robotics-er-1.5-preview`) |
| **카메라 드라이버** | Intel RealSense SDK 2.0 (`pyrealsense2`) |
| **그리퍼 통신** | Modbus RTU over Flange RS-485 (DRL 스크립트 내장) |
| **좌표 변환** | NumPy · Homogeneous Transformation Matrix |
| **인터페이스 정의** | Custom ROS 2 msgs/srvs (`dakae_interfaces`) |
| **언어** | Python 3.10+ |

### 하드웨어

| 장비 | 사양 |
|------|------|
| 협동 로봇 | **Doosan E0509** (Payload 5 kg, Reach 900 mm) |
| 그리퍼 | **RH-P12-RN-A** (ROBOTIS, Modbus RTU, Stroke 0–1000 가변) |
| 카메라 | **Intel RealSense D4xx 시리즈** (RGB 1280×720 @ 30fps, 정렬 Depth) |
| 마커 | ArUco `DICT_4X4_50` (큐브용 0–9, 타겟용 10–19) |
| 통신 | 로봇 Flange RS-485 (그리퍼 직결, 외부 컨버터 불필요) |

### 필수 환경

```
OS          : Ubuntu 22.04 LTS
ROS 2       : Humble Hawksbill
Python      : 3.10 이상
Gemini API  : google-genai (유효한 API Key 필요)
```

---

## 🏗️ 시스템 아키텍처 (Architecture)

```
┌─────────────────────────────────────────────────────────────────┐
│                       TetriSpace System                         │
│                                                                 │
│  ┌─────────────────┐     /camera/color/image_raw               │
│  │  RealSense Cam  │────────────────────────────────┐           │
│  │  (camera_node)  │     /camera/depth/image_raw    │           │
│  └─────────────────┘     /camera/color/camera_info  │           │
│                                                     ▼           │
│  ┌──────────────────────┐      ┌───────────────────────────┐   │
│  │  CubeServiceServer   │◄─────│   MarkerServiceServer     │   │
│  │  (Gemini Vision AI)  │      │   (ArUco Pose Estimation) │   │
│  │  /detect_cubes       │      │   /detect_markers         │   │
│  └──────────┬───────────┘      └──────────┬────────────────┘   │
│             │ 색상+3D좌표                  │ 마커ID+3D좌표+회전  │
│             └──────────────┬──────────────┘                    │
│                            ▼                                    │
│           ┌────────────────────────────┐                        │
│           │      PickPlaceNode         │                        │
│           │  (cube_place_client.py)    │                        │
│           │  DSR_ROBOT2 직접 호출      │                        │
│           └───────────┬────────────────┘                        │
│                       │ gripper_move (stroke 0~1000)            │
│                       ▼                                         │
│           ┌────────────────────────────┐   DRL Script           │
│           │    GripperServerNode       │──────────────►  Robot  │
│           │  (Modbus RTU over RS-485)  │   drl/drl_start        │
│           └────────────────────────────┘                        │
└─────────────────────────────────────────────────────────────────┘
```

### 패키지 구성

| 패키지 | 역할 |
|--------|------|
| `dakae_interfaces` | 커스텀 msg/srv 정의 (CubeInfo, MarkerInfo, TaskInfo, DetectCubes, DetectMarkers, MoveGripper) |
| `dakae_vision` | 카메라 노드 + 큐브/마커 인식 서버 |
| `test_first` | 그리퍼 서버 + 로봇 Pick&Place 클라이언트 |
| `dakae_bringup` | 전체 시스템 통합 런치파일 |
| `doosan-robot2` | Doosan 공식 ROS 2 드라이버 (서브모듈) |
| `RH-P12-RN-A` | ROBOTIS 그리퍼 참조 패키지 (서브모듈) |

### 설계 주안점

- **관심사 분리 (Separation of Concerns)**: 비전·로봇 제어·그리퍼를 독립 노드로 분리하여 각 모듈을 독립 테스트 및 교체 가능하게 설계
- **ROS 2 Service 패턴**: 단발성 명령(감지, 이동, 그리퍼 제어)에 Action 대신 Service를 사용해 응답 완료 보장
- **20-frame 평균 필터**: 마커 인식 시 최신 20개 프레임의 검출 결과를 평균화하여 오검출 및 떨림 노이즈 제거. 7프레임 미만 검출된 마커는 불안정한 것으로 간주하고 폐기
- **ReentrantCallbackGroup + MultiThreadedExecutor**: 그리퍼 서버가 DRL 서비스에 콜백 내부에서 동기 호출 시 발생하는 데드락을 방지
- **RZ 최소 회전 전략**: 180° 대칭 파지가 가능한 그리퍼 특성을 활용해 현재 관절 상태에서 가장 적게 회전하는 파지 각도를 자동 선택

---

## 📋 커스텀 인터페이스 (dakae_interfaces)

### 메시지 타입

```
CubeInfo.msg
  string color                 # 큐브 색상 (e.g. "red", "white")
  geometry_msgs/Vector3 position  # 로봇 베이스 기준 3D 좌표 (m)

MarkerInfo.msg
  string type                  # "cube" 또는 "target"
  int32 id                     # ID (0~9)
  geometry_msgs/Point position # 베이스 기준 3D 좌표 (m)
  float64 rz                   # Z축 회전각 (degree)

TaskInfo.msg  
  int32 cube_id
  int32 target_id
  bool completed
```

### 서비스 타입

```
DetectCubes.srv
  Request : string[] target_colors  # 빈 배열 = 전체 색상 탐색
  Response: CubeInfo[] detected_cubes, bool success, string message

DetectMarkers.srv
  Request : (없음)
  Response: MarkerInfo[] markers, bool success, string message

MoveGripper.srv
  Request : int32 stroke            # 0(완전 열림) ~ 1000(완전 닫힘)
  Response: bool success, string message
```

---

## ⚙️ 설치 및 실행 방법 (Installation & Usage)

### 1. 저장소 클론

```bash
git clone --recurse-submodules https://github.com/MinsikPark7895/TetriSpace.git
cd TetriSpace
```

### 2. Python 의존성 설치

```bash
pip install -r src/dakae_vision/requirements.txt
# pyrealsense2, opencv-contrib-python, numpy, google-genai, python-dotenv
```

### 3. Gemini API Key 설정

`src/dakae_vision/dakae_vision/cube_service_server.py` 내 API Key를 설정합니다:

```python
# cube_service_server.py
self.api_key = "YOUR_GEMINI_API_KEY"
```

> ⚠️ API Key를 코드에 직접 입력하는 방식을 사용하고 있습니다.  
> 배포 시 `.env` 파일과 `python-dotenv`를 활용하는 방식으로 전환을 권장합니다.

### 4. ROS 2 워크스페이스 빌드

```bash
# Doosan ROS 2 환경이 이미 설치되어 있다고 가정
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash
```

### 5. 시스템 실행 (터미널 A — 로봇 드라이버)

```bash
# Doosan 로봇 드라이버 실행 (실제 로봇 연결 시)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<ROBOT_IP> model:=e0509
```

### 6. 카메라 + 비전 서버 실행 (터미널 B)

```bash
source install/setup.bash

# 카메라 노드
ros2 run dakae_vision camera_node

# 별도 터미널에서 마커 인식 서버
ros2 run dakae_vision marker_service_server

# 별도 터미널에서 큐브 감지 서버 (Gemini AI)
ros2 run dakae_vision cube_service_server
```

### 7. 그리퍼 서버 실행 (터미널 C)

```bash
source install/setup.bash
ros2 run test_first gripper_server
# 그리퍼 초기화(영점)까지 약 5초 소요
```

### 8. Pick & Place 태스크 실행 (터미널 D)

```bash
source install/setup.bash

# ArUco 마커 기반 Pick & Place (권장)
ros2 run test_first cube_place

# Gemini Vision 기반 Pick & Place (색상 인식)
ros2 run test_first move_line
```

### 통합 런치 (선택)

```bash
ros2 launch dakae_bringup system.launch.py use_vision:=true use_gripper:=true
```

---

## 🔍 주요 기능 상세 (Key Features)

### 1. Gemini Robotics Vision — 큐브 색상 인식

`CubeServiceServer`는 RealSense의 RGB 프레임을 JPEG로 인코딩하여 `gemini-robotics-er-1.5-preview` 모델에 전달합니다.  
모델은 큐브 상단면 중심 좌표를 0–1000 정규화 좌표(JSON)로 반환하며, 이를 Depth 이미지와 카메라 내부 파라미터(fx, fy, cx, cy)를 활용해 3D 카메라 좌표로 역투영합니다.  
최종적으로 사전에 측정된 **카메라→로봇 베이스 변환 행렬(T_base_cam)** 을 곱하여 로봇이 직접 사용 가능한 베이스 좌표를 출력합니다.

```
픽셀 (u, v) + Depth z
  → 카메라 좌표 [X_c, Y_c, Z_c]
    → 베이스 좌표 [X, Y, Z] = T_base_cam · [X_c, Y_c, Z_c, 1]ᵀ
```

### 2. ArUco 마커 — 포즈 추정 및 안정화

`MarkerServiceServer`는 최신 20개 프레임을 순환 버퍼(deque)로 유지하며, 서비스 요청 시 일괄 처리합니다.  
동일 마커가 **7회 이상** 연속 검출된 경우에만 유효한 데이터로 인정하고 평균 포즈를 계산합니다.  
이 방식으로 단일 프레임의 오검출 및 카메라 진동에 의한 좌표 불안정을 원천 차단합니다.

큐브 마커(ID 0–9)의 RZ는 **4방향 대칭** 특성(90° 단위)을 고려하여 ±45° 범위로 정규화합니다.

### 3. Modbus RTU 그리퍼 제어

그리퍼(`RH-P12-RN-A`)는 Doosan 로봇 Flange의 RS-485 포트에 직결되어 있어  
외부 USB 컨버터 없이 DRL(Doosan Robot Language) 스크립트를 통해 Modbus RTU 패킷을 직접 전송합니다.

```
GripperServerNode (ROS 2 Service Server)
  ↓ MoveGripper.srv (stroke: int32)
drl/drl_start 서비스 호출 (Doosan 드라이버)
  ↓ DRL Python 스크립트 실행
flange_serial_open() → FC16 패킷 4회 전송 → flange_serial_close()
```

스트로크 값 `0`은 완전 열림, `1000`은 완전 닫힘이며 중간 값도 자유롭게 지정 가능합니다.  
DRL 제출이 비동기적으로 완료되는 특성상, 실제 물리 동작 완료까지 **4.5초 타임아웃**을 적용하여 동기화합니다.

### 4. Pick & Place 시퀀스

```
[1] 마커 검출 요청 → 큐브/타겟 위치·회전각 수신
[2] 홈 위치 이동 (movej)
[3] 각 작업 쌍(cube_id → target_id)에 대해:
    [3-1] pick_cycle():
          - 그리퍼 열기 (stroke=0)
          - 접근 위치(현재 자세 유지) → rz 정렬 위치 → 픽 위치 (movel)
          - 그리퍼 닫기 (stroke=500)
          - 접근 위치 복귀
    [3-2] place_cycle():
          - 접근 위치 → rz 정렬 → 플레이스 위치
          - 그리퍼 열기 (stroke=0)
          - 접근 위치 복귀
    [3-3] 홈 복귀
```

---

## 🐛 트러블슈팅 및 기술적 도전 (Troubleshooting & Achievements)

### ① 그리퍼 서버 데드락 문제

**문제**: `GripperServerNode`가 서비스 콜백 내부에서 `drl/drl_start` 서비스를 동기 호출하면, ROS 2 기본 싱글스레드 Executor가 콜백과 응답 수신을 동시에 처리하지 못해 **무한 대기(데드락)** 에 빠졌습니다.

**해결**: `ReentrantCallbackGroup` + `MultiThreadedExecutor`를 도입하고, 초기화 로직을 메인 스레드에서 별도 실행하도록 분리했습니다. Spin 엔진을 백그라운드 스레드에서 먼저 가동한 뒤 초기화를 호출하는 순서로 해결했습니다.

### ② DRL 비동기 타이밍 문제

**문제**: `drl/drl_start` 서비스는 스크립트 **제출 즉시** 응답을 반환하며, 실제 물리 동작은 비동기로 진행됩니다. 이로 인해 그리퍼가 동작 완료 전에 다음 로봇 명령이 실행되는 타이밍 오류가 발생했습니다.

**해결**: DRL 스크립트 내 `wait()` 명령 합계(≈4.1초)를 계산하고 안전 마진을 포함한 **4.5초 sleep**을 서버에서 강제 대기하도록 구현했습니다. 추후 Modbus Read로 현재 위치를 폴링하는 방식으로 교체 예정입니다.

### ③ Modbus RTU 패킷 유실

**문제**: RS-485 통신에서 단일 패킷 전송 시 그리퍼가 반응하지 않는 경우가 발생했습니다 (전기적 노이즈, 타이밍 이슈).

**해결**: FC16 패킷을 **4회 반복 전송** (`wait(0.1)` 간격)하고 전 후에 적절한 대기 시간을 삽입하여 통신 신뢰성을 확보했습니다.

### ④ 마커 인식 좌표 불안정

**문제**: 단일 프레임 기반 ArUco 포즈 추정은 카메라 진동·조도 변화에 민감하여 좌표값이 프레임마다 수mm씩 변동했습니다.

**해결**: 순환 버퍼(deque maxlen=20)로 최근 20프레임을 누적하고, 동일 마커가 **7회 이상** 검출된 경우에만 유효 데이터로 인정하여 평균 산출합니다. 이 방식으로 좌표 안정성을 크게 향상시켰습니다.

### ⑤ RZ 각도 모호성 문제

**문제**: 사각형 큐브의 4면은 구조적으로 동일하여 ArUco 마커의 RZ가 0°, 90°, 180°, 270° 중 어느 것이든 동등한 파지 자세를 의미합니다. 마커 추정 결과에 따라 로봇이 불필요하게 270° 회전하는 경우가 발생했습니다.

**해결**: 마커의 RZ를 **±45°** 범위로 정규화하고(`((rz + 45) % 90 - 45)`), 실제 파지 시에는 현재 관절 각도에서 가장 가까운 등가 각도(0°, ±180°)를 선택하는 `choose_shortest_grasp_rz()` 함수를 구현했습니다.

---

## 👥 팀원 역할 및 협업 (Team & Contribution)

| 이름 | 역할 | 주요 기여 |
|------|------|-----------|
| **박민식 (Minsik Park)** | 팀장 / 그리퍼 통신  | (Modbus RTU DRL), ROS 2 아키텍처 설계 |
| **백승호** | 비전 시스템, 인터페이스 설계 | (Gemini API) (ArUco) (RealSense) |
| **김제원** | 로봇 제어 / Pick&Place | E0509
| **모윤근** | 통합 런치 / 시스템 통합 | (msg/srv), (launch files) |

---

## 📁 디렉토리 구조

```
TetriSpace/
├── src/
│   ├── dakae_interfaces/          # 커스텀 메시지/서비스 정의
│   │   ├── msg/
│   │   │   ├── CubeInfo.msg
│   │   │   ├── MarkerInfo.msg
│   │   │   └── TaskInfo.msg
│   │   └── srv/
│   │       ├── DetectCubes.srv
│   │       ├── DetectMarkers.srv
│   │       ├── ExecuteTasks.srv
│   │       └── MoveGripper.srv
│   ├── dakae_vision/              # 비전 패키지
│   │   └── dakae_vision/
│   │       ├── camera_node.py             # RealSense 카메라 드라이버
│   │       ├── cube_service_server.py     # Gemini AI 큐브 색상 인식
│   │       ├── marker_service_server.py   # ArUco 마커 포즈 추정
│   │       └── marker_service_server_offset.py  # 오프셋 보정 버전
│   ├── test_first/                # 로봇 제어 패키지
│   │   └── test_first/
│   │       ├── cube_place_client.py   # ArUco 기반 Pick & Place
│   │       ├── move_line_client.py    # Vision 기반 Pick & Place
│   │       ├── gripper_server.py      # Modbus RTU 그리퍼 서버
│   │       ├── arm_gripper_test_client.py
│   │       ├── home_client.py
│   │       └── test_color.py
│   ├── dakae_bringup/             # 통합 런치 패키지
│   │   └── launch/
│   │       ├── system.launch.py       # 전체 시스템 통합 런치
│   │       └── vision_only.launch.py  # 비전 단독 테스트
│   ├── doosan-robot2/             # Doosan 공식 ROS 2 드라이버 (서브모듈)
│   └── RH-P12-RN-A/               # ROBOTIS 그리퍼 참조 패키지 (서브모듈)
├── build/
├── install/
└── log/
```

---

## 📄 라이선스

Apache License 2.0

---
