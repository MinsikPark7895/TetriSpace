# dev-ControlUI → ex-grip_vision 변경사항

## 1. 비전 시스템 교체 (YOLO8)

**기존 (`dev-ControlUI`)**
- ArUco 마커 기반 위치 인식 (`marker_service_server.py`)
- 색상 기반 큐브 인식 (`cube_service_server.py`)
- 서비스: `/detect_markers`, `/detect_cubes`

**변경 (`ex-grip_vision`)**
- YOLO8 딥러닝 기반 객체 인식 (`object_contour_service_server.py`)
- RealSense depth + 윤곽선으로 3D 좌표 계산
- 서비스: `/detect_objects`
- 인식 가능 라벨: `toy`, `bottle`, `box`, `cube`
- 학습 모델 추가: `dakae_v2_best.pt`, `best.pt`

**추가된 인터페이스**
```
dakae_interfaces/msg/ObjectInfo.msg      ← 물체 위치/각도/신뢰도
dakae_interfaces/srv/DetectObjects.srv   ← YOLO8 탐지 서비스
```

---

## 2. 그리퍼 TCP 패키지 추가 (`dsr_gripper_tcp`)

기존에는 `test_first/gripper_server.py`(단순 서비스)만 있었으나, TCP 직접 통신 기반의 완전한 그리퍼 제어 패키지가 추가됨.

| 파일 | 역할 |
|---|---|
| `gripper_tcp_bridge.py` | TCP 프로토콜 기반 그리퍼 직접 제어 |
| `gripper_service_node.py` | ROS2 서비스/액션 서버 |
| `web_dashboard.py` | 웹 UI (http://localhost:5000) |
| `web_dashboard_node.py` | 웹 대시보드 ROS2 노드 |

**추가된 인터페이스 (`dsr_gripper_tcp_interfaces`)**
```
action/SafeGrasp.action          ← 안전 파지 액션
msg/GripperState.msg             ← 그리퍼 상태
srv/SetPosition.srv              ← 위치 제어
srv/SetTorque.srv                ← 토크 제어
srv/GetPosition.srv              ← 위치 조회
srv/GetState.srv                 ← 상태 조회
srv/SetMotionProfile.srv         ← 모션 프로파일 설정
srv/GetMotionProfile.srv         ← 모션 프로파일 조회
```

---

## 3. 픽앤플레이스 액션 패키지 추가 (`dakae_action`)

마커 기반 픽앤플레이스 자동화 액션 서버/클라이언트 추가.

```
pick_place_action_server.py       ← 액션 서버 (DSR 로봇팔 제어)
pick_place_action_client.py       ← 단일 실행 클라이언트
pick_place_action_client_loop.py  ← 반복 실행 클라이언트
```

---

## 4. YOLO8 + 로봇팔 연동 테스트 스크립트 추가

**`test_first/vision_arm_test.py`**

YOLO8 탐지 결과를 로봇팔 모션으로 직접 연결하는 테스트 스크립트.

동작 순서:
```
물체 인식 → XY 이동 → 그리퍼 열기 → Z 하강
→ 그리퍼 닫기 → Z 상승 → 플레이스 이동
→ Z 하강 → 그리퍼 열기 → Z 상승 → 홈
```

실행:
```bash
ros2 run test_first vision_arm_test
```

---

## 5. 기타

- Gemini VLA 서브모듈 제거 (`src/gemini_vla_Doosan-robotic-arm`)
- Dockerfile, GitHub Actions CI 추가

---

## 실행 명령어 요약

```bash
# 1. 로봇팔 드라이버
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.56 model:=e0509

# 2. YOLO8 비전
ros2 run dakae_vision object_contour_service_server

# 3. 그리퍼 서버
ros2 run test_first gripper_server

# 4. 픽앤플레이스 테스트
ros2 run test_first vision_arm_test
```
