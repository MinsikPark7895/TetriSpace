# dsr_gripper_tcp

Doosan **RH-P12-RN(A)** 그리퍼 제어를 위한 독립 ROS2 패키지입니다.
DRL(Doosan Robot Language) 측 TCP 서버 + 호스트 측 Python 브리지 +
CLI 예제 + Flask/SocketIO 웹 대시보드(스레드 버전 / ROS2 노드 버전)를
한 패키지에 묶어 제공합니다.

원본은 `doosan-robot2/dsr_example2/dsr_example/dsr_example/simple/` 아래에
있고, 이 패키지는 그것을 독립 패키지로 분리한 것입니다.

## 구성

```
dsr_gripper_tcp/
├── dsr_gripper_tcp/
│   ├── gripper_tcp_protocol.py   # 패킷/상태 직렬화 + GripperState dataclass
│   ├── gripper_tcp_bridge.py     # DRL 서버 스크립트 + 호스트 측 TCP 브리지
│   ├── example_gripper_tcp.py    # CLI 예제 (open/close 동작)
│   ├── web_dashboard.py          # 원본 Flask/SocketIO 대시보드 (스레드 기반)
│   └── web_dashboard_node.py     # 동일 UI를 ROS2 Node로 감싼 버전
├── launch/
│   └── web_dashboard_node.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

## 의존성

- ROS2 (humble 이상 권장) + `rclpy`, `std_msgs`, `sensor_msgs`
- `dsr_msgs2` (doosan-robot2의 DRL/SetRobotMode 서비스)
- Python: `flask`, `flask-socketio`

```bash
pip install flask flask-socketio
```

## 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select dsr_gripper_tcp
source install/setup.bash
```

## 실행

### 1. CLI 예제 (open ↔ close 1회)

```bash
ros2 run dsr_gripper_tcp example_gripper_tcp \
  --controller-host 192.168.137.100 \
  --namespace dsr01 \
  --service-prefix dsr_controller2
```

### 2. 웹 대시보드 - 스레드 버전 (원본과 동일)

```bash
ros2 run dsr_gripper_tcp web_dashboard
# → http://localhost:5000
```

> 주의: 이 버전은 `controller_host`, `namespace`, `service_prefix`가
> 코드에 하드코딩되어 있습니다. 환경에 맞게 수정해서 사용하거나,
> 아래의 ROS2 노드 버전을 권장합니다.

### 3. 웹 대시보드 - ROS2 노드 버전 (권장)

```bash
ros2 launch dsr_gripper_tcp web_dashboard_node.launch.py \
  controller_host:=192.168.137.100 \
  namespace:=dsr01 \
  service_prefix:=dsr_controller2 \
  web_port:=5000
```

또는 `ros2 run`으로 직접:

```bash
ros2 run dsr_gripper_tcp web_dashboard_node \
  --ros-args \
  -p controller_host:=192.168.137.100 \
  -p namespace:=dsr01 \
  -p service_prefix:=dsr_controller2
```

## ROS2 노드 인터페이스 (`web_dashboard_node`)

기본 노드 이름: `gripper_web_dashboard`

### 파라미터

| 이름 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `controller_host` | string | `110.120.1.56` | 컨트롤러 IP |
| `tcp_port` | int | `20002` | DRL TCP 서버 포트 |
| `namespace` | string | `dsr01` | 로봇 네임스페이스 |
| `service_prefix` | string | `""` | DRL 서비스 prefix (예: `dsr_controller2`) |
| `skip_set_autonomous` | bool | `false` | autonomous 모드 설정 스킵 |
| `initialize_on_start` | bool | `true` | 시작 시 그리퍼 INITIALIZE 호출 |
| `goal_current` | int | `400` | 초기 grip 전류(mA) |
| `profile_velocity` | int | `1500` | 모션 프로파일 속도 |
| `profile_acceleration` | int | `1000` | 모션 프로파일 가속도 |
| `web_host` | string | `0.0.0.0` | Flask 바인딩 호스트 |
| `web_port` | int | `5000` | Flask 포트 |
| `poll_rate_hz` | double | `20.0` | TCP 상태 폴링 주기 |
| `joint_name` | string | `rh_p12_rn` | JointState `name[0]` |
| `move_timeout_sec` | double | `5.0` | move_to 타임아웃 |
| `connect_timeout_sec` | double | `20.0` | DRL TCP 서버 접속 재시도 타임아웃 |
| `post_drl_start_sleep_sec` | double | `0.5` | DrlStart 직후 TCP 접속 시도 전 대기 |
| `stop_existing_drl` | bool | `true` | 시작 시 기존 DRL이 실행 중이면 정지 후 재시작 |
| `drl_stop_mode` | int | `1` | DrlStop 모드 (0=QUICK_STO, 1=QUICK, 2=SLOW, 3=HOLD) |
| `drl_stop_settle_sec` | double | `5.0` | DrlStop 후 IDLE 전이까지 폴링 타임아웃 |
| `drl_start_retry_count` | int | `3` | DrlStart 실패 시 재시도 횟수 |
| `drl_start_retry_delay_sec` | double | `1.0` | DrlStart 재시도 사이 대기 |
| `init_attempts` | int | `5` | 부팅 시 INITIALIZE 명령 재시도 횟수 |
| `init_timeout_sec` | double | `30.0` | INITIALIZE 응답 대기 타임아웃 |
| `init_retry_delay_sec` | double | `1.0` | INITIALIZE 재시도 사이 대기 |

### Topics

**Publishers**

- `~/joint_state` (`sensor_msgs/JointState`) - 1축 그리퍼 상태.
  `position`은 `[0,1]`로 정규화 (`present_position / 1150`),
  `velocity`는 raw count, `effort`는 present_current.
- `~/raw_state` (`std_msgs/Float32MultiArray`) - 7개 값
  `[pos, current, temperature, velocity, moving, moving_status, torque_enabled]`

**Subscribers**

- `~/goal_position` (`std_msgs/Int32`) - 목표 위치 (0..1150 pulse)
- `~/torque_enable` (`std_msgs/Bool`) - 토크 ON/OFF
- `~/motion_profile` (`std_msgs/Float32MultiArray`) -
  `[goal_current, profile_velocity, profile_acceleration]`
- `~/emergency_stop` (`std_msgs/Bool`) - `true`면 현재 위치 유지

### 사용 예 (다른 터미널에서)

```bash
# 토크 ON
ros2 topic pub --once /gripper_web_dashboard/torque_enable std_msgs/msg/Bool "{data: true}"

# 위치 700으로 이동 (close)
ros2 topic pub --once /gripper_web_dashboard/goal_position std_msgs/msg/Int32 "{data: 700}"

# 프로파일 변경
ros2 topic pub --once /gripper_web_dashboard/motion_profile \
  std_msgs/msg/Float32MultiArray "{data: [400, 1500, 1000]}"

# 상태 확인
ros2 topic echo /gripper_web_dashboard/joint_state
```

### 4. 그리퍼 서비스/액션 서버 노드

다른 로봇팔 제어 노드가 그리퍼를 제어할 때는 이 노드를 사용하는 것을
권장합니다. 이 노드가 `DoosanGripperTcpBridge`를 단일 소유하고, 외부에는
service/action/topic만 노출합니다.

```bash
ros2 launch dsr_gripper_tcp gripper_service_node.launch.py \
  controller_host:=192.168.137.100 \
  namespace:=dsr01 \
  service_prefix:=dsr_controller2
```

제공 인터페이스:

- Topic: `/gripper_service/state` (`dsr_gripper_tcp_interfaces/msg/GripperState`)
- Topic: `/gripper_service/joint_state` (`sensor_msgs/msg/JointState`)
- Service: `/gripper_service/set_position`
- Service: `/gripper_service/get_position`
- Service: `/gripper_service/set_motion_profile`
- Service: `/gripper_service/get_motion_profile`
- Service: `/gripper_service/set_torque`
- Service: `/gripper_service/get_state`
- Action: `/gripper_service/safe_grasp`

#### Topic: `/gripper_service/state`

타입: `dsr_gripper_tcp_interfaces/msg/GripperState`

그리퍼의 최신 상태를 주기적으로 publish합니다. 로봇팔 제어 action server는
이 topic을 subscribe해서 잡힘/놓침/전류 상태를 계속 감시하면 됩니다.

주요 필드:

- `ready`: 그리퍼가 제어 가능한 상태인지
- `torque_enabled`: 토크 ON/OFF
- `moving`: 현재 이동 중인지
- `in_position`: 그리퍼 펌웨어가 목표 위치 도달로 보고 있는지
- `grasp_detected`: 전류 기준으로 잡힘이 감지됐는지
- `object_lost`: 한 번 잡은 뒤 전류/위치 변화로 놓침이 의심되는지
- `present_position`: 현재 위치 pulse
- `goal_position`: 마지막 목표 위치
- `present_current`: 현재 전류
- `current_limit`: 현재 goal current
- `present_velocity`: 현재 속도
- `present_temperature`: 그리퍼 온도
- `status_text`: 노드 내부 상태 메시지

예시:

```bash
ros2 topic echo /gripper_service/state
```

#### Topic: `/gripper_service/joint_state`

타입: `sensor_msgs/msg/JointState`

RViz나 다른 ROS 도구에서 보기 쉬운 1축 joint 상태입니다.
`position`은 `present_position / position_max`로 0.0~1.0 범위에 가깝게
정규화됩니다. `effort`는 현재 전류입니다.

예시:

```bash
ros2 topic echo /gripper_service/joint_state
```

#### Service: `/gripper_service/set_torque`

타입: `dsr_gripper_tcp_interfaces/srv/SetTorque`

그리퍼 토크를 켜거나 끕니다.

요청:

- `enabled`: `true`면 토크 ON, `false`면 토크 OFF

예시:

```bash
# 토크 ON
ros2 service call /gripper_service/set_torque \
  dsr_gripper_tcp_interfaces/srv/SetTorque "{enabled: true}"
```

토크 OFF:

```bash
ros2 service call /gripper_service/set_torque \
  dsr_gripper_tcp_interfaces/srv/SetTorque "{enabled: false}"
```

#### Service: `/gripper_service/set_position`

타입: `dsr_gripper_tcp_interfaces/srv/SetPosition`

그리퍼를 특정 위치로 이동합니다. 내부적으로 `bridge.move_to()`를 호출하므로
요청은 완료될 때까지 대기합니다.

요청:

- `position`: 목표 위치 pulse (`0` = open, `700` 정도 = 일반 close 예시)
- `timeout_sec`: 이동 완료 대기 시간. `0` 이하이면 노드 기본값 사용

예시:

```bash
# 열기
ros2 service call /gripper_service/set_position \
  dsr_gripper_tcp_interfaces/srv/SetPosition "{position: 0, timeout_sec: 5.0}"

# 닫기
ros2 service call /gripper_service/set_position \
  dsr_gripper_tcp_interfaces/srv/SetPosition "{position: 700, timeout_sec: 5.0}"
```

#### Service: `/gripper_service/get_position`

타입: `dsr_gripper_tcp_interfaces/srv/GetPosition`

위치 중심 상태만 간단히 조회합니다.

요청:

- `force_read`: `true`면 TCP로 실제 상태를 즉시 읽고, `false`면 캐시 상태 사용

예시:

```bash
ros2 service call /gripper_service/get_position \
  dsr_gripper_tcp_interfaces/srv/GetPosition "{force_read: true}"
```

#### Service: `/gripper_service/set_motion_profile`

타입: `dsr_gripper_tcp_interfaces/srv/SetMotionProfile`

그리퍼의 힘/속도/가속도 프로파일을 설정합니다.

요청:

- `goal_current`: 목표 전류. 잡는 힘/전류 제한에 해당
- `profile_velocity`: 이동 속도
- `profile_acceleration`: 이동 가속도

예시:

```bash
ros2 service call /gripper_service/set_motion_profile \
  dsr_gripper_tcp_interfaces/srv/SetMotionProfile \
  "{goal_current: 400, profile_velocity: 1500, profile_acceleration: 1000}"
```

#### Service: `/gripper_service/get_motion_profile`

타입: `dsr_gripper_tcp_interfaces/srv/GetMotionProfile`

현재 노드가 기억하고 있는 motion profile 값을 반환합니다. 이 값은 그리퍼
레지스터를 직접 다시 읽은 값이 아니라, 마지막으로 설정한 캐시 값입니다.

예시:

```bash
ros2 service call /gripper_service/get_motion_profile \
  dsr_gripper_tcp_interfaces/srv/GetMotionProfile "{}"
```

#### Service: `/gripper_service/get_state`

타입: `dsr_gripper_tcp_interfaces/srv/GetState`

전체 `GripperState`를 조회합니다.

요청:

- `force_read`: `true`면 TCP로 실제 상태를 즉시 읽고, `false`면 캐시 상태 사용

예시:

```bash
ros2 service call /gripper_service/get_state \
  dsr_gripper_tcp_interfaces/srv/GetState "{force_read: true}"
```

#### Action: `/gripper_service/safe_grasp`

타입: `dsr_gripper_tcp_interfaces/action/SafeGrasp`

안전하게 잡기 동작입니다. step 이동을 하지 않고 `target_position`까지 한 번에
닫는 모션을 수행합니다. 이후 최종 전류 또는 시작 전류 대비 증가량으로 잡힘을
판단합니다.

Goal:

- `target_position`: 닫을 목표 위치
- `max_current`: 이 전류 이상이면 잡았다고 판단
- `current_delta_threshold`: 시작 전류 대비 이 값 이상 증가하면 잡았다고 판단
- `timeout_sec`: 전체 이동/판정 제한 시간

Feedback:

- `present_position`: 현재 위치
- `present_current`: 현재 전류
- `current_delta`: 시작 전류 대비 증가량
- `grasp_detected`: 잡힘 감지 여부
- `object_lost`: 놓침 의심 여부
- `state`: 전체 그리퍼 상태

Result:

- `success`: 잡힘 감지 성공 여부
- `message`: 결과 메시지
- `final_position`: 최종 위치
- `final_current`: 최종 전류
- `grasp_detected`: 잡힘 감지 여부
- `object_lost`: 놓침 의심 여부
- `state`: 전체 그리퍼 상태

예시:

```bash
ros2 action send_goal /gripper_service/safe_grasp \
  dsr_gripper_tcp_interfaces/action/SafeGrasp \
  "{target_position: 700, max_current: 400, current_delta_threshold: 120, timeout_sec: 8.0}" \
  --feedback
```

더 약하게 잡기:

```bash
ros2 action send_goal /gripper_service/safe_grasp \
  dsr_gripper_tcp_interfaces/action/SafeGrasp \
  "{target_position: 700, max_current: 300, current_delta_threshold: 80, timeout_sec: 8.0}" \
  --feedback
```

더 강하게 잡기:

```bash
ros2 action send_goal /gripper_service/safe_grasp \
  dsr_gripper_tcp_interfaces/action/SafeGrasp \
  "{target_position: 900, max_current: 500, current_delta_threshold: 150, timeout_sec: 10.0}" \
  --feedback
```

로봇팔 제어 Action Server는 `/gripper_service/state`를 subscribe해 최신
잡힘/놓침 상태를 캐시하고, 잡기 단계에서는 `/gripper_service/safe_grasp`
action client를 호출하면 됩니다. `safe_grasp`는 step 이동을 하지 않고
`target_position`까지 한 번에 이동합니다. DRL 내부의 move 완료 조건과
최종 전류(`max_current` 또는 `current_delta_threshold`)로 잡힘을 판단합니다.

## 두 대시보드 버전 비교

| 항목 | `web_dashboard` (스레드) | `web_dashboard_node` (ROS2) |
|---|---|---|
| 설정 | 코드 하드코딩 | ROS 파라미터 |
| 폴링 | `threading.Thread` | `rclpy.Timer` (executor) |
| ROS 토픽 | 없음 | JointState, raw_state pub + cmd sub |
| 다른 ROS 노드와 통합 | 불편 | `ros2 launch`로 자연스럽게 통합 |
| 기능/UI | 동일 | 동일 |

웹 UI 자체(HTML/JS)는 **완전히 동일**합니다. ROS2 노드 버전은
원본의 `HTML_TEMPLATE`을 그대로 import해서 재사용합니다.

## 트러블슈팅

### `Failed to connect to controller TCP server: [Errno 111] Connection refused`

DRL 서버가 컨트롤러 측 포트(20002)에 listen을 시작하기 전에 호스트가 접속을
시도해서 발생합니다. 보통 다음 중 하나로 해결됩니다.

1. **이전 DRL이 좀비처럼 남아있는 경우** - `stop_existing_drl:=true` (기본값)이면
   자동으로 stop 후 재시작합니다. 그래도 안 되면 컨트롤러를 재기동.
2. **그리퍼 초기화가 오래 걸리는 경우** - `connect_timeout_sec`을 30~60으로 늘리세요.
3. **DRL 스크립트가 gripper init 단계에서 멈추는 경우** - 이 패키지의 DRL 스크립트는
   TCP 서버를 **먼저** 열고 init을 시도하므로, init 실패 시에도 클라이언트가 붙어서
   `INITIALIZE` 커맨드로 재시도할 수 있습니다.

### `Failed to start the DRL gripper TCP server.`

이전 DRL이 정지되는 도중에 새 `DrlStart`가 들어가면 컨트롤러가 거부합니다.
이 패키지는 다음을 수행해 자동 회복합니다.

1. `drl/drl_stop` 호출 후 **`get_drl_state`를 폴링**해서 PLAY가 아닌 상태가 될
   때까지 대기 (`drl_stop_settle_sec`).
2. `drl/drl_start`가 `success=False`로 응답하면 `drl_start_retry_count`만큼
   **자동 재시도** (`drl_start_retry_delay_sec` 간격).

그래도 실패하면 컨트롤러 측 DRL이 hung이거나 펜던트가 잠겨 있을 수 있습니다.
컨트롤러 펜던트의 **Manual → Auto** 모드 전환이 막혀있는지, 비상정지 상태가
풀렸는지 확인하세요.

### `INITIALIZE attempt N/M failed (socket): timed out`

DRL은 살아있고 TCP는 붙었는데 `INITIALIZE` 응답이 안 오는 상황입니다.
이 패키지의 DRL 스크립트는 부팅 시 그리퍼 자동 init을 하지 않고 명령
루프부터 진입하므로, 정상적이라면 INITIALIZE는 1초 내에 응답이 옵니다.
응답이 안 오면 보통 다음 중 하나:

1. **RS-485 케이블/그리퍼 전원 문제** - DRL은 modbus 응답을 못 받고 timeout 반복.
2. **이전 세션이 그리퍼를 이상한 상태로 남김** - 호스트가 자동으로
   소켓을 끊고 재연결한 뒤 `init_attempts`만큼 재시도합니다 (기본 5회).
3. **컨트롤러가 hung** - 컨트롤러 재기동이 가장 확실.

### `INITIALIZE attempt N/M failed (gripper): Controller returned error status 3`

`status 3 = STATUS_IO_ERROR` 으로, **그리퍼와 RS-485 modbus 통신이 안 됨**을
뜻합니다 (DRL과 호스트 간 TCP는 정상). 보통 다음 순서로 확인하세요:

1. **DRL이 자가 회복 시도 중** - 이 패키지의 `initialize_gripper`는 modbus를
   최대 4회까지 시도하면서 중간에 한 번 flange serial port를 close→open으로
   재초기화합니다. 호스트 측도 `init_attempts`만큼 (기본 5회) 재시도하므로
   잠깐 기다려 보세요.
2. **그리퍼 전원 확인** - LED, 정상 입력 전압 확인.
3. **컨트롤러 펜던트 재기동** - 가장 확실한 회복 방법. flange serial port가
   이전 세션의 이상한 상태로 남아 있는 경우 재기동이 즉시 해결합니다.
4. **RS-485 케이블/슬레이브 ID 확인** - `slave_id`가 그리퍼와 일치하는지
   (`BridgeConfig.slave_id` 기본 1).

### 웹 대시보드를 띄웠는데 그리퍼 상태가 `Not Ready`로만 나옴

DRL의 `initialize_gripper`가 RS-485 응답을 못 받은 상태입니다.
* 그리퍼 전원/케이블 확인
* 토크 토글 OFF→ON 또는 sliders 조정 후 다시 시도 (내부적으로 INITIALIZE 재시도)
