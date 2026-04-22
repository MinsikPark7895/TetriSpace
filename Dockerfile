# ─────────────────────────────────────────────────────────────────────────────
# TetriSpace Dockerfile
# 목적: ROS 2 Humble 기반 TetriSpace 빌드 환경을 컨테이너로 정의
# 베이스: osrf/ros:humble-desktop (Ubuntu 22.04 Jammy)
# 참고: src/doosan-robot2/.devcontainer/Dockerfile
# ─────────────────────────────────────────────────────────────────────────────

# ── 베이스 이미지: ROS 2 Humble 공식 이미지 (Ubuntu 22.04) ──────────────────
FROM osrf/ros:humble-desktop

# ── 시스템 패키지 설치 ───────────────────────────────────────────────────────
# Doosan 로봇 드라이버 의존성 + 비전/메시지 관련 ROS 2 패키지
RUN apt-get update && apt-get install -y \
    python3-pip \                          # Python 패키지 설치 도구
    libpoco-dev \                          # Doosan 드라이버 의존성
    libyaml-cpp-dev \                      # YAML 파싱 라이브러리 (Doosan)
    dbus-x11 \                             # X11 디스플레이 버스 (GUI 지원)
    ros-humble-cv-bridge \                 # OpenCV ↔ ROS 2 이미지 변환 (dakae_vision)
    ros-humble-sensor-msgs \               # 카메라/센서 메시지 타입
    ros-humble-geometry-msgs \             # 좌표/점 메시지 타입 (dakae_interfaces)
    ros-humble-std-srvs \                  # 표준 서비스 타입 (test_first)
    ros-humble-control-msgs \              # 로봇 제어 메시지
    ros-humble-realtime-tools \            # 실시간 제어 유틸리티 (Doosan)
    ros-humble-xacro \                     # URDF 매크로 처리 (로봇 모델)
    ros-humble-joint-state-publisher-gui \ # 관절 상태 시각화
    ros-humble-ros2-control \              # ROS 2 제어 프레임워크
    ros-humble-ros2-controllers \          # 표준 컨트롤러 모음
    ros-humble-gazebo-msgs \               # Gazebo 시뮬레이터 메시지
    ros-humble-moveit-msgs \               # MoveIt 경로계획 메시지
    ros-humble-gazebo-ros-pkgs \           # Gazebo-ROS 2 연동 패키지
    ros-humble-ign-ros2-control \          # Ignition Gazebo 컨트롤러
    && rm -rf /var/lib/apt/lists/*         # apt 캐시 제거 (이미지 크기 최소화)

# ── Python 패키지 설치 ───────────────────────────────────────────────────────
# dakae_vision/requirements.txt 기준:
#   pyrealsense2    : Intel RealSense D455f 카메라 SDK
#   opencv-contrib  : ArUco 마커 검출 포함 OpenCV
#   numpy           : 좌표 변환 연산
#   google-genai    : Gemini VLA 모델 API
#   python-dotenv   : .env 파일 로드 (API 키 관리)
COPY src/dakae_vision/requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

# ── 워크스페이스 구성 ────────────────────────────────────────────────────────
WORKDIR /workspace
COPY . .    # 호스트의 TetriSpace 디렉토리 전체를 컨테이너로 복사
            # (.dockerignore로 build/, install/, log/, .env 등은 제외됨)

# ── colcon 빌드 ──────────────────────────────────────────────────────────────
# --symlink-install : Python 패키지를 심링크로 설치 (소스 변경 즉시 반영)
# --packages-select : 커스텀 패키지 4개만 빌드 (doosan-robot2 전체 빌드 제외)
#   빌드 순서: dakae_interfaces → dakae_vision → test_first → dakae_bringup
RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install \
        --packages-select \
            dakae_interfaces \
            dakae_vision \
            test_first \
            dakae_bringup"

# ── 환경 자동 소싱 ───────────────────────────────────────────────────────────
# 컨테이너 진입(bash) 시 ROS 2 + 워크스페이스 환경이 자동으로 로드됨
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

# ── 환경 변수 ────────────────────────────────────────────────────────────────
ENV ROS_DOMAIN_ID=29        # doosan-robot2와 동일한 ROS 도메인 ID (네트워크 격리)
ENV PYTHONUNBUFFERED=1      # Python 출력 버퍼링 비활성화 (docker logs 즉시 출력)

# ── 기본 실행 명령 ───────────────────────────────────────────────────────────
# docker run 시 bash 쉘로 진입 (ros2 launch 등 수동 실행 가능)
CMD ["/bin/bash"]