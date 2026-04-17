#!/usr/bin/env python3
"""
dakae_bringup - 대화형 조작 도우미 (Python, 계층 메뉴)
system.launch.py가 자동으로 이 스크립트를 새 터미널에서 실행함
"""

import os
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

# ============================================================================
# 설정
# ============================================================================
LOG_DIR = Path.home() / "dakae_logs"
VISION_MARKER_LOG_DIR = LOG_DIR / "vision" / "marker"
VISION_COLOR_LOG_DIR = LOG_DIR / "vision" / "color"
ROBOT_LOG_DIR = LOG_DIR / "robot"
MAX_LOGS = 100
SERVICE_TIMEOUT = 30

# 한글 → 영어 색상 변환
COLOR_MAP = {
    "빨강": "red", "빨간색": "red", "빨간": "red", "적색": "red",
    "파랑": "blue", "파란색": "blue", "파란": "blue", "청색": "blue",
    "초록": "green", "초록색": "green", "녹색": "green",
    "노랑": "yellow", "노란색": "yellow", "노란": "yellow",
    "흰색": "white", "하양": "white", "하얀색": "white",
    "검정": "black", "검정색": "black", "검은색": "black", "흑색": "black",
    "주황": "orange", "주황색": "orange",
    "보라": "purple", "보라색": "purple",
    "분홍": "pink", "분홍색": "pink",
    "회색": "gray",
}

# 알려진 서비스/노드/토픽 설명
KNOWN_SERVICES = {
    "/detect_cubes": "비전 서비스 — 카메라로 큐브 탐지 (색상 기반), 3D 좌표 반환",
    "/detect_markers": "비전 서비스 — ArUco 마커 탐지, ID/좌표/회전각 반환",
    "/dsr01/gripper_move": "그리퍼 서비스 — stroke 값(0~1000)으로 그리퍼 이동",
}
KNOWN_NODES = {
    "/cube_service_server": "비전 노드 — RealSense + Gemini API로 큐브 인식",
    "/marker_service_server": "비전 노드 — RealSense + ArUco로 마커 인식",
    "/gripper_server_node": "그리퍼 노드 — 두산 DRL 통한 그리퍼 제어",
}
KNOWN_TOPICS = {
    "/rosout": "ROS2 기본 로그 토픽 (모든 노드의 로그 메시지)",
    "/parameter_events": "파라미터 변경 이벤트 (ROS2 기본)",
}

# 헤더 너비
HEADER_WIDTH = 62


# ============================================================================
# 유틸
# ============================================================================
def setup():
    VISION_MARKER_LOG_DIR.mkdir(parents=True, exist_ok=True)
    VISION_COLOR_LOG_DIR.mkdir(parents=True, exist_ok=True)
    ROBOT_LOG_DIR.mkdir(parents=True, exist_ok=True)


def run_command(cmd, timeout=SERVICE_TIMEOUT):
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout,
        )
        return result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        return f"[오류] 명령어 시간 초과 ({timeout}초)"
    except Exception as e:
        return f"[오류] {e}"


def clear_screen():
    os.system("clear")


def pause():
    input("\n  엔터 치면 돌아갑니다...")


def cleanup_logs(log_dir):
    files = sorted(log_dir.glob("*.txt"), reverse=True)
    if len(files) > MAX_LOGS:
        for old_file in files[MAX_LOGS:]:
            old_file.unlink()


def print_header(path):
    """모든 메뉴 상단에 공통 헤더 출력 (경로 길어도 잘리지 않게)"""
    w = HEADER_WIDTH
    title = "dakae 프로젝트 조작기"
    title_pad = (w - 4 - len(title)) // 2
    print("╔" + "═" * (w - 2) + "╗")
    print("║" + " " * title_pad + title + " " * (w - 2 - title_pad - len(title)) + "║")
    print("╟" + "─" * (w - 2) + "╢")
    # 경로는 길면 자동으로 넘침 — 잘리지 않게 ║ 뒤에 그냥 출력
    print(f"║  위치: {path}")
    print("╚" + "═" * (w - 2) + "╝")
    print()


def is_service_available(service_name):
    out = run_command("ros2 service list", timeout=5)
    return service_name in out.split()


def warn_service_unavailable(service_name, hint=""):
    print()
    print("  ┌───────────────────────────────────────────┐")
    print("  │  ⚠ 서비스 연결 안 됨                       │")
    print("  └───────────────────────────────────────────┘")
    print(f"\n  현재 '{service_name}' 서비스가 연결되어 있지 않습니다.")
    print("  실행할 수 없습니다.")
    if hint:
        print(f"\n  확인: {hint}")
    pause()


# ============================================================================
# 색상 비전 탐지 (공용)
# ============================================================================
def call_detect_service(color_filter, label):
    cmd = (
        f"ros2 service call /detect_cubes "
        f"dakae_interfaces/srv/DetectCubes "
        f"\"{{target_colors: {color_filter}}}\""
    )
    raw = run_command(cmd, timeout=SERVICE_TIMEOUT)

    cubes = re.findall(
        r"color='([^']+)',\s*position=geometry_msgs\.msg\.Vector3\("
        r"x=([-\d.e]+),\s*y=([-\d.e]+),\s*z=([-\d.e]+)\)",
        raw,
    )
    cubes = [(c, float(x), float(y), float(z)) for c, x, y, z in cubes]

    success = re.search(r"success=(True|False)", raw)
    success = success.group(1) if success else "Unknown"

    message = re.search(r"message='([^']*)'", raw)
    message = message.group(1) if message else ""

    lines = []
    if success == "True" and cubes:
        for i, (color, x, y, z) in enumerate(cubes, 1):
            lines.append(
                f"  큐브{i} - 색깔: {color:<8} "
                f"위치: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m"
            )
        lines.append("")
        lines.append(f"  성공: True | 총 {len(cubes)}개 탐지")
    elif success == "True":
        lines.append("  탐지 결과: 대상 없음")
        lines.append(f"  사유: 카메라 시야에 '{label}' 큐브가 없거나 인식 실패")
    else:
        lines.append("  탐지 실패")
        lines.append(f"  사유: {message or '알 수 없는 오류'}")

    return cubes, "\n".join(lines)


def save_color_vision_log(formatted, label):
    now = datetime.now()
    filename = f"color_{now.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
    filepath = VISION_COLOR_LOG_DIR / filename
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae 색상 비전 탐지 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  필터: {label}\n")
        f.write("-" * 50 + "\n")
        f.write(formatted + "\n")
    cleanup_logs(VISION_COLOR_LOG_DIR)
    return filepath


def detect_color_and_report(color_filter, label):
    if not is_service_available("/detect_cubes"):
        warn_service_unavailable(
            "/detect_cubes",
            hint="비전 노드 (cube_service_server)가 기동되었는지 확인하세요.",
        )
        return None

    print(f"\n  >>> 색상 큐브 탐지 중 ({label})... (최대 {SERVICE_TIMEOUT}초)\n")
    cubes, formatted = call_detect_service(color_filter, label)
    print(formatted)
    filepath = save_color_vision_log(formatted, label)
    print(f"\n  >>> 저장: {filepath}")
    return cubes


# ============================================================================
# 마커 비전 탐지
# ============================================================================
def call_marker_service():
    cmd = (
        "ros2 service call /detect_markers "
        "dakae_interfaces/srv/DetectMarkers \"{}\""
    )
    raw = run_command(cmd, timeout=SERVICE_TIMEOUT)

    # 마커 파싱
    markers = re.findall(
        r"type='([^']+)',\s*id=(\d+),\s*"
        r"position=geometry_msgs\.msg\.Point\("
        r"x=([-\d.e]+),\s*y=([-\d.e]+),\s*z=([-\d.e]+)\),\s*"
        r"rz=([-\d.e]+)",
        raw,
    )
    parsed = [(t, int(mid), float(x), float(y), float(z), float(rz))
              for t, mid, x, y, z, rz in markers]

    success = re.search(r"success=(True|False)", raw)
    success = success.group(1) if success else "Unknown"

    message = re.search(r"message='([^']*)'", raw)
    message = message.group(1) if message else ""

    lines = []
    if success == "True" and parsed:
        for i, (mtype, mid, x, y, z, rz) in enumerate(parsed, 1):
            lines.append(
                f"  마커{i} - 타입: {mtype:<8} ID: {mid:<4} "
                f"위치: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m  회전: {rz:.1f}°"
            )
        lines.append("")
        lines.append(f"  성공: True | 총 {len(parsed)}개 마커 감지")
    elif success == "True":
        lines.append("  탐지 결과: 마커 없음")
        lines.append("  사유: 카메라 시야에 ArUco 마커가 없거나 인식 실패")
    else:
        lines.append("  탐지 실패")
        lines.append(f"  사유: {message or '알 수 없는 오류'}")

    return parsed, "\n".join(lines)


def save_marker_vision_log(formatted, label):
    now = datetime.now()
    filename = f"marker_{now.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
    filepath = VISION_MARKER_LOG_DIR / filename
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae 마커 비전 탐지 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  모드: {label}\n")
        f.write("-" * 50 + "\n")
        f.write(formatted + "\n")
    cleanup_logs(VISION_MARKER_LOG_DIR)
    return filepath


def detect_marker_and_report():
    if not is_service_available("/detect_markers"):
        warn_service_unavailable(
            "/detect_markers",
            hint="마커 비전 노드 (marker_service_server)가 기동되었는지 확인하세요.",
        )
        return None

    print(f"\n  >>> ArUco 마커 탐지 중... (최대 {SERVICE_TIMEOUT}초)\n")
    markers, formatted = call_marker_service()
    print(formatted)
    filepath = save_marker_vision_log(formatted, "마커 탐지")
    print(f"\n  >>> 저장: {filepath}")
    return markers


# ============================================================================
# 통합 시나리오 실행
# ============================================================================
def run_integrated_scenario(executable, label, needs_vision=True, needs_marker=False):
    """팀원의 실행 스크립트를 실행 + 출력 파싱해서 로그 저장"""
    # 그리퍼 서비스 선행 확인
    if not is_service_available("/dsr01/gripper_move"):
        warn_service_unavailable(
            "/dsr01/gripper_move",
            hint="gripper_server가 떠있고,\n"
                 "         두산 로봇 드라이버도 실행 중이어야 합니다.",
        )
        return

    if needs_vision and not is_service_available("/detect_cubes"):
        warn_service_unavailable(
            "/detect_cubes",
            hint="비전 노드 (cube_service_server)가 기동되었는지 확인하세요.",
        )
        return

    if needs_marker and not is_service_available("/detect_markers"):
        warn_service_unavailable(
            "/detect_markers",
            hint="마커 비전 노드 (marker_service_server)가 기동되었는지 확인하세요.",
        )
        return

    clear_screen()
    print(f"  [전체 작동] > {label}\n")
    print("  ┌───────────────────────────────────────────┐")
    print("  │  ⚠ 로봇이 실제로 움직입니다                │")
    print("  │    주변에 사람/물건이 없는지 확인하세요     │")
    print("  │                                             │")
    print("  │  ※ 실행 중 화면에 출력이 안 보입니다       │")
    print("  │    완료 후 한 번에 표시됩니다               │")
    print("  │  ※ 중단: Ctrl+C                            │")
    print("  └───────────────────────────────────────────┘\n")
    confirm = input("  실행하시겠습니까? (y/n): ").strip().lower()
    if confirm != "y":
        print("  취소됨.")
        pause()
        return

    now = datetime.now()
    ts = now.strftime('%Y-%m-%d_%H-%M-%S')

    print(f"\n  >>> {label} 실행 중... (완료까지 대기)")
    print("      (로봇이 움직이고 있습니다)\n")

    try:
        result = subprocess.run(
            f"ros2 run test_first {executable}",
            shell=True, capture_output=True, text=True, timeout=300,
        )
        output = result.stdout + result.stderr
        exit_code = result.returncode
    except subprocess.TimeoutExpired:
        output = "[오류] 5분 타임아웃 초과"
        exit_code = -1
    except KeyboardInterrupt:
        output = "[중단됨] 사용자가 Ctrl+C로 중단"
        exit_code = -2

    print("=" * 50)
    print("  실행 출력")
    print("=" * 50)
    print(output)
    print("=" * 50)

    # robot 로그 저장
    robot_filepath = ROBOT_LOG_DIR / f"robot_{ts}_{executable}.txt"
    with open(robot_filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae 통합 시나리오 실행 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  시나리오: {label}\n")
        f.write(f"  명령: ros2 run test_first {executable}\n")
        f.write(f"  종료 코드: {exit_code}\n")
        f.write("-" * 50 + "\n")
        f.write(output)
    cleanup_logs(ROBOT_LOG_DIR)
    print(f"\n  >>> robot 로그: {robot_filepath}")

    # 비전 로그 추출
    if needs_vision:
        vision_lines = extract_color_vision_from_output(output, executable)
        if vision_lines:
            vision_filepath = VISION_COLOR_LOG_DIR / f"color_{ts}.txt"
            with open(vision_filepath, "w", encoding="utf-8") as f:
                f.write("=" * 50 + "\n")
                f.write("  dakae 색상 비전 탐지 기록 (통합 실행)\n")
                f.write("=" * 50 + "\n")
                f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"  필터: 통합-{executable}\n")
                f.write("-" * 50 + "\n")
                f.write(vision_lines + "\n")
            cleanup_logs(VISION_COLOR_LOG_DIR)
            print(f"  >>> vision 로그: {vision_filepath}")

    if needs_marker:
        marker_lines = extract_marker_vision_from_output(output, executable)
        if marker_lines:
            marker_filepath = VISION_MARKER_LOG_DIR / f"marker_{ts}.txt"
            with open(marker_filepath, "w", encoding="utf-8") as f:
                f.write("=" * 50 + "\n")
                f.write("  dakae 마커 비전 탐지 기록 (통합 실행)\n")
                f.write("=" * 50 + "\n")
                f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"  모드: 통합-{executable}\n")
                f.write("-" * 50 + "\n")
                f.write(marker_lines + "\n")
            cleanup_logs(VISION_MARKER_LOG_DIR)
            print(f"  >>> marker 로그: {marker_filepath}")

    pause()


def extract_color_vision_from_output(output, label):
    pattern = re.findall(
        r"\[\d+\]\s+(\w+)\s+cube\s*->\s*"
        r"x=([-\d.]+),\s*y=([-\d.]+),\s*z=([-\d.]+)",
        output,
    )
    if not pattern:
        return None

    lines = []
    for i, (color, x, y, z) in enumerate(pattern, 1):
        x_m = float(x) / 1000.0
        y_m = float(y) / 1000.0
        z_m = float(z) / 1000.0
        lines.append(
            f"  큐브{i} - 색깔: {color:<8} "
            f"위치: x={x_m:.3f}m, y={y_m:.3f}m, z={z_m:.3f}m"
        )
    lines.append("")
    lines.append(f"  성공: True | 총 {len(pattern)}개 탐지")
    return "\n".join(lines)


def extract_marker_vision_from_output(output, label):
    """cube_place 출력에서 마커 좌표 추출"""
    # cube 3: x=123.45, y=-67.89, z=155.00, rz=12.34
    # target 1: x=200.00, y=100.00, z=155.00, rz=0.00
    pattern = re.findall(
        r"(cube|target)\s+(\d+):\s*"
        r"x=([-\d.]+),\s*y=([-\d.]+),\s*z=([-\d.]+),\s*rz=([-\d.]+)",
        output,
    )
    if not pattern:
        return None

    lines = []
    for mtype, mid, x, y, z, rz in pattern:
        x_m = float(x) / 1000.0
        y_m = float(y) / 1000.0
        z_m = float(z) / 1000.0
        label_kr = "큐브" if mtype == "cube" else "목적지"
        lines.append(
            f"  {label_kr} {mid} - "
            f"위치: x={x_m:.3f}m, y={y_m:.3f}m, z={z_m:.3f}m  회전: {float(rz):.1f}°"
        )
    lines.append("")
    lines.append(f"  총 {len(pattern)}개 마커 정보 추출")
    return "\n".join(lines)


# ============================================================================
# [프로그램 작동] > [전체 작동]
# ============================================================================
def full_run_menu(parent_path):
    path = parent_path + " > [전체 작동]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 마커 인식 픽앤플레이스")
        print("      └ ArUco 마커 ID로 큐브/목적지를 인식하여 분류")
        print()
        print("   2) 전체 색 픽앤플레이스")
        print("      └ Gemini 비전으로 색상별 큐브를 인식하여 이동")
        print()
        print("   3) 팔 + 그리퍼 기본 테스트")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            run_integrated_scenario("cube_place", "마커 인식 픽앤플레이스", needs_vision=False, needs_marker=True)
        elif c == "2":
            run_integrated_scenario("move_line", "전체 색 픽앤플레이스", needs_vision=True)
        elif c == "3":
            run_integrated_scenario("arm_gripper_test", "팔+그리퍼 테스트", needs_vision=False)
        elif c == "b":
            return


# ============================================================================
# [프로그램 작동] > [비전 작동]
# ============================================================================
def vision_menu(parent_path):
    path = parent_path + " > [비전 작동]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 마커 비전 촬영")
        print("   2) 전체 색 비전 촬영")
        print("   3) 특정 색 비전 촬영")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            clear_screen()
            print_header(path + " > [마커 비전]")
            detect_marker_and_report()
        elif c == "2":
            clear_screen()
            print_header(path + " > [전체 색 비전]")
            detect_color_and_report("[]", "전체")
        elif c == "3":
            clear_screen()
            print_header(path + " > [특정 색 비전]")
            print("  한글 또는 영어로 색상 입력 (b = 뒤로)")
            print("  예: 빨강, red, 파랑, blue, 초록, green ...\n")
            user_input = input("  색상: ").strip().lower()
            if user_input == "b" or not user_input:
                continue
            color = COLOR_MAP.get(user_input, user_input)
            color_filter = f"['{color}']"
            label = user_input if user_input == color else f"{user_input}({color})"
            detect_color_and_report(color_filter, label)
        elif c == "b":
            return


# ============================================================================
# [프로그램 작동] > [로봇 작동] > [그리퍼]
# ============================================================================
def gripper_menu(parent_path):
    path = parent_path + " > [그리퍼]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 완전 열기 (stroke=0)")
        print("   2) 완전 닫기 (stroke=1000)")
        print("   3) 직접 입력 (0~1000)")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            call_gripper(0, "완전 열기")
        elif c == "2":
            call_gripper(1000, "완전 닫기")
        elif c == "3":
            val = input("  stroke 값 입력 (0~1000, b=취소): ").strip().lower()
            if val == "b" or not val:
                continue
            try:
                stroke = int(val)
                if not (0 <= stroke <= 1000):
                    print("  범위 초과 (0~1000)")
                    pause()
                    continue
                call_gripper(stroke, f"stroke={stroke}")
            except ValueError:
                print("  숫자를 입력해주세요.")
                pause()
        elif c == "b":
            return


def call_gripper(stroke, label):
    if not is_service_available("/dsr01/gripper_move"):
        warn_service_unavailable(
            "/dsr01/gripper_move",
            hint="gripper_server가 떠있고,\n"
                 "         두산 로봇 드라이버도 실행 중이어야 합니다.",
        )
        return

    print(f"\n  >>> 그리퍼 이동 중 ({label}, 최대 {SERVICE_TIMEOUT}초)...")
    cmd = (
        f"ros2 service call /dsr01/gripper_move "
        f"dakae_interfaces/srv/MoveGripper "
        f"\"{{stroke: {stroke}}}\""
    )
    raw = run_command(cmd, timeout=SERVICE_TIMEOUT)

    success = re.search(r"success=(True|False)", raw)
    success = success.group(1) if success else "Unknown"
    message = re.search(r"message='([^']*)'", raw)
    message = message.group(1) if message else ""

    if success == "True":
        print(f"\n  ✅ 성공: {message or label}")
    else:
        print(f"\n  ❌ 실패: {message or '알 수 없는 오류'}")

    now = datetime.now()
    filename = f"robot_{now.strftime('%Y-%m-%d_%H-%M-%S')}_gripper.txt"
    filepath = ROBOT_LOG_DIR / filename
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae 로봇 명령 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  명령: 그리퍼 {label}\n")
        f.write(f"  결과: {success}\n")
        f.write(f"  메시지: {message}\n")
    cleanup_logs(ROBOT_LOG_DIR)
    print(f"  >>> 저장: {filepath}")

    pause()


# ============================================================================
# [프로그램 작동] > [로봇 작동]
# ============================================================================
def robot_menu(parent_path):
    path = parent_path + " > [로봇 작동]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 그리퍼")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            gripper_menu(path)
        elif c == "b":
            return


# ============================================================================
# [프로그램 작동] — 메인
# ============================================================================
def program_menu(parent_path):
    path = parent_path + " > [프로그램 작동]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 전체 작동")
        print("   2) 비전 작동")
        print("   3) 로봇 작동")
        print()
        print("   h) 로봇팔 홈 위치로 복귀")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            full_run_menu(path)
        elif c == "2":
            vision_menu(path)
        elif c == "3":
            robot_menu(path)
        elif c == "h":
            run_integrated_scenario("go_home", "홈 위치 복귀", needs_vision=False)
        elif c == "b":
            return


# ============================================================================
# [환경 설정]
# ============================================================================
def print_with_description(items, known_map):
    for item in items:
        desc = known_map.get(item.strip(), "")
        if desc:
            print(f"  {item:<40} → {desc}")
        else:
            print(f"  {item}")


def show_services(path):
    clear_screen()
    print_header(path)
    out = run_command("ros2 service list", timeout=5)
    lines = [l for l in out.strip().split("\n") if l.strip()]
    print_with_description(lines, KNOWN_SERVICES)
    pause()


def show_nodes(path):
    clear_screen()
    print_header(path)
    out = run_command("ros2 node list", timeout=5)
    lines = [l for l in out.strip().split("\n") if l.strip()]
    print_with_description(lines, KNOWN_NODES)
    pause()


def show_topics(path):
    clear_screen()
    print_header(path)
    out = run_command("ros2 topic list", timeout=5)
    lines = [l for l in out.strip().split("\n") if l.strip()]
    print_with_description(lines, KNOWN_TOPICS)
    pause()


def view_logs_in(log_dir, title, path):
    while True:
        clear_screen()
        print_header(path)

        files = sorted(log_dir.glob("*.txt"), reverse=True)
        if not files:
            print("  기록이 없습니다.")
            pause()
            return

        for i, f in enumerate(files[:100], 1):
            name = f.stem
            parts = name.split("_", 1)
            if len(parts) == 2:
                ts = parts[1]
                date_part, _, time_part = ts.partition("_")
                time_display = time_part.replace("-", ":")
                display = f"{date_part} {time_display}"
            else:
                display = name
            print(f"  {i:3d}) {display}")

        print(f"\n  총 {len(files)}개")
        num = input("\n  번호 선택 (b = 뒤로): ").strip().lower()

        if num == "b" or not num:
            return

        try:
            idx = int(num) - 1
            if 0 <= idx < len(files):
                clear_screen()
                print_header(path)
                print(files[idx].read_text(encoding="utf-8"))
                pause()
        except ValueError:
            pass

def vision_log_menu(parent_path):
    path = parent_path + " > [비전 로그]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 마커 비전 로그")
        print("   2) 색 비전 로그")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            view_logs_in(VISION_MARKER_LOG_DIR, "마커 비전", path + " > [마커 비전]")
        elif c == "2":
            view_logs_in(VISION_COLOR_LOG_DIR, "색 비전", path + " > [색 비전]")
        elif c == "b":
            return


def log_menu(parent_path):
    path = parent_path + " > [로그 확인]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 비전 로그")
        print("   2) 로봇 로그")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            vision_log_menu(path)
        elif c == "2":
            view_logs_in(ROBOT_LOG_DIR, "로봇", path + " > [로봇 로그]")
        elif c == "b":
            return


def settings_menu(parent_path):
    path = parent_path + " > [환경 설정]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 서비스 목록")
        print("   2) 노드 목록")
        print("   3) 토픽 목록")
        print("   4) 로그 확인")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            show_services(path + " > [서비스 목록]")
        elif c == "2":
            show_nodes(path + " > [노드 목록]")
        elif c == "3":
            show_topics(path + " > [토픽 목록]")
        elif c == "4":
            log_menu(path)
        elif c == "b":
            return


# ============================================================================
# 메인
# ============================================================================
def main_menu():
    setup()
    path = "[메인]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) [프로그램 작동]")
        print("   2) [환경 설정]")
        print()
        print("   q) 종료")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            program_menu(path)
        elif c == "2":
            settings_menu(path)
        elif c == "q":
            print("\n  종료합니다.")
            sys.exit(0)


if __name__ == "__main__":
    main_menu()