#!/usr/bin/env python3
"""
dakae_bringup - 대화형 조작 도우미 (Python, 계층 메뉴)
system.launch.py가 자동으로 이 스크립트를 새 터미널에서 실행함
"""

import json
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
VISION_YOLO_LOG_DIR = LOG_DIR / "vision" / "yolo"
ROBOT_LOG_DIR = LOG_DIR / "robot"
MAX_LOGS = 100
SERVICE_TIMEOUT = 30

GRIPPER_SAFE_GRASP_ACTION = "/gripper_service/safe_grasp"
DETECT_OBJECTS_SERVICE = "/detect_objects"

GRIPPER_POS_OPEN = 0
GRIPPER_POS_CLOSE = 1150
GRIPPER_POS_MAX = 1150

ARM_MOVE_CONFIG = "/tmp/dakae_arm_move.json"
TASK_CONFIG = "/tmp/dakae_task.json"

KNOWN_SERVICES = {
    DETECT_OBJECTS_SERVICE: "비전 서비스 — YOLO8 물체 탐지, label/3D좌표/각도 반환",
    "/gripper_service/get_state": "그리퍼 서비스 — 현재 상태 조회",
    "/gripper_service/get_position": "그리퍼 서비스 — 현재 위치 조회",
}
KNOWN_NODES = {
    "/yolo_detector": "비전 노드 — YOLO8 물체 탐지 + 윤곽 분석 + 3D 좌표 변환",
    "/gripper_service": "그리퍼 노드 — dsr_gripper_tcp TCP 브리지 (RH-P12-RN)",
}
KNOWN_TOPICS = {
    "/rosout": "ROS2 기본 로그 토픽 (모든 노드의 로그 메시지)",
    "/parameter_events": "파라미터 변경 이벤트 (ROS2 기본)",
    "/gripper_service/state": "그리퍼 상태 토픽 (주기 발행)",
    "/gripper_service/joint_state": "그리퍼 관절 상태 토픽",
}

HEADER_WIDTH = 62


# ============================================================================
# 유틸
# ============================================================================
def setup():
    VISION_YOLO_LOG_DIR.mkdir(parents=True, exist_ok=True)
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
    w = HEADER_WIDTH
    title = "dakae 프로젝트 조작기"
    title_pad = (w - 4 - len(title)) // 2
    print("╔" + "═" * (w - 2) + "╗")
    print("║" + " " * title_pad + title + " " * (w - 2 - title_pad - len(title)) + "║")
    print("╟" + "─" * (w - 2) + "╢")
    print(f"║  위치: {path}")
    print("╚" + "═" * (w - 2) + "╝")
    print()


def is_vision_service_available():
    out = run_command("ros2 service list", timeout=5)
    return DETECT_OBJECTS_SERVICE in out.split()


def warn_service_unavailable(service_name, hint=""):
    print()
    print("  ┌───────────────────────────────────────────┐")
    print("  │  ⚠ 서비스 연결 안 됨                       │")
    print("  └───────────────────────────────────────────┘")
    print(f"\n  현재 '{service_name}' 서비스가 연결되어 있지 않습니다.")
    if hint:
        print(f"\n  확인: {hint}")
    pause()


# ============================================================================
# YOLO8 비전 탐지
# ============================================================================
def call_yolo_detect(target_labels=None):
    """
    /detect_objects 서비스 호출 → [(id, label, x, y, z), ...] 반환
    좌표는 미터 단위 (m).
    """
    if target_labels is None:
        target_labels = []
    labels_yaml = str(target_labels).replace("'", '"')
    cmd = (
        f"ros2 service call {DETECT_OBJECTS_SERVICE} "
        f"dakae_interfaces/srv/DetectObjects "
        f"\"{{target_labels: {labels_yaml}}}\""
    )
    raw = run_command(cmd, timeout=SERVICE_TIMEOUT)

    objects = re.findall(
        r"id=(\d+),\s*label='([^']+)',\s*position=geometry_msgs\.msg\.Vector3\("
        r"x=([-\d.e]+),\s*y=([-\d.e]+),\s*z=([-\d.e]+)\)",
        raw,
    )
    parsed = [(int(oid), label, float(x), float(y), float(z))
              for oid, label, x, y, z in objects]

    success = re.search(r"success=(True|False)", raw)
    if not success or success.group(1) != "True":
        return []
    return parsed


def format_yolo_result(objects):
    if not objects:
        return "  탐지된 물체 없음"
    lines = []
    for i, (oid, label, x, y, z) in enumerate(objects, 1):
        lines.append(
            f"  {i}) [{label}]  "
            f"x={x*1000:.1f}mm  y={y*1000:.1f}mm  z={z*1000:.1f}mm"
        )
    lines.append(f"\n  총 {len(objects)}개 탐지")
    return "\n".join(lines)


def save_yolo_log(objects, label_filter):
    now = datetime.now()
    filename = f"yolo_{now.strftime('%Y-%m-%d_%H-%M-%S')}.txt"
    filepath = VISION_YOLO_LOG_DIR / filename
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae YOLO8 비전 탐지 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  필터: {label_filter or '전체'}\n")
        f.write("-" * 50 + "\n")
        f.write(format_yolo_result(objects) + "\n")
    cleanup_logs(VISION_YOLO_LOG_DIR)
    return filepath


def detect_yolo_and_report(target_labels=None, filter_label="전체"):
    if not is_vision_service_available():
        warn_service_unavailable(
            DETECT_OBJECTS_SERVICE,
            hint="비전 노드 (object_contour_service_server)가 기동되었는지 확인하세요.",
        )
        return None

    print(f"\n  >>> YOLO8 탐지 중 ({filter_label})... (최대 {SERVICE_TIMEOUT}초)\n")
    objects = call_yolo_detect(target_labels)
    formatted = format_yolo_result(objects)
    print(formatted)
    filepath = save_yolo_log(objects, filter_label)
    print(f"\n  >>> 저장: {filepath}")
    return objects


# ============================================================================
# [프로그램 작동] > [전체 작동]  (YOLO8 탐지 → 선택 → 목적지 → 실행)
# ============================================================================
def full_run_menu(parent_path):
    path = parent_path + " > [전체 작동]"

    if not is_vision_service_available():
        warn_service_unavailable(
            DETECT_OBJECTS_SERVICE,
            hint="비전 노드 (object_contour_service_server)가 기동되었는지 확인하세요.",
        )
        return

    # ── 1단계: YOLO8 탐지 ──────────────────────────────────────────────────
    clear_screen()
    print_header(path)
    print("  [1/4] YOLO8 물체 탐지 중...\n")
    objects = call_yolo_detect([])

    if not objects:
        print("  탐지된 물체가 없습니다. 카메라/비전 노드를 확인하세요.")
        pause()
        return

    print(f"  탐지된 물체 ({len(objects)}개):\n")
    for i, (oid, label, x, y, z) in enumerate(objects, 1):
        print(f"    {i}) [{label}]  x={x*1000:.1f}mm  y={y*1000:.1f}mm  z={z*1000:.1f}mm")

    # ── 2단계: 옮길 물체 선택 ──────────────────────────────────────────────
    print()
    print("  [2/4] 옮길 물체 번호를 입력하세요.")
    print("        예: 1   또는   1,3   또는   all   (b=뒤로)")
    print()
    sel_input = input("  선택: ").strip().lower()

    if sel_input in ("b", ""):
        return

    if sel_input == "all":
        selected_indices = list(range(len(objects)))
    else:
        try:
            selected_indices = [int(s.strip()) - 1 for s in sel_input.split(",")]
            selected_indices = [i for i in selected_indices if 0 <= i < len(objects)]
        except ValueError:
            print("  잘못된 입력입니다.")
            pause()
            return

    if not selected_indices:
        print("  선택된 물체가 없습니다.")
        pause()
        return

    # ── 3단계: 각 물체 목적지 설정 ────────────────────────────────────────
    tasks = []
    for idx in selected_indices:
        oid, label, x_m, y_m, z_m = objects[idx]
        x_mm = x_m * 1000.0
        y_mm = y_m * 1000.0
        z_mm = z_m * 1000.0

        clear_screen()
        print_header(path)
        print(f"  [3/4] [{label}]  x={x_mm:.1f}mm  y={y_mm:.1f}mm  z={z_mm:.1f}mm\n")
        print("  목적지 선택:")
        print("    1) 기본 위치 (홈 복귀 — PLACE_X=373, Y=0, Z=200)")
        print("    2) 특정 위치 (좌표 직접 입력)")
        print()
        dest = input("  선택 (b=이 물체 건너뜀): ").strip().lower()

        if dest == "b":
            continue
        elif dest == "1":
            tasks.append({
                "label": label,
                "pick_x": x_mm, "pick_y": y_mm, "pick_z": z_mm,
                "dest_type": "home",
            })
        elif dest == "2":
            try:
                print()
                dx = float(input("    목적지 X (mm): ").strip())
                dy = float(input("    목적지 Y (mm): ").strip())
                dz = float(input("    목적지 Z (mm): ").strip())
                tasks.append({
                    "label": label,
                    "pick_x": x_mm, "pick_y": y_mm, "pick_z": z_mm,
                    "dest_type": "custom",
                    "dest_x": dx, "dest_y": dy, "dest_z": dz,
                })
            except ValueError:
                print("  좌표 입력 오류, 이 물체를 건너뜁니다.")
                pause()
        else:
            print("  잘못된 선택, 이 물체를 건너뜁니다.")

    if not tasks:
        print("\n  실행할 작업이 없습니다.")
        pause()
        return

    # ── 4단계: 확인 및 실행 ────────────────────────────────────────────────
    clear_screen()
    print_header(path)
    print("  [4/4] 실행 요약\n")
    for t in tasks:
        dest_str = (
            "기본 위치(홈)"
            if t["dest_type"] == "home"
            else f"({t['dest_x']:.0f}, {t['dest_y']:.0f}, {t['dest_z']:.0f})mm"
        )
        print(f"    [{t['label']}]  픽({t['pick_x']:.0f},{t['pick_y']:.0f},{t['pick_z']:.0f})  →  {dest_str}")

    print()
    print("  ┌───────────────────────────────────────────┐")
    print("  │  ⚠ 로봇이 실제로 움직입니다                │")
    print("  │    주변에 사람/물건이 없는지 확인하세요     │")
    print("  │  ※ 중단: 이 터미널에서 Ctrl+C             │")
    print("  └───────────────────────────────────────────┘")
    print()
    confirm = input("  실행하시겠습니까? (y/n): ").strip().lower()
    if confirm != "y":
        print("  취소됨.")
        pause()
        return

    # config 파일 작성
    with open(TASK_CONFIG, "w", encoding="utf-8") as f:
        json.dump({"objects": tasks}, f, ensure_ascii=False, indent=2)

    now = datetime.now()
    ts = now.strftime('%Y-%m-%d_%H-%M-%S')
    print(f"\n  >>> yolo_pick_place 실행 중... (완료까지 대기)\n")

    try:
        result = subprocess.run(
            "ros2 run test_first yolo_pick_place",
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

    robot_filepath = ROBOT_LOG_DIR / f"robot_{ts}_yolo_pick_place.txt"
    with open(robot_filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae YOLO8 픽앤플레이스 실행 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  작업 수: {len(tasks)}\n")
        f.write(f"  종료 코드: {exit_code}\n")
        f.write("-" * 50 + "\n")
        f.write(json.dumps({"objects": tasks}, ensure_ascii=False, indent=2) + "\n")
        f.write("-" * 50 + "\n")
        f.write(output)
    cleanup_logs(ROBOT_LOG_DIR)
    print(f"\n  >>> 로그: {robot_filepath}")
    pause()


# ============================================================================
# [프로그램 작동] > [비전 작동]
# ============================================================================
def vision_menu(parent_path):
    path = parent_path + " > [비전 작동]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) YOLO8 비전 촬영 (전체)")
        print("      └ 카메라 시야의 모든 물체 탐지")
        print()
        print("   2) YOLO8 비전 촬영 (라벨 지정)")
        print("      └ 특정 라벨만 탐지 (예: toy, bottle, box, cube)")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            clear_screen()
            print_header(path + " > [YOLO8 전체 탐지]")
            result = detect_yolo_and_report([], "전체")
            if result is not None:
                pause()
        elif c == "2":
            clear_screen()
            print_header(path + " > [YOLO8 라벨 지정]")
            print("  탐지할 라벨 입력 (쉼표 구분, b=뒤로)")
            print("  예: toy   또는   toy,bottle,box\n")
            user_input = input("  라벨: ").strip().lower()
            if user_input == "b" or not user_input:
                continue
            labels = [lb.strip() for lb in user_input.split(",") if lb.strip()]
            result = detect_yolo_and_report(labels, ",".join(labels))
            if result is not None:
                pause()
        elif c == "b":
            return


# ============================================================================
# [프로그램 작동] > [로봇 작동] > [그리퍼]
# ============================================================================
def call_safe_grasp(target_position, label):
    """dsr_gripper_tcp SafeGrasp 액션으로 그리퍼 제어."""
    cmd = (
        f"ros2 action send_goal {GRIPPER_SAFE_GRASP_ACTION} "
        f"dsr_gripper_tcp_interfaces/action/SafeGrasp "
        f"\"{{target_position: {target_position}, max_current: 400, "
        f"current_delta_threshold: 0, timeout_sec: 10.0}}\""
    )
    print(f"\n  >>> SafeGrasp 실행 중 ({label}, target={target_position})...")
    raw = run_command(cmd, timeout=20)

    # Result: 섹션만 파싱 (Feedback 섹션 제외)
    result_section = raw.split("Result:")[-1] if "Result:" in raw else raw
    grasp_m = re.search(r"grasp_detected:\s*(true|false)", result_section, re.IGNORECASE)
    grasp = grasp_m.group(1).lower() if grasp_m else "unknown"
    msg_m = re.search(r"message:\s*(.+)", result_section)
    msg = msg_m.group(1).strip() if msg_m else ""
    pos_m = re.search(r"final_position:\s*(\d+)", result_section)
    final_pos = pos_m.group(1) if pos_m else "?"

    if grasp == "true":
        print(f"  완료  파지 감지됨  position={final_pos}")
    else:
        print(f"  완료  position={final_pos}  ({msg or '목표 위치 도달'})")

    now = datetime.now()
    filepath = ROBOT_LOG_DIR / f"robot_{now.strftime('%Y-%m-%d_%H-%M-%S')}_gripper.txt"
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae 그리퍼 SafeGrasp 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  명령: {label}  target_position={target_position}\n")
        f.write(f"  결과: grasp={grasp}  final_position={final_pos}\n")
        f.write("-" * 50 + "\n")
        f.write(raw)
    cleanup_logs(ROBOT_LOG_DIR)
    print(f"  >>> 저장: {filepath}")
    pause()


def gripper_menu(parent_path):
    path = parent_path + " > [그리퍼]"
    while True:
        clear_screen()
        print_header(path)
        print(f"   1) 열기       (SafeGrasp  position=0)")
        print(f"   2) 닫기       (SafeGrasp  position={GRIPPER_POS_CLOSE})")
        print(f"   3) 직접 입력  (SafeGrasp  0~{GRIPPER_POS_MAX})")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            call_safe_grasp(GRIPPER_POS_OPEN, "열기")
        elif c == "2":
            call_safe_grasp(GRIPPER_POS_CLOSE, "닫기")
        elif c == "3":
            val = input(f"  position 값 입력 (0~{GRIPPER_POS_MAX}, b=취소): ").strip().lower()
            if val == "b" or not val:
                continue
            try:
                pos = int(val)
                if not (0 <= pos <= GRIPPER_POS_MAX):
                    print(f"  범위 초과 (0~{GRIPPER_POS_MAX})")
                    pause()
                    continue
                call_safe_grasp(pos, f"직접 입력")
            except ValueError:
                print("  숫자를 입력해주세요.")
                pause()
        elif c == "b":
            return


# ============================================================================
# [프로그램 작동] > [로봇 작동] > [로봇팔]
# ============================================================================
def call_arm_move(x, y, z, vel=20, acc=20):
    config = {"x": x, "y": y, "z": z, "vel": vel, "acc": acc}
    with open(ARM_MOVE_CONFIG, "w", encoding="utf-8") as f:
        json.dump(config, f)

    print(f"\n  >>> 로봇팔 이동 중 x={x:.1f} y={y:.1f} z={z:.1f}...")
    try:
        result = subprocess.run(
            "ros2 run test_first arm_move",
            shell=True, capture_output=True, text=True, timeout=60,
        )
        output = result.stdout + result.stderr
    except subprocess.TimeoutExpired:
        output = "[오류] 타임아웃"

    print(output)

    now = datetime.now()
    filepath = ROBOT_LOG_DIR / f"robot_{now.strftime('%Y-%m-%d_%H-%M-%S')}_arm_move.txt"
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("=" * 50 + "\n")
        f.write("  dakae 로봇팔 이동 기록\n")
        f.write("=" * 50 + "\n")
        f.write(f"  일시: {now.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"  목표: x={x:.1f}, y={y:.1f}, z={z:.1f}  vel={vel}  acc={acc}\n")
        f.write("-" * 50 + "\n")
        f.write(output)
    cleanup_logs(ROBOT_LOG_DIR)
    print(f"  >>> 저장: {filepath}")
    pause()


def arm_menu(parent_path):
    path = parent_path + " > [로봇팔]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 홈 위치로 이동")
        print()
        print("   2) 특정 좌표로 이동 (movel)")
        print("      └ x, y, z (mm) 직접 입력")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            clear_screen()
            print_header(path + " > [홈 이동]")
            print("  ⚠ 로봇팔이 홈 위치로 이동합니다.\n")
            confirm = input("  실행하시겠습니까? (y/n): ").strip().lower()
            if confirm == "y":
                print("\n  >>> 홈으로 이동 중...")
                try:
                    result = subprocess.run(
                        "ros2 run test_first go_home",
                        shell=True, capture_output=True, text=True, timeout=60,
                    )
                    print(result.stdout + result.stderr)
                except subprocess.TimeoutExpired:
                    print("[오류] 타임아웃")
                pause()
        elif c == "2":
            clear_screen()
            print_header(path + " > [좌표 이동]")
            print("  목표 좌표 입력 (mm), b=취소\n")
            try:
                x_in = input("  X (mm): ").strip().lower()
                if x_in == "b":
                    continue
                x = float(x_in)
                y = float(input("  Y (mm): ").strip())
                z = float(input("  Z (mm): ").strip())

                vel_in = input("  속도 vel (기본 20, 엔터=기본): ").strip()
                vel = int(vel_in) if vel_in else 20
                acc_in = input("  가속 acc (기본 20, 엔터=기본): ").strip()
                acc = int(acc_in) if acc_in else 20

                print()
                print(f"  목표: x={x:.1f}, y={y:.1f}, z={z:.1f}  vel={vel}  acc={acc}")
                print("  ⚠ 로봇팔이 실제로 움직입니다.\n")
                confirm = input("  실행하시겠습니까? (y/n): ").strip().lower()
                if confirm == "y":
                    call_arm_move(x, y, z, vel, acc)
            except ValueError:
                print("  숫자를 입력해주세요.")
                pause()
        elif c == "b":
            return


# ============================================================================
# [프로그램 작동] > [로봇 작동]
# ============================================================================
def robot_menu(parent_path):
    path = parent_path + " > [로봇 작동]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) 그리퍼  (SafeGrasp 액션)")
        print("   2) 로봇팔  (홈 / 좌표 이동)")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            gripper_menu(path)
        elif c == "2":
            arm_menu(path)
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
        print("      └ YOLO8 탐지 → 물체 선택 → 목적지 선택 → 실행")
        print()
        print("   2) 비전 작동")
        print("      └ YOLO8 촬영 (전체 / 라벨 지정)")
        print()
        print("   3) 로봇 작동")
        print("      └ 그리퍼 / 로봇팔 개별 제어")
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
        elif c == "b":
            return


# ============================================================================
# [환경 설정]
# ============================================================================
def print_with_description(items, known_map):
    for item in items:
        desc = known_map.get(item.strip(), "")
        if desc:
            print(f"  {item:<45} → {desc}")
        else:
            print(f"  {item}")


def show_services(path):
    clear_screen()
    print_header(path)
    out = run_command("ros2 service list", timeout=5)
    lines = [ln for ln in out.strip().split("\n") if ln.strip()]
    print_with_description(lines, KNOWN_SERVICES)
    pause()


def show_nodes(path):
    clear_screen()
    print_header(path)
    out = run_command("ros2 node list", timeout=5)
    lines = [ln for ln in out.strip().split("\n") if ln.strip()]
    print_with_description(lines, KNOWN_NODES)
    pause()


def show_topics(path):
    clear_screen()
    print_header(path)
    out = run_command("ros2 topic list", timeout=5)
    lines = [ln for ln in out.strip().split("\n") if ln.strip()]
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
                display = f"{date_part} {time_part.replace('-', ':')}"
            else:
                display = name
            print(f"  {i:3d}) {display}")

        print(f"\n  총 {len(files)}개")
        num = input("\n  번호 선택 (b=뒤로): ").strip().lower()

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


def log_menu(parent_path):
    path = parent_path + " > [로그 확인]"
    while True:
        clear_screen()
        print_header(path)
        print("   1) YOLO8 비전 로그")
        print("   2) 로봇 실행 로그")
        print()
        print("   b) 뒤로")
        print()
        c = input("  선택: ").strip().lower()

        if c == "1":
            view_logs_in(VISION_YOLO_LOG_DIR, "YOLO8 비전", path + " > [YOLO8 비전 로그]")
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
