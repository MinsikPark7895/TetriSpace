#!/usr/bin/env python3
"""
arm_move.py

dakae_helper.py 에서 생성한 /tmp/dakae_arm_move.json 을 읽어
로봇팔을 지정 좌표로 movel 이동합니다.

config 포맷:
{"x": 100.0, "y": 50.0, "z": 300.0, "vel": 20, "acc": 20}
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
import DR_init

if not hasattr(RcutilsLogger, "info"):
    RcutilsLogger.info = RcutilsLogger.dinfo

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"
CONFIG_PATH = "/tmp/dakae_arm_move.json"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class ArmMoveNode(Node):
    def __init__(self):
        super().__init__("arm_move_node", namespace=ROBOT_ID)


def main(args=None):
    try:
        with open(CONFIG_PATH, "r") as f:
            config = json.load(f)
    except Exception as e:
        print(f"[오류] config 파일 읽기 실패 ({CONFIG_PATH}): {e}")
        return

    x = float(config.get("x", 0.0))
    y = float(config.get("y", 0.0))
    z = float(config.get("z", 0.0))
    vel = int(config.get("vel", 20))
    acc = int(config.get("acc", 20))

    rclpy.init(args=args)
    node = ArmMoveNode()
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import movel, get_current_posx, DR_BASE, DR_MV_MOD_ABS
        from DR_common2 import posx
    except ImportError as e:
        node.get_logger().error(f"DSR import 실패: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        current_pos, _ = get_current_posx(ref=DR_BASE)
        target = posx(x, y, z, current_pos[3], current_pos[4], current_pos[5])
        node.get_logger().info(f"이동 중: x={x:.1f}, y={y:.1f}, z={z:.1f}")
        movel(target, vel, acc, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        node.get_logger().info("✅ 이동 완료")
    except Exception as e:
        node.get_logger().error(f"이동 실패: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
