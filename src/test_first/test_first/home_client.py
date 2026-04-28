#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "e0509"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class HomeNode(Node):
    def __init__(self):
        super().__init__("home_node", namespace=ROBOT_ID)


def main(args=None):
    rclpy.init(args=args)
    node = HomeNode()
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import movej
        from DR_common2 import posj
    except ImportError as e:
        node.get_logger().error(f"DSR import 실패: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        home = posj(0, 0, 90, 0, 90, 0)
        vel = 30
        acc = 30

        node.get_logger().info("홈 위치로 이동 중...")
        movej(home, vel, acc)
        node.get_logger().info("✅ 홈 도착")

    except Exception as e:
        node.get_logger().error(f"홈 이동 실패: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()