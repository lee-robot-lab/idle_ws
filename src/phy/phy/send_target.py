"""CLI helper: publish an EE target from x y z yaw_deg.

Usage:
    ros2 run phy send_target -- 0.3 0.0 0.6 45
    ros2 run phy send_target -- 0.3 0.0 0.6        # yaw defaults to 0
    ros2 run phy send_target -- --line --duration 1.2 0.3 0.0 0.4 45
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from msgs.msg import EETarget
from rclpy.node import Node


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Yaw (rotation about world Z) → (x, y, z, w) unit quaternion."""
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Node("send_target")

    parser = argparse.ArgumentParser(
        description="Publish EE target. Default path uses /ee_target_pose; --line uses /ee_target straight_line."
    )
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw_deg", type=float, nargs="?", default=0.0)
    parser.add_argument("--line", action="store_true", help="publish EETarget with straight_line=True")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="duration_override_s for /ee_target. Ignored for default /ee_target_pose.",
    )
    parser.add_argument(
        "--ee-target",
        action="store_true",
        help="publish /ee_target EETarget even when --line is false.",
    )
    parser.add_argument(
        "--safe-transit",
        action="store_true",
        help="set EETarget.use_safe_transit=True. Current MVP planner path may ignore it.",
    )
    argv = sys.argv[1:] if args is None else args
    try:
        parsed = parser.parse_args(argv)
    except SystemExit:
        rclpy.shutdown()
        return

    x, y, z = float(parsed.x), float(parsed.y), float(parsed.z)
    yaw_deg = float(parsed.yaw_deg)
    yaw_rad = math.radians(yaw_deg)

    qx, qy, qz, qw = _yaw_to_quaternion(yaw_rad)
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    use_ee_target = bool(parsed.line or parsed.ee_target or parsed.duration > 0.0 or parsed.safe_transit)
    if use_ee_target:
        msg = EETarget()
        msg.pose = pose
        msg.duration_override_s = float(parsed.duration)
        msg.use_safe_transit = bool(parsed.safe_transit)
        msg.straight_line = bool(parsed.line)
        pub = node.create_publisher(EETarget, "/ee_target", 10)
        topic = "/ee_target"
    else:
        msg = pose
        pub = node.create_publisher(PoseStamped, "/ee_target_pose", 10)
        topic = "/ee_target_pose"

    # spin briefly so the publisher has time to connect before publishing
    import time
    rclpy.spin_once(node, timeout_sec=0.3)
    time.sleep(0.1)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(
        f"published target on {topic}: xyz=({x}, {y}, {z}) yaw={yaw_deg:.1f}° "
        f"({yaw_rad:.4f} rad) line={bool(parsed.line)} duration={float(parsed.duration):.2f}s"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
