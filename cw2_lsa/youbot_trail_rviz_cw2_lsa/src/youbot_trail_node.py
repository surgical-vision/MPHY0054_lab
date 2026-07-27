#!/usr/bin/env python3

import math
from pathlib import Path
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

try:
    import rosbag
except ImportError:
    rosbag = None


class TrailNode(Node):
    """Visualise youBot checkpoints and trail in RViz."""

    def __init__(self) -> None:
        super().__init__('trail_node')

        qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', qos)
        self.create_subscription(LinkStates, '/gazebo/link_states', self._link_state_cb, qos)

        self.points_marker = self._make_marker(marker_id=0, marker_type=Marker.POINTS, color=(1.0, 0.0, 0.0))
        self.trail_marker = self._make_marker(marker_id=1, marker_type=Marker.LINE_STRIP, color=(1.0, 1.0, 1.0))
        self.trail_marker.scale.x = 0.005
        self.reached_marker = self._make_marker(marker_id=2, marker_type=Marker.POINTS, color=(0.0, 1.0, 0.0))
        self.reached_marker.scale.x = 0.01
        self.reached_marker.scale.y = 0.01

        self._checkpoint_threshold = 0.02
        self._load_checkpoints()

        self.create_timer(0.05, self._publish_markers)

    def _load_checkpoints(self) -> None:
        """Load checkpoint markers from the bundled bag file if the rosbag python API is available."""
        bag_path = Path(get_package_share_directory('youbot_trail_rviz_cw2_lsa')) / 'bags' / 'data.bag'
        if not bag_path.exists():
            self.get_logger().warn(f'Checkpoint bag not found at {bag_path}')
            return

        if rosbag is None:
            self.get_logger().warn('Python rosbag module not available; checkpoint markers will be empty.')
            return

        try:
            with rosbag.Bag(str(bag_path)) as bag:
                for _, msg, _ in bag.read_messages(topics=['joint_data']):
                    joint_msg: JointState = msg
                    youbot_pose = youbot_forward_kine(joint_msg.position)
                    self.points_marker.points.append(Point(
                        x=youbot_pose[0, 3],
                        y=youbot_pose[1, 3],
                        z=youbot_pose[2, 3]
                    ))
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f'Failed to read checkpoints from {bag_path}: {exc}')

    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        for marker in (self.points_marker, self.trail_marker, self.reached_marker):
            marker.header.stamp = now
            self.marker_pub.publish(marker)
        self._update_reached_points()

    def _link_state_cb(self, msg: LinkStates) -> None:
        if len(msg.pose) < 2:
            return
        mid_point = Point()
        mid_point.x = 0.5 * (msg.pose[-1].position.x + msg.pose[-2].position.x)
        mid_point.y = 0.5 * (msg.pose[-1].position.y + msg.pose[-2].position.y)
        mid_point.z = 0.5 * (msg.pose[-1].position.z + msg.pose[-2].position.z)

        self.trail_marker.points.append(mid_point)
        if len(self.trail_marker.points) > 800:
            self.trail_marker.points.pop(0)

    def _update_reached_points(self) -> None:
        if not self.points_marker.points or not self.trail_marker.points:
            return

        remaining: List[Point] = []
        for checkpoint in self.points_marker.points:
            reached = False
            for trail_pt in self.trail_marker.points:
                d = math.sqrt(
                    (trail_pt.x - checkpoint.x) ** 2 +
                    (trail_pt.y - checkpoint.y) ** 2 +
                    (trail_pt.z - checkpoint.z) ** 2
                )
                if d < self._checkpoint_threshold:
                    self.reached_marker.points.append(checkpoint)
                    reached = True
                    break
            if not reached:
                remaining.append(checkpoint)

        self.points_marker.points = remaining

    def _make_marker(self, marker_id: int, marker_type: int, color: Optional[tuple] = None) -> Marker:
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.id = marker_id
        marker.ns = 'points_and_lines'
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        if color:
            marker.color.r, marker.color.g, marker.color.b = color
        return marker


def dh_matrix_standard(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    A = np.zeros((4, 4))
    A[0, 0] = math.cos(theta)
    A[0, 1] = -math.sin(theta) * math.cos(alpha)
    A[0, 2] = math.sin(theta) * math.sin(alpha)
    A[0, 3] = a * math.cos(theta)

    A[1, 0] = math.sin(theta)
    A[1, 1] = math.cos(theta) * math.cos(alpha)
    A[1, 2] = -math.cos(theta) * math.sin(alpha)
    A[1, 3] = a * math.sin(theta)

    A[2, 0] = 0.0
    A[2, 1] = math.sin(alpha)
    A[2, 2] = math.cos(alpha)
    A[2, 3] = d

    A[3, 3] = 1.0
    return A


def youbot_forward_kine(joint: List[float]) -> np.ndarray:
    a = [-0.033, 0.155, 0.135, -0.002, 0.0]
    alpha = [math.pi / 2, 0.0, 0.0, math.pi / 2, math.pi]
    d = [0.145, 0.0, 0.0, 0.0, -0.185]
    theta = [
        math.radians(170) - math.pi,
        math.pi / 2 - math.radians(65),
        math.radians(146),
        -math.pi / 2 - math.radians(102.5),
        math.pi - math.radians(167.5)
    ]

    A = np.identity(4)
    for i in range(5):
        joint_angle = theta[i] - joint[i] if i == 0 else theta[i] + joint[i]
        A = A @ dh_matrix_standard(a[i], alpha[i], d[i], joint_angle)
    return A


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrailNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
