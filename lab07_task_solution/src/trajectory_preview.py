#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from lab_support import JOINT_NAMES


def sample_trajectory(times, positions, initial, elapsed):
    times = np.r_[0.0, times]
    positions = np.vstack([initial, positions])
    return np.array([np.interp(elapsed, times, positions[:, i]) for i in range(5)])


class TrajectoryPreview(Node):
    """Kinematic preview of the received message; no dynamics or controller simulation."""
    def __init__(self):
        super().__init__('trajectory_preview', namespace='lab07_task_solution')
        self.current = np.zeros(5)
        self.start = None
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(JointTrajectory, 'joint_trajectory', self.receive,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.05, self.update)

    def receive(self, message):
        try:
            assert len(message.joint_names) == 5 and set(message.joint_names) == set(JOINT_NAMES)
            order = [message.joint_names.index(name) for name in JOINT_NAMES]
            assert message.points
            assert all(len(p.positions) == 5 for p in message.points)
            times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in message.points])
            positions = np.array([[p.positions[i] for i in order] for p in message.points])
            assert np.all(np.isfinite(positions)) and times[0] > 0 and np.all(np.diff(times) > 0)
        except (AssertionError, ValueError, IndexError):
            self.get_logger().error('Invalid trajectory: expected five joints and increasing positive times.')
            return
        self.times, self.positions = times, positions
        self.initial = self.current.copy()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.start = stamp if stamp != 0 else self.get_clock().now().nanoseconds * 1e-9

    def update(self):
        stamp = self.get_clock().now()
        if self.start is not None:
            self.current = sample_trajectory(self.times, self.positions, self.initial,
                                             max(0., stamp.nanoseconds * 1e-9 - self.start))
        message = JointState()
        message.header.stamp = stamp.to_msg()
        message.name = JOINT_NAMES + ['gripper_finger_joint_l', 'gripper_finger_joint_r']
        message.position = self.current.tolist() + [0., 0.]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryPreview()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
