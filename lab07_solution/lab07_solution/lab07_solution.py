#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lab07_solution node: builds a PyKDL model from the URDF and publishes joint
trajectories, fake joint states, visualisation markers, and an end-effector TF.

High-level workflow aimed at beginners:
1. Parse the ``robot_description`` parameter and build a chain from ``base_link``
   to ``tip_link`` via PyKDL.
2. Predefine a couple of joint targets, run forward kinematics for each, and draw
   markers at the resulting poses in RViz.
3. Wrap the same joint angles into a ``JointTrajectory`` while periodically
   publishing them on ``/joint_states`` so ``robot_state_publisher`` can broadcast
   the arm_link_* frames.
4. Use a ``TransformBroadcaster`` to publish the KDL-computed end-effector frame
   (`arm_end_effector_kdl`) for sanity checks inside RViz.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

import PyKDL as kdl

from .src.urdf_kdl_utils import build_kdl_chain_from_urdf


class YoubotTrajectoryNode(Node):
    """Close ROS 2 port of the original ROS 1 trajectory publisher."""

    def __init__(self):
        super().__init__('lab07_youbot_traj')

        # Names of the five revolute joints on the youBot arm, matching arm_joint_* in the URDF
        self.joint_names = [
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'arm_joint_4',
            'arm_joint_5'
        ]
        # Publish gripper fingers as well so RViz can animate the claws
        self.gripper_joints = [
            'gripper_finger_joint_l',
            'gripper_finger_joint_r'
        ]

        # Sends the "next set of joint angles" to the controller
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/EffortJointInterface_trajectory_controller/command',
            5
        )
        # Mirrors the same angles on /joint_states so robot_state_publisher outputs TF
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        # Draw coloured spheres for each key pose inside RViz
        self.checkpoint_pub = self.create_publisher(
            Marker,
            'checkpoint_positions',
            marker_qos
        )

        # Parameters supply the URDF text plus the base/tip link names (defaults for youBot)
        self.declare_parameter('robot_description', '')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('tip_link', 'arm_link_5')

        robot_description = self.get_parameter(
            'robot_description'
        ).get_parameter_value().string_value
        base_link = self.get_parameter('base_link').get_parameter_value().string_value
        tip_link = self.get_parameter('tip_link').get_parameter_value().string_value
        self.base_link = base_link
        self.tip_link = tip_link

        if not robot_description:
            self.get_logger().error(
                'robot_description parameter is empty; FK will not work.'
            )
            self.chain = None
            self.fk_solver = None
        else:
            # Extract the unique chain between base and tip according to the URDF
            self.chain = build_kdl_chain_from_urdf(
                robot_description, base_link, tip_link
            )
            # PyKDL solver that turns joint angles into end-effector poses
            self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
            self.get_logger().info(
                f'KDL chain built: {self.chain.getNrOfJoints()} joints.'
            )

        # Two demo poses: every column is a waypoint, values are in radians
        self.joint_targets = np.array([
            [4.71, 1.38, -3.21, 1.79, 1.73],
            [1.44, 0.71, -2.51, 1.39, 1.60]
        ], dtype=float).T  # shape (5, 2)

        # Timer #1: every 2 seconds publish the next joint target on /joint_states
        self._timer = None
        self._joint_state_timer = self.create_timer(2.0, self._publish_joint_state_step)
        self._motion_index = 0  # indicates which waypoint is currently active
        # Broadcast a helper end-effector TF for comparing FK results with RViz
        self._tf_broadcaster = TransformBroadcaster(self)
        # Cache checkpoints so late RViz subscribers still receive them
        self._checkpoint_data = None
        self._checkpoint_timer = self.create_timer(1.0, self._republish_markers)
        self._animation_active = False
        self._run_once()

    def _run_once(self):
        """Pre-compute the demo once immediately after startup.

        Steps:
        1. Run FK for every waypoint to obtain end-effector transforms.
        2. Publish the matching markers so RViz shows the checkpoints.
        3. Build and publish the JointTrajectory for the controller.
        4. Enable `_animation_active` so the joint-state timer starts ticking.
        """
        if hasattr(self, '_timer') and self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None

        if self.chain is None or self.fk_solver is None:
            self.get_logger().error('KDL chain not available; aborting trajectory.')
            return

        checkpoints = np.zeros((4, 4, self.joint_targets.shape[1]), dtype=float)
        for idx in range(self.joint_targets.shape[1]):
            checkpoints[:, :, idx] = self.forward_kinematics(
                list(self.joint_targets[:, idx])
            )

        self._checkpoint_data = checkpoints
        self.publish_checkpoints(checkpoints)
        self.publish_trajectory(self.joint_targets)
        self._animation_active = True
        self.get_logger().info('Ready. View RViz for markers and robot motion.')

    def forward_kinematics(self, q_list):
        """Return the 4x4 homogeneous transform of the end-effector for ``q_list``."""
        nj = self.chain.getNrOfJoints()
        if len(q_list) != nj:
            self.get_logger().warn(
                f'Joint vector length {len(q_list)} != chain joints {nj}'
            )

        frame = self._compute_frame(q_list)

        H = np.eye(4)
        R = frame.M
        H[0:3, 0:3] = np.array([
            [R[0, 0], R[0, 1], R[0, 2]],
            [R[1, 0], R[1, 1], R[1, 2]],
            [R[2, 0], R[2, 1], R[2, 2]],
        ])
        p = frame.p
        H[0:3, 3] = [p[0], p[1], p[2]]
        return H

    def _compute_frame(self, q_list):
        """Helper that copies a Python list into a PyKDL JntArray and returns the FK frame."""
        nj = self.chain.getNrOfJoints()
        q = kdl.JntArray(nj)
        for i, qi in enumerate(q_list[:nj]):
            q[i] = qi
        frame = kdl.Frame()
        self.fk_solver.JntToCart(q, frame)
        return frame

    def publish_checkpoints(self, tfs: np.ndarray):
        """Publish sphere markers for every checkpoint in ``tfs`` (shape 4x4xN)."""
        marker_id = 0
        now = self.get_clock().now().to_msg()
        for column in range(tfs.shape[2]):
            marker = Marker()
            marker.id = marker_id
            marker_id += 1
            marker.ns = 'checkpoints'
            marker.header.frame_id = 'base_link'
            marker.header.stamp = now
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.b = 0.0
            marker.color.g = min(1.0, marker_id * 0.05)
            marker.color.r = max(0.0, 1.0 - marker_id * 0.05)
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(tfs[0, -1, column])
            marker.pose.position.y = float(tfs[1, -1, column])
            marker.pose.position.z = float(tfs[2, -1, column])
            self.checkpoint_pub.publish(marker)

    def publish_trajectory(self, joint_targets: np.ndarray):
        """Send the preset waypoints to the trajectory controller."""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        dt = 2.0
        accumulated_time = 10.0
        for column in range(joint_targets.shape[1]):
            point = JointTrajectoryPoint()
            point.positions = list(joint_targets[:, column])
            accumulated_time += dt
            sec = int(accumulated_time)
            nsec = int((accumulated_time - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(point)

        self.traj_pub.publish(traj)
        self.get_logger().info(
            f'Published trajectory with {len(traj.points)} points.'
        )

    def _publish_joint_state_step(self) -> None:
        """Timer callback: every 2 seconds publish the next joint target on /joint_states."""
        if not self._animation_active or self.joint_targets.size == 0:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names + self.gripper_joints
        positions = list(self.joint_targets[:, self._motion_index])
        positions.extend([0.0, 0.0])
        msg.position = positions
        self.joint_pub.publish(msg)

        frame = self._compute_frame(list(self.joint_targets[:, self._motion_index]))
        self._publish_end_effector_tf(frame)

        self._motion_index = (self._motion_index + 1) % self.joint_targets.shape[1]

    def _republish_markers(self) -> None:
        """Re-send cached checkpoints so late RViz subscribers still see them."""
        if self._checkpoint_data is None:
            return
        self.publish_checkpoints(self._checkpoint_data)

    def _publish_end_effector_tf(self, frame: kdl.Frame) -> None:
        """Broadcast the KDL-computed end-effector pose as a TF frame."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_link
        transform.child_frame_id = 'arm_end_effector_kdl'
        transform.transform.translation.x = frame.p[0]
        transform.transform.translation.y = frame.p[1]
        transform.transform.translation.z = frame.p[2]
        qx, qy, qz, qw = frame.M.GetQuaternion()
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = YoubotTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
