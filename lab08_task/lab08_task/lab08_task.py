#!/usr/bin/env python3
"""
lab08_task: Jacobian-transpose IK exercise (ROS 2).

Students fill the TODOs to make the IK solve and visualise:
- Implement the position-only Jacobian-transpose loop in ``ik_position_only``.
- Make sure the markers show the desired pose and the IK solution.
Everything else is provided (URDF->KDL, FK/Jacobian, launch, RViz).
"""

from typing import Tuple

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

import PyKDL as kdl

from .src.urdf_kdl_utils import build_kdl_chain_from_urdf


class YoubotIkTaskNode(Node):
    """Skeleton node for the Jacobian-transpose IK assignment."""

    def __init__(self):
        super().__init__('lab08_youbot_ik_task')

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
                'robot_description parameter is empty; IK demo will not run.'
            )
            return

        self.chain = build_kdl_chain_from_urdf(robot_description, base_link, tip_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

        # Publishers for RViz visualisation
        marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.checkpoint_pub = self.create_publisher(Marker, 'checkpoint_positions', marker_qos)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._tf_broadcaster = TransformBroadcaster(self)


        # IK inputs: target joint angles and initial guess
        self.hardcoded_joint_target = np.array([4.71, 1.38, -3.21, 1.79, 1.73], dtype=float)
        
        
         # TODO: Use an initial joint guess q for the upcoming iterations; if you’re unsure, zero-initialize it. 
        self.q0 = None

        self.desired_pose = self.forward_kinematics(self.hardcoded_joint_target)
        if self.q0 is None:
            raise NotImplementedError(
                'Set self.q0 to an initial guess before running IK (e.g., np.zeros_like(hardcoded_joint_target)).'
            )
        self.ik_solution, self.ik_error, self.ik_steps = self.ik_position_only(
            self.desired_pose, self.q0
        )
        self.get_logger().info(
            f'IK finished in {self.ik_steps} steps, position error={self.ik_error:.6f} m, '
            f'q_sol={self.ik_solution}'
        )
        self.get_logger().info(
            f'Original target q={self.hardcoded_joint_target}, IK q={self.ik_solution}'
        )

        self._target_frame = self._compute_frame(self.hardcoded_joint_target.tolist())
        self._solution_frame = self._compute_frame(self.ik_solution.tolist())

        self.publish_markers()
        self._checkpoint_timer = self.create_timer(1.0, self._republish_markers)
        self._timer = self.create_timer(0.5, self._publish_visualisation)

    def ik_position_only(
        self,
        pose: np.ndarray,
        q0: np.ndarray,
        alpha: float = None,
        max_iter: int = None,
        tol: float = None
    ) -> Tuple[np.ndarray, float, int]:
        """TODO: implement position-only Jacobian-transpose IK.

        Detailed steps (position error only, roll/pitch/yaw ignored):
        1) Extract desired position: ``target_pos = pose[:3, 3].ravel()``.
        2) Initialise ``q = np.array(q0, dtype=float)`` and a step counter.
        3) Loop up to ``max_iter``:
           a) Compute current FK: ``frame = self._compute_frame(q.tolist())`` and ``current_pos = [frame.p[0], frame.p[1], frame.p[2]]``.
           b) Position error: ``error_vec = target_pos - current_pos``; norm ``err_norm = np.linalg.norm(error_vec)``.
           c) Convergence: if ``err_norm < tol`` break.
           d) Jacobian (position rows only): ``J = self._compute_jacobian(q.tolist())[:3, :]``.
           e) Update (Jacobian transpose rule): ``q += alpha * J.T.dot(error_vec)`` implements q_{k+1} = q_k + α J^T (p* - p(q_k)).
        4) Return ``q, err_norm, steps``.
        """
        # TODO: set alpha, max_iter, tol before running IK (students choose these hyperparameters).
        if alpha is None or max_iter is None or tol is None:
            raise NotImplementedError('Set alpha, max_iter, tol before calling ik_position_only.')
        # TODO: implement the loop above and return (q, err_norm, steps).
        raise NotImplementedError('Fill in ik_position_only following the detailed steps.')

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

    def _compute_jacobian(self, q_list) -> np.ndarray:
        """Return the geometric Jacobian for the supplied joint configuration."""
        nj = self.chain.getNrOfJoints()
        q = kdl.JntArray(nj)
        for i, qi in enumerate(q_list[:nj]):
            q[i] = qi
        jacobian_kdl = kdl.Jacobian(nj)
        self.jac_solver.JntToJac(q, jacobian_kdl)
        jac = np.zeros((jacobian_kdl.rows(), jacobian_kdl.columns()), dtype=float)
        for row in range(jacobian_kdl.rows()):
            for col in range(jacobian_kdl.columns()):
                jac[row, col] = jacobian_kdl[row, col]
        return jac

    def publish_markers(self) -> None:
        """Publish markers for the desired pose and the IK solution."""
        now = self.get_clock().now().to_msg()
        self._checkpoint_data = []
        marker_desired = self._make_marker(
            marker_id=0,
            stamp=now,
            position=self.desired_pose[:3, 3],
            color=(0.1, 0.8, 0.2, 0.9),
            text='desired'
        )
        marker_solution = self._make_marker(
            marker_id=1,
            stamp=now,
            position=self.forward_kinematics(self.ik_solution)[:3, 3],
            color=(0.2, 0.4, 0.9, 0.9),
            text='ik_solution'
        )
        self._checkpoint_data = [marker_desired, marker_solution]
        for marker in self._checkpoint_data:
            self.checkpoint_pub.publish(marker)

    def _make_marker(self, marker_id, stamp, position, color, text) -> Marker:
        marker = Marker()
        marker.id = marker_id
        marker.ns = 'lab08_ik'
        marker.header.frame_id = self.base_link
        marker.header.stamp = stamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.text = text
        return marker

    def _republish_markers(self) -> None:
        """Re-send cached markers so late RViz subscribers see them."""
        if not getattr(self, '_checkpoint_data', None):
            return
        for marker in self._checkpoint_data:
            marker.header.stamp = self.get_clock().now().to_msg()
            self.checkpoint_pub.publish(marker)

    def _publish_visualisation(self) -> None:
        """Timer callback: publish joint states and TFs for RViz."""
        now = self.get_clock().now().to_msg()
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.name = [
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'arm_joint_4',
            'arm_joint_5',
            'gripper_finger_joint_l',
            'gripper_finger_joint_r',
        ]
        positions = list(self.ik_solution)
        positions.extend([0.0, 0.0])
        joint_msg.position = positions
        self.joint_pub.publish(joint_msg)
        self._broadcast_frame(self._target_frame, 'arm_end_effector_desired', now)
        self._broadcast_frame(self._solution_frame, 'arm_end_effector_ik', now)

    def _broadcast_frame(self, frame: kdl.Frame, child_frame_id: str, stamp) -> None:
        """Broadcast a frame as a TF transform."""
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.base_link
        transform.child_frame_id = child_frame_id
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
    node = YoubotIkTaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
