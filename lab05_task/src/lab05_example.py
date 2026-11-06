#!/usr/bin/env python3
import numpy as np
from numpy import pi
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import PyKDL as kdl
from urdf_kdl_utils import build_kdl_chain_from_urdf
FIXED_KDL_JOINT = kdl.Joint.Fixed
# Exact four-revolute-joint DH model; the gripper is a separate prismatic joint.
shoulder_angle = np.arctan2(0.128, 0.024)
open_man_dh_parameters = {
    'a': [0., np.hypot(0.024, 0.128), 0.124, 0.0536],
    'alpha': [-pi / 2, 0., 0., pi / 2],
    'd': [0.058, 0., 0., 0.],
    'theta': [0., -shoulder_angle, shoulder_angle, 0.],
}
open_man_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'grip_joint']

class OpenManKinematic:

    def __init__(self, node: Node, kdl_kine_chain, tf_suffix: str):
        self.node = node
        self.kine_chain = kdl_kine_chain
        self.tf_suffix = tf_suffix
        self.joint_names = self._extract_joint_names()
        self.joint_count = len(self.joint_names)
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        self._missing_joint_warned = set()
        self.br = TransformBroadcaster(node)
        self.sub = node.create_subscription(JointState, 'joint_states', self.fkine_wrapper, 10)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kine_chain)

    def forward_kinematics(self, joints_kdl):
        pose_kdl = kdl.Frame()
        error = self.fk_solver.JntToCart(joints_kdl, pose_kdl)
        if error != 0:
            raise RuntimeError(f'KDL FK solver failed with code {error}')
        pose = self.convert_kdl_frame_to_mat(pose_kdl)
        return pose

    def fkine_wrapper(self, joint_msg):
        transform = TransformStamped()
        joints_kdl = self.joint_state_to_kdl_array(joint_msg)
        if joints_kdl is None:
            return
        T0ee = self.forward_kinematics(joints_kdl)
        transform.header.stamp = joint_msg.header.stamp
        transform.header.frame_id = 'link1'
        transform.child_frame_id = self.tf_suffix + '_link'
        transform.transform.translation.x = T0ee[0, 3]
        transform.transform.translation.y = T0ee[1, 3]
        transform.transform.translation.z = T0ee[2, 3]
        transform.transform.rotation = self.rotmat2q(T0ee)
        self.br.sendTransform(transform)

    @staticmethod
    def rotmat2q(T):
        R = np.asarray(T, dtype=float)[:3, :3]
        K = np.array([
            [R[0, 0] - R[1, 1] - R[2, 2], R[0, 1] + R[1, 0], R[0, 2] + R[2, 0], R[2, 1] - R[1, 2]],
            [R[0, 1] + R[1, 0], R[1, 1] - R[0, 0] - R[2, 2], R[1, 2] + R[2, 1], R[0, 2] - R[2, 0]],
            [R[0, 2] + R[2, 0], R[1, 2] + R[2, 1], R[2, 2] - R[0, 0] - R[1, 1], R[1, 0] - R[0, 1]],
            [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1], np.trace(R)],
        ]) / 3.0
        (_, vectors) = np.linalg.eigh(K)
        values = vectors[:, -1]
        if values[3] < 0.0:
            values = -values
        q = Quaternion()
        (q.x, q.y, q.z, q.w) = map(float, values)
        return q

    @staticmethod
    def convert_kdl_frame_to_mat(frame):
        mat = np.identity(4)
        mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([
            [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
            [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
            [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]],
        ])
        return mat

    def joint_state_to_kdl_array(self, joint_msg):
        name_to_position = {name: pos for (name, pos) in zip(joint_msg.name, joint_msg.position)}
        updated = False
        for (joint_name, position) in name_to_position.items():
            if joint_name in self.joint_positions:
                self.joint_positions[joint_name] = position
                updated = True
        if not updated:
            return None
        kdl_array = kdl.JntArray(self.joint_count)
        for (idx, joint_name) in enumerate(self.joint_names):
            if joint_name not in name_to_position and joint_name not in self._missing_joint_warned:
                self.node.get_logger().warning(f'Joint "{joint_name}" missing from JointState message; using last known position.')
                self._missing_joint_warned.add(joint_name)
            kdl_array[idx] = self.joint_positions[joint_name]
        return kdl_array

    def _extract_joint_names(self):
        names = []
        for i in range(self.kine_chain.getNrOfSegments()):
            joint = self.kine_chain.getSegment(i).getJoint()
            if joint.getType() != FIXED_KDL_JOINT:
                names.append(joint.getName())
        return names

def kine_chain_urdf(robot_description: str):
    return build_kdl_chain_from_urdf(robot_description, 'link1', 'grip_link')

def kine_chain_dh(dh_params):
    """Provided worked example: fixed mount, four DH joints, then a sliding finger."""
    chain = kdl.Chain()
    chain.addSegment(kdl.Segment('mount', kdl.Joint('mount'), kdl.Frame(kdl.Vector(.012, 0., .017))))
    for index in range(4):
        joint = kdl.Joint(open_man_joint_names[index], kdl.Vector(), kdl.Vector(0, 0, 1), kdl.Joint.RotAxis)
        chain.addSegment(kdl.Segment(joint, kdl.Frame.DH(
            dh_params['a'][index], dh_params['alpha'][index],
            dh_params['d'][index], dh_params['theta'][index])))
    gripper = kdl.Joint('grip_joint', kdl.Vector(), kdl.Vector(0, -1, 0), kdl.Joint.TransAxis)
    chain.addSegment(kdl.Segment(gripper, kdl.Frame()))
    return chain


class OpenManipulatorKinematicsNode(Node):

    def __init__(self):
        super().__init__('open_kdl_forward_kinematic_node', namespace='lab05_task')
        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        if not robot_description:
            self.get_logger().error('Parameter "robot_description" is empty; unable to build URDF chain.')
            raise RuntimeError('robot_description parameter is required.')
        urdf_chain = kine_chain_urdf(robot_description)
        self.urdf_fk = OpenManKinematic(self, urdf_chain, 'urdf')
        dh_chain = kine_chain_dh(open_man_dh_parameters)
        self.dh_fk = OpenManKinematic(self, dh_chain, 'dh')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OpenManipulatorKinematicsNode()
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
