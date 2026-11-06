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
youbot_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
# Optional extension: these arbitrary values are not the YouBot DH solution.
############################################################################
# BEGIN STUDENT CODE: dh_comparison_parameters



random_dh_parameters = {
    'a': [0.05, 0.12, -0.08, 0.04, 0.02],
    'alpha': [0.0, -pi / 2, pi / 2, -pi / 2, 0.0],
    'd': [0.18, 0.02, -0.03, 0.16, 0.05],
    'theta': [0.0, pi / 6, -pi / 8, pi / 4, -pi / 3],
}



# END STUDENT CODE: dh_comparison_parameters
############################################################################
assert isinstance(random_dh_parameters, dict) and set(random_dh_parameters) == {'a', 'alpha', 'd', 'theta'}, 'Expected four DH parameter lists'
for _key in ('a', 'alpha', 'd', 'theta'):
    assert np.shape(random_dh_parameters[_key]) == (5,), 'Each DH parameter list must contain five values'
    assert np.isrealobj(random_dh_parameters[_key]) and np.all(np.isfinite(random_dh_parameters[_key])), 'DH values must be finite and real'

class YoubotKDLKinematic:

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
        return self.convert_kdl_frame_to_mat(pose_kdl)

    def fkine_wrapper(self, joint_msg):
        transform = TransformStamped()
        joints_kdl = self.joint_state_to_kdl_array(joint_msg)
        if joints_kdl is None:
            return
        T0ee = self.forward_kinematics(joints_kdl)
        transform.header.stamp = joint_msg.header.stamp
        ############################################################################
        # BEGIN STUDENT CODE: parent_frame



        transform.header.frame_id = 'base_link'



        # END STUDENT CODE: parent_frame
        ############################################################################
        assert isinstance(transform.header.frame_id, str) and transform.header.frame_id
        transform.child_frame_id = self.tf_suffix + '_link'
        transform.transform.translation.x = T0ee[0, 3]
        transform.transform.translation.y = T0ee[1, 3]
        transform.transform.translation.z = T0ee[2, 3]
        transform.transform.rotation = self.rotmat2q(T0ee)
        assert transform.header.frame_id != transform.child_frame_id
        assert T0ee.shape == (4, 4) and np.all(np.isfinite(T0ee))
        q = transform.transform.rotation
        assert np.isclose(np.linalg.norm([q.x, q.y, q.z, q.w]), 1., atol=1e-6)
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
    """Assign a KDL chain from the YouBot base to its end-effector, with five joints."""
    ############################################################################
    # BEGIN STUDENT CODE: urdf_chain



    chain = build_kdl_chain_from_urdf(robot_description, 'base_link', 'arm_link_ee')



    # END STUDENT CODE: urdf_chain
    ############################################################################
    assert isinstance(chain, kdl.Chain) and chain.getNrOfJoints() == 5
    return chain

def kine_chain_dh(dh_params):
    expected = len(youbot_joint_names)
    for key in ('a', 'alpha', 'd', 'theta'):
        if len(dh_params[key]) != expected:
            raise ValueError(f'DH parameter list "{key}" must contain {expected} entries.')
    kine_chain = kdl.Chain()
    for i in range(len(dh_params['a'])):
        joint_name = youbot_joint_names[i]
        joint_axis = kdl.Vector(0, 0, 1)
        joint = kdl.Joint(joint_name, kdl.Vector(), joint_axis, kdl.Joint.RotAxis)
        segment = kdl.Segment(joint, kdl.Frame.DH(dh_params['a'][i], dh_params['alpha'][i], dh_params['d'][i], dh_params['theta'][i]))
        kine_chain.addSegment(segment)
    return kine_chain

class YoubotKDLNode(Node):

    def __init__(self):
        super().__init__('youbot_kdl_forward_kinematic_node', namespace='lab05_task_solution')
        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        if not robot_description:
            self.get_logger().error('Parameter "robot_description" is empty; unable to build URDF chain.')
            raise RuntimeError('robot_description parameter is required.')
        urdf_chain = kine_chain_urdf(robot_description)
        self.urdf_fk = YoubotKDLKinematic(self, urdf_chain, 'urdf')
        dh_chain = kine_chain_dh(random_dh_parameters)
        self.dh_fk = YoubotKDLKinematic(self, dh_chain, 'dh_random')

def main(args=None):
    """Initialize rclpy, create node as YoubotKDLNode(), and spin it."""
    node = None
    try:
        ############################################################################
        # BEGIN STUDENT CODE: main



        rclpy.init(args=args)
        node = YoubotKDLNode()
        rclpy.spin(node)



        # END STUDENT CODE: main
        ############################################################################
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
