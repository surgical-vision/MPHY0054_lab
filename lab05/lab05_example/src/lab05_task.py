#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

import PyKDL as kdl

from urdf_kdl_utils import build_kdl_chain_from_urdf

FIXED_KDL_JOINT = getattr(kdl.Joint, 'None')


# The the YoubotKDLKinematic class provides the functionality to broadcast transforms using KDL.
# First, fill in the missing code for to perform forward kinematics with KDL and the robot description URDF.
# Second, test your DH parameters similarly as given in lab05_example.py
# These might not align perfectly - the figure given in the coursework is not a perfect representation.
class YoubotKDLKinematic:
    def __init__(self, node: Node, kdl_kine_chain, tf_suffix: str):
        # Set class inputs
        self.node = node
        self.kine_chain = kdl_kine_chain
        self.tf_suffix = tf_suffix
        self.joint_names = self._extract_joint_names()
        self.joint_count = len(self.joint_names)
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        self._missing_joint_warned = set()
        # Setup transform broadcaster and joint states subscriber
        self.br = TransformBroadcaster(node)
        self.sub = node.create_subscription(JointState, '/joint_states', self.fkine_wrapper, 10)

        # FK KDL solver
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kine_chain)

    # This function takes in joint readings, converts to appropriate KDL types
    # then calls the FK solver to get the transformation
    def forward_kinematics(self, joints_kdl):
        pose_kdl = kdl.Frame()
        self.fk_solver.JntToCart(joints_kdl, pose_kdl)
        # Convert KDL Pose to array
        pose = self.convert_kdl_frame_to_mat(pose_kdl)
        return pose

    # This function is a callback for the subscriber for joint_states.
    # The transformation is computed with forward kinematics then the transform is broadcasted.
    def fkine_wrapper(self, joint_msg):
        transform = TransformStamped()

        joints_kdl = self.joint_state_to_kdl_array(joint_msg)
        if joints_kdl is None:
            return

        T0ee = self.forward_kinematics(joints_kdl)

        # Here we define the transform from joint frame 0 to joint frame 1 using the tf2 broadcaster.
        # define the transform timestamp
        transform.header.stamp = self.node.get_clock().now().to_msg()

        ## Change the frame_id here appropriately depending on your kinematic chain. ##
        # define the transform header frame (parent frame)
        transform.header.frame_id = 'base_link'
        ## Change the frame_id here appropriately depending on your kinematic chain. ##

        # define the transform child frame
        transform.child_frame_id = self.tf_suffix + '_link'
        # populate the transform field. It consists of translation and rotation.
        transform.transform.translation.x = T0ee[0, 3]
        transform.transform.translation.y = T0ee[1, 3]
        transform.transform.translation.z = T0ee[2, 3]
        transform.transform.rotation = self.rotmat2q(T0ee)

        # broadcast the transform to tf2
        self.br.sendTransform(transform)

    @staticmethod
    def rotmat2q(T):
        # function that transforms a rotation matrix to quaternion representation
        q = Quaternion()
        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        if (angle == 0):
            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
        else:
            xr = T[2, 1] - T[1, 2]
            yr = T[0, 2] - T[2, 0]
            zr = T[1, 0] - T[0, 1]

            x = xr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            y = yr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            z = zr / np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))

            q.w = float(np.cos(angle / 2))
            q.x = float(x * np.sin(angle / 2))
            q.y = float(y * np.sin(angle / 2))
            q.z = float(z * np.sin(angle / 2))
        return q

    @staticmethod
    def convert_kdl_frame_to_mat(frame):
        # Helper function to convert kdl frames to numpy arrays
        mat = np.identity(4)
        mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
        return mat

    def joint_state_to_kdl_array(self, joint_msg):
        name_to_position = {name: pos for name, pos in zip(joint_msg.name, joint_msg.position)}
        updated = False
        for joint_name, position in name_to_position.items():
            if joint_name in self.joint_positions:
                self.joint_positions[joint_name] = position
                updated = True

        if not updated:
            return None

        kdl_array = kdl.JntArray(self.joint_count)
        for idx, joint_name in enumerate(self.joint_names):
            if joint_name not in name_to_position and joint_name not in self._missing_joint_warned:
                self.node.get_logger().warning(
                    f'Joint "{joint_name}" missing from JointState message; using last known position.'
                )
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


# Create a KDL kinematic chain with the URDF files (parameter from robot description).
def kine_chain_urdf(robot_description: str):
    return build_kdl_chain_from_urdf(robot_description, 'base_link', 'arm_link_ee')


class YoubotKDLNode(Node):
    def __init__(self):
        super().__init__('youbot_kdl_forward_kinematic_node')
        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        if not robot_description:
            self.get_logger().error('Parameter "robot_description" is empty; unable to build URDF chain.')
            raise RuntimeError('robot_description parameter is required.')

        urdf_chain = kine_chain_urdf(robot_description)
        self.urdf_fk = YoubotKDLKinematic(self, urdf_chain, 'urdf')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoubotKDLNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
