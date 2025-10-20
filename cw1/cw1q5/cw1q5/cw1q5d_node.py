#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
# Import from the other node file within the same ROS 2 package
from cw1q5.cw1q5b_node import forward_kinematics
from geometry_msgs.msg import TransformStamped, Quaternion

"""
This node subscribes to the /joint_states topic, applies the necessary polarity
and offset corrections to the joint angles, computes the forward kinematics for
each joint frame, and publishes the transformations to TF2 for visualization
in RViz.
"""


# ╔════════════════════════════════════════════════════════════════════════╗
# ║           SOLUTION FOR PART 1: DH PARAMETERS & JOINT OFFSETS           ║
# ╚════════════════════════════════════════════════════════════════════════╝
# DH parameters for the youbot arm
youbot_dh_parameters = {'a': [],
                        'alpha': [],
                        'd': [],
                        'theta': []}

# Joint offsets to align the DH model with the URDF representation
youbot_joint_offsets = []

# Create a new dictionary with the offsets applied to the theta values
youbot_dh_offset_paramters = youbot_dh_parameters.copy()
youbot_dh_offset_paramters['theta'] = [theta + offset for theta, offset in zip(youbot_dh_offset_paramters['theta'], youbot_joint_offsets)]

# Polarity correction for each joint reading
youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]
# ╔════════════════════════════════════════════════════════════════════════╗
# ║                        END OF SOLUTION FOR PART 1                      ║
# ╚════════════════════════════════════════════════════════════════════════╝


def rotmat2q(R):
    """Function for converting a 3x3 Rotation matrix R to quaternion q."""
    q = Quaternion()
    angle = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)

    # Use np.isclose for robust floating point comparison
    if np.isclose(angle, 0.0):
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
    else:
        xr = R[2, 1] - R[1, 2]
        yr = R[0, 2] - R[2, 0]
        zr = R[1, 0] - R[0, 1]
        norm = np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
        x = xr / norm
        y = yr / norm
        z = zr / norm
        q.w = np.cos(angle / 2)
        q.x = x * np.sin(angle / 2)
        q.y = y * np.sin(angle / 2)
        q.z = z * np.sin(angle / 2)

    return q


class ForwardKinematicsOffsetNode(Node):
    def __init__(self):
        super().__init__('forward_kinematic_offset_node')
        
        # Initialize the transform broadcaster
        self.br = TransformBroadcaster(self)
        
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                     PART 3: INITIALIZE ROS 2 SUBSCRIBER                ║
        # ╚════════════════════════════════════════════════════════════════════════╝

        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                              END OF PART 3                             ║
        # ╚════════════════════════════════════════════════════════════════════════╝

    def fkine_wrapper(self, joint_msg):
        """
        Callback function to compute FK and publish transforms.
        """
        assert isinstance(joint_msg, JointState), "Node must subscribe to a topic where JointState messages are published"
        
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                      PART 2: FKINE WRAPPER IMPLEMENTATION              ║
        # ╚════════════════════════════════════════════════════════════════════════╝
        
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                              END OF PART 2                             ║
        # ╚════════════════════════════════════════════════════════════════════════╝


def main(args=None):
    # Standard ROS 2 main function
    rclpy.init(args=args)
    fk_offset_node = ForwardKinematicsOffsetNode()
    rclpy.spin(fk_offset_node)
    
    # Destroy the node explicitly
    fk_offset_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
