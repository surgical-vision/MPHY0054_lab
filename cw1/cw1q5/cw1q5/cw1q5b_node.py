#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

"""
To complete this assignment, you must do the following:
    - Fill the "youbot_dh_parameters" dictionary with the youbot DH parameters.
    - Complete the definition of standard_dh().
    - Complete the definition of forward_kinematics().
    - Complete the definition of fkine_wrapper(). To do this you should use your implementation of standard_dh() and forward_kinematics(). 

Complete the function implementation within the indicated areas and do not modify the
assertions. The assertions are there to make sure that your code will accept and return
data with specific types and formats.

** Important: You can use the ROS 2 launch file to visualize the frames. The frames reported from 
your code are not supposed to match the preloaded youbot model.
Try to move the sliders in the joint_state_publisher_gui and observe your robot's behaviour. If your code 
is correct, when you specify zero angles for all the joints, you should see all the frames 
stacked on the z-axis.
"""

# ╔════════════════════════════════════════════════════════════════════════╗
# ║                          PART 1: DH PARAMETERS                         ║
# ╚════════════════════════════════════════════════════════════════════════╝
# Populate the values inside the youbot_dh_parameters dictionary with the ones you found in question 5a.

youbot_dh_parameters = {'a':[0, 0, 0, 0, 0],
                        'alpha': [0, 0, 0, 0, 0],
                        'd' : [0, 0, 0, 0, 0],
                        'theta' : [0, 0, 0, 0, 0]}


def rotmat2q(R):
    """Function for converting a 3x3 Rotation matrix R to quaternion q."""
    q = Quaternion()
    angle = np.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)

    if (np.isclose(angle, 0.0)):
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

def standard_dh(a, alpha, d, theta):
    """
    This function computes the homogeneous 4x4 transformation matrix T_i based
    on the four standard DH parameters associated with link i and joint i.
    """
    assert isinstance(a, (int, float)), "wrong input type for a"
    assert isinstance(alpha, (int, float)), "wrong input type for alpha"
    assert isinstance(d, (int, float)), "wrong input type for d"
    assert isinstance(theta, (int, float)), "wrong input type for theta"
    A = np.zeros((4, 4))
    # ╔════════════════════════════════════════════════════════════════════════╗
    # ║                    PART 2: COMPLETE THE DH FUNCTION                    ║
    # ╚════════════════════════════════════════════════════════════════════════╝

    # ╔════════════════════════════════════════════════════════════════════════╗
    # ╚════════════════════════════════════════════════════════════════════════╝
    
    assert isinstance(A, np.ndarray), "Output wasn't of type ndarray"
    assert A.shape == (4, 4), "Output had wrong dimensions"
    return A

def forward_kinematics(dh_dict, joints_readings, up_to_joint=5):
    """
    This function solves the forward kinematics by multiplying frame
    transformations up until a specified frame number.
    """
    assert isinstance(dh_dict, dict)
    assert isinstance(joints_readings, list)
    assert isinstance(up_to_joint, int)
    assert 0 <= up_to_joint <= len(dh_dict['a'])
    
    T = np.identity(4)

    # ╔════════════════════════════════════════════════════════════════════════╗
    # ║                PART 3: COMPLETE THE FORWARD KINEMATICS                 ║
    # ╚════════════════════════════════════════════════════════════════════════╝

    # ╔════════════════════════════════════════════════════════════════════════╗
    # ╚════════════════════════════════════════════════════════════════════════╝
    
    assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
    assert T.shape == (4, 4), "Output had wrong dimensions"
    return T


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematic_node')
        
        self.br = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.fkine_wrapper,
            10)
        self.subscription

    def fkine_wrapper(self, joint_msg):
        """
        This function is the callback for the subscriber. It integrates your robotics
        code with ROS 2 and is responsible for computing and publishing the transforms.
        """
        assert isinstance(joint_msg, JointState), "Node must subscribe to a topic where JointState messages are published"

        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                  PART 4: COMPLETE THE ROS 2 WRAPPER                  ║
        # ╚════════════════════════════════════════════════════════════════════════╝
        
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ╚════════════════════════════════════════════════════════════════════════╝


def main(args=None):
    rclpy.init(args=args)
    fk_node = ForwardKinematicsNode()
    rclpy.spin(fk_node)
    fk_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()