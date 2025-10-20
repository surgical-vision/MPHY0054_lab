#!/usr/bin/env python3

import numpy as np
from cw1q9.youbotKineBase import YoubotKinematicBase
import rclpy

class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super().__init__('youbot_kinematic_student', tf_suffix='student')
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                   FILL IN the JOINT OFFSETS FOUND IN CW1Q5             ║
        # ╚════════════════════════════════════════════════════════════════════════╝
        # Currently a set of dummy Joint Offsets for YOUR testing
        youbot_joint_offsets = [0.0,
                                np.pi / 2,
                                0.0,
                                -np.pi / 2,
                                0.0]
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ╚════════════════════════════════════════════════════════════════════════╝
        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        T = np.identity(4)
        
        joints_readings = [sign * angle for sign, angle in zip(self.youbot_joint_readings_polarity, joints_readings)]

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)
            
        return T

    def get_jacobian(self, joint):
        """Given the joint values of the robot, compute the Jacobian matrix. Coursework 1 Question 9a.
        Reference - Lecture 5 slide 24.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            Jacobian (numpy.ndarray): NumPy matrix of size 6x5 which is the Jacobian matrix.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5

        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                  YOUR CODE STARTS HERE: CALCULATE JACOBIAN             ║
        # ╚════════════════════════════════════════════════════════════════════════╝
        # For your solution to match the KDL Jacobian, z0 needs to be set [0, 0, -1] instead of [0, 0, 1], since that is how its defined in the URDF.
        # Both are correct.

            
        # Your code ends here ------------------------------
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ╚════════════════════════════════════════════════════════════════════════╝
        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """Check for singularity condition given robot joints. Coursework 1 Question 9c.
        Reference Lecture 5 slide 30.

        Args:
            joint (list): the state of the robot joints. In a youbot those are revolute

        Returns:
            singularity (bool): True if in singularity and False if not in singularity.

        """
        assert isinstance(joint, list)
        assert len(joint) == 5
        
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                  YOUR CODE STARTS HERE: CHECK SINGULARITY              ║
        # ╚════════════════════════════════════════════════════════════════════════╝

        # Your code ends here ------------------------------
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ╚════════════════════════════════════════════════════════════════════════╝
        assert isinstance(singularity, bool)
        return singularity

def main(args=None):
    rclpy.init(args=args)
    node = YoubotKinematicStudent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()