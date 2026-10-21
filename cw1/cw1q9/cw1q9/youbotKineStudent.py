#!/usr/bin/env python3
import numpy as np
from cw1q9.youbotKineBase import YoubotKinematicBase
import rclpy
from cw1q5.cw1q5b_node import forward_kinematics as q5_forward_kinematics

class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super().__init__('youbot_kinematic_student', tf_suffix='student')
        # Load your Q5d DH parameters; keep Q9's independent offset input below.
        from cw1q5.cw1q5d_node import youbot_dh_parameters
        self.dh_params = {key: list(values) for key, values in youbot_dh_parameters.items()}
        # Fixed URDF transform from the Q5d flange to Q9's arm_link_ee.
        self.flange_to_end_effector = np.eye(4)
        self.flange_to_end_effector[2, 3] = 0.055
        # Replace the placeholder offsets with your CW1 Q5 joint offsets.
        ############################################################################
        # BEGIN STUDENT CODE: joint_offsets



        youbot_joint_offsets = []



        # END STUDENT CODE: joint_offsets
        ############################################################################
        assert np.shape(youbot_joint_offsets) == (5,), "Supply five joint offsets"
        assert np.isrealobj(youbot_joint_offsets) and np.all(np.isfinite(youbot_joint_offsets)), "Offsets must be finite and real"

        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        """Reuse Q5b FK and Q5d parameters; frame 5 ends at arm_link_ee."""
        readings = [sign * angle for sign, angle in
                    zip(self.youbot_joint_readings_polarity, joints_readings)]
        T = q5_forward_kinematics(self.dh_params, readings, up_to_joint)
        if up_to_joint == 5:
            T = T @ self.flange_to_end_effector
        return T

    def get_jacobian(self, joint):
        """CW1 Q9a. Input: list of five URDF joint readings in radians.

        Assign jacobian: NumPy (6,5), linear rows first (m/rad), angular rows
        last, expressed in base_link. Account for the supplied joint polarity.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5
        ############################################################################
        # BEGIN STUDENT CODE: get_jacobian



        raise NotImplementedError("CW1 Q9a: assign jacobian")



        # END STUDENT CODE: get_jacobian
        ############################################################################
        assert isinstance(jacobian, np.ndarray), "jacobian must be a NumPy array"
        assert jacobian.shape == (6, 5), "jacobian has the wrong shape; expected (6, 5)"
        assert np.issubdtype(jacobian.dtype, np.number) and np.isrealobj(jacobian), "jacobian must be real numeric data"
        assert np.all(np.isfinite(jacobian)), "jacobian must contain only finite values"

        assert isinstance(jacobian, np.ndarray)
        assert jacobian.shape == (6, 5)
        return jacobian

    def check_singularity(self, joint):
        """CW1 Q9c. Input: five joint readings (rad); assign a Python bool.

        Return True when the Jacobian loses rank relative to its attainable
        rank for this five-joint robot. Explain your numerical tolerance.
        """
        assert isinstance(joint, list)
        assert len(joint) == 5
        ############################################################################
        # BEGIN STUDENT CODE: check_singularity



        raise NotImplementedError("CW1 Q9c: assign singularity")



        # END STUDENT CODE: check_singularity
        ############################################################################
        assert isinstance(singularity, bool)
        return singularity

def main(args=None):
    rclpy.init(args=args)
    node = YoubotKinematicStudent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################

if __name__ == '__main__':
    main()
