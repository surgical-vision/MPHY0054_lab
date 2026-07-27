#!/usr/bin/env python3

import numpy as np
import PyKDL
from ament_index_python.packages import get_package_share_directory
from cw2q4_lsa.iiwa14DynBase import Iiwa14DynamicBase
from cw2q4_lsa.urdf_kdl_utils import build_kdl_chain_from_urdf


class Iiwa14DynamicStudent(Iiwa14DynamicBase):
    def __init__(self):
        super(Iiwa14DynamicStudent, self).__init__(tf_suffix='student')
        urdf_path = get_package_share_directory('cw2q4_lsa') + '/model.urdf'
        with open(urdf_path, 'r', encoding='utf-8') as f:
            robot_description = f.read()
        self.kine_chain = build_kdl_chain_from_urdf(robot_description, "iiwa_link_0", "iiwa_link_ee")
        self.NJoints = self.kine_chain.getNrOfJoints()
        self.jac_calc = PyKDL.ChainJntToJacSolver(self.kine_chain)
        self.dyn_solver = PyKDL.ChainDynParam(self.kine_chain, PyKDL.Vector(0, 0, -self.g))

    def forward_kinematics(self, joints_readings, up_to_joint=7):
        """Compute forward kinematics up to the selected joint."""
        assert isinstance(joints_readings, list), "joint readings of type " + str(type(joints_readings))
        assert isinstance(up_to_joint, int)
        ############################################################################
        # QUESTION 4 START: implement FK only inside this file
        ############################################################################
        T = np.identity(4)
        T[2, 3] = 0.1575  # base offset

        raise NotImplementedError("Question 4: implement forward_kinematics()")
        ############################################################################
        # QUESTION 4 END
        ############################################################################

    def get_jacobian_centre_of_mass(self, joint_readings, up_to_joint=7):
        """Compute the Jacobian matrix at the centre of mass."""
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        ############################################################################
        # QUESTION 4a START: implement inside this function
        ############################################################################
        # jacobian = np.zeros((6, 7))

        raise NotImplementedError("Question 4a: implement get_jacobian_centre_of_mass()")
        ############################################################################
        # QUESTION 4a END
        ############################################################################

    def forward_kinematics_centre_of_mass(self, joints_readings, up_to_joint=7):
        """This function computes the forward kinematics up to the centre of mass for the given joint frame.
        Reference - Lecture 9 slide 14.
        Args:
            joints_readings (list): the state of the robot joints.
            up_to_joint (int, optional): Specify up to what frame you want to compute forward kinematicks.
                Defaults to 5.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix describing the pose of frame_{up_to_joint} for the
            centre of mass w.r.t the base of the robot.
        """
        T= np.identity(4)
        T[2, 3] = 0.1575

        T = self.forward_kinematics(joints_readings, up_to_joint-1)
        T = T.dot(self.T_rotationZ(joints_readings[up_to_joint-1]))
        T = T.dot(self.T_translation(self.link_cm[up_to_joint-1, :]))

        return T

    def get_B(self, joint_readings):
        """Given the joint positions of the robot, compute inertia matrix B.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            B (numpy.ndarray): The output is a numpy 7*7 matrix describing the inertia matrix B.
        """
        ############################################################################
        # QUESTION 4b START: implement inside this function
        ############################################################################
        # B = np.zeros((7, 7))

        raise NotImplementedError("Question 4b: implement get_B()")
        ############################################################################
        # QUESTION 4b END
        ############################################################################

    def get_C_times_qdot(self, joint_readings, joint_velocities):
        """Given the joint positions and velocities of the robot, compute Coriolis terms C.
        Args:
            joint_readings (list): The positions of the robot joints.
            joint_velocities (list): The velocities of the robot joints.

        Returns:
            C (numpy.ndarray): The output is a numpy 7*1 matrix describing the Coriolis terms C times joint velocities.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        assert isinstance(joint_velocities, list)
        assert len(joint_velocities) == 7
        ############################################################################
        # QUESTION 4c START: implement inside this function
        ############################################################################
        # C = np.zeros(7)

        raise NotImplementedError("Question 4c: implement get_C_times_qdot()")
        ############################################################################
        # QUESTION 4c END
        ############################################################################

    def get_G(self, joint_readings):
        """Given the joint positions of the robot, compute the gravity matrix g.
        Args:
            joint_readings (list): The positions of the robot joints.

        Returns:
            G (numpy.ndarray): The output is a numpy 7*1 numpy array describing the gravity matrix g.
        """
        assert isinstance(joint_readings, list)
        assert len(joint_readings) == 7
        ############################################################################
        # QUESTION 4d START: implement inside this function
        ############################################################################
        # g = np.zeros(7)

        raise NotImplementedError("Question 4d: implement get_G()")
        ############################################################################
        # QUESTION 4d END
        ############################################################################
