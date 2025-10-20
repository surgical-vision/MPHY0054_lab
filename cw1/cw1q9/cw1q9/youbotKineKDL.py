#!/usr/bin/env python3

import numpy as np
import PyKDL
from cw1q9.youbotKineBase import YoubotKinematicBase 
from ament_index_python.packages import get_package_share_directory
import os

class YoubotKinematicKDL(YoubotKinematicBase):
    def __init__(self, tf_suffix='kdl'):
        super().__init__('youbot_kinematic_kdl', tf_suffix)

        # Get path to the URDF file from your ROS 2 package
        pkg_path = get_package_share_directory('cw1q9') 
        urdf_path = os.path.join(pkg_path, 'model.urdf')

        # Directly create the KDL tree from the URDF file
        (ok, self.kine_tree) = PyKDL.Tree.from_urdf_file(urdf_path)

        if not ok:
            raise ValueError(f"Failed to parse URDF file at {urdf_path}")

        # The rest of your initialization remains the same
        self.kine_chain = self.kine_tree.getChain("base_link", "arm_link_ee")
        self.NJoints = self.kine_chain.getNrOfJoints()
        self.current_joint_position = PyKDL.JntArray(self.NJoints)
        
        # KDL solvers
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.kine_chain)        
        self.jac_calc = PyKDL.ChainJntToJacSolver(self.kine_chain)

    def get_jacobian(self, joint):
        joints_kdl = self.list_to_kdl_jnt_array(joint)
        jac_kdl = PyKDL.Jacobian(self.kine_chain.getNrOfJoints())
        self.jac_calc.JntToJac(joints_kdl, jac_kdl)
        jac = self.convert_kdl_to_mat(jac_kdl)
        return jac

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        joints_kdl = self.list_to_kdl_jnt_array(joints_readings)
        pose_kdl = PyKDL.Frame()
        self.fk_solver.JntToCart(joints_kdl, pose_kdl)
        pose = self.convert_kdl_frame_to_mat(pose_kdl)
        return pose

    @staticmethod
    def convert_kdl_frame_to_mat(frame):
        mat = np.identity(4)
        mat[:3, 3] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
        return mat

    @staticmethod
    def convert_kdl_to_mat(data):
        mat = np.zeros((data.rows(), data.columns()))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat

    @staticmethod
    def list_to_kdl_jnt_array(joints):
        kdl_array = PyKDL.JntArray(5)
        for i in range(5):
            kdl_array[i] = joints[i]
        return kdl_array