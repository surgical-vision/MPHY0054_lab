import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory
import math

class YoubotKinematicBase(Node):
    def __init__(self, node_name, tf_suffix=''):
        super().__init__(node_name)
        # Robot variables
        self.tf_suffix = tf_suffix
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ║                    FILL IN the DH PARAMETER FOUND IN CW1Q5             ║
        # ╚════════════════════════════════════════════════════════════════════════╝
        # Currently a set of dummy DH parameters for testing
        youbot_dh_parameters = {'a': [0, 0.25, 0.20, 0, 0],
                                'alpha': [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2],
                                'd': [0.3, 0, 0, 0, 0.1],
                                'theta': [0, 0, 0, 0, 0]}
        # ╔════════════════════════════════════════════════════════════════════════╗
        # ╚════════════════════════════════════════════════════════════════════════╝
        self.dh_params = youbot_dh_parameters.copy()

        self.current_joint_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_limit_min = np.array([-169 * np.pi / 180, -65 * np.pi / 180, -150 * np.pi / 180,
                                         -102.5 * np.pi / 180, -167.5 * np.pi / 180])
        self.joint_limit_max = np.array([169 * np.pi / 180, 90 * np.pi / 180, 146 * np.pi / 180,
                                         102.5 * np.pi / 180, 167.5 * np.pi / 180])

        # ROS 2 related
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.traj_publisher = self.create_publisher(JointTrajectory, '/EffortJointInterface_trajectory_controller/command', 10)
        self.pose_broadcaster = tf2_ros.TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        self.current_joint_position = list(msg.position)
        current_pose = self.forward_kinematics(self.current_joint_position)
        self.broadcast_pose(current_pose)

    def broadcast_pose(self, pose):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'arm_end_effector_' + self.tf_suffix

        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = self.rotmat2q(pose)

        self.pose_broadcaster.sendTransform(transform)

    def forward_kinematics(self, joint_readings, up_to_joint=5):
        raise NotImplementedError

    def get_jacobian(self, joint):
        raise NotImplementedError

    @staticmethod
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
        # ║                     THE SAME DH FUNCTION AS CW1Q5b                     ║
        # ╚════════════════════════════════════════════════════════════════════════╝

        # ╔════════════════════════════════════════════════════════════════════════╗
        # ╚════════════════════════════════════════════════════════════════════════╝
        
        assert isinstance(A, np.ndarray), "Output wasn't of type ndarray"
        assert A.shape == (4, 4), "Output had wrong dimensions"
        return A

    def rotmat2rodrigues(self, T):
        p = np.empty(6, float)
        q = self.rotmat2q(T)

        if q.w == 1 or q.w == -1:
            rx, ry, rz = 0, 0, 0
        else:
            theta = math.acos(q.w) * 2
            ux = q.x / math.sqrt(1 - q.w ** 2)
            uy = q.y / math.sqrt(1 - q.w ** 2)
            uz = q.z / math.sqrt(1 - q.w ** 2)
            rx, ry, rz = ux * theta, uy * theta, uz * theta
        
        p[3:6] = rx, ry, rz
        p[0:3] = T[0, 3], T[1, 3], T[2, 3]
        return p

    @staticmethod
    def rotmat2q(T):
        q = Quaternion()
        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        xr = T[2, 1] - T[1, 2]
        yr = T[0, 2] - T[2, 0]
        zr = T[1, 0] - T[0, 1]

        norm = np.sqrt(xr**2 + yr**2 + zr**2)
        x = xr / norm
        y = yr / norm
        z = zr / norm

        q.w = np.cos(angle / 2)
        q.x = x * np.sin(angle / 2)
        q.y = y * np.sin(angle / 2)
        q.z = z * np.sin(angle / 2)
        return q