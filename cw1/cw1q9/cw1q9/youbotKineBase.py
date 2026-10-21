import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory
from cw1q5.cw1q5b_node import standard_dh as q5_standard_dh

class YoubotKinematicBase(Node):
    def __init__(self, node_name, tf_suffix=''):
        super().__init__(node_name)
        # Robot variables
        self.tf_suffix = tf_suffix
        self.dh_params = {}

        self.current_joint_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_limit_min = np.array([-169 * np.pi / 180, -65 * np.pi / 180, -150 * np.pi / 180,
                                         -102.5 * np.pi / 180, -167.5 * np.pi / 180])
        self.joint_limit_max = np.array([169 * np.pi / 180, 90 * np.pi / 180, 146 * np.pi / 180,
                                         102.5 * np.pi / 180, 167.5 * np.pi / 180])

        # ROS 2 related
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.traj_publisher = self.create_publisher(JointTrajectory, '/EffortJointInterface_trajectory_controller/joint_trajectory', 10)
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
        """Reuse your Q5b implementation."""
        return q5_standard_dh(a, alpha, d, theta)


    @staticmethod
    def rotmat2q(T):
        """Provided helper: rotation block to ROS quaternion, robust at 0 and pi."""
        R = np.asarray(T, dtype=float)[:3, :3]
        K = np.array([
            [R[0,0]-R[1,1]-R[2,2], R[0,1]+R[1,0], R[0,2]+R[2,0], R[2,1]-R[1,2]],
            [R[0,1]+R[1,0], R[1,1]-R[0,0]-R[2,2], R[1,2]+R[2,1], R[0,2]-R[2,0]],
            [R[0,2]+R[2,0], R[1,2]+R[2,1], R[2,2]-R[0,0]-R[1,1], R[1,0]-R[0,1]],
            [R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1], np.trace(R)],
        ]) / 3.0
        _, vectors = np.linalg.eigh(K)
        values = vectors[:, -1]
        if values[3] < 0.0:
            values = -values
        q = Quaternion()
        q.x, q.y, q.z, q.w = map(float, values)
        return q
