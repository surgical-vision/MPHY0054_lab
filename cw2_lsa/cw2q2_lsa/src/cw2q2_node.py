#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

# --- ROS 2 Bag Reader Import ---
import sqlite3
import glob
from rclpy.serialization import deserialize_message
# -------------------------------

from youbot_kdl_utils import YoubotKinematicKDL
import itertools

class YoubotTrajectoryPlanning(Node):
    def __init__(self):
        super().__init__('youbot_traj_cw2')
        self.kdl_youbot = YoubotKinematicKDL(self)

        # 1. Trajectory Publisher
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/EffortJointInterface_trajectory_controller/command',
            5
        )

        # 2. Marker Publisher 
        # Topic: visualization_marker (Matches default RViz config you provided)
        marker_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, 
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.checkpoint_pub = self.create_publisher(Marker, 'visualization_marker', marker_qos)
        
        # 3. Joint State Publisher
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self._q_path = None
        self._path_index = 0
        self._publish_timer = None
        
        # Cache for markers
        self._checkpoint_markers = [] 
        self._checkpoint_positions = []
        self._checkpoint_reached = []
        self._marker_timer = None

    def run(self):
        """Runs the main coursework logic."""
        self.get_logger().info('Waiting 2 seconds for everything to load up.')
        time.sleep(2.0)
        traj, q_path = self.q2()
        self._q_path = q_path
        self._path_index = 0
        self.get_logger().info('Markers published. Starting movement in 1 second...')
        time.sleep(1.0)
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.traj_pub.publish(traj)
        self._publish_timer = self.create_timer(0.05, self._publish_next_state)
        self._marker_timer = self.create_timer(1.0, self._republish_markers)

    def q2(self):
        ############################################################################
        # QUESTION E START
        ############################################################################

        raise NotImplementedError("Question 2e: implement q2()")
        ############################################################################
        # QUESTION E END
        ############################################################################

    def load_targets(self):
        """
        Loads the target joint positions from the bagfile.
        """
        ############################################################################
        # QUESTION A START
        ############################################################################

        raise NotImplementedError("Question 2a: implement load_targets()")
        ############################################################################
        # QUESTION A END
        ############################################################################

    def get_shortest_path(self, checkpoints_tf):
        """
        Computes the order of checkpoints.
        """
        ############################################################################
        # QUESTION B START
        ############################################################################

        raise NotImplementedError("Question 2b: implement get_shortest_path()")
        ############################################################################
        # QUESTION B END
        ############################################################################

    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        """
        Create intermediate transformations.
        """
        ############################################################################
        # QUESTION C START
        ############################################################################

        raise NotImplementedError("Question 2c: implement intermediate_tfs()")
        ############################################################################
        # QUESTION C END
        ############################################################################

    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        """
        Interpolate between two transforms.
        """
        ############################################################################
        # QUESTION C START
        ############################################################################

        raise NotImplementedError("Question 2c: implement decoupled_rot_and_trans()")
        ############################################################################
        # QUESTION C END
        ############################################################################

    def full_checkpoints_to_joints(self, full_checkpoint_tfs, init_joint_position):
        """
        Compute associated joint positions.
        """
        ############################################################################
        # QUESTION D START
        ############################################################################

        raise NotImplementedError("Question 2d: implement full_checkpoints_to_joints()")
        ############################################################################
        # QUESTION D END
        ############################################################################

    def ik_position_only(self, pose, q0):
        """
        Iterative Inverse Kinematics.
        """
        ############################################################################
        # QUESTION D START
        ############################################################################

        raise NotImplementedError("Question 2d: implement ik_position_only()")
        ############################################################################
        # QUESTION D END
        ############################################################################

    def init_markers(self, tfs):
        """
        Creates markers ONLY for the 5 checkpoints.
        """
        self._checkpoint_markers = []
        marker_id = 0
        
        for i in range(0, tfs.shape[2]):
            marker = Marker()
            marker.id = marker_id
            marker_id += 1
            marker.header.frame_id = 'base_link' 
            marker.ns = "points_and_lines" 
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.scale.x = 0.04 
            marker.scale.y = 0.04
            marker.scale.z = 0.04
            
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker.lifetime = Duration(sec=0, nanosec=0)
            marker.frame_locked = True
            
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(tfs[0, -1, i])
            marker.pose.position.y = float(tfs[1, -1, i])
            marker.pose.position.z = float(tfs[2, -1, i])
            
            self._checkpoint_markers.append(marker)
        
        self._republish_markers()

    def _republish_markers(self):
        """Timer callback to keep markers visible and update colors."""
        if not self._checkpoint_markers:
            return
        stamp_zero = rclpy.time.Time(seconds=0).to_msg()
            
        for i, marker in enumerate(self._checkpoint_markers):
            marker.header.stamp = stamp_zero
            reached = self._checkpoint_reached[i] if i < len(self._checkpoint_reached) else False
            
            if reached:
                marker.color.r = 0.0
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
            marker.color.b = 0.0
            
            self.checkpoint_pub.publish(marker)

    def _publish_next_state(self):
        """Timer callback to visualize the robot moving."""
        if self._q_path is None:
            return
        if self._path_index >= self._q_path.shape[1]:
            self.destroy_timer(self._publish_timer)
            return
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        msg.position = self._q_path[:, self._path_index].tolist()
        self.joint_state_pub.publish(msg)
        
        ee_pose = self.kdl_youbot.forward_kinematics(self._q_path[:, self._path_index])
        ee_pos = ee_pose[:3, 3]
        
        for idx, cp in enumerate(self._checkpoint_positions):
            if cp is None or len(cp) == 0: continue
            if np.linalg.norm(ee_pos - cp) < 0.03: 
                self._checkpoint_reached[idx] = True
                
        self._path_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = YoubotTrajectoryPlanning()
    node.run()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
