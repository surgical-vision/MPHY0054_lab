#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
from lab_support import JOINT_NAMES, Kinematics, checked_array


class YoubotTrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory', namespace='lab07_task')
        self.declare_parameter('robot_description', '')
        self.model = Kinematics(self.get_parameter('robot_description').value, 'arm_link_5')
        self.traj_pub = self.create_publisher(JointTrajectory, 'joint_trajectory',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.checkpoint_pub = self.create_publisher(Marker, 'checkpoint_positions',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._checkpoint_data = None
        hardcoded_joint_targets = [[4.71, 1.38, -3.21, 1.79, 1.73],
                                   [1.44, 0.71, -2.51, 1.39, 1.60]]
        ############################################################################
        # BEGIN STUDENT CODE: load_targets



        raise NotImplementedError("Complete load_targets")



        # END STUDENT CODE: load_targets
        ############################################################################
        checked_array(self.joint_targets, (5, 2))
        assert np.allclose(self.joint_targets, np.asarray(hardcoded_joint_targets).T)
        ############################################################################
        # BEGIN STUDENT CODE: checkpoints



        raise NotImplementedError("Complete checkpoints")



        # END STUDENT CODE: checkpoints
        ############################################################################
        checked_array(checkpoints, (4, 4, 2))
        for i in range(2):
            assert np.allclose(checkpoints[:, :, i], self.model.forward_kinematics(self.joint_targets[:, i]))
        ############################################################################
        # BEGIN STUDENT CODE: publish_checkpoints



        raise NotImplementedError("Complete publish_checkpoints")



        # END STUDENT CODE: publish_checkpoints
        ############################################################################
        assert self._checkpoint_data is not None, 'Call publish_checkpoints with the computed checkpoints'
        self.publish_trajectory(self.joint_targets)
        self.marker_timer = self.create_timer(1.0, lambda: self.publish_checkpoints(self._checkpoint_data))

    def publish_trajectory(self, joint_targets):
        """Assign traj with the five joint names and two waypoints at 12 and 14 seconds."""
        ############################################################################
        # BEGIN STUDENT CODE: trajectory



        raise NotImplementedError("Complete trajectory")



        # END STUDENT CODE: trajectory
        ############################################################################
        assert isinstance(traj, JointTrajectory)
        assert traj.joint_names == JOINT_NAMES and len(traj.points) == 2
        for i, point in enumerate(traj.points):
            assert len(point.positions) == 5 and np.all(np.isfinite(point.positions))
            assert np.allclose(point.positions, joint_targets[:, i])
            assert point.time_from_start.sec == 12 + 2 * i and point.time_from_start.nanosec == 0
        self.traj_pub.publish(traj)
        self.get_logger().info('Published two trajectory waypoints at 12 s and 14 s.')

    def publish_checkpoints(self, transforms):
        checked_array(transforms, (4, 4, 2))
        self._checkpoint_data = transforms.copy()
        for i in range(transforms.shape[2]):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id = 'checkpoints', i
            marker.type, marker.action = Marker.SPHERE, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, transforms[:3, 3, i])
            marker.scale.x = marker.scale.y = marker.scale.z = 0.025
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0-i, float(i), 0.2, 1.0)
            self.checkpoint_pub.publish(marker)



############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoubotTrajectoryNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
