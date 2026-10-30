#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import pi
import numpy as np
from geometry_msgs.msg import Quaternion

############################################################################
# BEGIN STUDENT CODE: ros_imports



from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped



# END STUDENT CODE: ros_imports
############################################################################

# Supplied illustrative DH model; only frame 1 is used in this lab.
a = [0.012, 0.13, 0.124, 0.07]
alpha = [pi / 2, 0.0, 0.0, 0.0]
d = [0.075, 0.0, 0.0, 0.0]
theta = [0.0, -1.385, 1.385, 0.0]

############################################################################
# BEGIN STUDENT CODE: frame_names



name_link = ['fkine_link_1']



# END STUDENT CODE: frame_names
############################################################################
assert isinstance(name_link, list) and len(name_link) == 1
assert isinstance(name_link[0], str) and name_link[0] and name_link[0] not in ('world', 'link1', 'link2')


def forward_kinematics(a1, alpha1, d1, theta1):
    """Provided standard-DH transform for the single frame used in this lab."""
    c, s = np.cos(theta1), np.sin(theta1)
    ca, sa = np.cos(alpha1), np.sin(alpha1)
    return np.array([[c, -s * ca, s * sa, a1 * c],
                     [s, c * ca, -c * sa, a1 * s],
                     [0., sa, ca, d1], [0., 0., 0., 1.]])


def rotmat2q(R):
    """Provided helper: rotation block to ROS quaternion, robust at 0 and pi."""
    R = np.asarray(R, dtype=float)[:3, :3]
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


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics', namespace='lab04_task_solution')
        ############################################################################
        # BEGIN STUDENT CODE: broadcaster



        self.br = TransformBroadcaster(self)



        # END STUDENT CODE: broadcaster
        ############################################################################
        assert isinstance(self.br, TransformBroadcaster)
        ############################################################################
        # BEGIN STUDENT CODE: subscription



        self.subscription = self.create_subscription(JointState, 'joint_states', self.fkine_wrapper, 10)



        # END STUDENT CODE: subscription
        ############################################################################
        assert self.subscription in self.subscriptions
        assert self.subscription.topic_name == self.get_namespace().rstrip('/') + '/joint_states'

    def fkine_wrapper(self, joint_msg):
        """Assign one TransformStamped using joint1 by name and the supplied DH model.

        Use world as the parent, name_link[0] as the child, and the JointState
        timestamp. Select joint1 by name; other joints may be present or reordered.
        """
        assert isinstance(joint_msg, JointState)
        if 'joint1' not in joint_msg.name:
            return
        ############################################################################
        # BEGIN STUDENT CODE: transform_message



        index = joint_msg.name.index('joint1')
        T01 = forward_kinematics(a[0], alpha[0], d[0], theta[0] + joint_msg.position[index])
        transform = TransformStamped()
        transform.header.stamp = joint_msg.header.stamp
        transform.header.frame_id = 'world'
        transform.child_frame_id = name_link[0]
        transform.transform.translation.x = float(T01[0, 3])
        transform.transform.translation.y = float(T01[1, 3])
        transform.transform.translation.z = float(T01[2, 3])
        transform.transform.rotation = rotmat2q(T01)



        # END STUDENT CODE: transform_message
        ############################################################################
        assert isinstance(transform, TransformStamped)
        assert transform.header.frame_id == 'world'
        assert transform.child_frame_id == name_link[0]
        assert transform.header.stamp == joint_msg.header.stamp
        p, q = transform.transform.translation, transform.transform.rotation
        assert np.all(np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]))
        assert np.isclose(np.linalg.norm([q.x, q.y, q.z, q.w]), 1., atol=1e-6)
        ############################################################################
        # BEGIN STUDENT CODE: broadcast_transform



        self.br.sendTransform(transform)



        # END STUDENT CODE: broadcast_transform
        ############################################################################
        return None


############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ForwardKinematicsNode()
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
