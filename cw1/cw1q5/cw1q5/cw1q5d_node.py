#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from cw1q5.cw1q5b_node import forward_kinematics
from geometry_msgs.msg import TransformStamped, Quaternion



# DH parameters for the youbot arm
############################################################################
# BEGIN STUDENT CODE: urdf_dh_parameters



youbot_dh_parameters = {'a': [],
                        'alpha': [],
                        'd': [],
                        'theta': []}



# END STUDENT CODE: urdf_dh_parameters
############################################################################

assert isinstance(youbot_dh_parameters, dict) and set(youbot_dh_parameters) == {'a', 'alpha', 'd', 'theta'}, "Expected four DH parameter lists"
for _key in ('a', 'alpha', 'd', 'theta'):
    assert np.shape(youbot_dh_parameters[_key]) == (5,), "Each DH parameter list must contain five values"
    assert np.isrealobj(youbot_dh_parameters[_key]) and np.all(np.isfinite(youbot_dh_parameters[_key])), "DH values must be finite and real"

# Joint offsets to align the DH model with the URDF representation
############################################################################
# BEGIN STUDENT CODE: joint_offsets



youbot_joint_offsets = []



# END STUDENT CODE: joint_offsets
############################################################################

assert np.shape(youbot_joint_offsets) == (5,), "Supply five joint offsets in radians"
assert np.isrealobj(youbot_joint_offsets) and np.all(np.isfinite(youbot_joint_offsets)), "Offsets must be finite and real"

youbot_dh_offset_paramters = youbot_dh_parameters.copy()
youbot_dh_offset_paramters['theta'] = [theta + offset for theta, offset in zip(youbot_dh_offset_paramters['theta'], youbot_joint_offsets)]

# Polarity correction for each joint reading
youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]


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


class ForwardKinematicsOffsetNode(Node):
    def __init__(self):
        super().__init__('forward_kinematic_offset_node')
        self.br = TransformBroadcaster(self)
        # Input: /joint_states (JointState); callback: self.fkine_wrapper.
        ############################################################################
        # BEGIN STUDENT CODE: joint_state_subscription



        raise NotImplementedError("CW1 Q5d: create and retain self.subscription")



        # END STUDENT CODE: joint_state_subscription
        ############################################################################
        
        assert self.subscription in self.subscriptions, "Store the JointState subscription as self.subscription"
        assert self.subscription.topic_name == '/joint_states', "Subscribe to /joint_states"



    def fkine_wrapper(self, joint_msg):
        """Input: JointState; assign transforms: a list of five TransformStamped messages; return None.

        Order arm_joint_1..arm_joint_5 by name, apply the supplied polarity
        and your offsets, then use the Q5b FK function and rotmat2q helper.
        Express the standard-DH frames in base_link, including the fixed
        mounting transform. Frame 5 ends at the arm_link_5 flange.
        Use joint_msg.header.stamp for all five transform timestamps.
        DH frames 1-4 have z axes along joints 2-5; their origins and
        orientations need not match the URDF link frames with the same index.
        """
        assert isinstance(joint_msg, JointState)
        ############################################################################
        # BEGIN STUDENT CODE: fkine_wrapper



        raise NotImplementedError("CW1 Q5d: assign transforms for the five corrected joint frames")



        # END STUDENT CODE: fkine_wrapper
        ############################################################################
        
        assert isinstance(transforms, list) and len(transforms) == 5, "Expected five joint-frame transforms"
        assert all(isinstance(item, TransformStamped) for item in transforms), "Expected TransformStamped messages"
        assert len({item.child_frame_id for item in transforms}) == 5, "Use distinct child frames"
        for _item in transforms:
            assert _item.header.frame_id and _item.child_frame_id, "Frame names must not be empty"
            assert _item.header.frame_id != _item.child_frame_id, "Parent and child frames must differ"
            _p, _q = _item.transform.translation, _item.transform.rotation
            assert np.all(np.isfinite([_p.x, _p.y, _p.z, _q.x, _q.y, _q.z, _q.w])), "TF values must be finite"
            assert np.isclose(np.linalg.norm([_q.x, _q.y, _q.z, _q.w]), 1.0, atol=1e-6), "TF quaternion must have unit norm"
        self.br.sendTransform(transforms)
        return None

        


def main(args=None):
    rclpy.init(args=args)
    fk_offset_node = ForwardKinematicsOffsetNode()
    rclpy.spin(fk_offset_node)
    
    fk_offset_node.destroy_node()
    rclpy.shutdown()



############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################

if __name__ == "__main__":
    main()
