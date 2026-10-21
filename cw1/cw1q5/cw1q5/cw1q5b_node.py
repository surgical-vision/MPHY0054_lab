#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion


# Populate the values inside the youbot_dh_parameters dictionary with the ones you found in question 5a.

############################################################################
# BEGIN STUDENT CODE: dh_parameters



youbot_dh_parameters = {'a':[0, 0, 0, 0, 0],
                        'alpha': [0, 0, 0, 0, 0],
                        'd' : [0, 0, 0, 0, 0],
                        'theta' : [0, 0, 0, 0, 0]}



# END STUDENT CODE: dh_parameters
############################################################################


assert isinstance(youbot_dh_parameters, dict) and set(youbot_dh_parameters) == {'a', 'alpha', 'd', 'theta'}, "Expected four DH parameter lists"
for _key in ('a', 'alpha', 'd', 'theta'):
    assert np.shape(youbot_dh_parameters[_key]) == (5,), "Each DH parameter list must contain five values"
    assert np.isrealobj(youbot_dh_parameters[_key]) and np.all(np.isfinite(youbot_dh_parameters[_key])), "DH values must be finite and real"

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

def standard_dh(a, alpha, d, theta):
    """Input: scalar a,d in metres; alpha,theta in radians. Return A, shape (4,4)."""
    assert isinstance(a, (int, float)), "wrong input type for a"
    assert isinstance(alpha, (int, float)), "wrong input type for alpha"
    assert isinstance(d, (int, float)), "wrong input type for d"
    assert isinstance(theta, (int, float)), "wrong input type for theta"
    ############################################################################
    # BEGIN STUDENT CODE: standard_dh



    raise NotImplementedError("CW1 Q5b: assign the standard DH transform to A")



    # END STUDENT CODE: standard_dh
    ############################################################################
    assert isinstance(A, np.ndarray), "A must be a NumPy array"
    assert A.shape == (4, 4), "A has the wrong shape; expected (4, 4)"
    assert np.issubdtype(A.dtype, np.number) and np.isrealobj(A), "A must be real numeric data"
    assert np.all(np.isfinite(A)), "A must contain only finite values"
    assert np.allclose(A[3], [0, 0, 0, 1]), "Invalid homogeneous bottom row"
    assert np.allclose(A[:3, :3].T @ A[:3, :3], np.eye(3), atol=1e-6), "Rotation must be orthonormal"
    assert np.isclose(np.linalg.det(A[:3, :3]), 1.0, atol=1e-6), "Rotation determinant must be +1"

    assert isinstance(A, np.ndarray), "Output must be a NumPy array"
    assert A.shape == (4, 4), "Output must have shape (4,4)"
    return A

def forward_kinematics(dh_dict, joints_readings, up_to_joint=5):
    """Input: DH lists a/alpha/d/theta, five joint angles in rad, frame index 0..5.

    Assign T, a (4,4) NumPy transform from base to the requested frame.
    Frame 0 returns identity. Use your standard_dh implementation.
    """
    assert isinstance(dh_dict, dict)
    assert isinstance(joints_readings, list)
    assert isinstance(up_to_joint, int)
    assert 0 <= up_to_joint <= len(dh_dict['a'])
    ############################################################################
    # BEGIN STUDENT CODE: forward_kinematics



    raise NotImplementedError("CW1 Q5b: assign the forward-kinematic transform to T")



    # END STUDENT CODE: forward_kinematics
    ############################################################################
    assert isinstance(T, np.ndarray), "T must be a NumPy array"
    assert T.shape == (4, 4), "T has the wrong shape; expected (4, 4)"
    assert np.issubdtype(T.dtype, np.number) and np.isrealobj(T), "T must be real numeric data"
    assert np.all(np.isfinite(T)), "T must contain only finite values"
    assert np.allclose(T[3], [0, 0, 0, 1]), "Invalid homogeneous bottom row"
    assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-6), "Rotation must be orthonormal"
    assert np.isclose(np.linalg.det(T[:3, :3]), 1.0, atol=1e-6), "Rotation determinant must be +1"

    assert isinstance(T, np.ndarray), "Output must be a NumPy array"
    assert T.shape == (4, 4), "Output must have shape (4,4)"
    return T


# Q5b visualises DH frames before the URDF corrections in Q5d.
class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematic_node')
        self.br = TransformBroadcaster(self)
        # Input: /joint_states (sensor_msgs/JointState).
        # Store the subscription as self.subscription; callback: self.fkine_wrapper.
        ############################################################################
        # BEGIN STUDENT CODE: joint_state_subscription



        raise NotImplementedError("CW1 Q5b: create the JointState subscription")



        # END STUDENT CODE: joint_state_subscription
        ############################################################################

        assert self.subscription in self.subscriptions, "Store the JointState subscription as self.subscription"
        assert self.subscription.topic_name == '/joint_states', "Subscribe to /joint_states"

    def fkine_wrapper(self, joint_msg):
        """Input: JointState; assign transforms: a list of five TransformStamped messages; return None.

        Select arm_joint_1..arm_joint_5 by name, in that order. Use radians,
        metres, your DH/FK functions and the supplied rotmat2q helper.
        Stamp transforms with this node's clock and use unique child frame names.
        """
        assert isinstance(joint_msg, JointState)
        ############################################################################
        # BEGIN STUDENT CODE: fkine_wrapper



        raise NotImplementedError("CW1 Q5b: assign transforms for the five joint frames")



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
    fk_node = ForwardKinematicsNode()
    rclpy.spin(fk_node)
    fk_node.destroy_node()
    rclpy.shutdown()


############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################

if __name__ == "__main__":
    main()
