import numpy as np
import PyKDL as kdl
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from urdf_kdl_utils import build_kdl_chain_from_urdf

JOINT_NAMES = [f'arm_joint_{i}' for i in range(1, 6)]


def checked_array(value, shape):
    assert isinstance(value, np.ndarray) and value.shape == shape, f'Expected NumPy shape {shape}'
    assert np.issubdtype(value.dtype, np.number) and np.isrealobj(value)
    assert np.all(np.isfinite(value)), 'Values must be finite'


def joint_values(message):
    assert isinstance(message, JointState)
    assert len(message.name) == len(message.position), 'Joint names and positions must match'
    positions = dict(zip(message.name, message.position))
    if not all(name in positions for name in JOINT_NAMES):
        return None
    values = np.array([positions[name] for name in JOINT_NAMES], dtype=float)
    checked_array(values, (5,))
    return values


def to_kdl(values):
    values = np.asarray(values, dtype=float)
    checked_array(values, (5,))
    result = kdl.JntArray(5)
    for i, value in enumerate(values):
        result[i] = float(value)
    return result


def to_numpy(data):
    return np.array([[data[i, j] for j in range(data.columns())] for i in range(data.rows())])


class Kinematics:
    def __init__(self, description, tip='arm_link_ee'):
        if not description:
            raise ValueError('robot_description is required')
        self.chain = build_kdl_chain_from_urdf(description, 'base_link', tip)
        assert self.chain.getNrOfJoints() == 5
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

    def frame(self, values):
        frame = kdl.Frame()
        status = self.fk_solver.JntToCart(to_kdl(values), frame)
        if status != 0:
            raise RuntimeError(f'KDL FK failed: {status}')
        return frame

    def forward_kinematics(self, values):
        frame = self.frame(values)
        result = np.eye(4)
        result[:3, :3] = [[frame.M[i, j] for j in range(3)] for i in range(3)]
        result[:3, 3] = [frame.p[i] for i in range(3)]
        return result


def transform_message(frame, child, stamp):
    message = TransformStamped()
    message.header.frame_id = 'base_link'
    message.child_frame_id = child
    message.header.stamp = stamp
    p, q = message.transform.translation, message.transform.rotation
    p.x, p.y, p.z = [frame.p[i] for i in range(3)]
    q.x, q.y, q.z, q.w = frame.M.GetQuaternion()
    return message
