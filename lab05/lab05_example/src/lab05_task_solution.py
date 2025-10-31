#!/usr/bin/env python3

import numpy as np
from numpy import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

import PyKDL as kdl

from urdf_kdl_utils import build_kdl_chain_from_urdf

FIXED_KDL_JOINT = getattr(kdl.Joint, 'None')

youbot_joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']

# A reproducible set of arbitrary DH parameters for comparison against the URDF chain
random_dh_parameters = {
    'a': [0.05, 0.12, -0.08, 0.04, 0.02],
    'alpha': [0.0, -pi / 2, pi / 2, -pi / 2, 0.0],
    'd': [0.18, 0.02, -0.03, 0.16, 0.05],
    'theta': [0.0, pi / 6, -pi / 8, pi / 4, -pi / 3],
}


class YoubotKDLKinematic:
    """Broadcasts forward-kinematic transforms using a KDL chain built from the URDF."""

    def __init__(self, node: Node, kdl_kine_chain, tf_suffix: str):
        """Initialise the broadcaster, joint-state listener, and FK solver.

        Parameters
        ----------
        node : rclpy.node.Node
            ROS 2 node used to create subscriptions and log messages.
        kdl_kine_chain : kdl.Chain
            Kinematic chain representing either the URDF-derived model or a DH model.
        tf_suffix : str
            Identifier appended to the child frame id, allowing multiple FK sources
            to coexist on the ``/tf`` tree.

        Notes
        -----
        PyKDL stores the chain as an ordered list of segments. Once the chain is
        in place we instantiate ``ChainFkSolverPos_recursive`` which performs a
        depth-first traversal, multiplying each homogeneous transform to map the
        base frame onto the end-effector. The subscriber caches joint positions
        so intermittent JointState messages do not reset the FK pipeline.
        """
        self.node = node
        self.kine_chain = kdl_kine_chain
        self.tf_suffix = tf_suffix
        self.joint_names = self._extract_joint_names()
        self.joint_count = len(self.joint_names)
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        self._missing_joint_warned = set()
        self.br = TransformBroadcaster(node)
        self.sub = node.create_subscription(JointState, '/joint_states', self.fkine_wrapper, 10)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kine_chain)

    def forward_kinematics(self, joints_kdl):
        """Compute the end-effector pose as a homogeneous matrix.

        Parameters
        ----------
        joints_kdl : kdl.JntArray
            Joint coordinates ordered to match ``self.kine_chain``.

        Returns
        -------
        numpy.ndarray
            A 4x4 homogeneous matrix ``T_0^n`` representing the rigid transform
            from the root link to the terminal link. The upper-left 3x3 block is
            the rotation matrix ``R_0^n`` while the right-most column contains the
            translation vector ``p_0^n``.

        Mathematical Background
        -----------------------
        ``ChainFkSolverPos_recursive`` recursively evaluates the product
        ``T_0^n = prod_i exp([S_i] * q_i) * M_i`` where ``exp([S_i] * q_i)`` is the
        homogeneous transform induced by joint ``i`` (screw axis ``S_i`` and joint
        variable ``q_i``) and ``M_i`` encodes the fixed transform from joint ``i`` to
        ``i+1``. Internally PyKDL converts the joint array into incremental frames
        and multiplies them in sequence, mirroring the Denavit-Hartenberg or URDF
        link composition.
        """
        pose_kdl = kdl.Frame()
        error = self.fk_solver.JntToCart(joints_kdl, pose_kdl)
        if error != 0:
            raise RuntimeError(f'KDL FK solver failed with code {error}')
        return self.convert_kdl_frame_to_mat(pose_kdl)

    def fkine_wrapper(self, joint_msg):
        """ROS callback that converts JointState messages into TF transforms.

        Steps
        -----
        1. Map the message into a ``kdl.JntArray`` using ``joint_state_to_kdl_array``.
        2. Evaluate forward kinematics to obtain ``T_0^n``.
        3. Populate a ``TransformStamped`` message with translation ``p_0^n`` and
           quaternion converted from ``R_0^n``.
        4. Publish the transform so downstream nodes can consume ``/tf`` updates.

        The returned transform is time-stamped with the node clock and anchored at
        the frame ``base_link``; the child frame receives the suffix supplied at
        construction time, allowing multiple FK pipelines to broadcast simultaneously.
        """
        transform = TransformStamped()

        joints_kdl = self.joint_state_to_kdl_array(joint_msg)
        if joints_kdl is None:
            return

        T0ee = self.forward_kinematics(joints_kdl)

        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = self.tf_suffix + '_link'
        transform.transform.translation.x = T0ee[0, 3]
        transform.transform.translation.y = T0ee[1, 3]
        transform.transform.translation.z = T0ee[2, 3]
        transform.transform.rotation = self.rotmat2q(T0ee)
        self.br.sendTransform(transform)

    @staticmethod
    def rotmat2q(T):
        """Convert a 3x3 rotation matrix into a quaternion suitable for TF.

        Parameters
        ----------
        T : numpy.ndarray
            Homogeneous matrix where the upper-left 3x3 block is an orthonormal
            rotation matrix ``R``.

        Returns
        -------
        geometry_msgs.msg.Quaternion
            Quaternion ``(x, y, z, w)`` representing the same rotation.

        Derivation
        ----------
        The trace-based formula stems from the identity
        ``cos(theta) = (trace(R) - 1) / 2``. The quaternion axis components are
        proportional to the off-diagonal elements of ``R``:

        .. math::

            \\mathbf{v} =
            \\frac{1}{2 \\sin(\\theta)}
            \\begin{bmatrix}
            R_{32} - R_{23} \\\\
            R_{13} - R_{31} \\\\
            R_{21} - R_{12}
            \\end{bmatrix}

        Multiplying by ``sin(theta / 2)`` yields the vector part of the quaternion,
        while ``cos(theta / 2)`` forms the scalar component. The special-case branch
        guards against round-off errors when ``theta`` approaches zero.
        """
        q = Quaternion()
        angle = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1) / 2)

        if angle == 0:
            q.w = 1.0
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
        else:
            xr = T[2, 1] - T[1, 2]
            yr = T[0, 2] - T[2, 0]
            zr = T[1, 0] - T[0, 1]

            norm = np.sqrt(np.power(xr, 2) + np.power(yr, 2) + np.power(zr, 2))
            x = xr / norm
            y = yr / norm
            z = zr / norm

            q.w = float(np.cos(angle / 2))
            q.x = float(x * np.sin(angle / 2))
            q.y = float(y * np.sin(angle / 2))
            q.z = float(z * np.sin(angle / 2))
        return q

    @staticmethod
    def convert_kdl_frame_to_mat(frame):
        """Convert a PyKDL ``Frame`` into a homogeneous matrix.

        Parameters
        ----------
        frame : kdl.Frame
            Object containing a rotation ``frame.M`` and translation ``frame.p``.

        Returns
        -------
        numpy.ndarray
            4x4 homogeneous matrix with ``R = frame.M`` and ``p = frame.p``. The
            function explicitly copies each component to avoid implicit references
            to PyKDL internal storage, ensuring downstream numpy code operates on a
            standalone array.
        """
        mat = np.identity(4)
        mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
        mat[:3, :3] = np.array([[frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]]])
        return mat

    def joint_state_to_kdl_array(self, joint_msg):
        """Convert a ROS JointState message into a ``kdl.JntArray``.

        Parameters
        ----------
        joint_msg : sensor_msgs.msg.JointState
            Contains joint names and positions, possibly in arbitrary order.

        Returns
        -------
        kdl.JntArray or None
            Ordered joint vector aligned with ``self.joint_names``. Returns
            ``None`` if no relevant joints were present in the message.

        Implementation Details
        ----------------------
        The method caches the last value seen for each joint, allowing partial
        messages to update subsets of the configuration vector. When a joint name
        is absent we warn once, then reuse the stored position. This behaviour
        mirrors the expectation that a JointState stream may omit unchanged joints.
        """
        name_to_position = {name: pos for name, pos in zip(joint_msg.name, joint_msg.position)}
        updated = False
        for joint_name, position in name_to_position.items():
            if joint_name in self.joint_positions:
                self.joint_positions[joint_name] = position
                updated = True

        if not updated:
            return None

        kdl_array = kdl.JntArray(self.joint_count)
        for idx, joint_name in enumerate(self.joint_names):
            if joint_name not in name_to_position and joint_name not in self._missing_joint_warned:
                self.node.get_logger().warning(
                    f'Joint "{joint_name}" missing from JointState message; using last known position.'
                )
                self._missing_joint_warned.add(joint_name)
            kdl_array[idx] = self.joint_positions[joint_name]

        return kdl_array

    def _extract_joint_names(self):
        """Return the ordered list of non-fixed joint names in the chain.

        PyKDL segments may represent fixed transforms (``Joint.None``). This helper
        drops such entries because the FK solver only expects actuated joints in
        the ``JntArray`` supplied to ``JntToCart``. The iteration preserves the
        original chain ordering so joint indices align with segment indices.
        """
        names = []
        for i in range(self.kine_chain.getNrOfSegments()):
            joint = self.kine_chain.getSegment(i).getJoint()
            if joint.getType() != FIXED_KDL_JOINT:
                names.append(joint.getName())
        return names


def kine_chain_urdf(robot_description: str):
    """Construct a PyKDL chain from the URDF string between ``base_link`` and ``arm_link_ee``.

    The utility delegates to ``build_kdl_chain_from_urdf`` defined in
    ``urdf_kdl_utils.py``. It encapsulates the commonly used root and tip link
    names for the KUKA youBot so the rest of the module can remain agnostic to
    string literals.
    """
    return build_kdl_chain_from_urdf(robot_description, 'base_link', 'arm_link_ee')


def kine_chain_dh(dh_params):
    """Build a PyKDL chain from Denavit-Hartenberg parameters.

    Parameters
    ----------
    dh_params : dict
        Dictionary with four keys: `a`, `alpha`, `d`, and `theta`. Each entry must
        provide one scalar per actuated joint in the same order as
        `youbot_joint_names`. These values follow the classic DH convention where:
          * `a`   (link length) offsets the next joint along the current x-axis.
          * `alpha` (link twist) tilts the next joint's z-axis about the current x-axis.
          * `d`   (link offset) displaces the next joint along the current z-axis.
          * `theta` (joint angle) rotates the next joint about the current z-axis.

    Returns
    -------
    kdl.Chain
        Kinematic chain that mirrors the supplied DH table, suitable for the
        `ChainFkSolverPos_recursive` solver used elsewhere in this module.

    PyKDL building blocks
    ---------------------
    * ``kdl.Vector`` encapsulates a 3D column vector ``[x, y, z]^T``. We use the
      zero vector ``kdl.Vector()`` to position each joint at the origin of the
      current frame so that the DH transform alone encodes the spatial offset.
    * ``kdl.Joint`` represents a single degree of freedom. The constructor
      ``kdl.Joint(name, origin_vector, axis_vector, Joint.RotAxis)``
      creates a revolute joint located at ``origin_vector`` whose motion axis is
      the unit vector ``axis_vector``. By selecting ``axis = (0, 0, 1)`` we ensure
      the joint variable corresponds to the DH ``theta`` rotation about ``z``.
    * ``kdl.Segment`` couples a ``Joint`` with a rigid transform ``Frame``. The
      segment transform describes how the child link frame is placed relative to
      the joint frame. ``kdl.Frame().DH(a, alpha, d, theta)`` implements the
      homogeneous transform
      ``T = Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha)``, where ``R`` and ``T`` denote
      rotations and translations about/along a single axis.
    * ``kdl.Chain`` stores an ordered list of segments. Forward-kinematics can be
      viewed as multiplying the 4x4 transforms attached to each segment:
      ``T_0^n = prod_i T_i``. PyKDL performs this multiplication internally when
      ``ChainFkSolverPos_recursive`` is evaluated.

    Notes
    -----
    PyKDL expects a sequence of `Segment`s, each composed of a `Joint` definition
    and a rigid transform (`Frame`). The helper `Frame().DH(...)` applies the DH
    transform in the order Rz(theta) * Tz(d) * Tx(a) * Rx(alpha). By iterating
    over the DH table we reproduce the analytic kinematic model, enabling direct
    comparisons with the URDF-derived chain when both are driven by identical
    joint states.
    """
    expected = len(youbot_joint_names)
    for key in ('a', 'alpha', 'd', 'theta'):
        if len(dh_params[key]) != expected:
            raise ValueError(f'DH parameter list "{key}" must contain {expected} entries.')
    kine_chain = kdl.Chain()
    for i in range(len(dh_params['a'])):
        joint_name = youbot_joint_names[i]
        joint_axis = kdl.Vector(0, 0, 1)
        joint = kdl.Joint(joint_name, kdl.Vector(), joint_axis, kdl.Joint.RotAxis)
        segment = kdl.Segment(
            joint,
            kdl.Frame().DH(
                dh_params['a'][i],
                dh_params['alpha'][i],
                dh_params['d'][i],
                dh_params['theta'][i],
            ),
        )
        kine_chain.addSegment(segment)
    return kine_chain


class YoubotKDLNode(Node):
    def __init__(self):
        """Initialise the ROS node, load the URDF, and create FK broadcasters.

        The constructor fetches the ``robot_description`` parameter (typically
        provided by a ``robot_state_publisher``), converts it into a URDF-derived
        KDL chain, and instantiates both URDF and DH forward-kinematics pipelines.
        If the parameter is missing we raise an error immediately so launch files
        fail fast.
        """
        super().__init__('youbot_kdl_forward_kinematic_node')
        self.declare_parameter('robot_description', '')
        robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        if not robot_description:
            self.get_logger().error('Parameter "robot_description" is empty; unable to build URDF chain.')
            raise RuntimeError('robot_description parameter is required.')

        urdf_chain = kine_chain_urdf(robot_description)
        self.urdf_fk = YoubotKDLKinematic(self, urdf_chain, 'urdf')
        dh_chain = kine_chain_dh(random_dh_parameters)
        self.dh_fk = YoubotKDLKinematic(self, dh_chain, 'dh_random')


def main(args=None):
    """Entry point that spins the ROS node and handles shutdown.

    The function initialises the rclpy context, instantiates ``YoubotKDLNode`` and
    blocks on ``rclpy.spin`` until the process receives a shutdown signal. The
    try/finally block ensures resources are released regardless of exceptions.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = YoubotKDLNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
