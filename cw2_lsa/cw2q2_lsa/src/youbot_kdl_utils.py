"""URDF + PyKDL helpers for the CW2 youBot tasks."""

from collections import defaultdict, deque
import math
from typing import Dict, List

import numpy as np
import PyKDL as kdl
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from xml.etree import ElementTree as ET


def build_kdl_chain_from_urdf(robot_description: str, base_link: str, tip_link: str) -> kdl.Chain:
    """Parse the URDF and construct a PyKDL chain between ``base_link`` and ``tip_link``."""
    if not robot_description:
        raise ValueError('Robot description string is empty.')

    root = ET.fromstring(robot_description)
    joints = _extract_joint_map(root)
    parent_index = _index_joints_by_parent(joints)
    joint_path = _find_joint_path(parent_index, joints, base_link, tip_link)

    chain = kdl.Chain()
    for joint_name in joint_path:
        joint_data = joints[joint_name]
        frame = kdl.Frame(
            kdl.Rotation.RPY(*joint_data['rpy']),
            kdl.Vector(*joint_data['xyz'])
        )

        if joint_data['type'] == 'fixed':
            chain.addSegment(kdl.Segment(
                joint_data['child'],
                kdl.Joint(joint_data['name']),
                frame
            ))
            continue

        chain.addSegment(kdl.Segment(
            f"{joint_name}_origin",
            kdl.Joint(f"{joint_name}_origin"),
            frame
        ))

        joint = _create_kdl_joint(joint_data)
        chain.addSegment(kdl.Segment(joint_data['child'], joint))

    return chain


def kdl_frame_to_numpy(frame: kdl.Frame) -> np.ndarray:
    """Convert a PyKDL ``Frame`` to a 4x4 numpy homogeneous transform."""
    mat = np.identity(4)
    mat[:3, -1] = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
    mat[:3, :3] = np.array([
        [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
        [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
        [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]],
    ])
    return mat


def kdl_jacobian_to_numpy(jacobian: kdl.Jacobian) -> np.ndarray:
    """Convert a PyKDL Jacobian to a numpy ndarray."""
    mat = np.zeros((jacobian.rows(), jacobian.columns()))
    for i in range(jacobian.rows()):
        for j in range(jacobian.columns()):
            mat[i, j] = jacobian[i, j]
    return mat


class YoubotKinematicKDL:
    """Minimal KDL wrapper built directly from the URDF for coursework 2."""

    def __init__(self, node: Node, base_link: str = 'base_link', tip_link: str = 'arm_link_ee') -> None:
        self.node = node
        if not self.node.has_parameter('robot_description'):
            self.node.declare_parameter('robot_description', '')
        robot_description = self.node.get_parameter('robot_description').get_parameter_value().string_value
        if not robot_description:
            raise RuntimeError(
                'Parameter "robot_description" is empty. Make sure your launch file loads the youBot URDF.'
            )

        self.kine_chain = build_kdl_chain_from_urdf(robot_description, base_link, tip_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kine_chain)
        self.jac_calc = kdl.ChainJntToJacSolver(self.kine_chain)
        self.current_joint_position = kdl.JntArray(self.kine_chain.getNrOfJoints())

        qos = QoSProfile(depth=5, history=QoSHistoryPolicy.KEEP_LAST)
        self.node.create_subscription(JointState, '/joint_states', self._joint_state_callback, qos)

    def forward_kinematics(self, joints_readings) -> np.ndarray:
        """Compute forward kinematics for the provided joint array."""
        joints_kdl = self.list_to_kdl_jnt_array(joints_readings)
        pose_kdl = kdl.Frame()
        self.fk_solver.JntToCart(joints_kdl, pose_kdl)
        return kdl_frame_to_numpy(pose_kdl)

    def get_jacobian(self, joints_readings) -> np.ndarray:
        """Compute the manipulator Jacobian for the provided joint array."""
        joints_kdl = self.list_to_kdl_jnt_array(joints_readings)
        jac_kdl = kdl.Jacobian(self.kine_chain.getNrOfJoints())
        self.jac_calc.JntToJac(joints_kdl, jac_kdl)
        return kdl_jacobian_to_numpy(jac_kdl)

    def _joint_state_callback(self, msg: JointState) -> None:
        """Store the latest joint state in a PyKDL JntArray."""
        if len(msg.position) < self.current_joint_position.rows():
            return
        for idx in range(self.current_joint_position.rows()):
            self.current_joint_position[idx] = msg.position[idx]

    @staticmethod
    def list_to_kdl_jnt_array(joints) -> kdl.JntArray:
        kdl_array = kdl.JntArray(5)
        for i in range(0, 5):
            kdl_array[i] = joints[i]
        return kdl_array

    @staticmethod
    def kdl_jnt_array_to_list(kdl_array: kdl.JntArray):
        joints = []
        for i in range(0, 5):
            joints.append(kdl_array[i])
        return joints


def _extract_joint_map(root: ET.Element) -> Dict[str, Dict]:
    joint_map: Dict[str, Dict] = {}
    for joint_el in root.findall('joint'):
        name = joint_el.attrib.get('name')
        if not name:
            continue

        parent_el = joint_el.find('parent')
        child_el = joint_el.find('child')
        if parent_el is None or child_el is None:
            continue

        origin_el = joint_el.find('origin')
        xyz = _parse_vector(origin_el, 'xyz', [0.0, 0.0, 0.0])
        rpy = _parse_vector(origin_el, 'rpy', [0.0, 0.0, 0.0])

        axis_el = joint_el.find('axis')
        axis = _parse_vector(axis_el, 'xyz', [0.0, 0.0, 1.0])
        axis = _normalize(axis)

        joint_map[name] = {
            'name': name,
            'parent': parent_el.attrib.get('link'),
            'child': child_el.attrib.get('link'),
            'xyz': xyz,
            'rpy': rpy,
            'axis': axis,
            'type': joint_el.attrib.get('type', 'fixed').lower()
        }

    if not joint_map:
        raise ValueError('No joints found while parsing URDF.')
    return joint_map


def _index_joints_by_parent(joint_map: Dict[str, Dict]) -> Dict[str, List[str]]:
    index: Dict[str, List[str]] = defaultdict(list)
    for name, data in joint_map.items():
        parent_link = data['parent']
        if parent_link:
            index[parent_link].append(name)
    return index


def _find_joint_path(parent_index: Dict[str, List[str]],
                     joint_map: Dict[str, Dict],
                     base_link: str,
                     tip_link: str) -> List[str]:
    queue = deque([(base_link, [])])
    visited_links = {base_link}

    while queue:
        link, path = queue.popleft()
        if link == tip_link:
            return path
        for joint_name in parent_index.get(link, []):
            child_link = joint_map[joint_name]['child']
            if child_link in visited_links:
                continue
            visited_links.add(child_link)
            queue.append((child_link, path + [joint_name]))

    raise ValueError(f'Unable to find joint path from {base_link} to {tip_link}.')


def _parse_vector(element: ET.Element, attribute: str, default: List[float]) -> List[float]:
    if element is None:
        return default
    raw = element.attrib.get(attribute)
    if not raw:
        return default
    return [float(value) for value in raw.strip().split()]


def _normalize(vector: List[float]) -> List[float]:
    norm = math.sqrt(sum(component * component for component in vector))
    if norm == 0.0:
        return [0.0, 0.0, 1.0]
    return [component / norm for component in vector]


def _create_kdl_joint(joint_data: Dict) -> kdl.Joint:
    axis_vector = kdl.Vector(*joint_data['axis'])
    if joint_data['type'] in ('revolute', 'continuous'):
        return kdl.Joint(joint_data['name'], kdl.Vector(), axis_vector, kdl.Joint.RotAxis)
    if joint_data['type'] == 'prismatic':
        return kdl.Joint(joint_data['name'], kdl.Vector(), axis_vector, kdl.Joint.TransAxis)
    return kdl.Joint(joint_data['name'])
