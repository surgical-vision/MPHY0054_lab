"""Utilities for building PyKDL chains directly from URDF strings."""

from collections import defaultdict, deque
import math
from typing import Dict, List
from xml.etree import ElementTree as ET

import PyKDL as kdl
import numpy as np


def build_kdl_chain_from_urdf(robot_description: str, base_link: str, tip_link: str) -> kdl.Chain:
    """Parse the URDF and construct a PyKDL chain between ``base_link`` and ``tip_link``."""
    if not robot_description:
        raise ValueError('Robot description string is empty.')

    root = ET.fromstring(robot_description)
    joints = _extract_joint_map(root)
    link_inertias = _extract_link_inertias(root)
    parent_index = _index_joints_by_parent(joints)
    joint_path = _find_joint_path(parent_index, joints, base_link, tip_link)

    chain = kdl.Chain()
    for joint_name in joint_path:
        joint_data = joints[joint_name]
        inertia = _create_rigid_body_inertia(link_inertias.get(joint_data['child']))
        frame = kdl.Frame(
            kdl.Rotation.RPY(*joint_data['rpy']),
            kdl.Vector(*joint_data['xyz'])
        )

        if joint_data['type'] == 'fixed':
            chain.addSegment(kdl.Segment(
                joint_data['child'],
                kdl.Joint(joint_data['name']),
                frame,
                inertia
            ))
            continue

        chain.addSegment(kdl.Segment(
            f"{joint_name}_origin",
            kdl.Joint(f"{joint_name}_origin"),
            frame
        ))

        joint = _create_kdl_joint(joint_data)
        chain.addSegment(kdl.Segment(joint_data['child'], joint, kdl.Frame(), inertia))

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


def _extract_link_inertias(root: ET.Element) -> Dict[str, Dict]:
    link_inertias: Dict[str, Dict] = {}
    for link_el in root.findall('link'):
        name = link_el.attrib.get('name')
        inertial_el = link_el.find('inertial')
        if not name or inertial_el is None:
            continue

        mass_el = inertial_el.find('mass')
        inertia_el = inertial_el.find('inertia')
        if mass_el is None or inertia_el is None:
            continue

        origin_el = inertial_el.find('origin')
        link_inertias[name] = {
            'mass': float(mass_el.attrib.get('value', 0.0)),
            'xyz': _parse_vector(origin_el, 'xyz', [0.0, 0.0, 0.0]),
            'rpy': _parse_vector(origin_el, 'rpy', [0.0, 0.0, 0.0]),
            'inertia': [
                float(inertia_el.attrib.get('ixx', 0.0)),
                float(inertia_el.attrib.get('iyy', 0.0)),
                float(inertia_el.attrib.get('izz', 0.0)),
                float(inertia_el.attrib.get('ixy', 0.0)),
                float(inertia_el.attrib.get('ixz', 0.0)),
                float(inertia_el.attrib.get('iyz', 0.0)),
            ],
        }
    return link_inertias


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


def _create_rigid_body_inertia(inertia_data: Dict) -> kdl.RigidBodyInertia:
    if not inertia_data:
        return kdl.RigidBodyInertia()

    mass = inertia_data['mass']
    cog = kdl.Vector(*inertia_data['xyz'])

    rotation = kdl.Rotation.RPY(*inertia_data['rpy'])
    inertia_tensor = _rotate_inertia_tensor(inertia_data['inertia'], rotation)

    return kdl.RigidBodyInertia(
        mass,
        cog,
        kdl.RotationalInertia(
            inertia_tensor[0, 0],
            inertia_tensor[1, 1],
            inertia_tensor[2, 2],
            inertia_tensor[0, 1],
            inertia_tensor[0, 2],
            inertia_tensor[1, 2],
        ),
    )


def _rotate_inertia_tensor(inertia_values: List[float], rotation: kdl.Rotation) -> np.ndarray:
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz = inertia_values
    inertia_mat = np.array([
        [Ixx, Ixy, Ixz],
        [Ixy, Iyy, Iyz],
        [Ixz, Iyz, Izz],
    ])
    rot_mat = np.array([
        [rotation[0, 0], rotation[0, 1], rotation[0, 2]],
        [rotation[1, 0], rotation[1, 1], rotation[1, 2]],
        [rotation[2, 0], rotation[2, 1], rotation[2, 2]],
    ])
    return rot_mat @ inertia_mat @ rot_mat.T
