"""Utilities for building PyKDL chains directly from URDF strings."""

from collections import defaultdict, deque
import math
from typing import Dict, List
from xml.etree import ElementTree as ET

import PyKDL as kdl


def build_kdl_chain_from_urdf(robot_description: str, base_link: str, tip_link: str) -> kdl.Chain:
    """Parse the URDF and construct a PyKDL chain between base_link and tip_link."""
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

        # Add the fixed transform from parent link to the joint frame.
        chain.addSegment(kdl.Segment(
            f"{joint_name}_origin",
            kdl.Joint(f"{joint_name}_origin"),
            frame
        ))

        joint = _create_kdl_joint(joint_data)
        chain.addSegment(kdl.Segment(joint_data['child'], joint))

    return chain


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
