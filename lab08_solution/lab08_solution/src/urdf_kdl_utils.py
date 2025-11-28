"""Utilities for building PyKDL chains directly from URDF strings."""

from collections import defaultdict, deque
import math
from typing import Dict, List
from xml.etree import ElementTree as ET

import PyKDL as kdl


def build_kdl_chain_from_urdf(robot_description: str, base_link: str, tip_link: str) -> kdl.Chain:
    """Parse the URDF and construct a PyKDL chain between ``base_link`` and ``tip_link``.

    Parameters
    ----------
    robot_description : str
        XML string containing the full robot URDF, typically sourced from the
        ``robot_description`` parameter on the ROS parameter server.
    base_link : str
        Name of the root link where the kinematic chain should start.
    tip_link : str
        Name of the terminal link (end-effector) that anchors the chain.

    Returns
    -------
    kdl.Chain
        Forward-kinematics representation whose segments mirror the URDF joint
        sequence along the unique path from ``base_link`` to ``tip_link``.

    Implementation Outline
    ----------------------
    1. ``_extract_joint_map`` converts the URDF into a dictionary keyed by joint
       name. Each entry stores the joint type, parent/child links, axis, and the
       ``origin`` transform expressed as roll-pitch-yaw and xyz components.
    2. ``_index_joints_by_parent`` builds an adjacency map ``parent_link -> joints``
       which is subsequently traversed via breadth-first search to locate the
       joint path connecting the specified links.
    3. For every joint along that path we append PyKDL ``Segment`` objects to a new
       chain. Fixed joints contribute a single segment containing only the rigid
       ``origin`` transform, while actuated joints contribute two segments: a fixed
       transform to the joint frame followed by the actual ``kdl.Joint``.

    Mathematical Notes
    ------------------
    The URDF origin is interpreted as a homogeneous transform
    ``T_parent^joint = R_z(roll) * R_y(pitch) * R_x(yaw) * T(x, y, z)``. PyKDL stores
    this as a ``Frame`` composed of a ``Rotation`` and a ``Vector``. Revolute and
    prismatic joints are instantiated with screw axes defined by the URDF ``axis``
    vector, allowing PyKDL to compute exponentials ``exp([S] * q)`` during FK.
    """
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
    """Return a mapping from joint name to URDF metadata.

    Each entry contains:
    ``parent`` and ``child`` link names,
    the translation ``xyz`` (metres) and rotation ``rpy`` (radians) extracted
    from the ``origin`` tag,
    the unit axis vector for revolute/prismatic joints,
    and the lowercase joint ``type``.

    The helper tolerates missing optional tags by falling back to defaults (zero
    offset, identity rotation, ``z`` axis). An empty result raises a ValueError
    because a kinematic chain cannot be built without joints.
    """
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
    """Group joint names by their parent link for graph traversal.

    The URDF describes a tree where links own child joints. Constructing a
    dictionary of ``parent_link -> [joint_names]`` enables efficient breadth-first
    searches when determining a path between two links.
    """
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
    """Locate the ordered list of joint names connecting ``base_link`` to ``tip_link``.

    The algorithm performs a breadth-first search over the link graph. Each queue
    entry stores the current link and the accumulated path (joint sequence) used to
    reach it. When the desired ``tip_link`` is dequeued we return its joint path.
    Visited links prevent cycles. Failure to reach the tip raises a ValueError.
    """
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
    """Read a numeric vector attribute from an XML element.

    Attributes are space-separated numeric strings. If the element or attribute is
    absent we return the provided default. Converting to ``float`` ensures PyKDL
    receives native numeric types.
    """
    if element is None:
        return default
    raw = element.attrib.get(attribute)
    if not raw:
        return default
    return [float(value) for value in raw.strip().split()]


def _normalize(vector: List[float]) -> List[float]:
    """Normalise a 3D vector, returning a default ``z`` axis if the norm is zero."""
    norm = math.sqrt(sum(component * component for component in vector))
    if norm == 0.0:
        return [0.0, 0.0, 1.0]
    return [component / norm for component in vector]


def _create_kdl_joint(joint_data: Dict) -> kdl.Joint:
    """Instantiate the appropriate PyKDL joint based on URDF metadata.

    Revolute/continuous joints produce ``Joint.RotAxis`` instances, prismatic
    joints become ``Joint.TransAxis``, and all other types (including fixed joints)
    degrade to ``Joint.None``. The axis vector is lifted into a ``kdl.Vector`` so
    PyKDL can treat it as the screw axis during FK.
    """
    axis_vector = kdl.Vector(*joint_data['axis'])
    if joint_data['type'] in ('revolute', 'continuous'):
        return kdl.Joint(joint_data['name'], kdl.Vector(), axis_vector, kdl.Joint.RotAxis)
    if joint_data['type'] == 'prismatic':
        return kdl.Joint(joint_data['name'], kdl.Vector(), axis_vector, kdl.Joint.TransAxis)
    return kdl.Joint(joint_data['name'])
