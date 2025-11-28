"""Launch file for the lab08_solution IK demo (optionally with RViz).

Starts:
1. robot_state_publisher to expose TF frames from the URDF.
2. lab08_solution node that runs the Jacobian-transpose IK example.
3. RViz2 (optional) so you can visualise the TFs and markers.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 to visualise TFs and markers.'
    )

    pkg_youbot_description = get_package_share_directory('youbot_description')
    xacro_file = os.path.join(
        pkg_youbot_description, 'robots', 'youbot_arm_only.urdf.xacro'
    )
    robot_description = xacro.process_file(xacro_file).toxml()
    robot_params = {'robot_description': robot_description}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='lab08_state_publisher',
        output='screen',
        parameters=[robot_params]
    )

    ik_demo_node = Node(
        package='lab08_solution',
        executable='lab08_solution',
        name='lab08_youbot_ik_demo',
        output='screen',
        parameters=[robot_params]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='lab08_rviz',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('lab08_solution'),
            'config',
            'lab08_solution.rviz'
        )],
        parameters=[robot_params],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        ik_demo_node,
        rviz_node
    ])
