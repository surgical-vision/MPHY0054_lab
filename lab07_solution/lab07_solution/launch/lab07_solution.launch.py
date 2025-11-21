"""One-click launch file for the lab07_solution demo (optionally with RViz).

Components brought up in order:
1. A launch argument `rviz` that toggles RViz (default `true` to show the scene).
2. `robot_state_publisher`, which provides the arm_link_* TF frames and republishes
   `/robot_description`.
3. `youbot_controllers`, spawning the joint_state_broadcaster and trajectory interface.
4. The `lab07_solution` node publishing trajectories, joint states, markers, and the helper TF.
5. RViz pre-loaded with the matching configuration file.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 to visualise the robot and checkpoints.'
    )

    pkg_youbot_description = get_package_share_directory('youbot_description')
    pkg_youbot_simulator = get_package_share_directory('youbot_simulator')

    xacro_file = os.path.join(
        pkg_youbot_description, 'robots', 'youbot_arm_only.urdf.xacro'
    )
    robot_description = xacro.process_file(xacro_file).toxml()
    robot_params = {'robot_description': robot_description}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='lab07_state_publisher',
        output='screen',
        parameters=[robot_params]
    )

    youbot_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_youbot_simulator, 'launch', 'youbot_controller.launch.py')
        ),
        launch_arguments={'trajectory_interface': 'true'}.items()
    )

    trajectory_node = Node(
        package='lab07_solution',
        executable='lab07_solution',
        name='lab07_youbot_traj',
        output='screen',
        parameters=[robot_params]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('lab07_solution'),
        'config',
        'lab07_solution.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='lab07_rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[robot_params],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        youbot_controllers,
        trajectory_node,
        rviz_node
    ])
