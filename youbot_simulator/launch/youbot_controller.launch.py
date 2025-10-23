# launch/youbot_controller.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare the launch argument
    trajectory_interface_arg = DeclareLaunchArgument(
        'trajectory_interface',
        default_value='false',
        description='Whether to use the trajectory interface for controllers'
    )
    
    # Get path to the controller config file
    youbot_control_config = os.path.join(
        get_package_share_directory('youbot_simulator'),
        'config',
        'youbot_control.yaml'
    )

    # --- Controller Spawner for EffortJointInterface ---
    # This spawner is activated unless trajectory_interface is true
    spawner_effort_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'EffortJointInterface_J1_controller',
            'EffortJointInterface_J2_controller',
            'EffortJointInterface_J3_controller',
            'EffortJointInterface_J4_controller',
            'EffortJointInterface_J5_controller',
            '--controller-manager', '/controller_manager'
        ],
        condition=UnlessCondition(LaunchConfiguration('trajectory_interface'))
    )

    # --- Controller Spawner for Trajectory Interface ---
    # This spawner is activated only if trajectory_interface is true
    spawner_trajectory_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'EffortJointInterface_trajectory_controller',
            '--controller-manager', '/controller_manager'
        ],
        condition=IfCondition(LaunchConfiguration('trajectory_interface'))
    )

    return LaunchDescription([
        trajectory_interface_arg,
        spawner_effort_controllers,
        spawner_trajectory_controllers
    ])