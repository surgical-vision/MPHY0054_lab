# launch/youbot_sim.launch.py

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    # --- Declare Launch Arguments ---
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Flag to enable RViz'
    )
    trajectory_arg = DeclareLaunchArgument(
        'trajectory_interface', default_value='false',
        description='Whether to use the trajectory interface'
    )

    # --- Get Package Paths ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_youbot_description = get_package_share_directory('youbot_description')
    pkg_youbot_simulator = get_package_share_directory('youbot_simulator')

    # --- Process Robot Description (URDF from XACRO) ---
    xacro_file = os.path.join(pkg_youbot_description, 'robots', 'youbot_arm_only.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # --- Gazebo Simulation ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'gui': 'true'}.items()
    )

    # --- Robot State Publisher ---
    # Publishes robot state to TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # --- Spawn Robot Entity in Gazebo ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'youbot'],
        output='screen'
    )

    # --- Include Controller Launch File ---
    youbot_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_youbot_simulator, 'launch', 'youbot_controller.launch.py')
        ),
        launch_arguments={'trajectory_interface': LaunchConfiguration('trajectory_interface')}.items()
    )

    # --- RViz ---
    # Launches RViz with a specific config file, only if 'rviz' argument is true
    rviz_config_file = os.path.join(pkg_youbot_simulator, 'config', 'youbot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        gui_arg,
        rviz_arg,
        trajectory_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        youbot_controllers,
        rviz
    ])