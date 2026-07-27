from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    youbot_xacro = PathJoinSubstitution([
        FindPackageShare('youbot_description'),
        'robots',
        'youbot_arm_only.urdf.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        youbot_xacro
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    rviz_config = PathJoinSubstitution([
        FindPackageShare('youbot_simulator'),
        'config',
        'youbot.rviz'
    ])

    launch_rviz = LaunchConfiguration('rviz')
    launch_trail = LaunchConfiguration('trail')
    launch_gui = LaunchConfiguration('gui')
    launch_traj_iface = LaunchConfiguration('trajectory_interface')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz2 with the youBot config'
        ),
        DeclareLaunchArgument(
            'trail',
            default_value='true',
            description='Start the trail visualiser'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='(unused) kept for compatibility'
        ),
        DeclareLaunchArgument(
            'trajectory_interface',
            default_value='true',
            description='(unused) kept for compatibility'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='cw2q2_lsa',
            executable='cw2q2_node',
            name='youbot_traj_cw2',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
            output='screen'
        ),
        Node(
            package='youbot_trail_rviz_cw2_lsa',
            executable='youbot_trail_node',
            name='trail_node',
            condition=IfCondition(launch_trail),
            output='screen'
        )
    ])
