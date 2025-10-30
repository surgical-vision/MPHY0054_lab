from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_gui': True}]
    )

    youbot_kdl_node = Node(
        package='lab05_example',
        executable='lab05_task.py',
        name='youbot_kdl_node',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('youbot_simulator'),
        'config',
        'youbot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_gui,
        robot_state_publisher,
        youbot_kdl_node,
        rviz_node
    ])
