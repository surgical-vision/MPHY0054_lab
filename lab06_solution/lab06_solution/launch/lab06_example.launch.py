from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    robot_description_content = Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro')
        ]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('youbot_description'),
            'robots',
            'youbot_arm_only.urdf.xacro'
        ])
    ])

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    rviz_config = PathJoinSubstitution([
        FindPackageShare('youbot_simulator'),
        'config',
        'youbot.rviz'
    ])

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_gui': True}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description]
        ),
        Node(
            package='lab06_solution',
            executable='lab06_example',
            name='youbot_kdl_node',
            parameters=[robot_description]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
