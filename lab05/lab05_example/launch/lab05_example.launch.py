from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    open_manipulator_xacro = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'urdf',
        'open_manipulator.urdf.xacro'
    ])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        open_manipulator_xacro
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

    open_fkine_node = Node(
        package='lab05_example',
        executable='lab05_example.py',
        name='open_fkine_node',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }]
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('open_manipulator_description'),
        'rviz',
        'open_manipulator_links_hidden.rviz'
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
        open_fkine_node,
        rviz_node
    ])
