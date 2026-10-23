from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='lab03_task_solution'),
        DeclareLaunchArgument('start_client', default_value='true'),
        Node(package='lab03_task_solution', executable='server', namespace=namespace,
             output='screen'),
        Node(package='lab03_task_solution', executable='client', namespace=namespace,
             output='screen', condition=IfCondition(LaunchConfiguration('start_client'))),
    ])
