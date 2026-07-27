from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='youbot_trail_rviz_cw2_lsa',
            executable='youbot_trail_node',
            name='trail_node',
            output='screen'
        )
    ])
