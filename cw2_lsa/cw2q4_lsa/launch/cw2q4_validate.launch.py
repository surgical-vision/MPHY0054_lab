#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='cw2q4_lsa',
                executable='cw2q4_validate.py',
                name='cw2q4_validator',
                output='screen',
            )
        ]
    )
