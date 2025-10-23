# ROS2 Foxy – Launch template (NO answers)

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ===== Paths (edit if your package/layout differs) =====
    # If you use a different description package, change the name below.
    open_manipulator_desc = FindPackageShare('open_manipulator_description')

    # If your URDF/Xacro filename or folder differs, update this path.
    urdf_file = PathJoinSubstitution([
        open_manipulator_desc, 'urdf', 'open_manipulator.urdf.xacro'
    ])

    # If you use a different RViz config, update this path.
    rviz_config = PathJoinSubstitution([
        open_manipulator_desc, 'rviz', 'open_manipulator_links_hidden.rviz'
    ])

    return LaunchDescription([
        # 1) robot_description (xacro -> urdf)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ', urdf_file
                ])
            }]
        ),

        # 2) GUI joint publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_gui': True}],
            # If you have a custom source list, set it via rosparams here.
            # parameters=[{'use_gui': True, 'source_list': ['robotis/real_joint_states']}],
        ),

        # 3) TODO: Add your lab package node (e.g., FK forward kinematics publisher)
        # Node(
        #     package='YOUR_PACKAGE_NAME',          # TODO
        #     executable='YOUR_EXECUTABLE_NAME',    # TODO (matches console_scripts/target)
        #     name='open_fkine_node',
        #     output='screen',
        #     # parameters=[{ ... }],               # TODO: if you need params
        # ),

        # 4) RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])

