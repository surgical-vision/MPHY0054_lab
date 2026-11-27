from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'lab07_task_solution'
REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static'), ('/joint_states', 'joint_states'),
         ('/robot_description', 'robot_description')]


def launch_nodes(context):
    share = Path(get_package_share_directory(PACKAGE))
    namespace = LaunchConfiguration('namespace').perform(context)
    enabled = lambda key: IfCondition(LaunchConfiguration(key)).evaluate(context)
    description = (share / 'models/youbot.urdf').read_text()
    gui_environment = {key: '' for key in ('GTK_PATH', 'GIO_MODULE_DIR')
                       if '/snap/' in os.environ.get(key, '')}
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                    namespace=namespace, parameters=[{'robot_description': description}], remappings=REMAP)]
    if 7 == 6 or not enabled('start_task'):
        publisher = 'joint_state_publisher_gui' if enabled('gui') else 'joint_state_publisher'
        actions.append(Node(package=publisher, executable=publisher, namespace=namespace,
                            parameters=[{'robot_description': description}], remappings=REMAP,
                            additional_env=gui_environment))
    if enabled('start_task'):
        if 7 == 7:
            actions.append(Node(package=PACKAGE, executable='preview', namespace=namespace, remappings=REMAP))
        actions.append(Node(package=PACKAGE, executable='task', namespace=namespace, output='screen',
                            parameters=[{'robot_description': description}], remappings=REMAP))
    if enabled('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', namespace=namespace, remappings=REMAP,
                            arguments=['-d', str(share / 'config/lab.rviz')], additional_env=gui_environment))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=PACKAGE),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_task', default_value='true'),
        OpaqueFunction(function=launch_nodes),
    ])
