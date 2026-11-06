from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'lab05_task_solution'
REMAP = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
         ('/joint_states', 'joint_states'), ('/robot_description', 'robot_description')]


def launch_nodes(context):
    share = Path(get_package_share_directory(PACKAGE))
    namespace = LaunchConfiguration('namespace').perform(context)
    gui = IfCondition(LaunchConfiguration('gui')).evaluate(context)
    rviz = IfCondition(LaunchConfiguration('rviz')).evaluate(context)
    start_task = IfCondition(LaunchConfiguration('start_task')).evaluate(context)
    # Snap editor environments can inject incompatible GTK libraries into desktop ROS tools.
    gui_environment = {key: '' for key in ('GTK_PATH', 'GIO_MODULE_DIR')
                       if '/snap/' in os.environ.get(key, '')}
    mode = LaunchConfiguration('mode').perform(context)
    model, config = {
        'task': ('youbot.urdf', 'youbot.rviz'),
        'example': ('open_manipulator.urdf', 'open_manipulator.rviz'),
        'robot': ('my_robot.urdf', 'my_robot.rviz'),
    }[mode]
    description = (share / 'models' / model).read_text()
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=namespace, parameters=[{'robot_description': description}], remappings=REMAP),
        Node(package='joint_state_publisher_gui' if gui else 'joint_state_publisher',
             executable='joint_state_publisher_gui' if gui else 'joint_state_publisher',
             namespace=namespace, parameters=[{'robot_description': description}], remappings=REMAP,
             additional_env=gui_environment),
    ]
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', namespace=namespace,
                            arguments=['-d', str(share / 'config' / config)], remappings=REMAP,
                            additional_env=gui_environment))
    if start_task:
        if mode != 'robot':
            actions.append(Node(package=PACKAGE, executable='task' if mode == 'task' else 'example',
                                namespace=namespace, output='screen', remappings=REMAP,
                                parameters=[{'robot_description': description}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=PACKAGE),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_task', default_value='true'),
        DeclareLaunchArgument('mode', default_value='task', choices=['task', 'example', 'robot']),
        OpaqueFunction(function=launch_nodes),
    ])
