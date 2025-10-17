import launch
import launch_ros
from launch.actions import TimerAction


def generate_launch_description():

    action_node_pub_node = launch_ros.actions.Node(
        package='lab03_example01',
        executable='rot_publisher',
        output='screen'
    )

    action_node_sub_node = launch_ros.actions.Node(
        package='lab03_example01',
        executable='rot_converter',
        output='screen'
    )
    
    delayed_pub_node = TimerAction(period=3.0, actions=[action_node_pub_node])


    return launch.LaunchDescription([
        action_node_sub_node,
        delayed_pub_node
    ])
