import launch
import launch_ros


def generate_launch_description():

    action_node_pub_node = launch_ros.actions.Node(
        package='lab02_task01',
        executable='lab02_task01_talker',
        output='screen'
    )

    action_node_sub_node = launch_ros.actions.Node(
        package='lab02_task01',
        executable='lab02_task01_listener',
        output='screen'
    )


    return launch.LaunchDescription([
        action_node_pub_node,
        action_node_sub_node
    ])
