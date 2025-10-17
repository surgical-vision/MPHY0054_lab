import launch
import launch_ros


def generate_launch_description():

    action_node_client_node = launch_ros.actions.Node(
        package='lab03_task',
        executable='client',
        output='screen'
    )

    action_node_server_node = launch_ros.actions.Node(
        package='lab03_task',
        executable='server',
        output='screen'
    )


    return launch.LaunchDescription([
        action_node_client_node,
        action_node_server_node
    ])
