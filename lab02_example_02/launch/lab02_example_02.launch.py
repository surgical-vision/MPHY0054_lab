import launch
import launch_ros


def generate_launch_description():

    action_node_pub_node = launch_ros.actions.Node(

        package='lab02_example_02',
        executable='lab02_example_02_pub',
        output='screen'
    )

    action_node_sub_node = launch_ros.actions.Node(

        package='lab02_example_02',
        executable='lab02_example_02_sub',
        output='screen'
    )

    return launch.LaunchDescription([
        action_node_pub_node,
        action_node_sub_node
    ])
