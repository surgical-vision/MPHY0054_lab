import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the share directory of the 'youbot_description' and 'cw1q5' packages
    youbot_description_share_dir = get_package_share_directory('youbot_description')
    cw1q5_share_dir = get_package_share_directory('cw1q5')

    # --- 1. Get the robot description from the XACRO file ---
    xacro_file = os.path.join(youbot_description_share_dir, 'robots', 'youbot_arm_only.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # --- 2. Define the nodes to be launched ---

    # a. Robot State Publisher
    # This node publishes the robot's state to the /tf2 topic.
    # It requires the robot_description parameter.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # b. Joint State Publisher GUI
    # This node provides a GUI to manually control the robot's joint states.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # c. Custom Node Publisher
    cw1q5b_publisher_node = Node(
        package='cw1q5',
        executable='cw1q5b_node',
        name='cw1q5b_publisher'
    )

    # d. RViz2
    rviz_config_file = os.path.join(cw1q5_share_dir, 'rviz', 'q5b.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    # --- 3. Create the LaunchDescription and add the nodes ---
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        cw1q5b_publisher_node,
        rviz_node
    ])