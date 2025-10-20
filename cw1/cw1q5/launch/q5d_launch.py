import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the share directory for required packages
    youbot_description_share_dir = get_package_share_directory('youbot_description')
    youbot_simulator_share_dir = get_package_share_directory('youbot_simulator')

    # --- Process the URDF xacro file ---
    # This is the ROS 2 equivalent of the <param name="robot_description" .../> tag 
    xacro_file = os.path.join(youbot_description_share_dir, 'robots', 'youbot_arm_only.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # --- Define the nodes to be launched ---

    # a. Robot State Publisher
    # Publishes the robot's state to TF2 using the URDF information.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # b. Joint State Publisher GUI
    # Provides a GUI to manually control the robot's joint states.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    # c. Custom Node for this question
    # This launches your custom Python node.
    # Note: Assumes an entry point 'cw1q5d_node' is defined in setup.py.
    cw1q5d_publisher_node = Node(
        package='cw1q5',
        executable='cw1q5d_node', 
        name='cw1q5d_publisher'
    )

    # d. RViz2
    # Launches the RViz2 visualization tool with a specific config file.
    rviz_config_file = os.path.join(youbot_simulator_share_dir, 'config', 'youbot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    # --- Create the LaunchDescription and add the nodes ---
    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        cw1q5d_publisher_node,
        rviz_node
    ])