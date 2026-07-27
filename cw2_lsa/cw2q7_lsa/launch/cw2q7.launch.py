from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz alongside simulation',
    )
    rviz_enabled = LaunchConfiguration('rviz')

    cw2q7_pkg = FindPackageShare('cw2q7_lsa')
    iiwa_pkg = FindPackageShare('iiwa_ros2_gazebo')

    world_file = PathJoinSubstitution([iiwa_pkg, 'worlds', 'iiwa.world'])
    controllers_file = PathJoinSubstitution([iiwa_pkg, 'config', 'iiwa_controller.yaml'])
    rviz_config = PathJoinSubstitution([cw2q7_pkg, 'config', 'iiwa14.rviz'])

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ',
                PathJoinSubstitution([cw2q7_pkg, 'urdf', 'iiwa_cw2.xacro'])
            ]),
            value_type=str
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])]
        ),
        launch_arguments={
            'world': world_file,
            'gui': 'false',
            'headless': 'true',
        }.items(),
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file, {'use_sim_time': True}],
        output='screen',
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['iiwa_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'iiwa'],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'world'],
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz_enabled),
    )

    cw2q7_node = Node(
        package='cw2q7_lsa',
        executable='cw2q7',
        name='cw2q7',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'command_topic': '/iiwa_controller/joint_trajectory',
            'bag_topic': '/iiwa/EffortJointInterface_trajectory_controller/command',
            'joint_state_topic': '/joint_states'
        }],
    )

    delayed_cw2q7 = TimerAction(period=8.0, actions=[cw2q7_node])

    return LaunchDescription([
        use_rviz_arg,
        gazebo,
        control_node,
        state_publisher,
        joint_state_spawner,
        arm_controller_spawner,
        spawn_robot,
        static_tf,
        rviz,
        delayed_cw2q7,
    ])
