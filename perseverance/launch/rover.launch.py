from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources.any_launch_description_source import AnyLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    Launches the state estimation and sensor nodes for the Perseverance rover.
    """
    perseverance_pkg_share = get_package_share_directory('perseverance')

    # --- Declare Launch Arguments ---
    sensor_namespace_arg = DeclareLaunchArgument(
        'sensor_namespace',
        default_value='',
        description='Namespace for sensor topics'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for all nodes'
    )
    use_sim_sensors_arg = DeclareLaunchArgument(
        'use_sim_sensors',
        default_value='false',
        description='Use simulated or real sensors [true, false]'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 [true, false]'
    )

    # --- Launch Configurations ---
    sensor_namespace = LaunchConfiguration('sensor_namespace')
    log_level = LaunchConfiguration('log_level')
    use_sim_sensors = LaunchConfiguration('use_sim_sensors')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # --- Include Launch Files ---
    state_estimation_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                perseverance_pkg_share, 'launch',
                'state_estimation.launch.xml'
            ])
        ),
        launch_arguments={
            'log_level': log_level,
            'launch_rviz': launch_rviz
        }.items()
    )

    sensors_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                perseverance_pkg_share, 'launch',
                'sensors.launch.xml'
            ])
        ),
        launch_arguments={
            'namespace': sensor_namespace,
            'log_level': log_level,
            'use_sim_sensors': use_sim_sensors
        }.items()
    )

    communications_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                perseverance_pkg_share, 'launch',
                'communications.launch.xml'
            ])
        ),
        launch_arguments={
            'log_level': log_level
        }.items()
    )

    # --- Joint + Robot State Publisher ---
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ', PathJoinSubstitution(
                        [perseverance_pkg_share, 'urdf', 'rover.urdf.xacro'])
                ])
            }
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Robot State Publisher uses the same robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ', PathJoinSubstitution(
                        [perseverance_pkg_share, 'urdf', 'rover.urdf.xacro'])
                ])
            }
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution(
                [perseverance_pkg_share, 'config', 'rover.rviz']),
            '--ros-args', '--log-level', log_level
        ],
        condition=IfCondition(launch_rviz)
    )

    return LaunchDescription([
        sensor_namespace_arg,
        log_level_arg,
        use_sim_sensors_arg,
        launch_rviz_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        state_estimation_launch,
        sensors_launch,
        # communications_launch,
        rviz_node
    ])
