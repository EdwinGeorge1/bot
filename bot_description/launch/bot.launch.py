from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('bot_description')

    # Load URDF file and process xacro
    xacro_file = os.path.join(share_dir, 'urdf', 'bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Joint State Publisher (with GUI enabled)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': True}]
    )

    # Path to bot.world
    world_file = PathJoinSubstitution([
        FindPackageShare('bot_description'),
        'world',
        'bot.world'
    ])

    # Launch Gazebo server with the world file
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Launch Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Delay the spawn_entity node to allow robot_description to be published
    urdf_spawn_node = TimerAction(
        period=5.0,  # Wait 5 seconds before spawning
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'bot',
                    '-topic', 'robot_description'
                ],
                output='screen'
            )
        ]
    )
    # SLAM Toolbox Node for Mapping
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        slam_node,
    ])


##
