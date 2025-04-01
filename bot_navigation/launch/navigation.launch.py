#!/usr/bin/python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Set the correct map filename (Make sure this file exists in bot_navigation/maps/)
MAP_NAME = 'bot_cafe'  

def generate_launch_description():

    # Path to Nav2 bringup launch file
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    # Path to RViz configuration file
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('bot_description'), 'launch', 'display.rviz']
    )

    # Corrected Map Path (Ensure the map file exists in bot_navigation/maps/)
    default_map_path = PathJoinSubstitution(
        [FindPackageShare('bot_navigation'), 'maps', f'{MAP_NAME}.yaml']
    )

    # Navigation configuration parameters
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('bot_navigation'), 'config', 'navigation.yaml']
    )

    return LaunchDescription([
        # Use simulation time (Gazebo) or real-time
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Enable RViz visualization
        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run RViz'
        ),

        # Set the map file path
        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Full path to the map YAML file'
        ),

        # Include Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'params_file': nav2_config_path
            }.items()
        ),

        # Launch RViz if enabled
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
        ),
    ])
