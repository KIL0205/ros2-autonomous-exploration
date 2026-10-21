#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    # use the time of the sim
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )

    # grab the simulation world from world folder (map)
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='small-field',
        description='Name of the simulation world (without .sdf extension).',
    )

    # share directories and launch
    sim_share = get_package_share_directory('simulation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    exploration_share = get_package_share_directory('exploration')

    rviz_config_path = os.path.join(
        exploration_share,
        'rviz',
        'lab4_config.rviz'
    )

    slam_params_path = os.path.join(
        exploration_share,
        'config',
        'slam_params.yaml'
    )

    world_and_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'world_test.launch.py')
        ),
        launch_arguments={
            'world': world,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_path,
        }.items(),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    path_planner = Node(
        package='planning',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/map', '/cspace_map'),
        ],
    )

    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/move_base_simple/goal', '/go_to'),
        ],
    )

    explorer = Node(
        package='exploration',
        executable='explorer.py',
        name='explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        world_and_tb3,
        slam,
        rviz2,
        path_planner,
        controller,
        explorer,
    ])