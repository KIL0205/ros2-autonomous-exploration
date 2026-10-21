#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # unified sim flag (true/false)
    sim = LaunchConfiguration('sim')
    world = LaunchConfiguration('world')
    early_complete = LaunchConfiguration('early_complete')
    estop_enabled = LaunchConfiguration('estop_enabled')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation (true) or use robot (false).',
    )

    # grab the simulation world from world folder (map) when sim is true
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='small-field',
        description='Name of the simulation world (without .sdf extension).',
    )

    declare_early_complete = DeclareLaunchArgument(
        'early_complete',
        default_value='true',
        description='Treat path as complete after reaching completion_threshold of waypoints.'
    )

    declare_estop_enabled = DeclareLaunchArgument(
        'estop_enabled',
        default_value='true',
        description='Enable laser-based emergency stop behavior in controller.',
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
        condition=IfCondition(sim),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': sim,
            'slam_params_file': slam_params_path,
        }.items(),
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': sim}],
    )

    path_planner = Node(
        package='planning',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[{'use_sim_time': sim,
                     'early_completion_enabled': early_complete,}],
        remappings=[
            ('/map', '/cspace_map'),
        ],
    )

    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': sim,
                     'estop_enabled': estop_enabled,}],
        remappings=[
            ('/move_base_simple/goal', '/go_to'),
        ],
    )

    explorer = Node(
        package='exploration',
        executable='explorer.py',
        name='explorer',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )

    return LaunchDescription([
        declare_sim,
        declare_world,
        declare_early_complete,
        declare_estop_enabled,
        world_and_tb3,
        slam,
        rviz2,
        path_planner,
        controller,
        explorer,
    ])
