#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )

    declare_world = DeclareLaunchArgument(
        'world', default_value='small-field', description='Simulation world'
    )

    sim_share = get_package_share_directory('simulation')
    exploration_share = get_package_share_directory('exploration')

    rviz_config_path = os.path.join(exploration_share, 'rviz', 'lab4_config.rviz')
    amcl_params_path = os.path.join(exploration_share, 'config', 'amcl_params.yaml')
    map_yaml_path = os.path.join(exploration_share, 'maps', 'explored_map.yaml')
    print(f"Map YAML path: {map_yaml_path}")
    print(f"Map file exists: {os.path.exists(map_yaml_path)}")

    world_and_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'world_test.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_path}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_path,{
                     'set_initial_pose': True,
                     'initial_pose_x': 0.1,
                     'initial_pose_y': 0.0,
                     'initial_pose_z': 0.0,
                     'initial_pose_a': 0.0,}],
        remappings=[('scan', '/scan')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']}]
    )

    blowup = Node(
        package='exploration',  
        executable='blowup.py',  
        name='blowup',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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
        package='planning', executable='path_planner.py', name='path_planner', output='screen',
        parameters=[{'use_sim_time': use_sim_time}], remappings=[
            ('/map', '/cspace_map'),
        ],
    )

    controller = Node(
        package='control', executable='controller.py', name='controller', output='screen',
        parameters=[{'use_sim_time': use_sim_time}], remappings=[('/move_base_simple/goal', '/go_to')],
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
        map_server,
        amcl,
        lifecycle_manager,
        blowup,
        rviz2,
        path_planner,
        controller,
        explorer,
    ])

# change file path in explorer func
# change rviz map to be transient local instead of volatile 
# check ros2 param get /map_server yaml_filename