#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    sim = LaunchConfiguration('sim')
    replan = LaunchConfiguration('replan')
    world = LaunchConfiguration('world')
    estop_enabled = LaunchConfiguration('estop_enabled')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation (true) or use robot (false).',
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value='small-field', description='Simulation world'
    )
    declare_replan = DeclareLaunchArgument(
        'replan',
        default_value='false',
        description='Enable periodic replanning in path_planner (true/false).'
    )
    declare_estop_enabled = DeclareLaunchArgument(
        'estop_enabled',
        default_value='true',
        description='Enable laser-based emergency stop behavior in controller.',
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
        condition=IfCondition(sim),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': sim, 'yaml_filename': map_yaml_path}]
    )

    amcl = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[amcl_params_path, {
        'set_initial_pose': True,
        'initial_pose_x': 0.75,
        'initial_pose_y': 0.75,
        'initial_pose_z': 0.0,
        'initial_pose_a': 0.0,
    }],
    remappings=[('scan', '/scan')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': sim,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl']}]
    )

    blowup = Node(
        package='exploration',  
        executable='blowup.py',  
        name='blowup',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )

    rviz2 = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2', 
        output='screen',
        arguments=['-d', rviz_config_path], 
        parameters=[{'use_sim_time': sim}],
    )

    explorer = Node(
        package='exploration',
        executable='explorer.py',
        name='explorer',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )

    path_planner = Node(
        package='planning', executable='path_planner.py',
        name='path_planner', output='screen',
        parameters=[{
            'use_sim_time': sim,
            'enable_replanning': replan,
            'replan_period': 4.0,   # seconds
        }],
        remappings=[('/map', '/cspace_map')],
    )


    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        output='screen',
        parameters=[{
            'use_sim_time': sim,
            'estop_enabled': estop_enabled,
        }],
        remappings=[('/move_base_simple/goal', '/go_to')],
    )

    explorer = Node(
        package='exploration',
        executable='explorer.py',
        name='explorer',
        output='screen',
        parameters=[{'use_sim_time': sim,
                     'padding': 4,}],
    )

    return LaunchDescription([
        declare_sim,
        declare_world,
        declare_replan,
        declare_estop_enabled,
        world_and_tb3,
        map_server,
        blowup,
        lifecycle_manager,
        amcl,
        rviz2,
        path_planner,
        controller,
        explorer,
    ])

# change file path in explorer func
# change rviz map to be transient local instead of volatile 
# check ros2 param get /map_server yaml_filename