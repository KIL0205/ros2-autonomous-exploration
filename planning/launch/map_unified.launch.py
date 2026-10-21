import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():
    sim = LaunchConfiguration('sim')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation (true) or use robot (false).',
    )

    map_file_path = os.path.join(
        get_package_share_directory('planning'),
        'maps',
        'simple_map.yaml'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('planning'),
        'rviz',
        'lab3_config.rviz'
    )

    # map_server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file_path,
            'use_sim_time': sim,
        }]
    )

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': sim},
            {'autostart': True},
            {'node_names': ['map_server']},
        ]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map',
        '--child-frame-id', 'odom'], # supposed to use ros2 style args but i dont think it matters tbh
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    path_planner = Node(
        package='planning',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': sim}.items(),
        condition=IfCondition(sim),
    )


    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': sim}],
        remappings=[
                ('/move_base_simple/goal', '/go_to'),
                ],
    )

    return LaunchDescription([
        declare_sim,
        gz_sim,
        map_server,
        lifecycle_manager,
        static_tf,
        rviz2,
        path_planner,
        controller,
    ])

# reinitialize global localization within amcl
# pass empty service message
# request no motion update (spam that until figure out where are) (check covarience)
# seperate node to call the two services (blowup)
# set up of blowup node 
    # should do a function in order to constantly send an update of request_no_motion 
        # spam that to figure out where we are
        # check the covarience within that to figure out if we are in the correct location?
    # send an empty service message in order to reinitialize global localization within amcl
    