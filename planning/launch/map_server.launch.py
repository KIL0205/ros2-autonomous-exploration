import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    sim = LaunchConfiguration('sim')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description="Use simulation (true) or use robot (false).",
    )

    # map file path for simple_map
    map_file_path = os.path.join(
        get_package_share_directory('planning'), 
        'maps',
        'simple_map.yaml'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('planning'),
        'rviz',
        'lab3_config.rviz')
    

    # robot_urdf = os.path.join(
    #     get_package_share_directory('turtlebot3'),
    #     'turtlebot3_description',
    #     'urdf',
    #     'turtlebot3_burger.urdf'
    # )

    # robot_description = ParameterValue(
    #     Command(['xacro ', robot_urdf]),
    #     value_type=str
    # )

    # # Publishes /joint_states
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    # # Publishes /tf for the robot based on the URDF
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     parameters=[
    #         {'robot_description': robot_description},
    #         {'use_sim_time': True}
    #     ],
    #     output='screen'
    # )

    # map_server node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename':map_file_path}]
    )
    
    # start the node lifecyle manager
    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    ) 

    # map to odom static transform
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

    # launch rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    path_planner = Node(
        package='planning',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )


    return LaunchDescription([ # return all of the launch functions
        declare_sim,
        map_server,
        lifecycle_manager, 
        static_tf,
        joint_state_publisher,
        robot_state_publisher,
        rviz2,
        path_planner
    ])