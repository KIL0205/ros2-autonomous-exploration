import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim = LaunchConfiguration('sim')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description="Use simulation (true) or use robot (false).",
    )

    #control_share = get_package_share_directory("control")
    rviz_config_dir = os.path.join(
        get_package_share_directory('control'),
        'rviz',
        'control_pkg.rviz')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/empty_world.launch.py']),
        launch_arguments={"use_sim_time": sim}.items(),
        condition=IfCondition(sim),
    )

    rviz2 =  Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    static_transform = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments=["0.5", "0.2", "0", "0.78", "0", "0", "odom", "map"] 
            )

    controller = Node(
        package='control',
        executable='controller.py',
        name='controller',
        output='screen',
        parameters=[{'use_sim_time': sim}],
    )

    return LaunchDescription(
        [
            declare_sim,
            gz_sim,
            rviz2,
            static_transform,
            controller,
        ]
    )
