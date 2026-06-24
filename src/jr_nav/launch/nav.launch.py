import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# vendor nav2 launch + params (reference only, not modified)
NAV_PKG = '/home/ubuntu/ros2_ws/src/navigation'


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    use_teb = LaunchConfiguration('use_teb')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(NAV_PKG, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'rtabmap': 'false',
            'namespace': '',
            'use_namespace': 'false',
            'map': map_yaml,
            'use_sim_time': 'false',
            'params_file': os.path.join(NAV_PKG, 'config/nav2_params.yaml'),
            'autostart': 'true',
            'use_teb': use_teb,
        }.items(),
    )

    # bridge Nav2 /cmd_vel -> JetRover /controller/cmd_vel
    relay = Node(package='jr_teleop', executable='cmd_vel_relay',
                 name='cmd_vel_relay', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/ubuntu/jetrover_ws/maps/map_02.yaml'),
        DeclareLaunchArgument('use_teb', default_value='true'),
        nav2,
        relay,
    ])
