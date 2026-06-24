import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    bringup_dir = get_package_share_directory('jr_bringup')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'robot.launch.py')),
        launch_arguments={'enable_lidar': 'false', 'enable_camera': 'false'}.items(),
    )
    joy = Node(
        package='jr_teleop', executable='joy_teleop', name='jr_joy_teleop',
        parameters=[{'max_linear': max_linear, 'max_angular': max_angular}],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('max_linear', default_value='0.25'),
        DeclareLaunchArgument('max_angular', default_value='0.6'),
        bringup,
        joy,
    ])
