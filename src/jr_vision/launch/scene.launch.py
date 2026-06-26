#!/usr/bin/env python3
# Launch the jr_vision scene node (class-agnostic floor-removal segmentation).
# CAMERA ONLY -- needs the depth camera running, does NOT touch the control board.
#   ros2 launch peripherals depth_camera.launch.py     # camera (no board)
#   ros2 launch jr_vision scene.launch.py              # segmentation + annotated stream
#   scripts/view.sh /jr/scene/annotated                # view on the laptop
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate = LaunchConfiguration('rate')
    max_dist = LaunchConfiguration('max_dist')
    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='3.0'),
        DeclareLaunchArgument('max_dist', default_value='0.7'),
        Node(
            package='jr_vision',
            executable='scene',
            name='jr_scene',
            output='screen',
            parameters=[{'rate': rate, 'max_dist': max_dist}],
        ),
    ])
