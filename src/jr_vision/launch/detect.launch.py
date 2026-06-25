#!/usr/bin/env python3
# Launch the jr_vision detector. Pure consumer of vendor camera topics; assumes
# a bringup + depth camera are already running (and the arm held at a known pose
# so the eye-in-hand TF is valid). Does NOT touch vendor code.
#
#   ros2 launch jr_vision detect.launch.py
#   ros2 launch jr_vision detect.launch.py rate:=4.0 annotate:=true enable_3d:=false
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate = LaunchConfiguration('rate')
    annotate = LaunchConfiguration('annotate')
    enable_3d = LaunchConfiguration('enable_3d')
    conf = LaunchConfiguration('conf')
    model = LaunchConfiguration('model_path')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='8.0'),
        DeclareLaunchArgument('annotate', default_value='true'),
        DeclareLaunchArgument('enable_3d', default_value='true'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'),
        Node(
            package='jr_vision',
            executable='detector',
            name='jr_detector',
            output='screen',
            parameters=[{
                'rate': rate,
                'annotate': annotate,
                'enable_3d': enable_3d,
                'conf': conf,
                'model_path': model,
                'base_frame': base_frame,
            }],
        ),
    ])
