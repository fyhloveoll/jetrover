#!/usr/bin/env python3
# Launch the jr_vision grasp node. Assumes bringup + depth camera + the kinematics
# service are already running, and the arm is held at a known observe pose so the
# eye-in-hand FK endpoint is valid. Pure consumer of vendor services; does NOT
# touch vendor code.
#
#   ros2 launch jr_vision grasp.launch.py                       # service mode, dry-run
#   ros2 launch jr_vision grasp.launch.py dry_run:=false        # service mode, will actuate
#   ros2 launch jr_vision grasp.launch.py auto_grab:=true dry_run:=true   # one-shot dry-run
#   ros2 service call /jr/grasp/trigger std_srvs/srv/Trigger    # trigger a grasp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dry_run = LaunchConfiguration('dry_run')
    auto_grab = LaunchConfiguration('auto_grab')
    conf = LaunchConfiguration('conf')
    model = LaunchConfiguration('model_path')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('auto_grab', default_value='false'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/ubuntu/third_party/yolo/yolov11/yolo11n.pt'),
        Node(
            package='jr_vision',
            executable='grasp',
            name='jr_grasp',
            output='screen',
            parameters=[{
                'dry_run': dry_run,
                'auto_grab': auto_grab,
                'conf': conf,
                'model_path': model,
            }],
        ),
    ])
