#!/usr/bin/env python3
# Clean JetRover hardware bringup.
# = vendor bringup.launch.py MINUS demo apps / rosbridge / web_video_server / joystick.
# We *include* the vendor hardware launch files by their source paths (need_compile=False
# convention) and do NOT modify any vendor code.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# vendor source paths (exist on the robot; used because need_compile=False)
CONTROLLER_PKG = '/home/ubuntu/ros2_ws/src/driver/controller'
PERIPHERALS_PKG = '/home/ubuntu/ros2_ws/src/peripherals'


def generate_launch_description():
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    enable_odom = LaunchConfiguration('enable_odom', default='true')

    # chassis core: ros_robot_controller (board) + odom_publisher + imu_filter + ekf + servo + URDF/TF
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(CONTROLLER_PKG, 'launch/controller.launch.py')),
        launch_arguments={'enable_odom': enable_odom}.items(),
    )

    # G4 lidar -> scan_raw, laser_filters -> /scan
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PERIPHERALS_PKG, 'launch/lidar.launch.py')),
        condition=IfCondition(enable_lidar),
    )

    # Dabai depth camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(PERIPHERALS_PKG, 'launch/depth_camera.launch.py')),
        condition=IfCondition(enable_camera),
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_odom', default_value='true'),
        controller_launch,
        lidar_launch,
        camera_launch,
    ])
