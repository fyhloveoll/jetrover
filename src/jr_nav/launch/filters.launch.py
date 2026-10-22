"""filters.launch.py -- Nav2 costmap-filter servers for the zone masks drawn by
scripts/make_filter_mask.py (keepout = no-go zones / virtual walls, speed = speed-limit zones).

Run NEXT TO nav.launch.py in the same boot (no rebuild needed, launch by path):
  ros2 launch ~/jetrover_ws/src/jr_nav/launch/filters.launch.py \
      keepout_mask:=/home/ubuntu/jetrover_ws/maps/zones_keepout.yaml \
      speed_mask:=/home/ubuntu/jetrover_ws/maps/zones_speed.yaml
The costmaps only USE the masks if nav.launch.py was started with a params file that lists the
filters (scripts/nav_params_filters.py writes that copy; pass it as params_file:=...).
Topics produced:
  /keepout_filter_mask + /keepout_costmap_filter_info  -> nav2_costmap_2d::KeepoutFilter (global+local)
  /speed_filter_mask   + /speed_costmap_filter_info    -> nav2_costmap_2d::SpeedFilter  (global only)
                                                         -> publishes /speed_limit for controller_server
Toggle at runtime without restarting:
  ros2 param set /global_costmap/global_costmap keepout_filter.enabled false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    keepout = LaunchConfiguration('keepout_mask')
    speed = LaunchConfiguration('speed_mask')
    common = {'use_sim_time': False}

    return LaunchDescription([
        DeclareLaunchArgument('keepout_mask', default_value='/home/ubuntu/jetrover_ws/maps/zones_keepout.yaml'),
        DeclareLaunchArgument('speed_mask', default_value='/home/ubuntu/jetrover_ws/maps/zones_speed.yaml'),

        # keepout: mask value 100 -> lethal cost in the costmaps
        Node(package='nav2_map_server', executable='map_server', name='keepout_mask_server', output='screen',
             parameters=[dict(common, yaml_filename=keepout, topic_name='/keepout_filter_mask', frame_id='map')]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server', name='keepout_info_server',
             output='screen',
             parameters=[dict(common, type=0, filter_info_topic='/keepout_costmap_filter_info',
                              mask_topic='/keepout_filter_mask', base=0.0, multiplier=1.0)]),

        # speed: limit% = base + multiplier * mask value = 100 - value (type 1 = percent of max speed)
        Node(package='nav2_map_server', executable='map_server', name='speed_mask_server', output='screen',
             parameters=[dict(common, yaml_filename=speed, topic_name='/speed_filter_mask', frame_id='map')]),
        Node(package='nav2_map_server', executable='costmap_filter_info_server', name='speed_info_server',
             output='screen',
             parameters=[dict(common, type=1, filter_info_topic='/speed_costmap_filter_info',
                              mask_topic='/speed_filter_mask', base=100.0, multiplier=-1.0)]),

        # both servers are lifecycle nodes: bring them to ACTIVE
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_filters',
             output='screen',
             parameters=[dict(common, autostart=True,
                              node_names=['keepout_mask_server', 'keepout_info_server',
                                          'speed_mask_server', 'speed_info_server'])]),
    ])
