#!/usr/bin/env python3
# encoding: utf-8
"""nav_params_filters.py -- make OUR copy of the vendor nav2 params with costmap filters added
(zero vendor modification: the vendor yaml is read, never written).

  python3 nav_params_filters.py [VENDOR_YAML] [OUT_YAML] [--no-speed] [--mppi]
    VENDOR_YAML  default /home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml
    OUT_YAML     default ~/jetrover_ws/nav2_params_zones.yaml
    --no-speed   keepout filter only
    --mppi       also swap controller_server.FollowPath to nav2_mppi_controller (A/B candidate;
                 needs `ros2 pkg list | grep nav2_mppi_controller`; if the vendor launch loads the
                 controller params from a hard-coded yaml instead of params_file, this part is inert)

Then:  ros2 launch jr_nav nav.launch.py params_file:=/home/ubuntu/jetrover_ws/nav2_params_zones.yaml
       ros2 launch ~/jetrover_ws/src/jr_nav/launch/filters.launch.py

What is added (Nav2 Humble names):
  global_costmap: filters: [keepout_filter, speed_filter] + plugin blocks
  local_costmap : filters: [keepout_filter]                (speed filter belongs in ONE costmap only)
  controller_server: speed_limit_topic: speed_limit        (SpeedFilter -> controller max speeds)
Filters start DISABLED (enabled: false) so the plain navigation baseline is filmed first; switch on with
  ros2 param set /global_costmap/global_costmap keepout_filter.enabled true   (same for local, speed)
"""
import os
import sys

import yaml

VENDOR = '/home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml'
OUT = os.path.expanduser('~/jetrover_ws/nav2_params_zones.yaml')

KEEPOUT = {'plugin': 'nav2_costmap_2d::KeepoutFilter', 'enabled': False,
           'filter_info_topic': '/keepout_costmap_filter_info'}
SPEED = {'plugin': 'nav2_costmap_2d::SpeedFilter', 'enabled': False,
         'filter_info_topic': '/speed_costmap_filter_info', 'speed_limit_topic': '/speed_limit'}

# MPPI for a mecanum base: omni motion model, speeds inside the relay/smoother caps (0.15 m/s, 0.6 rad/s),
# PreferForward off so sideways motion is not penalised. Starting point only -- tune on the robot.
MPPI = {
    'plugin': 'nav2_mppi_controller::MPPIController',
    'time_steps': 40, 'model_dt': 0.05, 'batch_size': 1000,
    'vx_std': 0.1, 'vy_std': 0.1, 'wz_std': 0.3,
    'vx_max': 0.15, 'vx_min': -0.08, 'vy_max': 0.15, 'wz_max': 0.6,
    'iteration_count': 1, 'prune_distance': 1.2, 'transform_tolerance': 0.2,
    'temperature': 0.3, 'gamma': 0.015, 'motion_model': 'Omni', 'visualize': False, 'reset_period': 1.0,
    'TrajectoryVisualizer': {'trajectory_step': 5, 'time_step': 3},
    'critics': ['ConstraintCritic', 'ObstaclesCritic', 'GoalCritic', 'GoalAngleCritic',
                'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic'],
    'ConstraintCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 4.0},
    'GoalCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'threshold_to_consider': 1.0},
    'GoalAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 3.0, 'threshold_to_consider': 0.4},
    'ObstaclesCritic': {'enabled': True, 'cost_power': 1, 'repulsion_weight': 1.5, 'critical_weight': 20.0,
                        'consider_footprint': False, 'collision_cost': 10000.0,
                        'collision_margin_distance': 0.05, 'near_goal_distance': 0.5},
    'PathAlignCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 14.0, 'max_path_occupancy_ratio': 0.05,
                        'trajectory_point_step': 3, 'threshold_to_consider': 0.5, 'offset_from_furthest': 20,
                        'use_path_orientations': False},
    'PathFollowCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 5.0, 'offset_from_furthest': 5,
                         'threshold_to_consider': 1.0},
    'PathAngleCritic': {'enabled': True, 'cost_power': 1, 'cost_weight': 2.0, 'offset_from_furthest': 4,
                        'threshold_to_consider': 0.5, 'max_angle_to_furthest': 1.0, 'mode': 0},
}


def node_params(doc, *names):
    """return the ros__parameters dict of the first node name found (nav2 layout: node -> ros__parameters,
    costmaps: outer -> inner -> ros__parameters); None if absent"""
    for n in names:
        d = doc.get(n)
        if isinstance(d, dict):
            if 'ros__parameters' in d:
                return d['ros__parameters']
            inner = d.get(n)
            if isinstance(inner, dict) and 'ros__parameters' in inner:
                return inner['ros__parameters']
    return None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    src = args[0] if len(args) > 0 else VENDOR
    dst = args[1] if len(args) > 1 else OUT
    with_speed = '--no-speed' not in sys.argv
    with_mppi = '--mppi' in sys.argv
    doc = yaml.safe_load(open(src))
    changes = []

    g = node_params(doc, 'global_costmap')
    if g is None:
        print('WARNING: global_costmap not found in %s -- layout differs, inspect the file' % src)
    else:
        g['filters'] = ['keepout_filter'] + (['speed_filter'] if with_speed else [])
        g['keepout_filter'] = dict(KEEPOUT)
        if with_speed:
            g['speed_filter'] = dict(SPEED)
        changes.append('global_costmap.filters = %s' % g['filters'])

    l = node_params(doc, 'local_costmap')
    if l is None:
        print('WARNING: local_costmap not found')
    else:
        l['filters'] = ['keepout_filter']
        l['keepout_filter'] = dict(KEEPOUT)
        changes.append('local_costmap.filters = [keepout_filter]')

    c = node_params(doc, 'controller_server')
    if c is None:
        print('WARNING: controller_server not found (vendor may load it from another yaml)')
    else:
        if with_speed:
            c['speed_limit_topic'] = 'speed_limit'
            changes.append('controller_server.speed_limit_topic = speed_limit')
        if with_mppi:
            old = c.get('FollowPath', {}).get('plugin', '?')
            c['controller_plugins'] = ['FollowPath']
            c['FollowPath'] = dict(MPPI)
            changes.append('controller_server.FollowPath: %s -> nav2_mppi_controller::MPPIController' % old)

    with open(dst, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False, allow_unicode=True)
    print('wrote %s' % dst)
    for ch in changes:
        print('  + ' + ch)
    print('check on the robot that the vendor bringup really feeds params_file to these nodes:')
    print("  grep -n \"params_file\\|yaml\" /home/ubuntu/ros2_ws/src/navigation/launch/include/bringup.launch.py")


if __name__ == '__main__':
    main()
