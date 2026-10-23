# ROS 2 packages (source of truth = robot `~/jetrover_ws/src`)

Our own packages; none modify vendor code (vendor drivers are only *called*). Edit on the robot,
`rsync` back here, commit.

- `jr_bringup`     — clean chassis + lidar + camera bringup (calls the vendor drivers); `robot.launch.py enable_camera:=true`
- `jr_teleop`      — `keyboard_teleop`, `joy_teleop` (gamepad), `cmd_vel_relay` (/cmd_vel → /controller/cmd_vel,
                     proportional speed cap — the last hop of every Nav2 command, see docs/engineering_decisions.md D6)
- `jr_teleop_cpp`  — rclcpp port of `cmd_vel_relay` (C++ learning exercise, 68 lines; not launched anywhere yet)
- `jr_slam`        — slam_toolbox 2D mapping (`slam.yaml`, loop closure **on** for the apartment loop map)
- `jr_nav`         — Nav2 bringup on top of the vendor navigation launch: `nav.launch.py` (args `map`, default
                     `maps/apt_loop_20260903.yaml`; `params_file`; `use_teb`) and `filters.launch.py`
                     (keepout / speed costmap-filter mask servers)
- `jr_vision`      — `detector`, `grasp`, `scene` nodes (early M4/M5 vision pipeline; the current grasp stack
                     lives in `scripts/jr_grasp_all.py` + `scripts/jr_detect_objects.py`)
- `jr_interfaces`  — action definitions (`action/`)
