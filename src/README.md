# ROS2 packages (source of truth = robot ~/jetrover_ws/src)

These are pulled from the robot next session (robot was offline when the repo
was created). Each is our own clean package; none modify vendor code.

- `jr_bringup`  — clean chassis+lidar+camera bringup (calls vendor drivers)
- `jr_teleop`   — keyboard_teleop, joy_teleop (gamepad), cmd_vel_relay
- `jr_slam`     — slam_toolbox 2D mapping (own slam.yaml; loop closure off for small rooms)
- `jr_nav`      — Nav2 bringup with map_02 + /cmd_vel -> /controller/cmd_vel relay
