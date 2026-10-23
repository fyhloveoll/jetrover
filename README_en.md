[中文](README.md) | **English**

# JetRover mobile manipulation

A **language / vision driven mobile manipulation system** developed on a real Hiwonder **JetRover**
(Jetson Orin NX 16 GB, native Ubuntu 22.04 + ROS 2 Humble). The differentiator is **engineering depth**
rather than feature count: quantified evaluation, failure handling and documented trade-offs
(see [docs/engineering_decisions.md](docs/engineering_decisions.md), D1–D6, each backed by on-robot evidence;
the decision records themselves are in Chinese).

## Demo videos

Short clips from the real robot, plus one simulation clip. GitHub does not play video inline from a README,
so each row links to the file under `media/` (click ▶; GitHub's file page has a player).

| Clip | What it shows | Length |
|---|---|---|
| **01 Mapping**<br>[▶ watch](media/01_mapping_en.mp4) | SLAM Toolbox 2D-lidar mapping of the apartment (gamepad teleop, loop closed at the start point, map exported with a custom script). | 34 s |
| **02 Keep-out zones**<br>[▶ watch](media/02_keepout_zones_en.mp4) | Hand-drawn keep-out zones (red) turned into a Nav2 costmap filter mask; the global planner routes around them. | 15 s |
| **03 Full mission**<br>[▶ watch](media/03_full_mission_en.mp4) | Full mission run, 2026-10-22: start → Nav2 to the pickup station → pre-grasp visual closed loop → grasp → carry to the delivery station → placement¹. Phone footage synced with the Foxglove map view as picture-in-picture; played at 2×. | 62 s (2×) |
| **Sim: MoveIt 2 grasp**<br>[▶ watch](media/sim_moveit_grasp_en.mp4) | ROS 2 + MoveIt 2 pick-and-place in Gazebo. Simulation only — not the real robot. | 73 s |

¹ The placement segment of clip 03 is a separate take: the robot was positioned at the mat by hand and the arm
ran a scripted put-down (`scripts/put_down.py`, no vision). The autonomous placement skill
(`scripts/task_place.py`) is still being validated on the robot; in this run the autonomous part ends after
the carry to the delivery station.

Stills:

<p>
  <a href="media/stills/01_map_apartment_loop_20260903.png"><img src="media/stills/01_map_apartment_loop_20260903.png" width="300" alt="Apartment loop map, SLAM Toolbox, 2026-09-03"></a>
  <a href="media/stills/04_keepout_zones_red.png"><img src="media/stills/04_keepout_zones_red.png" width="300" alt="Keep-out zones (red) drawn over the map"></a>
</p>

Left: apartment loop map (SLAM Toolbox, 2026-09-03) · right: keep-out zones (red) over the same map.

## Core policy
**No vendor code is modified.** Our own packages live in a separate workspace and only *call* the vendor
drivers (chassis / lidar / camera / control board / IK service); they are never edited.

## Capabilities (all validated on the robot)
- **Object-agnostic grasping**: depth-based floor segmentation detects any object lying on the floor (no class
  or colour prior); automatic height estimate, yaw alignment (cubes rotated up to ±45° are graspable), gripper
  width check, multi-angle IK reachability test, blacklisting of failed targets so the loop cannot spin forever.
- **Mobile manipulation**: targets beyond the arm's reach → drive forward / mecanum strafe to fetch →
  odometry-closed-loop return along the same path → place; targets right at the wheels are cleared by backing
  up first; a head-up survey finds and fetches targets **0.84 m** away.
- **Semantic layer**: COCO (yolo11m) + **open-vocabulary (YOLO-World)** fusion — segmentation owns geometry,
  YOLO owns naming; classes are set by text at run time ("toy block" named 6 cubes in one go) and feed the LLM layer.
- **Robustness**: lidar clearance guard (clearance check before every base move, e-stop while moving);
  **camera-verified liveness check** for the control-board lockup (during a lockup odometry and servo read-backs
  all lie — the only honest witness is a change in the camera image).
- **Quantification**: every grasp attempt is logged to CSV; `jr_report.py` produces the evaluation report
  (broken down by angle / width / colour / session).
- **Pre-grasp visual closed loop** (2026-09-04, 6/6 on the robot): after detection the arm first goes to a
  pre-grasp pose above the target, the eye-in-hand camera measures the object's offset from the gripper centre
  and corrects it, then the arm descends; on by default in mission mode (`JR_CL=1`,
  `jr_grasp_all.py grasp_closed_loop`).
- **Calibration**: AprilTag hand-eye calibration (2026-09-03) confirmed the camera bracket was bent by 16.7° in
  shipping; compensated in software (`JR_CAM_PITCH`). Yaw now comes from an RGB edge method (±1.6°). Offline
  arm-kinematics calibration concluded "joint zero offsets are not identifiable; the remaining 8 mm is compliance /
  backlash" (`kin_calib_fit.py`).
- **Deployment chain** (2026-10): one command brings up the whole stack and self-checks it
  (`mission_boot.sh` + `stack_wait.py`: topics / Nav2 action server / AMCL / runtime parameter patches / initial
  pose / buzzer, ~30 s); keep-out zones drawn by hand on the map become a Nav2 costmap filter layer
  (`zone_paint.py`); Foxglove is used for remote visualisation from Windows (`deploy/foxglove_up.sh`).

## Key results
- Grasp success rate **85 %** (22/26, 2026-07-14 benchmark; baseline 68 % on 06-30; the +17 pp came mainly from
  yaw alignment paying off — cubes rotated 25–45° reach 90 %). The 43 mm cube with ±2.5 mm tolerance remains the
  precision yardstick of the system.
- **Full task chain on the robot** (2026-07-16): Chinese voice/text command → navigate to the pickup area →
  grasp → carry 1.5 m across rooms → survey-map the delivery point (AprilTag anchors) → autonomous micro-approach →
  metric-grid placement on the mat; two cubes, two trips.
- **Real objects**: a pen (9 mm, flat) and a USB stick were named through the open-vocabulary layer, grasped and
  placed (thin elongated objects are grasped across their long axis).
- **60+ grasp cycles in one day with zero lockups** (operating discipline: one long-lived node runs the batch,
  no CLI probing while actuators move, command stream rate-limited to 10 Hz — versus one lockup every 1–2 grasps
  when the discipline was violated).
- Hand-eye / coordinate check residual **2 mm** (tape-measure ground truth vs computed, 3-point fit).
- **Pre-grasp visual closed loop validated on the robot** (2026-09-04): 6 of 6 attempts converged and lifted,
  3 judged dead-centre by eye; a far target that failed open-loop succeeded closed-loop; a constant 15 mm radial
  bias is now folded in.
- **2026-10-22 mission chain** (mission 8, see demo 03): start → Nav2 to the pickup station → closed-loop grasp
  (20 mm correction, gripper colour-pixel hold check) → carry and navigate to the delivery station — all three
  legs succeeded; autonomous placement did not — the placement skill has since been rewritten as
  `task_place.py` and awaits validation on the robot.

## Milestones
| | Scope | Status |
|---|---|---|
| M1 | bringup + teleop + RViz | ✅ |
| M2 | 2D-lidar SLAM mapping | ✅ `map_02` → apartment loop map `apt_loop_20260903` (Nav2 default), `apt_demo2_20261021` (10-21 demo) |
| M3 | Nav2 autonomous navigation | ✅ controller TEB (wrapped by RotationShim); the four root causes of the spinning / oscillation were evidenced and fixed with runtime patches (D6, `stack_wait.py`); TEB vs MPPI still open; keep-out filter layer live (10-21) |
| M4 | detection + depth → 3D (evolved into object-agnostic segmentation) | ✅ |
| M4.5 | hand-eye / coordinate calibration check | ✅ 2 mm |
| M5 | grasp state machine + quantification | ✅ 68 % @ 31 |
| M6 | mobile manipulation + navigation mission (`jr_mission` carry) | ✅ two cubes, two trips "navigate → grasp → carry → cross rooms → place on the mat" on the robot (07-16); smoothness work ongoing |
| M7 | quantified evaluation (Gazebo evaluated and shelved, D2) | ✅ 22/26 = **85 %** (07-14, +17 pp vs baseline; docs/benchmark_2026-07-14.md) |
| M8 | productisation (auto-start / relay / docs) | 🟡 one-shot boot + self-check `mission_boot.sh` / `stack_wait.py` in use on the robot (10-21); systemd auto-start not installed yet; power relay waiting for parts (D1) |
| M9 | language → task | ✅ v1 multi-turn command of the real robot (Chinese → parse → execute); Claude API backend ready, key not configured |
| M10 | placement skill rewrite (`task_place.py`: dark mat first, AprilTags optional, drives itself into reach, keeps the cube on failure, exit code = result) | 🟡 code done and reviewed 10-22, **not yet validated on the robot** |

## Main scripts (scripts/)
| Script | Purpose |
|---|---|
| `jr_grasp_all.py` | clear-the-floor state machine (`run` / `survey` / `grasp1` / `drivetest`; env switches `JR_CL` (default 1, pre-grasp closed loop) / `JR_YOLO` / `JR_TARGET` / `JR_CARRY` / `JR_MAX_WIDTH`; the old `place` mode is retired, see `task_place.py`) |
| `jr_detect_objects.py` | object-agnostic instance detection + YOLO / open-vocabulary semantic fusion |
| `jr_mission.py` | navigate → grasp → deliver orchestration (`record` waypoints / `run` a mission.json; placement calls `task_place.py`, a non-zero exit keeps the cube and stops the mission) |
| `mission_boot.sh` / `stack_wait.py` / `mission_launch.sh` / `stop_mission.sh` | one-shot stack boot + self-check (READY in ~30 s) / start the mission / stop it by PID |
| `task_place.py` / `place_skill.py` | placement skill (dark mat first, AprilTags optional; `--preflight / --look-only / --dry-run / --handin`; exit code = result; 10-22, awaiting robot validation) |
| `zone_paint.py` / `nav_params_filters.py` / `make_filter_mask.py` | draw keep-out zones by hand in a paint program → Nav2 costmap filter mask / Nav2 params with the filter layers |
| `nav_goto.py` / `cost_at.py` / `setp.py` / `initpose.py` | send a navigation goal from rclpy (the `ros2` CLI hangs on this robot) / costmap + AMCL pose probe / runtime parameters / initial pose |
| `handeye_calib.py` / `yaw_calib.py` / `kin_calib_fit.py` | AprilTag hand-eye calibration / yaw calibration / offline arm-kinematics calibration |
| `station_dock.py` | precise docking on an AprilTag station board (experimental; no longer used by the mission) |
| `jr_report.py` | grasp statistics CSV → evaluation report |
| `jr_talk.py` | Chinese command → task (M9; rule-based parser as fallback, set `ANTHROPIC_API_KEY` to upgrade to LLM parsing) |
| `calib_measure.py` / `calib_fit.py` | hand-eye calibration check (capture on the robot / fit on the laptop) |
| `mjpeg_stream.py` | live camera view in the browser with detection overlay (:8080) |
| `scan_probe.py` | lidar mounting-yaw calibration |

## Repository layout
```
src/       our ROS 2 packages (jr_bringup / jr_teleop / jr_slam / jr_nav / jr_vision / jr_interfaces /
           jr_teleop_cpp = rclcpp port of cmd_vel_relay, a C++ learning exercise, not launched yet)
scripts/   grasp / mission / placement / calibration / evaluation / diagnostic scripts (deployed flat to ~/jetrover_ws/ on the robot)
deploy/    systemd auto-start (jr-stack.service + jr_stack.sh), grasp_up / slam_up / map_session, foxglove_up (remote visualisation)
firmware/  STM32 control-board IWDG watchdog (145 lines of C, D1 investigation, not flashed)
docs/      design docs, engineering decision records, hand-over notes and post-mortems (architecture_review_2026-10-22.md)
maps/      SLAM maps (apt_loop_20260903 = Nav2 default; apt_demo2/3_20261021; zones_keepout = keep-out mask)
media/     demo videos (English captions) and stills
test_data/ depth / colour samples for offline detection tests
rviz/      RViz configs (remote viewing is now mostly done with Foxglove)
```

## Running (on the robot, after `source ~/jetrover_ws/jr_env.sh`)
```
# The three-piece grasp stack:
ros2 launch jr_bringup robot.launch.py enable_camera:=true
ros2 launch kinematics kinematics_node.launch.py
python3 jr_grasp_all.py run          # clear the floor; pre-grasp closed loop on by default (JR_CL=1); semantic: JR_YOLO=1 JR_TARGET=banana
# Mission (navigate → grasp → deliver): one command boots and self-checks the stack, then start the mission
bash mission_boot.sh 1.0 0.0 0       # initial pose x y yaw(deg); READY in ~30 s, confirmed by the buzzer
bash mission_launch.sh               # jr_mission.py run starts after 10 s; bash stop_mission.sh stops it by PID
# Placement skill on its own (cube already in the gripper):
python3 task_place.py --preflight    # then --look-only / --dry-run / the real thing; exit codes in the file header
# Mapping / teleop / visualisation: deploy/map_session.sh, deploy/foxglove_up.sh; everyday commands in CLAUDE.md
```

## Key lessons (condensed; details in docs/engineering_decisions.md and the design docs)
- **Control-board lockup**: dense or malformed serial commands can lock the MCU ("commands dead, telemetry
  alive"); software cannot recover it, the only cure is a power cycle. Countermeasures: operating discipline
  (above) + camera-verified liveness check + automatic power relay (M8); the IWDG firmware watchdog was
  investigated and re-evaluated but downgraded to an optional improvement (D1).
- **During a lockup no telemetry can be trusted**: `servo_states` echoes the commanded targets and odometry
  integrates the commanded velocity (the board did not move at all yet reported "travelled 0.073 m" — the
  decisive experiment). Execution is confirmed only by a physical witness.
- **Raw camera over 2.4 GHz Wi-Fi**: choose the **compressed** transport in rqt (raw 30 Hz frames get
  fragmented by DDS and drop to 1 fps); use `rqt_image_view` (the local `image_view` cannot bind the compressed
  subscription).
- **Use Foxglove for remote viewing** (`deploy/foxglove_up.sh`, ws://robot:8765) and **never subscribe to the raw
  camera image**: the bridge burns 170 % CPU and the navigation control loop drops to 2 Hz (10-21).
- **After boot, NTP jumps the Jetson's clock by 80+ minutes in one step**: waits and timeouts must use
  `time.monotonic()`, and the stack should wait for clock sync before launching (10-21).
- **Cross-machine clock skew** ~16 ms: an initial pose published from RViz can be rejected by AMCL; send
  navigation goals from the robot side (`jr_mission` does).
- **The 2D lidar only covers the front ~180°** (the rear half is cropped by the vendor); turn often while mapping
  to refresh the view.
- **The active controller is TEB (wrapped by RotationShim)**; the early "use DWB" note was wrong (D6). The
  spinning came from TEB configured as a differential drive on a mecanum base plus a per-axis speed clamp
  outside the control loop; the fix is pushing the limits into the planner parameters and runtime patches
  (`stack_wait.py`). Do not add keep-out zones to the local costmap — the footprint gets trapped in the inscribed
  ring and every trajectory becomes "infeasible" (10-21).
- SLAM: loop closure must be on at apartment scale (`do_loop_closing: true`, the 09-03 loop map); it can be off
  in a single small room. Turn slowly in segments; pass bed-like "hollow underneath, thin legs" furniture at a
  distance and constant speed.
- **Eye-in-hand**: with the servos unpowered the arm sags under its own weight → the camera extrinsics are
  unknown; before any perception, command the arm to a known pose and hold it. The camera bracket was bent 16.7°
  in shipping; compensated with `JR_CAM_PITCH` and confirmed by the AprilTag hand-eye calibration (09-03).
- **The cube in the gripper hides the lower-middle of the eye-in-hand image**: when looking for the mat or the
  station board during placement, stop ≥ 0.9 m away (the cause of three failed staged takes on 10-22); run
  probes with the cube in the gripper — an empty-gripper probe is misleading.
