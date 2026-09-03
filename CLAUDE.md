# CLAUDE.md — JetRover 移动抓取项目

给任何在这个仓库里工作的 Claude 会话（Windows 桌面版 / iPad 云端 / 将来 Jetson 上）看的上下文。
用户是零基础学 C++ 的机器人求职者，中文交流；命令、代码、报错保持英文原样。

## 这是什么

Hiwonder **JetRover**（Jetson Orin NX 16GB，原生 Ubuntu 22.04 + ROS2 Humble）上的
语言/视觉驱动移动抓取系统。真机成果见 README。**语言构成：Python 约 5300 行，C++ 0 行**
（`firmware/watchdog/` 是 145 行 C）。`src/` 里 4 个 ament_cmake 包只有 launch/config，
2 个 ament_python 包（`jr_teleop`、`jr_vision`）才有节点代码。

## 铁律

1. **不改 vendor 代码**。车上 `~/ros2_ws`（厂家）和 `~/third_party/` 只读；自己的东西全在
   `~/jetrover_ws`。vendor launch 硬编码的参数用**运行时 `ros2 param set`** 覆写（见 `scripts/mission_up.sh` 3.5 步）。
2. **车上同一时刻只能有一个 bringup**（串口独占，双开会让控制板固件锁死）。
   起任何 launch 前先 `ros2 topic info /controller/cmd_vel` 看 Publisher 数。
3. **停进程只按记账 PID 发 SIGTERM，禁止 `pkill -f`**（ssh 包装命令含同字符串会连环误杀）。
4. **跑批 = 单一长命节点，动作期间零 CLI 探测**。短命进程连发会把板子搞锁死。
5. `ros2 topic pub` 必须 `--times N -r 10`，`timeout 1.3` 会在 discovery 完成前 kill。
6. **`/controller_manager/servo_states` 回显的是目标值不是实际位置**，不能当运动证据。
   板子锁死的唯一诚实证人是相机画面变化 / 蜂鸣器 / 人眼。
7. 传感器话题一律 sensor QoS（best effort）。

## 车的地址与登录

- 公寓 Wi‑Fi：`ubuntu@10.5.1.200`（NetworkManager 存档自连）。IP 漂了按 Wi‑Fi MAC `48:8f:4c` 扫网。
- 连不上网车会自开热点 **`HW-25058615` / `hiwonder`**，车在 `192.168.149.1`；长按扩展板 KEY1 5–10 s 强制热点。
- user / password / sudo 均 `ubuntu`。**登录 shell 是 zsh**，远程跑 ROS 必须包 `bash -lc`：
  ```
  ssh ubuntu@10.5.1.200 'bash -lc "source ~/jetrover_ws/jr_env.sh; ros2 topic list"'
  ```
  交互式手敲用 `source ~/jetrover_ws/jr_env.zsh`。`ROS_DOMAIN_ID=0`。
- 引号地狱：远程命令输出先写文件再 `tail`，别在 `bash -lc "..."` 里嵌套双引号。
- 后台起节点：`setsid ros2 launch ... >~/x.log 2>&1 </dev/null & echo $!`，PID 记下来。
- 出厂自启 `start_app_node.service`、`button_scan.service` 已永久 disable，别 enable 回去。

## 开发机（2026-09 起）

Windows 11 笔记本，Claude Code 桌面版。**没有 Linux 侧 RViz 了**：
- 终端：从 WSL2（Ubuntu-22.04，用户 `fyh`）`ssh jetrover`（`~/.ssh/config` 已配，指向 10.5.1.200）。
  从 Windows 侧调用：`wsl.exe -d Ubuntu-22.04 -- bash -lc '...'`，路径写绝对路径。
- 可视化：车上 `bash ~/jetrover_ws/foxglove_up.sh` 起 foxglove_bridge，Windows 的 Foxglove 桌面版连
  `ws://10.5.1.200:8765`。不在 WSL 装 ROS2。
- 双系统 Ubuntu（Disk 0 分区 5）留作兜底：刷 STM32、USB 硬件调试。它的 Claude 记忆已拷到
  `Desktop\C++\linux_claude\home\fyh\.claude\projects\-home-fyh\memory\`。
- **代码流向**：车上 `~/jetrover_ws/src` 和 `~/jetrover_ws/*.py` 是运行源头；改完 `rsync` 回仓库再 push。
  `scripts/` 里的脚本部署到车上 `~/jetrover_ws/`（同名，平铺）。

## 车上常用命令（先 `source ~/jetrover_ws/jr_env.sh`）

```
# 三件套（抓取）
ros2 launch jr_bringup robot.launch.py enable_camera:=true      # 底盘+雷达+相机+舵机桥
ros2 launch kinematics kinematics_node.launch.py                 # IK/FK 服务（是 launch，不是 run）
python3 jr_grasp_all.py run                                      # 抓光地面
# 导航 / 任务
ros2 launch jr_nav nav.launch.py                                 # map 默认 map_02（美国公寓旧图作废，需重建）
bash ~/jetrover_ws/mission_up.sh                                 # 一键净启动全栈（PID 清扫+体检+TEB 参数覆写）
python3 jr_mission.py run
# 建图 / 遥控
ros2 launch jr_slam slam.launch.py                               # slam_toolbox
ros2 run jr_teleop keyboard_teleop                               # 发 /controller/cmd_vel，直达电机
bash ~/jetrover_ws/save_map.sh <name>
# 健康
ros2 topic hz /odom_raw                                          # 板→主机活着 ~30-48Hz
python3 buzz.py                                                  # 命令通路：蜂鸣两声
```

## 硬件事实

- 控制板 `/dev/rrc→ttyACM0`，雷达 `/dev/lidar→ttyUSB0`，相机 Dabai DCW（USB）。
- `/scan` 被 laser_filters 裁到车头前方约 180°，后半圈盲区（倒车看不见）。
- 舵机 ID 1–5 = 臂，10 = 夹爪（安全范围 200–700，>700 撞限位），脉冲 0–1000 = 0–240°。
  观察位姿 `(1,500),(2,700),(3,15),(4,175),(5,500)`，夹爪 200 开 / 540 闭。
- 电池 11.1 V 6000 mAh，重载约 1.5–2 h；**低电压时板子锁死概率显著升高**，跑真机前充满。
- 手柄走控制板 `/ros_robot_controller/joy`（非 USB HID）；USB 接收器 2026-08 起失踪。

## 已知诊断结论（真机取证，别再重新猜）

- **导航打转根因**：Nav2 活动控制器是 **TEB**（RotationShim 包裹），但按差速车配置
  （`max_vel_y=0`、`weight_kinematics_nh=1000`），加 velocity_smoother 的 vy 上限 0，麦轮横移被禁用，
  侧向纠偏只能前进-转-后退。修复在自有 params 副本 + `mission_up.sh` 运行时 param set。
  `cmd_vel_relay` 已改等比缩放（保方向/曲率）。**回归验证等新地图。**
- **偏航角不是 YOLO 算的**，是深度分割轮廓 `cv2.minAreaRect`；残差嫌疑是夹爪机械中立位偏置和透视压缩。
- 控制板锁死根因（2026-06）：厂家 `button_scan.service` 与 bringup 双开串口，已 disable 后 60+ 循环零锁死。
- NVIDIA LocateAnything 不采用（无 mask、非商用许可）。

## 文档

- `docs/engineering_decisions.md` D1–D6：每条工程决策带真机证据。
- `docs/benchmark_2026-07-14.md`：85% 基准的口径。
- `docs/M4/M5/M6_*_design.md`：检测 / 抓取 / 任务设计。
