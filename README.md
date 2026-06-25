# JetRover 移动抓取系统

在 Hiwonder **JetRover**(Jetson Orin NX 16GB,原生 Ubuntu 22.04 + ROS2 Humble)真机上开发的
**语言/视觉驱动移动抓取系统**。北极星:用工程深度(量化评估、鲁棒性、sim-to-real)做差异化,
而非堆功能。

## 核心策略
**不修改厂家 vendor demo 代码**;在干净的自有 workspace `~/jetrover_ws` 里建自己的包,
只**调用** vendor 驱动(底盘/雷达/相机/控制板),从不编辑它们。

## 硬件 / 系统
- 主控 Jetson Orin NX 16GB,Ubuntu 22.04 + ROS2 Humble(无 Docker)
- 麦克纳姆轮底盘、G4 激光雷达、Dabai DCW 深度相机、5+1 DOF 机械臂
- 开发机:HP OMEN 笔记本(RTX 4060,跑 Gazebo 仿真),SSH 连机器人开发

## 里程碑
| | 内容 | 状态 |
|---|---|---|
| M1 | bringup + 遥控(键盘/手柄)+ RViz | ✅ |
| M2 | 2D 雷达 SLAM 建图(slam_toolbox) | ✅ `map_02` |
| M3 | Nav2 导航 | 🟡 软件全栈验证通过(`Goal succeeded`),待板子稳时跑一次物理移动收尾 |
| M4 | YOLO + 深度 → 3D 抓取位姿 | 🟡 M4.0-M4.3 全通(`jr_vision` 包:YOLO→深度→base_link 3D) |
| M5 | 抓取状态机(检测→接近→抓→放,失败检测/重试) | ⬜ |
| M6 | 整合(导航→检测→抓取→送达) | ⬜ |
| M7 | 量化评估 + Gazebo 数字孪生 sim-to-real(笔记本) | ⬜ |
| M8 | 工程化(systemd、电池监控、多地图、文档/测试) | ⬜ |
| M9 | LLM 自然语言→任务流(选做) | ⬜ |

## 仓库结构
```
src/      自有 ROS2 包(jr_bringup / jr_teleop / jr_slam / jr_nav / jr_vision),源头在机器人 ~/jetrover_ws/src
maps/     SLAM 地图(map_01 首张/质量一般,map_02 3楼主卧/质量好,推荐)
rviz/     笔记本端 RViz 配置(jr_view 看传感器 / jr_slam 看建图 / jr_nav 看导航)
```

## 运行手册(机器人上,先 `source ~/jetrover_ws/jr_env.sh`)
```
# 底盘 + 雷达 + 相机
ros2 launch jr_bringup robot.launch.py
# 遥控(二选一):键盘 / 手柄(手柄走控制板 /ros_robot_controller/joy)
ros2 run jr_teleop keyboard_teleop
ros2 run jr_teleop joy_teleop --ros-args -p max_linear:=0.2 -p max_angular:=0.4
# 建图
ros2 launch jr_slam slam.launch.py
./save_map.sh map_02            # 存 pgm/yaml 到 maps/
# 导航(关 slam,用 DWB)
ros2 launch jr_nav nav.launch.py use_teb:=false
```
笔记本看图:`source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=0; rviz2 -d rviz/jr_nav.rviz`

## 视觉 / 看相机(M4)
```
# 机器人:相机 + YOLO 检测 + 深度→3D 抓取点(需先起 bringup;臂 hold 在已知位姿以保证 eye-in-hand TF)
ros2 launch peripherals depth_camera.launch.py            # 相机(单独起,不经控制板)
ros2 launch jr_vision detect.launch.py                    # YOLO 检测节点(可选 rate:= annotate:= enable_3d:=)
# 笔记本:一条命令看画面(强制 compressed 传输,2.4G 也清晰流畅)
scripts/view.sh            # 原生相机 RGB(零额外负载)
scripts/view.sh yolo       # YOLO 标注流 /jr/camera/annotated(需 detect 在跑)
```
- **关键:2.4G wifi 上原生 30Hz raw 大帧被 DDS 分片丢成 ~1fps**——看原生相机要在 rqt 传输下拉框选 **compressed**(rqt 会记住)。jr_vision 标注流跑 4-8Hz,raw 即可看。
- `view.sh` 用 **`rqt_image_view`**(本机唯一可靠出窗口的 viewer)。⚠️ 别用 `image_view`:本机它的 compressed image_transport 订阅绑不上(Subscription=0)、不出窗口,连 vendor 流也这样。
- **笔记本依赖(已装一次性):** `ros-humble-compressed-image-transport`(rqt 解码压缩必需)。(`ros-humble-image-view` 也装了但本机不可用。)
- `jr_vision` 发布:`/jr/camera/annotated`(+`/compressed`)、`/jr/grasp/target`(base_link PointStamped)、`/jr/grasp/markers`(RViz)。

## 抓取(M5,🚧 真机未验证)
```
# 前置:bringup + 相机 + kinematics 服务,臂 hold 在观察位姿(eye-in-hand FK 有效)
ros2 launch kinematics kinematics_node.launch.py          # vendor IK/FK 服务
ros2 launch jr_vision grasp.launch.py                     # 抓取节点(默认 dry_run,服务模式)
ros2 service call /jr/grasp/trigger std_srvs/srv/Trigger  # 触发一次抓取(dry-run 只算不动)
ros2 launch jr_vision grasp.launch.py dry_run:=false      # 真实执行(下探→夹→抬→回)
```
- **手眼管线**(照 vendor `track_and_grab` 约定,自写不改 vendor):像素+对齐深度→相机系→`HAND2CAM`(相机→手爪固定标定)`@` FK 末端位姿(`/kinematics/get_current_pose`)→**臂基座系**→IK(`/kinematics/set_pose_target`)→舵机脉冲。**不走 TF 树。**
- **可达区**(ik_probe 实测):臂前向 x≈0.10~0.30m、z≥0.00;**地面物体须在车前 <30cm**,否则 IK 无解(移动抓取要先靠近)。
- **目标物**:不透明 + COCO 类(可乐→cup、塑料瓶→bottle);玻璃=深度无效抓不了。
- **加固**(板子锁死教训):舵机指令限流、`/imu` 静默即判板子卡死并中止、发指令前等舵机桥订阅、抓取前张爪、MultiThreadedExecutor 干净关闭。
- ⚠️ **控制板可靠性未解决**:密集舵机指令/硬杀串口驱动仍可能锁死 MCU(≠button_scan),只能 reboot 恢复;已反馈厂家。

## 已知问题
- **控制板偶发卡死**:通信过程中 USB-串口掉线,IMU/电机/编码器同时哑,重启机器人恢复。
  待查 `dmesg` 定因,目标做"检测到 IMU 静默→软重置 USB"的自动恢复看门狗。
- **跨机时钟差**:笔记本比机器人超前 ~16ms,RViz 设初始位姿可能被 AMCL 拒(`extrapolation`)。
- **2D 雷达 `/scan` 只有前方 ~180°**(后半被 vendor 滤波裁掉);建图时多转身刷视野。
- **TEB 局部规划器不可行**,改用 **DWB**。

## SLAM 调参教训(小房间)
`do_loop_closing: false`(关回环,防误闭合把整图拽乱);慢速、转身分段;
床等"床底空+腿细"的家具当墙,远距离匀速过、勿凑近勿在旁转身。
