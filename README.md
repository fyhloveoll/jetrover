# JetRover 移动抓取系统

在 Hiwonder **JetRover**(Jetson Orin NX 16GB,原生 Ubuntu 22.04 + ROS2 Humble)真机上开发的
**语言/视觉驱动移动抓取系统**。差异化靠**工程深度**:量化评估、失败处理、多方案权衡
(见 [docs/engineering_decisions.md](docs/engineering_decisions.md) D1–D5,每条含真机证据),
而非堆功能。

## 核心策略
**不修改厂家 vendor 代码**;在自有 workspace 里建包,只**调用** vendor 驱动
(底盘/雷达/相机/控制板/IK 服务),从不编辑它们。

## 能力一览(全部真机验证)
- **物体无关抓取**:深度地面分割检测任意落地物体(不依赖类别/颜色),自动判高、
  偏航对齐(±45° 斜置可抓)、夹爪宽度校验、多角度 IK 可达判定、失败拉黑防死循环
- **移动抓取**:超出臂展 → 自动前进/麦轮横移取货 → 里程计闭环原路返回 → 放置;
  贴轮超近目标先倒车让位;抬头扫视发现 **0.84m** 外目标并取回放置
- **语义层**:COCO(yolo11m)+ **开放词表(YOLO-World)** 融合——分割管几何、YOLO 管命名,
  运行时文字定类("toy block" 一次点名 6 个方块),为 LLM 层供词汇
- **鲁棒性**:雷达净空防护(移动前查净空/行进中急停);控制板锁死的**相机自证**检测
  (锁死时 odom/舵机回读都会说谎,唯一诚实证人=相机画面变化)
- **量化**:每次抓取入 CSV,`jr_report.py` 出报告(按角度/宽度/颜色/场次分解)

## 关键结果
- 抓取成功率:**85%**(22/26,2026-07-14 基准;基线 68% @ 06-30,+17pp 主要来自
  偏航对齐兑现——斜放 25-45° 达 90%);43mm 大方块 ±2.5mm 容错仍为系统精度标尺
- **完整任务链真机跑通**(2026-07-16):中文指令→导航到拿取区→抓取→携带跨房 1.5m→
  交付点扫视建图(AprilTag 锚点)→自主微调接近→米制格放置上垫,双方块双趟
- **真实物体**:笔(9mm 扁平)与U盘经开放词表点名后抓取放置成功(细长物垂直长轴抓取管线)
- 单日 **60+ 抓取循环零锁死**(运行纪律:单一长命节点跑批、动作期间零 CLI 探测、
  指令流 10Hz 限流——对比违反纪律时 1–2 抓一锁)
- 手眼/坐标核对残差 **2mm**(卷尺真值 vs 计算值,3 点拟合)

## 里程碑
| | 内容 | 状态 |
|---|---|---|
| M1 | bringup + 遥控 + RViz | ✅ |
| M2 | 2D 雷达 SLAM 建图 | ✅ `map_02` |
| M3 | Nav2 自主导航 | ✅ |
| M4 | 检测 + 深度 → 3D(演进为物体无关分割) | ✅ |
| M4.5 | 手眼/坐标标定核对 | ✅ 2mm |
| M5 | 抓取状态机 + 量化 | ✅ 68% @ 31 |
| M6 | 移动抓取 + 导航任务(jr_mission carry) | ✅ 双方块双趟"导航→抓→携带→跨房→放上垫"真机跑通(07-16);丝滑度优化进行中 |
| M7 | 量化评估(Gazebo 经评估搁置,D2) | ✅ 22/26 = **85%**(07-14,较基线 +17pp;docs/benchmark_2026-07-14.md)|
| M8 | 工程化(自启/继电器/文档) | 🟡 自启脚本+一键净启动(mission_up.sh)就绪待装机;继电器待购件(D1) |
| M9 | 语言→任务 | ✅ v1 真机指挥多轮(中文→解析→执行);Claude API 后端就绪待配 key |

## 主要脚本(scripts/)
| 脚本 | 用途 |
|---|---|
| `jr_grasp_all.py` | 抓光全部状态机(`run`/`survey`;`JR_YOLO`/`JR_TARGET`/`JR_CARRY` 环控) |
| `jr_detect_objects.py` | 物体无关实例检测 + YOLO/开放词表语义融合 |
| `jr_mission.py` | 导航→抓取→送达编排(`record` 录航点 / `run` 执行 mission.json) |
| `jr_report.py` | 抓取统计 CSV → 评估报告 |
| `jr_talk.py` | 中文指令 → 任务(M9;规则解析保底,设 `ANTHROPIC_API_KEY` 自动升级 LLM 解析) |
| `calib_measure.py` / `calib_fit.py` | 手眼标定核对(真机采集 / 笔记本拟合) |
| `mjpeg_stream.py` | 浏览器实时画面(检测叠加,:8080) |
| `scan_probe.py` | 雷达安装朝向标定 |

## 仓库结构
```
src/      自有 ROS2 包(jr_bringup / jr_teleop / jr_slam / jr_nav / jr_vision)
scripts/  抓取/任务/标定/评估脚本(部署到机器人 ~/jetrover_ws/)
deploy/   systemd 自启(jr-stack.service + jr_stack.sh)
docs/     设计文档 + 工程决策记录
maps/     SLAM 地图(map_02 推荐)
rviz/     笔记本端 RViz 配置
```

## 运行(机器人上,先 `source ~/jetrover_ws/jr_env.sh`)
```
# 三件套(抓取):
ros2 launch jr_bringup robot.launch.py enable_camera:=true
ros2 launch kinematics kinematics_node.launch.py
python3 jr_grasp_all.py run          # 抓光地面(语义:JR_YOLO=1 JR_TARGET=banana)
# 任务(导航→抓→送,另需 nav):
ros2 launch jr_nav nav.launch.py
python3 jr_mission.py run
# 建图 / 遥控 / RViz 详见开发机桌面《JetRover命令速查.txt》
```

## 关键教训(浓缩;详见 docs/engineering_decisions.md 与设计文档)
- **控制板锁死**:密集/畸形串口指令可致 MCU 锁死("命令死/遥测活"),软件无法自恢复,
  唯一解药=断电。对策=运行纪律(见上)+ 相机自证检测 + 继电器自动断电(M8);
  IWDG 固件看门狗经取证翻案但按 D1 降级为可选优化。
- **锁死时全部遥测不可信**:servo_states 回显目标值、odom 积分指令速度(板子纹丝不动
  却报"走了 0.073m",决定性实验)——执行确认只信物理证人。
- **2.4G WiFi 看原生相机**:rqt 传输选 **compressed**(raw 30Hz 大帧被 DDS 分片丢成 1fps);
  viewer 用 `rqt_image_view`(本机 `image_view` 的 compressed 订阅绑不上)。
- **跨机时钟差** ~16ms:RViz 发初始位姿可能被 AMCL 拒;导航目标从机器人侧发(jr_mission 即如此)。
- **2D 雷达只有前方 ~180°**(后半被 vendor 裁掉);建图多转身刷视野。
- **TEB 局部规划器不可行,用 DWB**;小房间 SLAM 关回环(`do_loop_closing: false`),
  慢速转身分段,床类"底空腿细"家具远距匀速过。
- **eye-in-hand**:舵机不上电臂自重下垂 → 相机外参未知;任何感知前先命令臂到已知位姿 hold。
