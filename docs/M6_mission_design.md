# M6 终章:导航→抓取→送达 任务编排(jr_mission.py)

> 2026-07-01 离线定稿,待真机验证。M6 核心(倒车让位/取货/横移/survey/返程/放置)已于
> 2026-07-01 真机全验证(0.84m 外目标取回放纸)。本文档覆盖最后一块:Nav2 整合。

## 架构

```
jr_mission.py(任务层,长命节点)
 ├─ Nav2 action 客户端(/navigate_to_pose,map 坐标,机器人本机时钟)
 ├─ 手臂位姿控制(travel=OBSERVE;投放=FLOOR 已知安全位姿)
 └─ 每到一个 zone:subprocess 跑 jr_grasp_all.py run(单批单进程=零锁死纪律)
```

两种任务模式(mission.json 的 `mode`):
- **carry**(demo 主线):到 zone → 抓**一个**方块(`JR_CARRY=1`,抓到即带着退出、不放置)
  → 握着导航到 `delivery` → FLOOR 位姿放低 → 松爪 → 完成"把那边的方块拿来放这里"。
- **clear**:到每个 zone → 本地抓光(放当地白纸)→ 下一个 zone。

## 运行前置(全栈)
1. `ros2 launch jr_bringup robot.launch.py`(全开:雷达+相机)
2. `ros2 launch kinematics kinematics_node.launch.py`
3. `ros2 launch jr_nav nav.launch.py`(map_02;AMCL 种子=启动点=map 原点)
4. 机器人从建图原点附近启动(M3 约定)。

## 航点录制
`python3 jr_mission.py record` + 手柄/键盘遥控走场,把打印的 `[x, y, yaw]` 填进
`~/jetrover_ws/mission.json`。

## 已知风险(真机验证清单)
- 臂 OBSERVE travel 位姿是否遮挡雷达扇区(影响 AMCL/costmap)→ 看 /scan 有无固定近距假障碍。
- Nav2 与 grasp 的 cmd_vel 共用:Nav2 无活动目标时不发速度,理论无冲突,首跑观察。
- 全栈(Nav2+相机+RANSAC)负载下 Jetson 是否卡顿。
- 雷达净空防护的 `JR_SCAN_YAW` 朝向先用 scan_probe.py 标定。

## M7(重定义):量化评估
Gazebo 数字孪生正式搁置(仿真无法复现现实抓取偏差——用户 2026-06-29 的判断,实践反复印证)。
M7 = **正式基准测试 + 评估报告**:
1. **基准协议**:清零 grasp_stats.csv → 固定场景(6 方块:2 大 4 小,2 个摆斜 >30°,
   1 个 >0.4m 远,白纸入视野)→ `jr_grasp_all.py run` 连跑 3 场(每场重摆)→ 板锁即重启续跑。
2. **报告**:`python3 jr_report.py` → grasp_report.md(总成功率 + 按角度/宽度/颜色/场次分解)。
3. 及格线:干净配方历史净值 68%(2026-06-30,21/31);基准目标 ≥70%。
