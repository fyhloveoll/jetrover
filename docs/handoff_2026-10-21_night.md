# 交接：2026-10-21 深夜，下次开机一遍过（给压缩上下文后的自己）

## 现在的状态
- 车关机充电（曾掉到 9.6 V），用户 1 小时后再开机，要求当晚录到完整任务（取货→放置，两块方块），一遍过。
- 所有代码改动已在本机仓库，未提交；车上还是旧文件，**开机后由 go.sh 推送**。
- Foxglove 单窗口开着（不要再用 open_application 开它，会开第二个窗口）；OBS 在托盘，`interview_media\obs_rec.ps1 up|start -Name x|stop|status`。每条镜头前先录 3 秒抽帧核对画面（用 WSL ffmpeg 取帧看）。

## 下次开机的命令顺序（WSL 里，从 Windows 调）
1. 用户说"开机了"（车放在起点前方 1 m、车头朝原方向；方块在取货区原位置、不放纸；垫子按绿框、纸在垫子正中）。
2. `bash /mnt/c/Users/86133/AppData/Local/Temp/claude/C--Users-86133-Desktop-C--/401e4ead-e1b9-4bea-9a78-8801026e8c7a/scratchpad/go.sh 1.0 0.0 0`
   → 推文件 + 车上 `mission_boot.sh` → 打印 READY in N s。若 NOT READY，看它打印的缺哪项。
3. Foxglove：桥起来后自动重连；需要时在应用菜单"最近查看"重连；三维面板全屏；不要开相机原图面板（负载）。
4. OBS：`obs_rec.ps1 start -Name mission5`，录 3 秒先 stop 抽帧核对，再 start。
5. 任务：`ssh jetrover 'bash ~/jetrover_ws/mission_launch.sh'`（JR_MAX_TRIPS=2、日志不缓冲，10 秒后动；放置段是 task_place.py 子进程，非 0 退出码 = 方块留在爪里、任务停止）。监听：Monitor tail -F ~/jetrover_ws/mission.log，过滤 NAV|MISSION|CARRY|PLACE|DRIVE|LIVENESS|attempt|CL\]|ZONE\] mapped|released|DONE|!!。
6. 跑完 `obs_rec.ps1 stop`，`sudo poweroff`。

## 关键事实（别重犯）
- 禁止 `pkill -f`（会杀掉自己的 ssh 会话）；远程命令里不要写 `===`（zsh）；嵌套引号用脚本文件。
- 取货区不放标定纸（码会被当放置区）；放置区标定纸在垫子中央；垫子不是禁行区（用户决定）。
- 任务 run 模式现在走闭环抓取（JR_CL 默认 1）；carry 抓空重试 2 次；放置前 face_station 一轮；扫描 1 视角；地面拟合每帧一次；控制器不倒车。
- DDS：jr_env.sh 里 UDP-only profile；导航 action 服务器在长时间运行后可能"消失"，任务要在起栈后 10 分钟内跑完。
- 站点：mission.json zones [[3.3,1.25,90]]，delivery [3.35,-0.30,-90]。

## 已有素材
- out/01_建图.mp4、out/02_禁行区.mp4、out/仿真抓取_字幕讲解版.mp4、out/项目简介_范彧恒_JetRover.pdf、stills/。手机素材在 raw/phone/（IMG_7925 建图实拍）。今晚任务实拍待用户拷。

---

# 10-22 凌晨复盘（23:32–00:21，五次起任务，车已关机，电池关机前 10.84 V）

## 结果
- **没有录到一遍过**。最好的一条是 `interview_media/raw/obs_mission8_20261022_000544.mp4`（20 MB，7.5 min）：
  起点→导航到取货区 ✓→闭环抓取 ✓（第 2 次尝试，HOLD 19030 px）→导航到放置区 ✓→放置失败（原地转一圈找板、扫描没拼出垫子）→低位放地→回取货区导航失败（定位被转丢）。
  `obs_mission7_20261021_235414.mp4`（46 MB）：导航+闭环抓取 ✓（第 1 次尝试，24 mm 修正），放置段导航失败。其余 5/6/7b/9 是废片。
- 手机实拍：用户今晚没说拍，待问。

## 今晚逐次失败原因（都已取证，别再猜）
| 次 | 失败点 | 根因 | 状态 |
|---|---|---|---|
| 5 | 起点不动 | 开机 NTP 把时钟往前跳 83 min：`stack_wait` 用 `time.time()` 立刻"超时"；Foxglove 相机原图面板让桥吃 172% CPU；AMCL 漂到 1.6 m 外 | 已修：monotonic；面板删掉 |
| 6 | 起点不动 | 用户把车头朝墙，初始位姿按朝房间发（差 180°） | 摆放问题 |
| 7 | 抓完去放置区 | 抓取后底盘停在离茶几禁行区 0.18 m；局部代价地图也开了 keepout → TEB 足迹在 inscribed 环内，每条轨迹"不可行"；清图服务超时让恢复失效；`max_vel_x_backwards 0.05 ≤ penalty_epsilon` 触发 TEB 未定义行为警告 | 已修：局部 keepout 关（全局保留）、backwards 0.08 + epsilon 0.02 |
| 7b | 取货区"没方块" | 斜放的 40 mm 方块检测宽 53 mm > `JR_MAX_WIDTH` 48 mm 被过滤 | 已修：mission_launch 里 `JR_MAX_WIDTH=0.06` |
| 8 | 放置 | **10-21 的地面拟合缓存用 `id(self.depth)` 当键，numpy 复用地址 → 一次 None 永久缓存 → 所有标签/轮廓投影 None**（4 个视角明明检出标签）；`face_station` 一圈 355° 把 AMCL 转丢（aa 1.06） | 已修：帧计数器键、None 不缓存；max_turns 4（140°） |
| 9 | 导航极慢 | 车 CPU 饱和：nav 容器 175%、foxglove_bridge 106%（无图像面板也如此）、load 8；控制循环 2 Hz，车一顿一顿；用户关机 | **未解决** |

## 已验证有效的东西
- `mission_boot.sh`：一条命令 30–46 s READY（topics 3 s / action 8 s / AMCL / 20 补丁 / 初始位姿 / 蜂鸣）。
- OBS 冻帧根因：Chromium 窗口被完全遮住就停止绘制。Foxglove 必须用
  `--disable-backgrounding-occluded-windows --disable-renderer-backgrounding --disable-background-timer-throttling --disable-features=CalculateNativeWinOcclusion` 启动（遮挡测试已通过）。每条镜头前仍要录 2 s 抽帧核对时间戳。
- 闭环抓取在任务里 2/2 成功；`HOLD` 像素 18–19k（可设 `JR_HOLD_MIN=8000`）。
- 运行时参数工具 `setp.py /node k=v`；`cost_at.py`（AMCL 位姿+代价地图采样）；`initpose.py x y yaw cov_xy cov_yaw`；`alive.py`/`motion_check.py`/`grab_frame.py`/`batt.py`。

## 下次开机前必须做（按优先级）
1. **降负载**：foxglove_bridge 加 `topic_whitelist`（map, scan, tf, tf_static, amcl_pose, particle_cloud, global_plan, local_plan, goal_pose, initialpose, global_costmap/costmap, local_costmap/costmap, keepout_filter_mask, global_costmap/published_footprint）；考虑录屏时干脆只订阅 map/scan/tf/plan。看 TEB 是否开了 `enable_homotopy_class_planning`（nav 容器 175% 的嫌疑），关掉。目标：控制循环回到 10 Hz。
2. `mission_boot.sh` 开头等 `timedatectl` NTPSynchronized=yes 再起节点（时钟跳变期间起的 AMCL 丢过雷达队列）。
3. 放置段已重写为 `task_place.py`（库 `place_skill.py`；`jr_grasp_all.py place` 退役 exit 2），**未在真机验证**：第一趟先 `python3 task_place.py --look-only`（带方块，`--handin` 先递方块）看每视角 `[PLACE/LOOK]` 的 tags/mat/self/pts 与 floor fit，再 `--dry-run`，最后跑真放置（见文末 10-22 节）。
4. 初始位姿：用户摆车朝向有 ±45° 误差，继续用 `initpose.py 1.0 0.0 0 0.15 0.6`（宽朝向协方差），起步转向时 AMCL 自己收敛——本次起步导航是成功出发的。
5. 取货站点离茶几禁行区太近，抓取后常停在区边：站点东移 0.15 m（zones [[3.45,1.25,90]]）或把禁行区东边缘往西收 2 格。

## 文件
- 仓库 `scripts/`：go.sh（已改指向仓库）、mission_launch.sh、rewait.sh、setp.py、cost_at.py、initpose.py、alive.py、motion_check.py、grab_frame.py、batt.py、stack_wait.py（monotonic、20 补丁）、jr_grasp_all.py（_depth_seq）、station_dock.py（max_turns=4）。车上同名文件已是最新（关机前推过）。

## 00:33 剪辑产出
- `interview_media/out/03_完整任务_取货到放置区.mp4`（54.8 s，母版）/ `_wechat.mp4`（已发给用户）。
  来源：mission8 一次运行。IMG_0450 全段（出发，OBS 37–51 s 小窗）→ OBS 51–79 s 地图过场 ×4 → IMG_0451 0–50 s 抓取 + 50–75 s 带方块到放置区（OBS 79–154 s 小窗），全部 ×2。
  **IMG_0449 第 51 s 后、IMG_0451 第 75 s 后是失败段，永不使用。** 0449（mission7 抓取特写，机位更清楚）未用，可单独剪"抓取特写"。
- 复现：`python3 cards/make_cards03.py && bash make03.sh`（WSL）。

## 00:45–01:15 摆拍放置（3 次，均未放到垫上；最后手动低位放地让用户录完）
- 轻量栈 `place_boot.sh`（bringup+kinematics，1.5 s 就绪）+ `place_run.sh`，电池 11.31 V。
- 三次都在 `face_station` 第一步"看不见板"。**根因：观察位画面下半中间被夹爪里的方块挡住**（cam2.jpg），板子在 0.4–0.65 m 正前方时整块落在遮挡区；mission8 能看见是因为导航停在约 1 m 外、且扫描时左右转头让板子偏出遮挡区。
- 误导点：夹爪空着做的静态探针（probe_measure.py）8 个标签、测量 0.1 s 成功——**探针必须带着方块做**。
- 已改 station_dock.py：看不见先后退 0.25 m 再看；每姿态等 0.6 s 连试 3 次；超过 1.05 m 才主动靠近（`JR_FACE_DRIVE_MIN`）；找板上限 140°。
- 下次摆拍：**车头前沿离纸近边 ≥ 80 cm**（板中心 ≥ 0.9 m），先用 grab_frame.py（带方块、观察位）确认 tags>0 再起；或给 look() 加一个把腕抬高/侧倾让方块移出画面的姿态。
- 直接放地：`place_floor.py`（place_on_paper 的安全低位释放，前方 0.22 m）。
- 01:05 **v2**：接入 IMG_0456（00:57 摆拍放置：车手摆在垫边，`put_down.py` 纯动作放下，无视觉）作为"放置"段（×1.5，无地图小窗），片头/片尾卡链路加"→ 放置"。成片 61.6 s，`_wechat.mp4` 已发用户。IMG_0452–0455 是失败的摆拍，不用。

---

# 10-22 放置技能重写（task_place.py / place_skill.py）—— 未真机验证

- 入口：`python3 task_place.py [--preflight|--look-only|--dry-run|--handin|--verify]`，退出码见文件头；`jr_grasp_all.py place` 退役（只打印提示，exit 2）；`jr_mission.py` 改调 task_place.py，非 0 → 方块留爪里、`break`、`sys.exit(3)`，人用 `put_down.py` 释放；张爪**之后**才出的错（verify / 回 carry 位）记 `[PLACE/WARN]` 并退出 0（方块已在垫上，不算失败）；`--handin` 要求 stdin 是终端（否则 rc 2、爪未动）。`mission_launch.sh` / `place_run.sh` 不再设 `JR_DOCK=1`（放置不再 face_station，不原地转）。
- 设计：暗垫（`_zone_detect(strict=False)`）为主线索、标签只修剪/锚定；底盘只走 `drive()`（雷达护航）且 scan 必须新鲜（<1 s），倒车 ≤0.15 m/腿、累计 ≤0.25 m；只有 hover(+0.07) 和落点双 IK 都有解才张爪；每条非 0 退出不碰舵机 10、臂回 carry 位、六次零速。
- 测试顺序（电池模式）：`--preflight --no-liveness` → `--preflight` → `--look-only --debug-dir ~/jetrover_ws/place_dbg`（空爪，垫中心 0.9 m）→ `--handin --look-only`（带方块，同摆位；这就是上文 00:45 节要求的"带方块探针"）→ 带方块、垫中心 0.5 m `--look-only`（看 OBS 500 是否 mat=N/self=Y、OBS 660/340 与 FLR 500 是否 mat=Y）→ 盖住垫子 `--look-only`（期望 rc=20）→ **负测试**：无垫、无标签、车停在深色地面或自身阴影处 `--dry-run`（期望 `NO_CELL` / rc 20 或 21，**绝不能出现 `DRY: would lower`**；出现 = 近处无标签暗块被当成了垫子）→ `--dry-run`（0.9 m 期望打印 leg 1；0.3 m 期望 cell+双 IK）→ 全流程空爪闭合 → `--handin --verify` → 失败注入（前方 0.3 m 纸箱 rc=31；拔雷达 rc=30）→ 第二趟（垫上已有一块：objs=1，所选 cell 距它 ≥0.055）→ `bash mission_launch.sh` 集成（盖垫 → `!! PLACE FAILED rc=20`、无 trip 2、jr_mission 退出码 3）。
- 未验证/未知项（不靠猜）：FLOOR 位带方块的遮挡范围；相机水平视场（±38° 转头在 0.5 m 时垫子是否出视场）；`_depth_seq` 缓存修复与 station_dock 10-22 改动均未真机验证；`JR_SCAN_YAW` 默认 0 未标定（jr_grasp_all.py:55）；非 strict 最大暗块会把阴影/深色家具当垫子（面积 ≤0.45 m² 与 x<1.2 门挡不住中等暗物）——**近处（0.12–0.36 m）的无标签暗块会张爪**：`self_blob` 10-22 改为所有视角生效（原来只在 OBSERVE；FLOOR 视角的车体/爪中方块/车前阴影暗块会被投影到 0.15–0.40 m，正落在 zone_cell 臂包络里并能通过双 IK），能挡住车体和爪中方块（深度 <0.25 m 无效），**挡不住车前阴影**（深度有效），只有标签锚定或 strict 模式能挡——这是"暗垫为主、标签可选"固有的风险，是否接受由用户定，不替他定；`self_blob` 是三条现有规则的新组合，只会丢轮廓（副作用未验证：FLOOR 视角里底边切掉、质心在爪 ROI 且深度刚好 <0.25 m 的真垫子也会被丢，只会少一视角，不会误张爪）；`X_TARGET` 0.28 / 接近窗口 0.34–0.20 / `REV_LEG` 0.15 是 10-22 选的数，出处（旧 place 块）已删、从未上过车（现存公式 jr_grasp_all.py:1178 用 0.30，实跑过的倒车腿是 0.13 :1577）；`placed`（已放方块排除表）只在 FLOOR 视角采集，OBSERVE 富视角早退时为空，第二块可能叠在第一块上——现在只打 `[PLACE/PLAN] WARNING: no FLOOR view`，没加视角（用户决定）；横移方向 ±24° 锥落在 180° 裁剪边界附近可能部分无回波（整锥 None 才拒走）；CPU 饱和（上文第 9 次）下 rc 30 会更常出现——这是刻意的，不盲走。
- 布局建议（用户决定，不随代码改）：`docs/mission_2026-10-21.json` delivery [3.35,-0.30] 到垫中心 (3.35,-0.83) 名义 0.53 m，正落在 OBSERVE 遮挡带；板中心应 ≥0.9 m → delivery y 约 +0.10（map 系粗值，臂基座到 base_link 偏移仓库无数值），并避开 keepout。
