# JetRover 架构问题合并报告（2026-10-22，只读仓库取证，未碰车）

> 合并自两份独立分析（下称 A0「最小改动」、A1「技能层」）。只保留能引用 `文件:行号` 的论断，**所有行号都已在本机工作副本 `C:\Users\86133\Desktop\C++\jetrover` 上重新核对**（路径相对仓库根）。两份分析不一致的地方在 §0.2 逐条裁定。凡写"未知"的，仓库里就是没有证据，不是没找。本报告没有在车上跑任何命令。

## 0. 取证基线与裁定

### 0.1 仓库真实状态（本次 `git status`）

- 最后提交 `5273127`（2026-09-03）。**已修改未提交**：`CLAUDE.md`、`scripts/jr_grasp_all.py`、`scripts/jr_mission.py`、`scripts/mission_up.sh`、`scripts/nav_lean_up.sh`、`scripts/nav_patch_all.sh`、`src/jr_nav/launch/nav.launch.py`。
- **未跟踪**：`scripts/` 下 21 个文件（`mission_boot.sh`、`mission_launch.sh`、`stack_wait.py`、`station_dock.py`、`place_boot.sh`、`place_run.sh`、`place_floor.py`、`put_down.py`、`nav_goto.py`、`nav_params_filters.py`、`initpose.py`、`setp.py`、`cost_at.py`、`rewait.sh`、`go.sh`、`grab_frame.py`、`alive.py`、`batt.py`、`motion_check.py`、`make_filter_mask.py`、`zone_paint.py`）；`docs/` 下 9 个（handoff、readback、mission json、URDF、nav_session 等）；`maps/` 下 17 个；`src/jr_nav/launch/filters.launch.py`；**`src/jr_teleop_cpp/`**（`cmd_vel_relay.py` 的 rclcpp 移植，68 行，`src/jr_teleop_cpp/src/cmd_vel_relay.cpp:1-3`，不订阅 `/scan`）。
- A1 说"8 个未跟踪任务脚本"是低估，A0 的清单也不全；`src/jr_teleop_cpp/` 两份都没提，但它决定了"雷达守卫加在哪个 relay"（§5.4）。

### 0.2 两份分析不一致处的裁定

| 议题 | A0 | A1 | 裁定 |
|---|---|---|---|
| 雷达守卫阈值 | 复用 `drive()` 的 0.20 / 0.28 作起点 | 必须先量 `d_contact`、`/scan` 频率、延迟；0.20/0.28 是 0.07 m/s 下的雷达读数 | **A1 更有依据**：导航最高线速 0.15 m/s（`cmd_vel_relay.py:24`、`stack_wait.py:37`）是抓取 `drive()` 默认 0.07 m/s（`jr_grasp_all.py:611`）的两倍多；0.20/0.28 是"雷达读数"不是"足迹间距"，足迹未读回。两份都要求先影子模式，所以折中：影子模式用 0.20/0.28 **只做计数**，正式值等读回与测量。 |
| 初始位姿协方差默认值 | 参数化但默认仍 0.05，先确认 mission8 用的哪组 | 默认改 0.15/0.6，引 handoff:58"本次起步导航是成功出发的" | **A0 更稳**：handoff:10 记载的开机流程是 `go.sh 1.0 0.0 0` → `mission_boot.sh:26` → `stack_wait.init_pose`，硬编码 0.05（`stack_wait.py:126`）；handoff:58 的"继续用 `initpose.py ... 0.15 0.6`"没写是否在 mission8 之前手动发过。参数化两份都同意；默认值等用户确认（§10）。 |
| `time.monotonic()` 替换 | 列为独立行动 | 不要和行为改动混在一个提交；先做 NTP 门禁（根因） | 不冲突：NTP 门禁先做，计时替换作为独立的机械提交。 |
| `jr_mission.py:191` `if r != 0` | "永远不可达" | "死代码" | 精确说法：`jr_grasp_all.py` 全文无 `sys.exit`（grep 零命中），所有出口 `return`（`:1298-1304`、`:1361-1362`、`:1388`、`:1764-1765`），进程正常路径恒 exit 0；**只有未捕获的 Python 异常**才会非 0。设计意图（放置失败→低位释放）确实到不了。 |
| 退出码约定 | `sys.exit(1)`/`sys.exit(2)` 两档 | 0/1/2/3/4/5 六档 + `RESULT` 末行 | 取 A1 的表（是 A0 的超集），照抄 `stack_wait.py:153/161` 已在用的 0/1 模式扩展。 |
| scan 过期时守卫怎么办 | "降速或停" | "零速；只在收到输入时输出，不加定时器" | 取 A1：零速、边沿触发；理由见 §5.4（M6_mission_design.md:32 记载 Nav2 无目标时不发速度，relay 平时静默，定时器会和 `drive()` 抢 `/controller/cmd_vel`）。 |
| `JR_HOLD_MIN=8000` | 直接设 | 可设但只有 2 个样本 | 设，并明说证据薄（handoff:51 两次 18–19k px）。 |

### 0.3 本报告新增、两份都没写的核对结果

- `station_dock.py` "雷达看不见低矮家具"注释在 **`:187-188`**（两份都写成 189-190）。
- `jr_grasp_all.py:1723-1725` 注释写 carry 模式"no return drive"，但 `:1729` 的 `break` 跳出循环后，`:1754-1757` 的 `retrace_home()` 在 `fetch_off != 0` 时**照样执行**（倒车方向雷达盲，`:1499`）。今晚是否触发：handoff 未记录，未知。
- readback 里 `FollowPath.max_vel_x_backwards` 读回 **0.2**（`docs/nav_params_readback_2026-10-21.txt:4`），开机补丁设 **0.08**（`stack_wait.py:34`），handoff:20 写"控制器不倒车"——三者互不一致，没有一个是 0。

## 1. 一句话结论

四个问题里真正属于"架构"的只有一类：**缺契约、缺守卫、缺健康判据**——子任务没有可判定的退出码，Nav2 速度链上没有我们自己的雷达门控，定位好坏没有任何自动判断。"一个文件跑完自退"的骨架（`mission_launch.sh` → `jr_mission.py run` → 子进程 `jr_grasp_all.py run / place`）**已经存在**，不需要重组。今晚（10-21 夜）的失败绝大多数是**流程错误**（当晚改代码当晚上车没验证、车头摆反、空夹爪做探针、没等 NTP）和**基础设施**（CPU 饱和），不是结构问题。"抓取没用闭环"**与事实不符**（§4.2）。

## 2. 已知 / 未知

**已知（有 file:line）**
- 任务链路：`scripts/mission_boot.sh:14-30`（清扫 → 4 个 setsid launch → `stack_wait.py` 体检 → 退出码）；`scripts/mission_launch.sh:5-7`（export `JR_DOCK=1 JR_MAX_TRIPS=2 PYTHONUNBUFFERED=1 JR_MAX_WIDTH=0.06` → `setsid bash -c 'sleep 10; python3 ~/jetrover_ws/jr_mission.py run'`）；`scripts/jr_mission.py:161-207`（zone × trip 循环，`run_grasp` 起子进程 `:114-115`，送达后 `subprocess.call([..., 'place'])` `:190`，打印 `MISSION DONE` 后退出 `:205-207`）。
- 唯一的雷达防撞：`scripts/jr_grasp_all.py:596-608`（`clearance`，±0.42 rad 锥）和 `:611-670`（`drive`），发布到 `/controller/cmd_vel`（`:128`）。
- Nav2 → 电机最后一跳 `src/jr_teleop/jr_teleop/cmd_vel_relay.py:26-42`：只做等比限速，无 `LaserScan`、无定时器，退出时发一个 `Twist()`（`:53`）；`src/jr_nav/launch/nav.launch.py:18-43` 只起厂家 bringup + 这个 relay。
- 已读回的 Nav2 实值只有 `docs/nav_params_readback_2026-10-21.txt` 29 行（controller_server 与 velocity_smoother）。
- 开机运行时补丁 `scripts/stack_wait.py:27-46`：TEB `min_obstacle_dist 0.10`、`inflation_dist 0.30`、`weight_obstacle 50.0`（`:30-31`）、`max_vel_x_backwards 0.08`、`penalty_epsilon 0.02`（`:34`）；smoother `max_velocity [0.15,0.15,0.6]`、`max_decel [-2.5,-2.5,-3.2]`（`:37-38`）；全局 keepout 开、局部关（`:41`、`:45`）。
- 硬件：`/scan` 被裁到车头前方约 180°（`CLAUDE.md:80`）；雷达在 `base_link` 前 0.090 m（`docs/robot_description_2026-09-04.urdf.txt` 单行文件中 `lidar_joint` origin xyz `0.0900 0 0.0405`）。

**未知（仓库里一条证据都没有）**
- 局部/全局 costmap 的 `plugins`、`obstacle_layer` 观测源、`obstacle_max_range`、`raytrace_max_range`、`inflation_radius`、`footprint`/`robot_radius`、`update_frequency`；TEB `include_costmap_obstacles`、`footprint_model`、`enable_homotopy_class_planning`；`controller_frequency` 标称值（readback:17 printf 合并、无值）；TEB `max_vel_x`（readback:1 读取失败）。读这些的脚本 `scripts/nav_params_costmap.sh:6-21` 已写好，`docs/` 下无输出。
- AMCL 全部参数。全仓库 grep `max_particles|alpha_slow|laser_model_type|update_min_d` 零命中；`scripts/nav_params_dump.sh:5-18` 不查 `/amcl`；`scripts/nav_params_filters.py:85-120` 对厂家 yaml 的 AMCL 段原样透传。
- `/scan` 实际频率（只有 `jr_grasp_all.py:111` 注释"10Hz"）、精确裁剪角、雷达安装偏航 `JR_SCAN_YAW` 真值（`:55` 默认 0，注释"probe on-robot"；`docs/M6_mission_design.md:34` 列为待标定）。
- 厂家行为树/恢复行为配置；控制板对 `/controller/cmd_vel` 发布者消失后有没有速度看门狗；`nav2_collision_monitor` 是否装在厂家镜像里。
- 今晚"定位演示时撞墙"：`docs/handoff_2026-10-21_night.md:38-45` 失败表**没有撞墙条目**；撞墙时谁在发速度、AMCL 是否已丢、局部 costmap 有没有标出墙——全部未取证。
- `scripts/jr_env.sh:1-19` 没有 handoff:21 说的 UDP-only DDS profile（车上副本是否不同：未知）。

## 3. Q1 任务封装："一个文件跑完自退"，远处物体自动开过去

**结论：骨架已有，缺的是契约。不要重组，只补包装。** 这一题没有"今天的流程错误"，纯结构缺口。

现状正是你要的形状：`mission_launch.sh:6` 一条命令起 `jr_mission.py run`；`jr_mission.py:205-207` 打印 `===== MISSION DONE =====` → `destroy_node` → `rclpy.shutdown` → 进程退出。每个 zone 的抓取是独立子进程（`:114-115`，`timeout=700`），放置是 `:190`。"长命任务节点 + 单批单进程"是 `docs/M6_mission_design.md:8-13` 的有意设计（`CLAUDE.md:20` 铁律 4），**不要改成一个常驻大进程**。

"抓取远处物体自动开过去"**已经实现**，不是新需求（`JR_AUTO_DRIVE` 默认 1 `jr_grasp_all.py:52`、预算 `JR_MAX_DRIVES=12` `:53`，mission 没有关它——`mission_launch.sh:5` 与 `jr_mission.py:107-112` 都不设）：
- 够不到（IK 无解 `OUT_OF_REACH`）且正前方：`:1668-1677`，前进 `d = pos0[0] - 0.24`（`:1670`）；偏侧先横移 `:1659-1667`。
- 地面视角没候选时抬头找远物：`survey_far` `:672-713`（只认 0.38–1.2 m、|y|≤0.45，`:700`）→ `:1611-1624` 开到 `fx_ - 0.30`。
- 太近（画面底边切掉）：`:1605-1610` 先倒车 0.13 m。
- 07-01 真机验证过 0.84 m 取回（`M6_mission_design.md:3-4`）。今晚是否触发过：handoff 无 `DRIVE_FETCH/SURVEY` 记录，mission8 第 1 次尝试失败类型也没写（handoff:33）——未知。

**结构性缺口（要补的）**
1. **无退出码语义。** `jr_grasp_all.py` 全文无 `sys.exit`；板死 `:1361-1362`、place 结束 `:1388`、run 结束 `:1764-1765` 都是 `return`；`grasp` 抛异常只 `print + break`（`:1643-1644`）。`jr_mission.py` 同样：导航失败只 `print + break`（`:165-166`、`:197-201`），clear 模式末尾 `goto(delivery)` 返回值被丢（`:202-204`），最后无条件 `MISSION DONE`。所以 `:191` 的低位安全释放除非子进程 traceback 否则到不了；外层无法区分"放上垫子 / 放地 / 板子死了"。
2. **步骤间靠子串和环境变量交接。** `carried = 'CARRYING' in r.stdout`（`:119`）；`JR_CARRY`（`jr_grasp_all.py:1686`）、`JR_DOCK`（`:1363`）、`JR_CL`（`:959`）各改一段行为；`run_grasp` 只把子进程最后 14 行非 Warning 行写进 `mission.log`（`:116-118`），`[CL]`/`[M6]`/`[DRIVE]` 过程行丢失——这直接导致事后说不清"用没用闭环"。
3. **没有记账与停止。** `mission_launch.sh:7` 只 `echo` 组号不写 pid 文件；`scripts/` 里没有 `stop_mission.sh`（`ls scripts/` 已确认），中途停只能 `stack_sweep.sh` 连 bringup 一起杀，违反 `CLAUDE.md:19` 铁律 3"按记账 PID 停"。
4. **子进程超时未处理。** `:114-115` 的 `timeout=700` 外无 `try`，`TimeoutExpired` 会让任务进程 traceback，子进程被 SIGKILL 时 `drive()` 末尾 6 次零速（`jr_grasp_all.py:667-668`）发不出；控制板有无看门狗未知。
5. **`main()` 闭包锁住状态。** `pickable / drive_leg / path_clear / strafe_clear / retrace_home` 都是 `main()` 内嵌函数，捕获 `fetch_off / fetch_lat / blacklist / drives / placed_pts / legs`（`:1414-1421`、`:1492-1495`），外部调不到，所以 `put_down.py:10-11`、`place_floor.py:9-10` 只能 `import jr_grasp_all as ga` 戳私有方法。
6. **每步新进程 = 每步重新 DDS 发现**：`fresh(t=15.0)`（`:1300-1304`）、`wait_for_server` 4×8 s（`jr_mission.py:67-73`，注释"discovery can lag after a long grasp run (10-21)"）；handoff:21 记载 action server 长时间后"消失"。已知约束，不在本次范围。

**最小做法**（纯包装，不碰抓取/放置算法）：`jr_grasp_all.main` 各出口 `sys.exit(code)` + 末行 `RESULT <mode> <status> k=v`；`jr_mission` 改读退出码并把子进程全文写 `mission_grasp_<n>.log`；`mission_launch.sh` 写 `mission.pid`，新增 `stop_mission.sh`（按 pgid TERM 后立刻零速，照抄 `nav_stop.sh:6-12`）；`run_grasp` 包 `try/except subprocess.TimeoutExpired`。之后才是把闭包提成类、`jr_skills.py` 薄封装（§8 第 14 项）。

## 4. Q2 今晚的问题

### 4.1 导航定位屡次失败

按 `docs/handoff_2026-10-21_night.md:38-45` 逐条分性质：

| 次 | 根因（handoff 已取证） | 性质 | 现状 |
|---|---|---|---|
| 5 | NTP 把时钟前跳 83 min，`stack_wait` 用 `time.time()` 立刻超时；Foxglove 原图面板让桥 172% CPU；AMCL 漂 1.6 m（`:40`） | **流程/基础设施** | `stack_wait.py` 已改 monotonic（`:52-59`、`:118-119`）；handoff:56 要求的"等 `timedatectl` NTPSynchronized 再起节点"在 `mission_boot.sh:1-30` **没有实现**；`jr_mission.py:50-58,90-91`、`jr_grasp_all.py:154,617-666`、`station_dock.py:138-163` 仍用 `time.time()` |
| 6 | 车头朝墙，初始位姿按朝房间发（差 180°，`:41`） | **摆放流程错误** | 结构侧：`stack_wait.py:126` 硬编码 `cov[35]=0.05`（σ_yaw≈12.8°）而 handoff:58 记录摆放误差 ±45°；发完只打印订阅者数（`:131`）不验证——摆错了系统发现不了 |
| 8 | `face_station` 转 355° 把 AMCL 转丢（aa 1.06，`:44`） | **当晚新加行为的设计错误** | 已限 `max_turns=4`（140°，`station_dock.py:208`、`:225`），先退 0.25 m 再看（`:215-223`）；**未真机验证**（handoff:57）；旋转仍由 `Docker.turn` 直发 `/controller/cmd_vel`（`:145-149`）绕过 Nav2，且**无转后定位检查** |
| 9 | CPU 饱和：nav 容器 175%、foxglove_bridge 106%、控制循环 2 Hz（`:45`） | **基础设施，未解决** | `deploy/foxglove_up.sh:64-65` 默认 launch 无 `topic_whitelist`；`enable_homotopy_class_planning` 未读回 |

**结构性缺口**：仓库里**没有任何东西自动判断定位是否丢失**。`jr_mission.py:47-48` 订阅 `/amcl_pose` 只存 `pose.pose`，从不读协方差；`nav_goto.py:59-60` 的 `number_of_recoveries` 只打印；唯一读协方差的是手动探针 `cost_at.py:51`。`goto` 失败后不重发初始位姿、不重试、不区分"定位丢了"和"路径不可行"（`jr_mission.py:165-166`、`:196-201`）。另：readback:21 RotationShim `rotate_to_heading_angular_vel 1.5` vs smoother 上限 0.6（`stack_wait.py:37`）仍是一例"栈内规划/执行互相说谎"（`docs/engineering_decisions.md:102` 同类）。

### 4.2 "抓取方块没用闭环"——核对结果：今晚任务里**用了**

- `jr_grasp_all.py:959-960`：`if os.environ.get('JR_CL', '1') == '1': return self.grasp_closed_loop(inst)`，默认闭环；`:955-958` 注释写明 **10-21 之前**任务模式一直走开环，闭环只接在 `yaw_calib` 路径上。
- 外层没有覆写：`mission_launch.sh:5` 和 `jr_mission.py:107-112` 都不设 `JR_CL`。
- handoff:20"任务 run 模式现在走闭环抓取（JR_CL 默认 1）"；`:33` mission8"闭环抓取 ✓（第 2 次尝试，HOLD 19030 px）"；`:34` mission7"闭环抓取 ✓（第 1 次尝试，24 mm 修正）"；`:51`"闭环抓取在任务里 2/2 成功"。

所以"没用闭环"对 10-21 之前的所有任务运行是对的，对今晚（mission7/8）不对；00:45–01:15 的摆拍是放置，没有抓取。

但有一个真实的结构缺口会让"用了闭环"和"闭环生效"看起来一样：`:925` 只有 stage1 返回 `converged/max-iters/diverged` 才进 stage2；若返回 `lost/implausible/no-ik`，`:941-952` 照样腕偏航、下探、合爪、抬起并返回 `LIFTED`——等价于**静默退回开环**。能分辨的只有 `[CL] pre:<status>` 一行（`:935-936`），而它会被 `mission.log` 的 14 行截断吃掉（`jr_mission.py:116-118`）。失败应显式可见，而不是继续抓。

### 4.3 "方块不在范围内 / 距离过近不会自己调，必须人说了才做"

两种所指分开看。

**(a) 放置时板子太近（最符合 handoff:72-75 的三次摆拍）**：`face_station` 在 10-22 之前没有"太近"假设，看不见就原地转；盲退 0.25 m 再看（`station_dock.py:215-223`）是摆拍失败**之后**加的，未真机验证。更深一层：`face_station` 返回 `None` 分不开"没有板"和"板被夹爪里的方块挡住"，空夹爪探针 8 标签成功是误导（handoff:73）——**测试流程错误**（探针没带方块）叠加结构缺口（前置条件"板可见"从未被检查）。"必须你说了才做"那部分是流程。

**(b) 方块本身**：代码**会**自动调（§3），但有几条**静默丢弃路径**既不调也不报告：
- 深度 < `SELF_MIN_DEPTH` 0.20 m 的检测当车体丢弃（`:207`、`:217-218`）；解出 x<0.08 的点拒绝（`:590-593`）。"太近"唯一可见触发是底边切掉（`:1435-1440` → `:1605-1610`），且只在 `fetch_off < 0.02 and |fetch_lat| < 0.02` 时做。
- 7b 那次是宽度过滤：斜放 40 mm 方块测 53 mm > `JR_MAX_WIDTH` 默认 48 mm（`:57`；代码 `:1452-1457`；handoff:43）→ 一帧 `pickable()` 为空 → `'floor clear'`（`:1625-1626`）→ 进程退出无 `CARRYING` → `jr_mission.py:180-182` 判 zone clear。看起来像"不会调距离"，其实**根本没进候选**；flicker 重试只在设了 `JR_TARGET` 时生效（`:1586`、`:1594`）。已修：`mission_launch.sh:5` `JR_MAX_WIDTH=0.06`。
- 追过一次没变近 → 永久停牌（`:1651-1658`）。
- 闭环的 `OUT_OF_REACH` 判定在预抓位（`:914-916`）和修正后的抓取高度（`:944-946`）；后者返回时臂在目标上方低位，主循环先 `drive_leg(d)`（`:1673`）再 `node.home()`（`:1677`）——代码可见，真机未验证。

结构上的共同点：`find_object` 没有"看到了但因为 X 拒绝了"的输出，任务层无法决定"退一点再看"，只能靠人。今晚究竟撞上哪条，handoff 没有逐 attempt 日志；`~/jetrover_ws/grasp_stats.csv`（`:59`、`:1268-1281`）有每次尝试的 `result` 字段（`OUT_OF_REACH/DRIVE_FETCH/CHASE_STALL/MISS/...`），下次开机先拷回来看。

## 5. Q3 防撞急停与避障

### 5.1 撞墙：取证状态

handoff 失败表（`:38-45`）**没有撞墙条目**。下午的"定位演示"只有 `docs/nav_session_2026-10-21.txt:1` 一行：错误初始位姿 (1.9,0.5,140) 改正为 (1.15,0,180) 后目标 11.9 s 成功。撞墙时谁在发速度（Nav2 / RotationShim 转向 / 恢复行为 / 遥控）、AMCL 是否已丢、局部 costmap 当时是否标出了墙——**全部未取证**。能肯定的只有一条：**定位演示是 Nav2 在开车，而车上唯一的雷达急停不在那条路上**（§5.2）。

### 5.2 "以前做过的急停"是什么、为什么觉得迟钝/笨

就是 `GraspAll.drive()`（`jr_grasp_all.py:611-670`）：
- 起步前最多等 0.4 s 要一帧新 scan（`:626-628`）；运动方向 ±0.42 rad（≈±24°）扇区最小距 `c < |dist| + CLEAR_MARGIN(0.28)` 就截短，截完 <0.02 m 不走（`:629-636`）。
- 行进中 10 Hz 发令（`:650-654`），**只在 scan 年龄 <1.0 s 时**查距离，<0.20 m 急停（`:659-663`）；scan 过期只打一次 `warn: scan stale ... e-stop blind` 然后**继续开**（`:664-666`）。
- `/scan` 用专用 mini-node + sensor QoS（`:116-121`），注释记录 07-10 因 ~4 s 过期扫描撞书两次。
- 倒车方向无数据（`CLAUDE.md:80`；`:1499` 注释 "reverse is lidar-blind"），`clearance` 返回 `None` 则**不拦**（`:630`、`:661`）。`station_dock.Docker.turn`（`station_dock.py:133-167`）原地转 0.35 rad/s（`:35`）最长 8 s，**完全不查雷达**。
- 只保护它自己：Nav2 段、键盘/手柄（`keyboard_teleop.py:54` 直发 `/controller/cmd_vel`）都不经过它。

"迟钝"的机制：进程内轮询，判定在每个 0.1 s 指令窗口末尾；scan 名义 10 Hz；用 `time.time()` 墙钟（`:154`、`:659`）——10-21 那种 83 min 时钟跳变会让它要么全判过期要么全判新鲜。0.07 m/s × 0.1 s = 7 mm/周期，物理上不算迟钝。"笨"的机制：只停不减速；只看一个 24° 锥、只看运动方向；低矮家具雷达看不见（`station_dock.py:187-188` 注释）、3 cm 方块看不见（`jr_grasp_all.py:1479`）；`JR_SCAN_YAW` 若不为 0，查的扇区就是错方向（`:55`、`:605`）。

**关键事实：今晚撞墙时是 Nav2 在开车，这个守卫根本不在那条路上。** Nav2 段的速度流是 `controller_server → velocity_smoother → /cmd_vel → cmd_vel_relay → /controller/cmd_vel`，relay 不看 `/scan`；`jr_mission.goto` 只等 action 结果（`:89-97`），150 s 才 cancel。人工急停只有 `nav_stop.sh:6-12`：TERM → `sleep 3` → KILL → 零速，零速在 3 s 之后才发，中间只靠 relay 退出时的一个 `Twist()`（`cmd_vel_relay.py:53`）。

### 5.3 Nav2 对"车前凭空出现一个人"会怎样——已知与未知分开

**已知（真机读回或开机补丁）**

| 项 | 值 | 出处 |
|---|---|---|
| 控制器 | TEB（RotationShim 包裹） | `docs/engineering_decisions.md:94-96` |
| 最大线速度 | 0.15 m/s | `cmd_vel_relay.py:24`；`stack_wait.py:37`；TEB `max_vel_x` 读回失败（readback:1） |
| 减速上限 | smoother `max_decel [-2.5,-2.5,-3.2]`；TEB `acc_lim_x 2.5` | readback:27、:5 |
| smoother | `velocity_timeout 1.0 s`、`smoothing_frequency 20 Hz` | readback:28-29 |
| TEB 障碍项 | `min_obstacle_dist 0.10`、`inflation_dist 0.30`、`weight_obstacle 50`（厂家原 0.26 / 0.6 / 100） | `stack_wait.py:30-31`；`nav_patch_teb.sh:4-6` |
| 可行性/进度/目标 | `feasibility_check_no_poses 3`；进度 0.25 m / 20 s；目标 xy 0.12 / yaw 0.2 | readback:13、:19-20、:14-15 |
| 倒车 | `max_vel_x_backwards` 补丁 0.08（readback 0.2）——**不是 0** | `stack_wait.py:34`；readback:4 |
| 角速度矛盾 | RotationShim `rotate_to_heading_angular_vel 1.5` vs smoother 0.6 | readback:21 vs `stack_wait.py:37` |
| 控制循环 | CPU 饱和时实测 2 Hz；标称 `controller_frequency` **未读回** | handoff:45；readback:17 |
| keepout | 全局开、局部关 | `stack_wait.py:41`、`:45` |

**未知（一条真机值都没有）**：局部 costmap 的 `plugins / observation_sources / obstacle_max_range / raytrace_max_range / inflation_radius / cost_scaling_factor / footprint 或 robot_radius / update_frequency`；TEB `include_costmap_obstacles / include_dynamic_obstacles / footprint_model / costmap_obstacles_behind_robot_dist / enable_homotopy_class_planning`；恢复行为配置。`scripts/nav_params_costmap.sh:6-21` 全列好了，**从来没有保存过输出**。

**只能从已知数推出的东西**（都是参数语义推论，不是真机实测）：
- 制动距离 v²/2a = 0.15²/(2×2.5) ≈ 4.5 mm，**可以忽略，反应延迟才是全部**。
- 一个 scan 周期（标称 0.1 s）内走 1.5 cm；控制循环 2 Hz 时每周期 0.15 × 0.5 = 7.5 cm 盲走；costmap 更新周期未知。
- 上游停止发令后 smoother 按 `velocity_timeout 1.0 s` 最多再输出 1.0 s ≈ 15 cm；relay 自己没有指令超时（`cmd_vel_relay.py` 无 timer）。
- progress_checker 0.25 m / 20 s：人站着不走，20 s 后判"无进展" → 控制器失败 → 行为树恢复（配置未知）→ 最终 goal ABORTED → `jr_mission.goto` 返回 False → `break`（`:165-166` / `:196-201`），**任务结束，不重试**。

**在"局部 costmap 用 `/scan` 做 obstacle_layer"这个未经证实的前提下**，人出现在前方会走四条路之一：(1) 有绕行空间：TEB 以 `min_obstacle_dist 0.10`（相对未知的 `footprint_model`）绕过——对人来说很近，接不接受是你决定；(2) 无绕行空间：全部轨迹不可行 → 恢复行为（若含 backup，倒车方向雷达盲）→ 20 s 后 ABORT → 任务结束；(3) 控制循环 2 Hz：反应距离无法估计；(4) 如果局部 costmap **不**用 `/scan`：Nav2 对人全盲，只按静态图走。**第一步就是读回。**

### 5.4 独立雷达守卫：放哪、门什么、阈值从哪来

**位置**：`src/jr_teleop/jr_teleop/cmd_vel_relay.py`——`nav.launch.py:34-35` 起的、Nav2 全部速度必经的最后一跳，是我们自己的 59 行代码（`CLAUDE.md:15` 铁律 1 不碰 vendor）。注意 `nav.launch.py:34` 现在起的是 **Python 包 `jr_teleop`** 的 relay，不是未跟踪的 `src/jr_teleop_cpp/`；守卫加在被 launch 的那个，两个副本别分叉。它**只覆盖 Nav2 路径**；`drive()`（`:128`）、`turn()`（`station_dock.py:149`）、键盘/手柄都直发 `/controller/cmd_vel`，不在它下游——这是有意的最小范围。

**输入**：`/scan`，sensor QoS（`CLAUDE.md:24` 铁律 7；`jr_grasp_all.py:117-119` 的 07-10 教训），扇区逻辑直接复用 `clearance()`（`:596-608`），只看**命令方向**的锥，不看全圆（过门、贴墙时全圆必误停）。

**规则（按优先级）**：
1. 命令方向扇区最小距 `< d_stop` → 输出零速。**只在收到输入时输出**，不加定时器连发零速。
2. `d_stop ≤ d < d_slow` → 三轴**等比**缩放（`cmd_vel_relay.py:16-23` 的 D6 教训：按轴钳位会改方向和曲率）。
3. scan 年龄 `> T_stale` → 零速。这与 `drive()` 现在"只警告继续走"（`:664-666`）**相反**；导航速度是抓取的两倍多，不能沿用那个选择。
4. 命令方向无雷达覆盖（倒车、后半圈）→ 不放行或限到极低速；要和 TEB `max_vel_x_backwards 0.08` 一起定（要么把 TEB 倒车设 0，要么守卫放行 ≤0.08）。
5. 纯旋转指令暂不门控（没有足迹数据无法定义安全半径），只记日志。
6. 时间用 `time.monotonic()`。

**阈值怎么来（仓库推不出来，要量）**：`d_contact` = 车头前沿贴墙时正前扇区 `/scan` 最小读数（雷达在 `base_link` 前 0.090 m，足迹前沿未知，直接量比换算可靠）；延迟 `L` = scan 周期（`ros2 topic hz /scan`）+ relay 处理 + 板子执行；`d_stop = d_contact + v_max·L + 余量`，`d_slow = d_stop + v_max × 若干周期`。`JR_DRIVE_DEBUG=1`（`:656-658`）能顺手打印 scan_age。**硬约束**：`d_stop` 换算到足迹边缘后必须**严格小于** TEB `min_obstacle_dist 0.10`，否则 TEB 认为可行的轨迹被守卫拦下 → 20 s 无进展 → 恢复行为——这就是"避障和急停打架"的具体形态。现成经验值 0.20/0.28（`:661`、`:56`）只做影子模式初值。

**替代方案**：Humble 的 `nav2_collision_monitor` 就是干这个的（stop/slowdown 多边形门控 cmd_vel）。是否装在厂家镜像里、厂家 yaml 有没有配——未知；如果有，优先用它而不是手写，但它需要的数（多边形尺寸、减速比）和上面一样要量。

### 5.5 怎么平衡避障和急停

分工：**规划层负责"绕"，守卫负责"最后一道不撞"**。三条规则：(1) 守卫的停止区被规划层的禁止区**严格包含**（§5.4 硬约束）；(2) 减速区可以比停止区大，但不能大到让 TEB 全程被限速——否则 progress_checker 0.25 m/20 s 把它当"没进展"；(3) 锥角只看运动方向（±24° 是 `clearance` 现用值），侧向交给 TEB `inflation_dist 0.30` 软惩罚。"人一直挡着"守卫解决不了（它只会一直停），需要任务层：`goto` 返回失败后有显式重试策略，而不是现在的 `break`。两个守卫覆盖不到的盲区要承认：后半圈（倒车）和低矮/悬空家具。

### 5.6 "在建好的图里导航都不稳"

证据里的不稳都有具体来源，不是"导航本身不行"：栈内参数互相矛盾（`engineering_decisions.md:100-104`；readback:21 又一例）、局部 keepout 把足迹关进 inscribed 环（handoff:42，已关）、CPU 2 Hz（`:45`，未解决）、时钟跳变（`:40`，门禁未加）、取货站离禁行区 0.18 m（`:42`、`:59`）。这些修一个少一个。

## 6. Q4 放置区：是架构问题吗？

**一半是，一半不是。**

### 6.1 现状链路（`place` 模式，`jr_grasp_all.py:1357-1388`）

`board_alive` → `[JR_DOCK=1] face_station`（失败只打印 `NOT FOUND, continuing`，`:1368`）→ OBSERVE → `scan_zone`（`:1055-1130`；面积 >0.45 m² 拒绝 `:1118`）→ 凸包中心 x>0.34/<0.20 或 |y|>0.18 时微接近并重扫（`:1377-1384`）→ `place_on_paper(off)`（`:1385`，**不传 `placed`**）→ 四层兜底串联（米制凸包 `:1184-1189` → 未确认暗块逼近 `:1163-1183` → 旧像素路径 `:1190-1234` → 固定前方点 `:1235-1239` "paper never seen!"）→ `_lower_and_release` **无条件张爪**（`:1261`），连固定点 IK 都无解也在当前位姿空中释放（`:1250-1261`）→ `return`（`:1388`，exit 0）。

放置本身的物理链很长：导航只能停到 ±12 cm/0.2 rad（`stack_wait.py:32-33`）且 `jr_mission.py:187-189` 注释承认 ±20 cm → 必须靠视觉找垫子 → 暗垫 + AprilTag（`_zone_detect` `:445-483`）→ 标签 >0.55–0.7 m 读不出（`:1164-1167`、`station_dock.py:248-249`）→ 必须靠近 → 夹爪里的方块挡住画面下半中间（handoff:72；与 `gripper_colour_px` 的 ROI `h*0.6` 以下、`w*0.3–0.7` 同区域，`:765`）→ 必须退开/转头 → 转头毁 AMCL（handoff:44）。这些是**感知几何约束叠加**，换什么架构都得面对。

### 6.2 今晚三类放置失败的性质

| 失败 | 根因 | 性质 |
|---|---|---|
| mission8 扫描拼不出垫子（4 视角明明检出标签） | 地面拟合缓存用 `id(self.depth)` 当键，numpy 复用地址 → 一次 `None` 永久缓存（handoff:44） | **当天写、当天上车、没测的 bug——流程错误**。已改 `_depth_seq` 帧计数键（`:148-150`、`:1019-1025`），**未真机验证**（handoff:57） |
| mission8 找板转 355° 把 AMCL 转丢 | `face_station` 的"找不到就转"没有上限、没有对定位副作用的后置条件 | **当天新加行为的设计错误**。已限 140°（`station_dock.py:208`），仍无转后定位检查，未验证 |
| 00:45–01:15 三次"看不见板" | 夹爪里的方块挡住 OBSERVE 画面下半中间，板在 0.4–0.65 m 正前方整块落在遮挡区（handoff:72）；空夹爪探针 8 标签成功是误导（`:73`） | **物理/视角** + **测试流程错误**（探针没带方块） |

### 6.3 结构性缺口（这些才是"为什么搞成这样"）

1. **失败不可见**：place 永远 exit 0；没见垫子也张爪放地（`:1235-1240` → `:1261`）；任务层区分不了"放上垫子"和"放地"，`jr_mission.py:191-193` 的安全释放到不了。
2. **兜底堆叠**：每次真机失败都在 `place_on_paper` 里再加一层（07-11 暗垫、07-15 面积上限、07-16 逼近确认），而不是让它返回失败把决定交给上层。
3. **两个检测器共用一个兜底、职责混了**：非 strict 的 `paper_contour`（`:439-443` → `:481-482` 最大暗块兜底）在取货区会把标定纸/阴影当放置区，把旁边方块排除成"已放置"（`:1406-1409` 注释；handoff:19"取货区不放标定纸"）。
4. **前置条件没被检查**："板在可见视野"从未被验证；`face_station` 返回 `None` 分不开"没有板"和"被遮挡"。
5. **跨进程状态丢失**：`zone_hull / place_n / placed_pts` 全是进程内变量（`:133-145`），`place` 模式 `placed=()`（`:1385`）→ 第二趟可能叠在同一"最深入"格上（`:1153-1156`）。
6. 放置段 10-22 的新代码（`_depth_seq` 键、盲退 0.25 m、140° 上限、每姿态 0.6 s×3 `station_dock.py:124-129`）**没有任何真机记录**。

### 6.4 直接回答"是架构问题吗"

今晚三次失败的**触发原因**都不是架构（bug、物理遮挡、测试没带方块）；但**放置"搞成这样"是架构**：一个只该做"把东西放到已知凸包里"的技能，因为没有返回合同，被迫把找板、逼近、四层兜底、放地全吞进去。

### 6.5 解法（不乱改，每步可验）

1. **先验证不改代码**：带方块、板中心 ≥0.9 m（车头前沿离纸近边 ≥80 cm，handoff:75），`grab_frame.py` 看 tags>0，再单独跑 `python3 jr_grasp_all.py place` 看 `[ZONE] mapped`（handoff:57）。顺便拿到遮挡区的真实像素范围（`cam2.jpg` 不在仓库）。
2. **把死代码接活**：`place` 模式在 `zone_hull` 为 `None` 时**不张爪**、`sys.exit(2)`，让 `jr_mission.py:191-193` 已有的 FLOOR 安全释放接管。给 `place_on_paper` 加 `release_on_floor` 参数，False 时不进旧像素路径直接返回 `(None, False)`。真机验证：故意把板遮住跑一次，看退出码 =2 且夹爪仍闭合。
3. **`find_zone` 三态输出**：MAPPED(area, turned_deg) / NOT_VISIBLE / REJECTED，任务层决定接不接受"转了 140° 才看见"。
4. 遮挡问题的两个候选（handoff:75）：站位 ≥80 cm，或给 `look()` 加一个抬腕/侧倾姿态。**哪个对取决于第 1 步量出的遮挡范围**，不先选。
5. 布置：取货区不放标定纸（handoff:19）。不要去"把放置做得更聪明"（更多视角、更松阈值、更大转角）——今晚的两个 bug 都来自这种改法。

## 7. 今天的流程错误 vs 结构性缺口

| 流程错误（下次靠纪律避免） | 结构性缺口（靠包装/守卫补） |
|---|---|
| 当晚改 `id(self.depth)` 缓存、当晚上车（handoff:44） | 子任务无退出码，`jr_mission.py:191` 兜底到不了 |
| 车头朝墙发初始位姿（`:41`） | 初始位姿协方差硬编码 0.05（`stack_wait.py:126`），无定位健康判据 |
| 空夹爪做标签探针（`:73`） | 放置无条件张爪、无失败状态（`jr_grasp_all.py:1261`、`:1388`） |
| 没等 NTP 就起栈（`:40`、`:56`） | `drive`/`jr_mission`/`station_dock` 仍用 `time.time()` |
| Foxglove 开原图面板（`:40`） | Nav2 路径无雷达守卫（`cmd_vel_relay.py` 全文） |
| 取货站离禁行区 0.18 m（`:42`、`:59`） | 无 pid 记账、无 `stop_mission.sh`、`TimeoutExpired` 未捕获 |
| `face_station` 当晚新加、无上限（`:44`） | costmap/TEB/AMCL 参数从未读回存档 |
| | `mission_boot.sh:14` `\| tail -1` 吞掉 `stack_sweep.sh:13` 的 `exit 1`，残留进程时照样起第二个 bringup（`CLAUDE.md:17` 铁律 2 的锁死场景） |
| | 闭环 stage1 失败静默回退开环（`:925` → `:941-952`）；子进程日志截 14 行 |

## 8. 优先行动（摘要；详细 why/evidence/effort 见 top_actions）

0. 把当前工作树提交成基线（含 `src/jr_teleop_cpp/`），之后每步单独提交。
1. 读回并存档 costmap / TEB 障碍 / AMCL 参数（零风险，10 分钟）。
2. 真机验证放置段新代码（不改代码，带方块探针，≥0.9 m）。
3. 量雷达三个数 + 标定 `JR_SCAN_YAW`。
4. 退出码契约 + `RESULT` 末行 + 子进程全文日志（零行为改动）。
5. place 不张爪于未知处 → exit 2 → `jr_mission` 低位释放接管。
6. 任务 pid 记账 + `stop_mission.sh` + `TimeoutExpired` 处理。
7. `mission_boot.sh` 等 NTP；不吞 `stack_sweep` 退出码。
8. `init_pose` 协方差可传参（默认待确认）；`goto` 后记录 `/amcl_pose` 协方差（先只记录）。
9. `cmd_vel_relay` 雷达守卫影子模式。
10. 降负载：foxglove `topic_whitelist`；查 `enable_homotopy_class_planning`。
11. 闭环 stage1 失败显式报告；`JR_HOLD_MIN=8000`。
12. 三个文件 `time.monotonic()`（独立机械提交，在 7 之后）。
13. 取货站东移 0.15 m（只改 `mission.json`）。
14. 之后：闭包提成 `RunState` → `jr_skills.py` → `task_fetch_place.py` 并行跑一趟对比。

## 9. 不要做的清单（都是"凭空臆想"式改法）

1. **不整体重写 `jr_grasp_all.py`**，不一上来把 `jr_mission` / `run` / `place` 合成一个进程——`M6_mission_design.md:8-13` 单批单进程纪律是锁死换来的，合并后对板子锁死和 DDS 的影响没有数据。
2. **不改 AMCL 任何参数**——仓库里没有一条它的实值，改了无法对照。
3. **不改 costmap `obstacle_layer`/`inflation_radius`/足迹，也不改 TEB `min_obstacle_dist`/`inflation_dist`**——一条真机值都没读回。
4. **不用拍脑袋的阈值上守卫**：0.20/0.28 是 0.07 m/s 抓取 `drive()` 的雷达读数；先量 `d_contact`、`/scan` 频率、scan_age，先影子模式。
5. **不让守卫"倒退避让"、不让 Nav2 更多倒车**：后向雷达盲（`CLAUDE.md:80`、`jr_grasp_all.py:1499`）。顺带：handoff:20"控制器不倒车"与补丁 `max_vel_x_backwards 0.08`（`stack_wait.py:34`）不一致，需要你定。
6. **不给 relay 加定时连发零速的看门狗**——会和 `drive()` 抢 `/controller/cmd_vel`；只做边沿触发。
7. **不把 0.15/0.6 协方差直接写死为默认**——先确认 mission8 用的是哪组。
8. **不重写 `scan_zone` / `place_on_paper` / `face_station`**，不加更多视角、更大转角、更松阈值。
9. **不把 place 改成"永不张爪"而没有任务层策略**——方块卡在爪里同样是失败，决定权要在任务脚本。
10. **不通过改 OBSERVE 位姿或 `HAND2CAM` 来"修"方块遮挡**——遮挡范围没量；`HAND2CAM` 是 09-03/09-04 手眼标定过的，动它会毁掉所有投影。
11. **不碰 vendor laser_filters 去开后半圈**（`CLAUDE.md:15` 铁律 1；厂家为什么裁不知道）。
12. **不重新开 `JR_DOCK_PICKUP`**（`jr_grasp_all.py:1406-1409` 的原因还在）。
13. **不在没有读回 `enable_homotopy_class_planning` 之前把它当 CPU 175% 的定论**（handoff:55 说的是"嫌疑"）。
14. **不把 `time.time()` 替换和行为改动混在一个提交里**；先做 NTP 门禁（根因）。
15. **不在同一晚既改代码又录像**：每个改动单独上车验证一条，再合。
16. **不 `pkill -f`**（`CLAUDE.md:19`、handoff:18）；插线模式不发任何底盘指令。
17. **不在 Windows 侧改完直接推**：本机没有 Python，语法只能靠 `go.sh:12` 的 `py_compile`；改 `jr_grasp_all.py` 后先在车上 `python3 -m py_compile`。

## 10. 需要你确认的事

1. mission8 起步用的初始位姿协方差是 0.05/0.05（`stack_wait` 路径）还是 0.15/0.6（手动 `initpose.py`）？这决定第 8 项的默认值。
2. "撞墙那几次"大致发生在哪个阶段（起步转向、长直线、到站前）？有没有 `~/nav.log` 片段？
3. 把 `~/jetrover_ws/grasp_stats.csv` 拷回来，看今晚每次 attempt 的 `result`。
4. 车上 `jr_env.sh` 是否真有 UDP-only DDS profile（仓库副本 `scripts/jr_env.sh:1-19` 没有）。
5. `src/jr_teleop_cpp/` 的 C++ relay 是学习用还是打算替换 `nav.launch.py:34` 起的 Python relay？守卫只加在被 launch 的那个。
6. 倒车到底要不要：handoff:20"不倒车" vs `stack_wait.py:34` 0.08。


## top_actions
[
 {
  "action": "提交当前工作树作为基线：7 个已修改文件（CLAUDE.md、scripts/jr_grasp_all.py、jr_mission.py、mission_up.sh、nav_lean_up.sh、nav_patch_all.sh、src/jr_nav/launch/nav.launch.py）+ scripts/ 下 21 个未跟踪脚本 + docs/ 9 个 + maps/ 17 个 + src/jr_nav/launch/filters.launch.py + src/jr_teleop_cpp/；之后每一步小改单独提交",
  "why": "没有基线就没有可审的 diff；今天被烧就是因为改动不可追溯；用户也明确要求 commit",
  "evidence": "本次 git status 全量输出；git log -1 = 5273127（2026-09-03）；src/jr_teleop_cpp/src/cmd_vel_relay.cpp:1-3（两份分析都漏掉的未跟踪 C++ relay 移植）",
  "effort": "小：10 分钟，不碰车"
 },
 {
  "action": "在车上跑 scripts/nav_params_costmap.sh 并把输出存进 docs/nav_params_costmap_<date>.txt；给 scripts/nav_params_dump.sh 加一段 ros2 param dump /amcl 存进 docs/amcl_params_<date>.txt",
  "why": "Q3 的核心问题（人出现在前面 costmap 会怎样、守卫阈值与 TEB 是否打架、CPU 175% 的嫌疑）和 Q2 的 AMCL 转丢，在仓库里没有一条实值；任何阈值/参数改动都没有对照基准；零风险纯读取",
  "evidence": "docs/nav_params_readback_2026-10-21.txt 仅 29 行且 :1 max_vel_x、:17 controller_frequency 无值；scripts/nav_params_costmap.sh:6-21 已列全 footprint/obstacle_layer/inflation/TEB 障碍/homotopy 键但 docs/ 无输出；全仓库 grep max_particles|alpha_slow|laser_model_type|update_min_d 零命中；scripts/nav_params_dump.sh:5-18 不查 /amcl；scripts/nav_params_filters.py:85-120 对 AMCL 段原样透传",
  "effort": "小：车上 2 条命令约 10 分钟，不改任何逻辑"
 },
 {
  "action": "真机验证放置段新代码（不改代码）：带方块、板中心 >=0.9 m（车头前沿离纸近边 >=80 cm），先 grab_frame.py 在观察位确认 tags>0，再单独跑 python3 jr_grasp_all.py place 看 [FACE]/[ZONE] mapped；同时记下带方块时画面被遮的像素范围；取货区不放标定纸",
  "why": "10-22 的 _depth_seq 缓存键修复、盲退 0.25 m、140° 上限、每姿态 0.6 s×3 全部未在真机验证；00:45 三次失败根因是带方块时板被遮挡而探针是空夹爪做的；遮挡范围未知决定了后续 find_zone 的方案（站位 vs 抬腕姿态）",
  "evidence": "docs/handoff_2026-10-21_night.md:57（未验证）、:72-75（遮挡根因、探针必须带方块、>=80 cm）、:19（取货区不放纸）；scripts/jr_grasp_all.py:148-150、:1019-1025（新缓存键）、:765（夹爪 ROI 与遮挡区同位置）；scripts/station_dock.py:208、:215-223、:124-129（新行为）",
  "effort": "小：不改代码，车上 10-15 分钟，电池模式"
 },
 {
  "action": "量雷达三个数并标定安装偏航：ros2 topic hz /scan、angle_min/angle_max（真实盲区）、车头前沿贴墙时正前扇区最小读数 d_contact；用 scripts/scan_probe.py 定 JR_SCAN_YAW；顺手用 JR_DRIVE_DEBUG=1 看 scan_age",
  "why": "守卫的 d_stop/d_slow/T_stale 只能从这三个数推，仓库推不出来；JR_SCAN_YAW 若不为 0，现有 drive() 急停查的就是错方向",
  "evidence": "scripts/jr_grasp_all.py:55（SCAN_YAW 默认 0，'probe on-robot'）、:605（偏航进扇区计算）、:111（10 Hz 仅为注释）、:656-658（JR_DRIVE_DEBUG 打印 scan_age）；docs/M6_mission_design.md:34；docs/robot_description_2026-09-04.urdf.txt lidar_joint xyz 0.0900（足迹前沿未读回，所以直接量）",
  "effort": "小：车上 15 分钟静止测量"
 },
 {
  "action": "退出码契约（零行为改动）：jr_grasp_all.py main() 各出口 sys.exit(0 达成 / 2 前置不满足：无相机、舵机桥、BOARD DEAD、看不见板 / 3 floor clear / 4 执行失败已安全：ABORT、放地) + 末行 RESULT <mode> <status> k=v；jr_mission.run_grasp 改读退出码并把子进程全文写到 mission_grasp_<n>.log；jr_mission 任一 goto 失败或 place 非 0 后 MISSION DONE 退出码非 0",
  "why": "这是 Q1 '一个文件跑完自退' 真正缺的东西：外层无法区分成功/失败，jr_mission.py:191 已写好的安全低位释放到不了；也是今晚说不清'用没用闭环'的直接原因（[CL] 行被 14 行截断吃掉）",
  "evidence": "scripts/jr_grasp_all.py 全文 grep sys.exit 零命中；:1298-1304、:1361-1362、:1388、:1764-1765 所有出口 return；:1643-1644 异常只 print+break；scripts/jr_mission.py:114-119（子串解析 'CARRYING'、tail 14 行 :116-118）、:190-193（r != 0 只有未捕获异常才到）、:165-166/:197-201（失败只 break）、:202-204（返回值被丢）、:205-207；scripts/stack_wait.py:153/161 已有可照抄的 0/1 模式",
  "effort": "小-中：约 30 行纯包装；本地只能靠 go.sh:12 的 py_compile；车上 python3 jr_grasp_all.py place; echo $? 验证一次"
 },
 {
  "action": "place 不张爪于未知处：place_on_paper 加 release_on_floor 参数，place 模式在 zone_hull 为 None 时不进旧像素路径、不张爪、sys.exit(2)；face_station 失败同样 exit 2；让 jr_mission.py:191-193 已有的 FLOOR 安全释放接管",
  "why": "把已有死代码接活而不是加第五层兜底；任务层从此能区分'放上垫子'和'放地'；放置段'失败'从此有一个状态",
  "evidence": "scripts/jr_grasp_all.py:1235-1240（fallback FLOOR spot 'paper never seen!'）、:1250-1261（IK 全无解仍 :1261 无条件张爪）、:1368（face_station NOT FOUND, continuing）、:1385（不传 placed）、:1388（exit 0）；scripts/jr_mission.py:190-193",
  "effort": "小-中：约 1 小时；真机验证一次：故意遮住板，看退出码=2 且夹爪仍闭合，再看 jr_mission 低位释放"
 },
 {
  "action": "任务记账与停止：mission_launch.sh 写 mission.pid（组号），新增 scripts/stop_mission.sh 按该 pgid 发 TERM 后立刻 ros2 topic pub --times 5 -r 10 /controller/cmd_vel 零速；jr_mission.run_grasp 用 try/except subprocess.TimeoutExpired 包住并在超时后发零速",
  "why": "没有 stop 脚本意味着中途只能连 bringup 一起 sweep（违反铁律 3）；TimeoutExpired 未捕获会 SIGKILL 子进程，drive() 末尾的零速发不出，控制板有无看门狗未知",
  "evidence": "scripts/mission_launch.sh:6-7 只 echo 组号不写 pid；ls scripts/ 无 stop_mission.sh；scripts/jr_mission.py:114-115 subprocess.run(... timeout=700) 外无 try；scripts/jr_grasp_all.py:667-668 零速在 drive 末尾；scripts/nav_stop.sh:6-12 已有可照抄的 TERM+零速写法（但它 sleep 3 后才发零速，新脚本要先发）；CLAUDE.md:19 铁律 3、:21 铁律 5",
  "effort": "小：一个 10 行 shell + jr_mission.py 约 8 行；车上验证：起任务 → stop_mission.sh → 看 /controller/cmd_vel 零速与进程退出"
 },
 {
  "action": "mission_boot.sh：开头等 timedatectl show -p NTPSynchronized 为 yes（上限 60 s）再起 launch；第 14 行 bash stack_sweep.sh | tail -1 改为检查其退出码（LEFTOVERS 时 exit 1 不起 bringup）",
  "why": "#5 的时钟跳变根因只修了 stack_wait 的计时没加门禁（时钟跳变期间起的 AMCL 丢过雷达队列）；| tail -1 吞掉 LEFTOVERS 的 exit 1，会在残留进程存在时起第二个 bringup，正是铁律 2 的板子锁死场景",
  "evidence": "docs/handoff_2026-10-21_night.md:40、:56；scripts/mission_boot.sh:1-30 无 timedatectl、:14-15 管道后无条件 setsid bringup；scripts/stack_sweep.sh:13 LEFTOVERS 时 exit 1；scripts/rewait.sh:6 只打印同步状态；CLAUDE.md:17 铁律 2",
  "effort": "小：shell 5-8 行；下次开机即可验证（故意留一个进程再跑 mission_boot 看是否拒起）"
 },
 {
  "action": "定位健康可见化（先只记录）：stack_wait.init_pose 增加 cov_xy/cov_yaw 命令行参数（默认仍 0.05，mission_boot.sh 透传；是否改 0.15/0.6 等用户确认 mission8 用的哪组）；jr_mission 的 /amcl_pose 回调同时保存 covariance，每段 goto 结束打印 xx/yy/aa",
  "why": "#6 的摆放误差 ±45° 超出 σ_yaw 12.8° 的先验而 handoff 建议的宽协方差没有落到启动脚本；今天没有任何代码能看出定位丢了，先积累'好/坏'协方差数据再定阈值",
  "evidence": "scripts/stack_wait.py:126 cov[0]=cov[7]=cov[35]=0.05 硬编码、:131 只打印订阅者数、:144-145 只解析 x y yaw；scripts/initpose.py:2,7-8 已有可照抄的参数写法；scripts/jr_mission.py:47-48 只存 pose.pose；scripts/cost_at.py:51 唯一打印协方差处（手动）；docs/handoff_2026-10-21_night.md:44（aa 1.06）、:58",
  "effort": "小：Python 约 12 行；跑一趟任务看日志"
 },
 {
  "action": "给 src/jr_teleop/jr_teleop/cmd_vel_relay.py（nav.launch.py 起的那个，不是 src/jr_teleop_cpp/）加 /scan 订阅（qos_profile_sensor_data）和按指令方向 ±24° 扇区最小距 + scan 年龄计算，用 time.monotonic()；第一版影子模式只打印 would-stop/would-slow/scan_age 不改输出（影子阈值暂用 0.20/0.28 计数），跑一整趟任务统计过门/贴茶几的误触率；之后用 JR_GUARD=1 开 gating：<d_stop 零速、d_stop–d_slow 三轴等比缩放、scan 过期零速、倒车方向不放行或 <=0.08、纯旋转只记日志、只在收到输入时输出不加定时器；正式 d_stop/d_slow 由第 2、4 项读数定且换算到足迹后必须 < TEB min_obstacle_dist 0.10",
  "why": "Q3 的根本事实：今晚撞墙时是 Nav2 在开车，而雷达急停只存在于 jr_grasp_all.drive() 里；relay 是 Nav2 到电机的唯一一跳，改它不碰 vendor；影子模式能在不冒撞车风险的前提下校准阈值并暴露与 TEB 打架的情况",
  "evidence": "src/jr_teleop/jr_teleop/cmd_vel_relay.py:26-27 订阅 cmd_vel 发布 controller/cmd_vel，全文无 LaserScan、无 timer，:16-23 等比缩放理由；src/jr_nav/launch/nav.launch.py:34-35 起的是 package='jr_teleop' 的 Python relay；scripts/jr_grasp_all.py:56 CLEAR_MARGIN 0.28、:661 急停 0.20、:596-608 锥选逻辑、:117-120 sensor QoS 教训、:664-666 stale 继续开（不要复制）、:611 drive 默认 0.07 m/s；stack_wait.py:30-31 TEB 0.10/0.30、:34 backwards 0.08、:37 0.15 m/s；readback:27-29 减速/超时；docs/M6_mission_design.md:32 Nav2 无目标时不发速度",
  "effort": "中：约 40-60 行；两阶段验证（影子 → 使能）；依赖第 2、4 项的读数定最终阈值"
 },
 {
  "action": "降负载：deploy/foxglove_up.sh 给 foxglove_bridge 加 topic_whitelist（按 handoff:55 列表）；从第 2 项的读回确认 FollowPath.enable_homotopy_class_planning，若为 True 用 setp.py 关掉并观察 nav 容器 CPU 与 ros2 topic hz /cmd_vel",
  "why": "#9 未解决：控制循环 2 Hz 直接决定守卫与避障的反应延迟（0.15 m/s × 0.5 s = 7.5 cm 盲走），比任何阈值都重要",
  "evidence": "docs/handoff_2026-10-21_night.md:45（控制循环 2 Hz 未解决）、:55（白名单与 homotopy 嫌疑）；deploy/foxglove_up.sh:64-65 默认 launch xml 无 whitelist；scripts/nav_params_costmap.sh:19 能读 homotopy 但无存档",
  "effort": "小-中：shell 1 行 + 可能 1 条 setp；验证用 top 与 ros2 topic hz"
 },
 {
  "action": "闭环可见性：grasp_closed_loop 在 stage1 返回 lost/implausible/no-ik 时打印醒目 '[CL] FALLBACK open-loop' 并 log_stat 一条 CL_FALLBACK；mission_launch.sh 加 JR_HOLD_MIN=8000",
  "why": "用户感觉'没用闭环'的一个真实机制是 stage1 失败后静默按开环目标下探合爪；HOLD 判空默认关闭导致抓空仍报 CARRYING（证据薄：只有 2 个样本）",
  "evidence": "scripts/jr_grasp_all.py:925 条件、:941-952 无条件下探合爪返回 LIFTED、:935-936 现有日志行、:1716-1717 JR_HOLD_MIN 默认 0；docs/handoff_2026-10-21_night.md:51 HOLD 像素 18-19k 可设 8000",
  "effort": "小：约 5 行 + 1 个环境变量；验证看 mission.log 的 [CL]/[HOLD] 行"
 },
 {
  "action": "独立的机械提交（在 NTP 门禁之后）：把 scripts/jr_grasp_all.py（_s/fresh/wait_bridge/drive）、scripts/jr_mission.py（spin/arm/goto）、scripts/station_dock.py（turn）的 time.time() 计时改为 time.monotonic()，不改任何逻辑",
  "why": "与 stack_wait 同一类 bug：NTP 跳变会让 drive() 的 scan 新鲜度判断、NAV_TIMEOUT、fresh(15 s) 全部失真；急停误判比体检误判更危险；但它是症状修复，不能和行为改动混在一起",
  "evidence": "scripts/jr_grasp_all.py:154、:162-163、:173-174、:617-618、:626-627、:644-645、:651-652、:658-659、:666；scripts/jr_mission.py:51-52、:56-57、:90-91；scripts/station_dock.py:138-139、:146-163；对照 scripts/stack_wait.py:52-59、:118-119 已改 monotonic",
  "effort": "小：机械替换约 25 处；车上 py_compile + 一次短距 drive 验证"
 },
 {
  "action": "取货站东移 0.15 m：docs/mission_2026-10-21.json zones 改为 [[3.45,1.25,90]]（go.sh:11 会推成车上 mission.json）；其余不动",
  "why": "#7 根因是抓取后停在离茶几禁行区 0.18 m；局部 keepout 已关但全局仍在，站点挪开比再改 costmap 参数更可控",
  "evidence": "docs/handoff_2026-10-21_night.md:42、:59；scripts/stack_wait.py:41-45 全局 keepout 开/局部关；docs/mission_2026-10-21.json:3-9；scripts/go.sh:11",
  "effort": "小：改 1 个数字；车上验证一次 goto 到新站点并 cost_at.py 看代价"
 },
 {
  "action": "（在退出码契约落地之后）把 main() 的闭包和 nonlocal 状态提成 RunState 类（纯搬家，方法签名不变），再写 jr_skills.py 薄封装（goto/find_object/approach/grasp/carry_verify/find_zone/place 各返回明确状态）和 task_fetch_place.py，与 jr_mission.py 并行存在，跑一趟对比日志后再切换",
  "why": "这才是'一文件一任务'真正的形态，也是让 put_down.py/place_floor.py 不再戳私有方法的前提；但必须在返回合同落地、且每步能用 survey/grasp1 dry 对比验证之后做",
  "evidence": "scripts/jr_grasp_all.py:1414-1421、:1492-1495、:1423-1584（闭包与状态）；scripts/put_down.py:10-11、scripts/place_floor.py:9-10（只能戳私有方法）；docs/M6_mission_design.md:8-13（现架构，不合并进程）",
  "effort": "大：半天；验证 = survey + grasp1 dry + 一趟任务对比"
 }
]