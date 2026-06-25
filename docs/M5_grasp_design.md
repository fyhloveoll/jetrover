# M5 抓取状态机 — 设计文档

> 状态:设计稿(2026-06-25)。当前 `jr_vision/grasp.py` 只实现了"一次性 happy-path 抓取"
> (检测→IK→接近→闭合→抬→回观察),**没有失败检测 / 重试 / 放置 / 状态机**。本文档定义 M5 要补齐的完整状态机。

## 1. 目标与边界

**M5 = 单点抓取状态机**:在机器人**已停在物体前**(底盘不动)的前提下,把视野里一个不透明 COCO 物体可靠地抓起、(可选)放到固定位置,并**检测抓取是否成功、失败则重试**。

- **属于 M5**:检测→规划→接近→抓→**抓取成功验证**→抬→放→失败重试;鲁棒性(超时、板子卡死中止);量化(成功率统计,为 M7 铺路)。
- **不属于 M5(留给 M6)**:用 Nav2 移动底盘靠近物体(M5 假设物体已在臂可达区 <30cm);动态放置点(导航到投放区)。M5 只做"物体太远→报告'需底盘靠近'"这个**M6 钩子**。

**已确认的硬约束(来自今天实测):**
- 臂可达区:前向 x≈0.10–0.30m、z≥0.00m;**地面物体须在车前 <30cm**,否则 IK 无解。
- 目标物须**不透明**(玻璃深度无效)且属 YOLO COCO 类(可乐→cup、塑料瓶→bottle)。
- 控制板可能**固件锁死**(≠button_scan,未根治):每个状态都要 `board_alive()` 守卫,卡死即安全中止并提示 reboot。

## 2. 状态机

```
              trigger
   IDLE ───────────────► DETECT
    ▲                       │ found target (opaque, valid depth)
    │ done/fail             ▼
    │                     PLAN ──── no IK solution ────► OUT_OF_REACH ─(M6 hook: "drive closer")─► FAIL
    │                       │ IK ok
    │                       ▼
    │                    APPROACH (hover above → descend)
    │                       │ settled
    │                       ▼
    │                     GRASP (close gripper)
    │                       │
    │                       ▼
    │                    VERIFY ── not held ──► RETRY (n<N? re-DETECT : FAIL)
    │                       │ held
    │                       ▼
    │                     LIFT
    │                       │
    │                       ▼   (still held after lift?  no → RETRY)
    │                     PLACE (to drop pose → open gripper)
    │                       │
    └─────────── HOME ◄──────┘
```

**状态职责 / 转移守卫:**

| 状态 | 动作 | 成功转移 | 失败转移 |
|---|---|---|---|
| IDLE | 等 trigger | →DETECT | — |
| DETECT | 快照 rgb+depth,YOLO,选**有效深度+居中**的目标 | →PLAN | 无目标→(重试/FAIL) |
| PLAN | depth→相机系→hand-eye→臂基座系→IK(pitch by height) | →APPROACH | 无解→OUT_OF_REACH |
| OUT_OF_REACH | 报告 base-frame 坐标 + "需底盘靠近 Δ" | (M6 接管) | →FAIL |
| APPROACH | **先到悬停位(目标上方 +5cm)再竖直下探**到抓取位 | settle→GRASP | 中途板子卡死→ABORT |
| GRASP | 闭合夹爪(限流) | →VERIFY | — |
| VERIFY | **读夹爪实际开度**判断是否夹住(见 §3) | held→LIFT | not held→RETRY |
| LIFT | 抬升 5–8cm,再次 VERIFY(防滑脱) | held→PLACE | dropped→RETRY |
| PLACE | 到固定投放位姿→张爪→确认放下 | →HOME | — |
| RETRY | n++;n<N 回 DETECT(物体可能移位/倒了),否则 FAIL | →DETECT | →FAIL |
| HOME | 回观察位姿,张爪(若空手)/闭合(若 PLACE 跳过) | →IDLE | — |
| ABORT/FAIL | 安全停,发布失败原因(板子卡死则提示 reboot) | →IDLE | — |

## 3. 抓取成功验证(M5 的核心难点)

**主判据 — 夹爪开度回读(immediate):**
- 闭合夹爪命令到固定 close 脉冲(如 600)后,settle ~0.5s,**读舵机 10 的实际位置**。
- 来源:话题 **`/controller_manager/servo_states`**(bringup 已发,含各舵机 id+实际 position;**待在真机确认确切字段名/单位**)。无需低层 SDK。
- 判定:
  - 空夹(夹爪一路合到底)→ 实际位置 ≈ 命令的全闭值 → **没夹住**。
  - 夹住物体 → 物体宽度挡住,夹爪**停在半途**(实际 < 全闭值 − margin) → **夹住**。
  - 阈值/ margin 需**在真机标定**:分别记录"空合到底"和"夹住可乐/杯子"的回读值,取中间。
- 优点:即时、不依赖视觉、不受 eye-in-hand 视角变化影响。

**辅判据 — 视觉消失(confirm after HOME):**
- 抬起 + 回到观察位后,在**原像素位置附近**重新检测:物体**不见了**=大概率被抓走;**还在原地**=抓空了(主判据误判/滑脱)。
- 处理 eye-in-hand:必须**回到观察位**(相机视角恢复已知)再做视觉复检。

**融合策略**:VERIFY 用开度(快、即时)做主判;LIFT 后 + HOME 后用视觉消失做二次确认。开度说"夹住"但视觉说"还在原地"=抬升中滑脱→RETRY。

## 4. 失败处理与重试

- **重试次数** N(默认 2,参数化)。每次 RETRY 重新 DETECT(物体可能被碰倒/移位,必须重新感知,不能用旧坐标)。
- **失败模式分类**(为量化/调试):`no_target` / `out_of_reach` / `ik_fail` / `grasp_empty`(开度判空)/ `slip`(抬升后掉)/ `board_hang` / `timeout`。每次尝试记录模式。
- **超时**:每个运动状态设 settle 超时;DETECT 设感知超时。超时→ABORT。
- **板子卡死**:任何状态 `board_alive()` 转 False → ABORT,夹爪/臂尽量停在安全态,报 "reboot"。

## 5. 接近几何(比 vendor 直接接近更稳)

- **悬停-下探**:先到"目标上方 +5cm"(同 x,y、z+0.05、同 pitch)的悬停位,再**竖直下探**到抓取位。避免斜插把物体撞倒。各需一次 IK。
- **pitch**:`80°(z<0.2 低物体,近俯冲)` / `30°(较高)`,沿用 vendor。
- **下探深度补偿**:`dist += 0.03`(半径+误差,沿视线射线,vendor 约定,已在 grasp.py)。

## 6. 放置(PLACE,M5 做)

- 抓起 + 验证 + 抬升后,移到**固定投放位姿** `place_pose`(预设 5 关节脉冲,参数化,如车体一侧 / 一个小筐上方),张爪放下,**确认放下**(夹爪开度回到"空"值)。
- `place_pose` 在真机录一个安全位姿(参考 vendor `track_and_grab` 的固定 place 位姿)。
- `do_place=false` 时跳过放置,抓住回观察位(纯 pick)。
- M6 再升级成"导航到投放区"的动态放置(M5 的 PLACE 接口/状态原样复用,只是位姿来源从固定改为导航目标)。

## 7. 接口设计 — Action 主接口(M5 做)

**主接口 = Action `/jr/grasp`**(长任务 + 实时反馈 + 可取消,适合多秒、带重试的抓取)。
需要一个小接口包 **`jr_interfaces`**(ament_cmake + rosidl)定义 `action/Grasp.action`:

```
# Goal
string target_class       # "" = 用节点配置的默认目标类
bool   do_place           # true: 抓后放到 place_pose;false: 抓住回观察位
---
# Result
bool                 success
string               failure_mode   # 成功为"";否则 no_target/out_of_reach/grasp_empty/slip/board_hang/timeout
string               object_class
geometry_msgs/Point  grasp_pose     # 臂基座系抓取点
int32                attempts
---
# Feedback
string state              # 当前状态名(DETECT/PLAN/APPROACH/...)
int32  attempt            # 第几次尝试
```

- **ActionServer** 用 `rclpy.action.ActionServer`,跑在 grasp 节点的 MultiThreadedExecutor 上;`execute_callback` 里跑状态机,逐状态 `goal_handle.publish_feedback(...)`;支持 `goal_handle.is_cancel_requested` 中途取消。
- **保留** `/jr/grasp/trigger`(std_srvs/Trigger)作便捷封装(内部调一次抓取,默认参数),命令行/调试用;`/jr/grasp/status`(String)继续发状态。

**参数**:`target_classes`、`max_retries`、`hover_height`、`close_pulse`、`grasp_detect_margin`(开度阈值)、`place_pose`、各状态超时、`servo_min_interval`、`imu_timeout`、`dry_run`。

## 8. 量化(为 M7 铺路)

M5 就开始记:每次抓取的 `(object_class, attempts, success, failure_mode, grasp_pose, duration)` → 落 CSV/rosbag。
跑 N=20–30 次(每种物体)算**成功率 + 失败模式分布**。这既是 M5 的验收,也是 M7 sim-to-real 对照的基线。

## 9. 真机待定项(机器人回来第一批要确认的)

1. `/controller_manager/servo_states` 的**确切消息类型/字段**(position 字段名、单位),以及夹爪 10 的"空合到底回读值" vs "夹住可乐回读值"→ 定 margin。
2. 悬停-下探两段 IK 在可达区边缘是否都有解(可能要微调 hover 高度)。
3. 放置固定位姿(录一个安全的车体侧位姿)。
4. 板子在"限流后的抓取指令序列"下还会不会卡(验证 §4 中止逻辑真能优雅停)。

## 10. 增量实施计划(每步可单测)

0. **`jr_interfaces` 接口包**(ament_cmake + rosidl,定义 `Grasp.action`)→ colcon build 通过。**离线可先建好骨架**,机器人回来只需 build。
1. **VERIFY 先行**:写"读 `/controller_manager/servo_states`→判夹爪开度"的小工具,标定空/满阈值(不动整条流程)。← 解锁核心难点
2. 把当前 `grasp_once` 线性流程**重构成显式状态机**(enum + 转移函数),happy-path 不变。
3. 加 **APPROACH 悬停-下探**(两段 IK)。
4. 接 **VERIFY + RETRY**(开度主判 + 重试 N 次)。
5. 加 **LIFT 后视觉复检**(回观察位重检测)。
6. 加 **PLACE**(固定 `place_pose`,`do_place` 控制)。
7. **Action 接口**:`jr_vision` 依赖 `jr_interfaces`,把状态机包进 `ActionServer /jr/grasp`(逐状态 publish_feedback、支持 cancel);Trigger 留作便捷封装。
8. 加**量化记录**(CSV:object/attempts/success/failure_mode/pose/duration)+ 跑 20–30 次出成功率。

> 实现时继续遵守:不改 vendor;只 SIGTERM 不 kill -9;舵机指令限流;每状态 `board_alive()` 守卫。
