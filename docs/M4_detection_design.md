# M4 物体检测 — 多后端设计

> 起因(2026-06-25,用户指出):只靠机器人内置 `yolo11n` 不够——**连标准色块都识别/抓取不了**。
> 本文档分析根因并定多检测后端方案。配合 M5 抓取(`docs/M5_grasp_design.md`):任何后端输出统一的
> "目标像素 (u,v) + 框/掩膜",喂同一条 深度→3D→IK→抓取 管线。

## 1. 根因:两个独立问题

| 问题 | 本质 | 模型变大能解吗 |
|---|---|---|
| **覆盖**:色块/螺丝刀/U盘检测不到 | 它们**不在 COCO 80 类**里 | ❌ 再大的 COCO 模型也认不出"红方块" |
| **精度**:COCO 物体也低置信(瓶0.3、牙刷漏检) | `yolo11n` 是最小 nano 模型 | ✅ 换 yolo11s/m 或微调能改善 |

→ 必须**分开治**:覆盖问题靠"换检测方式"(颜色/几何分割),精度问题靠"换大模型/微调"。

## 2. 方案:多检测后端,统一接口

下游抓取管线只关心一个**目标**:`(u, v, label, score, bbox)`。检测后端可插拔:

```
                       ┌──────────────────────────────┐
  RGB(+depth) ───►     │  detect(rgb, depth) -> Target │  ───► (u,v,label,bbox) ──► 深度→3D→IK→抓取
                       └──────────────────────────────┘
                          ▲           ▲            ▲
                   ColorDetector  YoloDetector  (未来:微调模型)
```

### 后端 A — 颜色+深度分割(色块/有色物体,**默认、首选**)
- 经典 CV:BGR→HSV(或 **LAB,匹配 vendor 标定**)→ `inRange(color)` → 形态学开闭 → 最大轮廓(>min_area)→ 质心 (u,v) + bbox。
- **红色 Hue 在 OpenCV 里跨 0/180,需两段范围合并。**
- 选哪个颜色:参数 `target_color`(red/green/blue),demo = "抓红色方块"。
- 优点:**不在 COCO 限制内、确定性、可标定、不吃 GPU、快**。色块/有色物体的正解。
- **复用 vendor 标定**:`track_and_grab.py` 用 `lab_data` + 颜色 tracker;vendor 有 LAB 颜色配置(红/绿/蓝阈值)+ 标定工具。**只读不改**,把阈值拿来用(真机确认路径,疑似 `lab_config`/`lab_data.yaml`)。

### 后端 B — YOLO(语义 COCO 物体:bottle/cup/can)
- 保留,用于需要**类别语义**的日用品。
- **精度免费提升**:nano→`yolo11s.pt`(机器人已有),GPU 仍跑得动。
- **微调(可选,后期)**:若锁定具体目标物,在**笔记本 RTX4060** 上拿少量标注数据微调 yolo11s。仅当 color-seg + 大模型仍不够时才上(成本:采集+标注+训练)。

### 选择策略(grasp 节点参数 `detector`)
- `color`:只颜色分割(抓色块,默认 demo)。
- `yolo`:只 YOLO(抓 bottle/cup)。
- `both`:先 color 后 yolo,或都跑取置信高的(灵活,后期)。

## 3. 为什么 color-seg 做默认是对的
- 机器人抓取的**经典目标就是彩色方块**;color-seg 是它的标准解(vendor + Panda 都这么做)。
- 确定性、可标定、零训练、零 GPU 负载,鲁棒于"不在 COCO"。
- YOLO 留给"我要抓那个**瓶子**(按类别)"这类语义任务。

## 4. 统一输出契约
所有后端返回(或 None):
```
Target = { u:int, v:int, label:str, score:float, bbox:(x1,y1,x2,y2) }
```
M5 的 `detect_target` 改为:按 `detector` 参数调对应后端 → 得 Target → 后面 depth 取中值、反投影、hand-eye、IK **完全不变**。颜色后端的 `label` = 颜色名,YOLO 后端的 `label` = COCO 类名。

## 5. 真机待定项
1. 读 vendor LAB 颜色配置/标定文件路径与格式(red/green/blue 阈值);确认 `track_and_grab` 的 tracker 实现细节。
2. 标定/微调本机光照下红/绿/蓝 HSV 或 LAB 阈值(`color_detect.py` 已留默认值待调)。
3. 对比 color-seg vs yolo11s 在实际目标上的检出率,定默认后端。

## 6. 实施计划
1. `jr_vision/color_detect.py`:参数化 HSV/LAB 颜色分割类(**离线先写骨架+默认阈值**,真机调)。
2. 真机标定颜色阈值(或读 vendor LAB 配置),验证色块质心+深度正确。
3. M5 `grasp` 节点加 `detector` 参数,`detect_target` 按后端取 Target(color/yolo)。
4. 抓色块跑通 → 与 yolo 路径并存。
5. (可选,后期)yolo11n→s,或笔记本微调。
