# 交互式"点击即抓"(class-agnostic interactive grasp) — 设计

> 起因(2026-06-25,用户提出):不靠类别识别——**只要分出"地板之外的物体"轮廓**,在笔记本看到相机画面+标注,
> **鼠标点一个物体就抓它**。这从根本上绕开 COCO 类别限制(不需要"认识"物体)。
> 复用 M5 抓取管线(`docs/M5_grasp_design.md`)和多后端检测框架(`docs/M4_detection_design.md`)。

## 1. 概念

```
深度去地板 → 物体团块(任意物体) → 叠标注发压缩流 → 笔记本显示
                                                        │ 用户点击某物体像素
                                                        ▼
                                          /jr/click (u,v) → 命中哪个团块 → 该团块质心+深度
                                                        ▼
                                          已有 深度→3D→hand-eye→IK→抓取 管线
```

**三个关键决策**:① 分割靠**深度去平面**(不靠模型);② 看图靠**压缩流 + 自写点击小窗**(低带宽、直观);③ 点击像素**映射到团块**再走老管线。

## 2. 类无关分割:深度去地板平面(本方案的核心)

**思路**:eye-in-hand 在观察位俯视,**地板是一个占主导的 3D 平面**;放在地板上的物体会**凸出于该平面**。

**算法**(在深度图/点云上,GPU-free):
1. 深度图反投影成 3D 点(用深度内参),可降采样(如每 4 像素)提速。
2. **RANSAC 拟合最大平面** = 地板(法向应≈竖直,可加约束剔除墙面)。
3. 标记**到平面距离 > 阈值(如 1.5cm)且在平面上方**的点 = 物体点。
4. 把物体点投回图像 → 形态学 → **连通域/轮廓** → 每个团块:bbox + 质心 (u,v) + 像素数 + 平均高度。
5. 过滤:面积下限(去噪)、高度下限(去地面起伏)、可达性(质心 3D 在臂可达区内才算可抓,否则标灰)。

**为什么对**:不依赖任何类别/颜色;红方块、螺丝刀、瓶子、U盘**一视同仁**都能分出来。这是 tabletop/floor-segmentation 的经典做法,Jetson 上够快(RANSAC + 连通域,无需 SAM 之类重模型)。
**备选**:点云 `/depth_cam/depth/points` 直接做 RANSAC + 欧式聚类(PCL 风格);或重模型 SAM(Jetson 偏重,不首选)。

## 3. 标注画面

- segmenter 节点把每个团块画框 + 编号(+可达=绿/不可达=灰)叠到 RGB,发 `/jr/scene/annotated`(+`/compressed`,JPEG,同 M4 做法,2.4G 友好)。
- 同时发 `/jr/scene/objects`:团块列表 `[{id,u,v,bbox,reachable,dist}]`(供点击映射 + 调试)。

## 4. 笔记本点击小窗(交互层)

**自写 rclpy 小节点(笔记本端)** `jr_click_viewer`:
- 订阅 `/jr/scene/annotated/compressed`(CompressedImage)→ `cv2.imdecode`(**不需要 cv_bridge**,np.frombuffer+imdecode)→ `cv2.imshow`。
- `cv2.setMouseCallback`:左键点击 → 取像素 (u,v) → 发布 `/jr/click`(`geometry_msgs/PointStamped`,x=u,y=v,frame="image";或自定义小msg)。
- 低带宽(压缩流)、直观(直接点相机画面里的物体)、零额外依赖(cv2 已有)。
- 放进 `scripts/`,一条命令起(像 view.sh)。
**备选**:RViz 的 PublishPoint 工具点点云→`/clicked_point`(3D),机器人找最近团块。直观性差些、点云过网带宽大,不首选。

## 5. 点击 → 物体 → 抓

- 机器人侧 grasp/segmenter 订阅 `/jr/click` (u,v):
  - 找**掩膜包含 (u,v)** 的团块(或质心最近的团块)。
  - 命中 → 取该团块**质心 (u,v) + 中值深度** → 走 M5 管线(深度→相机系→hand-eye→臂基座→IK→抓取序列 + 验证/重试)。
  - 没命中物体(点到地板/空白)→ 状态反馈 "no object at click"。
  - 团块不可达 → 反馈 "out of reach"(M6 钩子:底盘靠近)。

## 6. 接口

- `/jr/scene/annotated` (+`/compressed`):标注画面。
- `/jr/scene/objects`(自定义 msg 或 String/JSON):团块列表。
- `/jr/click`(PointStamped,像素):笔记本点击。
- 抓取触发:点击直接触发,或点击→选中→再调 M5 的 `/jr/grasp` Action(target 用"选中团块的像素/id")。建议**点击=选中并触发**,简单直接。
- 参数:平面距离阈值、最小团块面积/高度、可达性范围、降采样步长。

## 7. 与多后端检测的关系

这是检测的**第三个后端 = "agnostic(去地板)" + 交互选择层**,与 color / yolo 并列,输出同一 `Target {u,v,label,bbox}`(label 可为 "object#3")。下游抓取管线不变。三种模式:
- `color`:按颜色自动选(抓红块)
- `yolo`:按类别自动选(抓 bottle)
- `click`(本方案):**人点哪个抓哪个,任意物体** ← 最通用、最稳

## 8. 边界/风险

- 物体**紧贴/堆叠** → 连通域粘连成一块:可用距离变换+分水岭分,或先做简单版(粘连就当一个,点击取点击点附近局部)。
- 地板**非纯平**(地毯/反光/地板纹理) → RANSAC 阈值要调;深度对**透明/黑色**物体仍可能空洞(玻璃问题依旧,但黑色不透明物体 OK)。
- eye-in-hand:分割/标注须在**臂稳定在观察位**时做(外参已知);点击→抓取期间用点击瞬间的团块坐标(臂动后视角变无所谓,坐标已定)。
- 点击延迟:压缩流~几Hz够用;点击的是"当前帧"的物体,抓取用那一刻的分割结果。

## 9. 实施计划(每步可单测)

1. **segmenter 离线先验证算法**:用一帧真机深度图(回来抓一张)在笔记本上跑 RANSAC 去平面+连通域,确认能分出方块/螺丝刀/瓶子(class-agnostic)。
2. segmenter 节点:发 `/jr/scene/annotated(/compressed)` + `/jr/scene/objects`。
3. 笔记本 `jr_click_viewer`(imdecode+imshow+鼠标→`/jr/click`),`scripts/` 起。
4. grasp 节点订阅 `/jr/click` → 命中团块 → 走 M5 抓取。
5. 端到端:笔记本点物体 → 机器人抓。
6. 接 M5 的验证/重试/PLACE/Action。

## 10. 真机待定项

1. 抓一帧真机深度图/点云,离线标定 RANSAC 平面阈值 + 团块面积/高度下限(本机地板/光照)。
2. 确认 `/depth_cam/depth/points` 是否组织化(便于投回图像),还是用深度图自投影。
3. 笔记本 cv2 窗口在 DISPLAY 下的鼠标回调 + rclpy 发布联调。
4. 点击像素 (u,v) 与团块掩膜的命中匹配(depth/rgb 对齐已确认逐像素一致)。
