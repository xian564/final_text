# my-project

## 装甲板尺寸与 PnP 约定

- 尺寸参数（`config/params.yaml`，单位米），来源考核文档：
  - 大装甲板：`armor_large_w=0.235`、`armor_large_h=0.127`（235×127 mm）
  - 小装甲板：`armor_small_w=0.120`、`armor_small_h=0.125`（120×125 mm）
- 代码位置：`src/detection_node_v3.cpp` 参数 63–66 行，热加载 272–275 行，PnP 使用 559–560 行。
- 顶点定义（旋转四点）：左灯条**左边缘**、右灯条**右边缘**各自的上下角点，按
  **TL → TR → BR → BL** 排列；再按板体长宽比 Wb/bH 把四边形高度拉伸到与板体一致，
  避免用灯条长度当板高导致 z 偏大。物体点按 Wb×bH 构造（米）。
- 与轴对齐 bbox 的区别：旧实现取 `min_x/max_x/min_y/max_y`，倾斜时四角不在灯条边缘上；
  现在四点随灯条旋转，倾角大时位姿更稳。
- PnP 结果做合理性过滤：`tvec.z ∈ (0.3, 15)` 才认为有效，过滤镜像/退化解。
- 装甲板坐标系：x 向右、y 向下、z 向前，右手系；单位为米。
- `header.frame_id` 使用 `camera_info.header.frame_id`（本数据为 `hk_camera_ca`）。

## 参数文件

- `config/params.yaml`（检测节点 `vision_node_v3`）：
  话题名（image/camera_info/debug/detection）、订阅/发布队列、TF 子坐标系、
  HSV 红/蓝/黄阈值、最小轮廓面积、长宽比、配对间距/角度/比例、亮度阈值、
  大/小装甲板尺寸、去重/瞬移/门限等。
- `config/tracker.yaml`（跟踪节点 `ft_tracker`）：
  提前量 T、Q/R、丢失阈值、看门狗周期、关联门限 `assoc_gate`、
  话题名（detection/track/marker）、队列长度。
- 两个 YAML 都由 `launch/test_v3.launch` 通过 `<rosparam command="load">` 加载；
  检测节点每 0.5s 重读一次，`rosparam set` 可实时生效。

## 动态调参（dynamic_reconfigure）

- 配置：`cfg/Vision.cfg`（红/蓝 H/S/V、最小面积、最大间距/角度差、亮度、长宽比、比例等）
- 启动后可用：
  ```bash
  rosrun rqt_reconfigure rqt_reconfigure
  # 或命令行
  rosrun dynamic_reconfigure dynparam set /vision_node_v3 blue_h_min 120
  ```
- 服务名：`/vision_node_v3/set_parameters`
- 回调会同步写回 `~` 参数，检测节点 0.5s 轮询也不会覆盖

## debug 图坐标轴

对每个有 PnP 位姿的目标，用 `cv::projectPoints` 把
`(0,0,0) / (0.03,0,0) / (0,0.03,0) / (0,0,0.03)` 投影回图像并连线：
X 红、Y 绿、Z 蓝（BGR 顺序），画在识别框旁；目标倾斜时坐标轴跟着转。

## 共面过滤（反光剔除）

- 开关：`plane_filter`（默认 true）、阈值比例 `plane_dist_ratio`（默认 1.0）。
- 灯条级：以最亮且可配对的第二根灯条确定“装甲平面”，离该平面垂直距离超过
  `灯条长度 × plane_dist_ratio` 的灯条判为反光并剔除，然后才进入配对。
- 目标级：主装甲板确定平面后，其它候选目标中心到该平面距离超过灯条长度的也剔除。

## PnP 测距标定（pnp_scale）

- 参数：`pnp_scale`（YAML，默认 0.57），按比例缩放 PnP 物体点，等效修正测距。
- 标定方法：用 `red_4_with_detection.bag` 的真值 `/ft/detection` 对比，扫
  0.55 / 0.57 / 0.58 / 0.60 / 0.627 / 0.65 / 0.70：
  - 0.57 时输出 z 均值 3.27m，与真值 3.27m 一致；
  - 3D 误差 0.194m、z 误差 0.175m，为最优；
  - 0.55 误差 0.271m，0.627 误差 0.402m。
- 说明：该系数吸收“物体点尺寸/四点取法”与真值的系统偏差。
- **注意：换镜头或改装甲板尺寸后，`pnp_scale` 需要重新扫一遍。**
- 重新标定步骤：
  1. 回放带真值的 bag（如 `red_4_with_detection.bag` 同类数据）；
  2. 节点输出重映射：`rosrun final_text detect_node_v3 _detection_topic:=/ft/detection_mine _pnp_scale:=X`；
  3. 与真值 `/ft/detection` 按时间戳对比，扫 `0.50 ~ 0.70`，取 3D/z 误差最小；
  4. 把最优值写入 `config/params.yaml` 的 `pnp_scale`。

## v3 识别流程（简版）

1. **图像接入**：订阅 compressed 图 + 缓存 `camera_info`，`header.stamp` 用图像时间戳。
2. **HSV 分色**：红色两段（0~10、170~179）+ 蓝色 + 黄色，形态学开运算去噪。
3. **高光守卫**：取 V 通道最亮点；若最亮点在画面底部（`guard_y_ratio`），判为地面反光，**屏蔽底部区域，取次高光的灯条**继续识别。
4. **轮廓 → 灯条**：面积、长宽比、框内平均亮度过滤，矩质心定中心，角度归一化。
5. **清洗**：断笔合并（共线短灯条拼接）+ 共面过滤（离装甲平面超过灯条长度的判为反光）。
6. **配对**：间距、高度差、角度差 + 透视修正比例 `dx/cosθ`，按比例区分大/小装甲板。
7. **四点与 PnP**：左灯条左边缘、右灯条右边缘的上下角点构成旋转四点，按板体长宽比拉高；`solvePnP`（IPPE→ITERATIVE），物体尺寸乘 `pnp_scale`（默认 0.57），并做 `z∈(0.3,15)` 过滤。
8. **输出**：`/ft/detection`（`ft_msgs/Detections` 数组，含 `id/is_red/is_large_armor/pose`，position=tvec、orientation=rvec 四元数）；主目标平滑后发 TF；debug 图画旋转框、角点和 `projectPoints` 三轴。

## 阈值调优（真值评估）

用 `red_4_with_detection.bag` 真值 `/ft/detection` 评估“precision = 匹配真值帧 / 有输出帧”：

| 配置 | 有输出帧 | FP | precision | 平均误差 |
|---|---|---|---|---|
| 原默认（min_area=20, bar_v_min=160） | 475 | 24 | 94.9% | 0.072m |
| bar_v_min=180 | 357 | 14 | 96.1% | 0.076m |
| min_area=50 | 336 | 2 | 99.4% | 0.062m |
| min_area=40 + bar_v_min=170 | 373 | 2 | 99.5% | 0.067m |
| **当前默认（40/170）** | 498 | 7 | **98.6%** | 0.056m |

说明：调整“过滤条件先后顺序”不改变结果（条件之间是逻辑与），只会影响速度；真正降低误判的是阈值。当前默认取 `min_area=40`、`bar_v_min=170`。
