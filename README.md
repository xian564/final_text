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
