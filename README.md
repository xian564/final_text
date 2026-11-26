# final_text

## 装甲板尺寸与 PnP 约定

- 尺寸参数（`config/params.yaml`，单位米），采用**定义一：一对灯条**：
  - 大装甲板：`armor_large_w=0.230`、`armor_large_h=0.060`（灯条中心距 230mm × 灯条长 60mm）
  - 小装甲板：`armor_small_w=0.135`、`armor_small_h=0.060`（135mm × 60mm）
- 大小板判定：`ratio = 透视修正间距 / 灯条长`，小板≈2.25、大板≈3.8，
  因此 `ratio_s_max=3.0`、`ratio_l_max=5.0`。
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
- 灯条级：枚举灯条对定直线，取内点最多的一条作为“装甲平面”，离该平面垂直距离超过
  `灯条长度 × plane_dist_ratio` 的灯条判为反光并剔除，然后才进入配对。
- 目标级：主装甲板确定平面后，其它候选目标中心到该平面距离超过灯条长度的也剔除。

## PnP 测距标定（pnp_scale）

- 参数：`pnp_scale`（YAML，默认 **1.0**），按比例缩放 PnP 物体点；用灯条尺寸建模后为 1.0。
- 标定只允许用**物理实测**：卷尺量相机到装甲板的真实距离，测 3~5 个距离点，
  调 `pnp_scale` 使输出 z 与卷尺读数一致（误差 <5% 即可）。
- **禁止读取任何真值话题/bag**（如 `*_with_detection.bag`、`/ft/detection` 真值）做标定或调参。
- 换镜头或改装甲板尺寸后，用同样的实测方法重新标定。

## v3 识别流程（简版）

1. **图像接入**：订阅 compressed 图 + 缓存 `camera_info`，`header.stamp` 用图像时间戳。
2. **HSV 分色**：红色两段（0~10、170~179）+ 蓝色 + 黄色，形态学开运算去噪。
3. **高光守卫**：取 V 通道最亮点；若最亮点在画面底部（`guard_y_ratio`），判为地面反光，**屏蔽底部区域，取次高光的灯条**继续识别。
4. **轮廓 → 灯条**：面积、长宽比、框内平均亮度过滤，矩质心定中心，角度归一化。
5. **清洗**：断笔合并（共线短灯条拼接）+ 共面过滤（离装甲平面超过灯条长度的判为反光）。
6. **配对**：间距、高度差、角度差 + 透视修正比例 `dx/cosθ`，按比例区分大/小装甲板。
7. **四点与 PnP**：左灯条左边缘、右灯条右边缘的上下角点构成旋转四点，按板体长宽比拉高；`solvePnP`（IPPE→ITERATIVE），物体尺寸乘 `pnp_scale`（默认 1.0），并做 `z∈(0.3,15)` 过滤。
8. **输出**：`/ft/detection`（`ft_msgs/Detections` 数组，含 `id/is_red/is_large_armor/pose`，position=tvec、orientation=rvec 四元数）；主目标平滑后发 TF；debug 图画旋转框、角点和 `projectPoints` 三轴。

## 阈值调优

- 只用普通 bag / 实车画面目测调参，不读真值。
- 关键阈值：`min_area`（默认 40）、`bar_v_min`（默认 170）、`ratio_s_max`（3.0）、
  `ratio_l_max`（5.0）、`bar_h_ratio_min`（0.6）、`pair_dy_ratio`（0.4）。
- 调参原则：先保证真灯条不被亮度/面积误杀，再用几何否决压误判。

## 几何否决（剔除斜四边形/假配对）

配对阶段新增三项硬否决（参数见 `config/params.yaml`）：
1. 两根灯条高度比：`min(h1,h2)/max(h1,h2) >= bar_h_ratio_min`（默认 0.6）；
2. 竖直错位：`|cy1-cy2| <= pair_dy_ratio × 灯条长`（默认 0.4）；
3. 四点矩形性：对边长度比 `>= quad_align_min`（默认 0.6），两条对角线相对差
   `<= quad_diag_tol`（默认 0.35）。

作用：灯条高度差过大、上下错位、被拉成平行四边形/斜长条的“假装甲”不再进入 PnP。
现场目测验证：斜四边形、上下错位、高度差过大的假配对明显减少。

> 阈值越严误判越低，但远距离/半遮挡会更多漏检。若现场发现漏检，优先把 `bar_h_ratio_min` 调到 0.45、`pair_dy_ratio` 调到 0.6；要更严就反过来。

## 神经网络识别模式（NN）

- 模型：`nn_model.zip/armor.onnx`（输入 1×3×480×640、输出 [1,21,6300]）。
- 模型是 FP16 + opset22，OpenCV DNN 无法加载（`Unsupported FLOAT16` / Conv 3 输入），
  需要先转换：
  ```bash
  python3 scripts/convert_onnx_fp32.py armor.onnx armor_allfp32.onnx
  ```
  转换内容：FP16 张量/ Cast 目标 → FP32，opset → 19，IR → 9。
- 推理后端：ONNX Runtime。当前环境没有 ORT C++ 开发包（头文件/libonnxruntime.so），
  因此 `scripts/nn_detector_node.py` 用 ORT Python 后端跑，接口层仍保留
  `include/final_text/detector_interface.hpp` 的 `IDetector / OnnxDetector`。
- 启动：
  ```bash
  roslaunch final_text test_nn.launch model_path:=/path/armor_allfp32.onnx
  ```
  输出 `/ft/detection_nn`（`ft_msgs/Detections`）和 `/ft/debug/image_nn`，
  跟踪节点通过 remap 订阅 NN 结果。
- 相邻帧锁定：多目标时选与上一帧中心最近的一块（`lock_gate_px`），避免左右板来回跳。

## 丢帧匀速外推（v3）

- 最近两次成功位姿估速度：`v = (t_k - t_{k-1}) / dt`（用平滑后的 tvec）。
- 漏检/遮挡时，若 `lost_frames < hold_frames`（默认 8）且速度 ≤ `hold_max_speed`（默认 8 m/s），
  按 `p = t_last + v * (now - t_last)` 发布预测 TF，画面标 `PRED`。
- 连续丢失超过 `hold_frames` 后清空状态，等待重新捕获。
- 只外推 TF，不伪造 `/ft/detection`（避免污染检测精度统计）。

## 共面过滤（共识平面）

- 灯条数 ≥3 时，枚举所有灯条对定直线，取**内点最多**的那条作为装甲平面。
- 离平面距离 > `灯条长度 × plane_dist_ratio`（默认 1.0）的灯条判为反光剔除。
- 至少保留 2 根才生效。

> v3.1 同步：加入“最多灯条共识定平面”共面过滤，并同步灯条尺寸
> （大 230×60、小 135×60）与大小板比例阈值（3.0 / 5.0）。

> v3.1 调整：去掉“共识平面”过滤，改为丢帧恒速外推（最近两帧 tvec 估速，
> `p = t_last + v·dt` 发预测 TF，最多 `hold_frames=8` 帧，速度上限 `hold_max_speed=8 m/s`）。

## v3_2 车辆几何推断

- 在 v3 基础上增加：装甲位姿 + 安装偏移 → 车体中心，发布 TF `car_v3_2`。
- 解析运动预测：`hc::sampleMotion`（匀速/匀速圆周）→ 车体预测位置 → `hc::project` 画 `PRED-CAR`。
- 接口：`include/final_text/motion_geometry.hpp`（`MotionParams / sampleMotion / cameraPose / project`）。
- 启动：
  ```bash
  roslaunch final_text test_v3_2.launch
  ```
- 参数：`mount_x/mount_y/mount_z/mount_yaw`（装甲到车体偏移）、
  `pred_v/pred_R/pred_omega/pred_t`、`car_tf_child`（默认 `car_v3_2`）。

## 环境与构建

```bash
# ROS Noetic + OpenCV 4.2
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

```bash
# 仿真/回放 bag
rosbag play <bag> --clock -l

# v3（完整版）：RViz + rqt_image_view + rqt_reconfigure 一键打开
roslaunch final_text test_v3.launch

# v3_2（车辆几何推断）
roslaunch final_text test_v3_2.launch

# v3.1（作业精简版，可选 NN 分支）
rosrun final_text detect_node_v3_1 _nn_enable:=true _nn_model_path:=/path/armor_allfp32.onnx
```

## 消息接口

- `/ft/detection`：`ft_msgs/Detections`
  - `header.frame_id`：相机光学系
  - `detections[i].id`：帧内编号
  - `is_red` / `is_large_armor`
  - `pose.position = tvec`（相机系，米）
  - `pose.orientation = rvec 四元数`
- `/ft/track`：`ft_msgs/Track`
  - `id`、`tracking`、`position`、`velocity`、`position_predict`
- TF：`camera_frame -> armor_v3`；v3_2 额外 `car_v3_2`

## 工况与鲁棒性

| 工况 | 处理 |
|---|---|
| 静止/匀速 | 20 帧锁定 + 匀速外推 |
| 近距离 | 近战重分割（低 S/V） |
| 地面反光 | 底部高光守卫 + 共面过滤 |
| 遮挡/漏检 | FCS 残影补位 + 丢帧外推 |

## 已知限制

- 编号（1–5）传统链路不识别；NN 模型可输出编号；
- 旋转/绕行预测只有匀速直线外推；
- C++ ONNX 后端受 OpenCV DNN 限制，当前 NN 由 ORT Python 节点运行；
- 单文件行数较大（v3），如需可拆分。
