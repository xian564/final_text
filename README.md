# final_text — 装甲板识别与跟踪（ROS Noetic）

## 1. 项目结构

```
final_text/
├── src/
│   ├── detection_node_v3_1.cpp   # 正稿：识别 + PnP + 发布
│   └── ft_tracker_node.cpp       # 正稿：Kalman 跟踪 + 提前量
├── bonus/                        # 附加题/实验版本（见 bonus/README.md）
├── include/final_text/           # 工具头文件（NN 接口、几何、拆分实验）
├── config/                       # params.yaml / tracker.yaml
├── cfg/Vision.cfg                # dynamic_reconfigure（22 项）
├── launch/test_v3_1.launch       # 正稿一键启动
├── ft_msgs_pkg/                  # Detection / Detections / Track
├── rviz/armor_v3.rviz
└── docs/                         # 测试流程、记录、教程
```

## 2. 环境与构建

- Ubuntu 20.04 + ROS Noetic + OpenCV 4.2
- `ft_msgs` 消息包需与 `final_text` 同工作空间：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 3. 运行（正稿）

```bash
# 终端 1：回放数据（任意普通 bag）
rosbag play bag/red_4.bag --clock -l

# 终端 2：一键启动 检测 + 跟踪 + RViz + 两个 rqt
roslaunch final_text test_v3_1.launch
```

- 图像：`/hk_camera/image_raw/compressed`
- 检测：`/ft/detection`（`ft_msgs/Detections`）
- 跟踪：`/ft/track`（`ft_msgs/Track`）
- 调试图：`/ft/debug/image`
- 预测 marker：`/ft/debug/marker`
- TF：`camera_frame -> armor_v3`

## 4. 正稿识别思路（v3.1）

1. **图像接入**：订阅 compressed 图与 `camera_info`；`header.stamp` 用图像时间戳，内参/畸变从 `camera_info` 读。
2. **HSV 分色**：红色两段（H 0–10、170–179）+ 蓝色 + 黄色，开运算去噪。
3. **灯条提取**：面积、长宽比、框内平均亮度筛选；矩质心定中心，角度归一化。
4. **断笔合并 + 共面**：合并共线短灯条。
5. **配对**：水平间距、高度差、角度差；`ratio = 透视修正间距 / 灯条长` 区分大/小板。
6. **四点与 PnP**：左灯条左边缘 + 右灯条右边缘的上下角点，按板体长宽比拉高；`solvePnP`（IPPE→ITERATIVE），灯条尺寸 230×60 / 135×60 mm，`z` 过滤。
7. **目标选择与平滑**：相邻 20 帧锁定同一目标（位移 ≤50px、比例 ±10%），位置/yaw 滑动平滑，`z` 中值滤波。
8. **发布**：`/ft/detection` 数组（id、颜色、大小、tvec、rvec 四元数）、TF `armor_v3`。

## 5. 参数与动态调参

- `config/params.yaml`：识别阈值、灯条/装甲尺寸、配对/锁定、话题与队列。
- `config/tracker.yaml`：Kalman `Q/R`、提前量 `T`、关联门限、丢失阈值、marker 话题。
- `cfg/Vision.cfg`：22 项识别参数支持 `rqt_reconfigure` 在线调。

```bash
rosrun rqt_reconfigure rqt_reconfigure
rosrun dynamic_reconfigure dynparam set /vision_node_v3 blue_h_min 120
```

## 6. 消息接口

- `/ft/detection`：`ft_msgs/Detections`
  - `header.stamp` = 图像时间戳，`frame_id` = 相机 frame
  - `detections[i].id`：帧内唯一
  - `number`（0=未知，1–5；NN 分支输出）
  - `is_red` / `is_large_armor`
  - `pose.position` = tvec（米），`pose.orientation` = rvec 四元数
- `/ft/track`：`ft_msgs/Track`
  - `id`、`tracking`（观测命中 true / 纯预测 false）
  - `position`、`velocity`、`position_predict`
- 所有消息 `header.stamp` 用图像时间戳；纯预测沿用最近一帧图像时间戳。

## 7. 跟踪节点

- `cv::KalmanFilter`，状态 `[x,y,z,vx,vy,vz]`，观测 `[x,y,z]`。
- `q_pos=0.1`、`q_vel=0.5`、`r_pos=0.05`；位置噪声小、速度噪声大。
- 提前量 `T=0.1s`，`position_predict = position + velocity*T`，限幅 `0.6m`。
- 关联门限 `assoc_gate=0.5m`：覆盖单帧位移、避免换板误关联。
- 丢检：50ms 看门狗 `predict`，发 `tracking=false`；连续丢失 >0.5s 删除轨迹并重捕。

## 8. 作业要求对照（基础部分）

| 要求 | 实现 |
|---|---|
| 不用 `cv::imshow/waitKey` | ✅ 调试图发布 `/ft/debug/image` |
| 阈值/尺寸/滤波参数不硬编码 | ✅ YAML + 22 项 dynamic_reconfigure |
| 内参/畸变从 `camera_info` 读 | ✅ |
| 消息 `header.stamp` 用图像时间戳 | ✅（纯预测用最近一帧） |
| 四点定义/坐标系写清 | ✅ 本节第 4 节 |
| 消息定义在仓库 | ✅ `ft_msgs_pkg/` |
| 一条 launch 启动检测+跟踪+可视化 | ✅ `test_v3_1.launch` |
| 跟踪语义、丢检、提前量、marker | ✅ |
| 2–3 分钟演示视频 | 见 `docs/TESTING.md` 录制流程 |

## 9. 测试

完整流程与记录见 [`docs/TESTING.md`](docs/TESTING.md)。

## 10. 已知限制

- 正稿 v3.1 为**核心链路**：不含 FCS 残影、几何否决、共识平面、丢帧外推；这些在 `bonus/detection_node_v3.cpp`。
- 神经网络：已提供 **ONNX Runtime C++ 后端**（`include/final_text/detector_interface.hpp`），
  使用 `third_party/ort` 的头文件/库，可加载转换后的 `armor_allfp32.onnx`；
  启动 `roslaunch final_text test_nn_cpp.launch`，参数 `nn_enable` 可切换传统/NN。
- 传统链路不识别装甲板编号；NN 分支（`number` 字段）可输出 1–5。
- 不估计车体旋转（旋转模型为进阶项，未实现）。

## 11. 附加题

见 [`bonus/README.md`](bonus/README.md)：v3 完整功能、v3_2 车体几何、v3_split 分块、v4 早期骨架。

## 12. AI 使用

见 [`ai_usage.md`](ai_usage.md)。

## 旋转目标建模（进阶，`ft_rotation_tracker_node`）

- 车体中心：对最近 `fit_window` 秒的装甲中心做圆拟合（Kasa），得到中心与半径；不足 8 点沿用上次结果。
- 相位/角速度：`phase = atan2(z-cz, x-cx)`，0.5s 窗口内差分并低通；正=左转、负=右转。
- 预测：`pred = body_center + R·[cos(θ+ωT), sin(θ+ωT)]`，y 用直线速度外推。
- 选择：首次取最靠近相机轴的检测；之后取离预测最近且在 `assoc_gate` 内的检测。
- 断帧：`lost_time_thresh` 内继续预测，超过则断开 ID 重新捕获；遮挡/丢失 `valid=false`。
- 车体 ID：订阅 `/ft/body_id`（`std_msgs/Int32`），留作神经网络编号接入。
- 输出：`/ft/track_rot`（`position` 当前装甲、`position_predict` 预测、`body_center` 车体中心、`valid`）；
  `/ft/debug/marker_rot` 画车体中心（蓝）、当前装甲（绿）、预测装甲（红）。
- 启动：`roslaunch final_text test_rotation.launch`
