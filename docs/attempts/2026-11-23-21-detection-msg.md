# 21 · /ft/detection 消息 + YAML/launch + /ft/track 预测 marker

## 1) 新增 ft_msgs 包
- `ft_msgs_pkg/msg/Detection.msg`
- 构建：拷贝为 `~/catkin_ws/src/ft_msgs`（与 final_text 同级），`catkin_make`

## 2) v3 发布
- `/ft/detection` (`ft_msgs/Detection`)：每帧成功检测发布
  - `header.stamp`、`id`、`is_red`、`is_large_armor`
  - `pose.position` = 平滑后 tvec（与 TF 同一坐标系）
  - `pose.orientation` = 平滑后 yaw 的四元数
- `/ft/track` (`visualization_msgs/Marker`)
  - id=0：预测点球体（实际点 + 最小二乘速度 × 0.15s）
  - id=1：实际点 → 预测点 的黄色连线
- 速度：最近 6 帧最小二乘，限幅 5 m/s

## 3) YAML + launch
- `config/params.yaml`：42 个参数全部外置
- `launch/test_v3.launch`：`<rosparam command="load" .../>` 加载，同时启动
  static TF（camera2_optical_frame → hk_camera_ca）与 RViz

## 4) 验证（red_4.bag）
- `rostopic info /ft/detection` → `ft_msgs/Detection`
- 参数确实来自 YAML：`rosparam get /vision_node_v3/blue_h_min` → 85
- 预测领先量：同帧 `|pred - det|` 平均 **0.182 m**，最大 0.750 m（限幅）
- RViz：Displays 中出现 `Track`，可见绿色球 + 黄色连线
