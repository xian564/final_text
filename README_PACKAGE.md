# final_text 视觉功能块包

默认识别链路为传统 OpenCV / ROS1 Noetic；神经网络（onnx）为可选对照分支，不作为默认依赖。

## 功能块

| 文件 | 职责 |
|---|---|
| `include/final_text/vision_preprocess.hpp` | CLAHE(LAB) + 自适应 Gamma + 饱和度拉伸；颜色/高光/Canny 融合掩膜 |
| `include/final_text/armor_detector.hpp` | 平A：动态 ROI、矩质心、透视比例、极值厂字框、PnP、Alpha-Beta |
| `include/final_text/rune_detector.hpp` | 大招：高亮轮廓、最小二乘圆拟合、PLL 相位预测 |
| `src/detection_node_v4.cpp` | 模块化节点：收图/调度/发布 |
| `src/detection_node_v3.cpp` | 稳定单文件版本（默认） |
| `config/vision_params.yaml` | 参数示例 |
| `docs/ARCHITECTURE.md` | 架构与数据流 |
| `docs/attempts/*.md` | 调试记录 |

## 接口（视觉 → 火控）

- `/ft/armor_pose` (geometry_msgs/PoseStamped)
- `/ft/armor_velocity` (geometry_msgs/TwistStamped)
- `/ft/debug/image_v4` (sensor_msgs/Image)

视觉只给：时间戳、3D 位置、线速度、四元数；**不算**弹道/飞行时间。

## 编译

```bash
cd ~/catkin_ws
cp -r final_text_package/* src/final_text/   # 按需合并
catkin_make
```

## 运行

```bash
roscore
rosbag play bag/red_4.bag --clock -l
rosrun final_text detect_node_v3       # 稳定版
# 或
rosrun final_text detect_node_v4       # 模块化版
rostopic pub /vision_mode std_msgs/Int8 "data: 1"   # 切大招
```

## 已知限制

- v4 位姿偶发跳变（Alpha-Beta 参数未精调）；
- 蓝包/红包曝光差异大，阈值需按场地标定；
- 白场兜底默认关闭（`white_enable=false`，极端过曝时开）。
