# final_text 视觉工程 · 功能块说明

## 目录结构

```
final_text/
├── include/final_text/
│   ├── vision_preprocess.hpp   # 预处理：CLAHE(LAB)+Gamma+饱和度+融合掩膜
│   ├── armor_detector.hpp      # 平A：ROI/质心/透视校验/极值框/PnP/Alpha-Beta
│   └── rune_detector.hpp       # 大招：高亮轮廓/最小二乘圆/PLL
├── src/
│   ├── detection_node_v4.cpp   # 模块化节点（平A + 大招 + 模式切换）
│   └── detection_node_v3.cpp   # 稳定单文件版本（默认）
├── launch/  test_v2.launch  test_v3.launch  test_v4.launch
├── rviz/    armor_v2.rviz
└── docs/    调试记录（attempts/）
```

## 数据流（v4）

```
相机 compressed → cv_bridge → Preprocessor::enhance
   → Preprocessor::buildMask（颜色 | 高光 | Canny）
   → 模式判定（/vision_mode: 0=平A, 1=大招）
       ├─ NormalArmorDetector::detect → Target（3D + yaw）
       └─ EnergyRuneDetector::detect  → 预测 3D
   → /ft/armor_pose (PoseStamped)
   → /ft/armor_velocity (TwistStamped)
   → /ft/debug/image_v4 (调试图)
```

## 视觉-火控交接边界

视觉只输出：曝光时间戳、3D 位置、线速度、姿态四元数。
**不计算**弹道下坠、飞行时间、电机延迟。

## 编译与运行

```bash
cd ~/catkin_ws && catkin_make
rosrun final_text detect_node_v3          # 稳定版
rosrun final_text detect_node_v4          # 模块化版（平A/大招）
# 模式切换：
rostopic pub /vision_mode std_msgs/Int8 "data: 1"   # 1=大招
```

## v3 / v4 差异

| 项 | v3 | v4 |
|---|---|---|
| 结构 | 单文件 ~1100 行 | 头文件拆分 + 节点 |
| 预处理 | HSV + 高光 + 亮度 | CLAHE/Gamma + Canny 融合 |
| 输出 | TF `armor_v3` | PoseStamped + TwistStamped |
| 平滑 | 5 帧加权 + z 中值 | Alpha-Beta |
| 模式 | 固定平A | `/vision_mode` 切换 |
| 状态 | 默认、验证充分 | 可选、功能块化 |
