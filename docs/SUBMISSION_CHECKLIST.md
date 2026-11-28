# 提交清单（对照 PDF）

## 基础（70）
| 要求 | 状态 | 证据 |
|---|---|---|
| 不用 imshow/waitKey | ✅ | grep 0；调试图 `/ft/debug/image` |
| 阈值/尺寸/T 不硬编码 | ✅ | YAML + cfg 27 项 |
| 内参/畸变读 camera_info | ✅ | `cameraInfoCallback` |
| 消息 stamp 用图像时间 | ✅ | 检测/TF/跟踪；纯预测用最近图像时间 |
| 四点/坐标系/单位写清 | ✅ | README |
| 消息定义进仓库 | ✅ | `ft_msgs_pkg/` |
| 一条 launch | ✅ | `test_vision.launch mode:=...` |
| 跟踪 Kalman/丢检/T/marker | ✅ | `ft_tracker_node` |
| README 滤波/Q-R/门限 | ✅ | README 第 7 节 |
| 2–3 分钟视频 | ✅ | `docs/demo.mp4`（2:03） |
| 三工况记录 | ✅ | `scene_static/velocity/near.jpg` |

## 进阶（选做 30）
### 旋转目标建模（10）
| 要求 | 状态 |
|---|---|
| 圆周模型/R 可配 | ✅ `ft_rotation_tracker_node` |
| 车体中心/相位/角速度 | ✅ |
| dt 图像时间、跨 2π | ✅ |
| 多板选择/关联 | ✅ 预测最近 + 门限 |
| T 可视化对比/失效解释 | ✅ `ROTATION_COMPARE.md`（直线段旋转模型更差，已解释） |

### 神经网络（10）
| 要求 | 状态 |
|---|---|
| C++ 后端 | ✅ ORT 1.31.1 |
| letterbox/RGB/归一化/还原 | ✅ |
| 解码颜色/编号/角点 | ✅（编号索引即编号） |
| 稳定角点顺序 | ✅ `orderCorners` |
| 置信度/NMS | ✅ 0.5/0.5/0.45 |
| 接入 PnP/跟踪/发布 | ✅ `/ft/detection_nn` |
| 参数切换传统/NN | ✅ `mode:=tradition|nn` |
| Bag 展示四角点/位姿 | ✅ `nn_pose.png`、`rviz_pose.png` |
| 传统 vs NN 同帧 | ✅ `nn_compare.png` |
| 平均耗时 | ✅ 64–81 ms |
| 训练/练习数据 | ✅ `practice/` + `scripts/train_practice.py`（未实际训练：环境无 torch） |

## 提交
- 代码/tag：待推（本地 `main` 已含全部改动）
- 视频：`docs/demo.mp4`
- AI 记录：`ai_usage.md`
- 手册：`docs/EMERGENCY_MANUAL.md`

## 仍缺/风险
1. 旋转对比只在直线 bag 上，缺“明显旋转”片段；
2. NN 编号 5 类不使用，靠置 0；
3. 训练脚本未在本机跑通（无 ultralytics/torch）；
4. RViz 场景为空，仅 TF 可见。
