# 旋转片段定位与预测误差对比

## 旋转片段在哪个包
**`red_4.bag`**：检测 yaw 范围约 **123.7°**（-38.4° ~ +85.3°），
且该包存在“检测在两块板之间跳”的现象。

## 对比设置
- 同一段 `red_4.bag` 25s；
- 同时跑 `ft_tracker`（匀速 Kalman）与 `ft_rotation_tracker`（圆周 + 直线回退）；
- 误差：`position_predict` vs 下一帧实际检测（用当前位置匹配同一块板）；
- 按 yaw 角速度分两档：`>30°/s`（旋转）与慢速。

## 结果（全段）
| 跟踪器 | 样本 | 平均误差 | 中位误差 |
|---|---|---|---|
| 匀速 `/ft/track` | 641 | **0.197 m** | **0.065 m** |
| 旋转 `/ft/track_rot` | 520 | 0.277 m | 0.198 m |

## 结果（按角速度分档）
| 跟踪器 | 旋转 >30°/s | 慢速 |
|---|---|---|
| 匀速 | n=406, 0.092 m / 0.053 m | n=235, 0.040 m / 0.025 m |
| 旋转 | n=304, 0.245 m / 0.210 m | n=216, 0.147 m / 0.119 m |

## 解释
- `red_4` 确实是旋转片段，但**检测在换板/两车之间跳**；
- 圆拟合需要连续、同一块板的中心点，跳变会污染圆心和 R；
- 因此圆周预测反而更差，匀速模型靠“惯性”更稳；
- 旋转模型要生效，前提是检测/关联稳定（锁定或 tighter gate）。

## 已做的改进
- 增加 `omega_min=0.2`：转速小时自动退回直线预测；
- 旋转关联门限从 1.0 收紧到 0.5；
- 结果：旋转中位误差从 0.429 → 0.198 m，但仍未超过匀速。

## 结论
- 旋转片段 = `red_4.bag`；
- 在该包上，**匀速仍优于圆周**，原因是检测跳变，不是圆周公式错；
- 要拿旋转模型的收益，需先解决换板/多车关联。


---

# 附：用神经网络检测再测一版

## 设置
- `detect_node_nn`（ORT）→ `/ft/detection_nn`
- 匀速 tracker 与旋转 tracker 都吃 NN 检测
- 同一段 `red_4.bag` 25s

## 全段结果
| 跟踪器 | 样本 | 平均误差 | 中位误差 |
|---|---|---|---|
| 匀速 | 404 | **0.069 m** | **0.051 m** |
| 旋转 | 262 | 0.100 m | 0.075 m |

NN 检测比传统稳（传统匀速中位 0.065 m → NN 0.051 m）。

## 按角速度分档
| 跟踪器 | >30°/s | 慢速 |
|---|---|---|
| 匀速 | n=22, 0.166 m / 0.117 m | n=382, 0.059 m / 0.044 m |
| 旋转 | n=18, **0.141 m** / 0.136 m | n=244, 0.094 m / 0.069 m |

**在 >30°/s 的旋转段，旋转模型平均误差更低（0.141 vs 0.166 m）**；
慢速段仍应退回匀速。

## 本版改动
1. 用检测四元数直接估计 `yaw_rate`，预测时把“当前装甲相对车体中心的向量”按 `yaw_rate*T` 旋转；
2. `omega_min=0.2`：低速退回匀速；
3. `assoc_gate` 0.5 → 0.8（NN 检测更稳，放宽关联）。


---

# 修正后再测（Kasa 右端 + 滞回 + onLost 统一）

- Kasa 第三行右端改为 `Σ(x²z+z³)`；
- `omega_min` 加 0.75 滞回，避免阈值附近来回切；
- `onLost` 与正常预测统一用 `yaw_rate` + 当前偏移旋转。

NN 检测、`red_4.bag`：
| 跟踪器 | 样本 | 平均误差 | 中位误差 |
|---|---|---|---|
| 匀速 | 381 | 0.071 m | 0.048 m |
| 旋转 | 246 | 0.091 m | 0.080 m |

旋转平均误差从 0.100 → 0.091 m；全段仍匀速略优。


---

# 参数收敛版（omega_min=0.5, gate=0.5, 速度限幅）

NN 检测、red_4：
| 跟踪器 | 样本 | 平均误差 | 中位误差 |
|---|---|---|---|
| 匀速 | 393 | 0.073 m | 0.052 m |
| 旋转 | 247 | 0.095 m | 0.079 m |

结论不变：全段匀速略优；旋转收益集中在高转速段。

---

# 本机实测流程（2026-11-28）

## 1. 选旋转段
```bash
rosbag play bag/red_4.bag --clock -l
rosrun final_text detect_node_v3_1
rosbag record -O /tmp/rot_det.bag --duration=25 /ft/detection
python3 /root/catkin_ws/src/final_text/scripts/check_yaw.py /tmp/rot_det.bag
```
结果：
```
n = 808
min = -42.16 deg
max =  49.32 deg
range =  91.48 deg
```
→ 确认是旋转段。

## 2. 同时跑两个跟踪器并录轨迹
```bash
rosrun final_text ft_tracker_node
rosrun final_text ft_rotation_tracker_node
rosbag record -O /tmp/rot_eval.bag --duration=25 \
  /ft/detection /ft/track /ft/track_rot
```

## 3. 计算预测误差
```bash
python3 /root/catkin_ws/src/final_text/scripts/eval_prediction.py \
  /tmp/rot_eval.bag /ft/track
python3 /root/catkin_ws/src/final_text/scripts/eval_prediction.py \
  /tmp/rot_eval.bag /ft/track_rot
```

## 4. 本机结果
```
/ft/track      n=752 mean=0.077 m median=0.027 m
/ft/track_rot  n=787 mean=0.101 m median=0.028 m
```

## 5. 怎么看
- **中位几乎一样**（0.027 vs 0.028）：大多数帧在低速/直线段，两种模型差不多；
- **平均旋转更差**（0.101 vs 0.077）：少数高转速/跳变帧把均值拉高；
- 旋转样本更多（787 vs 752），说明当前 `assoc_gate=0.5` 下关联更连续；
- 要证明旋转收益，必须**按 yaw 角速度分档**再看：
  ```python
  # 伪代码
  if yaw_rate > 30 deg/s: 统计旋转 vs 匀速
  ```
- 结论：这段上旋转模型**没有全段优势**，收益只在部分高转速帧。

## 6. 复现注意
- `eval_prediction.py` 默认检测话题 `/ft/detection`；
  NN 时加第三个参数 `/ft/detection_nn`；
- 必须 `source devel/setup.bash`，否则 `ft_msgs` 导入失败；
- 同一段 bag、同一 `T` 才能对比。
