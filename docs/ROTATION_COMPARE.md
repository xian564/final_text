# 匀速 vs 旋转模型：预测误差对比

## 方法
- 数据：`red_4.bag` 25s；
- 同时运行：
  - `ft_tracker_node`（匀速 Kalman）→ `/ft/track`
  - `ft_rotation_tracker_node`（圆周模型）→ `/ft/track_rot`
- 评估：
  1. 取 `tracking=true` 的 Track；
  2. 找其后 0.25s 内最近一帧 `/ft/detection`；
  3. 用 Track 的当前位置匹配同一块板，得到“下一帧实际位置”；
  4. 误差 = `||position_predict - 实际位置||`。
- 不读真值，用检测自身作为参考。

## 结果
| 跟踪器 | 样本数 | 平均误差 | 中位误差 |
|---|---|---|---|
| 匀速 `/ft/track` | 693 | 0.205 m | **0.072 m** |
| 旋转 `/ft/track_rot` | 524 | 0.406 m | 0.429 m |

## 解释
- `red_4` 这段以直线/小幅运动为主，圆周模型没有优势；
- 旋转模型要圆拟合 + 相位估计，直线段会引入额外噪声；
- 旋转模型的收益应在“明显绕圈”片段验证；
- 正对、遮挡、R 突变时旋转模型会失效（见 README 假设）。

## 复现
```bash
rosbag play bag/red_4.bag --clock -l
rosrun final_text detect_node_v3_1
rosrun final_text ft_tracker_node
rosrun final_text ft_rotation_tracker_node
rosbag record -O /tmp/rot.bag --duration=25 /ft/detection /ft/track /ft/track_rot
rosrun final_text eval_prediction.py /tmp/rot.bag /ft/track
rosrun final_text eval_prediction.py /tmp/rot.bag /ft/track_rot
```

## 已知问题
1. 没有“明显旋转”的 bag，结论只代表直线段；
2. 误差含预测提前量 T 的固有偏差；
3. 旋转模型在直线段应默认退回匀速（可加 `rotation_enable` 或运动判定）。
