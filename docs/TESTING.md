# 测试流程与说明

> 所有测试都**不读真值**：只用普通 bag、卷尺、画面目测。

## 1. 构建

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
预期：`EXIT 0`，`devel/lib/final_text/` 下有 `detect_node_v3_1`、`ft_tracker_node`。

## 2. 启动正稿

```bash
# 终端 1
rosbag play bag/red_4.bag --clock -l

# 终端 2
roslaunch final_text test_v3_1.launch
```
预期：RViz 打开；rqt_image_view 显示 `/ft/debug/image`；rqt_reconfigure 出现 `vision_node_v3`。

## 3. 基础验收检查

```bash
rostopic hz /ft/detection      # 期望 15–35 Hz
rostopic hz /ft/track          # 期望 40–80 Hz
rostopic hz /ft/debug/image    # 期望 15–35 Hz
rostopic hz /ft/debug/marker   # 期望 10–30 Hz
rosnode list | grep -E "vision|tracker"
```
- `/ft/detection` 有 `detections[]`，`pose.position.z` 在 0.3–15m；
- `/ft/track` 有 `tracking`、`position_predict`；
- RViz 可见 `armor_v3` TF 与预测 marker；
- 画面无 NaN/空消息。

## 4. 动态调参

```bash
rosrun dynamic_reconfigure dynparam set /vision_node_v3 blue_h_min 120
rosrun dynamic_reconfigure dynparam get /vision_node_v3 blue_h_min
```
预期：值变为 120，识别效果实时变化；恢复：
```bash
rosrun dynamic_reconfigure dynparam set /vision_node_v3 blue_h_min 85
```

## 5. 三工况 + 演示视频

| 工况 | bag 建议 | 录什么 |
|---|---|---|
| 静止/远距 | `red_4.bag` | 稳定框、z 基本不变 |
| 匀速平移 | `red_1&3(1).bag` | 框随目标移动、预测点在前 |
| 近距/遮挡 | `blue_guard.bag` | 近距稳定、短暂遮挡后恢复 |

录制步骤：
1. 起 `test_v3_1.launch`；
2. 录屏 40–60s/工况（rqt_image_view + RViz）；
3. 拼接 2–3 min，标注工况。

## 6. 附加题测试（bonus/）

```bash
roslaunch final_text test_v3.launch      # v3 完整版
roslaunch final_text test_v3_2.launch    # 车体几何
rosrun final_text detect_node_v3_split   # 分块版
```
- v3：看 FCS 虚线框、几何否决后的框稳定性；
- v3_2：`rostopic echo /tf | grep car_v3_2`；
- v3_split：`rostopic hz /ft/detection_v3split`。

## 7. 本次实测记录

| 项目 | 结果 |
|---|---|
| catkin_make | EXIT 0 |
| /ft/detection | 33.3 Hz |
| /ft/track | 58.5 Hz |
| /ft/debug/image | 27.8 Hz |
| /ft/debug/marker | 68.0 Hz |
| 动态调参 | blue_h_min 85 → 120 → 85，生效 |

## 8. 截图与对比流程（传统 vs NN）

### 8.1 同帧对比（推荐）
```bash
# 终端 1：数据
rosbag play bag/red_4.bag --clock -l

# 终端 2：传统链路（/ft/debug/image）
rosrun final_text detect_node_v3_1

# 终端 3：NN 链路（/ft/debug/image_nn）
rosrun final_text detect_node_nn _model_path:=/root/catkin_ws/bag/armor_allfp32.onnx

# 终端 4：抓 3 对同帧图
rosrun final_text capture_compare.py _out:=/tmp/compare _n:=3
```
输出：`/tmp/compare/compare_000.png ...`，左=传统、右=NN。

示例已保存：`docs/compare_demo.png`（左传统 ID:1/5 + 三轴，右 NN 框）。

### 8.2 单模式截图
```bash
roslaunch final_text test_vision.launch mode:=nn
rosrun final_text capture_debug.py _topic:=/ft/debug/image _out:=/tmp/nn _n:=5
```

### 8.3 RViz 位姿截图
1. 启动 `test_vision.launch`（默认带 RViz）；
2. RViz 里确认 `armor_v3` TF、三轴、预测 marker；
3. 用桌面截图（PrtSc）保存 `rviz_pose.png`。

### 8.4 漏检说明模板

| 项 | 内容 |
|---|---|
| 帧号/时间 | `rosbag info` 或画面时间 |
| 现象 | 无框 / 框错 / 编号错 |
| 原因分类 | 过曝、遮挡、远距（z>8m）、大角度、颜色偏 |
| 证据 | 该帧 z、框面积、亮度、NN score |
| 处理 | 调哪个参数 / 已知限制 |

## 9. 一条 launch 切换 tradition / nn

```bash
roslaunch final_text test_vision.launch mode:=tradition
roslaunch final_text test_vision.launch mode:=nn model_path:=/root/catkin_ws/bag/armor_allfp32.onnx
```
实测：
```
tradition: /ft/detection 30.5 Hz, number=0（无模板）
nn:        /ft/detection 15.2 Hz, number=5（NN 输出）
```


## 10. RViz 截图（无 ffmpeg 时）
```bash
export DISPLAY=:0
xwininfo -root -tree | grep -i rviz   # 找窗口 id
import -window 0xc0000c /tmp/rviz_pose.png
```
注意：`import -window root` 在部分 X 服务上会报 `Resource temporarily unavailable`，要指定 RViz 窗口 id。
