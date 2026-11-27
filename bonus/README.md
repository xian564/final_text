# bonus/（附加题 / 实验版本）

这里放非“正稿”的代码，便于答辩时展示扩展与实验：

| 文件 | 说明 |
|---|---|
| `detection_node_v3.cpp` | 完整功能版（FCS、几何否决、共面、锁定、丢帧外推）；作为附加题/扩展展示 |
| `detection_node_v3_2.cpp` | 车辆几何推断实验（装甲位姿+安装偏移→车体 TF） |
| `detection_node_v3_split.cpp` | Knee& 分块重构实验（7 个 Block） |
| `detection_node_v4.cpp` | 早期模块化骨架（未完成） |

正稿提交版本：`src/detection_node_v3_1.cpp` + `src/ft_tracker_node.cpp`。

启动正稿：
```bash
roslaunch final_text test_v3_1.launch
```
