# bonus/ — 附加题与实验版本

> 正稿是 `src/detection_node_v3_1.cpp` + `src/ft_tracker_node.cpp`（核心链路）。
> 本目录放“做完了但没放进正稿”的扩展与实验，供答辩展示。

## 1. 想做什么 / 为什么

- **v3 完整版**：在核心链路上增加 FCS 残影补位、几何否决、共识平面、相邻帧锁定、丢帧匀速外推，
  解决反光、斜四边形、遮挡丢帧这些实际 badcase。
- **v3_2 车体几何**：由装甲位姿 + 安装偏移推车体中心，输出 `car_v3_2` TF 与投影点。
- **v3_split 分块**：把 1600 行单函数按数据流拆成 `Knee` + 7 个 `xxxBlock(Knee&)`，便于讲解。
- **v4**：早期模块化骨架（未完成）。

## 2. 做到哪一步

| 版本 | 内容 | 状态 |
|---|---|---|
| `detection_node_v3.cpp` | FCS、几何否决、共识平面、锁定、外推 | 可编译、bag 实测有检测 |
| `detection_node_v3_2.cpp` | 车体 TF + `CAR`/`PRED-CAR` 投影 | 可编译、bag 实测有 TF |
| `detection_node_v3_split.cpp` | Knee + 7 Block | 可编译、24Hz 检测 |
| `detection_node_v4.cpp` | 模块化骨架 | 未完成 |

## 3. 运行与测试

```bash
# 需要先 catkin_make
rosbag play bag/red_4.bag --clock -l

# v3 完整版（检测 + 跟踪）
roslaunch final_text test_v3.launch

# v3_2 车体几何
roslaunch final_text test_v3_2.launch

# v3_split（单独节点，话题带 _split 后缀）
rosrun final_text detect_node_v3_split
```

测试步骤与记录见 [`docs/TESTING.md`](../docs/TESTING.md) 第 6 节。

## 4. AI 帮了什么 / 踩了什么坑

- AI 用于：报错定位、参数含义解释、文档草稿、拆分脚本；
- 坑：
  - 按注释标记切 `if/else` 会把 `else` 切断（`expected unqualified-id before 'else'`）；
  - 变量全量替换会把声明改成 `cv::Mat knee.xxx`，要再删声明；
  - 3D 预测锁定实测跳变更严重，已回退；
  - 车辆 ID 跟踪在“两车靠近”时仍会切换，已回退，只保留几何推断。

## 5. 对话与启发链接

- 对话《如何判断是否整包生成的代码》（本地会话记录，`~/.dsh/...`）：
  讨论“整包生成 vs 逐步修改”的判定标准，本目录的版本划分受其启发。
- 对话导出：
  - [`docs/ai_chats/01_OpenCV轮廓与RoboMaster自瞄.md`](../docs/ai_chats/01_OpenCV轮廓与RoboMaster自瞄.md)
  - [`docs/ai_chats/02_ROS_C++视觉代码与Java入门.md`](../docs/ai_chats/02_ROS_C++视觉代码与Java入门.md)
  - [`docs/ai_chats/04_ROS_C++视觉模块化与PnP.md`](../docs/ai_chats/04_ROS_C++视觉模块化与PnP.md)
- 合规自查（RE 调讨论）：
  [`docs/ai_compliance_scan.md`](../docs/ai_compliance_scan.md)
- 可选功能教程：
  [`docs/tutorial_optional.md`](../docs/tutorial_optional.md)

## 6. 加分点对应

- 完整实现（FCS/几何/共面/外推）+ 正稿前后对比：对应“完整实现 + 效果明显提升”档；
- 分块重构（Knee + Block）：对应“有思路推演/架构图”档；
- 车体几何推断：对应“有想法 + 说清楚为什么”档。
