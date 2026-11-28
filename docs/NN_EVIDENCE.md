# 神经网络任务 · 完整操作与证据

## 1. 环境/版本
| 项 | 值 |
|---|---|
| 后端 | ONNX Runtime C++ 1.31.1 |
| 模型 | `armor_allfp32.onnx`（fp32, opset19） |
| 输入 | `1×3×480×640` |
| 输出 | `1×21×6300` |
| 设备 | CPU，单线程 |

## 2. 构建
```bash
cd ~/catkin_ws && catkin_make
# EXIT: 0
```

## 3. 模型加载
```bash
rosrun final_text detect_node_nn _model_path:=/root/catkin_ws/bag/armor_allfp32.onnx
```
日志：
```
NN ready=1
```

独立 ORT 测试：
```
LOADED inputs=1 outputs=1
```

## 4. 样例图（先跑通）
```
ready=1
img=1440x1080
ok=1 n=2
color=1 num=4 score=0.927529
color=1 num=2 score=0.743388
```

## 5. Bag 实测
```bash
rosbag play bag/red_4.bag --clock -l
rosrun final_text detect_node_nn _model_path:=/root/catkin_ws/bag/armor_allfp32.onnx
rosrun final_text ft_tracker_node _detection_topic:=/ft/detection_nn
```
输出：
```
NN armors=1 infer_avg=64.7 ms
NN armors=2 infer_avg=65.7 ms
/ft/detection_nn: 13.5 Hz
/ft/track:        43.4 Hz
```
检测字段：
```
id: 1
number: 5
x: 0.106  y: -0.299  z: 4.50
```

## 6. 截图证据
- 四角点/编号：`docs/nn_demo.png`
- 传统 vs NN 同帧：`docs/nn_compare.png`（左传统、右 NN）
- 位姿三轴+距离：`docs/nn_pose.png`
- RViz TF：`docs/rviz_pose.png`
- 传统/NN 对比另一张：`docs/compare_demo.png`

## 7. 模式切换
```bash
roslaunch final_text test_vision.launch mode:=nn
# /ft/detection 15.2 Hz, number=5

roslaunch final_text test_vision.launch mode:=tradition
# /ft/detection 30.5 Hz, number=0
```

## 8. 对作业要求逐条核对
| 要求 | 状态 | 证据/说明 |
|---|---|---|
| C++ 后端（ORT/OpenVINO） | ✅ | ORT 1.31.1 |
| letterbox/补边/RGB/归一化 | ✅ | `detector_interface.hpp` |
| 保存 scale/pad 并还原 | ✅ | 每帧局部变量 |
| 解码颜色/编号/四角点 | ✅ | 21 行解码 |
| 角点还原原图、稳定顺序 | ✅ | `orderCorners()` 几何重排 TL/TR/BR/BL |
| 置信度筛选 + 去重 | ✅ | 0.5/0.5/0.45 |
| 接入 PnP/跟踪、发布 | ✅ | `/ft/detection_nn`、tracker |
| 参数切换传统/NN | ✅ | `mode:=tradition|nn` |
| Bag 展示原图/四角点/位姿 | ✅ | `nn_pose.png`（四角点+三轴+tvec）；`rviz_pose.png`（TF） |
| 后端/预处理/张量/阈值说明 | ✅ | `docs/nn_tutorial.md` |
| 传统 vs NN 同帧 | ✅ | `docs/nn_compare.png` |
| 平均单帧耗时 | ✅ | 64–81 ms |

## 9. 已知问题
1. 编号仍有 5：已加“连续 3 帧不一致才切换”的平滑，但模型原始输出就是 5；
2. 传统 vs NN 对比图中，传统该帧漏检、NN 多检 1 个；
3. 四角点顺序未做几何校验，倾斜大时可能错序；
4. RViz 视角只看到 TF 轴，场景为空（未加 camera 显示）；
5. 未用 OpenVINO（作业允许 ORT）。

## 10. 复现命令汇总
```bash
# 构建
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 数据
rosbag play bag/red_4.bag --clock -l

# NN + 跟踪
rosrun final_text detect_node_nn _model_path:=/root/catkin_ws/bag/armor_allfp32.onnx
rosrun final_text ft_tracker_node _detection_topic:=/ft/detection_nn

# 截图
python3 - <<'PY'
import rospy,cv_bridge,cv2
from sensor_msgs.msg import Image
rospy.init_node('shot'); b=cv_bridge.CvBridge()
m=rospy.wait_for_message('/ft/debug/image_nn',Image,timeout=5)
cv2.imwrite('/tmp/nn.png',b.imgmsg_to_cv2(m,'bgr8'))
PY
```
