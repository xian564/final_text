# 神经网络识别教程（ORT C++ 后端）

> 目标：按模型协议完成预处理 → 解码 → 还原 → 去重 → PnP/跟踪 → `/ft/detection`，
> 并能在 Bag 上展示、对比传统/NN、记录耗时。

## 0. 后端与版本

| 项 | 值 |
|---|---|
| 推理后端 | ONNX Runtime C++ |
| 版本 | 1.31.1（头文件 + `libonnxruntime.so`） |
| 模型 | `armor_allfp32.onnx`（fp32，opset 19） |
| 图像库 | OpenCV 4.2 |
| 框架 | ROS Noetic |

头文件/库位置：`third_party/ort/include`、`third_party/ort/lib`。
编译要求：**C++17**。

## 1. 先用两张样例图跑通

```bash
# 样例图路径按实际改
ls /tmp/nn_model/nn_model/sample_1.jpg
```

最小测试程序：

```cpp
#include "final_text/detector_interface.hpp"
#include <iostream>
int main() {
    OnnxDetector d("/path/armor_allfp32.onnx");
    std::cout << "ready=" << d.ready() << "\n";
    cv::Mat img = cv::imread("/path/sample_1.jpg");
    std::vector<ArmorObservation> a;
    d.detect(img, a);
    std::cout << "n=" << a.size() << "\n";
    for (auto& o : a)
        std::cout << "color=" << o.color << " num=" << o.number
                  << " score=" << o.score << "\n";
}
```

```bash
g++ -std=c++17 test.cpp \
  -Ifinal_text/include -Ifinal_text/third_party/ort/include \
  $(pkg-config --cflags opencv4) \
  -Lfinal_text/third_party/ort/lib -l:libonnxruntime.so.1 \
  -Wl,-rpath,$PWD/final_text/third_party/ort/lib \
  $(pkg-config --libs opencv4) -o test
./test
```

实测样例：
```
ready=1
n=2
color=1 num=4 score=0.927529
color=1 num=2 score=0.743388
```

> 先确认输出张量形状和分数范围，再接 Bag。

## 2. 输入协议与预处理

模型输入：`1 × 3 × 480 × 640`，RGB，`/255`。

```
scale = min(640/W, 480/H)
nw = round(W*scale), nh = round(H*scale)
pad_x = (640-nw)/2, pad_y = (480-nh)/2
canvas = 124 填充，图像贴到 (pad_x,pad_y)
```

代码位置：`include/final_text/detector_interface.hpp` 的 `detect()`。

**保存缩放/补边参数**（还原必须用）：
```cpp
float scale; int pad_x, pad_y;   // 每帧保存
```
还原：
```cpp
x_orig = (x_net - pad_x) / scale;
y_orig = (y_net - pad_y) / scale;
```

## 3. 输出张量解释

输出：`[1, 21, 6300]`，21 行 × 6300 个候选。

| 行 | 含义 |
|---|---|
| 0–3 | 颜色分数（蓝/红/熄灭/紫，4 类） |
| 4–12 | 编号分数（9 类，取 argmax+1） |
| 13–20 | 四个角点 (x1,y1)…(x4,y4)，网络坐标 |

当前 C++ 代码取 0–2 三类颜色（蓝/红/熄灭），第 3 类未用；
编号取 4–12 共 9 类。

## 4. 解码与稳定顺序

1. 颜色：行 0–2 取最大，`< color_thres` 丢弃；
2. 编号：行 4–12 取最大，**输出索引即编号**，`< number_thres` 丢弃；
3. 角点：8 个数两两成点，还原到原图；
4. 顺序统一为 **TL → TR → BR → BL**：
   - 当前按模型输出顺序映射 `p0→TL, p3→TR, p2→BR, p1→BL`；
   - 若模型/镜像导致顺序不稳，用几何校正：
     ```cpp
     // 以质心为中心按角度排序，再从左上开始
     ```
5. 用 `cv::dnn::NMSBoxes` 去重。

## 5. 置信度筛选与去重阈值

| 参数 | 默认 | 说明 |
|---|---|---|
| `color_thres` | 0.5 | 颜色分数低于此丢弃 |
| `number_thres` | 0.5 | 编号分数低于此丢弃 |
| `nms_thres` | 0.45 | IoU 阈值，重叠框去重 |

NMS 前先按四角点算外接矩形；同一块板多候选时保留分数最高。

## 6. 接入 PnP / 跟踪 / 发布

`src/detect_node_nn.cpp`：

1. `OnnxDetector::detect()` 得到 `ArmorObservation`；
2. 用四点 + `solvePnP`（IPPE→ITERATIVE）算 `tvec/rvec`；
3. 填 `ft_msgs/Detection`：`id`、`number`、`is_red`、`is_large_armor`、`pose`；
4. 发布 `/ft/detection_nn`；
5. 跟踪节点 remap 到 `/ft/detection_nn` 即可。

启动：

```bash
roslaunch final_text test_nn_cpp.launch
# 或
rosrun final_text detect_node_nn _model_path:=/path/armor_allfp32.onnx
```

## 7. 传统 / NN 切换

### 方式一：同一节点参数
`detection_node_v3_1.cpp` 有 `nn_enable`：
```bash
rosrun final_text detect_node_v3_1 _nn_enable:=true _nn_model_path:=/path/armor_allfp32.onnx
```

### 方式二：两个 launch
```bash
roslaunch final_text test_v3_1.launch     # 传统
roslaunch final_text test_nn_cpp.launch   # NN
```

## 8. Bag 上展示与对比

```bash
rosbag play bag/red_4.bag --clock -l
roslaunch final_text test_nn_cpp.launch
```

需要截图/录屏：
1. 原图 + 四角点（`/ft/debug/image_nn`）；
2. 最终位姿（RViz TF `armor_v3` + 三轴）；
3. 传统 `/ft/debug/image` 与 NN 同帧对比；
4. 漏检帧说明原因（过曝、遮挡、远距、角度）。

## 9. 单帧耗时

`detect_node_nn` 每秒打印：
```
NN armors=1 infer_avg=55.2 ms
```
只统计 `detect()`，不设 CPU 门槛。

## 10. 常见坑

| 现象 | 原因 |
|---|---|
| `ready=0` | 模型路径为空/版本不匹配 |
| `libonnxruntime.so.1` 找不到 | 缺 `.so.1` 符号链接 |
| `declared with an exception specification` | 没开 C++17 |
| 角点乱序 | 模型输出顺序与假设不一致，需几何排序 |
| 编号 >5 | 模型 9 类，业务上可截断到 1–5 |

## 11. 文件对照

| 内容 | 文件 |
|---|---|
| 预处理/解码/NMS | `include/final_text/detector_interface.hpp` |
| 节点/PnP/发布 | `src/detect_node_nn.cpp` |
| 启动 | `launch/test_nn_cpp.launch` |
| 传统链路 NN 分支 | `src/detection_node_v3_1.cpp`（`processNN`） |
