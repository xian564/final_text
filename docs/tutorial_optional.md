# 可选项教程（编号识别 / 旋转预测 / 文件拆分）

> 三个都可以独立做，互不影响。建议先做“旋转预测”，再做“文件拆分”，最后“编号识别”。

---

## A. 编号识别（1–5）

传统 HSV 链路不识别数字，最省的路线是复用 NN 模型输出的 number。

### A1. 消息加字段
`ft_msgs_pkg/msg/Detection.msg`（以及 `~/catkin_ws/src/ft_msgs/msg/`）：
```
int32 id
int32 number        # 0=未知，1..5=编号
```

### A2. NN 分支填 number
`src/detect_node_nn.cpp`：
```cpp
d.number = a.number;   // ArmorObservation.number 已经是 0..8
```
`scripts/nn_detector_node.py`：
```python
from ft_msgs.msg import Detection
det.number = int(d['num'])
```

### A3. 传统分支（可选）
不想依赖 NN 时，用模板匹配：
1. 采集 1–5 数字样本，存 `models/digit_1..5.png`；
2. 对装甲框内灰度图二值化、缩放到 20×30；
3. `cv::matchTemplate` 取最高分，>0.6 判为该编号；
4. 在 `Target` 加 `int number`，发布时填入。

### A4. 测试
```bash
rosrun final_text detect_node_nn _model_path:=/root/catkin_ws/bag/armor_allfp32.onnx
rostopic echo /ft/detection | grep number
```

---

## B. 旋转/绕行预测

当前只有匀速直线，旋转需要估计 yaw 角速度。

### B1. 消息加字段
`ft_msgs_pkg/msg/Track.msg`：
```
float32 yaw
float32 omega
```

### B2. 跟踪里估计 omega
`src/ft_tracker_node.cpp`：
```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double yaw = 0.0;
if (det.pose.orientation.w != 0.0) {
    tf2::Quaternion q(det.pose.orientation.x, det.pose.orientation.y,
                      det.pose.orientation.z, det.pose.orientation.w);
    double r, p; tf2::Matrix3x3(q).getRPY(r, p, yaw);
}
double dt = (stamp - last_det_time_).toSec();
if (dt > 1e-3) {
    double w = std::atan2(std::sin(yaw - last_yaw_), std::cos(yaw - last_yaw_)) / dt;
    omega_ = 0.8 * omega_ + 0.2 * w;       // 低通
}
last_yaw_ = yaw;
```

### B3. 预测时外推 yaw
```cpp
out.yaw   = (float)(last_yaw_ + omega_ * (now - last_det_time_).toSec());
out.omega = (float)omega_;
```

### B4. 测试
```bash
rostopic echo /ft/track | grep -E "yaw|omega"
```
匀速直线时 omega≈0；绕行时 omega 应稳定非零。

---

## C. 单文件拆分（v3 1671 行 → 目标 ≤1000）

### C1. 抽类型
新建 `include/final_text/types.hpp`：
```cpp
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct Bar { ... };      // 从 v3 原样搬
struct Target { ... };
```

### C2. 抽工具函数
新建 `src/v3_utils.cpp` + `include/final_text/v3_utils.hpp`：
- `colorSegment`
- `brightGradient`
- `fourBrightness`
- `dashedRect`

v3 里改成 `#include "final_text/v3_utils.hpp"`，删除原实现。

### C3. CMake
```cmake
add_library(v3_utils src/v3_utils.cpp)
target_link_libraries(v3_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(detect_node_v3 src/detection_node_v3.cpp)
target_link_libraries(detect_node_v3 v3_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})
```

### C4. 验证
```bash
catkin_make
rosrun final_text detect_node_v3
```
行为应与拆分前一致；只动了代码组织，不动算法。

---

## 建议顺序
1. B（旋转预测，改动最小、最容易看到效果）
2. C（拆分，纯重构、便于答辩讲结构）
3. A（编号识别，最好直接用 NN 的 number）
