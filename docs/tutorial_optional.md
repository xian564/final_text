# 可选项教程 v1.1（编号识别 / 旋转预测 / 文件拆分）

> v1.1 变化：每一步都标了**具体文件 + 行号 + 插入锚点**。
> 行号基于当前仓库版本，改动后会有偏移，以锚点为准。

---

## A. 编号识别（1–5）

### A1. 消息加字段
**文件**：`ft_msgs_pkg/msg/Detection.msg`（同步改 `~/catkin_ws/src/ft_msgs/msg/Detection.msg`）
**位置**：第 2 行 `int32 id` 下面
```diff
  int32 id
+ int32 number        # 0=未知, 1..5=编号
  bool is_red
```

### A2. NN C++ 节点填编号
**文件**：`src/detect_node_nn.cpp`
**位置**：第 133 行 `d.id = (int)k + 1;` 下面
```cpp
d.id = (int)k + 1;
d.number = a.number;       // ArmorObservation.number（0..8）
```

### A3. v3.1 NN 分支填编号
**文件**：`src/detection_node_v3_1.cpp`
**位置**：第 345 行 `d.id = (int)k + 1;` 下面
```cpp
d.id = (int)k + 1;
d.number = a.number;
```

### A4. Python ORT 节点填编号
**文件**：`scripts/nn_detector_node.py`
**位置**：第 103 行 `det.id = k+1; ...` 同一行追加
```python
det.id = k+1; det.number = int(d['num'])
det.is_red = (d['color']==1); det.is_large_armor = bool(large)
```

### A5. 编译 + 测试
```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
rosrun final_text detect_node_v3_1 _nn_enable:=true _nn_model_path:=/root/catkin_ws/bag/armor_allfp32.onnx
rostopic echo /ft/detection | grep number
```

### A6.（可选）传统链路加模板匹配
- **结构体加字段**：`src/detection_node_v3.cpp` 第 171 行 `struct Target {` 内加 `int number = 0;`
- **发布处填字段**：`src/detection_node_v3.cpp` 第 892 行 `d.id = tg.id;` 下面加 `d.number = tg.number;`
- 识别逻辑放在 PnP 成功后（`tg.has_pose` 分支内），对 `tg.corners` 外接框做 `matchTemplate`。

---

## B. 旋转/绕行预测

### B1. 消息加字段
**文件**：`ft_msgs_pkg/msg/Track.msg`
**位置**：第 3 行 `bool tracking` 下面
```diff
  bool tracking
+ float32 yaw
+ float32 omega
  geometry_msgs/Point position
```

### B2. 跟踪节点加头文件
**文件**：`src/ft_tracker_node.cpp`
**位置**：第 12 行 `#include <ft_msgs/Track.h>` 下面
```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
```

### B3. 加状态成员
**文件**：`src/ft_tracker_node.cpp`
**位置**：第 244 行 `bool is_tracking_ = false;` 下面
```cpp
double last_yaw_ = 0.0;
double omega_ = 0.0;
bool   has_yaw_ = false;
```

### B4. 从四元数取 yaw 并估 omega
**文件**：`src/ft_tracker_node.cpp`
**位置**：第 111 行 `const ft_msgs::Detection& det = msg->detections[pick];` 下面
```cpp
double yaw = 0.0;
if (det.pose.orientation.w != 0.0) {
    tf2::Quaternion q(det.pose.orientation.x, det.pose.orientation.y,
                      det.pose.orientation.z, det.pose.orientation.w);
    double r, p; tf2::Matrix3x3(q).getRPY(r, p, yaw);
}
if (has_yaw_) {
    double dt = (stamp - last_det_time_).toSec();
    if (dt > 1e-3) {
        double w = std::atan2(std::sin(yaw - last_yaw_), std::cos(yaw - last_yaw_)) / dt;
        omega_ = 0.8 * omega_ + 0.2 * w;   // 低通
    }
}
last_yaw_ = yaw;
has_yaw_ = true;
```

### B5. 发布 yaw / omega
**文件**：`src/ft_tracker_node.cpp`
**位置**：第 176 行 `out.tracking = tracking;` 下面
```cpp
out.yaw   = (float)(last_yaw_ + omega_ * (stamp - last_det_time_).toSec());
out.omega = (float)omega_;
```

### B6. 编译 + 测试
```bash
catkin_make && roslaunch final_text test_v3.launch
rostopic echo /ft/track | grep -E "yaw|omega"
```

---

## C. 单文件拆分（v3 1671 行 → ≤1000）

### C1. 新建 `include/final_text/types.hpp`
**搬移来源**：`src/detection_node_v3.cpp` 第 161–184 行
- `struct Bar`（161–170）
- `struct Target`（171–184）

内容：
```cpp
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct Bar { ... };       // 原样搬
struct Target { ... };    // 原样搬
```
然后 v3 里删除这两段，顶部加：
```cpp
#include "final_text/types.hpp"
```

### C2. 新建 `src/v3_utils.cpp` + `include/final_text/v3_utils.hpp`
**搬移来源**（`src/detection_node_v3.cpp`）：
| 函数 | 行号 |
|---|---|
| `dashedRect` | 234–254 |
| `brightGradient` | 256–274 |
| `fourBrightness` | 276–约 300 |
| `colorSegment`（定义） | 1597–文件末 |

头文件声明 + 实现放 `v3_utils.cpp`，v3 顶部加：
```cpp
#include "final_text/v3_utils.hpp"
```
并删除第 37 行的前向声明。

### C3. CMake 加库
**文件**：`CMakeLists.txt`
**位置**：第 250 行 `add_executable(detect_node_v3 ...)` 之前
```cmake
add_library(v3_utils src/v3_utils.cpp)
target_link_libraries(v3_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})
```
并把 v3 链接改为：
```cmake
target_link_libraries(detect_node_v3 v3_utils ${catkin_LIBRARIES} ${OpenCV_LIBS})
```

### C4. 验证
```bash
catkin_make && rosrun final_text detect_node_v3
rostopic hz /ft/debug/image     # 应在 25–30Hz
```
行为应与拆分前一致（只动代码组织，不动算法）。

---

## 建议顺序
1. B（旋转预测，改动最小）
2. C（拆分，纯重构）
3. A（编号识别，优先用 NN 的 number）
