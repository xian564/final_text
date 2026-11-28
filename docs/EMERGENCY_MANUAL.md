# 装甲板视觉系统 · 工程设计总图与应急手册

> 用途：答辩/调试时快速定位。先看第 0 节“一分钟排障”，再查对应章节。
> 正稿：`src/detection_node_v3_1.cpp` + `src/ft_tracker_node.cpp`
> 进阶：`src/ft_rotation_tracker_node.cpp`、`src/detect_node_nn.cpp`
> 附加题：`bonus/`（v3 完整版、v3_2 车体几何、v3_split 分块、v4 骨架）

---

## 0. 一分钟排障（先看这里）

| 现象 | 最可能原因 | 一条命令验证 | 修复 |
|---|---|---|---|
| 没有 `/ft/detection` | 没图像 / 尺寸不符 / 节点没起 | `rostopic hz /hk_camera/image_raw/compressed` | 起 bag、检查 `width_/height_` |
| 有图但无框 | 阈值/曝光/面积 | `rqt_image_view /ft/debug/image` | 调 `blue_h_min`、`min_area`、`bar_v_min` |
| 框乱跳 | 锁定失效 / 多目标 | 看 `lock_left_` 日志、`/ft/detection` 的 id | 调 `lock_gate_px`、`lock_ratio_tol` |
| z 跳变 | PnP 退化/角点顺序错 | 看 `z`、四角点 | 检查四点顺序、`quad_*` 过滤 |
| `/ft/track` 没有 | 没检测 / remap 错 | `rostopic info /ft/track` | 确认订阅 `/ft/detection` |
| 跟踪断 | 丢检 > `lost_time_thresh` | 看 `/ft/track` 的 `tracking` | 调 `lost_time_thresh`、`assoc_gate` |
| 预测点乱飞 | Q/R 太大 / 速度异常 | 看 `velocity` | 调 `q_pos/q_vel/r_pos`、`vmax_mps` |
| NN `ready=0` | 路径空 / 版本不匹配 | 看启动日志 | 检查 `model_path`、ORT 版本 |
| NN 无框 | 预处理/阈值 | 样例图测试 | 看 `score` 范围、调 `color_thres` |
| 编译 `libonnxruntime.so.1` 找不到 | 缺符号链接 | `ls third_party/ort/lib` | `ln -sf libonnxruntime.so libonnxruntime.so.1` |
| 编译 `exception specification` | 没开 C++17 | 看 CMake | `set(CMAKE_CXX_STANDARD 17)` |
| `roslaunch` 起不来 | 重名节点/旧进程 | `rosnode list` | `pkill -x 节点名` 后重起 |
| 改参数不生效 | 没走 YAML / 节点没重读 | `rosparam get` | 用 `test_vision.launch` 或 `dynparam` |

---

## 1. 系统总图

```
/hk_camera/image_raw/compressed
        │  image_transport (compressed)
        ▼
┌─────────────────────────────┐
│ detect_node_v3_1            │
│  ① 分色 HSV/灰度            │
│  ② 灯条: 轮廓/面积/长宽/亮度 │
│  ③ 配对: 间距/角度/ratio     │
│  ④ 四点 + solvePnP          │
│  ⑤ 锁定/平滑/TF             │
└─────────────┬───────────────┘
              │ /ft/detection (ft_msgs/Detections)
              ▼
┌─────────────────────────────┐        /ft/track (ft_msgs/Track)
│ ft_tracker_node             │ ──────► /ft/debug/marker
│  Kalman [x,y,z,vx,vy,vz]    │
│  assoc_gate / lost / T      │
└─────────────────────────────┘
              │
              ▼
        RViz: TF armor_v3 + marker

NN 链路：
image ─► detect_node_nn (ORT) ─► /ft/detection_nn ─► ft_tracker (remap)
旋转链路：
/ft/detection ─► ft_rotation_tracker ─► /ft/track_rot + /ft/debug/marker_rot
```

### 话题 / 消息 / 坐标系

| 名称 | 类型 | 说明 |
|---|---|---|
| `/hk_camera/image_raw/compressed` | CompressedImage | 输入 |
| `/hk_camera/camera_info` | CameraInfo | 内参/畸变，frame `hk_camera_ca` |
| `/ft/detection` | `ft_msgs/Detections` | 每帧检测数组 |
| `/ft/track` | `ft_msgs/Track` | `id/tracking/position/velocity/position_predict` |
| `/ft/track_rot` | `ft_msgs/Track` | 旋转模型：`body_center/valid` |
| `/ft/debug/image` | Image | 传统调试图 |
| `/ft/debug/image_nn` | Image | NN 调试图 |
| `/ft/debug/marker` | Marker | 预测位置（球/线） |
| `/ft/debug/marker_rot` | Marker | 车体中心/当前/预测 |
| TF | `camera -> armor_v3` | 位姿；v3_2 有 `car_v3_2` |

消息字段：
```
Detection: header,id,number,is_red,is_large_armor,pose
Detections: header,detections[]
Track: header,id,tracking,position,velocity,position_predict,body_center,valid
```

### 文件地图

| 文件 | 职责 |
|---|---|
| `src/detection_node_v3_1.cpp` | 正稿检测（传统 + `nn_enable` 分支 + 数字模板） |
| `src/ft_tracker_node.cpp` | Kalman 跟踪 + 提前量 + marker |
| `src/ft_rotation_tracker_node.cpp` | 圆周模型/相位预测 |
| `src/detect_node_nn.cpp` | ORT NN 节点 |
| `include/final_text/detector_interface.hpp` | ORT 预处理/解码/NMS |
| `include/final_text/motion_geometry.hpp` | 车体几何（v3_2） |
| `config/params.yaml` | 检测参数 |
| `config/tracker.yaml` | 跟踪参数 |
| `config/rotation.yaml` | 旋转参数 |
| `cfg/Vision.cfg` | dynamic_reconfigure 27 项 |
| `launch/test_vision.launch` | 一条 launch 切 tradition/nn |
| `launch/test_v3_1.launch` | 正稿 |
| `launch/test_nn_cpp.launch` | NN + 跟踪 |
| `launch/test_rotation.launch` | 旋转模型 |
| `scripts/capture_compare.py` | 传统/NN 同帧截图 |
| `scripts/capture_debug.py` | 单路截图 |
| `scripts/nn_detector_node.py` | ORT Python 对照 |

---

## 2. 作业要求速查

### 基础（70）
- 不用 `imshow/waitKey`；调试图发 `/ft/debug/image`
- 阈值/尺寸/滤波/T 不硬编码：YAML + dynamic_reconfigure
- 内参/畸变从 `camera_info` 读
- 所有消息 `header.stamp` = 图像时间戳
- 消息定义进仓库；帧内 id 唯一
- 一条 launch 起检测+跟踪+可视化
- 跟踪：Kalman/Alpha-Beta、丢检 predict `tracking=false`、>0.5s 删除、T 提前量、marker
- README 写清：四点定义、坐标系、单位、滤波/Q/R、关联门限
- 2–3 分钟视频

### 进阶（选做 30）
- 旋转目标建模与装甲板选择（10）：车体中心/相位/角速度、R、跨 2π、多板选择、T 对比
- 神经网络（10）：C++ 后端、预处理/解码/角点还原/NMS、接入 PnP/跟踪、参数切换、同帧对比、耗时

---

## 3. 检测链路知识

### 3.1 分色
- BGR→HSV，红色两段：`H∈[0,10] ∪ [170,179]`，蓝/黄各一段；
- `S,V ≥ hsv_sv_min`；
- 形态学开运算去噪（`morph_size`）；
- 输出 `seg_mask`；同时保留 `v_chan` 给高光判断。

### 3.2 灯条
对每个轮廓：
1. `contourArea ≥ min_area`；
2. `minAreaRect` 得中心/宽高/角度；
3. 长宽比、填充率、框内平均亮度 (`mean(high_mask)` 或 V)；
4. 矩质心定中心，角度归一化到 [-90,90]；
5. 断笔合并：共线、间距小、角度近的短灯条合并。

### 3.3 配对
两块灯条 `i,j`：
- 水平间距 `dx`、高度差 `dy`、角度差 `ddeg`；
- `ratio = 透视修正间距 / 灯条长` 判断大小板：
  - 小板 ≈ 2.25，大板 ≈ 3.8
  - `ratio_s_max=3.0`、`ratio_l_max=5.0`
- 几何否决：对角线差、对边比、平行度、共面距离。

### 3.4 四点与 PnP
- 四点：左灯条左边缘、右灯条右边缘的上下角点，**TL→TR→BR→BL**；
- 按板体长宽比把四边形高度拉到 `Wb×bH`；
- 物体点（米）：
  - 大：`0.230 × 0.060`
  - 小：`0.135 × 0.060`
- `solvePnP(..., SOLVEPNP_IPPE)`，失败退 `ITERATIVE`；
- `z∈[z_min,z_max]` 过滤，`isfinite` 过滤 NaN；
- `rvec→四元数`，`tvec` 米。

### 3.5 目标选择（v3.1）
```
if lock_left_ > 0:
    按 lock_pred_ 距离 + 比例 ±lock_ratio_tol 匹配
    命中 → 锁定；连续 7 帧未命中 → 解锁
else:
    选 |avg_tilt| 最小（最正对）
选中后：
    lock_prev_=lock_center_; lock_center_=mt.center2d
    lock_pred_=center+(center-prev)
    lock_ratio_=mt.ratio
```

### 3.6 平滑与 TF
- `pushSmooth`：位置/yaw 滑动平均，`z` 中值；
- TF：`camera_frame -> armor_v3`，时间戳用图像时间。

---

## 4. 跟踪与旋转模型

### 4.1 Kalman
状态 `[x,y,z,vx,vy,vz]`，观测 `[x,y,z]`：
```
predict:  x += vx*dt ...
correct:  K = P Hᵀ (H P Hᵀ + R)⁻¹
```
参数（`tracker.yaml`）：
- `q_pos=0.1`、`q_vel=0.5`、`r_pos=0.05`
- 位置噪声小、速度噪声大 → 速度更灵活
- `T=0.1`：`position_predict = position + velocity*T`
- `predict_cap_m=0.6`：预测偏移限幅
- `assoc_gate=0.5m`：观测关联门限
- `lost_time_thresh=0.5s`：超时删除轨迹
- `max_dt_s=0.5`：dt 异常重捕

### 4.2 旋转模型（`ft_rotation_tracker_node`）
```
圆拟合（Kasa）：
  2a·x + 2b·z + c0 = x² + z²
  → 圆心 (a,b)，R = sqrt(c0 + a² + b²)
相位：θ = atan2(z-cz, x-cx)
角速度：Δθ 用 atan2(sin,cos) 归一，ω = Δθ/dt，低通
预测：
  center' = center + v*T
  θ' = θ + ω*T
  armor' = center' + R·[cos θ', sin θ']
```
- 正角=左转，负角=右转（相机系约定，README 已写）
- `valid=false`：遮挡/丢失
- 三个 marker：车体中心（蓝）、当前（绿）、预测（红）

### 4.3 旋转模型适用/失效
| 适用 | 失效 |
|---|---|
| 目标在水平面绕车体中心转 | 正对时相位退化 |
| 能看到至少 8 个点/2s | 遮挡/远距点少 |
| R 在 0.1–2m | 半径突变/多车 |
| 匀速旋转 | 换向、急停 |

---

## 5. 神经网络知识（含培训 PPT 要点）

### 5.1 网络结构（PPT 第 4–11 页）
YOLO 分三段：

| 段 | 作用 | 输出 |
|---|---|---|
| Backbone | 逐级下采样提特征 | P3 60×80、P4 30×40、P5 15×20 |
| Neck | 自顶向下 + 自底向上融合 | 同三档，通道 64/128/256 |
| Head | 每档预测，拼成一张表 | `1×21×6300` |

- 输入：`1×3×480×640`
- 输出：`1×21×6300`
- 21 行：
  - 0–3 颜色（蓝/红/白/紫）
  - 4–12 编号（0 哨兵、1–4 兵种、6 前哨、7–8 基地）
  - 13–20 四个点 x/y
- 6300 列来源：
  - P3 60×80 = 4800（步长 8）
  - P4 30×40 = 1200（步长 16）
  - P5 15×20 = 300（步长 32）

列号→坐标：
```
j = i - O; c = j % W; r = j / W
x = c*s + s/2; y = r*s + s/2
```

### 5.2 训练与导出（PPT 第 13–20 页）
1. 采集视频抽帧；
2. labelme 点四个角，顺序 **LT→RT→RB→LB**；
3. 标签一行 13 个数：`cls cx cy w h x1 y1 ... x4 y4`（归一化）；
4. `data.yaml`：`kpt_shape: [4,2]`、`flip_idx: [1,0,3,2]`；
5. 训练：`yolo pose train ... imgsz=640`；
6. 导出：`model.export(format="onnx", imgsz=[480,640], opset=12, simplify=True)`。

导出后核对：输入 `1×3×480×640`、输出 `1×21×6300`、通道顺序/类别数不变。

### 5.3 部署两条路
| 后端 | 优点 | 本项目状态 |
|---|---|---|
| OpenVINO | Intel CPU 快，PPT 主推 | 未接入（无归档包） |
| ONNX Runtime C++ | 跨平台，直接吃 ONNX | ✅ 已接入 `1.31.1` |

#### ORT C++ 关键代码
```cpp
Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "armor_nn");
Ort::SessionOptions opt; opt.SetIntraOpNumThreads(1);
Ort::Session session(env, path, opt);
auto out = session.Run(Ort::RunOptions{nullptr}, in_names, &tensor, 1, out_names, 1);
```

#### 预处理（每帧）
```cpp
scale = min(640.0/W, 480.0/H);
nw = round(W*scale); nh = round(H*scale);
pad_x = (640-nw)/2; pad_y = (480-nh)/2;
canvas = 124 填充；resize 后贴到 (pad_x,pad_y)
input[0..2] = RGB/255，NCHW
```
**保存** `scale/pad_x/pad_y`，还原角点：
```cpp
x_orig = (x_net - pad_x)/scale;
```

#### 解码
- 颜色：行 0–2 取 max（第 3 类紫未用），`< color_thres(0.5)` 丢；
- 编号：行 4–12 取 max，`< number_thres(0.5)` 丢；
- 角点：行 13–20 两两成点；
- 顺序：`p0→TL, p3→TR, p2→BR, p1→BL`（乱序时按质心角度重排）；
- 去重：`cv::dnn::NMSBoxes(boxes, scores, 0.5, 0.45)`。

### 5.4 数字模板（传统补充）
- `models/digit_1..5.png`；
- Otsu 二值化 + 自动反色 + resize 20×30；
- `matchTemplate(TM_CCOEFF_NORMED)`，最高分 >0.5 判编号，否则 0；
- 没有模板 → `number=0`。

### 5.5 耗时（实测）
```
NN armors=1 infer_avg=55.2 ms   （≈18 FPS）
传统 /ft/detection 20–38 Hz
```

### 5.6 常见 NN 错误
| 现象 | 原因 | 修复 |
|---|---|---|
| `ready=0` | 路径空/版本不匹配 | 检查 `model_path`、头/库同版本 |
| 角点错位 | 顺序假设错 | 按 TL/TR/BR/BL 重排 |
| 框重复 | NMS 阈值过大 | 降 `nms_thres` |
| 编号乱 | 编号阈值低 | 升 `number_thres` |
| 推理慢 | 线程过多/图太大 | `SetIntraOpNumThreads(1)`、固定 640×480 |

---

## 6. 代码讲解与诊断标准（单独拿出来）

> 这一段不是算法，是“怎么看代码、怎么判断代码是不是整包生成、怎么定位错误”。

### 6.1 判断“整包生成”的 8 个信号
1. **一次成型**：提交历史只有一个 `final`，没有失败/回退记录；
2. **风格过匀**：变量命名、注释密度、缩进完全一致，没有个人习惯差异；
3. **注释解释“是什么”而非“为什么”**：例如 `// 定义变量`，缺少踩坑说明；
4. **没有 badcase 痕迹**：没有“过曝/遮挡/换板”针对性分支；
5. **参数全硬编码**：没有 YAML、没有调参记录；
6. **异常处理模板化**：`try/catch` 只打印，没有恢复逻辑；
7. **无法逐行解释**：作者说不清某个阈值为什么是 50；
8. **文档与代码不一致**：README 写的功能代码里没有。

> 反例：本项目有 `docs/attempts/`、回退记录、参数化、badcase 分支，能逐段解释。

### 6.2 RE 调（Review & Debug）流程
```
1. 跑起来：最小输入 → 看到输出
2. 画数据流：输入 → 中间量 → 输出，标出每个变量的归属
3. 卡边界：在 block 入口/出口打印尺寸、范围、NaN
4. 找不变量：z>0、四点顺序、id 唯一、stamp=图像时间
5. 最小复现：固定一帧图/一个 bag，关掉其他分支
6. 二分定位：注释掉后半段，看前半段是否正常
7. 修一个点：只改一个变量/阈值，验证
8. 回归：再跑原 bag，确认没引入新问题
```

### 6.3 逐行讲解标准（答辩用）
对每个函数/Block 回答 4 个问题：
| 问题 | 例 |
|---|---|
| 输入是什么 | `Knee&`：图、掩膜、灯条 |
| 输出写哪里 | `knee.targets` |
| 失败会怎样 | 空 targets → 走预测 |
| 为什么这样 | 面积过滤去掉噪点 |

### 6.4 代码设计思路
- **数据归属唯一**：跨阶段变量进 `Knee`，局部变量不外泄；
- **阶段单一职责**：color / guard / bars / pair / publish / draw / output；
- **参数外置**：阈值、尺寸、T、Q/R 全走 YAML + cfg；
- **可观测**：每阶段有 debug 图/日志；
- **可回退**：一次只改一处，保留能跑的版本。

### 6.5 诊断决策树
```
无输出
├─ 有图像? ──否──► bag/驱动/压缩传输
└─ 是
   ├─ 有检测? ──否──► 阈值/面积/曝光/颜色
   └─ 是
      ├─ 框错? ──► 四点顺序/PnP/几何否决
      ├─ 跳? ──► 锁定/多目标/检测抖动
      └─ 跟踪断? ──► assoc_gate/lost/Q-R
```

---

## 7. 故障手册（按模块）

### 7.1 构建/运行
| 错误 | 原因 | 修复 |
|---|---|---|
| `catkin_make` 找不到 OpenCV | 依赖没装 | `sudo apt install libopencv-dev` |
| `libonnxruntime.so.1` 找不到 | 缺 `.so.1` | 建符号链接；`BUILD_RPATH` |
| `exception specification` | C++ 标准低 | C++17 |
| `roslaunch` 节点重名 | 旧进程 | `pkill -x 名字`，`rosnode cleanup` |
| `use_sim_time` 无数据 | 没 `--clock` | `rosbag play --clock` |

### 7.2 检测
| 现象 | 检查 | 调参 |
|---|---|---|
| 全黑/全白 | 曝光、`hsv_sv_min` | 降/升 `hsv_sv_min` |
| 灯条断 | `min_area`、`morph_size` | 降 `min_area`、加闭运算 |
| 配对错 | `pair_dy_ratio`、`max_angle_diff` | 收紧 |
| 大小板错 | `ratio_s_max/ratio_l_max` | 按 2.25/3.8 调 |
| z 偏大 | 四点高度用了灯条长 | 按 `Wb×bH` 拉高 |
| 框跳 | 锁定 | `lock_gate_px`、`lock_ratio_tol` |

### 7.3 跟踪
| 现象 | 检查 | 调参 |
|---|---|---|
| 不跟 | 话题/remap | `rostopic info` |
| 跟错板 | `assoc_gate` 太大 | 收紧到 0.5 |
| 预测滞后 | `T`、`q_vel` | 调 T、Q |
| 断得太快 | `lost_time_thresh` | 增大 |
| marker 不动 | `marker_topic`、frame | 检查 TF |

### 7.4 旋转
| 现象 | 检查 | 修复 |
|---|---|---|
| 中心乱 | 拟合点数/R 范围 | 增大 `fit_window`、限制 R |
| 角速度抖 | 相位跳变 | 低通、检查跨 2π |
| 预测绕反 | 符号约定 | 改 `omega` 符号 |
| 一直 valid=false | 关联失败 | 放宽 `assoc_gate` |

---

## 8. 常用命令

```bash
# 构建
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 数据
rosbag play bag/red_4.bag --clock -l

# 正稿
roslaunch final_text test_v3_1.launch
roslaunch final_text test_vision.launch mode:=nn

# 检查
rostopic hz /ft/detection /ft/track /ft/debug/image
rostopic echo -n1 /ft/detection
rosrun rqt_reconfigure rqt_reconfigure
rosrun dynamic_reconfigure dynparam set /vision_node_v3 blue_h_min 120

# 截图
rosrun final_text capture_compare.py _out:=/tmp/compare _n:=3
rosrun final_text capture_debug.py _topic:=/ft/debug/image _out:=/tmp/shot _n:=5

# 记录
rosbag record -O run.bag /ft/detection /ft/track /ft/debug/image
```

---

## 9. 参数速查

| 文件 | 关键参数 |
|---|---|
| `params.yaml` | `red/blue/yellow_h_*`、`hsv_sv_min`、`min_area`、`bar_v_min`、`ratio_*`、`armor_*_w/h`、`max_z`、`z_min/z_max`、`lock_*`、`time_window_s`、`nn_enable`、`nn_model_path` |
| `tracker.yaml` | `T`、`q_pos/q_vel/r_pos`、`assoc_gate`、`lost_time_thresh`、`max_dt_s`、`vmax_mps`、`predict_cap_m` |
| `rotation.yaml` | `T`、`fit_window`、`phase_window`、`assoc_gate`、`default_radius`、`lost_time_thresh` |
| `cfg/Vision.cfg` | 识别阈值 27 项（rqt_reconfigure） |

---

## 10. 测试与截图

完整流程：[`docs/TESTING.md`](TESTING.md)
- 正稿：`test_vision.launch mode:=tradition`
- NN：`mode:=nn`
- 同帧对比：`capture_compare.py`
- 漏检说明模板：TESTING 8.4
- 对比示例：`docs/compare_demo.png`（左传统、右 NN）

---

## 11. 安全 / 来源 / AI

### ORT 来源（需能说清）
| 项 | 值 |
|---|---|
| 版本 | 1.31.1 |
| 头文件 | jsDelivr（GitHub 镜像） |
| 库 | PyPI 清华镜像的 `onnxruntime` wheel |
| SHA256 | lib `fc44112f…c1d3`；cxx_api `1e80d494…467` |
| 风险 | 未与官方 release 校验；已固定版本，运行在容器内 |

### AI 使用
- 必须能逐行解释；
- 不能读真值；
- 不能整包生成后原样提交；
- 记录见 `ai_usage.md`、`docs/ai_chats/`。

---

## 12. 30 秒答辩提纲

1. 系统：检测 → `/ft/detection` → Kalman → `/ft/track`，一条 launch；
2. 检测：HSV→灯条→配对→四点 PnP→锁定；
3. 跟踪：6 状态 Kalman，Q/R、T、门限、丢检；
4. 进阶：ORT C++ NN、旋转圆周模型、编号；
5. 工程：YAML/cfg 参数化、NaN/时间戳、截图/测试流程；
6. 不足：旋转对比数据、视频、OpenVINO 未接。
