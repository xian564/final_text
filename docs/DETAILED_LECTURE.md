# 装甲板视觉 · 深度讲解版

> 写法参照导出对话里“彻底讲清楚”的格式：
> **代码位置 → 它是什么 → 核心原理/矩阵 → 工程坑点 → 总结**。
> 目标：不省略推导，能对着代码讲。

---

# 第一讲：图像接入（image_transport + cv_bridge + camera_info）

## 1.1 代码位置

```cpp
// detection_node_v3_1.cpp 构造函数
image_transport::ImageTransport it_(nh_);
sub_ = it_.subscribe("/hk_camera/image_raw", 1,
                     &VisionNodeV3::imageCallback, this,
                     image_transport::TransportHints("compressed"));
camera_info_sub_ = nh_.subscribe("/hk_camera/camera_info", 1,
                                 &VisionNodeV3::cameraInfoCallback, this);
```

```cpp
// imageCallback 开头
cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
cv::Mat img = cv_ptr->image;
if (img.cols != width_ || img.rows != height_) return;
```

## 1.2 它是什么

### image_transport
ROS 的 `image_transport` 是对 `sensor_msgs/Image` 的封装，允许同一路图像有
`raw / compressed / compressedDepth / theora` 多种传输插件。

- 你订阅的**基话题**是 `/hk_camera/image_raw`；
- `TransportHints("compressed")` 告诉它：**实际去连 `/hk_camera/image_raw/compressed`**；
- 好处：带宽小（JPEG 压缩），适合无线/仿真；
- 坑：如果你写死 `ros::Subscriber` 订阅 `/hk_camera/image_raw`，但 bag 里只有
  `/compressed`，就永远收不到数据。

### cv_bridge
把 ROS 的 `sensor_msgs/Image` 转成 OpenCV `cv::Mat`。

`toCvShare` 返回 `CvImageConstPtr`，**不拷贝**图像数据，只做浅引用；
`toCvCopy` 会拷贝。回调里只读不改时用 `toCvShare` 省一次内存拷贝。

`"bgr8"` 是目标编码：ROS 图像可能是 `rgb8`、`mono8`、`bayer_*`，
`cv_bridge` 会按需转换。**不要假设输入就是 BGR**。

### camera_info
`CameraInfo` 里：
- `K`：3×3 内参矩阵
  ```
  [fx  0  cx]
  [ 0 fy  cy]
  [ 0  0   1]
  ```
- `D`：畸变系数（k1,k2,p1,p2,k3…）
- `header.frame_id`：相机光学坐标系（本数据 `hk_camera_ca`）

PnP 必须用真实内参，不能用写死的 `fx=800`。

## 1.3 为什么先判断尺寸

```cpp
if (img.cols != width_ || img.rows != height_) return;
```
仿真/回放时可能出现分辨率不一致（例如 1440×1080 vs 1280×720）。
如果直接跑，后续的像素阈值、ROI、角点都会错位。先挡掉是最便宜的防御。

## 1.4 工程坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 订阅基话题但只有 compressed | 无图 | `TransportHints("compressed")` |
| 用 `toCvCopy` 每帧拷贝 | CPU 高 | 只读用 `toCvShare` |
| 编码写死 `bgr8` 但输入 mono | 通道错 | 先看 `msg->encoding` |
| `camera_info` 晚于图像 | 首帧 K 为空 | `is_ready_` 标志，等 K 到齐 |
| `frame_id` 用错 | TF 树断 | 用 `camera_info.header.frame_id` |

## 1.5 总结

图像接入只做三件事：**拿到图、拿到内参、拿到 frame**。
三件事任何一件错，后面全错。所以回调第一段永远是“校验 + 转换”，不做算法。

---

# 第二讲：HSV 分色与形态学

## 2.1 代码位置

```cpp
// colorSegment(...)
cv::cvtColor(canvas, hsv_full, cv::COLOR_BGR2HSV);
cv::inRange(hsv_full, cv::Scalar(red_h_min_, hsv_sv_min_, hsv_sv_min_),
            cv::Scalar(red_h_max_, 255, 255), mask_red1);
cv::inRange(hsv_full, cv::Scalar(170, hsv_sv_min_, hsv_sv_min_),
            cv::Scalar(179, 255, 255), mask_red2);
cv::bitwise_or(mask_red1, mask_red2, mask_red);
cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN,
                 cv::getStructuringElement(cv::MORPH_RECT,
                                           cv::Size(morph_size_, morph_size_)));
```

## 2.2 为什么用 HSV 不用 RGB

RGB 三个通道都随光照一起变：
- 同一块红布，亮处 `(255,0,0)`，暗处 `(80,0,0)`；
- 用 RGB 阈值要写三维立方体，难调。

HSV 把“颜色”和“亮度”拆开：
- `H`：色相（红≈0/180，蓝≈120，黄≈30）；
- `S`：饱和度（越鲜艳越大）；
- `V`：亮度。

红色横跨 0° 和 180°，所以**必须两段**：
```
H ∈ [0, 10] ∪ [170, 179]
```

## 2.3 inRange 的参数

```cpp
cv::inRange(src, lowerb, upperb, dst);
```
`dst` 是 `CV_8UC1`，满足范围的像素为 255，否则 0。

注意 `S/V` 下限 `hsv_sv_min_` 同时限制饱和度和亮度：
- 太低：灰墙、反光也被当成颜色；
- 太高：暗处灯条丢失。

## 2.4 形态学开运算

```cpp
MORPH_OPEN = 先腐蚀后膨胀
```
作用：去掉小噪点（腐蚀），再把主体恢复（膨胀）。
`morph_size_` 一般 3 或 5，太大把细灯条腐蚀没。

## 2.5 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 只取一段红 | 一半红丢失 | 两段 `bitwise_or` |
| 忘记 `S/V` 下限 | 白墙/地面反光进掩膜 | 调高 `hsv_sv_min` |
| 开运算核太大 | 灯条断裂 | 核 ≤5，或改闭运算 |
| 在 BGR 上直接阈值 | 颜色随光照漂 | 统一转 HSV |

## 2.6 总结

分色的本质是**把“颜色”变成二值掩膜**。
红色两段、蓝/黄各一段，再用形态学清理。后面所有轮廓都来自这张掩膜，
所以掩膜质量决定上限。

---

# 第三讲：轮廓、矩、minAreaRect

## 3.1 代码位置

```cpp
std::vector<std::vector<cv::Point>> contours;
cv::findContours(seg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area < min_area_) continue;

    cv::RotatedRect rr = cv::minAreaRect(contours[i]);
    cv::Moments m = cv::moments(contours[i]);
    cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
}
```

## 3.2 findContours 两个参数

- `RETR_EXTERNAL`：只取最外层轮廓。灯条是实心亮块，外层就够；
  用 `RETR_TREE` 会多出内孔，增加配对负担。
- `CHAIN_APPROX_SIMPLE`：只保留拐点，压缩水平/垂直/对角段。
  比 `CHAIN_APPROX_NONE` 省内存，且 `contourArea` 仍然准确。

## 3.3 contourArea 与 minAreaRect 的区别

- `contourArea`：轮廓多边形面积，受噪声影响，但能快速筛大小；
- `minAreaRect`：最小外接**旋转矩形**，给中心、宽高、角度。

为什么不用 `boundingRect`？
倾斜灯条的轴对齐框会包含大量背景，长宽比失真；
`minAreaRect` 跟着灯条转，长宽比才稳定。

## 3.4 矩（moments）

```
m00 = 面积
m10 = Σ x
m01 = Σ y
质心 = (m10/m00, m01/m00)
```
质心比 `minAreaRect.center` 更抗遮挡：灯条缺一角时，质心偏移小。

## 3.5 角度归一化

`RotatedRect.angle` 范围不直观（OpenCV 版本差异），常见处理：
```cpp
float ang = rr.angle;
if (ang > 45) ang -= 90;
```
把灯条统一到 `[-45, 45]` 或 `[-90, 90]`，后续比较角度差才一致。

## 3.6 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| `m00==0` | 除零/NaN | 先判断 `m.m00 > 1e-6` |
| 轮廓太多 | CPU 高 | 先开运算，再 `min_area` |
| `RETR_TREE` | 内孔重复 | 用 `RETR_EXTERNAL` |
| 角度未归一 | 配对角度差乱 | 归一化到同一区间 |

## 3.7 总结

轮廓阶段的目标不是“识别”，而是**把亮块变成候选灯条**：
面积筛掉噪点，旋转矩形给几何，矩给稳定中心，角度归一化给配对。

---

# 第四讲：灯条筛选与断笔合并

## 4.1 代码位置

```cpp
// 面积、长宽比、亮度
if (area < min_area_) continue;
float ar = std::max(rr.size.width, rr.size.height) /
           std::min(rr.size.width, rr.size.height);
if (ar < 2.0 || ar > 12.0) continue;
if (mean_v < bar_v_min_) continue;

// 断笔合并：两段灯条共线、间距小
if (std::abs(a.tilt - b.tilt) < merge_angle &&
    dist(a.center, b.center) < merge_dist) { ... }
```

## 4.2 为什么需要“断笔合并”

真实灯条可能被：
- 装甲板上的黑条/螺丝遮挡；
- 过曝导致中间断开；
- 运动模糊。

于是一根灯条被检测成两段。如果不合并：
- 配对时把两段当成两根灯条 → 配出错误的板；
- 或者数量不够 → 漏检。

合并条件通常是：
1. 角度接近（共线）；
2. 中心距离小于阈值；
3. 合并后的长宽比仍像灯条。

## 4.3 亮度筛选

```cpp
cv::Mat roi = v_chan(rr.boundingRect());
double mean_v = cv::mean(roi)[0];
```
- 灯条是主动发光，V 高；
- 白色装甲板、地面反光也可能 V 高，所以还要靠长宽比/颜色。

`bar_v_min_` 太低会引入反光，太高会丢暗处灯条。

## 4.4 共面/几何过滤

在多灯条场景，可能出现“看起来像一对，但不在同一平面”的组合。
用：
- 两灯条高度差；
- 对角线差；
- 四点构成的四边形是否凸；
- 与历史平面的距离。

目的是在配对前减少错误候选。

## 4.5 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 合并不设角度 | 把相邻两根灯条并了 | 角度 < 5° |
| 亮度阈值过高 | 暗处漏 | 降 `bar_v_min` |
| 只按面积筛 | 大反光块进候选 | 加长宽比 |
| 合并后不重算角度 | 配对角度差大 | 合并后重算 tilt |

## 4.6 总结

灯条阶段是“提纯”：把轮廓变成**数量可控、几何可信**的灯条。
合并是为了对抗遮挡/过曝，几何过滤是为了对抗反光。

---

# 第五讲：灯条配对与大小板判定

## 5.1 代码位置

```cpp
// 2. 配对
double dx = std::abs(l.center.x - r.center.x);
double dy = std::abs(l.center.y - r.center.y);
double ddeg = std::abs(l.tilt - r.tilt);
if (dx < min_dist_x_) continue;
if (dx > max_dist_x_) continue;
if (dy > pair_dy_ratio_ * std::min(l.height, r.height)) continue;
if (ddeg > max_angle_diff_) continue;
```

```cpp
// 大小板：透视修正间距 / 灯条长
double ratio = corrected_dx / bar_len;
bool is_large = (ratio > ratio_s_max_);
```

## 5.2 为什么要“透视修正”

近大远小：同一块板，远时两灯条像素间距变小。
直接用像素 `dx` 判大小板会错。

做法：用灯条长度 `bar_len`（像素）做尺度归一化：
```
ratio = dx / bar_len
```
- 小板：`230mm/60mm ≈ 3.8`？
  实际是“灯条中心距/灯条长”，小板的物理比值约 2.25，大板约 3.8；
- 代码用 `ratio_s_max=3.0` 分界。

如果相机倾斜，还要按 `cos(倾角)` 修正 `dx`。

## 5.3 配对约束的物理意义

| 约束 | 物理意义 |
|---|---|
| `dx ≥ min_dist_x` | 太近是同一根断笔，不是一对 |
| `dx ≤ max_dist_x` | 太远不是同一块板 |
| `dy ≤ pair_dy_ratio * min(h)` | 两灯条高度应接近 |
| `ddeg ≤ max_angle_diff` | 两灯条应平行 |

## 5.4 几何否决

配对后还会检查：
- 对角线差 `|d1-d2| / max(d1,d2) < quad_diag_tol`；
- 对边比 `min/max > quad_align_min`；
- 四点凸性；
- 与历史平面的距离。

这些是“防幽灵”的：画面里两根不相干灯条可能碰巧满足距离/角度，
但构成不了合理的装甲板四边形。

## 5.5 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| `dx` 不归一化 | 远近大小板误判 | 用 `dx/bar_len` |
| 只判角度 | 斜板配错 | 加对角线/对边比 |
| 阈值太松 | 幽灵框 | 收紧 `quad_*` |
| 阈值太紧 | 漏检 | 先保 recall，再谈精度 |

## 5.6 总结

配对是把“灯条集合”变成“装甲板候选”的组合优化。
先距离/角度粗筛，再几何否决，最后用 ratio 判大小板。

---

# 第六讲：四点、物体点与 solvePnP

## 6.1 代码位置

```cpp
std::vector<cv::Point2f> img_pts = {lt, rt, rb, lb};  // TL,TR,BR,BL
std::vector<cv::Point3f> obj_pts = {
    {-Wb/2, -bH/2, 0}, {Wb/2, -bH/2, 0},
    {Wb/2,  bH/2, 0}, {-Wb/2, bH/2, 0}};

cv::Mat rvec, tvec;
bool ok = cv::solvePnP(obj_pts, img_pts, K_, D_, rvec, tvec, false,
                       cv::SOLVEPNP_IPPE);
if (!ok) ok = cv::solvePnP(obj_pts, img_pts, K_, D_, rvec, tvec, false,
                           cv::SOLVEPNP_ITERATIVE);
```

## 6.2 四个点到底取哪里

本工程取：
- 左灯条的**左边缘**上下角点；
- 右灯条的**右边缘**上下角点。

顺序固定 **TL → TR → BR → BL**（左上、右上、右下、左下，顺时针）。

为什么不用灯条中心？
- 灯条中心构成的是“窄矩形”，高度=灯条长；
- 真实的装甲板本体比灯条高（有上下边框），
  用灯条长当板高会让 `z` 偏大。

所以代码把四边形高度按 `Wb×bH` 拉伸到板体尺寸。

## 6.3 物体点为什么是 `±Wb/2, ±bH/2`

装甲板坐标系定义：
- 原点：板中心；
- x：向右；
- y：向下；
- z：垂直板面向外（右手系）。

四个角点就是：
```
(-Wb/2, -bH/2, 0)  TL
( Wb/2, -bH/2, 0)  TR
( Wb/2,  bH/2, 0)  BR
(-Wb/2,  bH/2, 0)  BL
```
单位米：
- 大板 `0.230 × 0.060`
- 小板 `0.135 × 0.060`

**单位必须与 tvec 输出一致**：物体点用米，tvec 就是米；
用毫米，tvec 就是毫米，发布前要换算。

## 6.4 IPPE 与 ITERATIVE

- `SOLVEPNP_IPPE`：针对**平面目标**的解析解，快、稳，适合装甲板；
- `SOLVEPNP_ITERATIVE`：通用迭代，初值差时可能局部最优；
- 代码先 IPPE，失败再 ITERATIVE。

还有 `IPPE_SQUARE`（正方形）、`EPNP`、`SQPNP` 等，平面四点首选 IPPE。

## 6.5 重投影误差

求解后应检查：
```cpp
std::vector<cv::Point2f> proj;
cv::projectPoints(obj_pts, rvec, tvec, K_, D_, proj);
double err = 0;
for (int i=0;i<4;++i) err += cv::norm(proj[i]-img_pts[i]);
err /= 4;
if (err > 10) continue;   // 像素
```
误差大说明四点顺序错/遮挡/几何不对。

## 6.6 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 四点顺序错 | z 为负/乱跳 | 固定 TL,TR,BR,BL |
| 物体点用灯条尺寸 | z 偏大 | 用板体 Wb×bH |
| 单位混用 | z 差 1000 倍 | 全链路米 |
| 不检查 z | 镜像解 | `z_min/z_max` + 重投影 |
| `D` 为空 | 畸变未校正 | 从 camera_info 读 |

## 6.7 总结

PnP 的输入是“四对点 + 内参 + 物体尺寸”，输出是 `rvec/tvec`。
四点顺序、物体尺寸、单位，是三处最容易错的地方。

---

# 第七讲：TF 广播（rvec → 四元数）

## 7.1 代码位置

```cpp
geometry_msgs::TransformStamped ts;
ts.header.stamp = msg->header.stamp;
ts.header.frame_id = camera_frame_;
ts.child_frame_id = "armor_v3";
ts.transform.translation.x = tvec.at<double>(0);
ts.transform.translation.y = tvec.at<double>(1);
ts.transform.translation.z = tvec.at<double>(2);

cv::Mat R;
cv::Rodrigues(rvec, R);
tf2::Matrix3x3 m(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                 R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                 R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
tf2::Quaternion q;
m.getRotation(q);
ts.transform.rotation = tf2::toMsg(q);
tf_broadcaster_.sendTransform(ts);
```

## 7.2 为什么是 rvec

`solvePnP` 输出旋转向量 `rvec`：
- 方向 = 旋转轴；
- 模长 = 旋转角（弧度）。

它不是欧拉角，不能直接填 TF。要：
```
rvec --Rodrigues--> R (3×3) --Matrix3x3--> 四元数
```

## 7.3 父子坐标系

- 父：`camera_info.header.frame_id`（`hk_camera_ca`）；
- 子：`armor_v3`；
- 时间戳：**图像时间**，不是 `now()`。

如果父 frame 在 TF 树里不存在，RViz 会报 “No transform from [armor_v3]”。
解决：加静态 TF：
```xml
<node pkg="tf2_ros" type="static_transform_publisher"
      args="0 0 0 0 0 0 camera2_optical_frame hk_camera_ca" />
```

## 7.4 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 用 now() | 时间对不上，TF 抖动 | 用 `msg->header.stamp` |
| 父 frame 写错 | RViz 不显示 | 与 camera_info 一致 |
| 四元数未归一化 | TF 报错 | `q.normalize()` |
| 每帧发多个同名 child | TF 冲突 | 只发一个/加 id |

## 7.5 总结

TF 是把“OpenCV 的位姿”翻译成“ROS 的坐标变换”。
三件事：平移= tvec，旋转= rvec→四元数，时间= 图像时间。

---

# 第八讲：消息、id 与发布

## 8.1 代码位置

```cpp
ft_msgs::Detections arr;
arr.header = msg->header;
arr.header.frame_id = camera_frame_;

ft_msgs::Detection d;
d.header = arr.header;
d.id = tg.id;
d.number = tg.number;
d.is_red = tg.is_red;
d.is_large_armor = tg.is_large;
d.pose.position.x = tg.t[0];
...
arr.detections.push_back(d);
detection_array_pub_.publish(arr);
```

## 8.2 为什么用数组

一帧可能看到多块板（多车/多面）。用 `Detections` 数组：
- 每帧一个消息；
- 每个元素一个 `Detection`；
- `id` 只要求**帧内唯一**，不要求跨帧跟踪。

跨帧 ID 由跟踪节点负责，检测节点不要假装自己能跨帧。

## 8.3 时间戳一致性

所有 `header.stamp` 都来自 `msg->header.stamp`：
- 检测数组；
- 每个 Detection；
- TF；
- debug 图。

这样下游做时间同步/插值时不会错位。

## 8.4 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 用 now() | 与图像不同步 | 统一图像时间 |
| id 重复 | 下游混淆 | 帧内自增 |
| 空数组也发布 | 下游误判 | 可以发空数组表示“本帧无目标” |
| 字段没填 | 默认 0/NaN | 逐字段检查 |

## 8.5 总结

消息是“检测结果的合同”：字段、单位、时间戳、frame 都要固定。
合同错了，跟踪和 RViz 都会错。

---

# 第九讲：目标选择、锁定与平滑

## 9.1 代码位置

```cpp
if (lock_left_ > 0) {
    double bestd = 1e18; int cand = -1;
    for (size_t k = 0; k < targets.size(); ++k) {
        if (std::abs(targets[k].ratio - lock_ratio_) >
            lock_ratio_tol_ * lock_ratio_) continue;
        double d = cv::norm(targets[k].center2d - lock_pred_);
        if (d < bestd) { bestd = d; cand = (int)k; }
    }
    if (cand >= 0 && bestd < lock_pred_gate_px_) {
        best = cand; locked = true; lock_miss_ = 0;
    } else if (++lock_miss_ > 7) { lock_left_ = 0; }
}
if (!locked) { /* 最正对兜底 */ }
```

## 9.2 为什么需要锁定

一帧有多块板时，如果每帧都“选最正对”，会在两块板之间跳：
- 跟踪器看到位置突变，关联失败；
- RViz 里框闪烁。

锁定逻辑：
1. 记住上一帧选中板的位置/比例；
2. 预测它这一帧大概在哪（`lock_pred_`）；
3. 在预测附近找比例相近的候选；
4. 连续 7 帧找不到才解锁。

这相当于在检测层做一次“轻量跟踪”。

## 9.3 50px / 10% 怎么来的

- 50px：帧率 20–30Hz、目标速度 <6m/s，单帧位移约 20–40px，50 留余量；
- 10%：大小板比例约 2.25/3.8，10% 能区分，又不会因透视变化丢锁。

参数化后可在 YAML 调，不要写死。

## 9.4 平滑

```cpp
pushSmooth(tvec, yaw, avg_t, avg_yaw);
```
- 位置：滑动平均；
- yaw：角度平均要处理 ±π 跳变（用 `atan2(sin,cos)`）；
- z：中值滤波，抗偶发跳变。

## 9.5 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 锁太死 | 换板后不跟 | 调 `lock_miss_`/门限 |
| 锁太松 | 跳板 | 收紧 `lock_gate_px` |
| 角度平均直接加 | ±π 处跳 | 用向量平均 |
| 平滑窗口太大 | 延迟 | 窗口 5–10 帧 |

## 9.6 总结

选择解决“选谁”，锁定解决“别乱换”，平滑解决“别抖”。
三者都在检测层，跟踪层只负责滤波和预测。

---

# 第十讲：cv::KalmanFilter 彻底讲清楚

## 10.1 代码位置

```cpp
// ft_tracker_node.cpp
kf_ = cv::KalmanFilter(6, 3, 0, CV_32F);

kf_.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);
kf_.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
for (int i = 0; i < 3; ++i) kf_.measurementMatrix.at<float>(i, i) = 1.0f;
cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(q_pos_));
cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(r_pos_));
cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));
```

```cpp
double dt = (stamp - last_t_).toSec();
kf_.transitionMatrix.at<float>(0,3) = dt;
kf_.transitionMatrix.at<float>(1,4) = dt;
kf_.transitionMatrix.at<float>(2,5) = dt;
kf_.predict();
kf_.correct(meas);
```

## 10.2 它是什么

OpenCV 的线性卡尔曼滤波器。你告诉它：
- 状态怎么演化（A）；
- 观测怎么来（H）；
- 你多不信运动模型（Q）；
- 你多不信观测（R）。

它每帧做两件事：
1. `predict()`：用 A 把状态往前推，同时放大协方差；
2. `correct(z)`：用观测 z 修正，算卡尔曼增益 K。

## 10.3 状态向量的选择

```
state = [x, y, z, vx, vy, vz]ᵀ
measurement = [x, y, z]ᵀ
```

为什么把速度放进状态？
- 只有位置观测时，速度是“隐变量”；
- 放进状态后，滤波器能估计速度；
- 有了速度，才能算提前量 `position + v*T`。

如果只估位置，预测只能靠“上一帧位置”，没有速度项。

## 10.4 转移矩阵 A（匀速模型）

```
x_t  = x_{t-1} + vx_{t-1}·dt
y_t  = y_{t-1} + vy_{t-1}·dt
z_t  = z_{t-1} + vz_{t-1}·dt
vx_t = vx_{t-1}
vy_t = vy_{t-1}
vz_t = vz_{t-1}
```
写成矩阵就是对角 + 右上 `dt`。

**dt 必须是两帧图像时间戳相减**：
- 图像有曝光/传输延迟；
- 用 `ros::Time::now()` 会把延迟算进 dt，速度估计偏大/抖动；
- 回放时 `use_sim_time` 下 now() 与 bag 时间也可能不同步。

## 10.5 观测矩阵 H

```
H = [I3  0]   （3×6）
```
表示“只能看到位置，看不到速度”。

## 10.6 Q 与 R 的物理意义

- `Q`（processNoiseCov）：运动模型有多不可信。
  - 小：认为目标严格匀速 → 平滑但转弯滞后；
  - 大：认为目标可能突然机动 → 跟得快但抖。
- `R`（measurementNoiseCov）：观测有多不可信。
  - 小：完全信 PnP → 抖；
  - 大：完全信预测 → 跟丢。

本工程：
```
q_pos=0.1, q_vel=0.5, r_pos=0.05
```
位置噪声小、速度噪声大：位置更信观测，速度允许快速变化。

## 10.7 predict / correct 的调用时机

```
每帧：
  kf.predict()        // 不管有没有观测，先预测
  if (有观测) kf.correct(z)   // 有观测才修正
  else         tracking=false // 纯预测
```

注意：`predict()` 每帧只能调一次；重复调会把状态推多次。

## 10.8 提前量

```cpp
pred = statePost[0..2] + statePost[3..5] * T;
```
- `T` 是 YAML 参数（默认 0.1s）；
- 预测太远会“冲到目标前面”，太近没有提前量；
- 加限幅 `predict_cap_m` 防止速度异常时飞出去。

## 10.9 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| dt 用 now() | 速度抖/发散 | 图像时间戳 |
| 忘记 setIdentity | Q/R 为 0，滤波器不更新 | 初始化 |
| 重复 predict | 状态被推多次 | 每帧一次 |
| 单位混用 | 速度差 1000 倍 | 全米 |
| 丢检还 correct | 用旧观测 | 丢检只 predict |

## 10.10 总结

Kalman = “运动模型 + 观测修正”。
A 决定怎么动，H 决定能看到什么，Q/R 决定信谁。
dt 和单位是两条生死线。

---

# 第十一讲：丢检、纯预测与 Marker

## 11.1 代码位置

```cpp
void predictOnly(ros::Time now) {
    if (!is_tracking_) return;
    double gap = (now - last_det_time_).toSec();
    if (gap > lost_time_thresh_) {
        publishTrack(last_image_stamp_, false);
        is_tracking_ = false;
        return;
    }
    kf_.predict();
    publishTrack(last_image_stamp_, false);
    publishMarker(last_image_stamp_);
}
```

## 11.2 三种状态

| 状态 | 条件 | tracking |
|---|---|---|
| 正常 | 本帧有观测 | true |
| 纯预测 | 丢检但 < 0.5s | false |
| 丢失 | > 0.5s | false，删除轨迹 |

## 11.3 为什么纯预测也要发

下游（云台/决策）需要知道“目标大概在哪”，即使这一帧没看到。
发 `tracking=false` 表示“这是预测，不是观测”。

## 11.4 时间戳

纯预测没有新图像，但消息仍要用**最近一帧图像时间戳**：
- 用 now() 会让下游以为有新的观测时间；
- 用最近图像时间，语义是“基于这帧的预测”。

## 11.5 Marker

```cpp
mk.type = visualization_msgs::Marker::SPHERE;
mk.pose.position = pred;
mk.scale.x = mk.scale.y = mk.scale.z = 0.08;
```
- 红色球 = 预测位置；
- 可再加一条线连接当前位置与预测位置，直观看到“提前量”。

## 11.6 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 丢检还 correct | 轨迹被旧值拖住 | 只 predict |
| 不删轨迹 | 目标消失后还发 | 超时重置 |
| marker 无 frame | RViz 不显示 | 填 frame_id |
| marker 太多 | RViz 卡 | 用同一个 id 覆盖 |

## 11.7 总结

丢检处理是“用模型兜底”，不是“假装看到”。
`tracking` 字段是给下游的诚实信号。

---

# 第十二讲：旋转目标建模（圆周模型）

## 12.1 代码位置

```cpp
// 圆拟合
if (fitCircle(hist_, c, R)) { center_ = c; R_ = R; has_center_ = true; }

// 相位
double phase = std::atan2(z - center_.y, x - center_.x);
double dph = std::atan2(std::sin(phase - phase_), std::cos(phase - phase_));
double w = dph / dt;
omega_ = 0.8 * omega_ + 0.2 * w;

// 预测
double cx = center_.x + vx * T_;
double cz = center_.y + vz * T_;
double th = phase + omega_ * T_;
pred_.x = cx + R_ * std::cos(th);
pred_.z = cz + R_ * std::sin(th);
pred_.y = y + vy * T_;
```

## 12.2 模型假设

- 车体在水平面（x–z）绕自身中心旋转；
- 装甲板中心到车体中心距离 R 近似不变；
- 旋转角速度 ω 在短时间内近似恒定；
- 车体本身可匀速平移。

不满足时（正对、遮挡、R 突变、急停）会失效，要写进 README。

## 12.3 为什么要圆拟合

单帧只能看到一块板，无法知道车体中心。
多帧的装甲中心点落在同一个圆上：
```
(x-a)² + (z-b)² = R²
```
展开：
```
2a·x + 2b·z + c0 = x² + z²
c0 = R² - a² - b²
```
这是线性方程，用最小二乘（Kasa）解 `a,b,c0`，再算 R。

## 12.4 相位与角速度

```
θ = atan2(z-cz, x-cx)
```
- 相位是“装甲板在圆上的角度”；
- 跨 2π 时直接相减会从 +π 跳到 -π，所以用：
  ```
  dθ = atan2(sin(θ-θ_prev), cos(θ-θ_prev))
  ```
- `ω = dθ/dt`，再做低通 `0.8*old + 0.2*new`。

符号：相机系下，正 Δθ 对应逆时针/左转，负对应右转。
如果实际相反，改符号或改坐标定义，并在 README 写清。

## 12.5 预测

```
center' = center + v*T          // 车体先直线走
θ' = θ + ω*T                    // 相位再转
armor' = center' + R*[cosθ', sinθ']
```
先直线后旋转，顺序不能反：
- 先转再平移，等价于绕旧中心转，位置会偏；
- 正确是“车体中心先到新位置，装甲板在新中心上转”。

## 12.6 多板选择

```cpp
if (has_pred_) {
    // 选离预测最近且在门限内的
} else {
    // 首次选最靠近相机轴的
}
```
不能把帧内 `id` 当跨帧 ID：
- 检测的 id 每帧重排；
- 关联要靠“预测位置 + 门限”。

## 12.7 断帧

```
gap < lost_time_thresh:
    按 gap 继续推算（center + v*gap, θ+ω*gap）
gap ≥ lost_time_thresh:
    tracking=false, valid=false, 重置
```

## 12.8 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 点数太少就拟合 | 中心乱飞 | ≥8 点、跨度 ≥0.3s |
| R 不限制 | 拟合出巨大圆 | R∈[0.1,2] |
| 相位不归一 | ω 爆跳 | atan2(sin,cos) |
| 先转后平移 | 预测偏 | 先平移后转 |
| 正对时相位退化 | ω 乱 | 正对片段标为失效 |

## 12.9 总结

旋转模型 = 圆拟合（中心/R）+ 相位/角速度 + “先直线后旋转”的预测。
它的价值在“提前量”，不是“替代检测”。

---

# 第十三讲：YOLO 结构（为什么是 1×21×6300）

## 13.1 代码位置

模型：`armor.onnx` / 转换后的 `armor_allfp32.onnx`
输入：`1×3×480×640`
输出：`1×21×6300`

## 13.2 三段式

```
Backbone（提特征）
  Conv 3×3 s2 ×5，尺寸逐级减半
  3→16→32→64→128→256
  输出 P3 60×80、P4 30×40、P5 15×20

Neck（融尺度）
  自顶向下：15×20 → 30×40 → 60×80（上采样，把语义送给细网格）
  自底向上：60×80 → 30×40 → 15×20（下采样，把位置送给粗网格）
  Concat + C2f

Head（出结果）
  P3/P4/P5 各预测一遍
  颜色 4 + 编号 9 + 关键点 8 = 21
  拼起来：21 × (4800+1200+300) = 21×6300
```

## 13.3 6300 怎么来的

| 档 | 网格 | 数量 | 步长 |
|---|---|---|---|
| P3 | 60×80 | 4800 | 8 |
| P4 | 30×40 | 1200 | 16 |
| P5 | 15×20 | 300 | 32 |
| 合计 | | 6300 | |

每一列对应输入图上的一个“格子中心”：
```
j = i - O
c = j % W
r = j / W
x = c*s + s/2
y = r*s + s/2
```
例：`i=3150`（P3，O=0，W=80，s=8）：
```
c=30, r=39
x=30*8+4=244
y=39*8+4=316
```

## 13.4 21 行

| 行 | 含义 | 处理 |
|---|---|---|
| 0–3 | 颜色（蓝/红/白/紫） | 取 max，过阈值 |
| 4–12 | 编号（0 哨兵、1–4 兵种、6 前哨、7–8 基地） | 取 max，过阈值 |
| 13–20 | 四点 x/y | 两两成点 |

注意：这不是标准的 `[x,y,w,h]` 检测框，而是 **pose/关键点**输出。
用普通 YOLO 的框解码会完全错。

## 13.5 训练/导出（PPT 摘要）

1. labelme 点四个角，顺序 **LT→RT→RB→LB**；
2. 标签一行 13 数：`cls cx cy w h x1 y1 ... x4 y4`（归一化）；
3. `data.yaml`：`kpt_shape:[4,2]`、`flip_idx:[1,0,3,2]`；
4. `yolo pose train ... imgsz=640`；
5. `model.export(format="onnx", imgsz=[480,640], opset=12)`；
6. 核对输入/输出形状。

## 13.6 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 用框解码 | 位置全错 | 按 keypoint 解码 |
| 归一化坐标未还原 | 点在左上角 | `x*W, y*H` |
| 点序不一致 | 框交叉 | 固定 LT/RT/RB/LB |
| 输入尺寸不一致 | 精度崩 | 训练/导出/部署同一尺寸 |

---

# 第十四讲：NN 预处理（letterbox）与还原

## 14.1 代码位置

```cpp
// detector_interface.hpp
float scale = std::min(IN_W / (float)bgr.cols, IN_H / (float)bgr.rows);
int nw = std::round(bgr.cols * scale);
int nh = std::round(bgr.rows * scale);
int pad_x = (IN_W - nw) / 2, pad_y = (IN_H - nh) / 2;

cv::Mat canvas(IN_H, IN_W, CV_8UC3, cv::Scalar(124,124,124));
cv::resize(bgr, canvas(cv::Rect(pad_x, pad_y, nw, nh)), cv::Size(nw, nh));
```

## 14.2 为什么不能直接 resize

直接 `resize` 到 640×480 会把 4:3 的图拉成 4:3？本数据是 1440×1080=4:3，
目标 640×480=4:3，正好不变形。但如果输入是 16:9，直接拉会改变长宽比，
装甲板变扁，关键点位置全错。

letterbox 的做法：
1. 等比缩放 `scale = min(640/W, 480/H)`；
2. 缩放后居中粘贴；
3. 空白填 124（灰色，接近模型训练时的 padding）。

## 14.3 还原公式

```
x_orig = (x_net - pad_x) / scale
y_orig = (y_net - pad_y) / scale
```
**必须用当帧的 scale/pad_x/pad_y**，不能用全局变量/上一帧的。
多路视频/分辨率切换时，这是最常见错误。

## 14.4 颜色与归一化

- OpenCV 读进来是 BGR；
- 模型训练用 RGB；
- 所以 `BGR→RGB`，再 `/255`；
- 布局从 HWC 变 CHW（NCHW）。

顺序：`resize → RGB → /255 → NCHW`。
少一步，精度都会掉。

## 14.5 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 直接 resize | 点偏 | letterbox |
| 忘记减 pad | 点整体偏移 | 减 pad 再除 scale |
| BGR 当 RGB | 颜色通道错，分数低 | `cvtColor` 或手动换序 |
| 不除 255 | 输出乱 | `/255.f` |
| 用错 scale | 远距点缩水 | 每帧保存 |

---

# 第十五讲：ORT 推理与解码

## 15.1 代码位置

```cpp
Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "armor_nn");
Ort::SessionOptions opt;
opt.SetIntraOpNumThreads(1);
Ort::Session session(env, model_path.c_str(), opt);

auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
Ort::Value tensor = Ort::Value::CreateTensor<float>(
    mem, input.data(), input.size(), shape.data(), 4);
auto outputs = session.Run(Ort::RunOptions{nullptr},
                           in_names, &tensor, 1, out_names, 1);
```

## 15.2 会话只建一次

- `Ort::Env` / `Ort::Session` 构造很贵（加载模型、图优化）；
- 必须放在构造函数/初始化，不能每帧建；
- 每帧只 `Run()`。

## 15.3 输入张量

```
shape = {1, 3, 480, 640}
data  = float[3*480*640]，NCHW
```
`Ort::Value::CreateTensor` 不拷贝数据？它引用 `input.data()`，
所以 `input` 必须在 `Run()` 期间有效（局部 vector 可以，别提前释放）。

## 15.4 解码

```cpp
float* o = outputs[0].GetTensorMutableData<float>();
int cols = 6300;
for (int i=0;i<cols;++i) {
    // 颜色
    int color=0; float cs=o[0*cols+i];
    if (o[1*cols+i] > cs) { color=1; cs=o[1*cols+i]; }
    if (o[2*cols+i] > cs) { color=2; cs=o[2*cols+i]; }
    if (cs < color_thres) continue;

    // 编号
    int num=0; float ns=o[4*cols+i];
    for (int k=1;k<9;++k)
        if (o[(4+k)*cols+i] > ns) { ns=o[(4+k)*cols+i]; num=k; }
    if (ns < number_thres) continue;

    // 角点
    cv::Point2f p[4];
    for (int k=0;k<4;++k) {
        float x=o[(13+2*k)*cols+i], y=o[(14+2*k)*cols+i];
        p[k] = {(x-pad_x)/scale, (y-pad_y)/scale};
    }
    // 顺序
    a.corners[0]=p[0]; a.corners[1]=p[3];
    a.corners[2]=p[2]; a.corners[3]=p[1];
}
```

## 15.5 置信度筛选与 NMS

- 颜色阈值 `color_thres=0.5`：低于此认为“没有颜色”；
- 编号阈值 `number_thres=0.5`：低于此认为“编号不可信”；
- NMS `nms_thres=0.45`：同一块板在相邻格子会出多个候选，按 IoU 去重。

NMS 的输入是外接矩形，不是四点；四点仅用于 PnP。

## 15.6 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 会话每帧重建 | 慢 10 倍 | 构造一次 |
| 张量数据提前释放 | 崩溃/乱码 | 保证生命周期 |
| 行列搞反 | 分数乱 | 确认 `[1,21,6300]` |
| NMS 阈值过大 | 重复框 | 降 0.45 |
| 阈值过低 | 误检 | 升 0.5 |

## 15.7 总结

NN 部署 = 预处理（letterbox/RGB/NCHW）+ 推理（一次会话）+ 解码（21 行）+ 去重（NMS）。
任何一步与训练不一致，精度都会掉。

---

# 第十六讲：数字模板识别（传统补充）

## 16.1 代码位置

```cpp
void loadDigitTemplates() {
    std::string dir = ros::package::getPath("final_text") + "/models/";
    for (int i = 1; i <= 5; ++i) {
        cv::Mat img = cv::imread(dir + "digit_" + std::to_string(i) + ".png",
                                 cv::IMREAD_GRAYSCALE);
        if (img.empty()) continue;
        cv::Mat bin;
        cv::threshold(img, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        if (cv::mean(bin)[0] > 127) cv::bitwise_not(bin, bin);
        cv::resize(bin, bin, cv::Size(20, 30));
        digit_templates_.push_back(bin);
        digit_labels_.push_back(i);
    }
}
```

## 16.2 原理

`matchTemplate` 做归一化互相关（`TM_CCOEFF_NORMED`）：
- 结果在 [-1,1]，越接近 1 越像；
- 取最高分，超过阈值就判为该编号。

## 16.3 关键处理

1. 灰度；
2. Otsu 自动阈值（比固定 127 稳）；
3. 自动反色，统一“白字黑底”；
4. 统一 20×30。

## 16.4 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 模板与 ROI 明暗相反 | 全判错 | 自动反色 |
| 透视变形 | 匹配低 | 先 `warpPerspective` |
| 模板字体不同 | 匹配低 | 用同一字体采样 |
| 没模板 | number=0 | 正常，不崩 |

## 16.5 总结

模板匹配适合“固定字体、正对”的场景；
倾斜大/光照复杂时，NN 的编号头更稳。

---

# 第十七讲：dynamic_reconfigure 与 YAML

## 17.1 代码位置

```cpp
dr_srv_.reset(new dynamic_reconfigure::Server<final_text::VisionConfig>(pnh_));
dr_srv_->setCallback(boost::bind(&VisionNodeV3::drCallback, this, _1, _2));

void drCallback(final_text::VisionConfig& cfg, uint32_t) {
    pnh_.setParam("blue_h_min", cfg.blue_h_min);
    blue_h_min_ = cfg.blue_h_min;
}
```

## 17.2 两套参数的关系

- YAML：启动默认值；
- cfg：在线可调，回调里同步到成员 + `setParam`；
- 两边默认值要一致，否则“launch 后”和“拖滑块后”行为不同。

## 17.3 为什么需要

调参时不想反复重启：
- 拖 `blue_h_min` 看掩膜变化；
- 拖 `min_area` 看噪点；
- 拖 `z_min/z_max` 看距离过滤。

## 17.4 坑点

| 坑 | 现象 | 处理 |
|---|---|---|
| 只 setParam 不改成员 | 滑块动了没效果 | 同时赋值成员 |
| cfg 默认值与 YAML 不同 | 启动就漂 | 对齐 |
| 回调里做重活 | 卡 | 只赋值 |

## 17.5 总结

YAML 管“启动”，cfg 管“在线”；两者必须同步到同一组成员变量。

---

# 第十八讲：怎么读这份代码（讲解顺序）

## 18.1 调用链

```
main
 └─ ros::spin
     └─ imageCallback(msg)                     // 每帧
         ├─ 检查 is_ready_ / 尺寸
         ├─ cv_bridge::toCvShare
         ├─ Knee knee; 填 msg/bgr/canvas/stamp
         ├─ if (nn_enable_) processNN(...)      // NN 分支
         ├─ colorSegment(...)                   // 分色
         ├─ 高光守卫/掩膜
         ├─ findContours → bars
         ├─ 配对 → targets
         ├─ 发布 /ft/detection
         ├─ 选择/锁定/平滑
         ├─ TF
         └─ 画图 → /ft/debug/image
```

## 18.2 按“数据”读，不按“行”读

每一段只问：
1. 输入是哪个变量？
2. 输出写到哪个变量？
3. 失败/空的时候走哪条分支？

例如：
```
seg_mask → contours → bars → targets → detections
```
任何一个环节为空，后面都不该崩。

## 18.3 断点/日志位置

| 位置 | 打印 |
|---|---|
| 回调入口 | 图像尺寸、stamp |
| 分色后 | `countNonZero(seg_mask)` |
| 灯条后 | `bars.size()` |
| 配对后 | `targets.size()` |
| PnP 后 | `tvec`、重投影误差 |
| 选择后 | `best/locked` |
| 跟踪 | `dt`、`statePost` |

## 18.4 最小复现

- 固定一张图：`cv::imread` 后直接调回调逻辑；
- 固定一段 bag：`rosbag play -r 0.2` 慢放；
- 关掉 NN/旋转，只留主链路。

## 18.5 二分定位

```cpp
// 怀疑后半段
if (false) { /* 选择/跟踪 */ }
```
先确认“检测框”对，再查“选择”，最后查“跟踪”。
不要一上来改 Kalman。

---

# 第十九讲：测试与验证（讲解版）

## 19.1 启动顺序

```bash
# 1. master + bag
rosbag play bag/red_4.bag --clock -l

# 2. 检测 + 跟踪
roslaunch final_text test_vision.launch mode:=tradition

# 3. 看话题
rostopic hz /ft/detection /ft/track /ft/debug/image
```

## 19.2 每一步验证什么

| 步骤 | 命令 | 期望 |
|---|---|---|
| 图像 | `rostopic hz /hk_camera/image_raw/compressed` | 20–30Hz |
| 检测 | `rostopic echo -n1 /ft/detection` | 有 detections，z 合理 |
| 跟踪 | `rostopic echo -n1 /ft/track` | tracking/position_predict |
| TF | `rosrun tf tf_echo hk_camera_ca armor_v3` | 位姿连续 |
| marker | RViz 看 `/ft/debug/marker` | 预测点在前 |
| 调参 | `dynparam set` | 画面实时变 |

## 19.3 三工况

| 工况 | bag | 看什么 |
|---|---|---|
| 静止 | red_4 | 框稳定、z 不跳 |
| 匀速 | red_1&3 | 预测点领先 |
| 近距/遮挡 | blue_guard | 丢检后恢复 |

## 19.4 失败案例怎么讲

1. 截图/帧号；
2. 当时 `z`、框面积、亮度；
3. 归类：过曝/遮挡/远距/角度/颜色；
4. 说明“是检测错还是跟踪错”；
5. 给出可调参数或已知限制。

---

# 第二十讲：代码审查与诊断（答辩标准）

## 20.1 判断“整包生成”的信号

1. 提交历史只有一次 `final`；
2. 注释只写“定义变量”，没有为什么；
3. 没有 badcase 分支（过曝/遮挡/换板）；
4. 阈值全硬编码；
5. 异常处理只有 `catch(...) {}`；
6. 作者说不清某个参数为什么是 50；
7. README 与代码不一致。

## 20.2 逐行讲解四问

| 问题 | 示例 |
|---|---|
| 输入 | `seg_mask` |
| 输出 | `contours` |
| 失败 | 空 → 跳过 |
| 为什么 | 面积小于 20 是噪点 |

## 20.3 诊断流程

```
现象
 ├─ 无数据 → 话题/时间/驱动
 ├─ 有图无框 → 阈值/曝光/面积
 ├─ 有框错 → 四点/物体尺寸/单位
 ├─ 框跳 → 锁定/多目标
 ├─ 跟踪断 → 门限/丢检/Q-R
 └─ 预测偏 → T/速度/旋转模型
```

## 20.4 修改原则

- 一次只改一个参数；
- 改前记录原值；
- 改后跑同一段 bag；
- 变差就回退，不要“再调调看”。

---

# 第二十一讲：性能与工程化

## 21.1 耗时分布

| 阶段 | 典型耗时 |
|---|---|
| 解码/转换 | 1–3 ms |
| HSV + 形态学 | 2–5 ms |
| 轮廓/灯条 | 5–10 ms |
| 配对/PnP | 1–3 ms |
| NN 推理 | ~55 ms |
| 画图/发布 | 1–3 ms |

## 21.2 优化顺序

1. 降分辨率/ROI；
2. 减少轮廓数（先开运算/面积过滤）；
3. NN 用固定输入、单线程；
4. 不要每帧 clone 大图；
5. 发布用 compressed。

## 21.3 参数与配置

- 所有阈值进 YAML；
- 识别阈值进 cfg；
- 模型路径进 launch arg；
- 话题名进 YAML；
- 不写死绝对路径。

## 21.4 提交检查

- `catkin_make` 0；
- 三话题有数据；
- 无 imshow；
- README/ai_usage 更新；
- 视频/截图；
- tag。

---

# 第二十二讲：30 秒讲清整个系统

> 图像经 compressed 进 `imageCallback`；HSV 分色得到灯条掩膜；
> 轮廓转灯条，配对成装甲板；四点做 IPPE PnP 得 tvec/rvec；
> 相邻帧锁定 + 平滑后发 `/ft/detection` 和 TF；
> 跟踪节点用 6 状态 Kalman 滤波，丢检时 predict 并 `tracking=false`，
> 按 T 算 `position_predict`，RViz 画 marker；
> 进阶有 ORT C++ NN 和圆周旋转模型；参数全部 YAML/cfg 可调。

---

# 第二十三讲：坐标变换与针孔模型（把公式摊开）

## 23.1 四个坐标系

```
世界/车体 world
   │  R_wc, t_wc
相机 camera（x右, y下, z前）
   │  K
像素 pixel（u,v）
```

针孔模型：
```
[u]       [fx  0  cx] [X]
[v] = 1/Z [ 0 fy  cy] [Y]
[1]       [ 0  0   1] [Z]
```
展开：
```
u = fx * X/Z + cx
v = fy * Y/Z + cy
```
反过来（已知深度 Z）：
```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

## 23.2 为什么单目测距必须知道物体尺寸

单张图只有 `u,v`，未知 `X,Y,Z`；一个像素点对应一条射线，不是点。
PnP 能解出 Z，是因为你给了**物体点的真实尺寸**（Wb×bH）：
- 尺寸给大 → 解出的 Z 偏大；
- 尺寸给小 → Z 偏小。

所以 `armor_large_w/h` 必须和实际一致，且单位与 tvec 一致。

## 23.3 旋转的三种表示

| 表示 | 维度 | 用途 |
|---|---|---|
| 旋转矩阵 R | 9（正交约束） | 变换向量 |
| 旋转向量 rvec | 3（轴×角） | solvePnP 输出 |
| 四元数 q | 4（单位） | ROS TF/消息 |

转换：
```
rvec --Rodrigues--> R --Matrix3x3--> q
q --toRotationMatrix--> R --Rodrigues--> rvec
```

## 23.4 手算一个例子

设：
```
fx=fy=800, cx=720, cy=540
tvec=(0.1, -0.05, 3.2)
板中心在相机前方 3.2m
```
板中心投影：
```
u = 800*0.1/3.2 + 720 = 25 + 720 = 745
v = 800*(-0.05)/3.2 + 540 = -12.5 + 540 = 527.5
```
若板宽 0.23m，左右边缘 X=±0.115：
```
Δu = 800*0.115/3.2 ≈ 28.75 px
```
所以板上宽约 57.5 px。若实测只有 40px，说明 Z 估大了或尺寸设小了。

## 23.5 重投影误差的手算

四个点投影后与检测点差：
```
e = (1/4) Σ ||p_proj - p_det||
```
- <2px：很好；
- 2–5px：可接受；
- >10px：顺序/尺寸/畸变有问题。

---

# 第二十四讲：solvePnP 深入（DLT、IPPE、迭代）

## 24.1 PnP 问题定义

已知：
- n 个 3D 点 `P_i`（物体坐标）；
- 对应 2D 点 `p_i`（像素）；
- 内参 K、畸变 D。

求：`R, t`，使重投影最小：
```
min Σ ||π(K(R P_i + t)) - p_i||²
```

## 24.2 DLT（直接线性变换）

把投影写成齐次：
```
p_i × (K [R|t] P_i) = 0
```
每个点给 2 个独立方程，12 个未知量（3×4 矩阵），
至少 6 个点可解；4 个点时需用平面约束。

DLT 的缺点：
- 不解正交约束，R 可能不是正交矩阵；
- 对噪声敏感；
- 需要后处理（SVD 投影到 SO(3)）。

## 24.3 IPPE（Infinitesimal Plane-Based Pose Estimation）

针对**平面目标**：
- 4 个共面点；
- 解析解，先解单应 H，再分解出 R,t；
- 有 4 个解，用“点在相机前方”筛选；
- 快、稳，适合装甲板。

代码：
```cpp
cv::solvePnP(obj, img, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
```

## 24.4 ITERATIVE

以当前 `rvec/tvec` 为初值，LM 迭代最小化重投影误差。
- 初值好 → 收敛快；
- 初值差 → 局部最优；
- 代码在 IPPE 失败时用它兜底。

## 24.5 为什么装甲板用 IPPE

- 四点共面；
- 尺寸已知；
- 需要每帧快；
- IPPE 对平面四点有解析解，比迭代稳。

## 24.6 退化情况

- 四点共线 → 无解；
- 板正对相机（法线指向光轴）→ 深度方向弱约束，z 抖；
- 板太远（<20px）→ 像素噪声放大，z 跳；
- 遮挡导致点偏移 → 解偏。

对策：
- `quad_*` 几何否决；
- `z_min/z_max`；
- 锁定/平滑；
- 重投影误差过滤。

---

# 第二十五讲：卡尔曼滤波完整推导

## 25.1 线性系统

```
x_k = A x_{k-1} + w,   w ~ N(0, Q)
z_k = H x_k + v,       v ~ N(0, R)
```

## 25.2 预测

```
x̂_k⁻ = A x̂_{k-1}
P_k⁻ = A P_{k-1} Aᵀ + Q
```

## 25.3 更新

```
K_k = P_k⁻ Hᵀ (H P_k⁻ Hᵀ + R)⁻¹
x̂_k = x̂_k⁻ + K_k (z_k - H x̂_k⁻)
P_k = (I - K_k H) P_k⁻
```

## 25.4 直觉

- `K` 大：更信观测；
- `K` 小：更信预测；
- `Q` 大 → `P⁻` 大 → `K` 大 → 更信观测；
- `R` 大 → `K` 小 → 更信预测。

## 25.5 本工程的 A/H

```
A = [I  dt·I]
    [0   I  ]
H = [I  0]
```
状态 6 维，观测 3 维。

## 25.6 数值例子（一维）

```
x=0, v=1, dt=0.1
A: x' = x + v*dt = 0.1
预测后 P 增大
观测 z=0.12, R=0.01
K = P/(P+R)
x = 0.1 + K*(0.12-0.1)
```
若 `P=0.02, R=0.01`：
```
K = 0.02/0.03 = 0.667
x = 0.1 + 0.667*0.02 = 0.1133
```
观测与预测折中。

## 25.7 为什么 dt 错会发散

速度项 `v*dt`：
- dt 偏大 → 预测走太远 → 残差大 → 速度被拉偏；
- dt 抖动 → 速度估计抖动；
- 用 now() 时，网络/回放延迟让 dt 不稳定。

## 25.8 丢检时的协方差

只 predict 不 correct，P 会越来越大：
- 短期：预测仍可用；
- 长期：不确定度爆炸，必须删除轨迹。

这就是 `lost_time_thresh` 的数学依据。

## 25.9 Q/R 的调法

1. 先固定 R（PnP 噪声，0.05 左右）；
2. 调 Q：抖 → 减 Q；滞后 → 加 Q；
3. 用同一段 bag 对比；
4. 记录“改了哪个值、效果怎么变”。

## 25.10 与 Alpha-Beta 的关系

Alpha-Beta 是 Kalman 的稳态特例：
```
x += α (z - x)
v += β (z - x)/dt
```
Q/R 固定时，Kalman 增益会收敛到常数，等价于 α/β。
本工程用完整 Kalman，参数更直观（Q/R）。

---

# 第二十六讲：Kasa 圆拟合推导与手算

## 26.1 圆方程

```
(x-a)² + (z-b)² = R²
```
展开：
```
x² - 2ax + a² + z² - 2bz + b² = R²
```
令 `c0 = R² - a² - b²`：
```
2a·x + 2b·z + c0 = x² + z²
```
这是关于 `(a,b,c0)` 的**线性**方程。

## 26.2 最小二乘

对 n 个点，误差：
```
e_i = 2a x_i + 2b z_i + c0 - (x_i²+z_i²)
```
最小化 `Σ e_i²`，对 a,b,c0 求偏导=0，得法方程：

```
[ 2Σx    2Σz    n   ] [a ]   [ Σ(x²+z²) ]
[ 2Σx²   2Σxz   Σx  ] [b ] = [ Σ(x³+xz²)]
[ 2Σxz   2Σz²   Σz  ] [c0]   [ Σ(x³+z³)]
```
（第三行在代码里写作 `Σx³+Σz³`，注意与 b 行区分。）

解出 a,b,c0 后：
```
R = sqrt(c0 + a² + b²)
```

## 26.3 手算三点

三点：
```
(0,0), (2,0), (0,2)
```
圆心应为 `(1,1)`，R=√2。

代入：
```
Σx=2, Σz=2, n=3
Σx²=4, Σz²=4, Σxz=0
Σ(x²+z²)=0+4+4=8
Σ(x³+xz²)=0+8+0=8
Σ(x³+z³)=0+8+8=16
```
法方程：
```
[ 4   4   3 ] [a ]   [8 ]
[ 8   0   2 ] [b ] = [8 ]
[ 0   8   2 ] [c0]   [16]
```
解得 a=1,b=1,c0=-2 → R=√( -2+1+1)=0？不对，检查 c0 定义：
`c0 = R² - a² - b²` → `R² = c0 + a² + b² = -2+1+1=0`。

说明手算法方程右端/系数有一处笔误。正确 Kasa 法方程：
```
[ 2Σx   2Σz   n  ] [a ]   [ Σ(x²+z²) ]
[ 2Σx²  2Σxz  Σx ] [b ] = [ Σ(x³+xz²)]
[ 2Σxz  2Σz²  Σz ] [c0]   [ Σ(xz²+z³)]
```
第三行右端是 `Σ(xz²+z³)`，不是 `Σ(x³+z³)`。
代码里 b 向量第三项写的是 `Sx3+Sz3`，需要核对——这是一个**潜在 bug 点**。

> 结论：圆拟合公式第三行右端应为 `Σ(x z² + z³)`；
> 若代码写成 `Σ(x³+z³)`，对称数据下巧合，非对称数据会偏。
> 讲解时如实指出，答辩可加分。

## 26.4 数值稳定性

- 坐标先减去均值（`x-=mean, z-=mean`），避免大数相减；
- 点数 ≥8；
- R 限制 0.1–2m；
- 残差过大丢弃。

---

# 第二十七讲：YOLO 张量数学（为什么 6300）

## 27.1 卷积输出尺寸

```
out = floor((in + 2p - k)/s) + 1
```
例：`k=3, s=2, p=1`：
```
out = floor((480+2-3)/2)+1 = floor(479/2)+1 = 239+1 = 240
```
所以 480→240→120→60→30→15。

## 27.2 三档特征图

| 档 | 尺寸 | 步长 | 格子数 |
|---|---|---|---|
| P3 | 60×80 | 8 | 4800 |
| P4 | 30×40 | 16 | 1200 |
| P5 | 15×20 | 32 | 300 |
| 合计 | | | 6300 |

步长 = 输入尺寸 / 特征图尺寸：
```
640/80 = 8
640/40 = 16
640/20 = 32
```

## 27.3 一列 21 个数

```
[颜色×4][编号×9][x1,y1,x2,y2,x3,y3,x4,y4]
```
- 类别是**多标签**：颜色 4 类可同时 sigmoid；
- 编号 9 类也是 sigmoid；
- 角点是回归值，单位是**输入图 640×480 的像素**。

## 27.4 列号到格子

```
j = i - O
c = j % W
r = j / W
cx = c*s + s/2
cy = r*s + s/2
```
`O` 是每档起始偏移：P3=0，P4=4800，P5=6000。

## 27.5 为什么角点不是相对格子

本模型输出**绝对像素坐标**（输入图坐标系），不是相对格子偏移。
所以解码时不需要加 `cx,cy`，只需减 pad 再除 scale。

如果换成相对偏移模型，则：
```
x = (cx + dx)/scale - pad_x/scale
```
两种协议不能混。

## 27.6 关键点顺序

训练标注固定 `LT→RT→RB→LB`。
模型输出 `p0..p3` 应对应这个顺序。
若发现 `p0` 在右下，说明导出/训练顺序不同，需要重排：
```
按 y 最小 → 左上；再按 x 分左右
```
或用质心角度排序。

---

# 第二十八讲：letterbox 数值例子与边界

## 28.1 1440×1080 → 640×480

```
scale = min(640/1440, 480/1080) = min(0.4444, 0.4444) = 0.4444
nw = 1440*0.4444 = 640
nh = 1080*0.4444 = 480
pad_x = 0, pad_y = 0
```
正好铺满，无补边。

## 28.2 1280×720（16:9）

```
scale = min(640/1280, 480/720) = min(0.5, 0.6667) = 0.5
nw = 640, nh = 360
pad_x = 0, pad_y = (480-360)/2 = 60
```
上下各补 60 像素灰色。

还原：
```
x_orig = x_net / 0.5
y_orig = (y_net - 60) / 0.5
```

## 28.3 边界检查

- `pad_x/pad_y` 可能为负？不会，`min` 保证 ≤；
- 浮点 `round` 后 `nw` 可能 639/641，粘贴矩形要 clamp；
- `cv::Rect` 超出 canvas 会崩，先 `&` 一下。

## 28.4 还原顺序

```
1. 减 pad
2. 除 scale
3. 可选：去畸变（undistortPoints）
```
若先除 scale 再减 pad，会错。

## 28.5 常见错误

| 错误 | 结果 |
|---|---|
| 用 `resize` 后忘了 pad | 点整体偏移 |
| pad 用 `(640-nw)` 不除 2 | 偏到一边 |
| scale 用 `max` | 图像超出 |
| 还原用 `x*scale+pad` | 方向反 |

---

# 第二十九讲：NMS 与 IoU 手算

## 29.1 IoU

```
IoU = 交集面积 / 并集面积
```

例：
```
A = [0,0,10,10]  面积100
B = [5,5,15,15]  面积100
交集 = [5,5,10,10] = 25
并集 = 100+100-25 = 175
IoU = 25/175 ≈ 0.143
```
若 B 与 A 几乎重合（偏移 1px）：
```
交集≈81, 并集≈119, IoU≈0.68
```
NMS 阈值 0.45：>0.45 抑制。

## 29.2 NMS 流程

1. 按分数降序；
2. 取最高分框，保留；
3. 删除与它 IoU > 阈值的框；
4. 重复。

## 29.3 为什么同一块板多个候选

- 相邻格子都预测到同一块板；
- 关键点略有差异；
- 颜色/编号分数不同。

NMS 保留分数最高，抑制邻近。

## 29.4 阈值选择

- 0.45：常规；
- 0.3：框很密时更狠；
- 0.6：目标密集、怕误删。

## 29.5 坑点

| 坑 | 结果 |
|---|---|
| 用四点做 NMS | 需要转矩形，易错 |
| 先 NMS 再阈值 | 低分框浪费计算 |
| 阈值为 0 | 不去重 |
| 只按颜色分组 | 不同颜色同框不去重 |

---

# 第三十讲：关键函数逐行注释版

## 30.1 回调骨架

```cpp
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 1. 相机内参没到齐，直接返回；避免用空 K 做 PnP
    if (!is_ready_.load()) return;

    try {
        // 2. 零拷贝取图；bgr8 保证三通道
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 3. 分辨率保护：后续阈值/ROI 都按固定尺寸写
        if (img.cols != width_ || img.rows != height_) return;

        // 4. canvas 是绘图底板；clone 防止改原图
        cv::Mat canvas = img.clone();

        // 5. NN 分支：走 processNN，发布后直接 return
        if (nn_enable_) {
            processNN(img, msg, canvas);
            cv_bridge::CvImage out;
            out.header = msg->header;
            out.encoding = "bgr8";
            out.image = canvas;
            image_pub_.publish(out.toImageMsg());
            return;
        }

        // 6. 传统分支
        cv::Mat colored;
        colorSegment(canvas, colored, ...);   // 分色
        // ... 灯条 → 配对 → PnP → 选择 → TF → 发布
    } catch (const cv_bridge::Exception& e) {
        // 7. 只捕获转换异常；算法异常应单独处理
        ROS_ERROR("cv_bridge: %s", e.what());
    }
}
```

## 30.2 配对循环注释

```cpp
for (size_t i = 0; i < bars.size(); ++i) {
    for (size_t j = i + 1; j < bars.size(); ++j) {
        // 1. 距离门限：太近是断笔，太远不是同板
        double dx = std::abs(bars[i].center.x - bars[j].center.x);
        if (dx < min_dist_x_ || dx > max_dist_x_) continue;

        // 2. 高度差：两灯条应大致同高
        double dy = std::abs(bars[i].center.y - bars[j].center.y);
        if (dy > pair_dy_ratio_ * std::min(bars[i].height, bars[j].height)) continue;

        // 3. 平行度：倾角差
        double ddeg = std::abs(bars[i].tilt - bars[j].tilt);
        if (ddeg > max_angle_diff_) continue;

        // 4. 比例：判大小板
        double ratio = corrected_dx / bar_len;
        bool is_large = ratio > ratio_s_max_;

        // 5. 四点（TL,TR,BR,BL）
        // 6. solvePnP
        // 7. 重投影/几何否决
        // 8. 通过则 push 到 targets
    }
}
```

## 30.3 跟踪回调注释

```cpp
void detectionCallback(const ft_msgs::Detections::ConstPtr& msg) {
    // 1. 空数组：当作丢检，走预测
    if (msg->detections.empty()) { predictOnly(last_image_stamp_); return; }

    // 2. 时间戳：图像时间
    ros::Time stamp = msg->header.stamp;
    last_image_stamp_ = stamp;

    // 3. 关联：选离预测最近且在门限内
    int pick = -1; double best = assoc_gate_;
    for (size_t i = 0; i < msg->detections.size(); ++i) {
        double d = distance(det[i].pose.position, kf_.statePost);
        if (d < best) { best = d; pick = i; }
    }
    if (pick < 0) { predictOnly(stamp); return; }

    // 4. NaN 保护：PnP 可能出 NaN
    if (!isfinite(det.pose.position.x)) return;

    // 5. 首次：初始化状态
    // 6. 后续：dt→predict→correct
    // 7. 提前量：position + velocity*T
    // 8. 发布 Track + marker
}
```

## 30.4 旋转回调注释

```cpp
void detCb(const ft_msgs::Detections::ConstPtr& msg) {
    // 1. 空：onLost（短时预测/超时重置）
    if (msg->detections.empty()) { onLost(stamp); return; }

    // 2. 选择：有预测选最近，否则选 |x| 最小
    // 3. Kalman：首帧初始化，后续 predict+correct
    // 4. 历史：push (t, x, z)，裁剪窗口
    // 5. 圆拟合：≥8 点且 R 合理才更新中心
    // 6. 相位：atan2(z-cz, x-cx)
    // 7. 角速度：跨 2π 归一 + 低通
    // 8. 预测：先 center+v*T，再 θ+ω*T
    // 9. 发布 Track（valid）+ 三个 marker
}
```

---

# 第三十一讲：参数敏感性实验设计

## 31.1 单变量原则

一次只改一个参数，其余固定；跑同一段 bag；记录：
- 检测帧率；
- 目标数；
- 框位置（目测/截图）；
- z 的均值/方差；
- 丢检率。

## 31.2 关键参数表

| 参数 | 增大效果 | 减小效果 | 建议范围 |
|---|---|---|---|
| `min_area` | 少噪点、漏小目标 | 多噪点 | 20–100 |
| `bar_v_min` | 少反光、漏暗灯条 | 多反光 | 100–180 |
| `hsv_sv_min` | 更纯、漏暗 | 更多噪 | 40–80 |
| `pair_dy_ratio` | 更宽容、误配多 | 更严、漏 | 0.2–0.5 |
| `ratio_s_max` | 大板判多 | 小板判多 | 2.8–3.2 |
| `lock_gate_px` | 更容易锁、跳板 | 更稳、易丢 | 30–80 |
| `lock_ratio_tol` | 宽容 | 严 | 0.05–0.2 |
| `q_vel` | 跟得快、抖 | 平滑、滞后 | 0.1–1.0 |
| `r_pos` | 信预测、滞后 | 信观测、抖 | 0.02–0.2 |
| `assoc_gate` | 易关联错 | 易丢 | 0.3–0.8 |
| `lost_time_thresh` | 轨迹保留久 | 删得快 | 0.3–1.0 |
| `color_thres` | 少误检、漏 | 多检 | 0.3–0.7 |
| `nms_thres` | 少抑制、重复 | 多抑制 | 0.3–0.6 |

## 31.3 记录模板

```
日期/段：
参数：xxx 从 A → B
现象：框数/抖动/丢检
结论：保留/回退
```

## 31.4 不要做的事

- 同时改 3 个参数；
- 用不同 bag 对比；
- 只看一帧；
- 改完不记录。

---

# 第三十二讲：失败案例库（讲解用）

## 32.1 过曝

- 现象：灯条糊成一片，轮廓巨大；
- 原因：V 饱和，HSV 失效；
- 证据：`mean(V)>250`，面积异常；
- 处理：降曝光/加 `dyn_v_min`/白场兜底（v3）。

## 32.2 地面反光

- 现象：画面底部高亮块被当灯条；
- 原因：地面镜面反射；
- 证据：`max_loc.y` 接近底部；
- 处理：`guard_y_ratio` 屏蔽底部。

## 32.3 换板跳变

- 现象：框在两面之间跳；
- 原因：多板同时可见，选择不稳定；
- 证据：`lock_left_` 频繁解锁；
- 处理：锁定 + `lock_gate_px`/`lock_ratio_tol`。

## 32.4 远距 z 跳

- 现象：z 在 6–10m 跳；
- 原因：板只占十几像素，PnP 病态；
- 证据：框宽 <20px；
- 处理：`z_max` 限制/中值/锁定。

## 32.5 遮挡丢检

- 现象：几帧无框，跟踪断；
- 原因：灯条被挡；
- 证据：`targets` 空；
- 处理：`lost_time_thresh` + 预测，`tracking=false`。

## 32.6 NN 误检

- 现象：背景出现框；
- 原因：颜色分数过阈值；
- 证据：`score` 0.5–0.6，角点乱；
- 处理：升 `color_thres`/NMS/几何校验。

## 32.7 NN 编号错

- 现象：编号跳变；
- 原因：数字小、模糊；
- 证据：`number` 分数低；
- 处理：升 `number_thres`，或只信连续帧多数。

## 32.8 旋转失效

- 现象：预测绕反/乱飞；
- 原因：正对相位退化、R 突变；
- 证据：`R` 跳变、`ω` 大；
- 处理：限制 R、低通、正对时退回匀速。

---

# 第三十三讲：答辩问答（背下来）

**Q1：为什么用 IPPE 不用 EPNP？**
A：装甲板是平面四点，IPPE 有解析解、快且稳；EPNP 通用但需要更多点/迭代。

**Q2：z 为什么跳？**
A：远距像素少、平面正对时深度弱约束、四点顺序/尺寸误差；用锁定/中值/`z_max` 抑制。

**Q3：Kalman 的 dt 为什么不能用 now()？**
A：图像有延迟，now() 把延迟算进 dt，速度估计抖；必须用两帧图像时间戳差。

**Q4：丢检后怎么处理？**
A：`predict()` 继续外推，`tracking=false`；超过 0.5s 删除轨迹重捕。

**Q5：为什么需要锁定？**
A：多板时“最正对”会跳；锁定用上一帧位置/比例约束，保证 ID 稳定。

**Q6：旋转模型假设？**
A：水平圆周、R 近似不变、ω 短时恒定、车体匀速平移；正对/遮挡/急停会失效。

**Q7：NN 输出为什么不是框？**
A：模型是 pose/关键点头，输出 21 行（颜色/编号/四点），不是 `[x,y,w,h]`。

**Q8：怎么证明不是整包生成？**
A：有 attempts 记录、回退、参数化、badcase 分支，能逐行解释每个阈值。

**Q9：传统和 NN 怎么切换？**
A：`nn_enable` 参数，或 `test_vision.launch mode:=tradition|nn`。

**Q10：怎么保证消息时间一致？**
A：所有 `header.stamp` 用图像时间；纯预测用最近一帧图像时间。

---

# 第三十四讲：一页纸总复习

```
输入：compressed → cv_bridge → BGR
分色：HSV 红两段/蓝/黄 → mask → 开运算
灯条：findContours → 面积/长宽/亮度 → minAreaRect → 矩质心
配对：dx/dy/角度 → ratio 判大小板 → 四点
PnP：物体点 Wb×bH（米）→ IPPE → tvec/rvec → z 过滤/重投影
选择：锁定（预测+比例）→ 最正对兜底 → 平滑
发布：Detections + TF（图像时间）
跟踪：Kalman 6 状态 → predict/correct → T 提前量 → marker
丢检：predict + tracking=false；>0.5s 删除
旋转：圆拟合 → 相位/ω → 先平移后旋转
NN：letterbox → RGB/NCHW → ORT → 21×6300 → 解码 → NMS
配置：YAML + cfg；测试：rostopic/tf/rqt/截图
```
