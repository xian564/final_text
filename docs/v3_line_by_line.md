# v3 逐行代码作用对照

> 行号对应 `src/detection_node_v3.cpp`（含注释后的版本）。

## 1. 回调入口与预处理（408-465）
```cpp
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {   // 每帧入口
    if (!is_ready_.load()) return;                            // 内参未到，丢帧
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(msg, "bgr8");                    // ROS Image -> cv::Mat(BGR)
    cv::Mat img = cv_ptr->image;                              // 原图（只读）
    cv::Mat canvas = img.clone();                             // 绘图底板
    cv::rectangle(canvas, Rect(10,10,50,50), green, 2);       // debug 绿框标记

    cv::Mat colored;
    colorSegment(canvas, colored, ...);                       // HSV 分色 -> colored

    cv::Mat hsv_full;
    cv::cvtColor(colored, hsv_full, COLOR_BGR2HSV);           // -> HSV
    cv::Mat v_chan;
    cv::extractChannel(hsv_full, v_chan, 2);                  // -> V 亮度通道

    cv::minMaxLoc(v_chan, ..., &max_v, ..., &max_loc);        // 最亮点
    bool guard_block = (max_loc.y > height_ * guard_y_ratio_);// 底部反光？
    if (guard_block) colored.rowRange(...).setTo(0);          // 屏蔽底部，保留其余灯条

    cv::compare(v_chan, dyn_v_thresh, high_mask, CMP_GT);     // 过曝高亮掩膜
```

## 2. 轮廓 -> 灯条（466-520）
```cpp
cv::findContours(seg_mask, contours, RETR_EXTERNAL, ...);     // 外轮廓
for (size_t i = 0; i < contours.size(); ++i) {
    if (cv::contourArea(contours[i]) < min_area_) continue;   // 小噪点
    cv::RotatedRect rr = cv::minAreaRect(contours[i]);        // 旋转矩形
    double aspect = max(w,h)/min(w,h);                        // 长宽比
    if (aspect < aspect_min_ || aspect > aspect_max_) continue;
    cv::drawContours(cmask, contours, i, 255, FILLED);        // 轮廓掩膜
    double mean_v = cv::mean(v_chan, cmask)[0];               // 平均亮度
    if (mean_v < bar_v_min_) continue;                        // 暗的当反光
    cv::Moments m = cv::moments(contours[i]);                 // 矩
    center = (m.m10/m.m00, m.m01/m.m00);                      // 质心
    tilt = rr.angle 归一化到 [-90,90);                        // 倾角
    bars.push_back(Bar{center, pts, rr, long_h, tilt, v});    // -> 灯条
}
// 断笔合并、按亮度排序、保留 bar_keep_n 根
// 共识平面：灯条对定直线 -> 内点最多 -> 离面>灯条长度的剔除
```

## 3. 配对与几何否决（683-770）
```cpp
for i in bars: for j in bars (j>i):
    double dx = bars[j].center.x - bars[i].center.x;          // 水平间距
    if (dx > max_dist_x_ || dx <= min_dist_x_) continue;      // 距离窗
    if (|cy_i-cy_j| > 50) continue;                           // 粗错位
    if (|tilt_i-tilt_j|*180/pi >= max_angle_diff_) continue;  // 平行度
    corrected = dx / cos(avg_angle);                          // 透视修正
    long_h = max(long_h_i, long_h_j);                         // 灯条长
    ratio = corrected / long_h;                               // 宽高比
    if (ratio > ratio_l_max_ || ratio < ratio_min_) continue; // 幽灵
    if (min(long_h)/max(long_h) < bar_h_ratio_min_) continue; // 高度差否决
    if (|cy_i-cy_j| > pair_dy_ratio_*long_h) continue;        // 竖直错位否决
    bool is_large = (ratio > ratio_s_max_);                   // 大/小板
```

## 4. 四点 + PnP（771-843）
```cpp
double half_w = (is_large?armor_large_w_:armor_small_w_)*0.5*pnp_scale_;
double half_h = (is_large?armor_large_h_:armor_small_h_)*0.5*pnp_scale_;
obj = {(-w,-h,0), (w,-h,0), (w,h,0), (-w,h,0)};              // 物体点 TL/TR/BR/BL
cv::solvePnP(obj, tg.corners, K_, D_, rvec, tvec, false, IPPE);   // -> rvec/tvec
if (!ok) cv::solvePnP(..., ITERATIVE);                            // 兜底
if (tvec.z <= 0.3 || tvec.z > 15) continue;                       // 距离合理性
cv::Rodrigues(rvec, R);                                            // 旋转矩阵
tg.t = tvec; tg.yaw = atan2(...);                                  // 存进 Target
```

## 5. 发检测数组（882-946）
```cpp
ft_msgs::Detections arr; arr.header = msg->header;
for each target:
    int n_red  = cv::countNonZero(red_mask_in_box);           // 框内红像素
    int n_blue = cv::countNonZero(blue_mask_in_box);          // 框内蓝像素
    d.is_red = (n_red >= n_blue);                             // 颜色归属
    d.pose.position = tvec;                                   // 位置
    m3.getRotation(q); d.pose.orientation = q;                // rvec->四元数
    arr.detections.push_back(d);                              // 一帧多目标
detection_array_pub_.publish(arr);
```

## 6. 选择 / 平滑 / TF（1004-1116）
```cpp
int best = 0; bool locked = false;
if (prev_had_target_) {                                       // 有历史
    best = argmin |center - prev_center2d_|;                  // 最近的一块
    if (dist < 300) locked = true;                            // 锁定，不换板
}
if (!locked) best = argmin |avg_tilt|;                        // 无历史 -> 最正对
Target& mt = targets[best];
cv::Vec3d t2(mt.t[0], -mt.t[1], mt.t[2]);                     // 光学系 -> 约定系
pushSmooth(t2, mt.yaw, avg_t, avg_yaw);                       // 平滑
z_hist_.push_back(mt.z); avg_t[2] = median(z_hist_);          // z 中值
if (moved) { ts.translation = avg_t; q.setRPY(0,0,avg_yaw);   // TF
             tf_broadcaster_.sendTransform(ts); }
// 速度 = (avg_t - v_prev_pos_)/dt -> vel_，供丢帧外推
```

## 7. 丢帧匀速外推（1396-1450）
```cpp
if (has_last_ && has_vel_ && lost_frames_ < hold_frames_) {
    double pdt = stamp - v_prev_stamp_;                       // 距上次成功时间
    if (pdt > 0 && pdt < 0.5 && |vel_| <= hold_max_speed_) {
        cv::Vec3d pp = v_prev_pos_ + vel_ * pdt;              // 直线外推
        ts.translation = pp; q.setRPY(0,0,last_yaw_);         // 预测 TF
        tf_broadcaster_.sendTransform(ts);
        cv::putText(canvas, "PRED", ...);                     // 画面标记
    }
}
++lost_frames_;
if (lost_frames_ >= max(hold_frames_,5)) clear_state();       // 超时清空
```
