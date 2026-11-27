# v3_2 逐行代码作用对照（车辆几何推断）

> v3_2 = v3 + `hc::` 几何接口 + 车体 TF/投影/预测。
> 与 v3 相同的检测主流程见 `v3_line_by_line.md`，本文只讲新增部分。

## 1. `include/final_text/motion_geometry.hpp`
```cpp
 1  #pragma once
 2  // 车辆几何推断：装甲位姿 + 安装偏移 -> 车体；匀速/圆周解析预测；针孔投影
 3  #include <opencv2/opencv.hpp>      // cv::Mat / Point3d
 4  #include <cmath>
 6  namespace hc {
 8  struct MotionParams { double v=0.5, R=2.0, omega_yaw=0.3; }; // 线速度/半径/角速度
 9  struct Pose2     { double x=0, y=0, yaw=0; };                 // 平面位姿
10  struct TrackState{ Pose2 pose; double t=0; };                 // 位姿 + 时刻
11  struct Mount     { double x=0, y=0, z=0, yaw=0; };            // 车体->相机安装偏移
12  struct CameraPose{ double x=0, y=0, z=0, yaw=0; };            // 相机在世界系位姿
13  struct Pixel     { double u=0, v=0; bool in_image=false; };   // 投影像素与可见性
15  // 任意 t 的解析位姿：ω≠0 走圆弧，否则直线
16  inline TrackState sampleMotion(const MotionParams& m, double t) {
17      TrackState s; s.t = t;
18      if (std::abs(m.omega_yaw) > 1e-9) {
19          double th = m.omega_yaw * t;          // 转过角度
20          s.pose.x = m.R * std::sin(th);        // 圆弧 x
21          s.pose.y = m.R * (1.0 - std::cos(th));// 圆弧 y（起点切线为 x）
22          s.pose.yaw = th;                      // 航向角
23      } else {
24          s.pose.x = m.v * t;                   // 直线：s = v*t
25          s.pose.y = 0.0; s.pose.yaw = 0.0;
26      }
27      return s;
28  }
30  // 车体位姿 + 安装偏移 -> 相机位姿
32  inline CameraPose cameraPose(const Pose2& body, const Mount& mount) {
33      CameraPose c;
34      double cy=cos(body.yaw), sy=sin(body.yaw);
35      c.x = body.x + cy*mount.x - sy*mount.y;   // 偏移随车体旋转
36      c.y = body.y + sy*mount.x + cy*mount.y;
37      c.z = mount.z;                            // 高度
38      c.yaw = body.yaw + mount.yaw;             // 相机相对车体偏航
39      return c;
40  }
42  // 世界点 -> 像素（相机 x 右、y 下、z 前）
43  inline Pixel project(const CameraPose& cam, const cv::Mat& K,
44                       const cv::Point3d& p, int W, int H) {
45      Pixel px;
46      if (K.empty() || K.rows < 3) return px;   // 没内参直接返回
47      double dx=p.x-cam.x, dy=p.y-cam.y, dz=p.z-cam.z;
48      double cy=cos(cam.yaw), sy=sin(cam.yaw);
49      double Xc =  cy*dx + sy*dy;               // 前
50      double Yc = -sy*dx + cy*dy;               // 右
51      double Zc =  dz;                          // 下
52      if (Xc <= 1e-6) return px;                // 在相机后面
53      double fx=K.at<double>(0,0), fy=K.at<double>(1,1);
54      double cx=K.at<double>(0,2), cyy=K.at<double>(1,2);
55      px.u = fx*Yc/Xc + cx;                     // 针孔投影
56      px.v = fy*Zc/Xc + cyy;
57      px.in_image = (px.u>=0 && px.u<W && px.v>=0 && px.v<H);
58      return px;
59  }
```

## 2. v3_2 新增参数（127-135）
```cpp
127  pnh_.param("mount_x", mount_x_, 0.0);      // 装甲->车体 安装偏移(相机系)
128  pnh_.param("mount_y", mount_y_, 0.0);
129  pnh_.param("mount_z", mount_z_, 0.0);
130  pnh_.param("mount_yaw", mount_yaw_, 0.0);
131  pnh_.param("pred_v", pred_v_, 0.5);        // 解析运动模型参数
132  pnh_.param("pred_R", pred_R_, 2.0);
133  pnh_.param("pred_omega", pred_omega_, 0.3);
134  pnh_.param("pred_t", pred_t_, 0.2);        // 预测时长 s
135  pnh_.param<std::string>("car_tf_child", car_tf_child_, "car_v3_2");
```

## 3. 车辆几何推断块（1118-1180）
```cpp
1118  // ================= 车辆几何推断（v3_2） =================
1119  if (mt.has_r) {                                  // 该目标有 PnP 旋转
1120      cv::Mat Rm = mt.R;
1121      // 装甲中心 + 安装偏移 -> 车体中心（相机系）
1122      cv::Mat off = (Mat_<double>(3,1) << mount_x_, mount_y_, mount_z_);
1123      cv::Mat car_m = (Mat_<double>(3,1) << mt.t[0], mt.t[1], mt.t[2]) + Rm*off;
1124      cv::Vec3d car(car_m(0), car_m(1), car_m(2));

1126      // 1) 车体 TF：camera_frame -> car_v3_2
1127      geometry_msgs::TransformStamped cs;
1128      cs.header.stamp = msg->header.stamp;
1129      cs.header.frame_id = camera_frame_;
1130      cs.child_frame_id = car_tf_child_;
1131      cs.transform.translation = car;              // 位置
1132      tf2::Quaternion cq; cq.setRPY(0,0,mt.yaw);   // 朝向
1135      tf_broadcaster_.sendTransform(cs);

1137      // 2) 车体中心投影像素（相机系 x右/y下/z前 -> hc 前/右/下）
1138      hc::CameraPose cam0;
1139      hc::Mount mount0;
1140      auto px = hc::project(cam0, K_,
1141                            cv::Point3d(car[2], car[0], car[1]),
1142                            width_, height_);
1143      if (px.in_image) {                           // 画 CAR
1144          cv::circle(canvas, Point(px.u,px.v), 6, Scalar(255,0,255), -1);
1145          cv::putText(canvas, "CAR", ...);
1146      }

1148      // 3) 解析预测：sampleMotion(0.2s)，沿车体前向/右向叠加
1149      hc::MotionParams mp; mp.v=pred_v_; mp.R=pred_R_; mp.omega_yaw=pred_omega_;
1150      hc::TrackState st = hc::sampleMotion(mp, pred_t_);
1151      cv::Vec3d fwd(Rm(0,2), Rm(1,2), Rm(2,2));    // 车体前向（相机系）
1152      cv::Vec3d right(Rm(0,0), Rm(1,0), Rm(2,0));  // 车体右向
1153      cv::Vec3d car_pred = car + fwd*st.pose.x + right*st.pose.y;
1154      auto pp = hc::project(cam0, K_,
1155                            cv::Point3d(car_pred[2], car_pred[0], car_pred[1]),
1156                            width_, height_);
1157      if (pp.in_image) {                           // 画 PRED-CAR
1158          cv::circle(canvas, Point(pp.u,pp.v), 5, Scalar(0,255,255), -1);
1159          cv::putText(canvas, "PRED-CAR", ...);
1160      }

1162      // 4) cameraPose 接口接入（车体->相机）
1163      hc::Pose2 body; body.x=car[0]; body.y=car[1]; body.yaw=mt.yaw;
1164      hc::Mount mount; mount.x=mount_x_; mount.y=mount_y_;
1165      mount.z=mount_z_; mount.yaw=mount_yaw_;
1166      hc::CameraPose cam_pose = hc::cameraPose(body, mount);
1167      (void)cam_pose;
1168  }
```

## 4. 新增成员（1562-1564）
```cpp
1562  double mount_x_, mount_y_, mount_z_, mount_yaw_;   // 安装偏移
1563  double pred_v_, pred_R_, pred_omega_, pred_t_;      // 运动模型
1564  std::string car_tf_child_;                         // 车体 TF 子 frame
```

## 5. 与 v3 的关系
- 检测主流程（分色→灯条→配对→PnP→平滑→TF/检测消息→FCS/外推）与 v3 完全一致；
- v3_2 只在“主目标成功”后追加车体几何推断；
- 节点名 `robot_vision_node_v3_2`，TF：`armor_v3`（沿用 `tf_child_`）+ `car_v3_2`。
