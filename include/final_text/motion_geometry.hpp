#pragma once
// 车辆几何推断：装甲位姿 + 安装偏移 -> 车体；匀速/匀速圆周解析预测；针孔投影
#include <opencv2/opencv.hpp>
#include <cmath>

namespace hc {

struct MotionParams { double v = 0.5; double R = 2.0; double omega_yaw = 0.3; };
struct Pose2 { double x = 0, y = 0, yaw = 0; };
struct TrackState { Pose2 pose; double t = 0; };
struct Mount { double x = 0, y = 0, z = 0, yaw = 0; };
struct CameraPose { double x = 0, y = 0, z = 0, yaw = 0; };
struct Pixel { double u = 0, v = 0; bool in_image = false; };

// 任意 t 的解析位姿：ω≠0 走圆弧，否则直线
inline TrackState sampleMotion(const MotionParams& m, double t) {
    TrackState s; s.t = t;
    if (std::abs(m.omega_yaw) > 1e-9) {
        double th = m.omega_yaw * t;
        s.pose.x = m.R * std::sin(th);
        s.pose.y = m.R * (1.0 - std::cos(th));
        s.pose.yaw = th;
    } else {
        s.pose.x = m.v * t;
        s.pose.y = 0.0;
        s.pose.yaw = 0.0;
    }
    return s;
}

// 车体位姿 + 安装偏移 -> 相机位姿
inline CameraPose cameraPose(const Pose2& body, const Mount& mount) {
    CameraPose c;
    double cy = std::cos(body.yaw), sy = std::sin(body.yaw);
    c.x = body.x + cy * mount.x - sy * mount.y;
    c.y = body.y + sy * mount.x + cy * mount.y;
    c.z = mount.z;
    c.yaw = body.yaw + mount.yaw;
    return c;
}

// 世界点 -> 像素（相机 x 右、y 下、z 前）
inline Pixel project(const CameraPose& cam, const cv::Mat& K,
                     const cv::Point3d& p, int W, int H) {
    Pixel px;
    if (K.empty() || K.rows < 3) return px;
    double dx = p.x - cam.x, dy = p.y - cam.y, dz = p.z - cam.z;
    double cy = std::cos(cam.yaw), sy = std::sin(cam.yaw);
    double Xc =  cy * dx + sy * dy;   // 前
    double Yc = -sy * dx + cy * dy;   // 右
    double Zc = dz;                    // 下
    if (Xc <= 1e-6) return px;
    double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2), cyy = K.at<double>(1, 2);
    px.u = fx * Yc / Xc + cx;
    px.v = fy * Zc / Xc + cyy;
    px.in_image = (px.u >= 0 && px.u < W && px.v >= 0 && px.v < H);
    return px;
}

}  // namespace hc
