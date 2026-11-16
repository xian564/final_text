// ============================================================
//  detection_node_v3.cpp  —— 终极升级版（22 点融合）
//
//  与 v1（detection_node.cpp）的关系：
//    * 新建文件，不修改、不覆盖 v1 / v2
//    * 保留 v1 的结构：VisionNode 类 + colorSegment + main
//    * 保留 ROS 收发包逻辑：订阅 /hk_camera/camera_info、/hk_camera/image_raw(compressed)
//    * 保留左上角 debug 绿框（Rect(10,10,50,50)，2px）
//
//  22 点实现索引见 docs/attempts/2026-11-15-08-v3.md
// ============================================================

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <cmath>
#include <deque>
#include <iostream>
#include <string>
#include <vector>

// ---------- 前向声明 ----------
inline void colorSegment(const cv::Mat& bgr, cv::Mat& res,
                         const cv::Scalar& lo_r, const cv::Scalar& hi_r,
                         const cv::Scalar& lo_b, const cv::Scalar& hi_b,
                         const cv::Scalar& lo_g, const cv::Scalar& hi_g,
                         int morph_size, int sv_min,
                         int y_h_min, int y_h_max, int y_s_min, int y_v_min);

// ============================================================
//  A. VisionNodeV3：ROS 节点
// ============================================================
class VisionNodeV3 {
public:
    VisionNodeV3(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh),
          is_ready_(false), transport_ready_(true),
          tf_broadcaster_(), has_last_(false), lost_frames_(0),
          has_sent_(false)
    {
        // ---------- 动态参数（22 点之 13） ----------
        pnh_.param("aspect_min",  aspect_min_, 1.3);
        pnh_.param("aspect_max",  aspect_max_, 6.0);
        pnh_.param("min_dist_x",  min_dist_x_, 15.0);
        pnh_.param("max_dist_x",  max_dist_x_, 500.0);
        pnh_.param("max_angle_diff", max_angle_diff_, 10.0);
        pnh_.param("ratio_s_max", ratio_s_max_, 1.3);
        pnh_.param("ratio_l_max", ratio_l_max_, 2.5);
        pnh_.param("ratio_min",   ratio_min_, 0.4);
        pnh_.param("max_z",       max_z_, 8.0);
        pnh_.param("deadzone_px", deadzone_px_, 5.0);
        pnh_.param("uniqueness_dt", uniqueness_dt_, 0.15);
        pnh_.param("max_speed",   max_speed_, 8.0);     // 车体物理极限 m/s
        pnh_.param("morph_size",  morph_size_, 3);
        pnh_.param("hsv_sv_min",   hsv_sv_min_, 100);
        pnh_.param("roi_y_ratio",  roi_y_ratio_, 0.0);   // 先关闭 ROI
        pnh_.param("bar_v_min",    bar_v_min_, 160);
        pnh_.param("bar_keep_n",   bar_keep_n_, 6);
        pnh_.param("z_median_win", z_median_win_, 13);
                pnh_.param("max_z_jump",    max_z_jump_, 1.5);
        pnh_.param("near_area",    near_area_, 3000);
        pnh_.param("near_sv",      near_sv_, 30);
        pnh_.param("edge_fuse",    edge_fuse_, false);
        pnh_.param("merge_bars",   merge_bars_, true);
        pnh_.param("guard_y_ratio", guard_y_ratio_, 0.85);
        pnh_.param("dyn_v_offset",  dyn_v_offset_, 50.0);
        pnh_.param("pair_y_ratio",  pair_y_ratio_, 0.75);
        pnh_.param("fcs_four_span", fcs_four_span_, 30.0);
        pnh_.param("fcs_four_tol",  fcs_four_tol_, 60.0);
        pnh_.param("blue_h_min",   blue_h_min_, 100);
        pnh_.param("blue_h_max",   blue_h_max_, 140);
        pnh_.param("blue_s_min",   blue_s_min_, 50);
        pnh_.param("blue_v_min",   blue_v_min_, 50);
        pnh_.param("white_v_min",  white_v_min_, 240);
        pnh_.param("white_s_max",  white_s_max_, 30);
        pnh_.param("white_area_min", white_area_min_, 500);
        pnh_.param("white_aspect_min", white_aspect_min_, 1.3);
        pnh_.param("white_aspect_max", white_aspect_max_, 6.0);
        pnh_.param("white_enable", white_enable_, false);   // 极端场景才开，默认关闭
        pnh_.param("yellow_h_min", yellow_h_min_, 10);
        pnh_.param("yellow_h_max", yellow_h_max_, 35);
        pnh_.param("yellow_s_min", yellow_s_min_, 100);
        pnh_.param("yellow_v_min", yellow_v_min_, 100);

        info_sub_ = nh_.subscribe(
            "/hk_camera/camera_info", 1,
            &VisionNodeV3::cameraInfoCallback, this);

        try {
            image_sub_ = it_.subscribe(
                "/hk_camera/image_raw", 1,
                &VisionNodeV3::imageCallback, this,
                image_transport::TransportHints(
                    "compressed", ros::TransportHints(), pnh_));
        } catch (const image_transport::TransportLoadException& e) {
            ROS_FATAL("compressed 传输插件加载失败: %s", e.what());
            transport_ready_ = false;
        }

        // 与原版一致：发布 /ft/debug/image
        image_pub_ = it_.advertise("/ft/debug/image", 1);
    }

    bool isReady() const {
        return is_ready_.load() && transport_ready_;
    }

private:
    struct Bar {
        cv::RotatedRect rr;
        cv::Point2f pts[4];
        cv::Point2f center;              // 矩质心（抗眩光）
        double tilt;                     // 相对竖直方向
        double long_h;
        double v;                        // 轮廓内 V 通道平均亮度
        std::vector<cv::Point> contour;
    };

    struct Target {
        int id;
        bool is_large;
        double ratio;
        double avg_tilt;
        std::vector<cv::Point2f> corners; // TL TR BR BL
        cv::Point2f center2d;
        cv::Vec3d t;                      // 光学系
        double yaw;
        double z;
        bool has_pose;
    };

    // ---------- 内参 ----------
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (msg->width == 0 || msg->height == 0) {
            ROS_WARN_THROTTLE(2.0, "空 CameraInfo, 忽略");
            return;
        }
        cv::Mat K(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                K.at<double>(r, c) = msg->K[r * 3 + c];

        cv::Mat D(1, static_cast<int>(msg->D.size()), CV_64F);
        for (size_t i = 0; i < msg->D.size(); ++i)
            D.at<double>(0, static_cast<int>(i)) = msg->D[i];

        K_ = K; D_ = D;
        width_ = msg->width; height_ = msg->height;
        camera_frame_ = msg->header.frame_id;
        distortion_model_ = msg->distortion_model;

        if (!is_ready_.load())
            ROS_INFO("CameraInfo 已缓存: %ux%u (frame=%s)",
                     msg->width, msg->height, camera_frame_.c_str());
        is_ready_.store(true);
    }

    // ---------- 队列加权平滑 ----------
    void pushSmooth(const cv::Vec3d& p, double yaw,
                    cv::Vec3d& out_p, double& out_yaw) {
        pos_queue_.push_back(p);
        yaw_queue_.push_back(yaw);
        while (pos_queue_.size() > 5) pos_queue_.pop_front();
        while (yaw_queue_.size() > 5) yaw_queue_.pop_front();

        double wsum = 0.0, sy = 0.0, cy = 0.0;
        cv::Vec3d acc(0, 0, 0);
        for (size_t k = 0; k < pos_queue_.size(); ++k) {
            double w = static_cast<double>(k + 1);
            acc += pos_queue_[k] * w;
            sy += w * std::sin(yaw_queue_[k]);
            cy += w * std::cos(yaw_queue_[k]);
            wsum += w;
        }
        out_p = acc / wsum;
        out_yaw = std::atan2(sy, cy);
    }

    // ---------- 虚线 ----------
    static void dashedRect(cv::Mat& img, const cv::Rect& r,
                           const cv::Scalar& c, int thick) {
        int step = 8;
        for (int x = r.x; x < r.x + r.width; x += step * 2)
            cv::line(img, cv::Point(x, r.y),
                     cv::Point(std::min(x + step, r.x + r.width), r.y), c, thick);
        for (int x = r.x; x < r.x + r.width; x += step * 2)
            cv::line(img, cv::Point(x, r.y + r.height),
                     cv::Point(std::min(x + step, r.x + r.width), r.y + r.height), c, thick);
        for (int y = r.y; y < r.y + r.height; y += step * 2)
            cv::line(img, cv::Point(r.x, y),
                     cv::Point(r.x, std::min(y + step, r.y + r.height)), c, thick);
        for (int y = r.y; y < r.y + r.height; y += step * 2)
            cv::line(img, cv::Point(r.x + r.width, y),
                     cv::Point(r.x + r.width, std::min(y + step, r.y + r.height)), c, thick);
    }

    // ---------- 亮度梯度（内部均值 - 边缘环均值） ----------
    static double brightGradient(const cv::Mat& v, const cv::Rect& rect) {
        cv::Rect r = rect & cv::Rect(0, 0, v.cols, v.rows);
        if (r.width < 3 || r.height < 3) return 0.0;
        cv::Mat m = cv::Mat::zeros(v.size(), CV_8UC1);
        cv::rectangle(m, r, 255, cv::FILLED);
        cv::Mat md;
        cv::dilate(m, md, cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size(11, 11)));
        cv::Mat ring;
        cv::subtract(md, m, ring);
        double inner = cv::mean(v, m)[0];
        double outer = cv::mean(v, ring)[0];
        return inner - outer;
    }

    // ---------- 四点亮度（2x2 四个光斑核心的峰值亮度） ----------
    static void fourBrightness(const cv::Mat& v, const cv::Rect& rect, double out[4]) {
        cv::Rect r = rect & cv::Rect(0, 0, v.cols, v.rows);
        for (int i = 0; i < 4; ++i) out[i] = 0.0;
        if (r.width < 4 || r.height < 4) return;
        int w = r.width / 2, h = r.height / 2;
        cv::Rect q[4] = {
            cv::Rect(r.x,     r.y,     w,         h),
            cv::Rect(r.x + w, r.y,     r.width - w, h),
            cv::Rect(r.x,     r.y + h, w,         r.height - h),
            cv::Rect(r.x + w, r.y + h, r.width - w, r.height - h)
        };
        for (int i = 0; i < 4; ++i) {
            double mx = 0.0;
            cv::minMaxLoc(v(q[i]), nullptr, &mx);
            out[i] = mx;
        }
    }

    // ---------- 图像回调 ----------
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!is_ready_.load()) return;

        try {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat img = cv_ptr->image;
            if (img.cols != width_ || img.rows != height_) return;

            cv::Mat canvas = img.clone();

            // 保留原版 debug 绿框（绝对不动）
            cv::rectangle(canvas, cv::Rect(10, 10, 50, 50),
                          cv::Scalar(0, 255, 0), 2);

            cv::Mat colored;
            colorSegment(canvas, colored,
                         cv::Scalar(0, hsv_sv_min_, hsv_sv_min_), cv::Scalar(10, 255, 255),
                         cv::Scalar(blue_h_min_, blue_s_min_, blue_v_min_), cv::Scalar(blue_h_max_, 255, 255),
                         cv::Scalar(50, hsv_sv_min_, hsv_sv_min_), cv::Scalar(70, 255, 255),
                         morph_size_, hsv_sv_min_,
                         yellow_h_min_, yellow_h_max_, yellow_s_min_, yellow_v_min_);

            // 亮度筛选用：单独提取 V 通道
            cv::Mat hsv_full, v_chan;
            cv::cvtColor(canvas, hsv_full, cv::COLOR_BGR2HSV);
            cv::extractChannel(hsv_full, v_chan, 2);

            // 1) 全图最高光定位
            double max_v = 0.0;
            cv::Point max_loc;
            cv::minMaxLoc(v_chan, nullptr, &max_v, nullptr, &max_loc);

            // 2) 全局空间守卫：最亮点在画面极底部 -> 大概率地面反光
            bool guard_block = (max_loc.y > height_ * guard_y_ratio_);
            if (guard_block) {
                ROS_INFO_THROTTLE(1.0, "[Guard] 最亮点在底部 y=%d, 跳过本帧", max_loc.y);
            }

            // 3) 动态高亮掩膜（解决过曝）
            double dyn_v_thresh = std::max(180.0, max_v - dyn_v_offset_);
            cv::Mat high_mask;
            cv::compare(v_chan, dyn_v_thresh, high_mask, cv::CMP_GT);

            const double stamp = msg->header.stamp.toSec();
            int lost_this_frame = 1;

            if (!colored.empty() && !guard_block) {
                // 显示用底板：保留原始相机画面（不再用分割结果覆盖成黑底）
                // canvas 仍是 img.clone() + 绿框，检测/绘制都叠加在原视频上
                // =======================================================
                // 1. 灯条提取：矩质心 + 长宽比 2.0~6.0
                // =======================================================
                std::vector<Bar> bars;
                std::vector<std::vector<cv::Point>> contours;
                cv::Mat seg_mask;
                cv::cvtColor(colored, seg_mask, cv::COLOR_BGR2GRAY);

                // 3) 过曝中心融合：颜色区域膨胀后与高亮 mask 相交，再并入分割 mask
                if (!high_mask.empty()) {
                    cv::Mat color_mask, color_dil, high_fused;
                    cv::threshold(seg_mask, color_mask, 1, 255, cv::THRESH_BINARY);
                    cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT,
                                                           cv::Size(3, 3));
                    cv::dilate(color_mask, color_dil, k3, cv::Point(-1, -1), 2);
                    cv::bitwise_and(high_mask, color_dil, high_fused);
                    cv::bitwise_or(seg_mask, high_fused, seg_mask);
                }

                if (edge_fuse_) {
                    cv::Mat gray, edges, sx, absx, em;
                    cv::cvtColor(canvas, gray, cv::COLOR_BGR2GRAY);
                    cv::Canny(gray, edges, 60, 160);
                    cv::Sobel(gray, sx, CV_16S, 1, 0, 3);
                    cv::convertScaleAbs(sx, absx);
                    cv::threshold(absx, em, 60, 255, cv::THRESH_BINARY);
                    cv::bitwise_or(edges, em, em);
                    cv::dilate(em, em, cv::getStructuringElement(cv::MORPH_RECT,
                                                                 cv::Size(3, 3)));
                    cv::bitwise_and(seg_mask, em, seg_mask);
                }

                cv::findContours(seg_mask, contours, cv::RETR_EXTERNAL,
                                 cv::CHAIN_APPROX_SIMPLE);

                // 近战自适应：最大颜色轮廓足够大 -> 用更低 S/V 重分割一次
                double max_c_area = 0;
                for (size_t ci = 0; ci < contours.size(); ++ci)
                    max_c_area = std::max(max_c_area, cv::contourArea(contours[ci]));
                if (max_c_area > near_area_) {
                    cv::Mat colored2;
                    colorSegment(canvas, colored2,
                                 cv::Scalar(0, near_sv_, near_sv_), cv::Scalar(10, 255, 255),
                                 cv::Scalar(blue_h_min_, near_sv_, near_sv_), cv::Scalar(blue_h_max_, 255, 255),
                                 cv::Scalar(50, near_sv_, near_sv_), cv::Scalar(70, 255, 255),
                                 morph_size_, near_sv_,
                                 yellow_h_min_, yellow_h_max_, near_sv_, near_sv_);
                    cv::bitwise_or(colored, colored2, colored);
                    cv::cvtColor(colored, seg_mask, cv::COLOR_BGR2GRAY);
                    cv::findContours(seg_mask, contours, cv::RETR_EXTERNAL,
                                     cv::CHAIN_APPROX_SIMPLE);
                    ROS_INFO_THROTTLE(1.0, "[Near] 近战重分割 (area=%.0f, S/V=%d)",
                                      max_c_area, near_sv_);
                }

                for (size_t i = 0; i < contours.size(); ++i) {
                    if (cv::contourArea(contours[i]) < 20) continue;
                    cv::RotatedRect rr = cv::minAreaRect(contours[i]);
                    double w = rr.size.width, h = rr.size.height;
                    if (std::min(w, h) <= 0) continue;
                    double aspect = std::max(w, h) / std::min(w, h);
                    if (aspect < aspect_min_ || aspect > aspect_max_) continue;

                    // 最高亮度过滤：真灯条主动发光，反光达不到这条线
                    cv::Mat cmask(canvas.size(), CV_8UC1, cv::Scalar(0));
                    cv::drawContours(cmask, contours, static_cast<int>(i),
                                     cv::Scalar(255), cv::FILLED);
                    double mean_v = cv::mean(v_chan, cmask)[0];
                    if (mean_v < bar_v_min_) continue;

                    Bar bar;
                    bar.rr = rr; rr.points(bar.pts);

                    cv::Moments m = cv::moments(contours[i]);
                    bar.center = rr.center;
                    if (std::abs(m.m00) > 1e-6)
                        bar.center = cv::Point2f(
                            static_cast<float>(m.m10 / m.m00),
                            static_cast<float>(m.m01 / m.m00));

                    cv::Point2f e1 = bar.pts[1] - bar.pts[0];
                    cv::Point2f e2 = bar.pts[2] - bar.pts[1];
                    cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
                    double tilt = std::atan2(lv.x, lv.y);
                    if (tilt > CV_PI / 2) tilt -= CV_PI;
                    if (tilt < -CV_PI / 2) tilt += CV_PI;
                    bar.tilt = tilt;
                    bar.long_h = std::max(cv::norm(e1), cv::norm(e2));
                    bar.v = mean_v;
                    bar.contour = contours[i];

                    // 补丁4：y 方向 ROI，滤掉画面下部的地面反光
                    if (roi_y_ratio_ > 0.0 &&
                        bar.center.y > height_ * roi_y_ratio_) continue;

                    bars.push_back(bar);

                    for (int k = 0; k < 4; ++k)
                        cv::line(canvas, bar.pts[k], bar.pts[(k + 1) % 4],
                                 cv::Scalar(0, 255, 255), 2);
                }

                // 断笔重连：共线且竖直方向相邻的短灯条合并
                if (merge_bars_) {
                    bool merged_any = true;
                    while (merged_any) {
                        merged_any = false;
                        for (size_t i = 0; i < bars.size() && !merged_any; ++i) {
                            for (size_t j = i + 1; j < bars.size(); ++j) {
                                if (std::abs(bars[i].center.x - bars[j].center.x) > 18.0)
                                    continue;
                                double h1 = bars[i].long_h, h2 = bars[j].long_h;
                                if (std::min(h1, h2) < 0.5 * std::max(h1, h2)) continue;
                                double t1 = bars[i].center.y - h1 * 0.5;
                                double b1 = bars[i].center.y + h1 * 0.5;
                                double t2 = bars[j].center.y - h2 * 0.5;
                                double b2 = bars[j].center.y + h2 * 0.5;
                                double gap = std::max(t1, t2) - std::min(b1, b2);
                                if (gap > 0.6 * std::max(h1, h2)) continue;

                                std::vector<cv::Point> all = bars[i].contour;
                                all.insert(all.end(), bars[j].contour.begin(), bars[j].contour.end());
                                cv::RotatedRect rr = cv::minAreaRect(all);
                                cv::Point2f pts[4]; rr.points(pts);
                                bars[i].rr = rr;
                                for (int k = 0; k < 4; ++k) bars[i].pts[k] = pts[k];
                                bars[i].contour = all;
                                bars[i].center = cv::Point2f((t1 + std::min(b1, b2)) * 0.5f,
                                                             (t1 + std::min(b1, b2)) * 0.5f);
                                bars[i].center = cv::Point2f(
                                    (bars[i].rr.center.x), (bars[i].rr.center.y));
                                bars[i].long_h = std::max(b1, b2) - std::min(t1, t2);
                                bars[i].v = std::max(bars[i].v, bars[j].v);
                                bars.erase(bars.begin() + j);
                                merged_any = true;
                                break;
                            }
                        }
                    }
                }

                // 择优保留：只留最亮的 N 根
                std::sort(bars.begin(), bars.end(),
                          [](const Bar& a, const Bar& b) { return a.v > b.v; });
                if (static_cast<int>(bars.size()) > bar_keep_n_)
                    bars.resize(bar_keep_n_);

                std::sort(bars.begin(), bars.end(),
                          [](const Bar& a, const Bar& b) {
                              return a.center.x < b.center.x;
                          });

                // =======================================================
                // 2. 配对 + 几何修正 + 幽灵剔除
                // =======================================================
                std::vector<Target> targets;
                int armor_id = 1;

                for (size_t i = 0; i < bars.size(); ++i) {
                    for (size_t j = i + 1; j < bars.size(); ++j) {
                        double dx = bars[j].center.x - bars[i].center.x;
                        // 补丁1：绝对距离拦截（>250 或 <15 一律不配对）
                        if (dx > max_dist_x_ || dx <= min_dist_x_) continue;
                        if (std::abs(bars[i].center.y - bars[j].center.y) > 50) continue;

                        double ddeg = std::abs(bars[i].tilt - bars[j].tilt)
                                      * 180.0 / CV_PI;
                        if (ddeg >= max_angle_diff_) continue;           // 平行度

                        double avg_angle = 0.5 * (bars[i].tilt + bars[j].tilt);
                        double corrected = dx / std::max(0.2, std::abs(std::cos(avg_angle)));
                        double long_h = std::max(bars[i].long_h, bars[j].long_h);
                        if (long_h < 1.0) continue;
                        double ratio = corrected / long_h;
                        if (ratio > ratio_l_max_ || ratio < ratio_min_) continue; // 幽灵

                        bool is_large = (ratio > ratio_s_max_);

                        // ---------- 极值中点定厂字框 ----------
                        const Bar& longer = (bars[i].long_h >= bars[j].long_h)
                                            ? bars[i] : bars[j];
                        int min_x = 1 << 30, max_x = -1;
                        for (int s = 0; s < 2; ++s) {
                            const std::vector<cv::Point>& c =
                                (s == 0) ? bars[i].contour : bars[j].contour;
                            for (size_t k = 0; k < c.size(); ++k) {
                                min_x = std::min(min_x, c[k].x);
                                max_x = std::max(max_x, c[k].x);
                            }
                        }
                        int min_y = 1 << 30, max_y = -1;
                        for (size_t k = 0; k < 4; ++k) {
                            min_y = std::min(min_y, (int)longer.pts[k].y);
                            max_y = std::max(max_y, (int)longer.pts[k].y);
                        }

                        Target tg;
                        tg.id = armor_id++;
                        tg.is_large = is_large;
                        tg.ratio = ratio;
                        tg.avg_tilt = avg_angle;
                        tg.corners.push_back(cv::Point2f(min_x, min_y));
                        tg.corners.push_back(cv::Point2f(max_x, min_y));
                        tg.corners.push_back(cv::Point2f(max_x, max_y));
                        tg.corners.push_back(cv::Point2f(min_x, max_y));
                        tg.center2d = cv::Point2f((min_x + max_x) * 0.5f,
                                                  (min_y + max_y) * 0.5f);

                        // 4) 相对位置验证：配对中心必须在中上部
                        if (pair_y_ratio_ > 0.0 &&
                            tg.center2d.y > height_ * pair_y_ratio_) continue;
                        tg.has_pose = false;
                        tg.z = -1.0;

                        // 去重：中心距离 <25px 视为同一块板，避免同一根灯条被拆段后重复配对
                        bool dup = false;
                        for (size_t d = 0; d < targets.size(); ++d) {
                            if (cv::norm(targets[d].center2d - tg.center2d) < 120.0) {
                                dup = true; break;
                            }
                        }
                        if (dup) continue;

                        // ---------- solvePnP ----------
                        if (K_.rows == 3 && K_.cols == 3) {
                            double half_w = is_large ? 0.1125 : 0.0675;
                            double half_h = 0.0275;
                            std::vector<cv::Point3f> obj;
                            obj.push_back(cv::Point3f(-half_w,  half_h, 0));
                            obj.push_back(cv::Point3f( half_w,  half_h, 0));
                            obj.push_back(cv::Point3f( half_w, -half_h, 0));
                            obj.push_back(cv::Point3f(-half_w, -half_h, 0));

                            cv::Mat rvec, tvec;
                            bool ok = cv::solvePnP(obj, tg.corners, K_, D_,
                                                   rvec, tvec, false, cv::SOLVEPNP_IPPE);
                            if (!ok)
                                ok = cv::solvePnP(obj, tg.corners, K_, D_,
                                                  rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
                            if (ok) {
                                tg.has_pose = true;
                                tg.t = cv::Vec3d(tvec.at<double>(0),
                                                 tvec.at<double>(1),
                                                 tvec.at<double>(2));
                                tg.z = tg.t[2];
                                cv::Mat R;
                                cv::Rodrigues(rvec, R);
                                cv::Mat S = (cv::Mat_<double>(3, 3) <<
                                             1, 0, 0, 0, -1, 0, 0, 0, 1);
                                cv::Mat R2 = S * R * S;
                                tg.yaw = std::atan2(R2.at<double>(1, 0),
                                                    R2.at<double>(0, 0));
                            }
                        }

                        targets.push_back(tg);
                    }
                }

                // =======================================================
                // 3. 时空校验 + 择优 + 车体中心 + TF
                // =======================================================
                if (!targets.empty()) {
                    // ---- 众数滤波（过去 10 帧 7 票）----
                    {
                        Target& first = targets[0];
                        size_hist_.push_back(first.is_large);
                        while (size_hist_.size() > 10) size_hist_.pop_front();
                        int big = 0;
                        for (size_t k = 0; k < size_hist_.size(); ++k)
                            if (size_hist_[k]) ++big;
                        int small = static_cast<int>(size_hist_.size()) - big;
                        if (size_hist_.size() >= 10) {
                            if (big >= 7) first.is_large = true;
                            else if (small >= 7) first.is_large = false;
                        }
                    }

                    // ---- 时间重叠 / 尺寸突变沿用上一帧 ----
                    if (has_last_ && (stamp - last_stamp_) >= 0.0 &&
                        (stamp - last_stamp_) < uniqueness_dt_) {
                        if (targets[0].is_large != last_is_large_)
                            targets[0].is_large = last_is_large_;
                    }

                    // ---- 超距 / 瞬移 ----
                    std::vector<Target> keep;
                    for (size_t k = 0; k < targets.size(); ++k) {
                        if (!targets[k].has_pose) { keep.push_back(targets[k]); continue; }
                        if (targets[k].z > max_z_) {
                            ROS_INFO_THROTTLE(1.0, "[Drop] 超距 z=%.2f m", targets[k].z);
                            continue;
                        }
                        if (has_last_ && (stamp - last_stamp_) > 1e-4 &&
                            (stamp - last_stamp_) < 0.5) {
                            double dt = stamp - last_stamp_;
                            double fx = K_.at<double>(0, 0);
                            double max_px = fx * max_speed_ * dt /
                                            std::max(0.5, targets[k].z) * 1.5 + 20.0;
                            double d = cv::norm(targets[k].center2d - last_center2d_);
                            if (d > max_px) {
                                ROS_INFO_THROTTLE(1.0, "[Drop] 瞬移 %.1f px > %.1f", d, max_px);
                                continue;
                            }
                        }
                        keep.push_back(targets[k]);
                    }
                    targets.swap(keep);
                }

                if (!targets.empty()) {
                    lost_this_frame = 0;
                    lost_frames_ = 0;


                    // ---- 二维平行度择优（最正对）----
                    int best = 0;
                    for (size_t k = 1; k < targets.size(); ++k)
                        if (std::abs(targets[k].avg_tilt) < std::abs(targets[best].avg_tilt))
                            best = static_cast<int>(k);

                    // ---- 整车中心聚合 ----
                    cv::Point2f vc(0, 0);
                    for (size_t k = 0; k < targets.size(); ++k) vc += targets[k].center2d;
                    vc *= (1.0f / targets.size());

                    // ---- 主目标平滑 + 下坠补偿 + TF ----
                    Target& mt = targets[best];
                    if (mt.has_pose) {
                        // 离群 z 拒绝：与中值差 > 1.5m 的目标不进 TF
                        bool z_outlier = false;
                        double z_med_ref = mt.z;
                        if (z_hist_.size() >= 3) {
                            std::vector<double> zs(z_hist_.begin(), z_hist_.end());
                            std::sort(zs.begin(), zs.end());
                            z_med_ref = zs[zs.size() / 2];
                            if (std::abs(mt.z - z_med_ref) > max_z_jump_)
                                z_outlier = true;
                        }
                        if (z_outlier) {
                            ROS_INFO_THROTTLE(1.0, "[Drop] z 离群 %.2f (中值 %.2f)",
                                              mt.z, z_med_ref);
                        } else {
                        cv::Vec3d t2(mt.t[0], -mt.t[1], mt.t[2]);

                        cv::Vec3d avg_t;
                        double avg_yaw = 0.0;
                        pushSmooth(t2, mt.yaw, avg_t, avg_yaw);

                        // z 中值滤波（窗口 13），压测距抖动
                        if (mt.id != last_z_id_ || mt.is_large != last_z_large_) {
                            z_hist_.clear();
                            last_z_id_ = mt.id;
                            last_z_large_ = mt.is_large;
                        }
                        z_hist_.push_back(mt.z);
                        while (static_cast<int>(z_hist_.size()) > z_median_win_)
                            z_hist_.pop_front();
                        {
                            std::vector<double> zs(z_hist_.begin(), z_hist_.end());
                            std::sort(zs.begin(), zs.end());
                            avg_t[2] = zs[zs.size() / 2];
                        }

                        // ---- 坐标死区 ----
                        bool moved = true;
                        if (has_sent_)
                            moved = cv::norm(mt.center2d - last_sent_px_) > deadzone_px_;

                        if (moved) {
                            geometry_msgs::TransformStamped ts;
                            ts.header.stamp = msg->header.stamp;
                            ts.header.frame_id = camera_frame_;
                            ts.child_frame_id = "armor_v3";
                            ts.transform.translation.x = avg_t[0];
                            ts.transform.translation.y = avg_t[1];
                            ts.transform.translation.z = avg_t[2];

                            tf2::Quaternion q;
                            q.setRPY(0.0, 0.0, avg_yaw);
                            ts.transform.rotation.x = q.x();
                            ts.transform.rotation.y = q.y();
                            ts.transform.rotation.z = q.z();
                            ts.transform.rotation.w = q.w();
                            tf_broadcaster_.sendTransform(ts);

                            last_sent_px_ = mt.center2d;
                            has_sent_ = true;
                        }

                        last_stamp_ = stamp;
                        last_center2d_ = mt.center2d;
                        last_is_large_ = mt.is_large;
                        last_pos_ = avg_t;
                        last_yaw_ = avg_yaw;
                        has_last_ = true;
                        }
                    }

                    // ---- 绘图 ----
                    for (size_t k = 0; k < targets.size(); ++k) {
                        const Target& tg = targets[k];
                        int thick = (static_cast<int>(k) == best) ? 4 : 2;
                        cv::rectangle(canvas,
                                      cv::Rect((int)tg.corners[0].x, (int)tg.corners[0].y,
                                               (int)(tg.corners[1].x - tg.corners[0].x),
                                               (int)(tg.corners[3].y - tg.corners[0].y)),
                                      cv::Scalar(0, 0, 255), thick);
                        for (int c = 0; c < 4; ++c)
                            cv::circle(canvas, tg.corners[c], 3, cv::Scalar(255, 0, 0), -1);

                        std::string info = "ID:" + std::to_string(tg.id) +
                                           (tg.is_large ? " L" : " S");
                        cv::putText(canvas, info,
                                    cv::Point((int)tg.corners[0].x,
                                              std::max(0, (int)tg.corners[0].y - 10)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                    cv::Scalar(0, 0, 255), thick > 2 ? 2 : 1);
                    }

                    // 车体中心十字星
                    cv::line(canvas, cv::Point((int)vc.x - 12, (int)vc.y),
                             cv::Point((int)vc.x + 12, (int)vc.y),
                             cv::Scalar(255, 255, 0), 2);
                    cv::line(canvas, cv::Point((int)vc.x, (int)vc.y - 12),
                             cv::Point((int)vc.x, (int)vc.y + 12),
                             cv::Scalar(255, 255, 0), 2);

                    if (mt.has_pose) {
                        // 离群 z 拒绝：与中值差 > 1.5m 的目标不进 TF
                        bool z_outlier = false;
                        double z_med_ref = mt.z;
                        if (z_hist_.size() >= 3) {
                            std::vector<double> zs(z_hist_.begin(), z_hist_.end());
                            std::sort(zs.begin(), zs.end());
                            z_med_ref = zs[zs.size() / 2];
                            if (std::abs(mt.z - z_med_ref) > max_z_jump_)
                                z_outlier = true;
                        }
                        if (z_outlier) {
                            ROS_INFO_THROTTLE(1.0, "[Drop] z 离群 %.2f (中值 %.2f)",
                                              mt.z, z_med_ref);
                        } else {
                        cv::Vec3d t2(mt.t[0], -mt.t[1], mt.t[2]);

                        cv::Vec3d avg_t;
                        double avg_yaw = 0.0;
                        pushSmooth(t2, mt.yaw, avg_t, avg_yaw);

                        // z 中值滤波（窗口 13），压测距抖动
                        if (mt.id != last_z_id_ || mt.is_large != last_z_large_) {
                            z_hist_.clear();
                            last_z_id_ = mt.id;
                            last_z_large_ = mt.is_large;
                        }
                        z_hist_.push_back(mt.z);
                        while (static_cast<int>(z_hist_.size()) > z_median_win_)
                            z_hist_.pop_front();
                        {
                            std::vector<double> zs(z_hist_.begin(), z_hist_.end());
                            std::sort(zs.begin(), zs.end());
                            avg_t[2] = zs[zs.size() / 2];
                        }

                        // ---- 坐标死区 ----
                        bool moved = true;
                        if (has_sent_)
                            moved = cv::norm(mt.center2d - last_sent_px_) > deadzone_px_;

                        if (moved) {
                            geometry_msgs::TransformStamped ts;
                            ts.header.stamp = msg->header.stamp;
                            ts.header.frame_id = camera_frame_;
                            ts.child_frame_id = "armor_v3";
                            ts.transform.translation.x = avg_t[0];
                            ts.transform.translation.y = avg_t[1];
                            ts.transform.translation.z = avg_t[2];

                            tf2::Quaternion q;
                            q.setRPY(0.0, 0.0, avg_yaw);
                            ts.transform.rotation.x = q.x();
                            ts.transform.rotation.y = q.y();
                            ts.transform.rotation.z = q.z();
                            ts.transform.rotation.w = q.w();
                            tf_broadcaster_.sendTransform(ts);

                            last_sent_px_ = mt.center2d;
                            has_sent_ = true;
                        }

                        last_stamp_ = stamp;
                        last_center2d_ = mt.center2d;
                        last_is_large_ = mt.is_large;
                        last_pos_ = avg_t;
                        last_yaw_ = avg_yaw;
                        has_last_ = true;
                        }
                    }

                    // ---- 绘图 ----
                    for (size_t k = 0; k < targets.size(); ++k) {
                        const Target& tg = targets[k];
                        int thick = (static_cast<int>(k) == best) ? 4 : 2;
                        cv::rectangle(canvas,
                                      cv::Rect((int)tg.corners[0].x, (int)tg.corners[0].y,
                                               (int)(tg.corners[1].x - tg.corners[0].x),
                                               (int)(tg.corners[3].y - tg.corners[0].y)),
                                      cv::Scalar(0, 0, 255), thick);
                        for (int c = 0; c < 4; ++c)
                            cv::circle(canvas, tg.corners[c], 3, cv::Scalar(255, 0, 0), -1);

                        std::string info = "ID:" + std::to_string(tg.id) +
                                           (tg.is_large ? " L" : " S");
                        cv::putText(canvas, info,
                                    cv::Point((int)tg.corners[0].x,
                                              std::max(0, (int)tg.corners[0].y - 10)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                    cv::Scalar(0, 0, 255), thick > 2 ? 2 : 1);
                    }

                    // 车体中心十字星
                    cv::line(canvas, cv::Point((int)vc.x - 12, (int)vc.y),
                             cv::Point((int)vc.x + 12, (int)vc.y),
                             cv::Scalar(255, 255, 0), 2);
                    cv::line(canvas, cv::Point((int)vc.x, (int)vc.y - 12),
                             cv::Point((int)vc.x, (int)vc.y + 12),
                             cv::Scalar(255, 255, 0), 2);

                    if (mt.has_pose) {
                        ROS_INFO_THROTTLE(1.0,
                            "v3 id=%d %s ratio=%.2f z=%.2f yaw=%.1f",
                            mt.id, mt.is_large ? "L" : "S", mt.ratio, mt.z,
                            mt.yaw * 180.0 / CV_PI);
                    }

                    // ---- 记录上一帧成功状态，供 FCS 残影补位使用 ----
                    {
                        int bx1 = 1 << 30, by1 = 1 << 30, bx2 = -1, by2 = -1;
                        for (int k = 0; k < 4; ++k) {
                            bx1 = std::min(bx1, (int)mt.corners[k].x);
                            by1 = std::min(by1, (int)mt.corners[k].y);
                            bx2 = std::max(bx2, (int)mt.corners[k].x);
                            by2 = std::max(by2, (int)mt.corners[k].y);
                        }
                        cv::Rect mr(bx1, by1, std::max(1, bx2 - bx1), std::max(1, by2 - by1));
                        prev_had_target_ = true;
                        prev_id_ = mt.id;
                        prev_is_large_ = mt.is_large;
                        prev_center2d_ = mt.center2d;
                        prev_stamp_ = stamp;
                        prev_aspect_ = (double)std::max(mr.width, mr.height) /
                                       std::max(1, std::min(mr.width, mr.height));
                        double bar_ratio = 1.0;
                        for (size_t bi = 0; bi < bars.size(); ++bi) {
                            cv::Point2f e1 = bars[bi].pts[1] - bars[bi].pts[0];
                            cv::Point2f e2 = bars[bi].pts[2] - bars[bi].pts[1];
                            double a = cv::norm(e1), b2 = cv::norm(e2);
                            if (a > 1 && b2 > 1)
                                bar_ratio = std::max(bar_ratio,
                                                     std::max(a, b2) / std::min(a, b2));
                        }
                        prev_bar_ratio_ = bar_ratio;
                        prev_grad_ = brightGradient(v_chan, mr);
                        fourBrightness(v_chan, mr, prev_four_);
                    }
                } else {
                    // ===================================================
                    // 纯白掩膜兜底：两条白灯条强行配对
                    // ===================================================
                    bool white_ok = false;
                    // 白场兜底：属于极端场景（近战过曝纯白），默认关闭，_white_enable:=true 开启
                    if (white_enable_) {
                        cv::Mat white_mask;
                        cv::inRange(hsv_full,
                                    cv::Scalar(0, 0, white_v_min_),
                                    cv::Scalar(180, white_s_max_, 255),
                                    white_mask);
                        std::vector<std::vector<cv::Point> > wcnts;
                        cv::findContours(white_mask, wcnts, cv::RETR_EXTERNAL,
                                         cv::CHAIN_APPROX_SIMPLE);
                        struct WB { cv::Point2f c; double tilt, long_h, ratio; std::vector<cv::Point> cnt; };
                        std::vector<WB> wb;
                        for (size_t wi = 0; wi < wcnts.size(); ++wi) {
                            if (cv::contourArea(wcnts[wi]) < white_area_min_) continue;
                            cv::RotatedRect rr = cv::minAreaRect(wcnts[wi]);
                            double w = rr.size.width, h = rr.size.height;
                            if (std::min(w, h) <= 0) continue;
                            double asp = std::max(w, h) / std::min(w, h);
                            if (asp < white_aspect_min_ || asp > white_aspect_max_) continue;
                            cv::Moments m = cv::moments(wcnts[wi]);
                            if (std::abs(m.m00) < 1e-6) continue;
                            cv::Point2f pts[4]; rr.points(pts);
                            cv::Point2f e1 = pts[1] - pts[0], e2 = pts[2] - pts[1];
                            cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
                            double tilt = std::atan2(lv.x, lv.y);
                            if (tilt > CV_PI / 2) tilt -= CV_PI;
                            if (tilt < -CV_PI / 2) tilt += CV_PI;
                            WB b;
                            b.c = cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));
                            b.tilt = tilt;
                            b.long_h = std::max(cv::norm(e1), cv::norm(e2));
                            b.ratio = asp;
                            b.cnt = wcnts[wi];
                            wb.push_back(b);
                        }
                        if (wb.size() == 2) {
                            double dx = std::abs(wb[1].c.x - wb[0].c.x);
                            double dy = std::abs(wb[1].c.y - wb[0].c.y);
                            double ddeg = std::abs(wb[0].tilt - wb[1].tilt) * 180.0 / CV_PI;
                            double ang = 0.5 * (wb[0].tilt + wb[1].tilt);
                            double corrected = dx / std::max(0.2, std::abs(std::cos(ang)));
                            double lh = std::max(wb[0].long_h, wb[1].long_h);
                            double ratio = corrected / std::max(1.0, lh);
                            if (dx > 15 && dx <= 300 && dy <= 60 && ddeg < 12 &&
                                ratio >= 0.4 && ratio <= 2.5) {
                                cv::Rect b0 = cv::boundingRect(wb[0].cnt);
                                cv::Rect b1 = cv::boundingRect(wb[1].cnt);
                                int min_x = std::min(b0.x, b1.x);
                                int min_y = std::min(b0.y, b1.y);
                                int max_x = std::max(b0.x + b0.width, b1.x + b1.width);
                                int max_y = std::max(b0.y + b0.height, b1.y + b1.height);
                                bool is_large = ratio > 1.3;
                                int wid = prev_had_target_ ? prev_id_ : 1;

                                for (int s2 = 0; s2 < 2; ++s2) {
                                    const std::vector<cv::Point>& cc = (s2 == 0) ? wb[0].cnt : wb[1].cnt;
                                    cv::RotatedRect rr2 = cv::minAreaRect(cc);
                                    cv::Point2f p2[4]; rr2.points(p2);
                                    for (int k = 0; k < 4; ++k)
                                        cv::line(canvas, p2[k], p2[(k + 1) % 4],
                                                 cv::Scalar(0, 255, 255), 2);
                                }
                                cv::rectangle(canvas, cv::Rect(min_x, min_y,
                                                               max_x - min_x, max_y - min_y),
                                              cv::Scalar(0, 0, 255), 2);
                                cv::putText(canvas,
                                            "[WHITE] ID:" + std::to_string(wid) +
                                            (is_large ? " L" : " S"),
                                            cv::Point(min_x, std::max(0, min_y - 8)),
                                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                            cv::Scalar(0, 255, 255), 2);
                                ROS_INFO_THROTTLE(0.5, "[WHITE] 白场兜底配对 ratio=%.2f", ratio);

                                prev_had_target_ = true;
                                prev_id_ = wid;
                                prev_is_large_ = is_large;
                                prev_center2d_ = cv::Point2f((min_x + max_x) * 0.5f,
                                                             (min_y + max_y) * 0.5f);
                                prev_stamp_ = stamp;
                                prev_aspect_ = (double)std::max(max_x - min_x, max_y - min_y) /
                                               std::max(1, std::min(max_x - min_x, max_y - min_y));
                                prev_grad_ = brightGradient(v_chan,
                                                cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y));
                                fourBrightness(v_chan,
                                    cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y), prev_four_);
                                white_ok = true;
                                lost_frames_ = 0;
                            }
                        }
                    }

                    // ===================================================
                    // 模块6：目标丢失重捕与残影补位（FCS）
                    // ===================================================
                    bool fcs_ok = false;
                    double dt_fcs = stamp - prev_stamp_;
                    if (prev_had_target_ && bars.size() <= 2 &&
                        dt_fcs > 0.0 && dt_fcs < 2.0) {
                        float best_dist = 1e9f;
                        cv::Rect best_rect;
                        int best_idx = -1;
                        for (size_t ci = 0; ci < contours.size(); ++ci) {
                            if (cv::contourArea(contours[ci]) < 150) continue;
                            std::vector<cv::Point> poly;
                            cv::approxPolyDP(contours[ci], poly,
                                             0.02 * cv::arcLength(contours[ci], true), true);
                            if (poly.size() != 4) continue;
                            cv::Rect r = cv::boundingRect(poly);
                            if (r.width < 8 || r.height < 5) continue;
                            double asp = (double)std::max(r.width, r.height) /
                                         std::max(1, std::min(r.width, r.height));
                            bool ok_armor = (asp >= 0.5 * prev_aspect_ && asp <= 1.8 * prev_aspect_);
                            bool ok_bar   = (asp >= 0.6 * prev_bar_ratio_ &&
                                             asp <= 1.6 * prev_bar_ratio_);
                            if (!ok_armor && !ok_bar) continue;
                            cv::Point2f c(r.x + r.width * 0.5f, r.y + r.height * 0.5f);
                            float d = (float)cv::norm(c - prev_center2d_);
                            if (d > 400.0f) continue;
                            double grad = brightGradient(v_chan, r);
                            if (std::abs(grad - prev_grad_) > 45.0) continue;

                            // 四点亮度一致性：四个光斑核心互相近似，且与上一帧渐变一致
                            double c4[4];
                            fourBrightness(v_chan, r, c4);
                            double mn = c4[0], mx = c4[0];
                            for (int q = 1; q < 4; ++q) {
                                mn = std::min(mn, c4[q]);
                                mx = std::max(mx, c4[q]);
                            }
                            if (mx - mn > fcs_four_span_) continue;
                            bool four_ok = true;
                            for (int q = 0; q < 4; ++q) {
                                if (std::abs(c4[q] - prev_four_[q]) > fcs_four_tol_) {
                                    four_ok = false; break;
                                }
                            }
                            if (!four_ok) continue;

                            if (d < best_dist) { best_dist = d; best_rect = r; best_idx = (int)ci; }
                        }
                        if (best_idx >= 0) {
                            dashedRect(canvas, best_rect, cv::Scalar(0, 255, 255), 2);
                            std::string fcs = "[FCS] LOCK ID:" + std::to_string(prev_id_) +
                                              (prev_is_large_ ? " L" : " S");
                            cv::putText(canvas, fcs,
                                        cv::Point(best_rect.x, std::max(0, best_rect.y - 8)),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                        cv::Scalar(0, 255, 255), 2);
                            ROS_INFO_THROTTLE(0.5, "[FCS] 遮挡补位中 id=%d %s",
                                              prev_id_, prev_is_large_ ? "L" : "S");
                            prev_center2d_ = cv::Point2f(best_rect.x + best_rect.width * 0.5f,
                                                         best_rect.y + best_rect.height * 0.5f);
                            prev_stamp_ = stamp;
                            fcs_ok = true;
                            lost_frames_ = 0;
                        }
                    }

                    if (!fcs_ok) {
                        ++lost_frames_;
                        ROS_INFO_THROTTLE(1.0, "[Hold] 丢帧 %d/5", lost_frames_);
                        if (lost_frames_ >= 5) {
                            has_last_ = false; has_sent_ = false;
                            pos_queue_.clear(); yaw_queue_.clear();
                            size_hist_.clear(); yaw_hist_.clear();
                            z_hist_.clear();
                            lost_frames_ = 0;
                        }
                    }
                }
            } else {
                ++lost_frames_;
            }

            // 兜底模式可视化标记
            if (assist_mode_) {
                cv::putText(canvas, "ASSIST", cv::Point(70, 32),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            }

            cv_bridge::CvImage out_img;
            out_img.header.stamp = msg->header.stamp;
            out_img.header.frame_id = msg->header.frame_id;
            out_img.encoding = "bgr8";
            out_img.image = canvas;
            image_pub_.publish(out_img.toImageMsg());
            std::cout << "ok" << std::endl;

        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge: %s", e.what());
        }
    }

    // ---------- 成员 ----------
    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    ros::Subscriber info_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 动态参数（22 点之 13）
    double aspect_min_, aspect_max_;
    double min_dist_x_, max_dist_x_, max_angle_diff_;
    double ratio_s_max_, ratio_l_max_, ratio_min_;
    double max_z_, deadzone_px_, uniqueness_dt_, max_speed_;
    int morph_size_;
    int hsv_sv_min_;
    double roi_y_ratio_;
    int yellow_h_min_, yellow_h_max_, yellow_s_min_, yellow_v_min_;
    int bar_v_min_, bar_keep_n_;
    int z_median_win_;
    double max_z_jump_;
    int    near_area_, near_sv_;
    bool   edge_fuse_, merge_bars_;
    double guard_y_ratio_, dyn_v_offset_, pair_y_ratio_;
    double fcs_four_span_, fcs_four_tol_;
    int blue_h_min_, blue_h_max_, blue_s_min_, blue_v_min_;
    int white_v_min_, white_s_max_, white_area_min_;
    double white_aspect_min_, white_aspect_max_;
    bool   white_enable_;

    // 状态
    std::deque<cv::Vec3d> pos_queue_;
    std::deque<double> z_hist_;
    int  last_z_id_ = -1;
    bool last_z_large_ = false;
    std::deque<double> yaw_queue_;
    std::deque<bool> size_hist_;
    std::deque<std::pair<double, double> > yaw_hist_;

    bool has_last_;
    double last_stamp_ = 0.0;
    cv::Point2f last_center2d_ = cv::Point2f(0, 0);
    bool last_is_large_ = false;
    cv::Vec3d last_pos_ = cv::Vec3d(0, 0, 0);
    double last_yaw_ = 0.0;
    double omega_deg_s_ = 0.0;
    int lost_frames_ = 0;
    bool assist_mode_ = false;
    int  assist_left_ = 0;
    int  assist_lost_ = 0;

    // FCS 残影补位状态
    bool        prev_had_target_ = false;
    int         prev_id_ = 0;
    bool        prev_is_large_ = false;
    cv::Point2f prev_center2d_ = cv::Point2f(0, 0);
    double      prev_stamp_ = 0.0;
    double      prev_aspect_ = 2.0;
    double      prev_grad_ = 0.0;
    double      prev_bar_ratio_ = 5.0;
    double      prev_four_[4] = {0, 0, 0, 0};

    bool has_sent_ = false;
    cv::Point2f last_sent_px_ = cv::Point2f(0, 0);

    std::atomic<bool> is_ready_;
    bool transport_ready_;

    cv::Mat K_, D_;
    int width_ = 0, height_ = 0;
    std::string camera_frame_, distortion_model_;
};

// ============================================================
//  C. colorSegment（与 v1 相同逻辑，参数化）
// ============================================================
inline void colorSegment(const cv::Mat& bgr, cv::Mat& res,
                         const cv::Scalar& lo_r, const cv::Scalar& hi_r,
                         const cv::Scalar& lo_b, const cv::Scalar& hi_b,
                         const cv::Scalar& lo_g, const cv::Scalar& hi_g,
                         int morph_size, int sv_min,
                         int y_h_min, int y_h_max, int y_s_min, int y_v_min) {
    if (bgr.empty()) { res = cv::Mat(); return; }

    cv::Mat hsv;
    cv::Mat mask_r1, mask_r2, mask_r, mask_b, mask_g, mask_all;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, lo_r, hi_r, mask_r1);
    cv::inRange(hsv, cv::Scalar(170, sv_min, sv_min), cv::Scalar(179, 255, 255), mask_r2);
    cv::bitwise_or(mask_r1, mask_r2, mask_r);

    cv::inRange(hsv, lo_b, hi_b, mask_b);
    cv::inRange(hsv, lo_g, hi_g, mask_g);

    // 黄色/橙色（H 10~40 可调）
    cv::Mat mask_y;
    cv::inRange(hsv, cv::Scalar(y_h_min, y_s_min, y_v_min),
                     cv::Scalar(y_h_max, 255, 255), mask_y);

    cv::bitwise_or(mask_r, mask_b, mask_all);
    cv::bitwise_or(mask_all, mask_g, mask_all);
    cv::bitwise_or(mask_all, mask_y, mask_all);

    // C1：3x3 + MORPH_OPEN
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(morph_size, morph_size));
    cv::morphologyEx(mask_all, mask_all, cv::MORPH_OPEN, kernel);

    cv::bitwise_and(bgr, bgr, res, mask_all);
}

// ============================================================
//  D. main
// ============================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_vision_node_v3");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    VisionNodeV3 node(nh, pnh);

    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(30.0);
    ros::Rate rate(20.0);
    while (ros::ok() && !node.isReady()) {
        ros::spinOnce();
        if (ros::WallTime::now() > deadline) {
            ROS_FATAL("启动超时: camera_info 未到达 / compressed 插件缺失");
            return 1;
        }
        ROS_INFO_THROTTLE(2.0, "等待 CameraInfo / 插件就绪...");
        rate.sleep();
    }

    ROS_INFO("VisionNodeV3 就绪，开始处理图像！");
    ros::spin();
    return 0;
}
