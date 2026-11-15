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
                         int morph_size, int sv_min);

// ============================================================
//  A. VisionNodeV3：ROS 节点
// ============================================================
class VisionNodeV3 {
public:
    VisionNodeV3(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh),
          is_ready_(false), transport_ready_(true),
          tf_broadcaster_(), has_last_(false), lost_frames_(0),
          has_sent_(false), blind_(false)
    {
        // ---------- 动态参数（22 点之 13） ----------
        pnh_.param("aspect_min",  aspect_min_, 2.0);
        pnh_.param("aspect_max",  aspect_max_, 6.0);
        pnh_.param("min_dist_x",  min_dist_x_, 15.0);
        pnh_.param("max_dist_x",  max_dist_x_, 250.0);
        pnh_.param("max_angle_diff", max_angle_diff_, 10.0);
        pnh_.param("ratio_s_max", ratio_s_max_, 1.3);
        pnh_.param("ratio_l_max", ratio_l_max_, 2.5);
        pnh_.param("ratio_min",   ratio_min_, 0.4);
        pnh_.param("max_z",       max_z_, 8.0);
        pnh_.param("deadzone_px", deadzone_px_, 5.0);
        pnh_.param("uniqueness_dt", uniqueness_dt_, 0.15);
        pnh_.param("max_speed",   max_speed_, 8.0);     // 车体物理极限 m/s
        pnh_.param("omega_blind", omega_blind_, 300.0); // 盲射阈值 deg/s
        pnh_.param("morph_size",  morph_size_, 3);
        pnh_.param("hsv_sv_min",   hsv_sv_min_, 150);

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

    // ---------- 查表法下坠补偿（22 点之 21） ----------
    double dropCompensation(double z, double* flight_t) {
        static const double zs[]   = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        static const double ts[]   = {0.03, 0.06, 0.09, 0.12, 0.15, 0.18, 0.21, 0.24};
        static const double angs[] = {0.10, 0.35, 0.75, 1.30, 2.00, 2.85, 3.85, 5.00};
        const int n = 8;
        if (z <= zs[0]) { if (flight_t) *flight_t = ts[0]; return z * std::tan(angs[0] * CV_PI / 180.0); }
        if (z >= zs[n - 1]) { if (flight_t) *flight_t = ts[n - 1]; return z * std::tan(angs[n - 1] * CV_PI / 180.0); }
        for (int i = 0; i < n - 1; ++i) {
            if (z >= zs[i] && z <= zs[i + 1]) {
                double k = (z - zs[i]) / (zs[i + 1] - zs[i]);
                double ang = angs[i] + k * (angs[i + 1] - angs[i]);
                double t   = ts[i]   + k * (ts[i + 1] - ts[i]);
                if (flight_t) *flight_t = t;
                return z * std::tan(ang * CV_PI / 180.0);
            }
        }
        return 0.0;
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
                         cv::Scalar(110, hsv_sv_min_, hsv_sv_min_), cv::Scalar(130, 255, 255),
                         cv::Scalar(50, hsv_sv_min_, hsv_sv_min_), cv::Scalar(70, 255, 255),
                         morph_size_, hsv_sv_min_);

            const double stamp = msg->header.stamp.toSec();
            int lost_this_frame = 1;

            if (!colored.empty()) {
                // 显示用底板：保留原始相机画面（不再用分割结果覆盖成黑底）
                // canvas 仍是 img.clone() + 绿框，检测/绘制都叠加在原视频上
                // =======================================================
                // 1. 灯条提取：矩质心 + 长宽比 2.0~6.0
                // =======================================================
                std::vector<Bar> bars;
                std::vector<std::vector<cv::Point>> contours;
                cv::Mat seg_mask;
                cv::cvtColor(colored, seg_mask, cv::COLOR_BGR2GRAY);
                cv::findContours(seg_mask, contours, cv::RETR_EXTERNAL,
                                 cv::CHAIN_APPROX_SIMPLE);

                for (size_t i = 0; i < contours.size(); ++i) {
                    if (cv::contourArea(contours[i]) < 20) continue;
                    cv::RotatedRect rr = cv::minAreaRect(contours[i]);
                    double w = rr.size.width, h = rr.size.height;
                    if (std::min(w, h) <= 0) continue;
                    double aspect = std::max(w, h) / std::min(w, h);
                    if (aspect < aspect_min_ || aspect > aspect_max_) continue;

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
                    bar.contour = contours[i];
                    bars.push_back(bar);

                    for (int k = 0; k < 4; ++k)
                        cv::line(canvas, bar.pts[k], bar.pts[(k + 1) % 4],
                                 cv::Scalar(0, 255, 255), 2);
                }

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
                        tg.has_pose = false;
                        tg.z = -1.0;

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
                // 3. 时空校验 + 择优 + 车体中心 + 预测 + TF
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
                        cv::Vec3d t2(mt.t[0], -mt.t[1], mt.t[2]);
                        double flight_t = 0.0;
                        double drop = dropCompensation(mt.z, &flight_t);
                        t2[1] += drop;   // 叠加到最终发送的 Y 轴

                        cv::Vec3d avg_t;
                        double avg_yaw = 0.0;
                        pushSmooth(t2, mt.yaw, avg_t, avg_yaw);

                        // ---- 旋转角速度 Omega（滑动窗口 0.6s）----
                        yaw_hist_.push_back(std::make_pair(stamp, avg_yaw));
                        while (!yaw_hist_.empty() && stamp - yaw_hist_.front().first > 0.6)
                            yaw_hist_.pop_front();
                        double omega = 0.0;
                        if (yaw_hist_.size() >= 3) {
                            double dt = yaw_hist_.back().first - yaw_hist_.front().first;
                            if (dt > 1e-3) {
                                double dyaw = avg_yaw - yaw_hist_.front().second;
                                while (dyaw > CV_PI / 2) dyaw -= CV_PI;
                                while (dyaw < -CV_PI / 2) dyaw += CV_PI;
                                omega = dyaw / dt * 180.0 / CV_PI;
                            }
                        }
                        omega_deg_s_ = omega;

                        // ---- 高转速盲射 ----
                        if (std::abs(omega) > omega_blind_) {
                            blind_ = true;
                            ROS_INFO_THROTTLE(1.0, "[Blind] Omega=%.0f deg/s 锁车体中心", omega);
                        } else {
                            blind_ = false;
                        }

                        // ---- 坐标死区 ----
                        bool moved = true;
                        if (has_sent_)
                            moved = cv::norm(mt.center2d - last_sent_px_) > deadzone_px_;

                        if (!blind_ && moved) {
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

                    // ---- 预测下一块板（T = 90 / Omega）----
                    double t_pred = -1.0;
                    cv::Point2f pred_center = mt.center2d;
                    if (std::abs(omega_deg_s_) > 30.0) {
                        t_pred = 90.0 / std::abs(omega_deg_s_);
                        if (t_pred < 3.0) {
                            float sign = (omega_deg_s_ > 0) ? 1.0f : -1.0f;
                            pred_center.x += sign * 120.0f;
                        } else t_pred = -1.0;
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

                    // 预测虚线框
                    if (t_pred > 0) {
                        int w = (int)(mt.corners[1].x - mt.corners[0].x);
                        int h = (int)(mt.corners[3].y - mt.corners[0].y);
                        cv::Rect pr((int)pred_center.x - w / 2,
                                    (int)pred_center.y - h / 2, w, h);
                        dashedRect(canvas, pr, cv::Scalar(255, 0, 255), 2);
                        cv::putText(canvas, "PRED " + std::to_string((int)(t_pred * 1000)) + "ms",
                                    cv::Point(pr.x, std::max(0, pr.y - 8)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                                    cv::Scalar(255, 0, 255), 1);
                    }

                    if (mt.has_pose) {
                        ROS_INFO_THROTTLE(1.0,
                            "v3 id=%d %s ratio=%.2f z=%.2f yaw=%.1f omega=%.0f pred=%.2fs",
                            mt.id, mt.is_large ? "L" : "S", mt.ratio, mt.z,
                            mt.yaw * 180.0 / CV_PI, omega_deg_s_, t_pred);
                    }
                } else {
                    ++lost_frames_;
                    ROS_INFO_THROTTLE(1.0, "[Hold] 丢帧 %d/5", lost_frames_);
                    if (lost_frames_ >= 5) {
                        has_last_ = false; has_sent_ = false;
                        pos_queue_.clear(); yaw_queue_.clear();
                        size_hist_.clear(); yaw_hist_.clear();
                        lost_frames_ = 0;
                    }
                }
            } else {
                ++lost_frames_;
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
    double max_z_, deadzone_px_, uniqueness_dt_, max_speed_, omega_blind_;
    int morph_size_;
    int hsv_sv_min_;

    // 状态
    std::deque<cv::Vec3d> pos_queue_;
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

    bool has_sent_ = false;
    cv::Point2f last_sent_px_ = cv::Point2f(0, 0);
    bool blind_ = false;

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
                         int morph_size, int sv_min) {
    if (bgr.empty()) { res = cv::Mat(); return; }

    cv::Mat hsv;
    cv::Mat mask_r1, mask_r2, mask_r, mask_b, mask_g, mask_all;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, lo_r, hi_r, mask_r1);
    cv::inRange(hsv, cv::Scalar(170, sv_min, sv_min), cv::Scalar(179, 255, 255), mask_r2);
    cv::bitwise_or(mask_r1, mask_r2, mask_r);

    cv::inRange(hsv, lo_b, hi_b, mask_b);
    cv::inRange(hsv, lo_g, hi_g, mask_g);

    cv::bitwise_or(mask_r, mask_b, mask_all);
    cv::bitwise_or(mask_all, mask_g, mask_all);

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
