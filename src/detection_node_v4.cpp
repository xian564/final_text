// ============================================================
//  detection_node_v4.cpp
//  架构：
//    Preprocess  —— CLAHE(LAB) + Gamma + 饱和度拉伸 + Canny/高光融合掩膜
//    NormalArmorDetector —— 平A：动态 ROI + 轮廓/质心 + 透视校验 + 极值框 + PnP + Alpha-Beta
//    EnergyRuneDetector  —— 大招：高亮轮廓 + 最小二乘圆拟合 + PLL 相位预测
//    VisionNodeV4 —— ROS 收图/发图/发数据 + 模式切换
//
//  【视觉-火控交接边界】只输出位姿/速度，不算弹道下坠与飞行时间
// ============================================================
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

// ============================================================
//  公共预处理
// ============================================================
struct Preprocessor {
    double clip_ = 2.0;
    double sat_gain_ = 1.2;
    int    canny_lo_ = 50, canny_hi_ = 150;
    int    sv_min_ = 120, yellow_s_ = 100, yellow_v_ = 100;
    int    blue_h_min_ = 100, blue_h_max_ = 140;
    int    blue_s_min_ = 50, blue_v_min_ = 50;

    cv::Mat enhance(const cv::Mat& bgr) const {
        cv::Mat lab, l, out;
        cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> ch;
        cv::split(lab, ch);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clip_, cv::Size(8, 8));
        clahe->apply(ch[0], l);
        ch[0] = l;
        cv::merge(ch, lab);
        cv::cvtColor(lab, out, cv::COLOR_Lab2BGR);

        // 自适应 Gamma
        cv::Mat gray;
        cv::cvtColor(out, gray, cv::COLOR_BGR2GRAY);
        double mean = cv::mean(gray)[0];
        double m = std::max(1.0, std::min(254.0, mean));
        double gamma = std::log(0.5) / std::log(m / 255.0);
        gamma = std::max(0.6, std::min(2.2, gamma));
        cv::Mat lut(1, 256, CV_8U);
        for (int i = 0; i < 256; ++i)
            lut.at<uchar>(i) = cv::saturate_cast<uchar>(std::pow(i / 255.0, gamma) * 255.0);
        cv::LUT(out, lut, out);

        // 饱和度拉伸
        cv::Mat hsv;
        cv::cvtColor(out, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, ch);
        ch[1].convertTo(ch[1], -1, sat_gain_, 0);
        cv::merge(ch, hsv);
        cv::cvtColor(hsv, out, cv::COLOR_HSV2BGR);
        return out;
    }

    // 颜色 ∪ 高光 ∪ 边缘 融合
    void buildMask(const cv::Mat& bgr, cv::Mat& mask, cv::Mat& vchan) const {
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> ch;
        cv::split(hsv, ch);
        vchan = ch[2];

        cv::Mat r1, r2, r, b, g, y, color;
        cv::inRange(hsv, cv::Scalar(0, sv_min_, sv_min_), cv::Scalar(10, 255, 255), r1);
        cv::inRange(hsv, cv::Scalar(170, sv_min_, sv_min_), cv::Scalar(179, 255, 255), r2);
        r = r1 | r2;
        cv::inRange(hsv, cv::Scalar(blue_h_min_, blue_s_min_, blue_v_min_),
                    cv::Scalar(blue_h_max_, 255, 255), b);
        cv::inRange(hsv, cv::Scalar(50, sv_min_, sv_min_), cv::Scalar(70, 255, 255), g);
        cv::inRange(hsv, cv::Scalar(10, yellow_s_, yellow_v_), cv::Scalar(35, 255, 255), y);
        color = r | b | g | y;
        cv::morphologyEx(color, color, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        double maxv = 0; cv::minMaxLoc(vchan, nullptr, &maxv);
        double dyn = std::max(180.0, maxv - 50.0);
        cv::Mat high;
        cv::compare(vchan, dyn, high, cv::CMP_GT);

        cv::Mat gray, edges, ed;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(gray, edges, canny_lo_, canny_hi_);
        cv::dilate(edges, ed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        cv::Mat color_dil;
        cv::dilate(color, color_dil, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)),
                   cv::Point(-1, -1), 2);

        cv::Mat a = color & (ed | high);
        cv::Mat bb = high & color_dil;
        mask = a | bb;
    }
};

// ============================================================
//  平A检测器
// ============================================================
class NormalArmorDetector {
public:
    struct Bar {
        cv::RotatedRect rr;
        cv::Point2f pts[4];
        cv::Point2f c;
        double tilt, long_h, v;
        std::vector<cv::Point> cnt;
    };
    struct Target {
        int id;
        bool is_large;
        double ratio;
        std::vector<cv::Point2f> corners;
        cv::Point2f c2;
        bool has_pose = false;
        cv::Vec3d t;
        double yaw = 0, z = -1;
    };

    void load(ros::NodeHandle& pnh) {
        pnh.param("aspect_min", aspect_min_, 1.3);
        pnh.param("aspect_max", aspect_max_, 6.0);
        pnh.param("min_dist_x", min_dist_x_, 15.0);
        pnh.param("max_dist_x", max_dist_x_, 500.0);
        pnh.param("max_angle", max_angle_, 10.0);
        pnh.param("ratio_s_max", ratio_s_max_, 1.3);
        pnh.param("ratio_l_max", ratio_l_max_, 2.5);
        pnh.param("bar_v_min", bar_v_min_, 120);
        pnh.param("roi_size", roi_size_, 300);
        pnh.param("alpha", alpha_, 0.6);
        pnh.param("beta", beta_, 0.2);
    }

    void detect(const cv::Mat& bgr, const cv::Mat& mask, const cv::Mat& vchan,
                const cv::Mat& K, const cv::Mat& D, cv::Mat& canvas,
                std::vector<Target>& targets, const ros::Time& stamp) {
        targets.clear();
        published_ = false;
        cv::Rect roi(0, 0, bgr.cols, bgr.rows);
        if (has_last_) {
            int half = roi_size_ / 2;
            roi = cv::Rect(std::max(0, (int)last_c_.x - half),
                           std::max(0, (int)last_c_.y - half),
                           std::min(roi_size_, bgr.cols), std::min(roi_size_, bgr.rows));
            roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);
            if (roi.width < 80 || roi.height < 60) roi = cv::Rect(0, 0, bgr.cols, bgr.rows);
        }

        cv::Mat sub = mask(roi);
        std::vector<std::vector<cv::Point> > cnts;
        cv::findContours(sub, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<Bar> bars;
        for (size_t i = 0; i < cnts.size(); ++i) {
            if (cv::contourArea(cnts[i]) < 20) continue;
            cv::RotatedRect rr = cv::minAreaRect(cnts[i]);
            double w = rr.size.width, h = rr.size.height;
            if (std::min(w, h) <= 0) continue;
            double asp = std::max(w, h) / std::min(w, h);
            if (asp < aspect_min_ || asp > aspect_max_) continue;

            cv::Point2f pts[4]; rr.points(pts);
            for (int k = 0; k < 4; ++k) { pts[k].x += roi.x; pts[k].y += roi.y; }
            cv::Moments m = cv::moments(cnts[i]);
            if (std::abs(m.m00) < 1e-6) continue;

            Bar bar;
            bar.rr = cv::RotatedRect(cv::Point2f(rr.center.x + roi.x, rr.center.y + roi.y),
                                     rr.size, rr.angle);
            for (int k = 0; k < 4; ++k) bar.pts[k] = pts[k];
            bar.c = cv::Point2f((float)(m.m10 / m.m00 + roi.x),
                                (float)(m.m01 / m.m00 + roi.y));
            cv::Point2f e1 = pts[1] - pts[0], e2 = pts[2] - pts[1];
            cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
            double tilt = std::atan2(lv.x, lv.y);
            if (tilt > CV_PI / 2) tilt -= CV_PI;
            if (tilt < -CV_PI / 2) tilt += CV_PI;
            bar.tilt = tilt;
            bar.long_h = std::max(cv::norm(e1), cv::norm(e2));
            bar.v = 0;
            bar.cnt = cnts[i];
            for (size_t k = 0; k < cnts[i].size(); ++k)
                bar.cnt[k] += cv::Point(roi.x, roi.y);
            bars.push_back(bar);
        }

        // 亮度过滤
        std::vector<Bar> keep;
        for (size_t i = 0; i < bars.size(); ++i) {
            cv::Mat m = cv::Mat::zeros(bgr.size(), CV_8UC1);
            cv::drawContours(m, std::vector<std::vector<cv::Point> >(1, bars[i].cnt),
                             -1, cv::Scalar(255), cv::FILLED);
            bars[i].v = cv::mean(vchan, m)[0];
            if (bars[i].v >= bar_v_min_) keep.push_back(bars[i]);
        }
        bars.swap(keep);
        std::sort(bars.begin(), bars.end(), [](const Bar& a, const Bar& b) { return a.c.x < b.c.x; });

        int id = 1;
        for (size_t i = 0; i < bars.size(); ++i) {
            for (size_t j = i + 1; j < bars.size(); ++j) {
                double dx = bars[j].c.x - bars[i].c.x;
                if (dx > max_dist_x_) break;
                if (dx <= min_dist_x_) continue;
                if (std::abs(bars[i].c.y - bars[j].c.y) > 50) continue;
                if (std::abs(bars[i].tilt - bars[j].tilt) * 180 / CV_PI > max_angle_) continue;
                double ang = 0.5 * (bars[i].tilt + bars[j].tilt);
                double corrected = dx / std::max(0.2, std::abs(std::cos(ang)));
                double lh = std::max(bars[i].long_h, bars[j].long_h);
                double ratio = corrected / std::max(1.0, lh);
                if (ratio < 0.4 || ratio > ratio_l_max_) continue;

                int min_x = 1 << 30, max_x = -1, min_y = 1 << 30, max_y = -1;
                for (int s2 = 0; s2 < 2; ++s2) {
                    const std::vector<cv::Point>& c = (s2 == 0) ? bars[i].cnt : bars[j].cnt;
                    for (size_t k = 0; k < c.size(); ++k) {
                        min_x = std::min(min_x, c[k].x); max_x = std::max(max_x, c[k].x);
                        min_y = std::min(min_y, c[k].y); max_y = std::max(max_y, c[k].y);
                    }
                }
                // 几何硬校验：太小的框直接丢，避免 PnP 退化
                if (max_x - min_x < 20 || max_y - min_y < 10) continue;

                Target tg;
                tg.id = id++;
                tg.is_large = ratio > ratio_s_max_;
                tg.ratio = ratio;
                tg.corners.push_back(cv::Point2f(min_x, min_y));
                tg.corners.push_back(cv::Point2f(max_x, min_y));
                tg.corners.push_back(cv::Point2f(max_x, max_y));
                tg.corners.push_back(cv::Point2f(min_x, max_y));
                tg.c2 = cv::Point2f((min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f);

                double half_w = tg.is_large ? 0.1125 : 0.0675, half_h = 0.0275;
                std::vector<cv::Point3f> obj;
                obj.push_back(cv::Point3f(-half_w, half_h, 0));
                obj.push_back(cv::Point3f(half_w, half_h, 0));
                obj.push_back(cv::Point3f(half_w, -half_h, 0));
                obj.push_back(cv::Point3f(-half_w, -half_h, 0));
                cv::Mat rvec, tvec;
                bool ok = cv::solvePnP(obj, tg.corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
                if (ok && tvec.at<double>(2) > 0.3 && tvec.at<double>(2) < 20.0) {
                    tg.has_pose = true;
                    tg.t = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                    tg.z = tg.t[2];
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    tg.yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
                }
                targets.push_back(tg);
                break;
            }
        }

        // 主目标 + Alpha-Beta 平滑
        if (!targets.empty()) {
            int best = 0;
            for (size_t k = 1; k < targets.size(); ++k)
                if (std::abs(targets[k].yaw) < std::abs(targets[best].yaw)) best = (int)k;
            Target& mt = targets[best];
            if (mt.has_pose) {
                double dt = has_ab_ ? std::max(1e-3, (stamp - last_stamp_).toSec()) : 0.033;
                if (!has_ab_) {
                    ab_x_ = mt.t; ab_yaw_ = mt.yaw; ab_vx_ = cv::Vec3d(0, 0, 0); ab_w_ = 0; has_ab_ = true;
                } else {
                    cv::Vec3d pred = ab_x_ + ab_vx_ * dt;
                    double pred_yaw = ab_yaw_ + ab_w_ * dt;
                    cv::Vec3d r = mt.t - pred;
                    ab_x_ = pred + alpha_ * r;
                    ab_vx_ += beta_ * r / dt;
                    double ry = std::atan2(std::sin(mt.yaw - pred_yaw), std::cos(mt.yaw - pred_yaw));
                    ab_yaw_ = pred_yaw + alpha_ * ry;
                    ab_w_ += beta_ * ry / dt;
                }
                last_stamp_ = stamp;
                last_c_ = mt.c2;
                has_last_ = true;

                cv::rectangle(canvas,
                              cv::Rect((int)mt.corners[0].x, (int)mt.corners[0].y,
                                       (int)(mt.corners[1].x - mt.corners[0].x),
                                       (int)(mt.corners[3].y - mt.corners[0].y)),
                              cv::Scalar(0, 0, 255), 3);
                cv::putText(canvas, "ID:" + std::to_string(mt.id) + (mt.is_large ? " L" : " S"),
                            cv::Point((int)mt.corners[0].x, std::max(0, (int)mt.corners[0].y - 8)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                pub_t_ = ab_x_;
                pub_yaw_ = ab_yaw_;
                pub_v_ = ab_vx_;
                published_ = true;
            }
        } else {
            has_last_ = false;
        }
    }

    cv::Vec3d pub_t_ = cv::Vec3d(0, 0, 0), pub_v_ = cv::Vec3d(0, 0, 0);
    double pub_yaw_ = 0;
    bool published_ = false;

private:
    static float tg_left(const Target& t) { return t.corners[0].x; }

    double aspect_min_, aspect_max_, min_dist_x_, max_dist_x_, max_angle_;
    double ratio_s_max_, ratio_l_max_;
    int bar_v_min_, roi_size_;
    double alpha_, beta_;
    bool has_last_ = false, has_ab_ = false;
    cv::Point2f last_c_ = cv::Point2f(0, 0);
    cv::Vec3d ab_x_ = cv::Vec3d(0, 0, 0), ab_vx_ = cv::Vec3d(0, 0, 0);
    double ab_yaw_ = 0, ab_w_ = 0;
    ros::Time last_stamp_;
};

// ============================================================
//  大招检测器（能量机关）：高亮轮廓 + 圆拟合 + PLL
// ============================================================
class EnergyRuneDetector {
public:
    void load(ros::NodeHandle& pnh) {
        pnh.param("rune_v_min", rune_v_min_, 235);
        pnh.param("rune_area_min", rune_area_min_, 80);
        pnh.param("rune_radius_m", rune_radius_m_, 0.05);
    }

    bool detect(const cv::Mat& bgr, const cv::Mat& vchan, const cv::Mat& K,
                const ros::Time& stamp, cv::Vec3d& out_t, double& out_yaw) {
        cv::Mat m;
        cv::compare(vchan, rune_v_min_, m, cv::CMP_GT);
        std::vector<std::vector<cv::Point> > cnts;
        cv::findContours(m, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Point2f best(0, 0); double best_a = 0;
        for (size_t i = 0; i < cnts.size(); ++i) {
            double a = cv::contourArea(cnts[i]);
            if (a > best_a) {
                float rad = 0;
                cv::minEnclosingCircle(cnts[i], best, rad);
                best_a = a;
            }
        }
        if (best_a < rune_area_min_) {
            if (has_) { out_t = pred_; out_yaw = theta_; return true; }
            return false;
        }
        hist_.push_back(best);
        if (hist_.size() > 5) hist_.pop_front();
        if (hist_.size() >= 3) {
            double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sx3 = 0, sy3 = 0, sxy2 = 0, sx2y = 0;
            int n = (int)hist_.size();
            for (size_t i = 0; i < hist_.size(); ++i) {
                double x = hist_[i].x, y = hist_[i].y;
                sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
                sx3 += x * x * x; sy3 += y * y * y; sxy2 += x * y * y; sx2y += x * x * y;
            }
            double den = n * (sxx + syy) - (sx * sx + sy * sy);
            if (std::abs(den) > 1e-6) {
                cx_ = (0.5 * (n * (sx3 + sxy2) + n * (sx2y + sy3)
                      - (sx * (sxx + syy) + sy * (sxx + syy))) ) / den;
                cy_ = (0.5 * (n * (sx2y + sy3) + n * (sx3 + sxy2)
                      - (sy * (sxx + syy) + sx * (sxx + syy))) ) / den;
            }
        }
        double theta = std::atan2(best.y - cy_, best.x - cx_);
        double dt = has_ ? std::max(1e-3, (stamp - last_).toSec()) : 0.033;
        if (has_) {
            double err = std::atan2(std::sin(theta - theta_), std::cos(theta - theta_));
            omega_ += kp_ * err * dt;
            theta_ += omega_ * dt + kp_ * err * dt;
        } else {
            theta_ = theta; omega_ = 0; has_ = true;
        }
        last_ = stamp;
        double pred_theta = theta_ + omega_ * 0.1;
        double fx = K.at<double>(0, 0);
        double r = std::max(2.0, std::sqrt(best_a / CV_PI));
        double z = fx * rune_radius_m_ * 2.0 / r;
        pred_ = cv::Vec3d((best.x - K.at<double>(0, 2)) * z / fx,
                          (best.y - K.at<double>(1, 2)) * z / fx, z);
        out_t = pred_; out_yaw = pred_theta;
        return true;
    }

private:
    int rune_v_min_, rune_area_min_;
    double rune_radius_m_;
    double kp_ = 0.5;
    bool has_ = false;
    ros::Time last_;
    std::deque<cv::Point2f> hist_;
    double cx_ = 0, cy_ = 0, theta_ = 0, omega_ = 0;
    cv::Vec3d pred_ = cv::Vec3d(0, 0, 0);
};

// ============================================================
//  ROS 节点
// ============================================================
class VisionNodeV4 {
public:
    VisionNodeV4(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh), ready_(false) {
        pre_.clip_ = 2.0;
        pnh_.param("clip", pre_.clip_, 2.0);
        pnh_.param("sat_gain", pre_.sat_gain_, 1.2);
        normal_.load(pnh_);
        rune_.load(pnh_);
        pnh_.param("mode", mode_, 0);

        info_sub_ = nh_.subscribe("/hk_camera/camera_info", 1,
                                  &VisionNodeV4::infoCb, this);
        image_sub_ = it_.subscribe("/hk_camera/image_raw", 1,
                                   &VisionNodeV4::imageCb, this,
                                   image_transport::TransportHints("compressed", ros::TransportHints(), pnh_));
        mode_sub_ = nh_.subscribe("/vision_mode", 1, &VisionNodeV4::modeCb, this);
        image_pub_ = it_.advertise("/ft/debug/image_v4", 1);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ft/armor_pose", 1);
        vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/ft/armor_velocity", 1);
    }

    bool ready() const { return ready_; }

private:
    void infoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
        if (!msg->width || !msg->height) return;
        K_ = cv::Mat(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
            K_.at<double>(r, c) = msg->K[r * 3 + c];
        D_ = cv::Mat(1, (int)msg->D.size(), CV_64F);
        for (size_t i = 0; i < msg->D.size(); ++i) D_.at<double>(0, (int)i) = msg->D[i];
        frame_ = msg->header.frame_id;
        ready_ = true;
    }
    void modeCb(const std_msgs::Int8ConstPtr& msg) { mode_ = msg->data; }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        if (!ready_) return;
        cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, "bgr8");
        const cv::Mat& img = cvp->image;
        cv::Mat enh = pre_.enhance(img);
        cv::Mat mask, vchan;
        pre_.buildMask(enh, mask, vchan);
        cv::Mat canvas = enh.clone();

        bool ok = false;
        cv::Vec3d t(0, 0, 0); double yaw = 0;
        if (mode_ == 1) {
            ok = rune_.detect(enh, vchan, K_, msg->header.stamp, t, yaw);
            cv::putText(canvas, "MODE: RUNE", cv::Point(60, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 255), 2);
        } else {
            std::vector<NormalArmorDetector::Target> ts;
            normal_.detect(enh, mask, vchan, K_, D_, canvas, ts, msg->header.stamp);
            ok = normal_.published_;
            if (ok) { t = normal_.pub_t_; yaw = normal_.pub_yaw_; }
            cv::putText(canvas, "MODE: ARMOR", cv::Point(60, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
        }

        if (ok && t[2] > 0.3 && t[2] < 20.0) {
            geometry_msgs::PoseStamped ps;
            ps.header.stamp = msg->header.stamp;
            ps.header.frame_id = frame_;
            ps.pose.position.x = t[0]; ps.pose.position.y = -t[1]; ps.pose.position.z = t[2];
            tf2::Quaternion q; q.setRPY(0, 0, yaw);
            ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y();
            ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
            pose_pub_.publish(ps);

            geometry_msgs::TwistStamped ts;
            ts.header = ps.header;
            ts.twist.linear.x = normal_.pub_v_[0];
            ts.twist.linear.y = -normal_.pub_v_[1];
            ts.twist.linear.z = normal_.pub_v_[2];
            vel_pub_.publish(ts);
        }

        cv_bridge::CvImage out;
        out.header = msg->header;
        out.encoding = "bgr8";
        out.image = canvas;
        image_pub_.publish(out.toImageMsg());
        std::cout << "ok" << std::endl;
    }

    ros::NodeHandle nh_, pnh_;
    image_transport::ImageTransport it_;
    ros::Subscriber info_sub_, mode_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pose_pub_, vel_pub_;
    Preprocessor pre_;
    NormalArmorDetector normal_;
    EnergyRuneDetector rune_;
    cv::Mat K_, D_;
    std::string frame_;
    int mode_ = 0;
    bool ready_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_vision_node_v4");
    ros::NodeHandle nh, pnh("~");
    VisionNodeV4 node(nh, pnh);
    ros::Rate r(20);
    ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(30.0);
    while (ros::ok() && !node.ready()) {
        ros::spinOnce();
        if (ros::WallTime::now() > deadline) { ROS_FATAL("启动超时"); return 1; }
        r.sleep();
    }
    ROS_INFO("VisionNodeV4 就绪");
    ros::spin();
    return 0;
}
