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
#include <dynamic_reconfigure/server.h>
#include <ros/package.h>
#include <final_text/VisionConfig.h>
#include "final_text/detector_interface.hpp"
#include <memory>
#include <ft_msgs/Detection.h>
#include <ft_msgs/Detections.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
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
        pnh_.param("ratio_s_max", ratio_s_max_, 3.0);
        pnh_.param("ratio_l_max", ratio_l_max_, 5.0);
        pnh_.param("ratio_min",   ratio_min_, 0.4);
        // 灯条尺寸（米），定义一：大 230×60mm，小 135×60mm
        pnh_.param("armor_large_w", armor_large_w_, 0.230);
        pnh_.param("armor_large_h", armor_large_h_, 0.060);
        pnh_.param("armor_small_w", armor_small_w_, 0.135);
        pnh_.param("armor_small_h", armor_small_h_, 0.060);
        pnh_.param("max_z",       max_z_, 8.0);
        pnh_.param("deadzone_px", deadzone_px_, 5.0);
        pnh_.param("uniqueness_dt", uniqueness_dt_, 0.15);
        pnh_.param("max_speed",   max_speed_, 8.0);     // 车体物理极限 m/s
        pnh_.param("morph_size",  morph_size_, 3);
        pnh_.param("hsv_sv_min",   hsv_sv_min_, 100);
        pnh_.param("red_h_min",    red_h_min_, 0);
        pnh_.param("red_h_max",    red_h_max_, 10);
        pnh_.param("roi_y_ratio",  roi_y_ratio_, 0.0);   // 先关闭 ROI
        pnh_.param("bar_v_min",    bar_v_min_, 160);
        pnh_.param("bar_keep_n",   bar_keep_n_, 6);
        pnh_.param("hold_frames", hold_frames_, 8);
        pnh_.param("hold_max_speed", hold_max_speed_, 8.0);
        pnh_.param("guard_y_ratio", guard_y_ratio_, 0.85);
        pnh_.param("dyn_v_offset",  dyn_v_offset_, 50.0);
        pnh_.param("dyn_v_min", dyn_v_min_, 180.0);
        pnh_.param("z_min", z_min_, 0.3);
        pnh_.param("z_max", z_max_, 15.0);
        pnh_.param("teleport_scale", teleport_scale_, 1.5);
        pnh_.param("min_speed_z", min_speed_z_, 0.5);
        pnh_.param("pair_y_ratio",  pair_y_ratio_, 0.75);
        pnh_.param("fcs_four_span", fcs_four_span_, 30.0);
        pnh_.param("fcs_four_tol",  fcs_four_tol_, 60.0);
        pnh_.param("blue_h_min",   blue_h_min_, 85);
        pnh_.param("blue_h_max",   blue_h_max_, 140);
        pnh_.param("blue_s_min",   blue_s_min_, 50);
        pnh_.param("blue_v_min",   blue_v_min_, 50);
        pnh_.param("white_enable", white_enable_, false);   // 极端场景才开，默认关闭
        pnh_.param("yellow_h_min", yellow_h_min_, 10);
        pnh_.param("yellow_h_max", yellow_h_max_, 35);
        pnh_.param("yellow_s_min", yellow_s_min_, 100);
        pnh_.param("yellow_v_min", yellow_v_min_, 100);

        // ---- 话题 / 队列 / 杂项阈值（全部外置）----
        pnh_.param<std::string>("image_topic", image_topic_, "/hk_camera/image_raw");
        pnh_.param<std::string>("camera_info_topic", info_topic_, "/hk_camera/camera_info");
        pnh_.param<std::string>("debug_topic", debug_topic_, "/ft/debug/image");
        pnh_.param<std::string>("detection_topic", detection_topic_, "/ft/detection");
        pnh_.param("sub_queue", sub_queue_, 1);
        pnh_.param("pub_queue", pub_queue_, 1);
        pnh_.param<std::string>("tf_child_frame", tf_child_, "armor_v3");
        pnh_.param("min_area", min_area_, 20);
        pnh_.param("time_window_s", time_window_s_, 0.5);
        pnh_.param("digit_thres", digit_thres_, 0.5);
        pnh_.param("lock_frames", lock_frames_, 20);
        pnh_.param("lock_gate_px", lock_gate_px_, 50.0);
        pnh_.param("lock_pred_gate_px", lock_pred_gate_px_, 250.0);
        pnh_.param("lock_ratio_tol", lock_ratio_tol_, 0.1);   // 时间窗（瞬移/外推）
        pnh_.param("nn_enable", nn_enable_, false);
        pnh_.param<std::string>("nn_model_path", nn_model_path_, "");
        pnh_.param("fcs_min_area", fcs_min_area_, 150.0);
        pnh_.param("dedup_px", dedup_px_, 120.0);
        pnh_.param("teleport_margin", teleport_margin_, 20.0);

        info_sub_ = nh_.subscribe(
            info_topic_, sub_queue_,
            &VisionNodeV3::cameraInfoCallback, this);

        try {
            image_sub_ = it_.subscribe(
                image_topic_, sub_queue_,
                &VisionNodeV3::imageCallback, this,
                image_transport::TransportHints(
                    "compressed", ros::TransportHints(), pnh_));
        } catch (const image_transport::TransportLoadException& e) {
            ROS_FATAL("compressed 传输插件加载失败: %s", e.what());
            transport_ready_ = false;
        }

        // 与原版一致：发布 /ft/debug/image
        image_pub_ = it_.advertise(debug_topic_, pub_queue_);

        dr_server_.reset(new dynamic_reconfigure::Server<final_text::VisionConfig>(pnh_));
        dr_server_->setCallback(
            boost::bind(&VisionNodeV3::drCallback, this, _1, _2));
        loadDigitTemplates();
        detection_array_pub_ = nh_.advertise<ft_msgs::Detections>(detection_topic_, pub_queue_);
        if (nn_enable_ && !nn_model_path_.empty()) {
            nn_detector_.reset(new OnnxDetector(nn_model_path_, 0.5f, 0.5f, 0.45f));
            ROS_INFO("NN detector: %s ready=%d", nn_model_path_.c_str(),
                     nn_detector_ ? (int)nn_detector_->ready() : 0);
        }
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
        cv::Mat R;                 // rvec 转出的旋转矩阵（相机系）
        bool has_r = false;
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
            // [cv] line：画线到 img
            cv::line(img, cv::Point(x, r.y),
                     cv::Point(std::min(x + step, r.x + r.width), r.y), c, thick);
        for (int x = r.x; x < r.x + r.width; x += step * 2)
            // [cv] line：画线到 img
            cv::line(img, cv::Point(x, r.y + r.height),
                     cv::Point(std::min(x + step, r.x + r.width), r.y + r.height), c, thick);
        for (int y = r.y; y < r.y + r.height; y += step * 2)
            // [cv] line：画线到 img
            cv::line(img, cv::Point(r.x, y),
                     cv::Point(r.x, std::min(y + step, r.y + r.height)), c, thick);
        for (int y = r.y; y < r.y + r.height; y += step * 2)
            // [cv] line：画线到 img
            cv::line(img, cv::Point(r.x + r.width, y),
                     cv::Point(r.x + r.width, std::min(y + step, r.y + r.height)), c, thick);
    }

    // ---------- 亮度梯度（内部均值 - 边缘环均值） ----------
    static double brightGradient(const cv::Mat& v, const cv::Rect& rect) {
        cv::Rect r = rect & cv::Rect(0, 0, v.cols, v.rows);
        if (r.width < 3 || r.height < 3) return 0.0;
        cv::Mat m = cv::Mat::zeros(v.size(), CV_8UC1);
        // [cv] rectangle：画矩形到 m
        cv::rectangle(m, r, 255, cv::FILLED);
        cv::Mat md;
        // [cv] dilate：m → md，膨胀补断裂
        cv::dilate(m, md, cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size(11, 11)));
        cv::Mat ring;
        cv::subtract(md, m, ring);
        // [cv] mean：v 掩膜内均值
        double inner = cv::mean(v, m)[0];
        // [cv] mean：v 掩膜内均值
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
            // [cv] minMaxLoc：最亮值/位置 → max_v/max_loc
            cv::minMaxLoc(v(q[i]), nullptr, &mx);
            out[i] = mx;
        }
    }


    // dynamic_reconfigure 回调：先写回参数服务器，再立即生效
    void drCallback(final_text::VisionConfig& cfg, uint32_t level) {
        (void)level;
        pnh_.setParam("dyn_v_min", cfg.dyn_v_min);
        pnh_.setParam("ratio_s_max", cfg.ratio_s_max);
        pnh_.setParam("z_min", cfg.z_min);
        pnh_.setParam("z_max", cfg.z_max);
        pnh_.setParam("red_h_min", cfg.red_h_min);
        pnh_.setParam("red_h_max", cfg.red_h_max);
        pnh_.setParam("hsv_sv_min", cfg.hsv_sv_min);
        pnh_.setParam("blue_h_min", cfg.blue_h_min);
        pnh_.setParam("blue_h_max", cfg.blue_h_max);
        pnh_.setParam("blue_s_min", cfg.blue_s_min);
        pnh_.setParam("blue_v_min", cfg.blue_v_min);
        pnh_.setParam("bar_v_min", cfg.bar_v_min);
        pnh_.setParam("min_area", cfg.min_area);
        pnh_.setParam("max_dist_x", cfg.max_dist_x);
        pnh_.setParam("max_angle_diff", cfg.max_angle_diff);
        pnh_.setParam("aspect_min", cfg.aspect_min);
        pnh_.setParam("aspect_max", cfg.aspect_max);
        pnh_.setParam("ratio_min", cfg.ratio_min);
        pnh_.setParam("ratio_l_max", cfg.ratio_l_max);
        pnh_.setParam("pair_y_ratio", cfg.pair_y_ratio);
        pnh_.setParam("white_enable", cfg.white_enable);

        dyn_v_min_ = cfg.dyn_v_min;
        z_min_ = cfg.z_min; z_max_ = cfg.z_max;
        ratio_s_max_ = cfg.ratio_s_max;
        red_h_min_ = cfg.red_h_min; red_h_max_ = cfg.red_h_max;
        hsv_sv_min_ = cfg.hsv_sv_min;
        blue_h_min_ = cfg.blue_h_min; blue_h_max_ = cfg.blue_h_max;
        blue_s_min_ = cfg.blue_s_min; blue_v_min_ = cfg.blue_v_min;
        bar_v_min_ = cfg.bar_v_min; min_area_ = cfg.min_area;
        max_dist_x_ = cfg.max_dist_x; max_angle_diff_ = cfg.max_angle_diff;
        aspect_min_ = cfg.aspect_min; aspect_max_ = cfg.aspect_max;
        ratio_min_ = cfg.ratio_min; ratio_l_max_ = cfg.ratio_l_max;
        pair_y_ratio_ = cfg.pair_y_ratio; white_enable_ = cfg.white_enable;
    }

    // ---------- 图像回调 ----------
    // ---------- 数字模板识别（传统，可选） ----------
    std::vector<cv::Mat> digit_templates_;
    std::vector<int> digit_labels_;

    void loadDigitTemplates() {
        std::string dir = ros::package::getPath("final_text") + "/models/";
        for (int i = 1; i <= 5; ++i) {
            cv::Mat img = cv::imread(dir + "digit_" + std::to_string(i) + ".png",
                                     cv::IMREAD_GRAYSCALE);
            if (img.empty()) { ROS_WARN("无法读取模板: digit_%d.png", i); continue; }
            cv::Mat bin;
            cv::threshold(img, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            if (cv::mean(bin)[0] > 127) cv::bitwise_not(bin, bin);
            cv::resize(bin, bin, cv::Size(20, 30));
            digit_templates_.push_back(bin);
            digit_labels_.push_back(i);
        }
        ROS_INFO("数字模板加载 %zu 个", digit_templates_.size());
    }

    int recognizeNumber(const cv::Mat& gray_roi) {
        if (digit_templates_.empty() || gray_roi.empty()) return 0;
        cv::Mat bin;
        cv::threshold(gray_roi, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        if (cv::mean(bin)[0] > 127) cv::bitwise_not(bin, bin);
        cv::resize(bin, bin, cv::Size(20, 30));
        double best = -1; int label = 0;
        for (size_t i = 0; i < digit_templates_.size(); ++i) {
            cv::Mat res;
            cv::matchTemplate(bin, digit_templates_[i], res, cv::TM_CCOEFF_NORMED);
            double score = res.at<float>(0, 0);
            if (score > best) { best = score; label = digit_labels_[i]; }
        }
        return (best > digit_thres_) ? label : 0;
    }

    // ---------- 神经网络接口（可选）：IDetector::detect -> PnP -> /ft/detection ----------
    bool processNN(const cv::Mat& bgr, const sensor_msgs::ImageConstPtr& msg, cv::Mat& canvas) {
        if (!nn_detector_ || !nn_detector_->ready()) return false;
        std::vector<ArmorObservation> armors;
        if (!nn_detector_->detect(bgr, armors) || armors.empty()) return false;

        ft_msgs::Detections arr;
        arr.header = msg->header;
        for (size_t k = 0; k < armors.size(); ++k) {
            const ArmorObservation& a = armors[k];
            double w = 0.5 * (cv::norm(a.corners[0] - a.corners[1]) +
                              cv::norm(a.corners[3] - a.corners[2]));
            double h = 0.5 * (cv::norm(a.corners[0] - a.corners[3]) +
                              cv::norm(a.corners[1] - a.corners[2]));
            if (w < 3 || h < 3) continue;
            bool is_large = (w / std::max(1.0, h) > 3.0);
            double Wb = (is_large ? armor_large_w_ : armor_small_w_) * 0.5;
            double Hb = (is_large ? armor_large_h_ : armor_small_h_) * 0.5;
            std::vector<cv::Point3f> obj = {
                {(float)-Wb, (float)-Hb, 0}, {(float)Wb, (float)-Hb, 0},
                {(float)Wb, (float)Hb, 0}, {(float)-Wb, (float)Hb, 0}};
            std::vector<cv::Point2f> img(a.corners, a.corners + 4);
            cv::Mat rvec, tvec;
            bool ok = cv::solvePnP(obj, img, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
            if (!ok) ok = cv::solvePnP(obj, img, K_, D_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            if (!ok || tvec.at<double>(2) <= z_min_ || tvec.at<double>(2) > z_max_) continue;

            ft_msgs::Detection d;
            d.header = msg->header;
            d.id = (int)k + 1;
            d.number = a.number;
            d.is_red = (a.color == 1);
            d.is_large_armor = is_large;
            if (!std::isfinite(tvec.at<double>(0)) || !std::isfinite(tvec.at<double>(1)) ||
                !std::isfinite(tvec.at<double>(2))) continue;
            d.pose.position.x = tvec.at<double>(0);
            d.pose.position.y = tvec.at<double>(1);
            d.pose.position.z = tvec.at<double>(2);
            cv::Mat R; cv::Rodrigues(rvec, R);
            tf2::Matrix3x3 m3(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                              R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                              R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
            tf2::Quaternion q; m3.getRotation(q);
            d.pose.orientation.x = q.x(); d.pose.orientation.y = q.y();
            d.pose.orientation.z = q.z(); d.pose.orientation.w = q.w();
            arr.detections.push_back(d);

            std::vector<cv::Point> quad;
            for (int c = 0; c < 4; ++c) quad.push_back(cv::Point((int)a.corners[c].x, (int)a.corners[c].y));
            cv::polylines(canvas, quad, true, cv::Scalar(0, 255, 0), 3);
            cv::putText(canvas, "NN ID:" + std::to_string(d.id),
                        cv::Point(quad[0].x, std::max(0, quad[0].y - 8)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            geometry_msgs::TransformStamped ts;
            ts.header = msg->header;
            ts.header.frame_id = camera_frame_;
            ts.child_frame_id = tf_child_;
            ts.transform.translation.x = d.pose.position.x;
            ts.transform.translation.y = d.pose.position.y;
            ts.transform.translation.z = d.pose.position.z;
            ts.transform.rotation = d.pose.orientation;
            tf_broadcaster_.sendTransform(ts);
        }
        if (!arr.detections.empty()) detection_array_pub_.publish(arr);
        return !arr.detections.empty();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!is_ready_.load()) return;

        try {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat img = cv_ptr->image;
            if (img.cols != width_ || img.rows != height_) return;

            cv::Mat canvas = img.clone();

            if (nn_enable_) {
                processNN(img, msg, canvas);
                cv_bridge::CvImage out_img;
                out_img.header = msg->header;
                out_img.encoding = "bgr8";
                out_img.image = canvas;
                image_pub_.publish(out_img.toImageMsg());
                return;
            }

            // 保留原版 debug 绿框（绝对不动）
            // [cv] rectangle：画矩形到 canvas
            cv::rectangle(canvas, cv::Rect(10, 10, 50, 50),
                          cv::Scalar(0, 255, 0), 2);

            cv::Mat colored;
            colorSegment(canvas, colored,
                         cv::Scalar(red_h_min_, hsv_sv_min_, hsv_sv_min_), cv::Scalar(red_h_max_, 255, 255),
                         cv::Scalar(blue_h_min_, blue_s_min_, blue_v_min_), cv::Scalar(blue_h_max_, 255, 255),
                         cv::Scalar(50, hsv_sv_min_, hsv_sv_min_), cv::Scalar(70, 255, 255),
                         morph_size_, hsv_sv_min_,
                         yellow_h_min_, yellow_h_max_, yellow_s_min_, yellow_v_min_);

            // 亮度筛选用：单独提取 V 通道
            cv::Mat hsv_full, v_chan;
            // [cv] cvtColor：canvas → hsv_full（HSV/灰度）；下游：inRange 取颜色掩膜
            cv::cvtColor(canvas, hsv_full, cv::COLOR_BGR2HSV);
            cv::extractChannel(hsv_full, v_chan, 2);

            // 1) 全图最高光定位
            double max_v = 0.0;
            cv::Point max_loc;
            // [cv] minMaxLoc：最亮值/位置 → max_v/max_loc
            cv::minMaxLoc(v_chan, nullptr, &max_v, nullptr, &max_loc);

            // 2) 全局空间守卫：最亮点在画面极底部 -> 大概率地面反光
            bool guard_block = (max_loc.y > height_ * guard_y_ratio_);
            if (guard_block) {
                ROS_INFO_THROTTLE(1.0, "[Guard] 最亮点在底部 y=%d, 跳过本帧", max_loc.y);
            }

            // 3) 动态高亮掩膜（解决过曝）
            double dyn_v_thresh = std::max(dyn_v_min_, max_v - dyn_v_offset_);
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
                // [cv] cvtColor：colored → seg_mask（HSV/灰度）；下游：形态学 → findContours 取轮廓
                cv::cvtColor(colored, seg_mask, cv::COLOR_BGR2GRAY);

                // 3) 过曝中心融合：颜色区域膨胀后与高亮 mask 相交，再并入分割 mask
                if (!high_mask.empty()) {
                    cv::Mat color_mask, color_dil, high_fused;
                    // [cv] threshold：seg_mask 二值化 → color_mask
                    cv::threshold(seg_mask, color_mask, 1, 255, cv::THRESH_BINARY);
                    // [cv] getStructuringElement：构造形态学核
                    cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT,
                                                           cv::Size(3, 3));
                    // [cv] dilate：color_mask → color_dil，膨胀补断裂
                    cv::dilate(color_mask, color_dil, k3, cv::Point(-1, -1), 2);
                    // [cv] bitwise_and：掩膜相交 → high_fused
                    cv::bitwise_and(high_mask, color_dil, high_fused);
                    // [cv] bitwise_or：合并掩膜 → seg_mask；下游：形态学 → findContours 取轮廓
                    cv::bitwise_or(seg_mask, high_fused, seg_mask);
                }

                // [cv] findContours：从 seg_mask 提取 → contours；下游：逐轮廓筛成灯条 bars
                cv::findContours(seg_mask, contours, cv::RETR_EXTERNAL,
                                 cv::CHAIN_APPROX_SIMPLE);

                for (size_t i = 0; i < contours.size(); ++i) {
                    // [cv] contourArea：算 cv::contourArea(contours[i]) < min_area_ 面积
                    if (cv::contourArea(contours[i]) < min_area_) continue;
                    // [cv] minAreaRect：contours[i] → rr（中心/长宽/倾角）；下游：生成 Bar（中心/长宽/角度）
                    cv::RotatedRect rr = cv::minAreaRect(contours[i]);
                    double w = rr.size.width, h = rr.size.height;
                    if (std::min(w, h) <= 0) continue;
                    double aspect = std::max(w, h) / std::min(w, h);
                    if (aspect < aspect_min_ || aspect > aspect_max_) continue;

                    // 最高亮度过滤：真灯条主动发光，反光达不到这条线
                    cv::Mat cmask(canvas.size(), CV_8UC1, cv::Scalar(0));
                    cv::drawContours(cmask, contours, static_cast<int>(i),
                                     cv::Scalar(255), cv::FILLED);
                    // [cv] mean：v_chan 掩膜内均值
                    double mean_v = cv::mean(v_chan, cmask)[0];
                    if (mean_v < bar_v_min_) continue;

                    Bar bar;
                    bar.rr = rr; rr.points(bar.pts);

                    // [cv] moments：contours[i] → m，算质心
                    cv::Moments m = cv::moments(contours[i]);
                    bar.center = rr.center;
                    if (std::abs(m.m00) > 1e-6)
                        bar.center = cv::Point2f(
                            static_cast<float>(m.m10 / m.m00),
                            static_cast<float>(m.m01 / m.m00));

                    cv::Point2f e1 = bar.pts[1] - bar.pts[0];
                    cv::Point2f e2 = bar.pts[2] - bar.pts[1];
                    // [cv] norm：距离/长度 → 
                    cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
                    double tilt = std::atan2(lv.x, lv.y);
                    if (tilt > CV_PI / 2) tilt -= CV_PI;
                    if (tilt < -CV_PI / 2) tilt += CV_PI;
                    bar.tilt = tilt;
                    // [cv] norm：距离/长度 → 
                    bar.long_h = std::max(cv::norm(e1), cv::norm(e2));
                    bar.v = mean_v;
                    bar.contour = contours[i];

                    // 补丁4：y 方向 ROI，滤掉画面下部的地面反光
                    if (roi_y_ratio_ > 0.0 &&
                        bar.center.y > height_ * roi_y_ratio_) continue;

                    bars.push_back(bar);

                    for (int k = 0; k < 4; ++k)
                        // [cv] line：画线到 canvas
                        cv::line(canvas, bar.pts[k], bar.pts[(k + 1) % 4],
                                 cv::Scalar(0, 255, 255), 2);
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

                        // ---------- 旋转四点：左灯条左边缘 + 右灯条右边缘 ----------
                        auto edge_top_bottom = [](const Bar& b, bool left_edge,
                                                  cv::Point2f& top, cv::Point2f& bot) {
                            float cx = b.rr.center.x;
                            std::vector<cv::Point2f> e;
                            for (int k = 0; k < 4; ++k) {
                                if (left_edge ? (b.pts[k].x <= cx) : (b.pts[k].x >= cx))
                                    e.push_back(b.pts[k]);
                            }
                            if (e.size() < 2) { e.clear(); e.push_back(b.pts[0]); e.push_back(b.pts[2]); }
                            top = e[0]; bot = e[0];
                            for (size_t k = 1; k < e.size(); ++k) {
                                if (e[k].y < top.y) top = e[k];
                                if (e[k].y > bot.y) bot = e[k];
                            }
                        };
                        cv::Point2f lt, lb, rt, rb;
                        edge_top_bottom(bars[i], true, lt, lb);    // 左灯条外边缘
                        edge_top_bottom(bars[j], false, rt, rb);   // 右灯条外边缘

                        // 按板体长宽比拉伸高度：灯条外框高度≈灯长，不是板高
                        {
                            double W = is_large ? armor_large_w_ : armor_small_w_;
                            double H = is_large ? armor_large_h_ : armor_small_h_;
                            // [cv] norm：距离/长度 → 
                            float wpx = (float)cv::norm(rt - lt);
                            // [cv] norm：距离/长度 → 
                            float cur_h = 0.5f * ((float)cv::norm(lb - lt) + (float)cv::norm(rb - rt));
                            float target_h = wpx * (float)(H / std::max(1e-6, W));
                            if (cur_h > 2.0f && target_h > 2.0f) {
                                float cxx = (lt.x + rt.x + rb.x + lb.x) * 0.25f;
                                float cyy = (lt.y + rt.y + rb.y + lb.y) * 0.25f;
                                float sy = target_h / cur_h;
                                lt.y = cyy + (lt.y - cyy) * sy;
                                rt.y = cyy + (rt.y - cyy) * sy;
                                rb.y = cyy + (rb.y - cyy) * sy;
                                lb.y = cyy + (lb.y - cyy) * sy;
                            }
                        }

                        Target tg;
                        tg.id = armor_id++;
                        tg.is_large = is_large;
                        tg.ratio = ratio;
                        tg.avg_tilt = avg_angle;
                        tg.corners.push_back(lt);   // TL
                        tg.corners.push_back(rt);   // TR
                        tg.corners.push_back(rb);   // BR
                        tg.corners.push_back(lb);   // BL
                        tg.center2d = cv::Point2f((lt.x + rt.x + rb.x + lb.x) * 0.25f,
                                                  (lt.y + rt.y + rb.y + lb.y) * 0.25f);

                        // 4) 相对位置验证：配对中心必须在中上部
                        if (pair_y_ratio_ > 0.0 &&
                            tg.center2d.y > height_ * pair_y_ratio_) continue;
                        tg.has_pose = false;
                        tg.z = -1.0;

                        // 去重：中心距离 <25px 视为同一块板，避免同一根灯条被拆段后重复配对
                        bool dup = false;
                        for (size_t d = 0; d < targets.size(); ++d) {
                            // [cv] norm：距离/长度 → 
                            if (cv::norm(targets[d].center2d - tg.center2d) < dedup_px_) {
                                dup = true; break;
                            }
                        }
                        if (dup) continue;

                        // ---------- solvePnP ----------
                        if (K_.rows == 3 && K_.cols == 3) {
                            double half_w = (is_large ? armor_large_w_ : armor_small_w_) * 0.5;
                            double half_h = (is_large ? armor_large_h_ : armor_small_h_) * 0.5;
                            std::vector<cv::Point3f> obj;
                            obj.push_back(cv::Point3f(-half_w, -half_h, 0));  // TL
                            obj.push_back(cv::Point3f( half_w, -half_h, 0));  // TR
                            obj.push_back(cv::Point3f( half_w,  half_h, 0));  // BR
                            obj.push_back(cv::Point3f(-half_w,  half_h, 0));  // BL

                            cv::Mat rvec, tvec;
                            // [cv] solvePnP：物体点+四点 → rvec/tvec
                            bool ok = cv::solvePnP(obj, tg.corners, K_, D_,
                                                   rvec, tvec, false, cv::SOLVEPNP_IPPE);
                            if (!ok)
                                // [cv] solvePnP：物体点+四点 → rvec/tvec
                                ok = cv::solvePnP(obj, tg.corners, K_, D_,
                                                  rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
                            if (ok && tvec.at<double>(2) > z_min_ &&
                                tvec.at<double>(2) < z_max_) {
                                tg.has_pose = true;
                                tg.t = cv::Vec3d(tvec.at<double>(0),
                                                 tvec.at<double>(1),
                                                 tvec.at<double>(2));
                                tg.z = tg.t[2];
                                cv::Mat R;
                                // [cv] Rodrigues：rvec ↔ R；下游：yaw/四元数 → TF
                                cv::Rodrigues(rvec, R);
                                tg.R = R.clone();
                                tg.has_r = true;
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

                // ---- 发布当前帧全部检测（数组，每帧唯一 id）----
                {
                    ft_msgs::Detections arr;
                    arr.header = msg->header;
                    for (size_t k = 0; k < targets.size(); ++k) {
                        const Target& tg = targets[k];
                        if (!tg.has_pose || !tg.has_r) continue;

                        ft_msgs::Detection d;
                        d.header = msg->header;
                        if (!std::isfinite(tg.t[0]) || !std::isfinite(tg.t[1]) ||
                            !std::isfinite(tg.t[2])) continue;
                        d.id = tg.id;
                        d.number = 0;
                        if (!digit_templates_.empty()) {
                            int bx1 = canvas.cols, by1 = canvas.rows, bx2 = 0, by2 = 0;
                            for (int c = 0; c < 4; ++c) {
                                bx1 = std::min(bx1, (int)tg.corners[c].x);
                                by1 = std::min(by1, (int)tg.corners[c].y);
                                bx2 = std::max(bx2, (int)tg.corners[c].x);
                                by2 = std::max(by2, (int)tg.corners[c].y);
                            }
                            cv::Rect bb(bx1, by1, std::max(1,bx2-bx1), std::max(1,by2-by1));
                            bb &= cv::Rect(0, 0, canvas.cols, canvas.rows);
                            if (bb.width > 5 && bb.height > 5) {
                                cv::Mat gray;
                                cv::cvtColor(canvas(bb), gray, cv::COLOR_BGR2GRAY);
                                d.number = recognizeNumber(gray);
                            }
                        }
                        d.is_large_armor = tg.is_large;

                        // 颜色判定：取框内红/蓝像素数
                        int bx1 = canvas.cols, by1 = canvas.rows, bx2 = 0, by2 = 0;
                        for (int c = 0; c < 4; ++c) {
                            bx1 = std::min(bx1, (int)tg.corners[c].x);
                            by1 = std::min(by1, (int)tg.corners[c].y);
                            bx2 = std::max(bx2, (int)tg.corners[c].x);
                            by2 = std::max(by2, (int)tg.corners[c].y);
                        }
                        cv::Rect bb(bx1, by1, std::max(1, bx2 - bx1), std::max(1, by2 - by1));
                        bb &= cv::Rect(0, 0, canvas.cols, canvas.rows);
                        int n_red = 0, n_blue = 0;
                        if (bb.width > 2 && bb.height > 2) {
                            cv::Mat hsv_bb = hsv_full(bb);
                            cv::Mat m1, m2, mb;
                            // [cv] inRange：hsv_bb 上取阈值 → （0/255 掩膜）
                            cv::inRange(hsv_bb, cv::Scalar(red_h_min_, hsv_sv_min_, hsv_sv_min_),
                                        cv::Scalar(red_h_max_, 255, 255), m1);
                            // [cv] inRange：hsv_bb 上取阈值 → （0/255 掩膜）
                            cv::inRange(hsv_bb, cv::Scalar(170, hsv_sv_min_, hsv_sv_min_),
                                        cv::Scalar(179, 255, 255), m2);
                            // [cv] inRange：hsv_bb 上取阈值 → （0/255 掩膜）
                            cv::inRange(hsv_bb, cv::Scalar(blue_h_min_, blue_s_min_, blue_v_min_),
                                        cv::Scalar(blue_h_max_, 255, 255), mb);
                            // [cv] countNonZero：m1 | m2 计数
                            n_red = cv::countNonZero(m1 | m2);
                            // [cv] countNonZero：mb 计数
                            n_blue = cv::countNonZero(mb);
                        }
                        d.is_red = (n_red >= n_blue);

                        // position = PnP tvec（相机系，未平滑）
                        d.pose.position.x = tg.t[0];
                        d.pose.position.y = tg.t[1];
                        d.pose.position.z = tg.t[2];

                        // orientation = rvec 转四元数
                        tf2::Matrix3x3 m3(tg.R.at<double>(0,0), tg.R.at<double>(0,1), tg.R.at<double>(0,2),
                                          tg.R.at<double>(1,0), tg.R.at<double>(1,1), tg.R.at<double>(1,2),
                                          tg.R.at<double>(2,0), tg.R.at<double>(2,1), tg.R.at<double>(2,2));
                        tf2::Quaternion qd;
                        m3.getRotation(qd);
                        d.pose.orientation.x = qd.x();
                        d.pose.orientation.y = qd.y();
                        d.pose.orientation.z = qd.z();
                        d.pose.orientation.w = qd.w();

                        arr.detections.push_back(d);
                    }
                    if (!arr.detections.empty())
                        detection_array_pub_.publish(arr);
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
                            (stamp - last_stamp_) < time_window_s_) {
                            double dt = stamp - last_stamp_;
                            double fx = K_.at<double>(0, 0);
                            double max_px = fx * max_speed_ * dt /
                                            std::max(min_speed_z_, targets[k].z) * teleport_scale_ + teleport_margin_;
                            // [cv] norm：距离/长度 → d
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


                    // ---- 相邻帧锁定 + 最正对兜底 ----
                    int best = 0;
                    bool locked = false;
                    if (lock_left_ > 0) {
                        double bestd = 1e18; int cand = -1;
                        for (size_t k = 0; k < targets.size(); ++k) {
                            if (lock_ratio_ > 1e-3 &&
                                std::abs(targets[k].ratio - lock_ratio_) > lock_ratio_tol_ * lock_ratio_) continue;
                            double d = cv::norm(targets[k].center2d - lock_pred_);
                            if (d < bestd) { bestd = d; cand = (int)k; }
                        }
                        if (cand >= 0 && bestd < lock_pred_gate_px_) { best = cand; locked = true; lock_miss_ = 0; }
                        else if (++lock_miss_ > 7) { lock_left_ = 0; lock_miss_ = 0; }
                    }
                    if (!locked) {
                        for (size_t k = 1; k < targets.size(); ++k)
                            if (std::abs(targets[k].avg_tilt) < std::abs(targets[best].avg_tilt))
                                best = static_cast<int>(k);
                    }

                    // ---- 整车中心聚合 ----
                    cv::Point2f vc(0, 0);
                    for (size_t k = 0; k < targets.size(); ++k) vc += targets[k].center2d;
                    vc *= (1.0f / targets.size());

                    // ---- 主目标平滑 + 下坠补偿 + TF ----
                    Target& mt = targets[best];
                    if (lock_left_ > 0) --lock_left_; else lock_left_ = lock_frames_;
                    lock_prev_ = lock_center_;
                    lock_center_ = mt.center2d;
                    lock_pred_ = lock_center_ + (lock_center_ - lock_prev_);
                    lock_ratio_ = mt.ratio;
                    lock_miss_ = 0;
                    if (mt.has_pose) {
                        {
                        cv::Vec3d t2(mt.t[0], -mt.t[1], mt.t[2]);

                        cv::Vec3d avg_t;
                        double avg_yaw = 0.0;
                        pushSmooth(t2, mt.yaw, avg_t, avg_yaw);

                        // ---- 坐标死区 ----
                        bool moved = true;
                        if (has_sent_)
                            // [cv] norm：距离/长度 → moved
                            moved = cv::norm(mt.center2d - last_sent_px_) > deadzone_px_;

                        if (moved) {
                            geometry_msgs::TransformStamped ts;
                            ts.header.stamp = msg->header.stamp;
                            ts.header.frame_id = camera_frame_;
                            ts.child_frame_id = tf_child_;
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


                        if (has_vsample_ && stamp > v_prev_stamp_ + 1e-4) {
                            double vdt = stamp - v_prev_stamp_;
                            cv::Vec3d nv = (avg_t - v_prev_pos_) * (1.0 / vdt);
                            // [cv] norm：距离/长度 → 
                            if (cv::norm(nv) <= max_speed_) { vel_ = nv; has_vel_ = true; }
                            else has_vel_ = false;
                        }
                        v_prev_pos_ = avg_t;
                        v_prev_stamp_ = stamp;
                        has_vsample_ = true;

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
                        // 旋转四点画封闭四边形（随倾角变化）
                        std::vector<cv::Point> quad;
                        for (int c = 0; c < 4; ++c)
                            quad.push_back(cv::Point((int)tg.corners[c].x,
                                                     (int)tg.corners[c].y));
                        // [cv] polylines：画四边形到 canvas
                        cv::polylines(canvas, quad, true, cv::Scalar(0, 0, 255), thick);
                        for (int c = 0; c < 4; ++c)
                            // [cv] circle：画角点到 canvas
                            cv::circle(canvas, tg.corners[c], 3, cv::Scalar(255, 0, 0), -1);

                        // projectPoints 三轴：原点 -> X/Y/Z（各 3cm）
                        if (tg.has_pose && tg.has_r) {
                            std::vector<cv::Point3f> axis;
                            axis.push_back(cv::Point3f(0, 0, 0));
                            axis.push_back(cv::Point3f(0.03f, 0, 0));
                            axis.push_back(cv::Point3f(0, 0.03f, 0));
                            axis.push_back(cv::Point3f(0, 0, 0.03f));
                            cv::Mat rvec_ax;
                            // [cv] Rodrigues：tg.R ↔ rvec_ax
                            cv::Rodrigues(tg.R, rvec_ax);
                            cv::Mat tvec_ax = (cv::Mat_<double>(3, 1) <<
                                               tg.t[0], tg.t[1], tg.t[2]);
                            std::vector<cv::Point2f> proj;
                            // [cv] projectPoints：3D 轴点 → D_ 像素
                            cv::projectPoints(axis, rvec_ax, tvec_ax, K_, D_, proj);
                            if (proj.size() == 4) {
                                // [cv] line：画线到 canvas
                                cv::line(canvas, proj[0], proj[1], cv::Scalar(0, 0, 255), 2);   // X 红
                                // [cv] line：画线到 canvas
                                cv::line(canvas, proj[0], proj[2], cv::Scalar(0, 255, 0), 2);   // Y 绿
                                // [cv] line：画线到 canvas
                                cv::line(canvas, proj[0], proj[3], cv::Scalar(255, 0, 0), 2);   // Z 蓝
                            }
                        }

                        std::string info = "ID:" + std::to_string(tg.id) +
                                           (tg.is_large ? " L" : " S");
                        // [cv] putText：文字到 canvas
                        cv::putText(canvas, info,
                                    cv::Point((int)tg.corners[0].x,
                                              std::max(0, (int)tg.corners[0].y - 10)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                    cv::Scalar(0, 0, 255), thick > 2 ? 2 : 1);
                    }

                    // 车体中心十字星
                    // [cv] line：画线到 canvas
                    cv::line(canvas, cv::Point((int)vc.x - 12, (int)vc.y),
                             cv::Point((int)vc.x + 12, (int)vc.y),
                             cv::Scalar(255, 255, 0), 2);
                    // [cv] line：画线到 canvas
                    cv::line(canvas, cv::Point((int)vc.x, (int)vc.y - 12),
                             cv::Point((int)vc.x, (int)vc.y + 12),
                             cv::Scalar(255, 255, 0), 2);

                    if (mt.has_pose) {
                        ROS_INFO_THROTTLE(1.0,
                            "v3 id=%d %s ratio=%.2f z=%.2f yaw=%.1f",
                            mt.id, mt.is_large ? "L" : "S", mt.ratio, mt.z,
                            mt.yaw * 180.0 / CV_PI);
                    }

                } else {
                    // ---- 丢帧匀速外推：最近两帧速度直线外推，维持轨迹 ----
                    bool pred_ok = false;
                    if (has_last_ && has_vel_ && lost_frames_ < hold_frames_) {
                        double pdt = stamp - v_prev_stamp_;
                        // [cv] norm：距离/长度 → 
                        if (pdt > 0.0 && pdt < time_window_s_ && cv::norm(vel_) <= hold_max_speed_) {
                            cv::Vec3d pp = v_prev_pos_ + vel_ * pdt;
                            geometry_msgs::TransformStamped ts;
                            ts.header.stamp = msg->header.stamp;
                            ts.header.frame_id = camera_frame_;
                            ts.child_frame_id = tf_child_;
                            ts.transform.translation.x = pp[0];
                            ts.transform.translation.y = pp[1];
                            ts.transform.translation.z = pp[2];
                            tf2::Quaternion q;
                            q.setRPY(0.0, 0.0, last_yaw_);
                            ts.transform.rotation.x = q.x();
                            ts.transform.rotation.y = q.y();
                            ts.transform.rotation.z = q.z();
                            ts.transform.rotation.w = q.w();
                            tf_broadcaster_.sendTransform(ts);
                            // [cv] putText：文字到 canvas
                            cv::putText(canvas, "PRED", cv::Point(70, 64),
                                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                        cv::Scalar(0, 200, 255), 2);
                            pred_ok = true;
                        }
                    }
                    ++lost_frames_;
                    if (lost_frames_ >= std::max(hold_frames_, 5)) {
                        has_last_ = false; has_sent_ = false;
                        has_vel_ = false; has_vsample_ = false;
                        pos_queue_.clear(); yaw_queue_.clear();
                        lost_frames_ = 0;
                    }
                    ROS_INFO_THROTTLE(0.5, pred_ok ? "[Hold] 匀速外推 %d/%d" : "[Hold] 丢帧 %d/%d",
                                      lost_frames_, hold_frames_);
                }
            } else {
                ++lost_frames_;
            }

            // 兜底模式可视化标记

            cv_bridge::CvImage out_img;
            out_img.header.stamp = msg->header.stamp;
            out_img.header.frame_id = msg->header.frame_id;
            out_img.encoding = "bgr8";
            out_img.image = canvas;
            image_pub_.publish(out_img.toImageMsg());

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
    ros::Publisher detection_array_pub_;
    boost::shared_ptr<dynamic_reconfigure::Server<final_text::VisionConfig> > dr_server_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 动态参数（22 点之 13）
    double aspect_min_, aspect_max_;
    double min_dist_x_, max_dist_x_, max_angle_diff_;
    double ratio_s_max_, ratio_l_max_, ratio_min_;
    double armor_large_w_, armor_large_h_, armor_small_w_, armor_small_h_;
    std::string image_topic_, info_topic_, debug_topic_, detection_topic_, tf_child_;
    int sub_queue_, pub_queue_, min_area_;
    double fcs_min_area_, dedup_px_, teleport_margin_;
    double max_z_, deadzone_px_, uniqueness_dt_, max_speed_;
    int morph_size_;
    int hsv_sv_min_;
    int red_h_min_, red_h_max_;
    double roi_y_ratio_;
    int yellow_h_min_, yellow_h_max_, yellow_s_min_, yellow_v_min_;
    int bar_v_min_, bar_keep_n_;
    double guard_y_ratio_, dyn_v_offset_, pair_y_ratio_;
    double fcs_four_span_, fcs_four_tol_;
    int blue_h_min_, blue_h_max_, blue_s_min_, blue_v_min_;
    bool   white_enable_;

    // 状态
    std::deque<cv::Vec3d> pos_queue_;
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
    int    hold_frames_;
    double hold_max_speed_;
    bool   has_vsample_ = false, has_vel_ = false;
    cv::Vec3d v_prev_pos_{0, 0, 0}, vel_{0, 0, 0};
    double v_prev_stamp_ = 0.0;
    int  assist_left_ = 0;
    int  assist_lost_ = 0;


    bool has_sent_ = false;
    cv::Point2f last_sent_px_ = cv::Point2f(0, 0);

    std::atomic<bool> is_ready_;
    bool transport_ready_;

    cv::Mat K_, D_;
    double time_window_s_ = 0.5;
    double digit_thres_ = 0.5;
    int    lock_frames_ = 20, lock_left_ = 0, lock_miss_ = 0;
    cv::Point2f lock_center_{0,0}, lock_prev_{0,0}, lock_pred_{0,0};
    double lock_ratio_ = 0.0, lock_ratio_tol_ = 0.1, lock_gate_px_ = 50.0, lock_pred_gate_px_ = 250.0;
    double dyn_v_min_ = 180.0, z_min_ = 0.3, z_max_ = 15.0;
    double teleport_scale_ = 1.5, min_speed_z_ = 0.5;
    bool nn_enable_ = false;
    std::string nn_model_path_;
    std::unique_ptr<IDetector> nn_detector_;
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
    // [cv] cvtColor：bgr → hsv（HSV/灰度）；下游：inRange 取颜色掩膜
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // [cv] inRange：hsv 上取阈值 → mask_r1（0/255 掩膜）
    cv::inRange(hsv, lo_r, hi_r, mask_r1);
    // [cv] inRange：hsv 上取阈值 → mask_r2（0/255 掩膜）
    cv::inRange(hsv, cv::Scalar(170, sv_min, sv_min), cv::Scalar(179, 255, 255), mask_r2);
    // [cv] bitwise_or：合并掩膜 → mask_r
    cv::bitwise_or(mask_r1, mask_r2, mask_r);

    // [cv] inRange：hsv 上取阈值 → mask_b（0/255 掩膜）
    cv::inRange(hsv, lo_b, hi_b, mask_b);
    // [cv] inRange：hsv 上取阈值 → mask_g（0/255 掩膜）
    cv::inRange(hsv, lo_g, hi_g, mask_g);

    // 黄色/橙色（H 10~40 可调）
    cv::Mat mask_y;
    // [cv] inRange：hsv 上取阈值 → （0/255 掩膜）
    cv::inRange(hsv, cv::Scalar(y_h_min, y_s_min, y_v_min),
                     cv::Scalar(y_h_max, 255, 255), mask_y);

    // [cv] bitwise_or：合并掩膜 → mask_all
    cv::bitwise_or(mask_r, mask_b, mask_all);
    // [cv] bitwise_or：合并掩膜 → mask_all
    cv::bitwise_or(mask_all, mask_g, mask_all);
    // [cv] bitwise_or：合并掩膜 → mask_all
    cv::bitwise_or(mask_all, mask_y, mask_all);

    // C1：3x3 + MORPH_OPEN
    // [cv] getStructuringElement：构造形态学核
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(morph_size, morph_size));
    // [cv] morphologyEx：mask_all → cv::MORPH_OPEN，去噪/连通断裂
    cv::morphologyEx(mask_all, mask_all, cv::MORPH_OPEN, kernel);

    // [cv] bitwise_and：掩膜相交 → res
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
