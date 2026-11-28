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
#include <final_text/VisionConfig.h>
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
        pnh_.param("bar_h_ratio_min", bar_h_ratio_min_, 0.6);   // 两灯条高度比下限
        pnh_.param("pair_dy_ratio", pair_dy_ratio_, 0.4);       // 两灯条竖直错位上限（×灯条长）
        pnh_.param("quad_align_min", quad_align_min_, 0.6);     // 四边形对边一致性下限
        pnh_.param("quad_diag_tol", quad_diag_tol_, 0.35);      // 对角线差上限（相对）
        pnh_.param("ratio_min",   ratio_min_, 0.4);
        // 装甲板本体尺寸（米），按考核文档：大 235×127mm，小 120×125mm
        pnh_.param("armor_large_w", armor_large_w_, 0.230);
        pnh_.param("armor_large_h", armor_large_h_, 0.060);
        pnh_.param("armor_small_w", armor_small_w_, 0.135);
        pnh_.param("armor_small_h", armor_small_h_, 0.060);
        pnh_.param("pnp_scale", pnp_scale_, 1.0);   // PnP 物体尺寸缩放（测距标定）
        pnh_.param("max_z",       max_z_, 8.0);
        pnh_.param("deadzone_px", deadzone_px_, 5.0);
        pnh_.param("uniqueness_dt", uniqueness_dt_, 0.15);
        pnh_.param("max_speed",   max_speed_, 8.0);     // 车体物理极限 m/s
        pnh_.param("morph_size",  morph_size_, 3);
        pnh_.param("hsv_sv_min",   hsv_sv_min_, 100);
        pnh_.param("red_h_min",    red_h_min_, 0);
        pnh_.param("red_h_max",    red_h_max_, 10);
        pnh_.param("roi_y_ratio",  roi_y_ratio_, 0.0);   // 先关闭 ROI
        pnh_.param("bar_v_min",    bar_v_min_, 170);
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
        pnh_.param("blue_h_min",   blue_h_min_, 85);
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

        // ---- 话题 / 队列 / 杂项阈值（全部外置）----
        pnh_.param<std::string>("image_topic", image_topic_, "/hk_camera/image_raw");
        pnh_.param<std::string>("camera_info_topic", info_topic_, "/hk_camera/camera_info");
        pnh_.param<std::string>("debug_topic", debug_topic_, "/ft/debug/image");
        pnh_.param<std::string>("detection_topic", detection_topic_, "/ft/detection_v3split");
        pnh_.param("sub_queue", sub_queue_, 1);
        pnh_.param("pub_queue", pub_queue_, 1);
        pnh_.param<std::string>("tf_child_frame", tf_child_, "armor_v3_split");
        pnh_.param("min_area", min_area_, 40);
        pnh_.param("fcs_min_area", fcs_min_area_, 150.0);
        pnh_.param("dedup_px", dedup_px_, 120.0);
        pnh_.param("teleport_margin", teleport_margin_, 20.0);
        pnh_.param("plane_filter", plane_filter_, true);      // 共面过滤（排除反光）
        pnh_.param("plane_dist_ratio", plane_dist_ratio_, 1.0); // 阈值 = 灯条长度 * ratio
        pnh_.param("hold_frames", hold_frames_, 8);
        pnh_.param("lock_frames", lock_frames_, 20);
        pnh_.param("lock_gate_px", lock_gate_px_, 50.0);
        pnh_.param("lock_ratio_tol", lock_ratio_tol_, 0.1);   // 长宽比相似容差(±10%)
        pnh_.param("hold_max_speed", hold_max_speed_, 8.0);      // 外推最大速度 m/s

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
        param_timer_ = pnh_.createTimer(ros::Duration(0.5), &VisionNodeV3::reloadParams, this);

        dr_server_.reset(new dynamic_reconfigure::Server<final_text::VisionConfig>(pnh_));
        dr_server_->setCallback(
            boost::bind(&VisionNodeV3::drCallback, this, _1, _2));
        detection_array_pub_ = nh_.advertise<ft_msgs::Detections>(detection_topic_, pub_queue_);
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

    struct Knee {
        sensor_msgs::ImageConstPtr msg;
        cv::Mat bgr, canvas; double stamp=0.0; std::string frame_id;
        cv::Mat colored, hsv_full, v_chan, high_mask, seg_mask;
        double max_v=0.0; cv::Point max_loc; bool guard_block=false; double dyn_v_thresh=0.0;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<Bar> bars; std::vector<Target> targets;
        int best=0; bool locked=false; Target mt; bool has_mt=false;
        cv::Point2f vc; bool has_vc=false;
        cv::Vec3d tvec{0,0,0}, avg_t{0,0,0}; double yaw=0.0, avg_yaw=0.0;
        ft_msgs::Detections detections; geometry_msgs::TransformStamped tf;
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
            // [cv] minMaxLoc：最亮值/位置 → knee.max_v/knee.max_loc
            cv::minMaxLoc(v(q[i]), nullptr, &mx);
            out[i] = mx;
        }
    }


    // dynamic_reconfigure 回调：先写回参数服务器，再立即生效
    void drCallback(final_text::VisionConfig& cfg, uint32_t level) {
        (void)level;
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
        pnh_.setParam("ratio_s_max", cfg.ratio_s_max);
        pnh_.setParam("pair_y_ratio", cfg.pair_y_ratio);
        pnh_.setParam("white_enable", cfg.white_enable);

        red_h_min_ = cfg.red_h_min; red_h_max_ = cfg.red_h_max;
        hsv_sv_min_ = cfg.hsv_sv_min;
        blue_h_min_ = cfg.blue_h_min; blue_h_max_ = cfg.blue_h_max;
        blue_s_min_ = cfg.blue_s_min; blue_v_min_ = cfg.blue_v_min;
        bar_v_min_ = cfg.bar_v_min; min_area_ = cfg.min_area;
        max_dist_x_ = cfg.max_dist_x; max_angle_diff_ = cfg.max_angle_diff;
        aspect_min_ = cfg.aspect_min; aspect_max_ = cfg.aspect_max;
        ratio_min_ = cfg.ratio_min; ratio_l_max_ = cfg.ratio_l_max;
        ratio_s_max_ = cfg.ratio_s_max;
        pair_y_ratio_ = cfg.pair_y_ratio; white_enable_ = cfg.white_enable;
        pnh_.setParam("bar_h_ratio_min", cfg.bar_h_ratio_min_v3); bar_h_ratio_min_ = cfg.bar_h_ratio_min_v3;
        pnh_.setParam("pair_dy_ratio", cfg.pair_dy_ratio_v3); pair_dy_ratio_ = cfg.pair_dy_ratio_v3;
        pnh_.setParam("quad_align_min", cfg.quad_align_min_v3); quad_align_min_ = cfg.quad_align_min_v3;
        pnh_.setParam("quad_diag_tol", cfg.quad_diag_tol_v3); quad_diag_tol_ = cfg.quad_diag_tol_v3;

    }

    // 每 0.5s 重新读取参数：rosparam set 后实时生效
    void reloadParams(const ros::TimerEvent&) {
        pnh_.getParam("bar_v_min", bar_v_min_);
        pnh_.getParam("hsv_sv_min", hsv_sv_min_);
        pnh_.getParam("red_h_min", red_h_min_);
        pnh_.getParam("red_h_max", red_h_max_);
        pnh_.getParam("blue_h_min", blue_h_min_);
        pnh_.getParam("blue_h_max", blue_h_max_);
        pnh_.getParam("blue_s_min", blue_s_min_);
        pnh_.getParam("blue_v_min", blue_v_min_);
        pnh_.getParam("yellow_h_min", yellow_h_min_);
        pnh_.getParam("yellow_h_max", yellow_h_max_);
        pnh_.getParam("yellow_s_min", yellow_s_min_);
        pnh_.getParam("yellow_v_min", yellow_v_min_);
        pnh_.getParam("aspect_min", aspect_min_);
        pnh_.getParam("aspect_max", aspect_max_);
        pnh_.getParam("min_dist_x", min_dist_x_);
        pnh_.getParam("max_dist_x", max_dist_x_);
        pnh_.getParam("ratio_min", ratio_min_);
        pnh_.getParam("ratio_l_max", ratio_l_max_);
        pnh_.getParam("armor_large_w", armor_large_w_);
        pnh_.getParam("armor_large_h", armor_large_h_);
        pnh_.getParam("armor_small_w", armor_small_w_);
        pnh_.getParam("armor_small_h", armor_small_h_);
        pnh_.getParam("pnp_scale", pnp_scale_);
        pnh_.getParam("bar_h_ratio_min", bar_h_ratio_min_);
        pnh_.getParam("pair_dy_ratio", pair_dy_ratio_);
        pnh_.getParam("quad_align_min", quad_align_min_);
        pnh_.getParam("quad_diag_tol", quad_diag_tol_);
        pnh_.getParam("pair_y_ratio", pair_y_ratio_);
        pnh_.getParam("guard_y_ratio", guard_y_ratio_);
        pnh_.getParam("near_area", near_area_);
        pnh_.getParam("merge_bars", merge_bars_);
        pnh_.getParam("white_enable", white_enable_);
        pnh_.getParam("min_area", min_area_);
        pnh_.getParam("fcs_min_area", fcs_min_area_);
        pnh_.getParam("dedup_px", dedup_px_);
        pnh_.getParam("teleport_margin", teleport_margin_);
        pnh_.getParam("plane_filter", plane_filter_);
        pnh_.getParam("plane_dist_ratio", plane_dist_ratio_);
        pnh_.getParam("hold_frames", hold_frames_);
        pnh_.getParam("hold_max_speed", hold_max_speed_);
    }

    // ---------- 图像回调 ----------
    // ============================================================
    // 数据流总览（变量怎么走）
    //   Image msg → bgr → colorSegment → knee.colored
    //        ├─ cvtColor → knee.hsv_full → knee.v_chan（亮度/高光守卫）
    //        └─ cvtColor → knee.seg_mask → findContours → knee.contours
    //                                                    └→ knee.bars（灯条）
    //                                                         └→ knee.targets（候选）
    //                                                              └→ best → mt
    //                                                                   └→ rvec/tvec
    //                                                                        ├→ avg_t/avg_yaw → TF armor_v3
    //                                                                        └→ /ft/detection
    //   knee.canvas（绘图底板）→ /ft/debug/image
    //   丢帧：vel_ + v_prev_pos_ → 预测 TF(PRED)；FCS 用 prev_center2d_ 残影
    //
    //   关键变量去向：
    //     knee.colored    → knee.hsv_full / knee.seg_mask            （颜色与灰度）
    //     knee.hsv_full   → inRange                        （颜色掩膜）
    //     knee.v_chan     → knee.max_v/knee.high_mask/bar_v_min      （高光与亮度过滤）
    //     knee.seg_mask   → findContours                   （轮廓）
    //     knee.contours   → 面积/长宽比/亮度 → knee.bars         （灯条）
    //     knee.bars       → 断笔合并/共面/配对 → knee.targets     （候选）
    //     knee.targets    → best → mt                      （主目标）
    //     rvec/tvec  → Target.t → avg_t/avg_yaw → TF  （位姿）
    //     tvec       → /ft/detection                  （下游跟踪）
    //     prev_center2d_/prev_had_target_ → 锁定/FCS/外推
    //     vel_/v_prev_pos_ → 丢帧预测 TF
    // ============================================================

    void colorBlock(Knee& knee) {
        colorSegment(knee.canvas, knee.colored,
                     cv::Scalar(red_h_min_, hsv_sv_min_, hsv_sv_min_), cv::Scalar(red_h_max_, 255, 255),
                     cv::Scalar(blue_h_min_, blue_s_min_, blue_v_min_), cv::Scalar(blue_h_max_, 255, 255),
                     cv::Scalar(50, hsv_sv_min_, hsv_sv_min_), cv::Scalar(70, 255, 255),
                     morph_size_, hsv_sv_min_,
                     yellow_h_min_, yellow_h_max_, yellow_s_min_, yellow_v_min_);

        // 亮度筛选用：单独提取 V 通道
        // [cv] cvtColor：knee.canvas → knee.hsv_full（HSV/灰度）；下游：inRange 取颜色掩膜
        cv::cvtColor(knee.canvas, knee.hsv_full, cv::COLOR_BGR2HSV);
        cv::extractChannel(knee.hsv_full, knee.v_chan, 2);
    }

    void guardBlock(Knee& knee) {
        // 1) 全图最高光定位
        // [cv] minMaxLoc：最亮值/位置 → knee.max_v/knee.max_loc
        cv::minMaxLoc(knee.v_chan, nullptr, &knee.max_v, nullptr, &knee.max_loc);

        // 2) 全局空间守卫：最亮点在画面极底部 -> 大概率地面反光
        knee.guard_block = (knee.max_loc.y > height_ * guard_y_ratio_);
        if (knee.guard_block) {
            // 最亮点在底部：判为地面反光，屏蔽底部区域，用其余灯条继续识别
            int y0 = (int)(height_ * guard_y_ratio_);
            if (y0 > 0 && y0 < knee.colored.rows)
                knee.colored.rowRange(y0, knee.colored.rows).setTo(cv::Scalar(0, 0, 0));
            ROS_INFO_THROTTLE(1.0, "[Guard] 底部反光 y=%d，取次高光灯条", knee.max_loc.y);
        }

        // 3) 动态高亮掩膜（解决过曝）
        double dyn_v_thresh = std::max(180.0, knee.max_v - dyn_v_offset_);
        cv::compare(knee.v_chan, dyn_v_thresh, knee.high_mask, cv::CMP_GT);
    }

    void barsBlock(Knee& knee) {
        // =======================================================
        // 1. 灯条提取：矩质心 + 长宽比 2.0~6.0
        // =======================================================
        // [flow] knee.bars：灯条集合 → 断笔合并/共面过滤/配对 → knee.targets
        // [cv] cvtColor：knee.colored → knee.seg_mask（HSV/灰度）；下游：形态学 → findContours 取轮廓
        cv::cvtColor(knee.colored, knee.seg_mask, cv::COLOR_BGR2GRAY);

        // 3) 过曝中心融合：颜色区域膨胀后与高亮 mask 相交，再并入分割 mask
        if (!knee.high_mask.empty()) {
            cv::Mat color_mask, color_dil, high_fused;
            // [cv] threshold：knee.seg_mask 二值化 → color_mask
            cv::threshold(knee.seg_mask, color_mask, 1, 255, cv::THRESH_BINARY);
            // [cv] getStructuringElement：构造形态学核
            cv::Mat k3 = cv::getStructuringElement(cv::MORPH_RECT,
                                                   cv::Size(3, 3));
            // [cv] dilate：color_mask → color_dil，膨胀补断裂
            cv::dilate(color_mask, color_dil, k3, cv::Point(-1, -1), 2);
            // [cv] bitwise_and：掩膜相交 → high_fused
            cv::bitwise_and(knee.high_mask, color_dil, high_fused);
            // [cv] bitwise_or：合并掩膜 → knee.seg_mask；下游：形态学 → findContours 取轮廓
            cv::bitwise_or(knee.seg_mask, high_fused, knee.seg_mask);
        }

        if (edge_fuse_) {
            cv::Mat gray, edges, sx, absx, em;
            // [cv] cvtColor：knee.canvas → gray（HSV/灰度）；下游：亮度/轮廓
            cv::cvtColor(knee.canvas, gray, cv::COLOR_BGR2GRAY);
            cv::Canny(gray, edges, 60, 160);
            cv::Sobel(gray, sx, CV_16S, 1, 0, 3);
            cv::convertScaleAbs(sx, absx);
            // [cv] threshold：absx 二值化 → em
            cv::threshold(absx, em, 60, 255, cv::THRESH_BINARY);
            // [cv] bitwise_or：合并掩膜 → em
            cv::bitwise_or(edges, em, em);
            // [cv] dilate：em → em，膨胀补断裂
            cv::dilate(em, em, cv::getStructuringElement(cv::MORPH_RECT,
                                                         cv::Size(3, 3)));
            // [cv] bitwise_and：掩膜相交 → knee.seg_mask；下游：形态学 → findContours 取轮廓
            cv::bitwise_and(knee.seg_mask, em, knee.seg_mask);
        }

        // [cv] findContours：从 knee.seg_mask 提取 → knee.contours；下游：逐轮廓筛成灯条 knee.bars
        cv::findContours(knee.seg_mask, knee.contours, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        // 近战自适应：最大颜色轮廓足够大 -> 用更低 S/V 重分割一次
        double max_c_area = 0;
        for (size_t ci = 0; ci < knee.contours.size(); ++ci)
            // [cv] contourArea：算 max_c_area 面积
            max_c_area = std::max(max_c_area, cv::contourArea(knee.contours[ci]));
        if (max_c_area > near_area_) {
            cv::Mat colored2;
            colorSegment(knee.canvas, colored2,
                         cv::Scalar(red_h_min_, near_sv_, near_sv_), cv::Scalar(red_h_max_, 255, 255),
                         cv::Scalar(blue_h_min_, near_sv_, near_sv_), cv::Scalar(blue_h_max_, 255, 255),
                         cv::Scalar(50, near_sv_, near_sv_), cv::Scalar(70, 255, 255),
                         morph_size_, near_sv_,
                         yellow_h_min_, yellow_h_max_, near_sv_, near_sv_);
            // [cv] bitwise_or：合并掩膜 → knee.colored
            cv::bitwise_or(knee.colored, colored2, knee.colored);
            // [cv] cvtColor：knee.colored → knee.seg_mask（HSV/灰度）；下游：形态学 → findContours 取轮廓
            cv::cvtColor(knee.colored, knee.seg_mask, cv::COLOR_BGR2GRAY);
            // [cv] findContours：从 knee.seg_mask 提取 → knee.contours；下游：逐轮廓筛成灯条 knee.bars
            cv::findContours(knee.seg_mask, knee.contours, cv::RETR_EXTERNAL,
                             cv::CHAIN_APPROX_SIMPLE);
            ROS_INFO_THROTTLE(1.0, "[Near] 近战重分割 (area=%.0f, S/V=%d)",
                              max_c_area, near_sv_);
        }

        for (size_t i = 0; i < knee.contours.size(); ++i) {
            // [cv] contourArea：算 cv::contourArea(knee.contours[i]) < min_area_ 面积
            if (cv::contourArea(knee.contours[i]) < min_area_) continue;
            // [cv] minAreaRect：knee.contours[i] → rr（中心/长宽/倾角）；下游：生成 Bar（中心/长宽/角度）
            cv::RotatedRect rr = cv::minAreaRect(knee.contours[i]);
            double w = rr.size.width, h = rr.size.height;
            if (std::min(w, h) <= 0) continue;
            double aspect = std::max(w, h) / std::min(w, h);
            if (aspect < aspect_min_ || aspect > aspect_max_) continue;

            // 最高亮度过滤：真灯条主动发光，反光达不到这条线
            cv::Mat cmask(knee.canvas.size(), CV_8UC1, cv::Scalar(0));
            cv::drawContours(cmask, knee.contours, static_cast<int>(i),
                             cv::Scalar(255), cv::FILLED);
            // [cv] mean：knee.v_chan 掩膜内均值
            double mean_v = cv::mean(knee.v_chan, cmask)[0];
            if (mean_v < bar_v_min_) continue;

            Bar bar;
            bar.rr = rr; rr.points(bar.pts);

            // [cv] moments：knee.contours[i] → m，算质心
            cv::Moments m = cv::moments(knee.contours[i]);
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
            bar.contour = knee.contours[i];

            // 补丁4：y 方向 ROI，滤掉画面下部的地面反光
            if (roi_y_ratio_ > 0.0 &&
                bar.center.y > height_ * roi_y_ratio_) continue;

            knee.bars.push_back(bar);

            for (int k = 0; k < 4; ++k)
                // [cv] line：画线到 knee.canvas
                cv::line(knee.canvas, bar.pts[k], bar.pts[(k + 1) % 4],
                         cv::Scalar(0, 255, 255), 2);
        }

        // 断笔重连：共线且竖直方向相邻的短灯条合并
        if (merge_bars_) {
            bool merged_any = true;
            while (merged_any) {
                merged_any = false;
                for (size_t i = 0; i < knee.bars.size() && !merged_any; ++i) {
                    for (size_t j = i + 1; j < knee.bars.size(); ++j) {
                        if (std::abs(knee.bars[i].center.x - knee.bars[j].center.x) > 18.0)
                            continue;
                        double h1 = knee.bars[i].long_h, h2 = knee.bars[j].long_h;
                        if (std::min(h1, h2) < 0.5 * std::max(h1, h2)) continue;
                        double t1 = knee.bars[i].center.y - h1 * 0.5;
                        double b1 = knee.bars[i].center.y + h1 * 0.5;
                        double t2 = knee.bars[j].center.y - h2 * 0.5;
                        double b2 = knee.bars[j].center.y + h2 * 0.5;
                        double gap = std::max(t1, t2) - std::min(b1, b2);
                        if (gap > 0.6 * std::max(h1, h2)) continue;

                        std::vector<cv::Point> all = knee.bars[i].contour;
                        all.insert(all.end(), knee.bars[j].contour.begin(), knee.bars[j].contour.end());
                        // [cv] minAreaRect：all → rr（中心/长宽/倾角）；下游：生成 Bar（中心/长宽/角度）
                        cv::RotatedRect rr = cv::minAreaRect(all);
                        cv::Point2f pts[4]; rr.points(pts);
                        knee.bars[i].rr = rr;
                        for (int k = 0; k < 4; ++k) knee.bars[i].pts[k] = pts[k];
                        knee.bars[i].contour = all;
                        knee.bars[i].center = cv::Point2f(rr.center.x, rr.center.y);
                        // 合并后按新矩形长边重算 tilt / 长边
                        cv::Point2f e1 = pts[1] - pts[0];
                        cv::Point2f e2 = pts[2] - pts[1];
                        // [cv] norm：距离/长度 → 
                        cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
                        double ntl = std::atan2(lv.x, lv.y);
                        if (ntl > CV_PI / 2) ntl -= CV_PI;
                        if (ntl < -CV_PI / 2) ntl += CV_PI;
                        knee.bars[i].tilt = ntl;
                        // [cv] norm：距离/长度 → 
                        knee.bars[i].long_h = std::max(cv::norm(e1), cv::norm(e2));
                        knee.bars[i].v = std::max(knee.bars[i].v, knee.bars[j].v);
                        knee.bars.erase(knee.bars.begin() + j);
                        merged_any = true;
                        break;
                    }
                }
            }
        }

        // 择优保留：只留最亮的 N 根
        std::sort(knee.bars.begin(), knee.bars.end(),
                  [](const Bar& a, const Bar& b) { return a.v > b.v; });
        if (static_cast<int>(knee.bars.size()) > bar_keep_n_)
            knee.bars.resize(bar_keep_n_);

        std::sort(knee.bars.begin(), knee.bars.end(),
                  [](const Bar& a, const Bar& b) {
                      return a.center.x < b.center.x;
                  });

        // ---- 灯条级共面过滤：最多灯条共识定平面，离面超过灯条长度的当反光 ----
        if (plane_filter_ && knee.bars.size() >= 3) {
            int best_cnt = 0;
            std::vector<int> best_keep;
            for (size_t i = 0; i < knee.bars.size(); ++i) {
                for (size_t j = i + 1; j < knee.bars.size(); ++j) {
                    cv::Point2f A = knee.bars[i].center, B = knee.bars[j].center;
                    cv::Point2f dir = B - A;
                    // [cv] norm：距离/长度 → dl
                    double dl = cv::norm(dir);
                    if (dl < 1.0) continue;
                    cv::Point2f n(-dir.y / dl, dir.x / dl);
                    double thr = std::max(knee.bars[i].long_h, knee.bars[j].long_h) * plane_dist_ratio_;
                    std::vector<int> keep;
                    for (size_t k = 0; k < knee.bars.size(); ++k) {
                        double d = std::abs((knee.bars[k].center.x - A.x) * n.x +
                                            (knee.bars[k].center.y - A.y) * n.y);
                        if (d <= thr) keep.push_back((int)k);
                    }
                    if ((int)keep.size() > best_cnt) { best_cnt = (int)keep.size(); best_keep = keep; }
                }
            }
            if (best_cnt >= 2 && best_cnt < (int)knee.bars.size()) {
                std::vector<Bar> kept;
                for (size_t k = 0; k < best_keep.size(); ++k) kept.push_back(knee.bars[best_keep[k]]);
                ROS_INFO_THROTTLE(1.0, "[Plane] 共识平面 %d 根，剔除反光 %d 根",
                                  best_cnt, (int)(knee.bars.size() - kept.size()));
                knee.bars.swap(kept);
            }
        }

        // =======================================================
    }

    void pairBlock(Knee& knee) {
        // 2. 配对 + 几何修正 + 幽灵剔除
        // =======================================================
        // [flow] knee.targets：候选目标 → best → mt → PnP/平滑/TF
        int armor_id = 1;

        for (size_t i = 0; i < knee.bars.size(); ++i) {
            for (size_t j = i + 1; j < knee.bars.size(); ++j) {
                double dx = knee.bars[j].center.x - knee.bars[i].center.x;
                // 补丁1：绝对距离拦截（>250 或 <15 一律不配对）
                if (dx > max_dist_x_ || dx <= min_dist_x_) continue;
                // 粗错位：以两根中较长灯条的一半长度为门限（自适应距离/尺度）
                double pair_dy_px = 0.5 * std::max(knee.bars[i].long_h, knee.bars[j].long_h);
                if (std::abs(knee.bars[i].center.y - knee.bars[j].center.y) > pair_dy_px) continue;

                double ddeg = std::abs(knee.bars[i].tilt - knee.bars[j].tilt)
                              * 180.0 / CV_PI;
                if (ddeg >= max_angle_diff_) continue;           // 平行度

                double avg_angle = 0.5 * (knee.bars[i].tilt + knee.bars[j].tilt);
                double corrected = dx / std::max(0.2, std::abs(std::cos(avg_angle)));
                double long_h = std::max(knee.bars[i].long_h, knee.bars[j].long_h);
                if (long_h < 1.0) continue;
                double ratio = corrected / long_h;
                if (ratio > ratio_l_max_ || ratio < ratio_min_) continue; // 幽灵

                double hmin = std::min(knee.bars[i].long_h, knee.bars[j].long_h);
                double hmax = std::max(knee.bars[i].long_h, knee.bars[j].long_h);
                if (hmin < bar_h_ratio_min_ * hmax) continue;          // 灯条高度差过大
                if (std::abs(knee.bars[i].center.y - knee.bars[j].center.y) >
                    pair_dy_ratio_ * std::max(1.0, hmin)) continue;    // 竖直错位

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
                edge_top_bottom(knee.bars[i], true, lt, lb);    // 左灯条外边缘
                edge_top_bottom(knee.bars[j], false, rt, rb);   // 右灯条外边缘

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

                // 四点矩形性：对边/对角线一致性，剔除斜四边形
                {
                    // [cv] norm：距离/长度 → wt
                    double wt = cv::norm(rt - lt);
                    // [cv] norm：距离/长度 → wb
                    double wb = cv::norm(rb - lb);
                    // [cv] norm：距离/长度 → hl
                    double hl = cv::norm(lb - lt);
                    // [cv] norm：距离/长度 → hr
                    double hr = cv::norm(rb - rt);
                    if (std::min(wt, wb) < quad_align_min_ * std::max(wt, wb)) continue;
                    if (std::min(hl, hr) < quad_align_min_ * std::max(hl, hr)) continue;
                    // [cv] norm：距离/长度 → d1
                    double d1 = cv::norm(rb - lt);
                    // [cv] norm：距离/长度 → d2
                    double d2 = cv::norm(rt - lb);
                    if (std::abs(d1 - d2) > quad_diag_tol_ * std::max(d1, d2)) continue;
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
                for (size_t d = 0; d < knee.targets.size(); ++d) {
                    // [cv] norm：距离/长度 → 
                    if (cv::norm(knee.targets[d].center2d - tg.center2d) < dedup_px_) {
                        dup = true; break;
                    }
                }
                if (dup) continue;

                // ---------- solvePnP ----------
                if (K_.rows == 3 && K_.cols == 3) {
                    double half_w = (is_large ? armor_large_w_ : armor_small_w_) * 0.5 *
                                    pnp_scale_;
                    double half_h = (is_large ? armor_large_h_ : armor_small_h_) * 0.5 *
                                    pnp_scale_;
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
                    if (ok && tvec.at<double>(2) > 0.3 &&
                        tvec.at<double>(2) < 15.0) {
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

                knee.targets.push_back(tg);
            }
        }

        // ---- 共面过滤：离主装甲板平面 > 灯条长度的候选判为反光 ----
        if (plane_filter_ && knee.targets.size() >= 2) {
            // 主目标：横向宽度最大
            int p = 0;
            double pw = 0;
            for (size_t k = 0; k < knee.targets.size(); ++k) {
                // [cv] norm：距离/长度 → w
                double w = cv::norm(knee.targets[k].corners[1] - knee.targets[k].corners[0]);
                if (w > pw) { pw = w; p = (int)k; }
            }
            const Target& pt = knee.targets[p];
            cv::Point2f A = (pt.corners[0] + pt.corners[3]) * 0.5f;  // 左灯条中点
            cv::Point2f B = (pt.corners[1] + pt.corners[2]) * 0.5f;  // 右灯条中点
            cv::Point2f dir = B - A;
            // [cv] norm：距离/长度 → dl
            double dl = cv::norm(dir);
            if (dl > 1.0) {
                cv::Point2f n(-dir.y / dl, dir.x / dl);
                // [cv] norm：距离/长度 → 
                double lamp_len = 0.5 * (cv::norm(pt.corners[0] - pt.corners[3]) +
                                         // [cv] norm：距离/长度 → 
                                         cv::norm(pt.corners[1] - pt.corners[2]));
                double thr = lamp_len * plane_dist_ratio_;
                std::vector<Target> kept;
                for (size_t k = 0; k < knee.targets.size(); ++k) {
                    if ((int)k == p) { kept.push_back(knee.targets[k]); continue; }
                    double d = std::abs((knee.targets[k].center2d.x - A.x) * n.x +
                                        (knee.targets[k].center2d.y - A.y) * n.y);
                    if (d <= thr) kept.push_back(knee.targets[k]);
                }
                if (kept.size() < knee.targets.size())
                    ROS_INFO_THROTTLE(1.0, "[Plane] 过滤反光 %d 个",
                                      (int)(knee.targets.size() - kept.size()));
                knee.targets.swap(kept);
            }
        }
    }

    void publishBlock(Knee& knee) {
        // ---- 发布当前帧全部检测（数组，每帧唯一 id）----
        {
            ft_msgs::Detections arr;
            arr.header = knee.msg->header;
            for (size_t k = 0; k < knee.targets.size(); ++k) {
                const Target& tg = knee.targets[k];
                if (!tg.has_pose || !tg.has_r) continue;

                ft_msgs::Detection d;
                d.header = knee.msg->header;
                d.id = tg.id;
                d.is_large_armor = tg.is_large;

                // 颜色判定：取框内红/蓝像素数
                int bx1 = knee.canvas.cols, by1 = knee.canvas.rows, bx2 = 0, by2 = 0;
                for (int c = 0; c < 4; ++c) {
                    bx1 = std::min(bx1, (int)tg.corners[c].x);
                    by1 = std::min(by1, (int)tg.corners[c].y);
                    bx2 = std::max(bx2, (int)tg.corners[c].x);
                    by2 = std::max(by2, (int)tg.corners[c].y);
                }
                cv::Rect bb(bx1, by1, std::max(1, bx2 - bx1), std::max(1, by2 - by1));
                bb &= cv::Rect(0, 0, knee.canvas.cols, knee.canvas.rows);
                int n_red = 0, n_blue = 0;
                if (bb.width > 2 && bb.height > 2) {
                    cv::Mat hsv_bb = knee.hsv_full(bb);
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
    }

    void drawBlock(Knee& knee) {
        // ---- 绘图 ----
        for (size_t k = 0; k < knee.targets.size(); ++k) {
            const Target& tg = knee.targets[k];
            int thick = (static_cast<int>(k) == knee.best) ? 4 : 2;
            // 旋转四点画封闭四边形（随倾角变化）
            std::vector<cv::Point> quad;
            for (int c = 0; c < 4; ++c)
                quad.push_back(cv::Point((int)tg.corners[c].x,
                                         (int)tg.corners[c].y));
            // [cv] polylines：画四边形到 knee.canvas
            cv::polylines(knee.canvas, quad, true, cv::Scalar(0, 255, 0), thick);
            for (int c = 0; c < 4; ++c)
                // [cv] circle：画角点到 knee.canvas
                cv::circle(knee.canvas, tg.corners[c], 3, cv::Scalar(255, 0, 0), -1);

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
                    // [cv] line：画线到 knee.canvas
                    cv::line(knee.canvas, proj[0], proj[1], cv::Scalar(0, 0, 255), 2);   // X 红
                    // [cv] line：画线到 knee.canvas
                    cv::line(knee.canvas, proj[0], proj[2], cv::Scalar(0, 255, 0), 2);   // Y 绿
                    // [cv] line：画线到 knee.canvas
                    cv::line(knee.canvas, proj[0], proj[3], cv::Scalar(255, 0, 0), 2);   // Z 蓝
                }
            }

            std::string info = "ID:" + std::to_string(tg.id) +
                               (tg.is_large ? " L" : " S");
            // [cv] putText：文字到 knee.canvas
            cv::putText(knee.canvas, info,
                        cv::Point((int)tg.corners[0].x,
                                  std::max(0, (int)tg.corners[0].y - 10)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 0, 255), thick > 2 ? 2 : 1);
        }

        // 车体中心十字星
        // [cv] line：画线到 knee.canvas
        cv::line(knee.canvas, cv::Point((int)knee.vc.x - 12, (int)knee.vc.y),
                 cv::Point((int)knee.vc.x + 12, (int)knee.vc.y),
                 cv::Scalar(255, 255, 0), 2);
        // [cv] line：画线到 knee.canvas
        cv::line(knee.canvas, cv::Point((int)knee.vc.x, (int)knee.vc.y - 12),
                 cv::Point((int)knee.vc.x, (int)knee.vc.y + 12),
                 cv::Scalar(255, 255, 0), 2);

        if (knee.mt.has_pose) {
            ROS_INFO_THROTTLE(1.0,
                "v3 id=%d %s ratio=%.2f z=%.2f yaw=%.1f",
                knee.mt.id, knee.mt.is_large ? "L" : "S", knee.mt.ratio, knee.mt.z,
                knee.mt.yaw * 180.0 / CV_PI);
        }
    }

    void outputBlock(Knee& knee) {
        // 兜底模式可视化标记
        if (assist_mode_) {
            // [cv] putText：文字到 knee.canvas
            cv::putText(knee.canvas, "ASSIST", cv::Point(70, 32),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }

        cv_bridge::CvImage out_img;
        out_img.header.stamp = knee.msg->header.stamp;
        out_img.header.frame_id = knee.msg->header.frame_id;
        out_img.encoding = "bgr8";
        out_img.image = knee.canvas;
        image_pub_.publish(out_img.toImageMsg());
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!is_ready_.load()) return;

        try {
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat img = cv_ptr->image;
            if (img.cols != width_ || img.rows != height_) return;

            cv::Mat canvas = img.clone();
            Knee knee;
            knee.msg=msg; knee.bgr=img; knee.canvas=canvas;
            knee.stamp=msg->header.stamp.toSec(); knee.frame_id=msg->header.frame_id;

            // 保留原版 debug 绿框（绝对不动）
            // [cv] rectangle：画矩形到 knee.canvas
            cv::rectangle(knee.canvas, cv::Rect(10, 10, 50, 50),
                          cv::Scalar(0, 255, 0), 2);

            // [flow] knee.colored：颜色分割结果 → knee.hsv_full / knee.seg_mask
            colorBlock(knee);
            guardBlock(knee);

            const double stamp = knee.stamp;
            int lost_this_frame = 1;

            if (!knee.colored.empty()) {
                // 显示用底板：保留原始相机画面（不再用分割结果覆盖成黑底）
                // knee.canvas 仍是 img.clone() + 绿框，检测/绘制都叠加在原视频上
                barsBlock(knee);

                pairBlock(knee);
                publishBlock(knee);

                // =======================================================
                // 3. 时空校验 + 择优 + 车体中心 + TF
                // =======================================================
                if (!knee.targets.empty()) {
                    // ---- 众数滤波（过去 10 帧 7 票）----
                    {
                        Target& first = knee.targets[0];
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
                        if (knee.targets[0].is_large != last_is_large_)
                            knee.targets[0].is_large = last_is_large_;
                    }

                    // ---- 超距 / 瞬移 ----
                    std::vector<Target> keep;
                    for (size_t k = 0; k < knee.targets.size(); ++k) {
                        if (!knee.targets[k].has_pose) { keep.push_back(knee.targets[k]); continue; }
                        if (knee.targets[k].z > max_z_) {
                            ROS_INFO_THROTTLE(1.0, "[Drop] 超距 z=%.2f m", knee.targets[k].z);
                            continue;
                        }
                        if (has_last_ && (stamp - last_stamp_) > 1e-4 &&
                            (stamp - last_stamp_) < 0.5) {
                            double dt = stamp - last_stamp_;
                            double fx = K_.at<double>(0, 0);
                            double max_px = fx * max_speed_ * dt /
                                            std::max(0.5, knee.targets[k].z) * 1.5 + teleport_margin_;
                            // [cv] norm：距离/长度 → d
                            double d = cv::norm(knee.targets[k].center2d - last_center2d_);
                            if (d > max_px) {
                                ROS_INFO_THROTTLE(1.0, "[Drop] 瞬移 %.1f px > %.1f", d, max_px);
                                continue;
                            }
                        }
                        keep.push_back(knee.targets[k]);
                    }
                    knee.targets.swap(keep);
                }

                if (!knee.targets.empty()) {
                    lost_this_frame = 0;
                    lost_frames_ = 0;


                    // ---- 目标选择：相邻帧优先锁同一块装甲板 ----
                    // [flow] best：选中索引（相邻帧中心锁定优先）→ mt
                    int best = 0;
                    bool locked = false;

                    // ---- 20 帧锁定：优先保持上一帧选中的目标 ----
                    if (lock_left_ > 0) {
                        double bestd = 1e18; int cand = -1;
                        for (size_t k = 0; k < knee.targets.size(); ++k) {
                            if (lock_ratio_ > 1e-3 &&
                                std::abs(knee.targets[k].ratio - lock_ratio_) > lock_ratio_tol_ * lock_ratio_) continue;
                            // 对准“预测中心”，避免两车之间来回跳
                            double d = cv::norm(knee.targets[k].center2d - lock_pred_);
                            if (d < bestd) { bestd = d; cand = (int)k; }
                        }
                        if (cand >= 0 && bestd < 250.0) {
                            best = cand; locked = true; lock_miss_ = 0;
                        } else {
                            ++lock_miss_;
                            if (lock_miss_ > 7) { lock_left_ = 0; lock_miss_ = 0; }  // 连续 7 帧找不到才解锁
                        }
                    }
                    if (prev_had_target_) {
                        double bestd = 1e18;
                        int cand = -1;
                        for (size_t k = 0; k < knee.targets.size(); ++k) {
                            // [cv] norm：距离/长度 → d
                            double d = cv::norm(knee.targets[k].center2d - prev_center2d_);
                            if (d > lock_gate_px_) continue;   // 相邻帧位移不超过 lock_gate_px_
                            if (prev_ratio_ > 1e-3 && std::abs(knee.targets[k].ratio - prev_ratio_) > lock_ratio_tol_ * prev_ratio_) continue;  // 长宽比相似
                            if (d < bestd) { bestd = d; cand = (int)k; }
                        }
                        if (cand >= 0) { best = cand; locked = true; }
                    }
                    if (!locked) {
                        for (size_t k = 1; k < knee.targets.size(); ++k)
                            if (std::abs(knee.targets[k].avg_tilt) < std::abs(knee.targets[best].avg_tilt))
                                best = static_cast<int>(k);
                    }

                    // ---- 整车中心聚合 ----
                    cv::Point2f vc(0, 0);
                    for (size_t k = 0; k < knee.targets.size(); ++k) vc += knee.targets[k].center2d;
                    vc *= (1.0f / knee.targets.size());

                    // ---- 主目标平滑 + 下坠补偿 + TF ----
                    Target& mt = knee.targets[best];
                    if (lock_left_ > 0) --lock_left_; else lock_left_ = lock_frames_;
                    lock_prev_ = lock_center_;
                    lock_center_ = mt.center2d;
                    lock_pred_ = lock_center_ + (lock_center_ - lock_prev_);  // 匀速预测下一帧
                    lock_ratio_ = mt.ratio;
                    lock_miss_ = 0;
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

                        // [flow] avg_t/avg_yaw：平滑位姿 → TF / Detections
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


                        // ---- 匀速直线模型：用最近两次成功位姿估速度 ----
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

                    knee.best=best;
                    knee.mt=mt; knee.has_mt=true;
                    knee.vc=vc; knee.has_vc=true;
                    drawBlock(knee);
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
                        prev_ratio_ = mt.ratio;
                        prev_is_large_ = mt.is_large;
                        prev_center2d_ = mt.center2d;
                        prev_stamp_ = stamp;
                        prev_aspect_ = (double)std::max(mr.width, mr.height) /
                                       std::max(1, std::min(mr.width, mr.height));
                        double bar_ratio = 1.0;
                        for (size_t bi = 0; bi < knee.bars.size(); ++bi) {
                            cv::Point2f e1 = knee.bars[bi].pts[1] - knee.bars[bi].pts[0];
                            cv::Point2f e2 = knee.bars[bi].pts[2] - knee.bars[bi].pts[1];
                            // [cv] norm：距离/长度 → a
                            double a = cv::norm(e1), b2 = cv::norm(e2);
                            if (a > 1 && b2 > 1)
                                bar_ratio = std::max(bar_ratio,
                                                     std::max(a, b2) / std::min(a, b2));
                        }
                        prev_bar_ratio_ = bar_ratio;
                        prev_grad_ = brightGradient(knee.v_chan, mr);
                        fourBrightness(knee.v_chan, mr, prev_four_);
                    }
                } else {
                    // ===================================================
                    // 纯白掩膜兜底：两条白灯条强行配对
                    // ===================================================
                    bool white_ok = false;
                    // 白场兜底：属于极端场景（近战过曝纯白），默认关闭，_white_enable:=true 开启
                    if (white_enable_) {
                        cv::Mat white_mask;
                        // [cv] inRange：knee.hsv_full 上取阈值 → （0/255 掩膜）
                        cv::inRange(knee.hsv_full,
                                    cv::Scalar(0, 0, white_v_min_),
                                    cv::Scalar(180, white_s_max_, 255),
                                    white_mask);
                        std::vector<std::vector<cv::Point> > wcnts;
                        // [cv] findContours：从 white_mask 提取 → wcnts
                        cv::findContours(white_mask, wcnts, cv::RETR_EXTERNAL,
                                         cv::CHAIN_APPROX_SIMPLE);
                        struct WB { cv::Point2f c; double tilt, long_h, ratio; std::vector<cv::Point> cnt; };
                        std::vector<WB> wb;
                        for (size_t wi = 0; wi < wcnts.size(); ++wi) {
                            // [cv] contourArea：算 cv::contourArea(wcnts[wi]) < white_area_min_ 面积
                            if (cv::contourArea(wcnts[wi]) < white_area_min_) continue;
                            // [cv] minAreaRect：wcnts[wi] → rr（中心/长宽/倾角）；下游：生成 Bar（中心/长宽/角度）
                            cv::RotatedRect rr = cv::minAreaRect(wcnts[wi]);
                            double w = rr.size.width, h = rr.size.height;
                            if (std::min(w, h) <= 0) continue;
                            double asp = std::max(w, h) / std::min(w, h);
                            if (asp < white_aspect_min_ || asp > white_aspect_max_) continue;
                            // [cv] moments：wcnts[wi] → m，算质心
                            cv::Moments m = cv::moments(wcnts[wi]);
                            if (std::abs(m.m00) < 1e-6) continue;
                            cv::Point2f pts[4]; rr.points(pts);
                            cv::Point2f e1 = pts[1] - pts[0], e2 = pts[2] - pts[1];
                            // [cv] norm：距离/长度 → 
                            cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
                            double tilt = std::atan2(lv.x, lv.y);
                            if (tilt > CV_PI / 2) tilt -= CV_PI;
                            if (tilt < -CV_PI / 2) tilt += CV_PI;
                            WB b;
                            b.c = cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00));
                            b.tilt = tilt;
                            // [cv] norm：距离/长度 → 
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
                                // [cv] boundingRect：wb[0].cnt → b0
                                cv::Rect b0 = cv::boundingRect(wb[0].cnt);
                                // [cv] boundingRect：wb[1].cnt → b1
                                cv::Rect b1 = cv::boundingRect(wb[1].cnt);
                                int min_x = std::min(b0.x, b1.x);
                                int min_y = std::min(b0.y, b1.y);
                                int max_x = std::max(b0.x + b0.width, b1.x + b1.width);
                                int max_y = std::max(b0.y + b0.height, b1.y + b1.height);
                                bool is_large = ratio > 1.3;
                                int wid = prev_had_target_ ? prev_id_ : 1;

                                for (int s2 = 0; s2 < 2; ++s2) {
                                    const std::vector<cv::Point>& cc = (s2 == 0) ? wb[0].cnt : wb[1].cnt;
                                    // [cv] minAreaRect：cc → rr2（中心/长宽/倾角）
                                    cv::RotatedRect rr2 = cv::minAreaRect(cc);
                                    cv::Point2f p2[4]; rr2.points(p2);
                                    for (int k = 0; k < 4; ++k)
                                        // [cv] line：画线到 knee.canvas
                                        cv::line(knee.canvas, p2[k], p2[(k + 1) % 4],
                                                 cv::Scalar(0, 255, 255), 2);
                                }
                                // [cv] rectangle：画矩形到 knee.canvas
                                cv::rectangle(knee.canvas, cv::Rect(min_x, min_y,
                                                               max_x - min_x, max_y - min_y),
                                              cv::Scalar(0, 0, 255), 2);
                                // [cv] putText：文字到 knee.canvas
                                cv::putText(knee.canvas,
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
                                prev_grad_ = brightGradient(knee.v_chan,
                                                cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y));
                                fourBrightness(knee.v_chan,
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
                    if (prev_had_target_ && knee.bars.size() <= 2 &&
                        dt_fcs > 0.0 && dt_fcs < 2.0) {
                        float best_dist = 1e9f;
                        cv::Rect best_rect;
                        int best_idx = -1;
                        for (size_t ci = 0; ci < knee.contours.size(); ++ci) {
                            // [cv] contourArea：算 cv::contourArea(knee.contours[ci]) < fcs_min_area_ 面积
                            if (cv::contourArea(knee.contours[ci]) < fcs_min_area_) continue;
                            std::vector<cv::Point> poly;
                            cv::approxPolyDP(knee.contours[ci], poly,
                                             0.02 * cv::arcLength(knee.contours[ci], true), true);
                            if (poly.size() != 4) continue;
                            // [cv] boundingRect：poly → r
                            cv::Rect r = cv::boundingRect(poly);
                            if (r.width < 8 || r.height < 5) continue;
                            double asp = (double)std::max(r.width, r.height) /
                                         std::max(1, std::min(r.width, r.height));
                            bool ok_armor = (asp >= 0.5 * prev_aspect_ && asp <= 1.8 * prev_aspect_);
                            bool ok_bar   = (asp >= 0.6 * prev_bar_ratio_ &&
                                             asp <= 1.6 * prev_bar_ratio_);
                            if (!ok_armor && !ok_bar) continue;
                            cv::Point2f c(r.x + r.width * 0.5f, r.y + r.height * 0.5f);
                            // [cv] norm：距离/长度 → 
                            float d = (float)cv::norm(c - prev_center2d_);
                            if (d > 400.0f) continue;
                            double grad = brightGradient(knee.v_chan, r);
                            if (std::abs(grad - prev_grad_) > 45.0) continue;

                            // 四点亮度一致性：四个光斑核心互相近似，且与上一帧渐变一致
                            double c4[4];
                            fourBrightness(knee.v_chan, r, c4);
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
                            dashedRect(knee.canvas, best_rect, cv::Scalar(0, 255, 255), 2);
                            std::string fcs = "[FCS] LOCK ID:" + std::to_string(prev_id_) +
                                              (prev_is_large_ ? " L" : " S");
                            // [cv] putText：文字到 knee.canvas
                            cv::putText(knee.canvas, fcs,
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
                        // ---- 丢帧匀速外推：最近两帧速度直线外推，维持轨迹 ----
                        bool pred_ok = false;
                        if (prev_had_target_ && has_last_ && has_vel_ &&
                            lost_frames_ < hold_frames_) {
                            double pdt = stamp - v_prev_stamp_;
                            if (pdt > 0.0 && pdt < 0.5) {
                                cv::Vec3d pp = v_prev_pos_ + vel_ * pdt;
                                // [cv] norm：距离/长度 → 
                                if (cv::norm(vel_) <= hold_max_speed_) {
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
                                    // [cv] putText：文字到 knee.canvas
                                    cv::putText(knee.canvas, "PRED", cv::Point(70, 64),
                                                cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                                cv::Scalar(0, 200, 255), 2);
                                    pred_ok = true;
                                }
                            }
                        }
                        ++lost_frames_;
                        ROS_INFO_THROTTLE(0.5, pred_ok ? "[Hold] 匀速外推 %d/%d" : "[Hold] 丢帧 %d/%d",
                                          lost_frames_, hold_frames_);
                        if (lost_frames_ >= std::max(hold_frames_, 5)) {
                            has_last_ = false; has_sent_ = false;
                            prev_had_target_ = false;   // 防止 FCS 用旧特征幽灵锁
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

            outputBlock(knee);

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
    ros::Timer param_timer_;
    boost::shared_ptr<dynamic_reconfigure::Server<final_text::VisionConfig> > dr_server_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 动态参数（22 点之 13）
    double aspect_min_, aspect_max_;
    double min_dist_x_, max_dist_x_, max_angle_diff_;
    double ratio_s_max_, ratio_l_max_, ratio_min_;
    double bar_h_ratio_min_, pair_dy_ratio_, quad_align_min_, quad_diag_tol_;
    double armor_large_w_, armor_large_h_, armor_small_w_, armor_small_h_;
    double pnp_scale_;
    std::string image_topic_, info_topic_, debug_topic_, detection_topic_, tf_child_;
    int sub_queue_, pub_queue_, min_area_;
    bool plane_filter_;
    double plane_dist_ratio_;
    int    hold_frames_;
    double hold_max_speed_;
    bool   has_vsample_ = false, has_vel_ = false;
    cv::Vec3d v_prev_pos_{0, 0, 0}, vel_{0, 0, 0};
    double v_prev_stamp_ = 0.0;
    double fcs_min_area_, dedup_px_, teleport_margin_;
    double max_z_, deadzone_px_, uniqueness_dt_, max_speed_;
    int morph_size_;
    int hsv_sv_min_;
    int red_h_min_, red_h_max_;
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
    // [flow] last_pos_/vel_/v_prev_pos_：成功帧更新 → 丢帧匀速外推 TF
    cv::Vec3d last_pos_ = cv::Vec3d(0, 0, 0);
    double last_yaw_ = 0.0;
    double omega_deg_s_ = 0.0;
    int lost_frames_ = 0;
    int lock_frames_ = 20, lock_left_ = 0;
    double lock_gate_px_ = 50.0;
    double lock_ratio_tol_ = 0.1;
    cv::Point2f lock_center_ = cv::Point2f(0,0);
    cv::Point2f lock_prev_ = cv::Point2f(0,0);
    cv::Point2f lock_pred_ = cv::Point2f(0,0);
    int lock_miss_ = 0;
    double lock_ratio_ = 0.0;
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
    double      prev_ratio_ = 0.0;   // 上一帧选中板 ratio
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
    ros::init(argc, argv, "robot_vision_node_v3_split");
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
