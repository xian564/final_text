// ============================================================
//  detection_node_v2.cpp  —— 抗眩光 / 残缺 / 抖动 升级版
//
//  与 v1（detection_node.cpp）的关系：
//    * 新建文件，不修改、不覆盖 v1
//    * 复用 v1 的 colorSegment（3x3 OPEN，阈值不变）
//    * 在 imageCallback 中实现 4 项核心优化：
//        1. 用 cv::moments 的加权质心代替 minAreaRect 几何中心
//        2. 按平均倾斜角修正间距，用 spacing_ratio 判定 S / L
//        3. 用两灯条轮廓的 min/max 极值构造横平竖直的 4 角点
//        4. solvePnP + 长度 5 的队列加权平滑 + Y 轴反向 + TF 广播
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
inline void colorSegment(const cv::Mat& bgr, cv::Mat& res);

// ============================================================
//  A. VisionNodeV2：ROS 节点
// ============================================================
class VisionNodeV2 {
public:
    VisionNodeV2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh),
          is_ready_(false), transport_ready_(true),
          tf_broadcaster_()
    {
        info_sub_ = nh_.subscribe(
            "/hk_camera/camera_info", 1,
            &VisionNodeV2::cameraInfoCallback, this);

        try {
            image_sub_ = it_.subscribe(
                "/hk_camera/image_raw", 1,
                &VisionNodeV2::imageCallback, this,
                image_transport::TransportHints(
                    "compressed", ros::TransportHints(), pnh_));
        } catch (const image_transport::TransportLoadException& e) {
            ROS_FATAL("compressed 传输插件加载失败: %s", e.what());
            transport_ready_ = false;
        }

        image_pub_ = it_.advertise("/ft/debug/image_v2", 1);
    }

    bool isReady() const {
        return is_ready_.load() && transport_ready_;
    }

private:
    // ---------- 单根灯条 ----------
    struct Bar {
        cv::RotatedRect rr;
        cv::Point2f pts[4];
        cv::Point2f center;              // 优化1：图像矩质心
        double tilt;                     // 相对竖直方向的倾角（弧度）
        double long_h;                   // 长边像素长度
        std::vector<cv::Point> contour;  // 优化3：保留轮廓用于取极值
    };

    // ---------- 内参回调 ----------
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

        K_ = K;
        D_ = D;
        width_ = msg->width;
        height_ = msg->height;
        camera_frame_ = msg->header.frame_id;
        distortion_model_ = msg->distortion_model;

        if (!is_ready_.load())
            ROS_INFO("CameraInfo 已缓存: %ux%u (frame=%s)",
                     msg->width, msg->height, camera_frame_.c_str());
        is_ready_.store(true);
    }

    // ---------- 优化4：队列加权平滑 ----------
    void pushSmooth(const cv::Vec3d& p, double yaw,
                    cv::Vec3d& out_p, double& out_yaw) {
        const size_t kLen = 5;
        pos_queue_.push_back(p);
        yaw_queue_.push_back(yaw);
        while (pos_queue_.size() > kLen) pos_queue_.pop_front();
        while (yaw_queue_.size() > kLen) yaw_queue_.pop_front();

        double wsum = 0.0, sy = 0.0, cy = 0.0;
        cv::Vec3d acc(0, 0, 0);
        for (size_t k = 0; k < pos_queue_.size(); ++k) {
            double w = static_cast<double>(k + 1);  // 越新权重越大
            acc += pos_queue_[k] * w;
            sy += w * std::sin(yaw_queue_[k]);
            cy += w * std::cos(yaw_queue_[k]);
            wsum += w;
        }
        out_p = acc / wsum;
        out_yaw = std::atan2(sy, cy);
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

            // debug 标识：左上角绿框
            cv::rectangle(canvas, cv::Rect(10, 10, 50, 50),
                          cv::Scalar(0, 255, 0), 2);

            cv::Mat colored;
            colorSegment(canvas, colored);

            if (!colored.empty()) {
                canvas = colored;

                // =======================================================
                // 1. 提取灯条（优化1：矩质心）
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
                    if (std::max(w, h) / std::min(w, h) <= 2.5) continue;

                    Bar bar;
                    bar.rr = rr;
                    rr.points(bar.pts);

                    // 优化1：加权质心（抗眩光尾巴）
                    cv::Moments m = cv::moments(contours[i]);
                    bar.center = rr.center;
                    if (std::abs(m.m00) > 1e-6)
                        bar.center = cv::Point2f(
                            static_cast<float>(m.m10 / m.m00),
                            static_cast<float>(m.m01 / m.m00));

                    // 相对竖直方向的倾角
                    cv::Point2f e1 = bar.pts[1] - bar.pts[0];
                    cv::Point2f e2 = bar.pts[2] - bar.pts[1];
                    cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
                    double tilt = std::atan2(lv.x, lv.y);  // 0 = 竖直
                    if (tilt > CV_PI / 2) tilt -= CV_PI;
                    if (tilt < -CV_PI / 2) tilt += CV_PI;
                    bar.tilt = tilt;
                    bar.long_h = std::max(cv::norm(e1), cv::norm(e2));
                    bar.contour = contours[i];

                    bars.push_back(bar);

                    // 黄框标出灯条
                    for (int k = 0; k < 4; ++k)
                        cv::line(canvas, bar.pts[k], bar.pts[(k + 1) % 4],
                                 cv::Scalar(0, 255, 255), 2);
                }

                std::sort(bars.begin(), bars.end(),
                          [](const Bar& a, const Bar& b) {
                              return a.center.x < b.center.x;
                          });

                // =======================================================
                // 2. 配对（优化2：倾斜补偿 + 比例判 S/L）
                // =======================================================
                int armor_id = 1;

                for (size_t i = 0; i < bars.size(); ++i) {
                    for (size_t j = i + 1; j < bars.size(); ++j) {

                        double dx = bars[j].center.x - bars[i].center.x;
                        if (dx > 400) break;
                        if (dx <= 1) continue;

                        if (std::abs(bars[i].center.y - bars[j].center.y) > 50)
                            continue;

                        double ddeg = std::abs(bars[i].tilt - bars[j].tilt)
                                      * 180.0 / CV_PI;
                        if (ddeg > 15) continue;

                        // 优化2：按平均倾斜角修正像素间距
                        double avg_angle = 0.5 * (bars[i].tilt + bars[j].tilt);
                        double corrected = dx / std::max(0.2, std::abs(std::cos(avg_angle)));
                        double long_h = std::max(bars[i].long_h, bars[j].long_h);
                        if (long_h < 1.0) continue;

                        double ratio = corrected / long_h;
                        if (ratio < 0.5 || ratio > 2.5) continue;
                        bool is_large = (ratio > 1.3);

                        // ===================================================
                        // 3. 极值中点修正（优化3）
                        // ===================================================
                        int min_x = 1 << 30, max_x = -1, min_y = 1 << 30, max_y = -1;
                        for (int s = 0; s < 2; ++s) {
                            const std::vector<cv::Point>& c =
                                (s == 0) ? bars[i].contour : bars[j].contour;
                            for (size_t k = 0; k < c.size(); ++k) {
                                min_x = std::min(min_x, c[k].x);
                                max_x = std::max(max_x, c[k].x);
                                min_y = std::min(min_y, c[k].y);
                                max_y = std::max(max_y, c[k].y);
                            }
                        }
                        std::vector<cv::Point2f> corners;
                        corners.push_back(cv::Point2f(min_x, min_y));  // 左上
                        corners.push_back(cv::Point2f(max_x, min_y));  // 右上
                        corners.push_back(cv::Point2f(max_x, max_y));  // 右下
                        corners.push_back(cv::Point2f(min_x, max_y));  // 左下

                        // ===================================================
                        // 4. solvePnP + 队列平滑 + TF
                        // ===================================================
                        double half_w = is_large ? 0.1125 : 0.0675;  // 225mm / 135mm
                        double half_h = 0.0275;                      // 55mm
                        std::vector<cv::Point3f> obj;
                        obj.push_back(cv::Point3f(-half_w,  half_h, 0));
                        obj.push_back(cv::Point3f( half_w,  half_h, 0));
                        obj.push_back(cv::Point3f( half_w, -half_h, 0));
                        obj.push_back(cv::Point3f(-half_w, -half_h, 0));

                        cv::Mat rvec, tvec;
                        bool pnp_ok = false;
                        // 只对第一个目标做 PnP / TF，避免同一帧多目标向同一 child_frame 重复广播
                        if (armor_id == 1 && K_.rows == 3 && K_.cols == 3) {
                            pnp_ok = cv::solvePnP(obj, corners, K_, D_, rvec, tvec,
                                                  false, cv::SOLVEPNP_IPPE);
                            if (!pnp_ok)
                                pnp_ok = cv::solvePnP(obj, corners, K_, D_, rvec, tvec,
                                                      false, cv::SOLVEPNP_ITERATIVE);
                        }

                        std::string info = "ID:" + std::to_string(armor_id) +
                                           (is_large ? " L" : " S");
                        if (pnp_ok) {
                            cv::Mat R;
                            cv::Rodrigues(rvec, R);

                            // 优化4：PnP 位姿 → ROS 右手系，Y 轴反向
                            cv::Mat S = (cv::Mat_<double>(3, 3) <<
                                         1, 0, 0,
                                         0, -1, 0,
                                         0, 0, 1);
                            cv::Mat R2 = S * R * S;
                            cv::Vec3d t2(tvec.at<double>(0),
                                         -tvec.at<double>(1),
                                          tvec.at<double>(2));
                            double yaw = std::atan2(R2.at<double>(1, 0),
                                                    R2.at<double>(0, 0));

                            cv::Vec3d avg_t;
                            double avg_yaw = 0.0;
                            pushSmooth(t2, yaw, avg_t, avg_yaw);

                            geometry_msgs::TransformStamped ts;
                            ts.header.stamp = msg->header.stamp;
                            ts.header.frame_id = camera_frame_;
                            ts.child_frame_id = "armor_v2";
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

                            ROS_INFO_THROTTLE(1.0,
                                "armor_v2 id=%d %s ratio=%.2f xyz=(%.3f, %.3f, %.3f) yaw=%.1f deg",
                                armor_id, is_large ? "L" : "S", ratio,
                                avg_t[0], avg_t[1], avg_t[2],
                                avg_yaw * 180.0 / CV_PI);
                        }

                        // 画红框 + 蓝色修正角点
                        cv::rectangle(canvas,
                                      cv::Rect(min_x, min_y,
                                               max_x - min_x, max_y - min_y),
                                      cv::Scalar(0, 0, 255), 2);
                        for (int k = 0; k < 4; ++k)
                            cv::circle(canvas, corners[k], 3,
                                       cv::Scalar(255, 0, 0), -1);
                        cv::putText(canvas, info,
                                    cv::Point(min_x, std::max(0, min_y - 10)),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.7,
                                    cv::Scalar(0, 0, 255), 2);

                        ++armor_id;
                        break;
                    }
                }
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
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;

    ros::Subscriber              info_sub_;
    image_transport::Subscriber  image_sub_;
    image_transport::Publisher   image_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::deque<cv::Vec3d> pos_queue_;
    std::deque<double>    yaw_queue_;

    std::atomic<bool> is_ready_;
    bool transport_ready_;

    cv::Mat K_, D_;
    int width_ = 0, height_ = 0;
    std::string camera_frame_;
    std::string distortion_model_;
};

// ============================================================
//  C. colorSegment：与 v1 相同（3x3 OPEN，阈值不变）
// ============================================================
inline void colorSegment(const cv::Mat& bgr, cv::Mat& res) {
    if (bgr.empty()) { res = cv::Mat(); return; }

    cv::Mat hsv;
    cv::Mat mask_r1, mask_r2, mask_r, mask_b, mask_g, mask_all;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0,   100, 100), cv::Scalar(10,  255, 255), mask_r1);
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(179, 255, 255), mask_r2);
    cv::bitwise_or(mask_r1, mask_r2, mask_r);

    cv::inRange(hsv, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), mask_b);
    cv::inRange(hsv, cv::Scalar(50, 100, 100), cv::Scalar(70, 255, 255), mask_g);

    cv::bitwise_or(mask_r, mask_b, mask_all);
    cv::bitwise_or(mask_all, mask_g, mask_all);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask_all, mask_all, cv::MORPH_OPEN, kernel);

    cv::bitwise_and(bgr, bgr, res, mask_all);
}

// ============================================================
//  D. main
// ============================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_vision_node_v2");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    VisionNodeV2 node(nh, pnh);

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

    ROS_INFO("VisionNodeV2 就绪，开始处理图像！");
    ros::spin();
    return 0;
}
