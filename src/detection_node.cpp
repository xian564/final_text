// ============================================================
//  robot_vision_node.cpp
//  结构：
//    A. VisionNode       —— ROS 节点（订阅/发布/内参缓存）
//    B. ArmorDetector    —— 纯 OpenCV 离线工具（轮廓检测）
//    C. colorSegment     —— 纯 OpenCV 颜色分割工具函数
//    D. main             —— 启动门控 + 主循环
// ============================================================

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <iostream>
#include <string>
#include <vector>

// ---------- 前向声明（定义在文件后面的 C 块） ----------
inline void colorSegment(const cv::Mat& bgr, cv::Mat& res);

// ============================================================
//  A. VisionNode：ROS 节点
// ============================================================
class VisionNode {
public:
    VisionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), it_(nh),
          is_ready_(false), transport_ready_(true)
    {
        // 1. 订阅相机内参
        info_sub_ = nh_.subscribe(
            "/hk_camera/camera_info", 1,
            &VisionNode::cameraInfoCallback, this);

        // 2. 订阅图像（compressed 插件解压）
        try {
            image_sub_ = it_.subscribe(
                "/hk_camera/image_raw", 1,
                &VisionNode::imageCallback, this,
                image_transport::TransportHints(
                    "compressed", ros::TransportHints(), pnh_));
        } catch (const image_transport::TransportLoadException& e) {
            ROS_FATAL("compressed 传输插件加载失败: %s", e.what());
            transport_ready_ = false;
        }

        // 3. 发布 debug 图像  ← 修复：原来误写成 return image_pub;
        image_pub_ = it_.advertise("/ft/debug/image", 1);
    }

    bool isReady() const {
        return is_ready_.load() && transport_ready_;
    }

private:
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

        K_      = K;
        D_      = D;
        width_  = msg->width;
        height_ = msg->height;
        camera_frame_      = msg->header.frame_id;
        distortion_model_  = msg->distortion_model;

        if (!is_ready_.load())
            ROS_INFO("CameraInfo 已缓存: %ux%u", msg->width, msg->height);

        is_ready_.store(true);
    }

    // ---------- 图像回调 ----------
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!is_ready_.load()) return;

        try {
            // 零拷贝转换
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat img = cv_ptr->image;

            // 尺寸校验
            if (img.cols != width_ || img.rows != height_) return;

            // 深拷贝一份画图
            cv::Mat canvas = img.clone();

            // debug 标识：左上角绿框
            cv::rectangle(canvas, cv::Rect(10, 10, 50, 50),
                          cv::Scalar(0, 255, 0), 2);

            // 调用颜色分割工具
            cv::Mat colored;
            colorSegment(canvas, colored);

            // ← 修复：让 colored 真正被使用（灰色底 + 彩色区域覆盖）
            if (!colored.empty()) {
                canvas = colored;   // 只覆盖有颜色的像素
                
                // =======================================================
                // 1. 定义数据结构：灯条 与 最终装甲板
                // =======================================================
                struct ValidBar {
                    cv::RotatedRect rr;
                    cv::Point2f pts[4];
                };
                
                struct ArmorTarget {
                    int id;
                    bool is_large;
                    std::vector<cv::Point2f> corners;
                };
                
                std::vector<ValidBar> valid_bars;
                std::vector<ArmorTarget> targets;

                // =======================================================
                // 2. 找轮廓，按比例过滤，并记录合格的灯条
                // =======================================================
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(colored, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                for (size_t i = 0; i < contours.size(); ++i) {
                    if (cv::contourArea(contours[i]) < 20) continue; 

                    cv::RotatedRect rr = cv::minAreaRect(contours[i]);
                    float w = rr.size.width;
                    float h = rr.size.height;
                    float aspect_ratio = std::max(w, h) / std::min(w, h);

                    if (aspect_ratio > 2.5) {
                        ValidBar vb;
                        vb.rr = rr;
                        rr.points(vb.pts);
                        valid_bars.push_back(vb);
                        
                        // 用黄框标出提取到的合格灯条
                        for (int k = 0; k < 4; ++k) {
                            cv::line(canvas, vb.pts[k], vb.pts[(k+1)%4], cv::Scalar(0, 255, 255), 2);
                        }
                    }
                }

                // =======================================================
                // 3. 排序与就近配对，生成最终记录
                // =======================================================
                std::sort(valid_bars.begin(), valid_bars.end(), [](const ValidBar& a, const ValidBar& b) {
                    return a.rr.center.x < b.rr.center.x;
                });

                int armor_id_counter = 1;

                for (size_t i = 0; i < valid_bars.size(); ++i) {
                    for (size_t j = i + 1; j < valid_bars.size(); ++j) {
                        
                        float distance_x = valid_bars[j].rr.center.x - valid_bars[i].rr.center.x;
                        
                        if (distance_x > 400) break; 

                        if (std::abs(valid_bars[i].rr.center.y - valid_bars[j].rr.center.y) > 50) continue;
                        
                        if (std::abs(valid_bars[i].rr.angle - valid_bars[j].rr.angle) > 15) continue;

                        // --------- 配对成功 ---------
                        ArmorTarget target;
                        target.id = armor_id_counter++;
                        target.is_large = (distance_x > 150); 
                        
                        for(int k=0; k<4; ++k) target.corners.push_back(valid_bars[i].pts[k]);
                        for(int k=0; k<4; ++k) target.corners.push_back(valid_bars[j].pts[k]);
                        
                        targets.push_back(target);

                        // 画红色大框
                        int min_x = std::min(valid_bars[i].rr.center.x, valid_bars[j].rr.center.x);
                        int max_x = std::max(valid_bars[i].rr.center.x, valid_bars[j].rr.center.x);
                        int min_y = std::min(valid_bars[i].rr.center.y, valid_bars[j].rr.center.y);
                        int max_y = std::max(valid_bars[i].rr.center.y, valid_bars[j].rr.center.y);
                        
                        cv::Rect armor_rect(min_x - 20, min_y - 30, (max_x - min_x) + 40, (max_y - min_y) + 60);
                        cv::rectangle(canvas, armor_rect, cv::Scalar(0, 0, 255), 2); 
                        
                        // 打上编号和大小标记
                        std::string info = "ID:" + std::to_string(target.id) + (target.is_large ? " L" : " S");
                        cv::putText(canvas, info, cv::Point(armor_rect.x, armor_rect.y - 10), 
                                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

                        break; 
                    }
                }
                // =======================================================
            }

            // 打包发送
            cv_bridge::CvImage out_img;
            out_img.header.stamp     = msg->header.stamp;
            out_img.header.frame_id  = msg->header.frame_id;
            out_img.encoding         = "bgr8";
            out_img.image            = canvas;
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

    ros::Subscriber             info_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher  image_pub_;

    std::atomic<bool> is_ready_;
    bool              transport_ready_;

    cv::Mat K_, D_;
    int     width_  = 0;
    int     height_ = 0;

    std::string camera_frame_;
    std::string distortion_model_;
};

// ============================================================
//  B. ArmorDetector：纯 OpenCV 离线工具（轮廓检测）
// ============================================================
struct ArmorDetector {
    cv::Mat img;
    cv::Mat gray;
    cv::Mat thresh;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i>              hierarchy;
    std::vector<cv::Point>              cnt;
    int  biggestIdx = -1;
    bool ok         = false;
};

inline ArmorDetector loadContours(
        const std::string& path,
        int  retrieval = cv::RETR_EXTERNAL,
        int  approx    = cv::CHAIN_APPROX_SIMPLE,
        bool useOtsu   = true,
        int  threshVal = 127)
{
    ArmorDetector R;

    // ① 读图
    R.img = cv::imread(path);
    if (R.img.empty()) {
        std::cerr << "读图失败: " << path << "\n";
        return R;
    }

    // ② 灰度
    cv::cvtColor(R.img, R.gray, cv::COLOR_BGR2GRAY);

    // ③ 二值化
    if (useOtsu)
        cv::threshold(R.gray, R.thresh, 0, 255,
                      cv::THRESH_BINARY | cv::THRESH_OTSU);
    else
        cv::threshold(R.gray, R.thresh, threshVal, 255,
                      cv::THRESH_BINARY);

    // ④ 找轮廓
    cv::findContours(R.thresh, R.contours, R.hierarchy,
                     retrieval, approx);
    if (R.contours.empty()) {
        std::cerr << "无轮廓\n";
        return R;
    }

    // ⑤ 取最大轮廓
    double maxArea = 0;
    for (size_t i = 0; i < R.contours.size(); ++i) {
        double a = cv::contourArea(R.contours[i]);
        if (a > maxArea) { maxArea = a; R.biggestIdx = static_cast<int>(i); }
    }
    R.cnt = R.contours[R.biggestIdx];
    R.ok  = true;
    return R;
}

// ============================================================
//  C. colorSegment：HSV 红/蓝/绿分割 + 去噪 + 取彩色区域
// ============================================================
inline void colorSegment(const cv::Mat& bgr, cv::Mat& res) {
    if (bgr.empty()) { res = cv::Mat(); return; }

    cv::Mat hsv;
    cv::Mat mask_r1, mask_r2, mask_r, mask_b, mask_g, mask_all;

    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // 红色（H 跨 0/179 分界，两段）
    cv::inRange(hsv, cv::Scalar(0,   100, 100), cv::Scalar(10,  255, 255), mask_r1);
    cv::inRange(hsv, cv::Scalar(170, 100, 100), cv::Scalar(179, 255, 255), mask_r2);
    cv::bitwise_or(mask_r1, mask_r2, mask_r);

    // 蓝色
    cv::inRange(hsv, cv::Scalar(110, 50, 50), cv::Scalar(130, 255, 255), mask_b);

    // 绿色
    cv::inRange(hsv, cv::Scalar(50, 100, 100), cv::Scalar(70, 255, 255), mask_g);

    // 合并
    cv::bitwise_or(mask_r, mask_b, mask_all);
    cv::bitwise_or(mask_all, mask_g, mask_all);

    // 去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask_all, mask_all, cv::MORPH_OPEN, kernel);

    // 保留原图彩色区域
    cv::bitwise_and(bgr, bgr, res, mask_all);
}

// ============================================================
//  D. main：启动门控 + 主循环
// ============================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_vision_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    VisionNode node(nh, pnh);

    // 30 秒启动超时
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(30.0);
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

    ROS_INFO("VisionNode 就绪，开始处理图像！");
    ros::spin();
    return 0;
}