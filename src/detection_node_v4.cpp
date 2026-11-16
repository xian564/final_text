// ============================================================
//  detection_node_v4.cpp
//  ROS 节点：收图 / 预处理 / 模式调度 / 发布
//  【视觉-火控交接边界】只输出 PoseStamped + TwistStamped
// ============================================================
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int8.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>
#include <opencv2/opencv.hpp>

#include "final_text/vision_preprocess.hpp"
#include "final_text/armor_detector.hpp"
#include "final_text/rune_detector.hpp"

#include <string>
#include <vector>

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
