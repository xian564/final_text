// ============================================================
//  ft_tracker_node.cpp
//  订阅 /ft/detection，卡尔曼滤波 + 定时看门狗：
//    - 收到检测：predict + correct，发布 tracking=true
//    - 20fps 周期内没检测：纯预测，发布 tracking=false
//    - 超过 lost_time_thresh：重置跟踪，等待重新捕获
//  发布 /ft/track (ft_msgs/Track)
// ============================================================
#include <ros/ros.h>
#include <ft_msgs/Detection.h>
#include <ft_msgs/Detections.h>
#include <ft_msgs/Track.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <visualization_msgs/Marker.h>

class TrackerNode {
public:
    TrackerNode() : nh_("~") {
        nh_.param<double>("T", predict_T_, 0.1);
        nh_.param<double>("q_pos", q_pos_, 0.1);
        nh_.param<double>("q_vel", q_vel_, 0.5);
        nh_.param<double>("r_pos", r_pos_, 0.05);
        nh_.param<double>("lost_time_thresh", lost_time_thresh_, 0.5);
        nh_.param<double>("watchdog_period", watchdog_period_, 0.05);   // 20fps
        nh_.param<double>("assoc_gate", assoc_gate_, 0.8);               // 关联门限 m
        nh_.param<std::string>("detection_topic", detection_topic_, "/ft/detection");
        nh_.param<std::string>("track_topic", track_topic_, "/ft/track");
        nh_.param<std::string>("marker_topic", marker_topic_, "/ft/debug/marker");
        nh_.param("sub_queue", sub_queue_, 10);
        nh_.param("pub_queue", pub_queue_, 10);
        nh_.param("marker_queue", marker_queue_, 1);

        kf_ = cv::KalmanFilter(6, 3, 0, CV_32F);
        kf_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
            1,0,0, 0,0,0,
            0,1,0, 0,0,0,
            0,0,1, 0,0,0,
            0,0,0, 1,0,0,
            0,0,0, 0,1,0,
            0,0,0, 0,0,1);
        kf_.measurementMatrix = (cv::Mat_<float>(3, 6) <<
            1,0,0, 0,0,0,
            0,1,0, 0,0,0,
            0,0,1, 0,0,0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(0));
        kf_.processNoiseCov.at<float>(0,0) = q_pos_;
        kf_.processNoiseCov.at<float>(1,1) = q_pos_;
        kf_.processNoiseCov.at<float>(2,2) = q_pos_;
        kf_.processNoiseCov.at<float>(3,3) = q_vel_;
        kf_.processNoiseCov.at<float>(4,4) = q_vel_;
        kf_.processNoiseCov.at<float>(5,5) = q_vel_;
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(r_pos_));
        cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

        sub_ = nh_.subscribe(detection_topic_, sub_queue_, &TrackerNode::detectionCallback, this);
        pub_ = nh_.advertise<ft_msgs::Track>(track_topic_, pub_queue_);
        mk_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, marker_queue_);
        param_timer_ = nh_.createTimer(ros::Duration(0.5), &TrackerNode::reloadParams, this);
    }

    // 主循环里每 10ms 调用一次：看门狗
    void spinOnce() {
        ros::spinOnce();
        ros::Time now = ros::Time::now();
        if (!is_tracking_) return;

        double gap = (now - last_det_time_).toSec();
        if (gap < 0.0) {                  // 仿真时间回跳
            is_tracking_ = false;
            return;
        }
        if (gap > lost_time_thresh_) {    // 超过 0.5s：发最后一帧 tracking=false，删除轨迹
            publishTrack(now, false);     // 明确告诉下游“跟踪已失效”
            is_tracking_ = false;         // 删除轨迹，下一帧检测重新捕获
            ROS_WARN_THROTTLE(1.0, "[tracker] 目标丢失 %.2fs，删除轨迹并重新捕获", gap);
            return;
        }
        if (gap > watchdog_period_ &&
            (now - last_pred_time_).toSec() >= watchdog_period_) {
            predictOnly(now);
        }
    }

private:
    void setDt(double dt) {
        kf_.transitionMatrix.at<float>(0, 3) = (float)dt;
        kf_.transitionMatrix.at<float>(1, 4) = (float)dt;
        kf_.transitionMatrix.at<float>(2, 5) = (float)dt;
    }

    void detectionCallback(const ft_msgs::Detections::ConstPtr& msg) {
        if (msg->detections.empty()) return;
        ros::Time stamp = msg->header.stamp;

        // ---- 多检测关联：已有轨迹时选离预测最近且在门限内的 ----
        int pick = 0;
        if (is_tracking_) {
            float px = kf_.statePost.at<float>(0);
            float py = kf_.statePost.at<float>(1);
            float pz = kf_.statePost.at<float>(2);
            double best = assoc_gate_;
            pick = -1;
            for (size_t i = 0; i < msg->detections.size(); ++i) {
                const auto& p = msg->detections[i].pose.position;
                double d = std::sqrt((p.x-px)*(p.x-px) + (p.y-py)*(p.y-py) + (p.z-pz)*(p.z-pz));
                if (d < best) { best = d; pick = (int)i; }
            }
            if (pick < 0) return;      // 门限内没有观测，交给看门狗纯预测
        }
        const ft_msgs::Detection& det = msg->detections[pick];

        if (!is_tracking_) {           // 初始化
            kf_.statePost = (cv::Mat_<float>(6, 1) <<
                det.pose.position.x, det.pose.position.y, det.pose.position.z,
                0, 0, 0);
            frame_id_ = msg->header.frame_id;
            last_det_time_ = stamp;
            last_step_time_ = stamp;
            last_pred_time_ = stamp;
            is_tracking_ = true;
            return;
        }

        double dt = (stamp - last_det_time_).toSec();
        if (dt <= 0.0 || dt > 0.5) {   // dt 异常，重新捕获
            is_tracking_ = false;
            return;
        }
        setDt(dt);
        kf_.predict();
        cv::Mat meas = (cv::Mat_<float>(3, 1) <<
            det.pose.position.x, det.pose.position.y, det.pose.position.z);
        kf_.correct(meas);
        clampVelocity();

        frame_id_ = msg->header.frame_id;
        last_det_time_ = stamp;
        last_step_time_ = stamp;
        last_pred_time_ = stamp;
        publishTrack(stamp, true);
    }

    // 纯预测：不 correct，只 predict + 发布 tracking=false
    void predictOnly(ros::Time now) {
        double dt = (now - last_step_time_).toSec();
        if (dt <= 0.0 || dt > 0.5) dt = watchdog_period_;
        setDt(dt);
        kf_.predict();
        clampVelocity();
        last_step_time_ = now;
        last_pred_time_ = now;
        publishTrack(now, false);
    }

    void clampVelocity() {
        const float vmax = 6.0f;   // 车体物理上限 m/s
        for (int i = 3; i < 6; ++i) {
            float v = kf_.statePost.at<float>(i);
            if (!std::isfinite(v)) v = 0.0f;
            if (v > vmax) v = vmax;
            if (v < -vmax) v = -vmax;
            kf_.statePost.at<float>(i) = v;
        }
    }

    void publishTrack(ros::Time stamp, bool tracking) {
        float x = kf_.statePost.at<float>(0), y = kf_.statePost.at<float>(1), z = kf_.statePost.at<float>(2);
        float vx = kf_.statePost.at<float>(3), vy = kf_.statePost.at<float>(4), vz = kf_.statePost.at<float>(5);

        ft_msgs::Track out;
        out.header.stamp = stamp;
        out.header.frame_id = frame_id_;
        out.id = 1;
        out.tracking = tracking;
        out.position.x = x; out.position.y = y; out.position.z = z;
        out.velocity.x = vx; out.velocity.y = vy; out.velocity.z = vz;
        double dx = vx * predict_T_, dy = vy * predict_T_, dz = vz * predict_T_;
        double dn = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (dn > 0.6) { dx *= 0.6/dn; dy *= 0.6/dn; dz *= 0.6/dn; }  // 预测偏移上限 0.6m
        out.position_predict.x = x + dx;
        out.position_predict.y = y + dy;
        out.position_predict.z = z + dz;
        pub_.publish(out);

        // RViz 预测 marker：绿球=预测点，黄线=实际点->预测点
        visualization_msgs::Marker mk;
        mk.header.stamp = stamp;
        mk.header.frame_id = frame_id_;
        mk.ns = "ft_track";
        mk.id = 0;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = out.position_predict.x;
        mk.pose.position.y = out.position_predict.y;
        mk.pose.position.z = out.position_predict.z;
        mk.pose.orientation.w = 1.0;
        mk.scale.x = mk.scale.y = mk.scale.z = 0.08;
        mk.color.r = tracking ? 0.0f : 1.0f;
        mk.color.g = tracking ? 1.0f : 0.0f;
        mk.color.b = 0.0f;
        mk.color.a = 1.0f;
        mk.lifetime = ros::Duration(0.2);
        mk_pub_.publish(mk);

        visualization_msgs::Marker ln;
        ln.header = mk.header;
        ln.ns = "ft_track";
        ln.id = 1;
        ln.type = visualization_msgs::Marker::LINE_STRIP;
        ln.action = visualization_msgs::Marker::ADD;
        ln.scale.x = 0.012;
        ln.color.r = 1.0f; ln.color.g = 0.8f; ln.color.b = 0.0f; ln.color.a = 1.0f;
        ln.lifetime = ros::Duration(0.2);
        geometry_msgs::Point p0, p1;
        p0.x = out.position.x; p0.y = out.position.y; p0.z = out.position.z;
        p1.x = out.position_predict.x; p1.y = out.position_predict.y; p1.z = out.position_predict.z;
        ln.points.push_back(p0);
        ln.points.push_back(p1);
        mk_pub_.publish(ln);
    }

    void reloadParams(const ros::TimerEvent&) {
        nh_.getParam("T", predict_T_);
        nh_.getParam("q_pos", q_pos_);
        nh_.getParam("q_vel", q_vel_);
        nh_.getParam("r_pos", r_pos_);
        nh_.getParam("lost_time_thresh", lost_time_thresh_);
        nh_.getParam("watchdog_period", watchdog_period_);
        nh_.getParam("assoc_gate", assoc_gate_);
        for (int i = 0; i < 3; ++i) {
            kf_.processNoiseCov.at<float>(i, i) = (float)q_pos_;
            kf_.processNoiseCov.at<float>(i + 3, i + 3) = (float)q_vel_;
            kf_.measurementNoiseCov.at<float>(i, i) = (float)r_pos_;
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_, mk_pub_;
    ros::Timer param_timer_;
    cv::KalmanFilter kf_;

    bool is_tracking_ = false;
    double assoc_gate_ = 0.8;
    std::string detection_topic_, track_topic_, marker_topic_;
    int sub_queue_ = 10, pub_queue_ = 10, marker_queue_ = 1;
    std::string frame_id_ = "hk_camera_ca";
    ros::Time last_det_time_, last_step_time_, last_pred_time_;

    double predict_T_;
    double q_pos_, q_vel_, r_pos_;
    double lost_time_thresh_, watchdog_period_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ft_tracker_node");
    TrackerNode tracker;
    ros::Rate rate(100);          // 10ms 主循环
    while (ros::ok()) {
        tracker.spinOnce();
        rate.sleep();
    }
    return 0;
}
