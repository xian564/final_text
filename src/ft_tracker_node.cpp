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
#include <ft_msgs/Track.h>
#include <opencv2/opencv.hpp>

class TrackerNode {
public:
    TrackerNode() : nh_("~") {
        nh_.param<double>("T", predict_T_, 0.1);
        nh_.param<double>("q_pos", q_pos_, 0.1);
        nh_.param<double>("q_vel", q_vel_, 0.5);
        nh_.param<double>("r_pos", r_pos_, 0.05);
        nh_.param<double>("lost_time_thresh", lost_time_thresh_, 0.5);
        nh_.param<double>("watchdog_period", watchdog_period_, 0.05);   // 20fps

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

        sub_ = nh_.subscribe("/ft/detection", 10, &TrackerNode::detectionCallback, this);
        pub_ = nh_.advertise<ft_msgs::Track>("/ft/track", 10);
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
        if (gap > lost_time_thresh_) {    // 彻底丢失
            is_tracking_ = false;
            ROS_WARN_THROTTLE(1.0, "[tracker] 目标丢失 %.2fs，重置跟踪", gap);
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

    void detectionCallback(const ft_msgs::Detection::ConstPtr& msg) {
        ros::Time stamp = msg->header.stamp;

        if (!is_tracking_) {              // 初始化
            kf_.statePost = (cv::Mat_<float>(6, 1) <<
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                0, 0, 0);
            frame_id_ = msg->header.frame_id;
            last_det_time_ = stamp;
            last_step_time_ = stamp;
            last_pred_time_ = stamp;
            is_tracking_ = true;
            return;
        }

        double dt = (stamp - last_det_time_).toSec();
        if (dt <= 0.0 || dt > 0.5) {      // dt 异常，重新捕获
            is_tracking_ = false;
            return;
        }
        setDt(dt);
        kf_.predict();
        cv::Mat meas = (cv::Mat_<float>(3, 1) <<
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        kf_.correct(meas);

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
        last_step_time_ = now;
        last_pred_time_ = now;
        publishTrack(now, false);
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
        out.position_predict.x = x + vx * predict_T_;
        out.position_predict.y = y + vy * predict_T_;
        out.position_predict.z = z + vz * predict_T_;
        pub_.publish(out);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    cv::KalmanFilter kf_;

    bool is_tracking_ = false;
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
