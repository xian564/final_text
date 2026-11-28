// 旋转目标建模 + 装甲板选择（进阶）
// - 多帧装甲中心圆拟合 -> 车体中心/半径
// - 相位 + 角速度（0.5s 窗口），正=左转、负=右转
// - 位置预测 = 车体直线外推 + 旋转相位外推
// - 装甲选择：首次最靠近相机轴；之后选离预测最近且在门限内
// - 断帧极限推算，超过 hold 断开 ID；遮挡 valid=false
#include <ros/ros.h>
#include <ft_msgs/Detections.h>
#include <ft_msgs/Track.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <cmath>

class RotTracker {
 public:
  RotTracker(ros::NodeHandle& nh) : nh_(nh), kf_(6, 3, 0, CV_32F) {
    nh_.param<double>("T", T_, 0.1);
    nh_.param<double>("fit_window", fit_window_, 2.0);
    nh_.param<double>("phase_window", phase_window_, 0.5);
    nh_.param<double>("assoc_gate", gate_, 1.0);
    nh_.param<double>("lost_time_thresh", lost_thresh_, 0.5);
    nh_.param<double>("default_radius", R_, 0.35);
    nh_.param<double>("omega_min", omega_min_, 0.2);
    nh_.param<double>("fit_res_max", fit_res_max_, 0.05);
    nh_.param<double>("predict_cap_m", predict_cap_m_, 0.6);
    nh_.param<double>("q_pos", q_pos_, 0.1);
    nh_.param<double>("q_vel", q_vel_, 0.5);
    nh_.param<double>("r_pos", r_pos_, 0.05);
    nh_.param<std::string>("detection_topic", det_topic_, "/ft/detection");
    nh_.param<std::string>("track_topic", track_topic_, "/ft/track_rot");
    nh_.param<std::string>("marker_topic", marker_topic_, "/ft/debug/marker_rot");
    nh_.param<std::string>("body_id_topic", body_id_topic_, "/ft/body_id");

    kf_.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);
    kf_.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
    for (int i = 0; i < 3; ++i) kf_.measurementMatrix.at<float>(i, i) = 1.0f;
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(0));
    for (int i = 0; i < 3; ++i) {
        kf_.processNoiseCov.at<float>(i, i) = (float)q_pos_;
        kf_.processNoiseCov.at<float>(i + 3, i + 3) = (float)q_vel_;
    }
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(r_pos_));
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

    sub_ = nh_.subscribe(det_topic_, 1, &RotTracker::detCb, this);
    id_sub_ = nh_.subscribe(body_id_topic_, 1, &RotTracker::idCb, this);
    pub_ = nh_.advertise<ft_msgs::Track>(track_topic_, 1);
    mk_pub_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 1);
  }

 private:
  void idCb(const std_msgs::Int32::ConstPtr& m) { body_id_ = m->data; }

  static double quatYaw(const geometry_msgs::Quaternion& q) {
    double s = 2.0 * (q.w * q.z + q.x * q.y);
    double c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(s, c);
  }

  // 圆拟合（Kasa）：x,z 平面
  bool fitCircle(const std::deque<std::pair<double, cv::Point2d>>& h,
                 cv::Point2d& c, double& R, double& rms) {
    if (h.size() < 8) return false;
    double Sx=0,Sz=0,Sxx=0,Szz=0,Sxz=0,Sx3=0,Sz3=0,Sxzz=0,Sxxz=0;
    for (auto& p : h) {
      double x=p.second.x, z=p.second.y;
      Sx+=x; Sz+=z; Sxx+=x*x; Szz+=z*z; Sxz+=x*z;
      Sx3+=x*x*x; Sz3+=z*z*z; Sxzz+=x*z*z; Sxxz+=x*x*z;
    }
    int n = (int)h.size();
    cv::Mat A = (cv::Mat_<double>(3,3) <<
      2*Sx, 2*Sz, n,
      2*Sxx, 2*Sxz, Sx,
      2*Sxz, 2*Szz, Sz);
    cv::Mat b = (cv::Mat_<double>(3,1) << Sxx+Szz, Sx3+Sxzz, Sxxz+Sz3);
    cv::Mat s; if (!cv::solve(A, b, s, cv::DECOMP_SVD)) return false;
    double a=s.at<double>(0), bb=s.at<double>(1), cc=s.at<double>(2);
    double r2 = cc + a*a + bb*bb;
    if (r2 <= 0) return false;
    c = cv::Point2d(a, bb); R = std::sqrt(r2);
    double e = 0;
    for (auto& p : h) e += std::pow(cv::norm(p.second - c) - R, 2);
    rms = std::sqrt(e / (double)h.size());
    return R > 0.1 && R < 2.0;
  }

  void detCb(const ft_msgs::Detections::ConstPtr& msg) {
    ros::Time stamp = msg->header.stamp;
    frame_id_ = msg->header.frame_id;
    if (msg->detections.empty()) { onLost(stamp); return; }

    // 观测选择
    int pick = -1;
    if (has_pred_ && rot_mode_) {
      double best = gate_;
      for (size_t i=0;i<msg->detections.size();++i) {
        auto& p = msg->detections[i].pose.position;
        double d = std::sqrt(std::pow(p.x-pred_.x,2)+std::pow(p.y-pred_.y,2)+std::pow(p.z-pred_.z,2));
        if (d < best) { best = d; pick = (int)i; }
      }
      if (pick < 0) { onLost(stamp); return; }   // 门限内没有，交给预测
    } else if (tracking_) {
      // 直线模式：像匀速跟踪器一样，用当前状态关联
      double best = gate_, cx = kf_.statePost.at<float>(0),
             cy = kf_.statePost.at<float>(1), cz = kf_.statePost.at<float>(2);
      for (size_t i=0;i<msg->detections.size();++i) {
        auto& p = msg->detections[i].pose.position;
        double d = std::sqrt((p.x-cx)*(p.x-cx)+(p.y-cy)*(p.y-cy)+(p.z-cz)*(p.z-cz));
        if (d < best) { best = d; pick = (int)i; }
      }
      if (pick < 0) { onLost(stamp); return; }
    } else {
      // 首次：最靠近相机轴
      double best = 1e9;
      for (size_t i=0;i<msg->detections.size();++i) {
        auto& p = msg->detections[i].pose.position;
        double d = std::hypot(p.x, p.y);
        if (d < best) { best = d; pick = (int)i; }
      }
    }
    auto& det = msg->detections[pick];

    // 用检测朝向估计角速度（比相位差分更直接）
    double yaw_now = quatYaw(det.pose.orientation);
    if (!has_yaw_) { yaw_ = yaw_now; has_yaw_ = true; }
    else {
      double dy = std::atan2(std::sin(yaw_now - yaw_), std::cos(yaw_now - yaw_));
      double dty = (stamp - yaw_t_).toSec();
      if (dty > 0.02) { yaw_rate_ = 0.8 * yaw_rate_ + 0.2 * (dy / dty); yaw_ = yaw_now; yaw_t_ = stamp; }
    }

    if (!tracking_) {
      kf_.statePost = (cv::Mat_<float>(6,1) << det.pose.position.x, det.pose.position.y,
                       det.pose.position.z, 0,0,0);
      tracking_ = true;
      last_t_ = stamp;
      has_phase_ = false;
    } else {
      double dt = (stamp - last_t_).toSec();
      if (dt <= 0 || dt > lost_thresh_) { tracking_ = false; onLost(stamp); return; }
      setDt(dt); kf_.predict();
      cv::Mat meas = (cv::Mat_<float>(3,1) << det.pose.position.x,
                      det.pose.position.y, det.pose.position.z);
      kf_.correct(meas);
      {   // 速度限幅，避免异常观测把预测拉飞
        float vx=kf_.statePost.at<float>(3), vy=kf_.statePost.at<float>(4), vz=kf_.statePost.at<float>(5);
        float n=std::sqrt(vx*vx+vy*vy+vz*vz);
        if (n > 6.0f) { float k=6.0f/n;
          kf_.statePost.at<float>(3)=vx*k; kf_.statePost.at<float>(4)=vy*k; kf_.statePost.at<float>(5)=vz*k; }
      }
      last_t_ = stamp;
    }
    float x=kf_.statePost.at<float>(0), y=kf_.statePost.at<float>(1), z=kf_.statePost.at<float>(2);

    // 历史与圆拟合
    hist_.push_back({stamp.toSec(), cv::Point2d(x, z)});
    while (!hist_.empty() && stamp.toSec() - hist_.front().first > fit_window_) hist_.pop_front();
    cv::Point2d c; double R;
    if (fitCircle(hist_, c, R, fit_res_)) { center_ = c; R_ = R; has_center_ = true; }

    // 相位/角速度
    double phase = has_center_ ? std::atan2(z - center_.y, x - center_.x)
                               : std::atan2(z, x);
    if (!has_phase_) { phase_ = phase; has_phase_ = true; }
    else {
      double dph = std::atan2(std::sin(phase - phase_), std::cos(phase - phase_));
      double dt = (stamp - phase_t_).toSec();
      if (dt > 0.05) {
        double w = dph / dt;
        omega_ = 0.8 * omega_ + 0.2 * w;
        phase_ = phase; phase_t_ = stamp;
      }
    }

    // 预测：转速足够才用圆周，否则退回匀速（避免直线段引入噪声）
    double vx = kf_.statePost.at<float>(3), vy = kf_.statePost.at<float>(4), vz = kf_.statePost.at<float>(5);
    double w = (std::fabs(yaw_rate_) > omega_min_) ? yaw_rate_ : omega_;
    bool fast = (std::fabs(w) >= omega_min_) && (fit_res_ < fit_res_max_) && has_center_;
    rot_cnt_ = std::max(0, std::min(10, rot_cnt_ + (fast ? 1 : -3)));
    rot_mode_ = (rot_cnt_ >= 3);          // 连续 3 帧可信才进旋转
    bool rotating = rot_mode_;
    double lin_x = x + vx * T_, lin_z = z + vz * T_;
    if (rotating) {
        double cx = center_.x + vx * T_;
        double cz = center_.y + vz * T_;
        double dx = x - center_.x, dz = z - center_.y;
        double ca = std::cos(w * T_), sa = std::sin(w * T_);
        double rx = cx + dx * ca - dz * sa;
        double rz = cz + dx * sa + dz * ca;
        double a = 1.0 - fit_res_ / std::max(1e-3, fit_res_max_);
        a = std::max(0.0, std::min(1.0, a));
        pred_.x = lin_x + a * (rx - lin_x);
        pred_.z = lin_z + a * (rz - lin_z);
    } else {
        pred_.x = lin_x;
        pred_.z = lin_z;
    }
    pred_.y = y + vy * T_;
    // 预测偏移限幅，避免速度尖峰把预测拉远
    {
      double dx = pred_.x - x, dy2 = pred_.y - y, dz = pred_.z - z;
      double dn = std::sqrt(dx*dx + dy2*dy2 + dz*dz);
      if (dn > predict_cap_m_) {
        double k = predict_cap_m_ / dn;
        pred_.x = x + dx*k; pred_.y = y + dy2*k; pred_.z = z + dz*k;
      }
    }
    has_pred_ = true;
    cur_.x = x; cur_.y = y; cur_.z = z;

    publish(stamp, true);
    publishMarkers(stamp);
  }

  void onLost(ros::Time stamp) {
    if (!tracking_) return;
    double gap = (stamp - last_t_).toSec();
    if (gap > lost_thresh_) {
      tracking_ = false; has_pred_ = false; has_phase_ = false;
      publish(stamp, false);
      return;
    }
    // 极限推算：转速足够用圆周，否则直线
    double vx = kf_.statePost.at<float>(3), vy = kf_.statePost.at<float>(4), vz = kf_.statePost.at<float>(5);
    double w = (std::fabs(yaw_rate_) > omega_min_) ? yaw_rate_ : omega_;
    if (has_center_ && rot_mode_ && fit_res_ < fit_res_max_) {
        double cx = center_.x + vx * gap;
        double cz = center_.y + vz * gap;
        double dx = cur_.x - center_.x, dz = cur_.z - center_.y;
        double ca = std::cos(w * gap), sa = std::sin(w * gap);
        pred_.x = cx + dx * ca - dz * sa;
        pred_.z = cz + dx * sa + dz * ca;
    } else {
        pred_.x = cur_.x + vx * gap;
        pred_.z = cur_.z + vz * gap;
    }
    pred_.y = cur_.y + vy * gap;
    publish(stamp, false);
    publishMarkers(stamp);
  }

  void setDt(double dt) {
    kf_.transitionMatrix.at<float>(0,3) = dt;
    kf_.transitionMatrix.at<float>(1,4) = dt;
    kf_.transitionMatrix.at<float>(2,5) = dt;
  }

  void publish(ros::Time stamp, bool tracking) {
    ft_msgs::Track out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id_;
    out.id = body_id_ > 0 ? body_id_ : 1;
    out.tracking = tracking;
    out.valid = tracking;
    out.position.x = cur_.x; out.position.y = cur_.y; out.position.z = cur_.z;
    out.velocity.x = kf_.statePost.at<float>(3);
    out.velocity.y = kf_.statePost.at<float>(4);
    out.velocity.z = kf_.statePost.at<float>(5);
    out.position_predict.x = pred_.x;
    out.position_predict.y = pred_.y;
    out.position_predict.z = pred_.z;
    if (has_center_) {
      out.body_center.x = center_.x;
      out.body_center.y = cur_.y;
      out.body_center.z = center_.y;
    }
    pub_.publish(out);
  }

  void publishMarkers(ros::Time stamp) {
    auto mk = [&](int id, const cv::Point3d& p, float r, float g, float b, const std::string& ns) {
      visualization_msgs::Marker m;
      m.header.stamp = stamp; m.header.frame_id = frame_id_;
      m.ns = ns; m.id = id; m.type = visualization_msgs::Marker::SPHERE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = p.x; m.pose.position.y = p.y; m.pose.position.z = p.z;
      m.scale.x = m.scale.y = m.scale.z = 0.08;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
      mk_pub_.publish(m);
    };
    if (has_center_) mk(0, cv::Point3d(center_.x, cur_.y, center_.y), 0,0,1, "body");
    mk(1, cur_, 0,1,0, "armor");
    if (has_pred_) mk(2, pred_, 1,0,0, "pred");
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_, id_sub_;
  ros::Publisher pub_, mk_pub_;
  cv::KalmanFilter kf_;
  std::deque<std::pair<double, cv::Point2d>> hist_;
  cv::Point2d center_{0,0};
  double R_ = 0.35, T_ = 0.1, fit_window_ = 2.0, phase_window_ = 0.5;
  double gate_ = 1.0, lost_thresh_ = 0.5, q_pos_=0.1, q_vel_=0.5, r_pos_=0.05;
  double omega_min_ = 0.2;
  int rot_cnt_ = 0;
  double fit_res_ = 1e9, fit_res_max_ = 0.05, predict_cap_m_ = 0.6;
  std::string det_topic_, track_topic_, marker_topic_, body_id_topic_;
  bool tracking_ = false, has_center_ = false, has_phase_ = false, has_pred_ = false;
  bool rot_mode_ = false;
  double phase_ = 0, omega_ = 0;
  double yaw_ = 0, yaw_rate_ = 0;
  bool has_yaw_ = false;
  ros::Time yaw_t_;
  ros::Time last_t_, phase_t_;
  int body_id_ = 0;
  cv::Point3d cur_{0,0,0}, pred_{0,0,0};
  std::string frame_id_ = "hk_camera_ca";
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ft_rotation_tracker");
  ros::NodeHandle nh, pnh("~");
  RotTracker node(nh);
  ros::spin();
  return 0;
}
