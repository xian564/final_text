// ============================================================
//  detect_node_nn.cpp
//  神经网络检测节点：/hk_camera/image_raw/compressed -> /ft/detection
//  使用 IDetector 接口 + OnnxDetector；相邻帧优先锁定同一块装甲板
// ============================================================
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PoseStamped.h>
#include <ft_msgs/Detections.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include "final_text/detector_interface.hpp"

class NnNode {
 public:
  NnNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), it_(nh), has_prev_(false) {
    std::string model;
    pnh_.param<std::string>("model_path", model, "");
    pnh_.param("color_thres", color_thres_, 0.5);
    pnh_.param("number_thres", number_thres_, 0.5);
    pnh_.param("nms_thres", nms_thres_, 0.45);
    pnh_.param("pnp_scale", pnp_scale_, 1.0);
    pnh_.param("armor_large_w", armor_large_w_, 0.230);
    pnh_.param("armor_large_h", armor_large_h_, 0.060);
    pnh_.param("armor_small_w", armor_small_w_, 0.135);
    pnh_.param("armor_small_h", armor_small_h_, 0.060);
    pnh_.param("lock_gate_px", lock_gate_px_, 300.0);

    detector_.reset(new OnnxDetector(model, color_thres_, number_thres_, nms_thres_));
    if (!model.empty() && !detector_->ready())
      ROS_WARN("ONNX 模型加载失败: %s", model.c_str());

    info_sub_ = nh_.subscribe("/hk_camera/camera_info", 1,
                              &NnNode::infoCb, this);
    image_sub_ = it_.subscribe("/hk_camera/image_raw", 1,
                               &NnNode::imageCb, this,
                               image_transport::TransportHints(
                                   "compressed", ros::TransportHints(), pnh_));
    image_pub_ = it_.advertise("/ft/debug/image_nn", 1);
    det_pub_ = nh_.advertise<ft_msgs::Detections>("/ft/detection_nn", 1);
  }

 private:
  void infoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (!msg->width || !msg->height) return;
    K_ = cv::Mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) K_.at<double>(r, c) = msg->K[r * 3 + c];
    D_ = cv::Mat(1, (int)msg->D.size(), CV_64F);
    for (size_t i = 0; i < msg->D.size(); ++i) D_.at<double>(0, (int)i) = msg->D[i];
    frame_ = msg->header.frame_id;
    ready_ = true;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    if (!ready_) return;
    cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, "bgr8");
    const cv::Mat& bgr = cvp->image;
    cv::Mat canvas = bgr.clone();

    std::vector<ArmorObservation> armors;
    detector_->detect(bgr, armors);

    // 相邻帧锁定：选离上一帧选中目标最近的一块
    int pick = -1;
    if (!armors.empty()) {
      pick = 0;
      if (has_prev_) {
        double best = 1e18;
        for (size_t k = 0; k < armors.size(); ++k) {
          cv::Point2f c((armors[k].corners[0].x + armors[k].corners[2].x) * 0.5f,
                        (armors[k].corners[0].y + armors[k].corners[2].y) * 0.5f);
          double d = cv::norm(c - prev_center_);
          if (d < best) { best = d; pick = (int)k; }
        }
        if (best > lock_gate_px_) { pick = 0; has_prev_ = false; }
      }
    }

    ft_msgs::Detections arr;
    arr.header = msg->header;
    for (size_t k = 0; k < armors.size(); ++k) {
      const ArmorObservation& a = armors[k];
      cv::Point2f c((a.corners[0].x + a.corners[2].x) * 0.5f,
                    (a.corners[0].y + a.corners[2].y) * 0.5f);
      bool selected = ((int)k == pick);

      int minx = 1e9, maxx = -1, miny = 1e9, maxy = -1;
      for (int q = 0; q < 4; ++q) {
        minx = std::min(minx, (int)a.corners[q].x);
        maxx = std::max(maxx, (int)a.corners[q].x);
        miny = std::min(miny, (int)a.corners[q].y);
        maxy = std::max(maxy, (int)a.corners[q].y);
      }
      cv::Scalar color = a.color == 1 ? cv::Scalar(0, 0, 255)
                     : a.color == 0 ? cv::Scalar(255, 0, 0)
                                    : cv::Scalar(128, 128, 128);
      std::vector<cv::Point> quad;
      for (int q = 0; q < 4; ++q) quad.push_back(cv::Point((int)a.corners[q].x, (int)a.corners[q].y));
      cv::polylines(canvas, quad, true, color, selected ? 3 : 1);
      cv::putText(canvas, "ID:" + std::to_string(a.number),
                  cv::Point(minx, std::max(0, miny - 6)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

      if (!selected || K_.empty()) continue;

      double w = 0.5 * (cv::norm(a.corners[0] - a.corners[1]) +
                        cv::norm(a.corners[3] - a.corners[2]));
      double h = 0.5 * (cv::norm(a.corners[0] - a.corners[3]) +
                        cv::norm(a.corners[1] - a.corners[2]));
      bool is_large = (h > 1e-3 && w / h > 3.0);
      double Wb = (is_large ? armor_large_w_ : armor_small_w_) * 0.5 * pnp_scale_;
      double Hb = (is_large ? armor_large_h_ : armor_small_h_) * 0.5 * pnp_scale_;
      std::vector<cv::Point3f> obj = {
          {- (float)Wb, - (float)Hb, 0}, {(float)Wb, - (float)Hb, 0},
          {(float)Wb, (float)Hb, 0}, {- (float)Wb, (float)Hb, 0}};
      std::vector<cv::Point2f> img(a.corners, a.corners + 4);

      cv::Mat rvec, tvec;
      bool ok = cv::solvePnP(obj, img, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
      if (!ok) ok = cv::solvePnP(obj, img, K_, D_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
      if (!ok || tvec.at<double>(2) <= 0.3 || tvec.at<double>(2) > 15.0) continue;

      ft_msgs::Detection d;
      d.header = msg->header;
      d.id = (int)k + 1;
      d.number = a.number;
      d.is_red = (a.color == 1);
      d.is_large_armor = is_large;
      d.pose.position.x = tvec.at<double>(0);
      d.pose.position.y = tvec.at<double>(1);
      d.pose.position.z = tvec.at<double>(2);
      tf2::Quaternion q;
      q.setRPY(0, 0, 0);
      cv::Mat R;
      cv::Rodrigues(rvec, R);
      tf2::Matrix3x3 m3(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
      m3.getRotation(q);
      d.pose.orientation.x = q.x(); d.pose.orientation.y = q.y();
      d.pose.orientation.z = q.z(); d.pose.orientation.w = q.w();
      arr.detections.push_back(d);

      geometry_msgs::TransformStamped ts;
      ts.header = msg->header;
      ts.header.frame_id = frame_;
      ts.child_frame_id = "armor_nn";
      ts.transform.translation.x = d.pose.position.x;
      ts.transform.translation.y = d.pose.position.y;
      ts.transform.translation.z = d.pose.position.z;
      ts.transform.rotation = d.pose.orientation;
      tf_broadcaster_.sendTransform(ts);

      if (selected) { prev_center_ = c; has_prev_ = true; }
    }

    if (!arr.detections.empty()) det_pub_.publish(arr);
    cv_bridge::CvImage out;
    out.header = msg->header;
    out.encoding = "bgr8";
    out.image = canvas;
    image_pub_.publish(out.toImageMsg());
  }

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport it_;
  ros::Subscriber info_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher det_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  std::unique_ptr<IDetector> detector_;

  cv::Mat K_, D_;
  std::string frame_;
  bool ready_ = false, has_prev_ = false;
  cv::Point2f prev_center_;
  double color_thres_, number_thres_, nms_thres_, pnp_scale_;
  double armor_large_w_, armor_large_h_, armor_small_w_, armor_small_h_;
  double lock_gate_px_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_vision_node_nn");
  ros::NodeHandle nh, pnh("~");
  NnNode node(nh, pnh);
  ros::spin();
  return 0;
}
