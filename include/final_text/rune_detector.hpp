#pragma once
// ============================================================
//  rune_detector.hpp
//  功能块：大招（能量机关）
//  高亮轮廓 / 最小二乘圆拟合 / PLL 相位预测
// ============================================================
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <deque>

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
