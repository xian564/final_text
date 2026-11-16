#pragma once
// ============================================================
//  armor_detector.hpp
//  功能块：平A（普通装甲板）
//  动态 ROI / 矩质心 / 透视比例 / 极值厂字框 / PnP / Alpha-Beta
// ============================================================
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

class NormalArmorDetector {
public:
    struct Bar {
        cv::RotatedRect rr;
        cv::Point2f pts[4];
        cv::Point2f c;
        double tilt, long_h, v;
        std::vector<cv::Point> cnt;
    };
    struct Target {
        int id;
        bool is_large;
        double ratio;
        std::vector<cv::Point2f> corners;
        cv::Point2f c2;
        bool has_pose = false;
        cv::Vec3d t;
        double yaw = 0, z = -1;
    };

    void load(ros::NodeHandle& pnh) {
        pnh.param("aspect_min", aspect_min_, 1.3);
        pnh.param("aspect_max", aspect_max_, 6.0);
        pnh.param("min_dist_x", min_dist_x_, 15.0);
        pnh.param("max_dist_x", max_dist_x_, 500.0);
        pnh.param("max_angle", max_angle_, 10.0);
        pnh.param("ratio_s_max", ratio_s_max_, 1.3);
        pnh.param("ratio_l_max", ratio_l_max_, 2.5);
        pnh.param("bar_v_min", bar_v_min_, 120);
        pnh.param("roi_size", roi_size_, 300);
        pnh.param("alpha", alpha_, 0.6);
        pnh.param("beta", beta_, 0.2);
    }

    void detect(const cv::Mat& bgr, const cv::Mat& mask, const cv::Mat& vchan,
                const cv::Mat& K, const cv::Mat& D, cv::Mat& canvas,
                std::vector<Target>& targets, const ros::Time& stamp) {
        targets.clear();
        published_ = false;
        cv::Rect roi(0, 0, bgr.cols, bgr.rows);
        if (has_last_) {
            int half = roi_size_ / 2;
            roi = cv::Rect(std::max(0, (int)last_c_.x - half),
                           std::max(0, (int)last_c_.y - half),
                           std::min(roi_size_, bgr.cols), std::min(roi_size_, bgr.rows));
            roi &= cv::Rect(0, 0, bgr.cols, bgr.rows);
            if (roi.width < 80 || roi.height < 60) roi = cv::Rect(0, 0, bgr.cols, bgr.rows);
        }

        cv::Mat sub = mask(roi);
        std::vector<std::vector<cv::Point> > cnts;
        cv::findContours(sub, cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<Bar> bars;
        for (size_t i = 0; i < cnts.size(); ++i) {
            if (cv::contourArea(cnts[i]) < 20) continue;
            cv::RotatedRect rr = cv::minAreaRect(cnts[i]);
            double w = rr.size.width, h = rr.size.height;
            if (std::min(w, h) <= 0) continue;
            double asp = std::max(w, h) / std::min(w, h);
            if (asp < aspect_min_ || asp > aspect_max_) continue;

            cv::Point2f pts[4]; rr.points(pts);
            for (int k = 0; k < 4; ++k) { pts[k].x += roi.x; pts[k].y += roi.y; }
            cv::Moments m = cv::moments(cnts[i]);
            if (std::abs(m.m00) < 1e-6) continue;

            Bar bar;
            bar.rr = cv::RotatedRect(cv::Point2f(rr.center.x + roi.x, rr.center.y + roi.y),
                                     rr.size, rr.angle);
            for (int k = 0; k < 4; ++k) bar.pts[k] = pts[k];
            bar.c = cv::Point2f((float)(m.m10 / m.m00 + roi.x),
                                (float)(m.m01 / m.m00 + roi.y));
            cv::Point2f e1 = pts[1] - pts[0], e2 = pts[2] - pts[1];
            cv::Point2f lv = (cv::norm(e1) >= cv::norm(e2)) ? e1 : e2;
            double tilt = std::atan2(lv.x, lv.y);
            if (tilt > CV_PI / 2) tilt -= CV_PI;
            if (tilt < -CV_PI / 2) tilt += CV_PI;
            bar.tilt = tilt;
            bar.long_h = std::max(cv::norm(e1), cv::norm(e2));
            bar.v = 0;
            bar.cnt = cnts[i];
            for (size_t k = 0; k < cnts[i].size(); ++k)
                bar.cnt[k] += cv::Point(roi.x, roi.y);
            bars.push_back(bar);
        }

        // 亮度过滤
        std::vector<Bar> keep;
        for (size_t i = 0; i < bars.size(); ++i) {
            cv::Mat m = cv::Mat::zeros(bgr.size(), CV_8UC1);
            cv::drawContours(m, std::vector<std::vector<cv::Point> >(1, bars[i].cnt),
                             -1, cv::Scalar(255), cv::FILLED);
            bars[i].v = cv::mean(vchan, m)[0];
            if (bars[i].v >= bar_v_min_) keep.push_back(bars[i]);
        }
        bars.swap(keep);
        std::sort(bars.begin(), bars.end(), [](const Bar& a, const Bar& b) { return a.c.x < b.c.x; });

        int id = 1;
        for (size_t i = 0; i < bars.size(); ++i) {
            for (size_t j = i + 1; j < bars.size(); ++j) {
                double dx = bars[j].c.x - bars[i].c.x;
                if (dx > max_dist_x_) break;
                if (dx <= min_dist_x_) continue;
                if (std::abs(bars[i].c.y - bars[j].c.y) > 50) continue;
                if (std::abs(bars[i].tilt - bars[j].tilt) * 180 / CV_PI > max_angle_) continue;
                double ang = 0.5 * (bars[i].tilt + bars[j].tilt);
                double corrected = dx / std::max(0.2, std::abs(std::cos(ang)));
                double lh = std::max(bars[i].long_h, bars[j].long_h);
                double ratio = corrected / std::max(1.0, lh);
                if (ratio < 0.4 || ratio > ratio_l_max_) continue;

                int min_x = 1 << 30, max_x = -1, min_y = 1 << 30, max_y = -1;
                for (int s2 = 0; s2 < 2; ++s2) {
                    const std::vector<cv::Point>& c = (s2 == 0) ? bars[i].cnt : bars[j].cnt;
                    for (size_t k = 0; k < c.size(); ++k) {
                        min_x = std::min(min_x, c[k].x); max_x = std::max(max_x, c[k].x);
                        min_y = std::min(min_y, c[k].y); max_y = std::max(max_y, c[k].y);
                    }
                }
                // 几何硬校验：太小的框直接丢，避免 PnP 退化
                if (max_x - min_x < 20 || max_y - min_y < 10) continue;

                Target tg;
                tg.id = id++;
                tg.is_large = ratio > ratio_s_max_;
                tg.ratio = ratio;
                tg.corners.push_back(cv::Point2f(min_x, min_y));
                tg.corners.push_back(cv::Point2f(max_x, min_y));
                tg.corners.push_back(cv::Point2f(max_x, max_y));
                tg.corners.push_back(cv::Point2f(min_x, max_y));
                tg.c2 = cv::Point2f((min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f);

                double half_w = tg.is_large ? 0.1125 : 0.0675, half_h = 0.0275;
                std::vector<cv::Point3f> obj;
                obj.push_back(cv::Point3f(-half_w, half_h, 0));
                obj.push_back(cv::Point3f(half_w, half_h, 0));
                obj.push_back(cv::Point3f(half_w, -half_h, 0));
                obj.push_back(cv::Point3f(-half_w, -half_h, 0));
                cv::Mat rvec, tvec;
                bool ok = cv::solvePnP(obj, tg.corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE);
                if (ok && tvec.at<double>(2) > 0.3 && tvec.at<double>(2) < 20.0) {
                    tg.has_pose = true;
                    tg.t = cv::Vec3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                    tg.z = tg.t[2];
                    cv::Mat R;
                    cv::Rodrigues(rvec, R);
                    tg.yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
                }
                targets.push_back(tg);
                break;
            }
        }

        // 主目标 + Alpha-Beta 平滑
        if (!targets.empty()) {
            int best = 0;
            for (size_t k = 1; k < targets.size(); ++k)
                if (std::abs(targets[k].yaw) < std::abs(targets[best].yaw)) best = (int)k;
            Target& mt = targets[best];
            if (mt.has_pose) {
                double dt = has_ab_ ? std::max(1e-3, (stamp - last_stamp_).toSec()) : 0.033;
                if (!has_ab_) {
                    ab_x_ = mt.t; ab_yaw_ = mt.yaw; ab_vx_ = cv::Vec3d(0, 0, 0); ab_w_ = 0; has_ab_ = true;
                } else {
                    cv::Vec3d pred = ab_x_ + ab_vx_ * dt;
                    double pred_yaw = ab_yaw_ + ab_w_ * dt;
                    cv::Vec3d r = mt.t - pred;
                    ab_x_ = pred + alpha_ * r;
                    ab_vx_ += beta_ * r / dt;
                    double ry = std::atan2(std::sin(mt.yaw - pred_yaw), std::cos(mt.yaw - pred_yaw));
                    ab_yaw_ = pred_yaw + alpha_ * ry;
                    ab_w_ += beta_ * ry / dt;
                }
                last_stamp_ = stamp;
                last_c_ = mt.c2;
                has_last_ = true;

                cv::rectangle(canvas,
                              cv::Rect((int)mt.corners[0].x, (int)mt.corners[0].y,
                                       (int)(mt.corners[1].x - mt.corners[0].x),
                                       (int)(mt.corners[3].y - mt.corners[0].y)),
                              cv::Scalar(0, 0, 255), 3);
                cv::putText(canvas, "ID:" + std::to_string(mt.id) + (mt.is_large ? " L" : " S"),
                            cv::Point((int)mt.corners[0].x, std::max(0, (int)mt.corners[0].y - 8)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
                pub_t_ = ab_x_;
                pub_yaw_ = ab_yaw_;
                pub_v_ = ab_vx_;
                published_ = true;
            }
        } else {
            has_last_ = false;
        }
    }

    cv::Vec3d pub_t_ = cv::Vec3d(0, 0, 0), pub_v_ = cv::Vec3d(0, 0, 0);
    double pub_yaw_ = 0;
    bool published_ = false;

private:
    static float tg_left(const Target& t) { return t.corners[0].x; }

    double aspect_min_, aspect_max_, min_dist_x_, max_dist_x_, max_angle_;
    double ratio_s_max_, ratio_l_max_;
    int bar_v_min_, roi_size_;
    double alpha_, beta_;
    bool has_last_ = false, has_ab_ = false;
    cv::Point2f last_c_ = cv::Point2f(0, 0);
    cv::Vec3d ab_x_ = cv::Vec3d(0, 0, 0), ab_vx_ = cv::Vec3d(0, 0, 0);
    double ab_yaw_ = 0, ab_w_ = 0;
    ros::Time last_stamp_;
};
