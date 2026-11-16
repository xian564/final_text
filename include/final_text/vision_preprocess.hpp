#pragma once
// ============================================================
//  vision_preprocess.hpp
//  功能块：CLAHE(LAB) + 自适应 Gamma + 饱和度拉伸
//          颜色/高光/Canny 边缘融合掩膜
// ============================================================
#include <opencv2/opencv.hpp>
#include <vector>

struct Preprocessor {
    double clip_ = 2.0;
    double sat_gain_ = 1.2;
    int    canny_lo_ = 50, canny_hi_ = 150;
    int    sv_min_ = 120, yellow_s_ = 100, yellow_v_ = 100;
    int    blue_h_min_ = 100, blue_h_max_ = 140;
    int    blue_s_min_ = 50, blue_v_min_ = 50;

    cv::Mat enhance(const cv::Mat& bgr) const {
        cv::Mat lab, l, out;
        cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> ch;
        cv::split(lab, ch);
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clip_, cv::Size(8, 8));
        clahe->apply(ch[0], l);
        ch[0] = l;
        cv::merge(ch, lab);
        cv::cvtColor(lab, out, cv::COLOR_Lab2BGR);

        // 自适应 Gamma
        cv::Mat gray;
        cv::cvtColor(out, gray, cv::COLOR_BGR2GRAY);
        double mean = cv::mean(gray)[0];
        double m = std::max(1.0, std::min(254.0, mean));
        double gamma = std::log(0.5) / std::log(m / 255.0);
        gamma = std::max(0.6, std::min(2.2, gamma));
        cv::Mat lut(1, 256, CV_8U);
        for (int i = 0; i < 256; ++i)
            lut.at<uchar>(i) = cv::saturate_cast<uchar>(std::pow(i / 255.0, gamma) * 255.0);
        cv::LUT(out, lut, out);

        // 饱和度拉伸
        cv::Mat hsv;
        cv::cvtColor(out, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, ch);
        ch[1].convertTo(ch[1], -1, sat_gain_, 0);
        cv::merge(ch, hsv);
        cv::cvtColor(hsv, out, cv::COLOR_HSV2BGR);
        return out;
    }

    // 颜色 ∪ 高光 ∪ 边缘 融合
    void buildMask(const cv::Mat& bgr, cv::Mat& mask, cv::Mat& vchan) const {
        cv::Mat hsv;
        cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> ch;
        cv::split(hsv, ch);
        vchan = ch[2];

        cv::Mat r1, r2, r, b, g, y, color;
        cv::inRange(hsv, cv::Scalar(0, sv_min_, sv_min_), cv::Scalar(10, 255, 255), r1);
        cv::inRange(hsv, cv::Scalar(170, sv_min_, sv_min_), cv::Scalar(179, 255, 255), r2);
        r = r1 | r2;
        cv::inRange(hsv, cv::Scalar(blue_h_min_, blue_s_min_, blue_v_min_),
                    cv::Scalar(blue_h_max_, 255, 255), b);
        cv::inRange(hsv, cv::Scalar(50, sv_min_, sv_min_), cv::Scalar(70, 255, 255), g);
        cv::inRange(hsv, cv::Scalar(10, yellow_s_, yellow_v_), cv::Scalar(35, 255, 255), y);
        color = r | b | g | y;
        cv::morphologyEx(color, color, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        double maxv = 0; cv::minMaxLoc(vchan, nullptr, &maxv);
        double dyn = std::max(180.0, maxv - 50.0);
        cv::Mat high;
        cv::compare(vchan, dyn, high, cv::CMP_GT);

        cv::Mat gray, edges, ed;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        cv::Canny(gray, edges, canny_lo_, canny_hi_);
        cv::dilate(edges, ed, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        cv::Mat color_dil;
        cv::dilate(color, color_dil, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)),
                   cv::Point(-1, -1), 2);

        cv::Mat a = color & (ed | high);
        cv::Mat bb = high & color_dil;
        mask = a | bb;
    }
};
