#pragma once
// ============================================================
//  detector_interface.hpp
//  神经网络检测接口：IDetector / OnnxDetector（OpenCV DNN）
//  模型协议：输入 1x3x480x640（等比缩放+补边124、RGB/255）
//            输出 [1,21,6300]：颜色0-3、编号4-12、四角点13-20
//  角点顺序（模型）：左上、左下、右下、右上
// ============================================================
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <algorithm>

struct ArmorObservation {
    cv::Point2f corners[4];   // TL, TR, BR, BL（已映射回原图）
    int   color = 2;          // 0 蓝, 1 红, 2 熄灭
    int   number = 0;         // 1..9，0 表示未分类
    float score = 0.f;
};

class IDetector {
 public:
    virtual ~IDetector() = default;
    virtual bool ready() const { return true; }
    virtual bool detect(const cv::Mat& bgr,
                        std::vector<ArmorObservation>& armors) = 0;
};

class OnnxDetector : public IDetector {
 public:
    OnnxDetector(const std::string& model_path,
                 float color_thres = 0.5f,
                 float number_thres = 0.5f,
                 float nms_thres = 0.45f)
        : color_thres_(color_thres), number_thres_(number_thres),
          nms_thres_(nms_thres) {
        try {
            net_ = cv::dnn::readNetFromONNX(model_path);
            ready_ = !net_.empty();
        } catch (const cv::Exception& e) {
            ready_ = false;
        }
    }

    bool ready() const override { return ready_; }

    bool detect(const cv::Mat& bgr,
                std::vector<ArmorObservation>& armors) override {
        armors.clear();
        if (!ready_ || bgr.empty()) return false;

        const int IN_W = 640, IN_H = 480;
        float scale = std::min(IN_W / (float)bgr.cols, IN_H / (float)bgr.rows);
        int nw = std::max(1, (int)std::round(bgr.cols * scale));
        int nh = std::max(1, (int)std::round(bgr.rows * scale));
        int pad_x = (IN_W - nw) / 2, pad_y = (IN_H - nh) / 2;

        cv::Mat canvas(IN_H, IN_W, CV_8UC3, cv::Scalar(124, 124, 124));
        cv::resize(bgr, canvas(cv::Rect(pad_x, pad_y, nw, nh)), cv::Size(nw, nh));

        cv::Mat blob;
        cv::dnn::blobFromImage(canvas, blob, 1.0 / 255.0,
                               cv::Size(IN_W, IN_H), cv::Scalar(),
                               true, false);   // swapRB=true, center=false
        net_.setInput(blob);
        cv::Mat out = net_.forward();
        if (out.empty()) return false;

        // 输出 [1,21,6300] -> 21 x 6300
        int rows = out.size[1], cols = out.size[2];
        cv::Mat m = out.reshape(1, rows);

        std::vector<ArmorObservation> cand;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (int i = 0; i < cols; ++i) {
            float c0 = m.at<float>(0, i), c1 = m.at<float>(1, i), c2 = m.at<float>(2, i);
            int color = 0; float cs = c0;
            if (c1 > cs) { color = 1; cs = c1; }
            if (c2 > cs) { color = 2; cs = c2; }
            if (cs < color_thres_) continue;

            int best_n = 0; float ns = m.at<float>(4, i);
            for (int k = 1; k < 9; ++k) {
                float v = m.at<float>(4 + k, i);
                if (v > ns) { ns = v; best_n = k; }
            }
            if (ns < number_thres_) continue;

            cv::Point2f p[4];
            for (int k = 0; k < 4; ++k) {
                float x = m.at<float>(13 + 2 * k, i);
                float y = m.at<float>(14 + 2 * k, i);
                p[k] = cv::Point2f((x - pad_x) / scale, (y - pad_y) / scale);
            }
            // 模型顺序 TL, BL, BR, TR -> PnP 需要 TL, TR, BR, BL
            ArmorObservation a;
            a.corners[0] = p[0];
            a.corners[1] = p[3];
            a.corners[2] = p[2];
            a.corners[3] = p[1];
            a.color = color;
            a.number = best_n;
            a.score = cs;
            cand.push_back(a);

            float minx = std::min(std::min(p[0].x, p[1].x), std::min(p[2].x, p[3].x));
            float maxx = std::max(std::max(p[0].x, p[1].x), std::max(p[2].x, p[3].x));
            float miny = std::min(std::min(p[0].y, p[1].y), std::min(p[2].y, p[3].y));
            float maxy = std::max(std::max(p[0].y, p[1].y), std::max(p[2].y, p[3].y));
            boxes.push_back(cv::Rect((int)minx, (int)miny, (int)(maxx - minx), (int)(maxy - miny)));
            scores.push_back(cs);
        }

        std::vector<int> keep;
        if (!boxes.empty())
            cv::dnn::NMSBoxes(boxes, scores, color_thres_, nms_thres_, keep);
        for (size_t k = 0; k < keep.size(); ++k)
            armors.push_back(cand[keep[k]]);
        return !armors.empty();
    }

 private:
    cv::dnn::Net net_;
    bool ready_ = false;
    float color_thres_, number_thres_, nms_thres_;
};
