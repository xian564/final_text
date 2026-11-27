#pragma once
// 神经网络检测接口：IDetector / OnnxDetector（ONNX Runtime C++ 后端）
// 模型协议：输入 1x3x480x640（等比缩放+补边124、RGB/255）
// 输出 [1,21,6300]：颜色0-3、编号4-12、四角点13-20
#include <opencv2/opencv.hpp>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

struct ArmorObservation {
    cv::Point2f corners[4];   // TL, TR, BR, BL（原图坐标）
    int   color = 2;          // 0 蓝, 1 红, 2 熄灭
    int   number = 0;         // 0=未知, 1..8
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
        if (model_path.empty()) return;
        try {
            env_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "armor_nn"));
            Ort::SessionOptions opt;
            opt.SetIntraOpNumThreads(1);
            session_.reset(new Ort::Session(*env_, model_path.c_str(), opt));
            Ort::AllocatorWithDefaultOptions alloc;
            auto in = session_->GetInputNameAllocated(0, alloc);
            auto out = session_->GetOutputNameAllocated(0, alloc);
            input_name_ = in.get();
            output_name_ = out.get();
            ready_ = true;
        } catch (const std::exception& e) {
            fprintf(stderr, "ORT load failed: %s\n", e.what());
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

        cv::Mat canvas(IN_H, IN_W, CV_8UC3, cv::Scalar(124,124,124));
        cv::resize(bgr, canvas(cv::Rect(pad_x, pad_y, nw, nh)), cv::Size(nw, nh));

        std::vector<float> input(3 * IN_H * IN_W, 0.f);
        const int plane = IN_H * IN_W;
        for (int y = 0; y < IN_H; ++y) {
            const cv::Vec3b* row = canvas.ptr<cv::Vec3b>(y);
            for (int x = 0; x < IN_W; ++x) {
                // BGR -> RGB, /255, NCHW
                input[0*plane + y*IN_W + x] = row[x][2] / 255.0f;
                input[1*plane + y*IN_W + x] = row[x][1] / 255.0f;
                input[2*plane + y*IN_W + x] = row[x][0] / 255.0f;
            }
        }
        std::array<int64_t,4> in_shape{1,3,IN_H,IN_W};
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value tensor = Ort::Value::CreateTensor<float>(
            mem, input.data(), input.size(), in_shape.data(), in_shape.size());

        const char* in_names[] = { input_name_.c_str() };
        const char* out_names[] = { output_name_.c_str() };
        std::vector<Ort::Value> outputs;
        try {
            outputs = session_->Run(Ort::RunOptions{nullptr},
                                    in_names, &tensor, 1, out_names, 1);
        } catch (const std::exception&) {
            return false;
        }
        if (outputs.empty()) return false;
        float* out = outputs[0].GetTensorMutableData<float>();
        auto info = outputs[0].GetTensorTypeAndShapeInfo();
        auto shape = info.GetShape();
        if (shape.size() != 3) return false;
        int rows = (int)shape[1], cols = (int)shape[2];

        std::vector<ArmorObservation> cand;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        for (int i = 0; i < cols; ++i) {
            float c0 = out[0*cols + i], c1 = out[1*cols + i], c2 = out[2*cols + i];
            int color = 0; float cs = c0;
            if (c1 > cs) { color = 1; cs = c1; }
            if (c2 > cs) { color = 2; cs = c2; }
            if (cs < color_thres_) continue;

            int best_n = 0; float ns = out[4*cols + i];
            for (int k = 1; k < 9; ++k) {
                float v = out[(4+k)*cols + i];
                if (v > ns) { ns = v; best_n = k; }
            }
            if (ns < number_thres_) continue;

            cv::Point2f p[4];
            for (int k = 0; k < 4; ++k) {
                float x = out[(13+2*k)*cols + i];
                float y = out[(14+2*k)*cols + i];
                p[k] = cv::Point2f((x - pad_x) / scale, (y - pad_y) / scale);
            }
            ArmorObservation a;
            a.corners[0] = p[0];   // TL
            a.corners[1] = p[3];   // TR
            a.corners[2] = p[2];   // BR
            a.corners[3] = p[1];   // BL
            a.color = color;
            a.number = best_n + 1;
            a.score = cs;
            cand.push_back(a);

            float minx = std::min(std::min(p[0].x,p[1].x), std::min(p[2].x,p[3].x));
            float maxx = std::max(std::max(p[0].x,p[1].x), std::max(p[2].x,p[3].x));
            float miny = std::min(std::min(p[0].y,p[1].y), std::min(p[2].y,p[3].y));
            float maxy = std::max(std::max(p[0].y,p[1].y), std::max(p[2].y,p[3].y));
            boxes.push_back(cv::Rect((int)minx, (int)miny, (int)(maxx-minx), (int)(maxy-miny)));
            scores.push_back(cs);
        }
        std::vector<int> keep;
        if (!boxes.empty())
            cv::dnn::NMSBoxes(boxes, scores, color_thres_, nms_thres_, keep);
        for (size_t k = 0; k < keep.size(); ++k) armors.push_back(cand[keep[k]]);
        return !armors.empty();
    }

 private:
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_, output_name_;
    bool ready_ = false;
    float color_thres_, number_thres_, nms_thres_;
};
