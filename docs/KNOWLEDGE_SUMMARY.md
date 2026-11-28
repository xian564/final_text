# 装甲板视觉 · 详细知识总结（安装 / 代码模板 / 测试）

> 配套：`docs/EMERGENCY_MANUAL.md`（排障）、`docs/TESTING.md`（流程）、`docs/nn_tutorial.md`（NN）。
> 本文偏“怎么做”：安装命令、模板代码、注释、测试命令，可直接抄改。

---

## 1. 安装过程（从零到能跑）

### 1.1 系统与 ROS
```bash
# Ubuntu 20.04
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
```
验证：
```bash
roscore
# 另开终端
rostopic list
```

### 1.2 依赖
```bash
sudo apt install -y \
  build-essential cmake git \
  libopencv-dev python3-opencv \
  ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  ros-noetic-dynamic-reconfigure ros-noetic-rqt-reconfigure \
  ros-noetic-rqt-image-view ros-noetic-rviz \
  ros-noetic-rosbag
```

### 1.3 工作空间
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 1.4 消息包 `ft_msgs`
```
ft_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── Detection.msg
    ├── Detections.msg
    └── Track.msg
```
`Detection.msg`：
```
Header header
int32 id
int32 number
bool is_red
bool is_large_armor
geometry_msgs/Pose pose
```
`Detections.msg`：
```
Header header
Detection[] detections
```
`Track.msg`：
```
Header header
int32 id
bool tracking
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Point position_predict
geometry_msgs/Point body_center
bool valid
```
`CMakeLists.txt` 关键：
```cmake
find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs)
add_message_files(FILES Detection.msg Detections.msg Track.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
```

### 1.5 编译本包
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 1.6 ONNX Runtime C++（NN 可选）
```bash
# 头文件（jsDelivr 镜像）
mkdir -p third_party/ort/include/onnxruntime/core/session
cd third_party/ort/include/onnxruntime/core/session
BASE="https://cdn.jsdelivr.net/gh/microsoft/onnxruntime@v1.31.1/include/onnxruntime/core/session"
for f in onnxruntime_c_api.h onnxruntime_cxx_api.h onnxruntime_cxx_inline.h \
         onnxruntime_float16.h onnxruntime_error_code.h onnxruntime_ep_c_api.h; do
  curl -L "$BASE/$f" -o "$f"
done

# 库（pip 镜像里的 wheel 自带 .so）
pip3 install --no-deps --target /tmp/ort -i https://pypi.tuna.tsinghua.edu.cn/simple onnxruntime==1.31.1
mkdir -p third_party/ort/lib
cp /tmp/ort/onnxruntime/capi/libonnxruntime.so.1.31.1 third_party/ort/lib/libonnxruntime.so
ln -sf libonnxruntime.so third_party/ort/lib/libonnxruntime.so.1
```
CMake：
```cmake
set(CMAKE_CXX_STANDARD 17)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ort/include)
target_link_libraries(detect_node_nn
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
  ${PROJECT_SOURCE_DIR}/third_party/ort/lib/libonnxruntime.so)
set_target_properties(detect_node_nn PROPERTIES
  BUILD_RPATH ${PROJECT_SOURCE_DIR}/third_party/ort/lib)
```
验证：
```bash
rosrun final_text detect_node_nn _model_path:=/path/armor_allfp32.onnx
# 日志: NN armors=1 infer_avg=55.2 ms
```

### 1.7 模型
- `armor_allfp32.onnx`（fp32、opset19）用于 ORT C++；
- 原始 `armor.onnx` 可能是 FP16/新算子，OpenCV DNN 加载失败；
- 模型路径不要写死，走参数 `model_path`。

---

## 2. 工程骨架模板

### 2.1 `package.xml`
```xml
<package format="2">
  <name>final_text</name>
  <version>0.1.0</version>
  <description>Armor detection and tracking</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>dynamic_reconfigure</depend>
  <depend>ft_msgs</depend>
  <exec_depend>rviz</exec_depend>
  <exec_depend>rqt_image_view</exec_depend>
  <exec_depend>rqt_reconfigure</exec_depend>
</package>
```

### 2.2 `CMakeLists.txt` 模板
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(final_text)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs geometry_msgs std_msgs
  cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs
  dynamic_reconfigure ft_msgs)
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/Vision.cfg)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs
  cv_bridge image_transport tf2 tf2_ros dynamic_reconfigure ft_msgs)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)

add_executable(detect_node_v3_1 src/detection_node_v3_1.cpp)
target_link_libraries(detect_node_v3_1 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(detect_node_v3_1 ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)
```

### 2.3 `launch` 模板（带模式切换）
```xml
<launch>
  <param name="use_sim_time" value="true" />
  <arg name="mode" default="tradition" />   <!-- tradition | nn -->
  <arg name="model_path" default="/path/armor_allfp32.onnx" />

  <node pkg="final_text" type="detect_node_v3_1" name="vision_node_v3" output="screen">
    <rosparam command="load" file="$(find final_text)/config/params.yaml" />
    <param name="nn_enable" value="$(eval arg('mode') == 'nn')" />
    <param name="nn_model_path" value="$(arg model_path)" />
  </node>

  <node pkg="final_text" type="ft_tracker_node" name="ft_tracker" output="screen">
    <rosparam command="load" file="$(find final_text)/config/tracker.yaml" />
  </node>

  <node pkg="tf2_ros" type="static_transform_publisher" name="cam_tf"
        args="0 0 0 0 0 0 camera2_optical_frame hk_camera_ca" />
</launch>
```

### 2.4 `cfg/Vision.cfg` 模板
```python
#!/usr/bin/env python3
PACKAGE = "final_text"
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
gen.add("blue_h_min", int_t, 0, "blue H min", 85, 0, 180)
gen.add("min_area",   int_t, 0, "min contour area", 20, 1, 5000)
gen.add("z_min", double_t, 0, "min z", 0.3, 0.1, 5.0)
gen.add("z_max", double_t, 0, "max z", 15.0, 1.0, 30.0)
exit(gen.generate(PACKAGE, "vision_node_v3", "Vision"))
```
节点里：
```cpp
dr_srv_.reset(new dynamic_reconfigure::Server<final_text::VisionConfig>(pnh_));
dr_srv_->setCallback(boost::bind(&VisionNode::drCallback, this, _1, _2));

void drCallback(final_text::VisionConfig& cfg, uint32_t) {
    pnh_.setParam("blue_h_min", cfg.blue_h_min);
    blue_h_min_ = cfg.blue_h_min;   // 立即生效
}
```

---

## 3. ROS 节点模板

### 3.1 图像订阅（compressed）
```cpp
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

image_transport::ImageTransport it_(nh_);
image_transport::Subscriber sub_ = it_.subscribe(
    "/hk_camera/image_raw", 1,
    &Node::imageCallback, this,
    image_transport::TransportHints("compressed"));

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (img.empty()) return;
    // ...
}
```

### 3.2 内参/畸变
```cpp
void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    K_ = cv::Mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            K_.at<double>(r, c) = msg->K[r * 3 + c];
    D_ = cv::Mat(1, msg->D.size(), CV_64F);
    for (size_t i = 0; i < msg->D.size(); ++i) D_.at<double>(0, i) = msg->D[i];
    camera_frame_ = msg->header.frame_id;
    is_ready_ = true;
}
```

### 3.3 发布检测数组
```cpp
ft_msgs::Detections arr;
arr.header = msg->header;          // 图像时间戳
arr.header.frame_id = camera_frame_;
for (auto& tg : targets) {
    ft_msgs::Detection d;
    d.header = arr.header;
    d.id = tg.id;
    d.number = tg.number;
    d.is_red = tg.is_red;
    d.is_large_armor = tg.is_large;
    d.pose.position.x = tg.t[0];
    d.pose.position.y = tg.t[1];
    d.pose.position.z = tg.t[2];
    d.pose.orientation = rvecToQuat(tg.r);
    arr.detections.push_back(d);
}
pub_.publish(arr);
```

### 3.4 TF
```cpp
geometry_msgs::TransformStamped ts;
ts.header.stamp = msg->header.stamp;
ts.header.frame_id = camera_frame_;
ts.child_frame_id = "armor_v3";
ts.transform.translation.x = t[0];
ts.transform.translation.y = t[1];
ts.transform.translation.z = t[2];
cv::Mat R; cv::Rodrigues(rvec, R);
tf2::Matrix3x3 m(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                 R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                 R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
tf2::Quaternion q; m.getRotation(q);
ts.transform.rotation = tf2::toMsg(q);
tf_broadcaster_.sendTransform(ts);
```

### 3.5 Marker
```cpp
visualization_msgs::Marker mk;
mk.header.stamp = stamp;
mk.header.frame_id = camera_frame_;
mk.ns = "pred"; mk.id = 0;
mk.type = visualization_msgs::Marker::SPHERE;
mk.action = visualization_msgs::Marker::ADD;
mk.pose.position = pred;          // geometry_msgs/Point
mk.scale.x = mk.scale.y = mk.scale.z = 0.08;
mk.color.r = 1; mk.color.g = 0; mk.color.b = 0; mk.color.a = 1;
marker_pub_.publish(mk);
```

### 3.6 参数读取 + 重载
```cpp
pnh_.param("blue_h_min", blue_h_min_, 85);
pnh_.param("min_area",  min_area_, 20);
// 定时重读，支持 rosparam set
void reload(const ros::TimerEvent&) {
    pnh_.getParam("blue_h_min", blue_h_min_);
}
```

---

## 4. 视觉算法模板

### 4.1 HSV 分色
```cpp
cv::Mat hsv, mask1, mask2, mask;
cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
cv::inRange(hsv, cv::Scalar(0, 43, 46),   cv::Scalar(10, 255, 255), mask1);
cv::inRange(hsv, cv::Scalar(170, 43, 46), cv::Scalar(179, 255, 255), mask2);
cv::bitwise_or(mask1, mask2, mask);
cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
```

### 4.2 灯条筛选
```cpp
std::vector<std::vector<cv::Point>> contours;
cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
for (auto& c : contours) {
    if (cv::contourArea(c) < min_area_) continue;
    cv::RotatedRect rr = cv::minAreaRect(c);
    float w = rr.size.width, h = rr.size.height;
    if (w < 1 || h < 1) continue;
    float ar = std::max(w,h) / std::min(w,h);
    if (ar < 2.0 || ar > 12.0) continue;
    // 亮度
    cv::Mat roi = v_chan(rr.boundingRect() & cv::Rect(0,0,v_chan.cols,v_chan.rows));
    if (cv::mean(roi)[0] < bar_v_min_) continue;
    // 角度归一化
    float ang = rr.angle;
    if (ang > 45) ang -= 90;
    // 中心用矩
    cv::Moments m = cv::moments(c);
    cv::Point2f ctr(m.m10/m.m00, m.m01/m.m00);
}
```

### 4.3 配对 + PnP
```cpp
// 两灯条
double dx = std::abs(l.center.x - r.center.x);
double dy = std::abs(l.center.y - r.center.y);
double ddeg = std::abs(l.tilt - r.tilt);
if (dx < min_dx || dx > max_dist_x) continue;
if (dy > pair_dy_ratio * std::min(l.height, r.height)) continue;
if (ddeg > max_angle_diff) continue;

// 四点 TL TR BR BL（灯条端点）
std::vector<cv::Point2f> img = {lt, rt, rb, lb};
// 物体点：Wb x bH（米）
std::vector<cv::Point3f> obj = {
    {-Wb/2, -bH/2, 0}, {Wb/2, -bH/2, 0},
    {Wb/2,  bH/2, 0}, {-Wb/2, bH/2, 0}};
cv::Mat rvec, tvec;
bool ok = cv::solvePnP(obj, img, K_, D_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
if (!ok) ok = cv::solvePnP(obj, img, K_, D_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
if (!ok || tvec.at<double>(2) < z_min_ || tvec.at<double>(2) > z_max_) continue;
```

### 4.4 Kalman
```cpp
cv::KalmanFilter kf(6, 3, 0, CV_32F);
kf.transitionMatrix = cv::Mat::eye(6,6,CV_32F);
kf.measurementMatrix = cv::Mat::zeros(3,6,CV_32F);
for (int i=0;i<3;++i) kf.measurementMatrix.at<float>(i,i)=1;
cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-2));
cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-2));
cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));

// 每帧
double dt = (stamp - last).toSec();
kf.transitionMatrix.at<float>(0,3)=dt;
kf.transitionMatrix.at<float>(1,4)=dt;
kf.transitionMatrix.at<float>(2,5)=dt;
kf.predict();
cv::Mat meas = (cv::Mat_<float>(3,1) << x,y,z);
kf.correct(meas);
// 提前量
float vx=kf.statePost.at<float>(3), vy=..., vz=...;
point_predict = point + v*T;
```

### 4.5 Kasa 圆拟合（旋转）
```cpp
// 2a*x + 2b*z + c = x^2+z^2
// 由 Σx,Σz,Σx²,Σz²,Σxz 组 3x3 法方程
cv::Mat A = (cv::Mat_<double>(3,3) <<
  2*Sx, 2*Sz, n,
  2*Sxx,2*Sxz,Sx,
  2*Sxz,2*Szz,Sz);
cv::Mat b = (cv::Mat_<double>(3,1) << Sxx+Szz, Sx3+Sxzz, Sx3+Sz3);
cv::Mat s; cv::solve(A,b,s,cv::DECOMP_SVD);
double a=s.at<double>(0), bb=s.at<double>(1), cc=s.at<double>(2);
double R=std::sqrt(cc+a*a+bb*bb);
```

### 4.6 旋转预测
```cpp
double phase = std::atan2(z - cz, x - cx);
double dph = std::atan2(std::sin(phase-phase_), std::cos(phase-phase_));
omega_ = 0.8*omega_ + 0.2*(dph/dt);
double th = phase + omega_*T;
double cx2 = cx + vx*T, cz2 = cz + vz*T;   // 先直线
pred.x = cx2 + R*std::cos(th);
pred.z = cz2 + R*std::sin(th);
pred.y = y + vy*T;
```

---

## 5. 神经网络模板

### 5.1 letterbox（保存缩放/补边）
```cpp
const int IN_W = 640, IN_H = 480;
float scale = std::min(IN_W / (float)bgr.cols, IN_H / (float)bgr.rows);
int nw = std::round(bgr.cols * scale);
int nh = std::round(bgr.rows * scale);
int pad_x = (IN_W - nw) / 2, pad_y = (IN_H - nh) / 2;

cv::Mat canvas(IN_H, IN_W, CV_8UC3, cv::Scalar(124,124,124));
cv::resize(bgr, canvas(cv::Rect(pad_x,pad_y,nw,nh)), cv::Size(nw,nh));
// 还原: x_orig = (x_net - pad_x) / scale
```

### 5.2 ORT 会话
```cpp
Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "armor_nn");
Ort::SessionOptions opt;
opt.SetIntraOpNumThreads(1);
Ort::Session session(env, model_path.c_str(), opt);

Ort::AllocatorWithDefaultOptions alloc;
auto in_name  = session.GetInputNameAllocated(0, alloc);
auto out_name = session.GetOutputNameAllocated(0, alloc);
```

### 5.3 输入张量（NCHW）
```cpp
std::vector<float> input(3*IN_H*IN_W);
for (int y=0;y<IN_H;++y)
  for (int x=0;x<IN_W;++x) {
    cv::Vec3b p = canvas.at<cv::Vec3b>(y,x);
    input[0*plane + y*IN_W + x] = p[2]/255.f; // R
    input[1*plane + y*IN_W + x] = p[1]/255.f; // G
    input[2*plane + y*IN_W + x] = p[0]/255.f; // B
  }
std::array<int64_t,4> shape{1,3,IN_H,IN_W};
auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
auto tensor = Ort::Value::CreateTensor<float>(
    mem, input.data(), input.size(), shape.data(), 4);
```

### 5.4 解码
```cpp
// out: [1,21,6300]
float* o = outputs[0].GetTensorMutableData<float>();
int cols = 6300;
for (int i=0;i<cols;++i) {
    // 颜色 0..2
    int color=0; float cs=o[0*cols+i];
    if (o[1*cols+i] > cs) { color=1; cs=o[1*cols+i]; }
    if (o[2*cols+i] > cs) { color=2; cs=o[2*cols+i]; }
    if (cs < color_thres) continue;

    // 编号 4..12
    int num=0; float ns=o[4*cols+i];
    for (int k=1;k<9;++k)
        if (o[(4+k)*cols+i] > ns) { ns=o[(4+k)*cols+i]; num=k; }
    if (ns < number_thres) continue;

    // 四角点 13..20
    cv::Point2f p[4];
    for (int k=0;k<4;++k) {
        float x = o[(13+2*k)*cols+i];
        float y = o[(14+2*k)*cols+i];
        p[k] = {(x-pad_x)/scale, (y-pad_y)/scale};
    }
    // 顺序 TL,TR,BR,BL
    a.corners[0]=p[0]; a.corners[1]=p[3];
    a.corners[2]=p[2]; a.corners[3]=p[1];
    a.color=color; a.number=num+1; a.score=cs;
}
```

### 5.5 NMS
```cpp
std::vector<cv::Rect> boxes; std::vector<float> scores;
// 由四点外接矩形填充
std::vector<int> keep;
cv::dnn::NMSBoxes(boxes, scores, color_thres, 0.45, keep);
```

### 5.6 数字模板匹配
```cpp
cv::Mat bin;
cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
if (cv::mean(bin)[0] > 127) cv::bitwise_not(bin, bin);
cv::resize(bin, bin, cv::Size(20,30));

double best=-1; int label=0;
for (size_t i=0;i<templates.size();++i) {
    cv::Mat res;
    cv::matchTemplate(bin, templates[i], res, cv::TM_CCOEFF_NORMED);
    if (res.at<float>(0,0) > best) { best=res.at<float>(0,0); label=labels[i]; }
}
return best > 0.5 ? label : 0;
```

---

## 6. 测试命令与测试代码

### 6.1 启动/检查
```bash
rosbag play bag/red_4.bag --clock -l
roslaunch final_text test_vision.launch mode:=tradition

rostopic hz /ft/detection /ft/track /ft/debug/image
rostopic echo -n1 /ft/detection
rosnode list
rosrun rqt_image_view rqt_image_view
```

### 6.2 检查字段（Python）
```python
import rospy
from ft_msgs.msg import Detections, Track

def cb(m):
    for d in m.detections:
        print(d.id, d.number, d.is_red, d.pose.position.z)

rospy.init_node('chk')
rospy.Subscriber('/ft/detection', Detections, cb)
rospy.spin()
```

### 6.3 检查 TF / marker
```bash
rosrun tf tf_echo hk_camera_ca armor_v3
rostopic echo -n1 /ft/debug/marker
```

### 6.4 动态调参
```bash
rosrun dynamic_reconfigure dynparam set /vision_node_v3 blue_h_min 120
rosparam get /vision_node_v3/blue_h_min
rosrun dynamic_reconfigure dynparam set /vision_node_v3 blue_h_min 85
```

### 6.5 截图
```bash
rosrun final_text capture_debug.py _topic:=/ft/debug/image _out:=/tmp/shot _n:=5
rosrun final_text capture_compare.py _out:=/tmp/compare _n:=3
```

### 6.6 录包
```bash
rosbag record -O run.bag /ft/detection /ft/track /ft/debug/image
rosbag info run.bag
```

### 6.7 单元测试模板（gtest）
```cpp
#include <gtest/gtest.h>
#include "final_text/detector_interface.hpp"

TEST(Detector, SampleImage) {
    OnnxDetector d("/path/armor_allfp32.onnx");
    ASSERT_TRUE(d.ready());
    cv::Mat img = cv::imread("/path/sample_1.jpg");
    std::vector<ArmorObservation> a;
    EXPECT_TRUE(d.detect(img, a));
    EXPECT_GT(a.size(), 0u);
    EXPECT_GE(a[0].score, 0.5f);
}
```
CMake：
```cmake
catkin_add_gtest(test_detector test/test_detector.cpp)
target_link_libraries(test_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
```

### 6.8 测试检查表
| 项 | 期望 |
|---|---|
| `catkin_make` | EXIT 0 |
| `/ft/detection` | 15–35 Hz，z 合理，无 NaN |
| `/ft/track` | 40–80 Hz，tracking 正确 |
| `/ft/debug/image` | 有框/三轴 |
| `/ft/debug/marker` | 预测点可见 |
| `tf_echo` | 位姿连续 |
| 动态调参 | 数值变化、画面变化 |
| 切换 mode | tradition/nn 都能检测 |

---

## 7. 用法速查（模板）

### 7.1 新增一个阈值
1. `params.yaml` 加键；
2. 构造函数 `pnh_.param("key", var_, default);`；
3. 如果是识别阈值，加 `cfg/Vision.cfg` + `drCallback`；
4. 代码里替换字面量；
5. 测试 `rosparam get` / rqt。

### 7.2 新增一个消息字段
1. `ft_msgs/msg/X.msg` 加字段；
2. `catkin_make` 重新生成；
3. 发布端填字段；
4. 订阅端按需读；
5. 记录到 README。

### 7.3 新增一个 launch 参数
```xml
<arg name="mode" default="tradition" />
<param name="nn_enable" value="$(eval arg('mode') == 'nn')" />
```

### 7.4 新增一个节点
1. `src/xxx.cpp`：`ros::init` + 订阅 + 发布；
2. CMake `add_executable` + `target_link_libraries`；
3. launch 里加 `<node>`；
4. `rosnode list` 验证。

---

## 8. 提交前检查

- [ ] `catkin_make` 通过
- [ ] `test_vision.launch mode:=tradition` 有数据
- [ ] `mode:=nn` 有数据（若启用）
- [ ] `/ft/detection`、`/ft/track`、`/ft/debug/image` 正常
- [ ] 无 `imshow/waitKey`
- [ ] 阈值在 YAML/cfg，无新增硬编码
- [ ] `header.stamp` = 图像时间
- [ ] README/ai_usage 更新
- [ ] 视频/截图归档
- [ ] tag `vision-final-test-submit`
