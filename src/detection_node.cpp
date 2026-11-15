#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <atomic>

class VisionNode {
public:
    // ==========================================================
    // 【一、初始化】建立通道、订阅数据、准备发布
    // ==========================================================
    VisionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
        : nh_(nh), it_(nh), pnh_(pnh), is_ready_(false), transport_ready_(true) 
    {
        // 1. 订阅相机内参（普通话题）
        info_sub_ = nh_.subscribe("/hk_camera/camera_info", 1, &VisionNode::cameraInfoCallback, this);

        // 2. 订阅图像（指定使用 compressed 插件解压，底层自动处理）
        try {
            image_sub_ = it_.subscribe(
                "/hk_camera/image_raw", 1,
                &VisionNode::imageCallback, this,
                image_transport::TransportHints("compressed", ros::TransportHints(), pnh_));
        } catch (const image_transport::TransportLoadException& e) {
            ROS_FATAL("compressed 传输插件加载失败: %s", e.what());
            transport_ready_ = false; // 插件缺失，标记失败
        }

        // 3. 准备发布处理后的图像（给 rqt 或 RViz 看）
        image_pub_ = it_.advertise("/ft/debug/image", 1);
    }

    // 供 main 函数查询：是否既拿到了内参，插件又正常
    bool isReady() const {
        return is_ready_.load() && transport_ready_;
    }

private:
    // ==========================================================
    // 【二、内参回调】把 ROS 的数据转换成 OpenCV 矩阵并缓存
    // ==========================================================
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        // 安全检查：丢弃空数据
        if (msg->width == 0 || msg->height == 0) {
            ROS_WARN_THROTTLE(2.0, "空 CameraInfo, 忽略");
            return;
        }

        // 1. 把一维数组 K 转换成 3x3 矩阵
        cv::Mat K(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                K.at<double>(r, c) = msg->K[r * 3 + c];

        // 2. 把 vector D 转换成 1xN 矩阵
        cv::Mat D(1, static_cast<int>(msg->D.size()), CV_64F);
        for (size_t i = 0; i < msg->D.size(); ++i)
            D.at<double>(0, static_cast<int>(i)) = msg->D[i];

        // 3. 缓存最新一份（如果换了分辨率，这里会直接覆盖旧数据）
        K_ = K;
        D_ = D;
        width_ = msg->width;
        height_ = msg->height;

        if (!is_ready_.load()) {
            ROS_INFO("CameraInfo 已缓存: %ux%u", msg->width, msg->height);
        }
        // 强制刷新状态位（绝对安全的多线程开关）
        is_ready_.store(true);
    }

    // ==========================================================
    // 【三、图像回调】核心流水线：转换 -> 校验 -> 画图 -> 打包发送
    // ==========================================================
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // 1. 如果内参还没准备好，直接扔掉这帧，不处理
        if (!is_ready_.load()) return;

        try {
            // 2. 零拷贝转换（toCvShare 只读，速度极快）
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat img = cv_ptr->image;

            // 3. 尺寸校验（防止相机换分辨率导致坐标计算错位）
            if (img.cols != width_ || img.rows != height_) return;

            // 4. 深拷贝一份用来画图（因为 toCvShare 是只读的）
            cv::Mat canvas = img.clone(); 
            // 在左上角画一个绿色的小方框作为 debug 标识
            cv::rectangle(canvas, cv::Rect(10, 10, 50, 50), cv::Scalar(0, 255, 0), 2);

            // 5. 打包成 ROS 消息格式
            cv_bridge::CvImage out_img;
            out_img.header.stamp = msg->header.stamp;    // 继承原图时间戳
            out_img.header.frame_id = msg->header.frame_id; // 继承原图坐标系
            out_img.encoding = "bgr8";
            out_img.image = canvas;
            
            // 6. 发送出去（推送到 /ft/debug/image）
            image_pub_.publish(out_img.toImageMsg());

        } catch (const cv_bridge::Exception& e) {
            // 如果转换失败，打印红字报错，程序不会崩溃
            ROS_ERROR("cv_bridge: %s", e.what());
        }
    }



    // --- 类成员变量 ---
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    
    ros::Subscriber info_sub_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_; 

    std::atomic<bool> is_ready_;     // 安全的状态开关
    bool transport_ready_;
    
    cv::Mat K_, D_;                  // 相机内参和畸变
    int width_ = 0, height_ = 0;     // 图像尺寸缓存
};

// ==========================================================
// 【四、主函数】启动门控与生命周期管理
// ==========================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_vision_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 实例化节点，触发上面的【一】
    VisionNode node(nh, pnh);

    // 设置 30 秒启动超时机制
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(30.0);
    ros::Rate rate(20.0);

    // 死等循环：直到内参缓存完毕（isReady 返回 true）
    while (ros::ok() && !node.isReady()) {
        ros::spinOnce(); // 处理回调，让内参进来
        if (ros::WallTime::now() > deadline) {
            ROS_FATAL("启动超时: camera_info 未到达 / compressed 插件缺失");
            return 1;
        }
        ROS_INFO_THROTTLE(2.0, "等待 CameraInfo / 插件就绪...");
        rate.sleep();
    }

    if (!ros::ok()) return 0;

    // 进入全自动模式：所有图像都会自动触发【三】里的流水线
    ROS_INFO("VisionNode 就绪，开始处理图像！");
    ros::spin();
    return 0;
}