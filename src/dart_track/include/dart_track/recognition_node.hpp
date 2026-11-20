#pragma once
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include "dart_track_interfaces/msg/detections.hpp"
#include "dart_track/draw_renderer.hpp"

using Detections = dart_track_interfaces::msg::Detections;
namespace dart_track 
{

    class RecognitionNode : public rclcpp::Node 
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr exposure_sub_;
        double exposure_us_ = 0.0;   // 0 = 还没收到
        rclcpp::Publisher<Detections>::SharedPtr detections_pub_;
        // 画好的图（识别 + 绘图在同一个回调里完成，没有额外一跳）
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr draw_pub_;

        cv::Point2d dart_last_center_ = cv::Point2d(-1.0, -1.0);
        // 灯条是刚体，相邻两帧的"尾->头"方向不会突然转 180°。
        // 用上一帧的方向先把头尾定下来，比只看运动方向稳得多。
        cv::Point2d dart_last_axis_ = cv::Point2d(0.0, 0.0);
        bool dart_has_last_center_ = false;
        bool dart_has_last_axis_ = false;
        int dart_miss_count_ = 0;   // 连续没找到灯条的帧数
        // ========================参数设置===========================
        // 引导灯HSV阈值
        int LIGHT_H_LO = 40, LIGHT_H_HI = 85;
        int LIGHT_S_LO = 80, LIGHT_S_HI = 255;
        int LIGHT_V_LO = 100, LIGHT_V_HI = 255;

        // 飞镖灯条HSV阈值
        int DART_H_LO = 40, DART_H_HI = 85;
        int DART_S_LO = 80, DART_S_HI = 255;
        int DART_V_LO = 100, DART_V_HI = 255;

        // 引导灯，灯条最小面积
        double LIGHT_MIN_AREA = 30.0;
        double DART_MIN_AREA = 60.0;
        // 灯条长宽比
        double LENGTH_WIDTH_RATIO = 1.8;

        // ===== 引导灯筛选（现实环境用）=====
        // 灯是圆形的，长宽比超过这个值就不当灯。
        // 但灯在地面上，倒影会和灯连成竖向长条，所以放到 6 是安全的
        double LIGHT_MAX_ASPECT = 6.0;
        // 自适应亮度门限：以画面里最亮的绿色像素为基准取这个比例。
        // 现实场景整体偏暗、灯却有饱和亮芯，相对阈值比固定阈值稳得多
        double LIGHT_V_AUTO_RATIO = 0.8;
        // 邻近候选合并半径：一个灯常被切成"灯芯 + 倒影"两段，合并回一个
        double LIGHT_CLUSTER_RADIUS_PX = 50.0;

        // 复用的中间缓冲：每帧 cvtColor/inRange 都重新分配 1.5MB 太浪费
        cv::Mat hsv_buf_;
        cv::Mat light_mask_buf_;
        cv::Mat dart_mask_buf_;

        // ===== 绘图 =====
        std::unique_ptr<DrawRenderer> renderer_;
        double DRAW_SCALE = 0.5;
        double DRAW_FPS = 30.0;              // 绘图发帖帧率：给人看的，不必跟满 160
        int64_t last_draw_ns_ = 0;
        bool DRAW_ENABLE = true;
        uint64_t cam_count_ = 0;      // 收到的帧数
        uint64_t draw_count_ = 0;     // 画出去的帧数
        double cam_fps_ = 0.0;
        double draw_fps_ = 0.0;
        std::chrono::steady_clock::time_point fps_window_start_{};



    private:
        void recognition_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);
        void compressed_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);
        // 真正干活的地方：拿到一帧 BGR 图和它的时间戳
        void process_frame(const cv::Mat& frame, const std_msgs::msg::Header& header);
        void detections_publish
        (
            const std_msgs::msg::Header & header,
            const cv::Mat& frame,
            std::vector<cv::Point2d>& light_centers,
            cv::Point2d& dart_front_mid,
            cv::Point2d& dart_back_mid);
        // 把这一帧画出来并发到 draw 话题
        void draw_publish(const std_msgs::msg::Header& header, const cv::Mat& frame,
                          const Detections& det);
        void refresh_fps();
    public:
        explicit RecognitionNode(const rclcpp::NodeOptions & options);
    };

}