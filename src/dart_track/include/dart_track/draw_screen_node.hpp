#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "dart_track_interfaces/msg/detections.hpp"
using Detections = dart_track_interfaces::msg::Detections;

namespace dart_track
{
    

    // 纯显示节点：把 /dart_detections 里的点画到 /camera/image_raw 上，然后弹窗显示。
    // 关键设计：先攒一小段图，等"同一帧"的识别结果到了再显示，
    // 这样画上去的点和这张图严丝合缝，不会出现"点比画面慢一帧"的错位。
    class DrawScreenNode : public rclcpp::Node
    {
    public:
        explicit DrawScreenNode(const rclcpp::NodeOptions & options);

    private:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);
        void detections_callback(const Detections::SharedPtr msg);

        void flush_pending();
        void render_frame(cv::Mat & frame, const Detections * det, double age_ms);
        void draw_detections(cv::Mat & frame, const Detections & det);
        void draw_hud(cv::Mat & frame, const Detections * det, double age_ms);
        void refresh_fps();

        // 一帧等待配对的图像
        struct PendingFrame
        {
            int64_t stamp_ns = 0;                              // 这一帧的时间戳，用来找它对应的识别结果
            std::chrono::steady_clock::time_point arrived;     // 什么时候收到的，用来判超时
            cv::Mat frame;
        };

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<Detections>::SharedPtr detections_sub_;

        std::deque<PendingFrame> pending_;                     // 按到达顺序排队，等识别结果
        std::map<int64_t, Detections::SharedPtr> det_cache_;   // 时间戳 -> 该帧的识别结果

        double DRAW_MAX_FPS = 60.0;         // 显示上限，超过就丢帧（不影响配对）
        double DRAW_SCALE = 0.5;            // 显示缩放
        double DRAW_PAIR_TIMEOUT = 0.2;     // 等识别结果的极限（秒），等不到就不带点直接显示
        bool SHOW_WINDOW = true;            // 无图形界面时可关掉

        std::chrono::steady_clock::time_point last_draw_time_;
        std::chrono::steady_clock::time_point fps_window_start_;
        uint64_t in_count_ = 0;             // 相机送来的帧数（限速前统计）
        uint64_t view_count_ = 0;           // 窗口真正刷新次数（imshow 之后统计）
        double cam_fps_ = 0.0;
        double view_fps_ = 0.0;

        bool window_ready_ = false;
        bool window_broken_ = false;

        // 队列长度上限，纯粹为了控制内存（每帧 1080x1440x3 ≈ 4.6MB）
        static constexpr size_t kMaxPending = 6;
        static constexpr size_t kMaxDetCache = 24;
    };
}  // namespace dart_track
