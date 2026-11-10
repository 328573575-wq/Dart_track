#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "dart_track_interfaces/msg/detections.hpp"
using Detections = dart_track_interfaces::msg::Detections;

namespace dart_track
{


    // 纯显示节点：把 /dart_detections 里的点画到 /camera/image_raw 上，然后弹窗显示。
    //
    // 显示策略是"最新帧优先"，不再排队等识别结果：
    //   - 图像回调只做缩放，然后把"最新一帧"存下来，立刻返回，绝不在回调里等；
    //   - 定时器按 draw_max_fps 的节奏取最新一帧画图、显示；
    //   - 识别结果只保留最新一份，HUD 里写明"点比这一帧旧了多少毫秒"；
    //     旧超过 draw_det_max_age 就不画点，宁可没点，也不把过期的点画到新画面上。
    // 这样显示延迟恒定为"一帧"，识别慢或没结果时最多是"没点"，不会越积越卡。
    class DrawScreenNode : public rclcpp::Node
    {
    public:
        explicit DrawScreenNode(const rclcpp::NodeOptions & options);

    private:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);
        void detections_callback(const Detections::SharedPtr msg);
        void render_tick();   // 定时器回调：唯一的绘制入口

        bool render_frame(cv::Mat & frame, const Detections * det, double det_age_ms,
                          double cam_fps, double view_fps);
        void draw_detections(cv::Mat & frame, const Detections & det);
        void draw_hud(cv::Mat & frame, const Detections * det, double det_age_ms,
                      double cam_fps, double view_fps);
        void refresh_fps_locked();   // 调用者必须已经持有 mtx_

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<Detections>::SharedPtr detections_sub_;
        rclcpp::TimerBase::SharedPtr render_timer_;
        rclcpp::CallbackGroup::SharedPtr render_group_;   // 让绘制和收图不在同一个组里排队

        std::mutex mtx_;                       // 保护下面这几个"最新值"
        cv::Mat latest_frame_;                 // 最新一帧（已经按 DRAW_SCALE 缩放过）
        int64_t latest_stamp_ns_ = 0;          // 这帧的时间戳
        int64_t last_rendered_stamp_ns_ = -1;  // 上一次真正画出去的是哪一帧
        int64_t last_painted_det_stamp_ns_ = -1;  // 上一次画出去时用的检测结果
        std::chrono::steady_clock::time_point last_paint_{};  // 上一次重画的时刻
        Detections::SharedPtr latest_det_;     // 最新一份识别结果
        int64_t latest_det_stamp_ns_ = 0;      // 它的时间戳

        double DRAW_MAX_FPS = 60.0;         // 绘制频率上限
        double DRAW_SCALE = 0.5;            // 显示缩放
        double DRAW_DET_MAX_AGE = 0.3;      // 识别结果比当前帧旧超过这么多秒就不画点
        bool SHOW_WINDOW = true;            // 无图形界面时可关掉

        std::chrono::steady_clock::time_point fps_window_start_{};
        uint64_t in_count_ = 0;             // 图像回调收到多少帧
        uint64_t view_count_ = 0;           // 窗口实际刷新多少次
        double cam_fps_ = 0.0;
        double view_fps_ = 0.0;

        bool window_ready_ = false;
        bool window_broken_ = false;
    };
}  // namespace dart_track
