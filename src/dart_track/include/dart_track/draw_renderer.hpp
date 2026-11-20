#pragma once

#include <opencv2/opencv.hpp>

#include "dart_track_interfaces/msg/detections.hpp"

namespace dart_track
{

    // 纯绘图层：把识别结果画到图像上，不依赖 ROS 节点、不发消息、不弹窗口。
    //
    // 这样 recognition_node 只负责"看图 -> 识别 -> 发结果 + 发一张画好的图"，
    // 绘制细节全部收在这个类里，节点那边的代码保持干净。
    class DrawRenderer
    {
    public:
        explicit DrawRenderer(double scale = 0.5);

        // bgr      : 原始图像（和 detections 同一帧，像素坐标同尺度）
        // det      : 识别结果，可以为 nullptr（只画 HUD）
        // det_age  : 检测比这一帧旧多少毫秒，同一帧传 0
        // 返回画好的图（已按 scale 缩小）
        cv::Mat render(const cv::Mat & bgr, const dart_track_interfaces::msg::Detections * det,
                       double det_age_ms, double cam_fps, double view_fps,
                       double exposure_us) const;

    private:
        void draw_hud(cv::Mat & frame, const dart_track_interfaces::msg::Detections * det,
                      double det_age_ms, double cam_fps, double view_fps,
                      double exposure_us) const;
        void draw_detections(cv::Mat & frame,
                             const dart_track_interfaces::msg::Detections & det) const;

        double scale_ = 0.5;
    };

}  // namespace dart_track