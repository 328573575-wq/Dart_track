#include "dart_track/draw_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace dart_track
{
    namespace
    {
        // 在图上盖一层半透明黑底，保证白字在亮背景上也看得清
        void fill_backdrop(cv::Mat & frame, const cv::Rect & rect, double alpha)
        {
            const cv::Rect r = rect & cv::Rect(0, 0, frame.cols, frame.rows);
            if (r.width <= 0 || r.height <= 0) {
                return;
            }
            cv::Mat roi = frame(r);
            const cv::Mat shade(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
            cv::addWeighted(roi, 1.0 - alpha, shade, alpha, 0.0, roi);
        }

        // 检测点用"原图像素"坐标，画的时候要先乘缩放系数。
        // 同时过滤无效点：(0,0) 是识别节点没找到目标时的默认值。
        bool to_display_point(const geometry_msgs::msg::Point & p, const cv::Mat & frame,
                              double scale, cv::Point & out)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
                return false;
            }
            if (p.x == 0.0 && p.y == 0.0) {
                return false;
            }
            const double x = p.x * scale;
            const double y = p.y * scale;
            if (x < 0.0 || y < 0.0 || x >= frame.cols || y >= frame.rows) {
                return false;
            }
            out = cv::Point(cvRound(x), cvRound(y));
            return true;
        }

        // 字号、线宽按画面宽度等比缩放，换分辨率时观感不变
        double ui_scale(const cv::Mat & frame)
        {
            return std::clamp(static_cast<double>(frame.cols) / 720.0, 0.5, 2.0);
        }
    }  // namespace

    DrawRenderer::DrawRenderer(double scale)
    : scale_(scale > 0.0 ? scale : 0.5)
    {
    }

    cv::Mat DrawRenderer::render(const cv::Mat & bgr,
                                 const dart_track_interfaces::msg::Detections * det,
                                 double det_age_ms, double cam_fps, double view_fps,
                                 double exposure_us) const
    {
        cv::Mat frame;
        if (std::abs(scale_ - 1.0) < 1e-6) {
            bgr.copyTo(frame);
        } else {
            // INTER_LINEAR：缩小 2 倍时和 INTER_AREA 观感几乎一样，但快 3~5 倍
            cv::resize(bgr, frame, cv::Size(), scale_, scale_, cv::INTER_LINEAR);
        }
        if (frame.empty()) {
            return frame;
        }

        if (det != nullptr) {
            draw_detections(frame, *det);
        }
        draw_hud(frame, det, det_age_ms, cam_fps, view_fps, exposure_us);
        return frame;
    }

    void DrawRenderer::draw_detections(
        cv::Mat & frame, const dart_track_interfaces::msg::Detections & det) const
    {
        const double ui = ui_scale(frame);
        const int radius = std::max(2, cvRound(5 * ui));
        const int th = std::max(1, cvRound(2 * ui));
        const double fs = 0.55 * ui;

        // 引导灯：绿圈 + 编号（编号就是左上古顺时针的 0 1 2 3）
        for (size_t i = 0; i < det.light_centers.size(); ++i) {
            cv::Point p;
            if (!to_display_point(det.light_centers[i], frame, scale_, p)) {
                continue;
            }
            cv::circle(frame, p, radius, cv::Scalar(0, 255, 0), th);
            cv::putText(frame, std::to_string(i),
                        p + cv::Point(cvRound(8 * ui), cvRound(-8 * ui)),
                        cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(0, 255, 0), th);
        }

        // 飞镖：镖头红、镖尾蓝、中间黄线。有一头无效就整条不画
        cv::Point front;
        cv::Point back;
        if (!to_display_point(det.dart_front_mid, frame, scale_, front) ||
            !to_display_point(det.dart_back_mid, frame, scale_, back)) {
            return;
        }
        cv::line(frame, back, front, cv::Scalar(0, 255, 255), th);
        cv::circle(frame, front, radius, cv::Scalar(0, 0, 255), -1);
        cv::circle(frame, back, radius, cv::Scalar(255, 0, 0), -1);
        cv::putText(frame, "F", front + cv::Point(cvRound(9 * ui), cvRound(6 * ui)),
                    cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(0, 0, 255), th);
        cv::putText(frame, "B", back + cv::Point(cvRound(9 * ui), cvRound(6 * ui)),
                    cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(255, 0, 0), th);
    }

    // 左上角信息条。文字用英文：cv::putText 只认 ASCII
    void DrawRenderer::draw_hud(cv::Mat & frame,
                                const dart_track_interfaces::msg::Detections * det,
                                double det_age_ms, double cam_fps, double view_fps,
                                double exposure_us) const
    {
        const int font = cv::FONT_HERSHEY_SIMPLEX;
        const cv::Scalar kCyan(255, 255, 0);
        const cv::Scalar kGreen(0, 255, 0);
        const cv::Scalar kRed(0, 0, 255);
        const cv::Scalar kWhite(255, 255, 255);

        const double ui = ui_scale(frame);
        const int base = cvRound(30 * ui);
        const int line_h = cvRound(26 * ui);
        const double fs = 0.7 * ui;
        const int th = std::max(1, cvRound(2 * ui));

        fill_backdrop(frame, cv::Rect(6, 6, cvRound(620 * ui), 3 * line_h + cvRound(10 * ui)), 0.45);

        const int src_w = scale_ < 1.0 ? cvRound(frame.cols / scale_) : frame.cols;
        const int src_h = scale_ < 1.0 ? cvRound(frame.rows / scale_) : frame.rows;
        char line[160];
        if (exposure_us > 0.0) {
            std::snprintf(line, sizeof(line),
                          "draw %.1f fps | cam %.1f fps | %dx%d | exp %.0f us",
                          view_fps, cam_fps, src_w, src_h, exposure_us);
        } else {
            std::snprintf(line, sizeof(line),
                          "draw %.1f fps | cam %.1f fps | %dx%d | exp --",
                          view_fps, cam_fps, src_w, src_h);
        }
        cv::putText(frame, line, cv::Point(cvRound(16 * ui), base), font, fs, kCyan, th);

        if (det != nullptr) {
            std::snprintf(line, sizeof(line), "det old %.0f ms | lights %zu",
                          det_age_ms, det->light_centers.size());
            cv::putText(frame, line, cv::Point(cvRound(16 * ui), base + line_h),
                        font, fs, kGreen, th);

            std::snprintf(line, sizeof(line), "front %.0f,%.0f  back %.0f,%.0f",
                          det->dart_front_mid.x, det->dart_front_mid.y,
                          det->dart_back_mid.x, det->dart_back_mid.y);
            cv::putText(frame, line, cv::Point(cvRound(16 * ui), base + 2 * line_h),
                        font, 0.85 * fs, kWhite, th);
        } else {
            cv::putText(frame, "no detection this frame",
                        cv::Point(cvRound(16 * ui), base + line_h), font, fs, kRed, th);
        }
    }

}  // namespace dart_track