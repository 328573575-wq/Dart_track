#include "dart_track/draw_screen_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

namespace dart_track
{
    namespace
    {
        constexpr char kWindowName[] = "draw_screen_node";

        // 在图上盖一层半透明黑底，保证白字在亮背景上也看得清。
        // 直接画实心黑块会很丑，所以用 addWeighted 混一下
        void fill_backdrop(cv::Mat& frame, const cv::Rect& rect, double alpha)
        {
            const cv::Rect r = rect & cv::Rect(0, 0, frame.cols, frame.rows);
            if (r.width <= 0 || r.height <= 0) {
                return;
            }
            cv::Mat roi = frame(r);
            const cv::Mat shade(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
            cv::addWeighted(roi, 1.0 - alpha, shade, alpha, 0.0, roi);
        }

        // 检测点用的是"原图像素"坐标，而这里显示的是缩小后的图，所以要先乘 DRAW_SCALE。
        // 同时过滤无效点：(0,0) 是识别节点没找到目标时留下的默认值，NaN 和越界点也不能画。
        bool to_display_point(
            const geometry_msgs::msg::Point& p, const cv::Mat& frame,
            double scale, cv::Point& out)
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

        // 界面元素（字号、线宽）按画面宽度等比缩放，这样换 DRAW_SCALE 时观感不会变。
        double ui_scale(const cv::Mat& frame)
        {
            return std::clamp(static_cast<double>(frame.cols) / 720.0, 0.5, 2.0);
        }
    }  // namespace

    DrawScreenNode::DrawScreenNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("draw_screen_node", options)
    {
        // ========================参数读取===========================
        DRAW_MAX_FPS = this->declare_parameter<double>("draw_max_fps", 60.0);
        DRAW_SCALE = this->declare_parameter<double>("draw_scale", 0.5);
        DRAW_DET_MAX_AGE = this->declare_parameter<double>("draw_det_max_age", 0.3);
        SHOW_WINDOW = this->declare_parameter<bool>("draw_show_window", true);

        if (DRAW_MAX_FPS <= 0.0) {
            RCLCPP_WARN(get_logger(), "draw_max_fps <= 0，已按 60 处理");
            DRAW_MAX_FPS = 60.0;
        }
        if (DRAW_SCALE <= 0.0) {
            RCLCPP_WARN(get_logger(), "draw_scale <= 0，已按 0.5 处理");
            DRAW_SCALE = 0.5;
        }
        if (DRAW_DET_MAX_AGE < 0.0) {
            RCLCPP_WARN(get_logger(), "draw_det_max_age < 0，已按 0.3 处理");
            DRAW_DET_MAX_AGE = 0.3;
        }

        fps_window_start_ = std::chrono::steady_clock::now();

        // 订阅图像。QoS 必须和相机发布端一致（SensorDataQoS，best effort），
        // 不然两个对不上就一帧都收不到
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>
        (
            "/camera/image_raw",
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& img_msg){
                this->image_callback(img_msg);
            }
        );

        // 订阅识别结果。QoS 和 recognition 节点的发布端保持一致（reliable）
        detections_sub_ = this->create_subscription<Detections>
        (
            "/dart_detections",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const Detections::SharedPtr msg){
                this->detections_callback(msg);
            }
        );

        // 绘制放在单独的回调组里。
        // 同一个互斥组里的回调是排队的，所以如果 imshow 和收图在同一个组，
        // 一次 imshow 就会把收图堵住（图在 DDS 里等着，best effort 直接丢）。
        // 放进独立的组，多线程执行器才能让"收图"和"画图"真的并行。
        render_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / DRAW_MAX_FPS));
        render_timer_ = this->create_wall_timer(
            period, [this]() { this->render_tick(); }, render_group_);

        RCLCPP_INFO(get_logger(),
                    "绘图节点已启动：最新帧优先，最高 %.0f fps，缩放 %.2f，点最多滞后 %.0f ms",
                    DRAW_MAX_FPS, DRAW_SCALE, DRAW_DET_MAX_AGE * 1000.0);
    }

    // 识别结果回调：只记下最新一份，不画图
    void DrawScreenNode::detections_callback(const Detections::SharedPtr msg)
    {
        const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

        std::lock_guard<std::mutex> lock(mtx_);
        // 只留最新的一份。乱序到达的旧结果（比手里的还旧）直接丢。
        if (!latest_det_ || stamp_ns >= latest_det_stamp_ns_) {
            latest_det_ = msg;
            latest_det_stamp_ns_ = stamp_ns;
        }
    }

    // 图像回调：只做"缩放 + 存下最新一帧"，不绘制、不等待，尽快返回。
    // 这样即使识别很慢、窗口很慢，收图这一头也不会被拖住。
    void DrawScreenNode::image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg
    )
    {
        cv::Mat small;
        try {
            // toCvShare 只是"借用"消息里的像素，不复制；紧接着的 resize 把
            // 1440×1080×3 ≈ 4.67 MB 变成 720×540×3 ≈ 1.17 MB，后面只在小图上工作。
            const cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);

            if (cv_ptr->image.empty()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "传入图像是空的");
                return;
            }

            if (DRAW_SCALE < 1.0) {
                cv::resize(cv_ptr->image, small, cv::Size(), DRAW_SCALE, DRAW_SCALE,
                           cv::INTER_AREA);
            } else {
                // 不缩放时要把像素复制出来：消息的内存是只读的，而后面要往上画
                small = cv_ptr->image.clone();
            }
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "cv_bridge 转换失败: %s", e.what());
            return;
        } catch (const cv::Exception & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "缩放图像失败: %s", e.what());
            return;
        }

        std::lock_guard<std::mutex> lock(mtx_);
        latest_frame_ = std::move(small);
        latest_stamp_ns_ = rclcpp::Time(img_msg->header.stamp).nanoseconds();
        ++in_count_;
        refresh_fps_locked();
    }

    // 每 0.5 秒结算一次帧率。
    // in_count_ 统计"相机实际送进回调多少帧"，view_count_ 统计"窗口实际刷新多少次"。
    // 这两个数一对比，卡在收图端还是显示端就一目了然。
    void DrawScreenNode::refresh_fps_locked()
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - fps_window_start_).count();
        if (elapsed < 0.5) {
            return;
        }
        cam_fps_ = static_cast<double>(in_count_) / elapsed;
        view_fps_ = static_cast<double>(view_count_) / elapsed;
        in_count_ = 0;
        view_count_ = 0;
        fps_window_start_ = now;
    }

    // 定时器回调：取"最新一帧"，画，显示。整个节点只有这里会碰窗口。
    void DrawScreenNode::render_tick()
    {
        cv::Mat src;
        int64_t frame_stamp_ns = 0;
        Detections::SharedPtr det;
        int64_t det_stamp_ns = 0;
        double cam_fps = 0.0;
        double view_fps = 0.0;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            refresh_fps_locked();

            if (latest_frame_.empty()) {
                return;  // 还没收到过图
            }

            const auto now = std::chrono::steady_clock::now();
            const bool frame_changed = (latest_stamp_ns_ != last_rendered_stamp_ns_);
            const bool det_changed = (latest_det_stamp_ns_ != last_painted_det_stamp_ns_);
            // 没新帧也要每 0.5 秒重画一次 HUD：
            // 否则画面一断，窗口里的 cam/view 数字就永远停在断开前那一帧，看着像"卡死 0 fps"。
            const bool hud_due =
                std::chrono::duration<double>(now - last_paint_).count() > 0.5;
            if (!frame_changed && !det_changed && !hud_due) {
                return;
            }

            src = latest_frame_;                   // 浅拷贝：只复制 Mat 头，不复制像素
            frame_stamp_ns = latest_stamp_ns_;
            det = latest_det_;
            det_stamp_ns = latest_det_stamp_ns_;
            cam_fps = cam_fps_;
            view_fps = view_fps_;
            last_rendered_stamp_ns_ = frame_stamp_ns;
            last_painted_det_stamp_ns_ = latest_det_stamp_ns_;
            last_paint_ = now;
        }

        // 像素复制放到锁外做：后面要在图上画 HUD，必须有自己的副本，
        // 否则下一次重画会把 HUD 又叠一层上去。
        cv::Mat frame = src.clone();

        // 配对：识别结果通常比画面晚到一帧，所以这里算的是"点比画面旧多少"。
        // 差得太多就当作没有结果，避免把很久以前的点画到现在的画面上。
        double det_age_ms = -1.0;
        if (det) {
            const double age_s = static_cast<double>(frame_stamp_ns - det_stamp_ns) / 1e9;
            if (age_s >= 0.0 && age_s <= DRAW_DET_MAX_AGE) {
                det_age_ms = age_s * 1000.0;
            } else {
                det.reset();
            }
        }

        if (render_frame(frame, det.get(), det_age_ms, cam_fps, view_fps)) {
            std::lock_guard<std::mutex> lock(mtx_);
            ++view_count_;
        }
    }

    bool DrawScreenNode::render_frame(
        cv::Mat& frame, const Detections* det, double det_age_ms,
        double cam_fps, double view_fps)
    {
        if (det != nullptr) {
            draw_detections(frame, *det);
        }
        // 不管有没有识别结果，左上角都画信息条：
        // 否则你分不清"这一帧真的没东西"和"程序卡住了"
        draw_hud(frame, det, det_age_ms, cam_fps, view_fps);

        if (!SHOW_WINDOW || window_broken_) {
            return false;
        }

        try {
            if (!window_ready_) {
                // WINDOW_NORMAL 允许你拖窗口大小
                cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
                window_ready_ = true;
            }
            cv::imshow(kWindowName, frame);
            // waitKey 必须调用，否则 OpenCV 不会真正刷新窗口画面。
            // 参数 1 表示最多等 1ms，不能是 0，否则这条线程会停在这
            cv::waitKey(1);
            return true;
        } catch (const cv::Exception & e) {
            window_broken_ = true;
            RCLCPP_ERROR(get_logger(),
                         "建窗口失败（这台机器可能没有图形界面）: %s。后续不再尝试显示。",
                         e.what());
            return false;
        }
    }

    // 左上角信息条。
    // 文字用英文是因为 cv::putText 只认 ASCII，写中文会变成一串问号
    void DrawScreenNode::draw_hud(
        cv::Mat& frame, const Detections* det, double det_age_ms,
        double cam_fps, double view_fps)
    {
        const int font = cv::FONT_HERSHEY_SIMPLEX;
        const cv::Scalar kCyan(255, 255, 0);
        const cv::Scalar kGreen(0, 255, 0);
        const cv::Scalar kRed(0, 0, 255);
        const cv::Scalar kWhite(255, 255, 255);

        const double ui = ui_scale(frame);
        const int base = cvRound(30 * ui);       // 第一行文字的基线
        const int line_h = cvRound(26 * ui);     // 行距
        const double fs = 0.7 * ui;
        const int th = std::max(1, cvRound(2 * ui));

        fill_backdrop(frame, cv::Rect(6, 6, cvRound(620 * ui), 3 * line_h + cvRound(10 * ui)), 0.45);

        // 第 1 行：两个帧率 + 分辨率。
        //   view = 窗口每秒刷新次数，cam = 送进回调的帧数。
        //   卡的时候先看这两个数：cam 掉 = 收图端堵了；cam 稳而 view 掉 = 显示端慢
        const int src_w = DRAW_SCALE < 1.0 ? cvRound(frame.cols / DRAW_SCALE) : frame.cols;
        const int src_h = DRAW_SCALE < 1.0 ? cvRound(frame.rows / DRAW_SCALE) : frame.rows;
        char line[160];
        std::snprintf(line, sizeof(line), "view %.1f fps | cam %.1f fps | %dx%d",
                      view_fps, cam_fps, src_w, src_h);
        cv::putText(frame, line, cv::Point(cvRound(16 * ui), base), font, fs, kCyan, th);

        // 第 2 行：配对情况。
        //   det 后面是"这个点比画面旧了多少毫秒"，正常应该是一帧左右的量级。
        if (det != nullptr) {
            std::snprintf(line, sizeof(line), "det old %.0f ms | lights %zu",
                          det_age_ms, det->light_centers.size());
            cv::putText(frame, line, cv::Point(cvRound(16 * ui), base + line_h),
                        font, fs, kGreen, th);
        } else {
            cv::putText(frame, "no recent detection",
                        cv::Point(cvRound(16 * ui), base + line_h), font, fs, kRed, th);
        }

        // 第 3 行：镖头镖尾坐标（原图像素），只有有结果时才画
        if (det != nullptr) {
            std::snprintf(line, sizeof(line), "front %.0f,%.0f  back %.0f,%.0f",
                          det->dart_front_mid.x, det->dart_front_mid.y,
                          det->dart_back_mid.x, det->dart_back_mid.y);
            cv::putText(frame, line, cv::Point(cvRound(16 * ui), base + 2 * line_h),
                        font, 0.85 * fs, kWhite, th);
        }
    }

    // 把识别结果画到图上
    void DrawScreenNode::draw_detections(cv::Mat& frame, const Detections& det)
    {
        const double ui = ui_scale(frame);
        const int radius = std::max(2, cvRound(5 * ui));
        const int th = std::max(1, cvRound(2 * ui));
        const double fs = 0.55 * ui;

        // 引导灯：绿圈 + 编号
        for (size_t i = 0; i < det.light_centers.size(); ++i) {
            cv::Point p;
            if (!to_display_point(det.light_centers[i], frame, DRAW_SCALE, p)) {
                continue;  // 无效点直接跳过，不画
            }
            cv::circle(frame, p, radius, cv::Scalar(0, 255, 0), th);
            cv::putText(frame, std::to_string(i),
                        p + cv::Point(cvRound(8 * ui), cvRound(-8 * ui)),
                        cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(0, 255, 0), th);
        }

        // 飞镖：镖头红点、镖尾蓝点，中间连一条黄线，这样方向一眼能看出来。
        // 只要有一头无效，就整条都不画（半条线比没线更容易误判）
        cv::Point front;
        cv::Point back;
        if (!to_display_point(det.dart_front_mid, frame, DRAW_SCALE, front) ||
            !to_display_point(det.dart_back_mid, frame, DRAW_SCALE, back))
        {
            return;
        }

        cv::line(frame, back, front, cv::Scalar(0, 255, 255), th);
        cv::circle(frame, front, radius, cv::Scalar(0, 0, 255), -1);
        cv::circle(frame, back, radius, cv::Scalar(255, 0, 0), -1);
        cv::putText(frame, "F", front + cv::Point(cvRound(9 * ui), cvRound(6 * ui)),
                    cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(0, 0, 255), th);
        cv::putText(frame, "B", back + cv::Point(cvRound(9 * ui), cvRound(6 * ui)),
                    cv::FONT_HERSHEY_SIMPLEX, fs, cv::Scalar(255, 0, 0), th);

        // 终端里也打一行（每秒最多一次），方便你对着数字看
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "引导灯 %zu 个 | 镖头 (%.1f, %.1f) | 镖尾 (%.1f, %.1f)",
                             det.light_centers.size(),
                             det.dart_front_mid.x, det.dart_front_mid.y,
                             det.dart_back_mid.x, det.dart_back_mid.y);
    }

} // namespace dart_track

RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::DrawScreenNode);
