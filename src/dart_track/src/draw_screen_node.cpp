#include "dart_track/draw_screen_node.hpp"

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
    }  // namespace

    DrawScreenNode::DrawScreenNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("draw_screen_node", options)
    {
        // ========================参数读取===========================
        DRAW_MAX_FPS = this->declare_parameter<double>("draw_max_fps", 60.0);
        DRAW_SCALE = this->declare_parameter<double>("draw_scale", 0.5);
        DRAW_PAIR_TIMEOUT = this->declare_parameter<double>("draw_pair_timeout", 0.2);
        SHOW_WINDOW = this->declare_parameter<bool>("draw_show_window", true);

        if (DRAW_MAX_FPS <= 0.0) {
            RCLCPP_WARN(get_logger(), "draw_max_fps <= 0，已按 60 处理");
            DRAW_MAX_FPS = 60.0;
        }
        if (DRAW_PAIR_TIMEOUT <= 0.0) {
            DRAW_PAIR_TIMEOUT = 0.2;
        }

        // 先往回退 1 秒，保证第一帧进来时不会被限速挡掉
        last_draw_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
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

        RCLCPP_INFO(get_logger(),
                    "绘图节点已启动：订阅 /camera/image_raw 和 /dart_detections，"
                    "最高 %.0f fps，缩放 %.2f，配对超时 %.0f ms",
                    DRAW_MAX_FPS, DRAW_SCALE, DRAW_PAIR_TIMEOUT * 1000.0);
    }

    // 识别结果回调：不画图，只把结果按时间戳存起来，等它对应的那一帧图来配对
    void DrawScreenNode::detections_callback(const Detections::SharedPtr msg)
    {
        const int64_t key = rclcpp::Time(msg->header.stamp).nanoseconds();
        det_cache_[key] = msg;

        // 只留最近的一小段，避免长时间运行内存一直涨
        while (det_cache_.size() > kMaxDetCache) {
            det_cache_.erase(det_cache_.begin());
        }

        flush_pending();
    }

    // 每 0.5 秒结算一次帧率。
    // in_count_ 在限速之前就加，统计的是"相机实际送来了多少帧"；
    // view_count_ 只在真正 imshow 之后加，统计的是"窗口实际刷新了多少次"。
    // 这两个数一对比，卡在哪一头就一目了然。
    void DrawScreenNode::refresh_fps()
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

    // 图像回调：只负责把图转成 Mat、塞进等待队列，真正画图交给 flush_pending()
    void DrawScreenNode::image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg
    )
    {
        // 先记一笔输入帧。注意这一句在限速判断之前，
        // 否则测出来的是"窗口帧率"而不是"相机帧率"
        ++in_count_;
        refresh_fps();

        // 转成 OpenCV 的 Mat。用 toCvCopy 而不是 toCvShare：
        // 共享过来的内存是只读的，而我们要在图上画线
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (const cv_bridge::Exception & e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "cv_bridge 转换失败: %s", e.what());
            return;
        }

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "传入图像是空的");
            return;
        }

        PendingFrame item;
        item.stamp_ns = rclcpp::Time(img_msg->header.stamp).nanoseconds();
        item.arrived = std::chrono::steady_clock::now();
        item.frame = std::move(frame);
        pending_.push_back(std::move(item));

        // 队列满了就丢最老的，防止内存无限涨（正常情况下识别很快，队列只有 1~2 帧）
        while (pending_.size() > kMaxPending) {
            pending_.pop_front();
        }

        flush_pending();
    }

    // 把队列里"能配对的"帧依次显示出去。
    // 配对成功 = 有和这一帧时间戳完全相同的识别结果，此时 age 天然是 0。
    void DrawScreenNode::flush_pending()
    {
        while (!pending_.empty())
        {
            PendingFrame & item = pending_.front();

            Detections::SharedPtr det;
            const auto it = det_cache_.find(item.stamp_ns);
            if (it != det_cache_.end()) {
                det = it->second;
            } else {
                // 还没有这一帧的识别结果。识别天生比取流慢一点，
                // 所以先别急，等一会儿（默认 200ms）
                const double waited = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - item.arrived).count();
                if (waited < DRAW_PAIR_TIMEOUT) {
                    return;  // 后面还有新帧在排着，等识别结果到了再一起处理
                }
                // 等太久了，说明这一帧的识别结果不会来了（识别慢/漏了），
                // 那就直接显示，只是不带点
            }

            // 限速：太密集就跳过显示，但必须把队头腾掉，否则会积压
            const auto now = std::chrono::steady_clock::now();
            const double since_last = std::chrono::duration<double>(now - last_draw_time_).count();
            if (since_last >= 0.9 / DRAW_MAX_FPS) {
                last_draw_time_ = now;
                // 配对上了：两个时间戳是同一个，差值就是 0
                const double age_ms = det ? 0.0 : -1.0;
                render_frame(item.frame, det.get(), age_ms);
            }

            // 这一帧处理完了，放掉它（4.6MB 内存）
            pending_.pop_front();
        }
    }

    void DrawScreenNode::render_frame(cv::Mat& frame, const Detections* det, double age_ms)
    {
        if (det != nullptr) {
            draw_detections(frame, *det);
        }
        // 不管有没有识别结果，左上角都画信息条：
        // 否则你分不清"这一帧真的没东西"和"程序卡住了"
        draw_hud(frame, det, age_ms);

        if (!SHOW_WINDOW || window_broken_) {
            return;
        }

        cv::Mat show = frame;
        if (DRAW_SCALE > 0.0 && DRAW_SCALE < 1.0) {
            cv::resize(frame, show, cv::Size(), DRAW_SCALE, DRAW_SCALE);
        }

        try {
            if (!window_ready_) {
                // WINDOW_NORMAL 允许你拖窗口大小
                cv::namedWindow("draw_screen_node", cv::WINDOW_NORMAL);
                window_ready_ = true;
            }
            cv::imshow("draw_screen_node", show);
            // waitKey 必须调用，否则 OpenCV 不会真正刷新窗口画面。
            // 参数 1 表示最多等 1ms，不能是 0，否则整个回调会停在这
            cv::waitKey(1);
            ++view_count_;
        } catch (const cv::Exception & e) {
            window_broken_ = true;
            RCLCPP_ERROR(get_logger(),
                         "建窗口失败（这台机器可能没有图形界面）: %s。后续不再尝试显示。",
                         e.what());
        }
    }

    // 左上角信息条。
    // 文字用英文是因为 cv::putText 只认 ASCII，写中文会变成一串问号
    void DrawScreenNode::draw_hud(cv::Mat& frame, const Detections* det, double age_ms)
    {
        const int font = cv::FONT_HERSHEY_SIMPLEX;
        const cv::Scalar kCyan(255, 255, 0);
        const cv::Scalar kGreen(0, 255, 0);
        const cv::Scalar kRed(0, 0, 255);
        const cv::Scalar kWhite(255, 255, 255);

        const int base = 44;          // 第一行文字的基线
        const int line_h = 40;        // 行距
        fill_backdrop(frame, cv::Rect(6, 6, 640, 3 * line_h + 6), 0.45);

        // 第 1 行：两个帧率 + 分辨率。
        //   view = 窗口每秒刷新次数，cam = 相机每秒送来多少帧。
        //   卡的时候先看这两个数：cam 掉 = 相机侧问题；cam 稳而 view 掉 = 显示侧问题
        char line[160];
        std::snprintf(line, sizeof(line), "view %.1f fps | cam %.1f fps | %dx%d",
                      view_fps_, cam_fps_, frame.cols, frame.rows);
        cv::putText(frame, line, cv::Point(20, base), font, 0.8, kCyan, 2);

        // 第 2 行：配对情况。
        // 配上了说明这张图和识别结果来自同一帧，点就是准的；
        // 没配上说明这一帧的识别结果没等到，画面里不会有点
        if (det != nullptr) {
            std::snprintf(line, sizeof(line), "age %.0f ms | lights %zu",
                          age_ms, det->light_centers.size());
            cv::putText(frame, line, cv::Point(20, base + line_h), font, 0.8, kGreen, 2);
        } else {
            cv::putText(frame, "no detection for this frame (pair timeout)",
                        cv::Point(20, base + line_h), font, 0.7, kRed, 2);
        }

        // 第 3 行：镖头镖尾坐标，只有配上对时才有意义
        if (det != nullptr) {
            std::snprintf(line, sizeof(line), "front %.0f,%.0f  back %.0f,%.0f",
                          det->dart_front_mid.x, det->dart_front_mid.y,
                          det->dart_back_mid.x, det->dart_back_mid.y);
            cv::putText(frame, line, cv::Point(20, base + 2 * line_h), font, 0.7, kWhite, 2);
        }
    }

    // 把识别结果画到图上
    void DrawScreenNode::draw_detections(cv::Mat& frame, const Detections& det)
    {
        // 引导灯：绿圈 + 编号
        for (size_t i = 0; i < det.light_centers.size(); ++i) {
            const cv::Point p(cvRound(det.light_centers[i].x),
                              cvRound(det.light_centers[i].y));
            cv::circle(frame, p, 6, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, std::to_string(i), p + cv::Point(8, -8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }

        // 飞镖：镖头红点、镖尾蓝点，中间连一条黄线，这样方向一眼能看出来
        const cv::Point front(cvRound(det.dart_front_mid.x), cvRound(det.dart_front_mid.y));
        const cv::Point back(cvRound(det.dart_back_mid.x), cvRound(det.dart_back_mid.y));

        cv::line(frame, back, front, cv::Scalar(0, 255, 255), 2);
        cv::circle(frame, front, 6, cv::Scalar(0, 0, 255), -1);
        cv::circle(frame, back, 6, cv::Scalar(255, 0, 0), -1);
        cv::putText(frame, "F", front + cv::Point(9, 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
        cv::putText(frame, "B", back + cv::Point(9, 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);

        // 终端里也打一行（每秒最多一次），方便你对着数字看
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "cam %.1f fps | 引导灯 %zu 个 | 镖头 (%.1f, %.1f) | 镖尾 (%.1f, %.1f)",
                             cam_fps_,
                             det.light_centers.size(),
                             det.dart_front_mid.x, det.dart_front_mid.y,
                             det.dart_back_mid.x, det.dart_back_mid.y);
    }

} // namespace dart_track

RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::DrawScreenNode);
