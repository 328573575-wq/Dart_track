#include "dart_track/recognition_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <utility>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "dart_track_interfaces/msg/detections.hpp"
using Detections = dart_track_interfaces::msg::Detections;

namespace dart_track 
{

    RecognitionNode::RecognitionNode(const rclcpp::NodeOptions & options) 
    : Node("recognition_node", options)
    {
        // ========================参数读取===========================
        // 引导灯 HSV 阈值
        LIGHT_H_LO = this->declare_parameter<int>("light_h_lo", 40);
        LIGHT_H_HI = this->declare_parameter<int>("light_h_hi", 85);
        LIGHT_S_LO = this->declare_parameter<int>("light_s_lo", 80);
        LIGHT_S_HI = this->declare_parameter<int>("light_s_hi", 255);
        LIGHT_V_LO = this->declare_parameter<int>("light_v_lo", 100);
        LIGHT_V_HI = this->declare_parameter<int>("light_v_hi", 255);

        // 飞镖灯条 HSV 阈值
        DART_H_LO = this->declare_parameter<int>("dart_h_lo", 40);
        DART_H_HI = this->declare_parameter<int>("dart_h_hi", 85);
        DART_S_LO = this->declare_parameter<int>("dart_s_lo", 80);
        DART_S_HI = this->declare_parameter<int>("dart_s_hi", 255);
        DART_V_LO = this->declare_parameter<int>("dart_v_lo", 100);
        DART_V_HI = this->declare_parameter<int>("dart_v_hi", 255);

        // 引导灯、灯条最小面积
        LIGHT_MIN_AREA = this->declare_parameter<double>("light_min_area", 30.0);
        DART_MIN_AREA = this->declare_parameter<double>("dart_min_area", 60.0);
        // 灯条长宽比
        LENGTH_WIDTH_RATIO = this->declare_parameter<double>("length_width_ratio", 1.8);

        // 引导灯筛选（现实环境里反光/杂光很多，光靠面积分不开）
        LIGHT_MAX_ASPECT = this->declare_parameter<double>("light_max_aspect", 6.0);
        LIGHT_V_AUTO_RATIO =
            this->declare_parameter<double>("light_v_auto_ratio", 0.8);

        // 话题参数化：两条流水线各订阅自己的图像、各发自己的检测
        const std::string image_topic =
            this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        const std::string detections_topic =
            this->declare_parameter<std::string>("detections_topic", "/dart_detections");

        // 图像源：默认订阅压缩流（JPEG），带宽比 raw 小一个量级。
        // 需要 raw 时把 use_compressed 设成 false。
        const bool use_compressed = this->declare_parameter<bool>("use_compressed", true);
        const std::string compressed_topic =
            this->declare_parameter<std::string>("compressed_topic", "/camera/compressed");
        if (use_compressed) {
            compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                compressed_topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg) {
                    this->compressed_callback(msg);
                });
            RCLCPP_INFO(get_logger(), "订阅压缩图像: %s", compressed_topic.c_str());
        } else {
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic, rclcpp::SensorDataQoS(),
                [this](const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
                    this->recognition_callback(img_msg);
                });
            RCLCPP_INFO(get_logger(), "订阅原始图像: %s", image_topic.c_str());
        }
        // 创建识别信息发布者
        detections_pub_ = this->create_publisher<Detections>(detections_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        // 绘图：识别和绘制在同一个回调里做完，少一次跨节点传递
        const std::string draw_topic =
            this->declare_parameter<std::string>("draw_topic", "/draw_screen/image");
        DRAW_SCALE = this->declare_parameter<double>("draw_scale", 0.5);
        DRAW_ENABLE = this->declare_parameter<bool>("draw_enable", true);
        DRAW_FPS = this->declare_parameter<double>("draw_fps", 30.0);
        if (DRAW_SCALE <= 0.0) {
            RCLCPP_WARN(get_logger(), "draw_scale <= 0，已按 0.5 处理");
            DRAW_SCALE = 0.5;
        }
        // 曝光值由相机节点 1Hz 广播，仅用于在图上显示
        const std::string exposure_topic =
            this->declare_parameter<std::string>("exposure_topic", "/camera/exposure");
        exposure_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            exposure_topic, rclcpp::QoS(10),
            [this](const std_msgs::msg::Float64::ConstSharedPtr msg) {
                exposure_us_ = msg->data;
            });

        renderer_ = std::make_unique<DrawRenderer>(DRAW_SCALE);
        draw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            draw_topic, rclcpp::SensorDataQoS());
        fps_window_start_ = std::chrono::steady_clock::now();

    }

    namespace
    {
        // 统一编号：先按 x 分左右两组，再各自按 y 定上下。
        //   左半（x 小的两个）：y 小的在前 -> 0=左上, 1=左下
        //   右半（x 大的两个）：y 小的在后 -> 2=右下, 3=右上
        // 结果是 0=左上，然后逆时针 1 2 3。换任何视角编号规则都一样。
        void order_lights_x_then_y(std::vector<cv::Point2d> & pts)
        {
            if (pts.size() < 3) {
                return;
            }

            // 1) 按 x 从小到大：前面是左边那几个，后面是右边那几个
            std::sort(pts.begin(), pts.end(),
                      [](const cv::Point2d & a, const cv::Point2d & b) { return a.x < b.x; });

            const size_t half = pts.size() / 2;

            // 2) 左半：y 小的排前（上 -> 下）
            std::sort(pts.begin(), pts.begin() + static_cast<long>(half),
                      [](const cv::Point2d & a, const cv::Point2d & b) { return a.y < b.y; });

            // 3) 右半：y 小的排后（下 -> 上）
            std::sort(pts.begin() + static_cast<long>(half), pts.end(),
                      [](const cv::Point2d & a, const cv::Point2d & b) { return a.y > b.y; });
        }

    }  // namespace

    //识别接收到的图片内容的回调函数
    void RecognitionNode::recognition_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg
    )
    {
        // 用 toCvShare：不复制整帧（1440x1080 一帧 4.6MB，复制一次就够呛）。
        // 下游只读不写，绘制是在缩小后的副本上做的。
        const cv::Mat frame = cv_bridge::toCvShare(
                            img_msg, sensor_msgs::image_encodings::BGR8)->image;
        process_frame(frame, img_msg->header);
    }

    // 压缩流入口：解码成 BGR 再走同一条处理链路
    void RecognitionNode::compressed_callback(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg
    )
    {
        if (msg->data.empty()) {
            return;
        }
        const cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8U,
                          const_cast<unsigned char*>(msg->data.data()));
        const cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);
        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "JPEG 解码失败");
            return;
        }
        process_frame(frame, msg->header);
    }

    void RecognitionNode::process_frame(const cv::Mat& frame,
                                        const std_msgs::msg::Header& header)
    {
        ++cam_count_;
        refresh_fps();

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "传入图像的是滚木");
            return;
        }

        // 复用缓冲：1440x1080 的 HSV/掩码各 1.5~4.6MB，每帧重新分配太浪费
        cv::cvtColor(frame, hsv_buf_, cv::COLOR_BGR2HSV);
        const cv::Mat & hsv = hsv_buf_;

        // ---------- 引导灯掩码（单次 inRange）----------
        // 亮度阈值用"画面里最亮的像素"做基准取相对值：灯的亮芯是饱和的。
        // V 通道直接取 hsv 的第 3 通道视图，不做 extractChannel（省一次 1.5MB 分配）。
        // vmax 取自整幅图（不套掩码）：minMaxLoc 带 mask 会慢好几倍，
        // 而这种场景里最亮的像素基本就是灯本身。
        const cv::Mat v_view(hsv.size(), CV_8U, hsv.data + 2, hsv.step);
        double vmax = 0.0;
        cv::minMaxLoc(v_view, nullptr, &vmax);
        const int v_floor = std::min(
            LIGHT_V_HI,
            std::max(LIGHT_V_LO, static_cast<int>(LIGHT_V_AUTO_RATIO * vmax)));

        cv::Mat & light_mask = light_mask_buf_;
        cv::Mat & dart_mask = dart_mask_buf_;
        cv::inRange(hsv, cv::Scalar(LIGHT_H_LO, LIGHT_S_LO, v_floor),
                    cv::Scalar(LIGHT_H_HI, LIGHT_S_HI, LIGHT_V_HI), light_mask);
        cv::inRange(hsv, cv::Scalar(DART_H_LO, DART_S_LO, DART_V_LO),
                    cv::Scalar(DART_H_HI, DART_S_HI, DART_V_HI), dart_mask);

        // 去除噪点 补充灯条缺损
        static const cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        // 引导灯只做闭运算：开运算会把远处只有十几个像素的灯整个抹掉
        cv::morphologyEx(light_mask, light_mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(dart_mask, dart_mask, cv::MORPH_OPEN, kernel);

        // 找引导灯
        std::vector<std::vector<cv::Point>> light_contours;
        cv::findContours(light_mask, light_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // ---------- 引导灯候选 ----------
        // 现在测试环境干净（没有反光、和仿真差不多），不再做"从一堆候选里
        // 组合搜索最优四边形"那一套 —— 那一步是 C(n,4) 的，最费时间。
        // 只保留面积/形状两道廉价筛选，然后按面积从大到小取前 4 个。
        std::vector<std::pair<double, cv::Point2d>> candidates;  // (面积, 中心)
        for (const auto & c : light_contours) {
            const cv::Moments m = cv::moments(c);
            if (m.m00 < LIGHT_MIN_AREA) continue;  // 面积筛选

            // 形状筛选：灯是圆的，细长的多半是反光/器件边缘
            const cv::RotatedRect rr = cv::minAreaRect(c);
            const double long_side = std::max(rr.size.width, rr.size.height);
            const double short_side = std::max(1.0f, std::min(rr.size.width, rr.size.height));
            if (long_side / short_side > LIGHT_MAX_ASPECT) continue;

            candidates.emplace_back(m.m00, cv::Point2d(m.m10 / m.m00, m.m01 / m.m00));
        }

        // 面积大的排前面：真灯比零碎噪点大得多
        std::sort(candidates.begin(), candidates.end(),
                  [](const auto & a, const auto & b) { return a.first > b.first; });

        std::vector<cv::Point2d> light_centers;
        for (size_t i = 0; i < candidates.size() && light_centers.size() < 4; ++i) {
            light_centers.push_back(candidates[i].second);
        }

        if (light_centers.size() < 4) {
            // 没凑齐 4 个也照样发：draw 会画出找到的那几个灯 + HUD，
            // 一个灯都没有时也会发一张只有 HUD 的图，
            // 这样"没识别到"和"节点卡死了"能一眼分开。
            // solvepose 会忽略这种消息（它要求恰好 4 个才能 PnP）。
            if (!light_centers.empty()) {
                order_lights_x_then_y(light_centers);
            }
            cv::Point2d invalid(0.0, 0.0);
            detections_publish(header, frame, light_centers, invalid, invalid);
            return;
        }

        // 编号统一成"左上角 = 0，然后逆时针"，和是哪台相机无关
        order_lights_x_then_y(light_centers);

        // 找飞镖灯条 TODO:等实物灯条出来了再细化
        std::vector<std::vector<cv::Point>> dart_contours;
        cv::findContours(dart_mask, dart_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Point2f vertexs[4];
        double best_area = 0.0;    // 多个候选时挑面积最大的
        bool dart_found = false;   // 这一帧到底有没有找到灯条

        for (const auto & c : dart_contours) {
            const double area = cv::contourArea(c);
            if (area < DART_MIN_AREA) continue;  // 面积筛选

            const cv::RotatedRect rr = cv::minAreaRect(c);
            const double long_side = std::max(rr.size.width, rr.size.height);
            const double short_side = std::max(1.0f, std::min(rr.size.width, rr.size.height));
            if (long_side / short_side < LENGTH_WIDTH_RATIO) continue;  // 长宽比筛选

            if (area > best_area) {  // 只有“目前最大的”才写进 vertexs
                best_area = area;
                rr.points(vertexs);
                dart_found = true;
            }
        }

        // 这一帧没有灯条：不要拿没初始化的 vertexs 去算坐标。
        // 仍然发布消息（引导灯还有用，solvepose 靠它算 TF），
        // 但把镖头/镖尾标成 (0,0) 表示"本帧无效"，下游看到这个值会跳过。
        if (!dart_found) {
            ++dart_miss_count_;
            if (dart_miss_count_ >= 10) {
                // 丢太久就别拿很久以前的位置当参考了
                dart_has_last_center_ = false;
                dart_has_last_axis_ = false;
            }
            cv::Point2d invalid(0.0, 0.0);
            detections_publish(header, frame, light_centers, invalid, invalid);
            return;
        }
        dart_miss_count_ = 0;

        double dis_1 = cv::norm(vertexs[0] - vertexs[1]);
        double dis_2 = cv::norm(vertexs[1] - vertexs[2]);
        cv::Point2d front_mid, back_mid; // 需要用到的飞镖前后中点
        if (dis_1 < dis_2) {
            // 如果 dis_1 短，说明窄边是 (v0, v1) 和 (v2, v3)
            front_mid = (vertexs[0] + vertexs[1]) / 2.0f;
            back_mid = (vertexs[2] + vertexs[3]) / 2.0f;
            } else {
            // 如果 dis_2 短，说明窄边是 (v1, v2) 和 (v3, v0)
            front_mid = (vertexs[1] + vertexs[2]) / 2.0f;
            back_mid = (vertexs[3] + vertexs[0]) / 2.0f;
        }

        // 分配一下镖头镖尾。
        // 第一步：先和上一帧的朝向对齐（刚体不会一帧转 180°）。
        const cv::Point2d center = (front_mid + back_mid) / 2.0;
        cv::Point2d axis = front_mid - back_mid;   // 当前帧 尾->头 向量
        double axis_len = cv::norm(axis);

        // 目标"瞬移"（仿真一次飞行结束回到起点、或丢失后重新捕获）：
        // 两帧之间不可能真的移动这么多，上一帧的方向和位置都不能用。
        // 这一帧不发布镖头镖尾（发 0,0），但记住中心，让下一帧用运动方向重新定向。
        const double teleport_px = 0.25 * static_cast<double>(frame.cols);
        if (dart_has_last_center_) {
            const cv::Point2d jump = center - dart_last_center_;
            if (cv::norm(jump) > teleport_px) {
                dart_has_last_axis_ = false;
                dart_last_center_ = center;
                cv::Point2d invalid(0.0, 0.0);
                detections_publish(header, frame, light_centers, invalid, invalid);
                return;
            }
        }

        if (dart_has_last_axis_ && axis_len > 1e-6) {
            if (axis.dot(dart_last_axis_) < 0.0) {
                std::swap(front_mid, back_mid);
                axis = -axis;
            }
        }

        // 第二步：再用运动方向纠正一次。这一步主要给"第一帧"和"丢失后重新出现"用，
        // 有了上一步之后它很少再触发。
        if (dart_has_last_center_) {
            const cv::Point2d move = center - dart_last_center_;
            const double move_len = cv::norm(move);
            if (move_len > 2.0 && axis_len > 1e-6) {
                const double cos_angle = axis.dot(move) / (axis_len * move_len);
                if (cos_angle < -0.3) {  // 指向反了，交换
                    std::swap(front_mid, back_mid);
                    axis = -axis;
                }
            }
        }

        // 只有真正找到灯条的帧才更新历史，坏帧不会再把方向带偏
        dart_last_center_ = center;
        dart_has_last_center_ = true;
        axis_len = cv::norm(axis);
        if (axis_len > 1e-6) {
            dart_last_axis_ = axis / axis_len;
            dart_has_last_axis_ = true;
        }

        detections_publish(header, frame, light_centers, front_mid, back_mid);
    }


    // 识别结果发布函数
    void RecognitionNode::detections_publish
    (
        const std_msgs::msg::Header & header,
        const cv::Mat & frame,
        std::vector<cv::Point2d>& light_centers,
        cv::Point2d& dart_front_mid,
        cv::Point2d& dart_back_mid)
    {
        Detections dart_detections;
        dart_detections.header.stamp = header.stamp;
        dart_detections.header.frame_id = "pixel";

        geometry_msgs::msg::Point front_pt, back_pt;
        front_pt.x = dart_front_mid.x;
        front_pt.y = dart_front_mid.y;
        back_pt.x = dart_back_mid.x;
        back_pt.y = dart_back_mid.y;
        dart_detections.dart_front_mid = front_pt;
        dart_detections.dart_back_mid = back_pt;

        for (auto pt : light_centers)
        {
            geometry_msgs::msg::Point center;
            center.x = pt.x;
            center.y = pt.y;
            dart_detections.light_centers.push_back(center);
        }
        detections_pub_->publish(dart_detections);

        // 同一帧顺手画出来发走（识别 + 绘图在同一个回调里，没有额外延迟）
        draw_publish(header, frame, dart_detections);
    }

    // 把"这一帧 + 这一帧的识别结果"画成一张图发出去
    void RecognitionNode::draw_publish(const std_msgs::msg::Header & header,
                                       const cv::Mat & frame,
                                       const Detections & det)
    {
        if (!DRAW_ENABLE || !renderer_ || frame.empty()) {
            return;
        }

        // 绘图只给人看，按 draw_fps 限频：不跟满 160 帧，
        // 省掉 resize + 1MB 图像序列化的开销
        const int64_t now_ns = get_clock()->now().nanoseconds();
        if (DRAW_FPS > 0.0) {
            const int64_t interval_ns = static_cast<int64_t>(1e9 / DRAW_FPS);
            if (now_ns - last_draw_ns_ < interval_ns) {
                return;
            }
        }
        last_draw_ns_ = now_ns;

        const cv::Mat drawn = renderer_->render(frame, &det, 0.0, cam_fps_, draw_fps_, exposure_us_);
        if (drawn.empty()) {
            return;
        }

        ++draw_count_;

        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = header.stamp;
        msg->header.frame_id = "draw";
        msg->height = static_cast<uint32_t>(drawn.rows);
        msg->width = static_cast<uint32_t>(drawn.cols);
        msg->encoding = "bgr8";
        msg->is_bigendian = 0;
        msg->step = static_cast<uint32_t>(drawn.cols) * 3;
        msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
        if (drawn.isContinuous()) {
            std::memcpy(msg->data.data(), drawn.data, msg->data.size());
        } else {
            for (int row = 0; row < drawn.rows; ++row) {
                std::memcpy(msg->data.data() + static_cast<size_t>(row) * msg->step,
                            drawn.ptr(row), msg->step);
            }
        }
        draw_pub_->publish(std::move(msg));
    }

    // 每 0.5 秒刷新一次帧率（画到 HUD 上）
    void RecognitionNode::refresh_fps()
    {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - fps_window_start_).count();
        if (elapsed < 0.5) {
            return;
        }
        cam_fps_ = static_cast<double>(cam_count_) / elapsed;
        draw_fps_ = static_cast<double>(draw_count_) / elapsed;
        cam_count_ = 0;
        draw_count_ = 0;
        fps_window_start_ = now;
    }

} // namespace dart_track


RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::RecognitionNode);