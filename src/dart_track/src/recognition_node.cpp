#include "dart_track/recognition_node.hpp"

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

        //创建接受图片的订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>
        (
            "/camera/image_raw",          
            rclcpp::SensorDataQoS(),    
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& img_msg){
                this->recognition_callback(img_msg);
            }
        );
        // 创建识别信息发布者
        detections_pub_ = this->create_publisher<Detections>("/dart_detections", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    }

    //识别接收到的图片内容的回调函数
    void RecognitionNode::recognition_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& img_msg
    )
    {
        cv::Mat frame = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

        if (frame.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "传入图像的是滚木");
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // 根据边界生成掩码
        auto in_range_hsv = [](const cv::Mat & src, int h_lo, int h_hi,
                               int s_lo, int s_hi, int v_lo, int v_hi) -> cv::Mat {
            cv::Mat mask;
            if (h_lo <= h_hi) {
                cv::inRange(src, cv::Scalar(h_lo, s_lo, v_lo),
                            cv::Scalar(h_hi, s_hi, v_hi), mask);
            } else {
                cv::Mat m1, m2;
                cv::inRange(src, cv::Scalar(h_lo, s_lo, v_lo),
                            cv::Scalar(180, s_hi, v_hi), m1);
                cv::inRange(src, cv::Scalar(0, s_lo, v_lo),
                            cv::Scalar(h_hi, s_hi, v_hi), m2);
                cv::bitwise_or(m1, m2, mask);
            }
            return mask;
        };
        cv::Mat light_mask = in_range_hsv(hsv, LIGHT_H_LO, LIGHT_H_HI,
                                          LIGHT_S_LO, LIGHT_S_HI, LIGHT_V_LO, LIGHT_V_HI);
        cv::Mat dart_mask = in_range_hsv(hsv, DART_H_LO, DART_H_HI,
                                         DART_S_LO, DART_S_HI, DART_V_LO, DART_V_HI);

        // 去除噪点 补充灯条缺损
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(light_mask, light_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(dart_mask, dart_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(dart_mask, dart_mask, cv::MORPH_CLOSE, kernel);

        // 找引导灯
        std::vector<std::vector<cv::Point>> light_contours;
        cv::findContours(light_mask, light_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point2d> light_centers; // 需要用到的四个引导灯中心像素坐标
        for (const auto & c : light_contours) {
            const cv::Moments m = cv::moments(c);
            if (m.m00 < LIGHT_MIN_AREA) continue;  // 面积筛选
            light_centers.emplace_back(m.m10 / m.m00, m.m01 / m.m00);
        }

        if (light_centers.size() != 4) {
            return;                     
        }

        std::sort(light_centers.begin(), light_centers.end(),
            [](const cv::Point2d & a, const cv::Point2d & b) {
                return a.y < b.y;
            });

        if (light_centers[1].x < light_centers[0].x) {
            std::swap(light_centers[0], light_centers[1]);
        }

        if (light_centers[3].x > light_centers[2].x) {
            std::swap(light_centers[2], light_centers[3]);
        }

        // 找飞镖灯条 TODO:等实物灯条出来了再细化
        std::vector<std::vector<cv::Point>> dart_contours;
        cv::findContours(dart_mask, dart_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Point2f vertexs[4];
        double best_area = 0.0;    // 多个候选时挑面积最大的

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
            }
        }

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

        // 分配一下镖头镖尾
        const cv::Point2d center = (front_mid + back_mid) / 2.0;
        cv::Point2d last_center = dart_last_center_;
        dart_last_center_ = center;


        const cv::Point2d axis = front_mid - back_mid;   // 当前帧尾到头向量
        const cv::Point2d move = center - last_center;   // 这一帧相对上一帧的位移
        const double move_len = cv::norm(move);
        const double axis_len = cv::norm(axis);

        if (last_center.x >= 0.0 && move_len > 2.0 && axis_len > 1e-6) {
            // 夹角余弦
            const double cos_angle = axis.dot(move) / (axis_len * move_len);
            if (cos_angle > 0.3 || cos_angle < -0.3) {
                if (cos_angle < 0.0) {  // 指向反了，交换
                    const cv::Point2d tmp = front_mid;
                    front_mid = back_mid;
                    back_mid = tmp;
                }
            }
        }

        detections_publish(img_msg->header, light_centers, front_mid, back_mid);
    }


    // 识别结果发布函数
    void RecognitionNode::detections_publish
    (
        const std_msgs::msg::Header & header,
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
    }
} // namespace dart_track


RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::RecognitionNode);