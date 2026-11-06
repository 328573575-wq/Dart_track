#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include "dart_track_interfaces/msg/detections.hpp"
using Detections = dart_track_interfaces::msg::Detections;
namespace dart_track 
{

    class RecognitionNode : public rclcpp::Node 
    {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<Detections>::SharedPtr detections_pub_;

        cv::Point2d dart_last_center_ = cv::Point2d(-1.0, -1.0);
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


    private:
        void recognition_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);
        void detections_publish
        (
            const std_msgs::msg::Header & header,
            std::vector<cv::Point2d>& light_centers, 
            cv::Point2d& dart_front_mid,
            cv::Point2d& dart_back_mid);
    public:
        explicit RecognitionNode(const rclcpp::NodeOptions & options);
    };

}