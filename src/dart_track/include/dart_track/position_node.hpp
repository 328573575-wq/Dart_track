#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include "dart_track_interfaces/msg/detections.hpp"
#include "dart_point_interfaces/msg/dart_point.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

using Detections = dart_track_interfaces::msg::Detections;
using DartPoint = dart_point_interfaces::msg::DartPoint;

namespace dart_track 
{

    class PositionNode : public rclcpp::Node
    {
    private:
        //内参和畸变
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        //TF监听
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

        //Detection_msg订阅器
        rclcpp::Subscription<Detections>::SharedPtr detections_sub_;

        rclcpp::Publisher<DartPoint>::SharedPtr real_point_pub_;
    private:
        void position_callback(const Detections::ConstSharedPtr& detections_msg);
        void dart_point_publish
        (
            const std_msgs::msg::Header & header,
            cv::Point3d& dart_real_front,
            cv::Point3d& dart_real_back,
            double& angle);

    public:
        explicit PositionNode(const rclcpp::NodeOptions & options);
    };

}