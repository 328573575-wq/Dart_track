#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include "dart_track_interfaces/msg/detections.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using Detections = dart_track_interfaces::msg::Detections;
namespace dart_track 
{

    class SolvePoseNode : public rclcpp::Node
    {
    private:
        rclcpp::Subscription<Detections>::SharedPtr detections_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
        // ========================参数设置===========================
        //真实坐标下四个引导灯的中心坐标
        std::vector<cv::Point3d> light_centers_3d_; //单位是m
        //内参和畸变
        cv::Mat camera_matrix_= (cv::Mat_<double>(3, 3) <<
                                460.0,   0.0, 320.0,
                                0.0, 460.0, 240.0,
                                0.0,   0.0,   1.0);
        cv::Mat dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
        0.0, 0.0, 0.0, 0.0, 0.0);

    private:
        void solve_pose_callback(const Detections::ConstSharedPtr& detections_msg);
        void tf_broadcast(const std_msgs::msg::Header& tf_header, const cv::Mat& rvec, const cv::Mat& tvec);
    public:
        explicit SolvePoseNode(const rclcpp::NodeOptions & options);
    };

}

