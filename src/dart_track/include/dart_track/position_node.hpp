#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "dart_point_interfaces/msg/dart_point.hpp"
#include "dart_track_interfaces/msg/detections.hpp"

using Detections = dart_track_interfaces::msg::Detections;
using DartPoint = dart_point_interfaces::msg::DartPoint;

namespace dart_track
{

    // 双相机解算节点。
    //
    // 两条流水线（cam1 / cam2）各自做识别 + PnP，本节点把两路结果合成一个三维点：
    //   相机 i 的像素 -> 去畸变 -> 射线(起点 C_i, 方向 d_i = R_i * p_norm)
    //   两条射线求最近点对，取中点作为飞镖位置
    //
    // 和以前单相机的区别：不再需要"飞镖在 x=0 平面上"这个假设，
    // x 坐标是三角化算出来的真实值。
    //
    // 只有一路数据时（另一路没插相机/没检测到），按 allow_plane_fallback 决定
    // 是丢弃还是退回平面假设。
    class PositionNode : public rclcpp::Node
    {
    public:
        explicit PositionNode(const rclcpp::NodeOptions & options);

    private:
        struct CameraStream
        {
            std::string label;                 // "cam1" / "cam2"，只用于日志
            std::string frame;                 // TF 子坐标系名 camera1 / camera2
            cv::Mat camera_matrix;             // 内参
            cv::Mat dist_coeffs;               // 畸变
            rclcpp::Subscription<Detections>::SharedPtr sub;

            bool has_data = false;   // 至少收到过一帧
            bool fresh = false;      // 这一帧还没被拿去融合过
            rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
            cv::Point2d dart_front{0.0, 0.0};
            cv::Point2d dart_back{0.0, 0.0};
        };

        // 一条射线：起点 + 方向（throw 系，方向未归一化）
        struct Ray
        {
            cv::Point3d origin;
            cv::Point3d direction;
        };

        void detections_callback(size_t index, const Detections::ConstSharedPtr & msg);

        // 像素 -> 射线（用该相机自己的内参去畸变，用该时刻的 TF 转到 throw 系）
        bool make_ray(size_t index, const cv::Point2d & pixel, const rclcpp::Time & stamp,
                      Ray & ray);

        // 两条射线求最近点对，返回中点。夹角太小/解退化时返回 false。
        bool triangulate(const Ray & r1, const Ray & r2, cv::Point3d & point,
                         double & angle_between_deg, double & gap);

        // 退化方案：单条射线和 x = 0 平面求交
        bool intersect_plane_x0(const Ray & ray, cv::Point3d & point);

        void publish_point(const std_msgs::msg::Header & header,
                           const cv::Point3d & front, const cv::Point3d & back);

        void load_camera(size_t index, const std::string & path, const std::string & frame,
                         const std::string & label);

        std::vector<CameraStream> cameras_;
        std::mutex data_mutex_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Publisher<DartPoint>::SharedPtr real_point_pub_;

        std::string parent_frame_ = "throw";
        double min_ray_angle_deg_ = 2.0;   // 两射线夹角下限，低于它深度不可信
        double max_stamp_diff_ = 0.05;     // 两路时间戳允许的最大差（秒）
        bool allow_plane_fallback_ = false;



        double angle_deg_ = 0.0;
    };

}  // namespace dart_track