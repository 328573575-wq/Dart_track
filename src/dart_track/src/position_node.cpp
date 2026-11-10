#include "dart_track/position_node.hpp"
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "dart_track_interfaces/msg/detections.hpp"
#include "dart_point_interfaces/msg/dart_point.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using Detections = dart_track_interfaces::msg::Detections;
using DartPoint = dart_point_interfaces::msg::DartPoint;
namespace dart_track
{
    PositionNode::PositionNode(const rclcpp::NodeOptions & options)
    : Node("position_node", options)
    {
        //真实点坐标的发布者
        real_point_pub_ = this->create_publisher<DartPoint>
        (
            "/dart_point",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        //接受识别信息的订阅者
        detections_sub_ = this->create_subscription<Detections>
        (
            "/dart_detections",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const Detections::ConstSharedPtr& detections_msg)
            {
                this->position_callback(detections_msg);
            });
        //TF监听
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        //读取相机内参yaml
        std::string camera_path;
        try 
        {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("dart_track");
            camera_path = pkg_share + "/config/ost.yaml";
        } 
        catch (const std::exception & e) 
        {
            RCLCPP_ERROR(get_logger(), "找不到包: %s", e.what());
            return;
        }

        cv::FileStorage fs(camera_path, cv::FileStorage::READ);
        if (!fs.isOpened()) 
        {
            RCLCPP_ERROR(get_logger(), "无法打开: %s", camera_path.c_str());
            return;
        }

        if (!fs["camera_matrix"].empty()) 
        {
            fs["camera_matrix"] >> camera_matrix_;
        } 
        else RCLCPP_ERROR(get_logger(), "缺少 camera_matrix");

        if (!fs["distortion_coefficients"].empty()) 
        {
            fs["distortion_coefficients"] >> dist_coeffs_;
        } 
        else RCLCPP_ERROR(get_logger(), "缺少 dist_coeffs");
    }

    void PositionNode::position_callback(
        const Detections::ConstSharedPtr& detections_msg)
    {
        //获取detections_msg
        std_msgs::msg::Header header = detections_msg->header;

        cv::Point2d dart_front_mid, dart_back_mid;
        dart_front_mid.x = detections_msg->dart_front_mid.x;
        dart_front_mid.y = detections_msg->dart_front_mid.y;
        dart_back_mid.x = detections_msg->dart_back_mid.x;
        dart_back_mid.y = detections_msg->dart_back_mid.y;
        // recognition 没找到灯条时会把镖头/镖尾标成 (0,0) 表示"本帧无效"，
        // 这种帧不能拿去解算（否则会算到几米外）。
        const bool dart_invalid =
            !std::isfinite(dart_front_mid.x) || !std::isfinite(dart_front_mid.y) ||
            !std::isfinite(dart_back_mid.x) || !std::isfinite(dart_back_mid.y) ||
            (dart_front_mid.x == 0.0 && dart_front_mid.y == 0.0) ||
            (dart_back_mid.x == 0.0 && dart_back_mid.y == 0.0);
        if (dart_invalid) {
            return;
        }

        std::vector<cv::Point2d> dart_mid = {dart_front_mid, dart_back_mid};
        std::vector<cv::Point2d> dart_undistort;
        cv::undistortPoints(dart_mid, dart_undistort, camera_matrix_, dist_coeffs_);

        cv::Mat_<double> p_front(3, 1), p_back(3, 1);
        p_front << dart_undistort[0].x, dart_undistort[0].y, 1;
        p_back << dart_undistort[1].x, dart_undistort[1].y, 1;

        //获取同时刻的TF
        geometry_msgs::msg::TransformStamped tf_msg;

        try 
        {
            tf_msg = tf_buffer_->lookupTransform
            (
                "throw",                                
                "camera",                                 
                header.stamp,                        
                rclcpp::Duration::from_seconds(0.05));   
        } 
        catch (const tf2::TransformException & ex) 
        {
        RCLCPP_WARN(this->get_logger(), "lookup failed: %s", ex.what());
        return;
        }    

        tf2::Quaternion q;
        tf2::fromMsg(tf_msg.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        cv::Mat_<double> R(3, 3);
        for (int i = 0; i < 3; i ++ )
        {
            for (int j = 0; j < 3; j ++ )
            {
                R(i, j) = m[i][j];
            }
        }

        cv::Point3d t = {tf_msg.transform.translation.x,
                         tf_msg.transform.translation.y,
                         tf_msg.transform.translation.z};

        cv::Mat_<double> D_front = R * p_front;
        cv::Mat_<double> D_back = R * p_back;

        cv::Point3d p_real_front = {0, 
                                    t.y - t.x * D_front(1, 0) / D_front(0, 0),
                                    t.z - t.x * D_front(2, 0) / D_front(0,0)};
        cv::Point3d p_real_back = {0, 
                                   t.y - t.x * D_back(1, 0) / D_back(0, 0),
                                   t.z - t.x * D_back(2, 0) / D_back(0,0)};
        cv::Point3d v = p_real_front - p_real_back;
        double angle_rad = std::atan2(v.z, v.y);
        double angle_deg = angle_rad * 180.0 / M_PI;

        // 发射过程中飞镖与 +y 轴的夹角不可能超过 ±90°（不会向上/向下翻过头）。
        // 识别有时会把镖头镖尾判反，导致向量反向、角度差 180°；
        // 这里把结果折回 [-90, 90]，方向标反也不会影响这个角度值。
        if (angle_deg > 90.0) {
            angle_deg -= 180.0;
        } else if (angle_deg < -90.0) {
            angle_deg += 180.0;
        }

        dart_point_publish(header ,p_real_front, p_real_back, angle_deg);

    }

    void PositionNode::dart_point_publish
        (
            const std_msgs::msg::Header & header,
            cv::Point3d& dart_real_front,
            cv::Point3d& dart_real_back,
            double& angle)
        {
            DartPoint dart_point;
            dart_point.header.stamp = header.stamp;
            dart_point.header.frame_id = "throw";

            dart_point.dart_real_front.x = dart_real_front.x;
            dart_point.dart_real_front.y = dart_real_front.y;
            dart_point.dart_real_front.z = dart_real_front.z;

            dart_point.dart_real_back.x = dart_real_back.x;
            dart_point.dart_real_back.y = dart_real_back.y;
            dart_point.dart_real_back.z = dart_real_back.z;

            dart_point.dart_angle_degree = angle;

            real_point_pub_->publish(dart_point);
        }
}

RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::PositionNode);