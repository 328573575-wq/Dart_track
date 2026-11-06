#include "dart_track/solvepose_node.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "dart_track_interfaces/msg/detections.hpp"

namespace dart_track
{

    SolvePoseNode::SolvePoseNode(const rclcpp::NodeOptions & options)
    : Node("solvepose_node", options)
    {
        //接受识别信息的订阅者
        detections_sub_ = this->create_subscription<Detections>
        (
            "/dart_detections",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const Detections::ConstSharedPtr& detections_msg)
            {
                this->solve_pose_callback(detections_msg);
            });
        //获取真实坐标系引导灯坐标参数
        const std::vector<std::string> point_names = {
            "point_0", "point_1",
            "point_2", "point_3"
        };

        light_centers_3d_.clear();
        for (int i = 0; i < 4; i ++ ) {
            auto v = this->declare_parameter<std::vector<double>>(
                point_names[i], {0.0, 0.0, 0.0});
            light_centers_3d_.emplace_back(v[0], v[1], v[2]);
        }
        //实例化动态TF广播器
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    void SolvePoseNode::solve_pose_callback(
        const Detections::ConstSharedPtr& detections_msg)
    {
        std_msgs::msg::Header header = detections_msg->header;

        cv::Point2d dart_front_mid, dart_back_mid;
        dart_front_mid.x = detections_msg->dart_front_mid.x;
        dart_front_mid.y = detections_msg->dart_front_mid.y;
        dart_back_mid.x = detections_msg->dart_back_mid.x;
        dart_back_mid.y = detections_msg->dart_back_mid.y;

        std::vector<cv::Point2d> light_centers;
        for (const geometry_msgs::msg::Point& msg_pt : detections_msg->light_centers)
        {
            light_centers.emplace_back(msg_pt.x, msg_pt.y);
        }
        // TO DO: 已将识别节点发布的识别消息解包读取了，现在需要pnp解算，相机内参yaml
        cv::Mat rvec, tvec, rvec_inv, tvec_inv;
        //求得在相机坐标系下，发射坐标系的位姿
        bool pnp_ok = cv::solvePnP(
                        light_centers_3d_, 
                        light_centers, 
                        camera_matrix_, 
                        dist_coeffs_,
                        rvec,
                        tvec,
                        false, 
                        cv::SOLVEPNP_IPPE);
        if (!pnp_ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "solvePnP 求解失败！");
            return;
        }    
        //求在发射坐标系下，相机的位姿
        rvec_inv = -rvec;
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        tvec_inv = - R.t() * tvec;
        tf_broadcast(header, rvec_inv, tvec_inv); 
    }

    void SolvePoseNode::tf_broadcast(const std_msgs::msg::Header& tf_header, const cv::Mat& rvec, const cv::Mat& tvec)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = tf_header.stamp;
        tf_msg.header.frame_id = "throw";
        tf_msg.child_frame_id = "camera";
        tf_msg.transform.translation.x = tvec.at<double>(0);
        tf_msg.transform.translation.y = tvec.at<double>(1);
        tf_msg.transform.translation.z = tvec.at<double>(2);

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        tf2::Matrix3x3 m(
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
        tf2::Quaternion q;
        m.getRotation(q);
        tf_msg.transform.rotation = tf2::toMsg(q);
        broadcaster_->sendTransform(tf_msg);
    }
}


RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::SolvePoseNode);