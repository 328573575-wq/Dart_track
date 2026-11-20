#include "dart_track/solvepose_node.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

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
        // 参数化：两条流水线各自订阅自己的检测、发布自己的 TF
        const std::string detections_topic =
            this->declare_parameter<std::string>("detections_topic", "/dart_detections");
        parent_frame_ = this->declare_parameter<std::string>("parent_frame", "throw");
        camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera");
        // 可选：相机大致安装位置 [x, y, z]（throw 系），用来消除对称灯阵的二义性
        expected_position_ = this->declare_parameter<std::vector<double>>(
            "expected_position", std::vector<double>{});
        has_expected_position_ = (expected_position_.size() == 3);

        //接受识别信息的订阅者
        detections_sub_ = this->create_subscription<Detections>
        (
            detections_topic,
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

        //读取相机内参yaml
        std::string camera_path;
        try 
        {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("dart_track");
            // 双相机：每台一条流水线，各用 ost1.yaml / ost2.yaml
            camera_path = this->declare_parameter<std::string>(
                "camera_file", pkg_share + "/config/ost1.yaml");
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
        // 识别可能会发"不足 4 个灯"的调试消息（让 draw 能画出来），
        // 那种消息不能进 PnP，否则 solvePnP 会因点数不匹配直接抛异常。
        if (light_centers.size() != 4) {
            return;
        }
        // ------------------------------------------------------------------
        // 灯序问题：recognition 是按图像位置给四个灯排序的，换一个视角，
        // 这个顺序会整体循环移位（比如 [0,1,2,3] 变成 [3,0,1,2]）。
        // 直接拿它去 PnP，位姿会算错但不报错。
        // 所以这里把 4 个循环移位都试一遍，取重投影误差最小的那个。
        //
        // 如果灯阵本身是对称的（比如标准矩形），180° 那一档会和正确的
        // 一样拟合得很好 —— 这时只能靠 expected_position（相机大致安装位置）
        // 来分辨，没配置就取误差最小的。
        // ------------------------------------------------------------------
        cv::Mat rvec, tvec;
        double best_err = 1e18;
        int best_shift = -1;
        std::vector<cv::Mat> candidate_rvec, candidate_tvec;
        std::vector<double> candidate_err;
        std::vector<cv::Point3d> candidate_C;

        for (int shift = 0; shift < 4; ++shift)
        {
            std::vector<cv::Point3d> obj(4);
            for (int i = 0; i < 4; ++i) {
                obj[i] = light_centers_3d_[(i + shift) % 4];
            }
            cv::Mat rv, tv;
            if (!cv::solvePnP(obj, light_centers, camera_matrix_, dist_coeffs_,
                              rv, tv, false, cv::SOLVEPNP_IPPE)) {
                continue;
            }
            std::vector<cv::Point2d> proj;
            cv::projectPoints(obj, rv, tv, camera_matrix_, dist_coeffs_, proj);
            double err = 0.0;
            for (int i = 0; i < 4; ++i) {
                err += cv::norm(proj[i] - light_centers[i]);
            }
            err /= 4.0;

            // 相机中心（throw 系）
            cv::Mat R;
            cv::Rodrigues(rv, R);
            cv::Mat C = -R.t() * tv;

            candidate_rvec.push_back(rv);
            candidate_tvec.push_back(tv);
            candidate_err.push_back(err);
            candidate_C.emplace_back(C.at<double>(0), C.at<double>(1), C.at<double>(2));
            if (err < best_err) {
                best_err = err;
                best_shift = static_cast<int>(candidate_err.size()) - 1;
            }
        }

        if (best_shift < 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "solvePnP 求解失败！");
            return;
        }

        // 二义性处理：
        //   1) 配了 expected_position -> 挑离它最近的
        //   2) 没配 -> 挑离"上一次解出的位置"最近的（保持连续，不会来回跳）
        // 对称灯阵（标准矩形）下 180° 那一档会和正确解拟合得一样好，
        // 光看重投影误差是分不出来的。
        {
            // 每台相机有自己的灯坐标文件（顺序就是它图像里的编号顺序），
            // 所以优先相信 shift = 0；只有它明显拟合不上时才用移位的结果。
            int pick = 0;
            double best_dist = 1e18;
            const bool shift0_ok =
                !candidate_err.empty() && candidate_err[0] <= best_err + 0.5;
            const bool use_expected = has_expected_position_;
            const bool use_last = !last_position_valid_;
            (void)use_last;
            if (shift0_ok) {
                // 保持 pick = 0
            } else if (has_expected_position_ || last_position_valid_) {
                for (size_t k = 0; k < candidate_err.size(); ++k) {
                    if (candidate_err[k] > best_err + 0.5) continue;  // 拟合明显更差的不考虑
                    double dx, dy, dz;
                    if (use_expected) {
                        dx = candidate_C[k].x - expected_position_[0];
                        dy = candidate_C[k].y - expected_position_[1];
                        dz = candidate_C[k].z - expected_position_[2];
                    } else {
                        dx = candidate_C[k].x - last_position_.x;
                        dy = candidate_C[k].y - last_position_.y;
                        dz = candidate_C[k].z - last_position_.z;
                    }
                    const double dist = dx * dx + dy * dy + dz * dz;
                    if (dist < best_dist) {
                        best_dist = dist;
                        pick = static_cast<int>(k);
                    }
                }
            }
            best_shift = pick;
            last_position_ = candidate_C[best_shift];
            last_position_valid_ = true;
        }

        rvec = candidate_rvec[best_shift];
        tvec = candidate_tvec[best_shift];

        //求在发射坐标系下，相机的位姿
        cv::Mat rvec_inv = -rvec;
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat tvec_inv = - R.t() * tvec;
        tf_broadcast(header, rvec_inv, tvec_inv);
    }

    void SolvePoseNode::tf_broadcast(const std_msgs::msg::Header& tf_header, const cv::Mat& rvec, const cv::Mat& tvec)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = tf_header.stamp;
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = camera_frame_;
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