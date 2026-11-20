#include "dart_track/position_node.hpp"

#include <algorithm>
#include <cmath>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace dart_track
{

    namespace
    {
        bool is_valid_pixel(const cv::Point2d & p)
        {
            return std::isfinite(p.x) && std::isfinite(p.y) &&
                   !(p.x == 0.0 && p.y == 0.0);
        }
    }  // namespace

    PositionNode::PositionNode(const rclcpp::NodeOptions & options)
    : Node("position_node", options)
    {
        parent_frame_ = this->declare_parameter<std::string>("parent_frame", "throw");
        min_ray_angle_deg_ =
            this->declare_parameter<double>("min_ray_angle_deg", 2.0);
        max_stamp_diff_ = this->declare_parameter<double>("max_stamp_diff", 0.05);
        // 只接了一台相机时是否退回老的平面假设；双目模式默认不允许，
        // 免得配错相机时静默退化成单目还不自知。
        allow_plane_fallback_ =
            this->declare_parameter<bool>("allow_plane_fallback", false);

        std::string pkg_share;
        try {
            pkg_share = ament_index_cpp::get_package_share_directory("dart_track");
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "找不到 dart_track 包: %s", e.what());
            return;
        }

        // 两路输入：话题、内参文件、TF 坐标系都可以分别配置
        cameras_.resize(2);
        const std::string topics[2] = {
            this->declare_parameter<std::string>("detections_topic_1", "/cam1/dart_detections"),
            this->declare_parameter<std::string>("detections_topic_2", "/cam2/dart_detections")};
        const std::string files[2] = {
            this->declare_parameter<std::string>("camera_file_1", pkg_share + "/config/ost1.yaml"),
            this->declare_parameter<std::string>("camera_file_2", pkg_share + "/config/ost2.yaml")};
        const std::string frames[2] = {
            this->declare_parameter<std::string>("camera_frame_1", "camera1"),
            this->declare_parameter<std::string>("camera_frame_2", "camera2")};

        for (size_t i = 0; i < 2; ++i) {
            load_camera(i, files[i], frames[i], "cam" + std::to_string(i + 1));
            // 每路一个回调组：一路慢不拖住另一路
            auto group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            rclcpp::SubscriptionOptions opt;
            opt.callback_group = group;
            cameras_[i].sub = this->create_subscription<Detections>(
                topics[i], rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
                [this, i](const Detections::ConstSharedPtr & msg) {
                    this->detections_callback(i, msg);
                },
                opt);
            RCLCPP_INFO(get_logger(), "[%s] 输入 %s，TF %s->%s，内参 %s",
                        cameras_[i].label.c_str(), topics[i].c_str(),
                        parent_frame_.c_str(), frames[i].c_str(), files[i].c_str());
        }

        real_point_pub_ = this->create_publisher<DartPoint>(
            "/dart_point", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(get_logger(),
                    "双相机解算已就绪：夹角下限 %.1f deg，两路时间差上限 %.0f ms%s",
                    min_ray_angle_deg_, max_stamp_diff_ * 1000.0,
                    allow_plane_fallback_ ? "，允许单相机退回平面假设" : "");
    }

    void PositionNode::load_camera(size_t index, const std::string & path,
                                   const std::string & frame, const std::string & label)
    {
        CameraStream & cam = cameras_[index];
        cam.frame = frame;
        cam.label = label;

        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(get_logger(), "[%s] 无法打开内参文件: %s", label.c_str(), path.c_str());
            return;
        }
        if (!fs["camera_matrix"].empty()) {
            fs["camera_matrix"] >> cam.camera_matrix;
        } else {
            RCLCPP_ERROR(get_logger(), "[%s] %s 缺少 camera_matrix", label.c_str(), path.c_str());
        }
        if (!fs["distortion_coefficients"].empty()) {
            fs["distortion_coefficients"] >> cam.dist_coeffs;
        } else {
            RCLCPP_ERROR(get_logger(), "[%s] %s 缺少 distortion_coefficients", label.c_str(),
                         path.c_str());
        }
    }

    bool PositionNode::make_ray(size_t index, const cv::Point2d & pixel,
                                const rclcpp::Time & stamp, Ray & ray)
    {
        CameraStream & cam = cameras_[index];
        if (cam.camera_matrix.empty() || cam.dist_coeffs.empty()) {
            return false;
        }

        // 1) 像素去畸变 + 归一化（z = 1）
        std::vector<cv::Point2d> src{pixel};
        std::vector<cv::Point2d> dst;
        cv::undistortPoints(src, dst, cam.camera_matrix, cam.dist_coeffs);

        // 2) 取该时刻的 TF：throw -> cameraN
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(
                parent_frame_, cam.frame, stamp, rclcpp::Duration::from_seconds(0.05));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[%s] TF lookup 失败: %s", cam.label.c_str(), ex.what());
            return false;
        }

        tf2::Quaternion q;
        tf2::fromMsg(tf_msg.transform.rotation, q);
        tf2::Matrix3x3 m(q);

        // 3) 方向：R * (x_n, y_n, 1)
        const tf2::Vector3 dir_cam(dst[0].x, dst[0].y, 1.0);
        const tf2::Vector3 dir_world = m * dir_cam;

        ray.origin = cv::Point3d(tf_msg.transform.translation.x,
                                 tf_msg.transform.translation.y,
                                 tf_msg.transform.translation.z);
        ray.direction = cv::Point3d(dir_world.x(), dir_world.y(), dir_world.z());
        return true;
    }

    bool PositionNode::triangulate(const Ray & r1, const Ray & r2, cv::Point3d & point,
                                   double & angle_between_deg, double & gap)
    {
        const cv::Point3d w0 = r1.origin - r2.origin;
        const double a = r1.direction.dot(r1.direction);
        const double b = r1.direction.dot(r2.direction);
        const double c = r2.direction.dot(r2.direction);
        const double d = r1.direction.dot(w0);
        const double e = r2.direction.dot(w0);
        const double denom = a * c - b * b;

        if (a <= 1e-12 || c <= 1e-12 || denom <= 1e-12) {
            return false;  // 某条射线长度为 0，或两条几乎平行
        }

        // 两条射线的夹角：cos = b / sqrt(a*c)
        const double cos_theta = std::max(-1.0, std::min(1.0, b / std::sqrt(a * c)));
        const double theta = std::acos(cos_theta);
        angle_between_deg = theta * 180.0 / M_PI;
        if (angle_between_deg < min_ray_angle_deg_) {
            return false;  // 夹角太小，深度方向对噪声过敏
        }

        // 最小二乘求最近点对：a*s - b*t = -d, b*s - c*t = -e
        const double s = (b * e - c * d) / denom;
        const double t = (a * e - b * d) / denom;
        if (s <= 0.0 || t <= 0.0) {
            return false;  // 交点在相机背后
        }

        const cv::Point3d p1 = r1.origin + s * r1.direction;
        const cv::Point3d p2 = r2.origin + t * r2.direction;
        point = (p1 + p2) * 0.5;
        gap = cv::norm(p1 - p2);
        return true;
    }

    bool PositionNode::intersect_plane_x0(const Ray & ray, cv::Point3d & point)
    {
        if (std::abs(ray.direction.x) < 1e-9) {
            return false;
        }
        const double s = -ray.origin.x / ray.direction.x;
        if (s <= 0.0) {
            return false;
        }
        point = ray.origin + s * ray.direction;
        point.x = 0.0;
        return true;
    }

    void PositionNode::detections_callback(size_t index, const Detections::ConstSharedPtr & msg)
    {
        const cv::Point2d front(msg->dart_front_mid.x, msg->dart_front_mid.y);
        const cv::Point2d back(msg->dart_back_mid.x, msg->dart_back_mid.y);
        if (!is_valid_pixel(front) || !is_valid_pixel(back)) {
            return;  // 这一帧没找到灯条，(0,0) 是无效标记
        }

        const rclcpp::Time stamp(msg->header.stamp);

        // 1) 决定这一次是"双目解算"还是"单目退回"
        bool stereo = false;
        bool use[2] = {false, false};
        rclcpp::Time stamps[2];
        cv::Point2d fronts[2];
        cv::Point2d backs[2];
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            CameraStream & cam = cameras_[index];
            cam.has_data = true;
            cam.fresh = true;
            cam.stamp = stamp;
            cam.dart_front = front;
            cam.dart_back = back;

            const size_t other = 1 - index;
            const bool other_ready = cameras_[other].has_data && cameras_[other].fresh;

            if (other_ready) {
                const double dt = std::abs((cameras_[0].stamp - cameras_[1].stamp).seconds());
                if (dt <= max_stamp_diff_) {
                    // 两路都是一帧没用过的新数据，且时间对得上 -> 双目
                    stereo = true;
                    for (size_t i = 0; i < 2; ++i) {
                        stamps[i] = cameras_[i].stamp;
                        fronts[i] = cameras_[i].dart_front;
                        backs[i] = cameras_[i].dart_back;
                        cameras_[i].fresh = false;
                    }
                } else {
                    // 时间差太大：丢掉较旧的一帧，留新的等下一次配对
                    const size_t older = (cameras_[0].stamp < cameras_[1].stamp) ? 0 : 1;
                    cameras_[older].fresh = false;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "两路时间差 %.0f ms 超过 %.0f ms，丢弃较旧的一帧",
                                         dt * 1000.0, max_stamp_diff_ * 1000.0);
                    return;
                }
            } else if (allow_plane_fallback_) {
                // 只有一路数据：退回"射线 ∩ x=0 平面"
                use[index] = true;
                stamps[index] = stamp;
                fronts[index] = front;
                backs[index] = back;
                cameras_[index].fresh = false;
            } else {
                return;  // 另一路还没起来；真机模式下宁可不出结果
            }
        }

        // 2) 算射线（各自内参、各自 TF、各自时间戳），锁已经放开
        cv::Point3d p_front;
        cv::Point3d p_back;
        bool ok = false;

        if (stereo) {
            Ray ray_front[2];
            Ray ray_back[2];
            bool rays_ok = true;
            for (size_t i = 0; i < 2; ++i) {
                if (!make_ray(i, fronts[i], stamps[i], ray_front[i]) ||
                    !make_ray(i, backs[i], stamps[i], ray_back[i])) {
                    rays_ok = false;
                }
            }
            if (rays_ok) {
                double ang_fe = 0.0;
                double ang_be = 0.0;
                double gap_fe = 0.0;
                double gap_be = 0.0;
                ok = triangulate(ray_front[0], ray_front[1], p_front, ang_fe, gap_fe) &&
                     triangulate(ray_back[0], ray_back[1], p_back, ang_be, gap_be);
                if (!ok) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "双目解算失败：夹角 %.1f / %.1f deg（下限 %.1f）",
                                         ang_fe, ang_be, min_ray_angle_deg_);
                }
            }
        } else {
            const size_t i = use[0] ? 0 : 1;
            Ray ray_front;
            Ray ray_back;
            if (make_ray(i, fronts[i], stamps[i], ray_front) &&
                make_ray(i, backs[i], stamps[i], ray_back)) {
                ok = intersect_plane_x0(ray_front, p_front) &&
                     intersect_plane_x0(ray_back, p_back);
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "只有一路检测，退回单相机平面假设（精度会下降）");
        }

        if (!ok) {
            return;
        }

        publish_point(msg->header, p_front, p_back);
    }

    void PositionNode::publish_point(const std_msgs::msg::Header & header,
                                     const cv::Point3d & front, const cv::Point3d & back)
    {
        const cv::Point3d v = front - back;
        double angle_deg = std::atan2(v.z, v.y) * 180.0 / M_PI;
        // 飞镖和 +y 轴的夹角不会超过 ±90°，折回这个区间，
        // 这样镖头镖尾判反也不影响角度值。
        if (angle_deg > 90.0) {
            angle_deg -= 180.0;
        } else if (angle_deg < -90.0) {
            angle_deg += 180.0;
        }
        angle_deg_ = angle_deg;

        DartPoint dart_point;
        dart_point.header = header;
        dart_point.header.frame_id = parent_frame_;

        dart_point.dart_real_front.x = front.x;
        dart_point.dart_real_front.y = front.y;
        dart_point.dart_real_front.z = front.z;

        dart_point.dart_real_back.x = back.x;
        dart_point.dart_real_back.y = back.y;
        dart_point.dart_real_back.z = back.z;

        dart_point.dart_angle_degree = angle_deg;

        real_point_pub_->publish(dart_point);
    }

}  // namespace dart_track

RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::PositionNode);