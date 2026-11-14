#include "dart_track/marker_node.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace dart_track
{
    namespace
    {
        // 单位向量：从坐标系 +x 轴转到目标方向。Marker 的 ARROW 默认沿 +x 伸直。
        tf2::Quaternion quaternion_from_x_to(const tf2::Vector3 & direction)
        {
            const tf2::Vector3 x_axis(1.0, 0.0, 0.0);
            const double dot = x_axis.dot(direction);

            if (dot < -1.0 + 1e-9) {
                // 正好反向：绕任意垂直轴转 180°
                return tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI);
            }
            if (dot > 1.0 - 1e-9) {
                return tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
            }

            tf2::Vector3 axis = x_axis.cross(direction);
            axis.normalize();
            const double angle = std::atan2(x_axis.cross(direction).length(), dot);
            return tf2::Quaternion(axis, angle);
        }

        bool is_valid_point(const geometry_msgs::msg::Point & p)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                return false;
            }
            // (0,0,0) 是 position_node / recognition 的"本帧无效"约定值
            return !(p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
        }
    }  // namespace

    MarkerNode::MarkerNode(const rclcpp::NodeOptions & options)
    : rclcpp::Node("marker_node", options)
    {
        // 和 solvepose 用同一份参数文件（real_light_centers.yaml）
        const std::vector<std::string> point_names = {
            "point_0", "point_1", "point_2", "point_3"
        };
        lights_.clear();
        for (int i = 0; i < 4; ++i) {
            const auto v = this->declare_parameter<std::vector<double>>(
                point_names[i], {0.0, 0.0, 0.0});
            geometry_msgs::msg::Point p;
            p.x = v[0];
            p.y = v[1];
            p.z = v[2];
            lights_.push_back(p);
        }

        // 和 position_node 的发布端 QoS 保持一致（reliable）
        dart_point_sub_ = this->create_subscription<DartPoint>(
            "/dart_point",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
            [this](const DartPoint::ConstSharedPtr & msg) {
                this->dart_point_callback(msg);
            });

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/dart_viz/markers", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

        // 引导灯是静态的，每秒重发一次：
        // 这样即使一只飞镖都没检测到（没有 /dart_point），3D 里也能看到灯；
        // 后连上来的 Foxglove 客户端最多等 1 秒也能收到。
        lights_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), [this]() { this->lights_tick(); });

        RCLCPP_INFO(get_logger(),
                    "可视化节点已启动：/dart_point -> /dart_viz/markers（frame=throw），"
                    "引导灯每秒刷新一次");
    }

    visualization_msgs::msg::Marker MarkerNode::make_marker(
        int id, int type, const std_msgs::msg::Header & header,
        float r, float g, float b, float a) const
    {
        visualization_msgs::msg::Marker m;
        m.header = header;
        m.header.frame_id = "throw";
        m.ns = "dart_viz";
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = a;
        // 0 = 一直显示，直到收到新的或 DELETE
        m.lifetime = rclcpp::Duration::from_seconds(0.0);
        return m;
    }

    void MarkerNode::add_lights(
        visualization_msgs::msg::MarkerArray & array,
        const std_msgs::msg::Header & header) const
    {
        for (size_t i = 0; i < lights_.size(); ++i) {
            auto m = make_marker(static_cast<int>(i),
                                 visualization_msgs::msg::Marker::SPHERE,
                                 header, 0.1f, 1.0f, 0.2f, 1.0f);
            m.pose.position = lights_[i];
            // 直径 0.12 m（场景里灯半径 0.06）
            m.scale.x = 0.12;
            m.scale.y = 0.12;
            m.scale.z = 0.12;
            array.markers.push_back(m);
        }
    }

    void MarkerNode::add_dart(
        visualization_msgs::msg::MarkerArray & array,
        const std_msgs::msg::Header & header,
        const geometry_msgs::msg::Point & front,
        const geometry_msgs::msg::Point & back,
        double angle_deg) const
    {
        const tf2::Vector3 f(front.x, front.y, front.z);
        const tf2::Vector3 b(back.x, back.y, back.z);
        const tf2::Vector3 v = f - b;
        const double length = v.length();

        // 1) 箭头：镖尾 -> 镖头
        if (length > 1e-6) {
            auto axis = make_marker(10, visualization_msgs::msg::Marker::ARROW,
                                    header, 1.0f, 0.15f, 0.15f, 1.0f);
            axis.pose.position = back;
            const tf2::Quaternion q = quaternion_from_x_to(v / length);
            axis.pose.orientation.x = q.x();
            axis.pose.orientation.y = q.y();
            axis.pose.orientation.z = q.z();
            axis.pose.orientation.w = q.w();
            axis.scale.x = length;   // 箭头长度
            axis.scale.y = 0.012;    // 轴径
            axis.scale.z = 0.024;    // 箭头头部直径
            array.markers.push_back(axis);
        }

        // 2) 镖头球
        auto head = make_marker(11, visualization_msgs::msg::Marker::SPHERE,
                                header, 1.0f, 0.1f, 0.1f, 1.0f);
        head.pose.position = front;
        head.scale.x = 0.03;
        head.scale.y = 0.03;
        head.scale.z = 0.03;
        array.markers.push_back(head);

        // 3) 角度文字（TEXT_VIEW_FACING 永远朝向观察者）
        auto text = make_marker(12, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
                                header, 1.0f, 1.0f, 1.0f, 1.0f);
        text.pose.position = front;
        text.pose.position.z += 0.1;
        text.scale.z = 0.08;  // 字高（米）
        char buffer[64];
        std::snprintf(buffer, sizeof(buffer), "ang %.1f deg", angle_deg);
        text.text = buffer;
        array.markers.push_back(text);
    }

    void MarkerNode::add_trajectory(
        visualization_msgs::msg::MarkerArray & array,
        const std_msgs::msg::Header & header)
    {
        auto line = make_marker(20, visualization_msgs::msg::Marker::LINE_STRIP,
                                header, 1.0f, 1.0f, 0.0f, 1.0f);
        line.scale.x = 0.01;
        line.points.assign(trajectory_.begin(), trajectory_.end());
        // 只有一个点时画不出线，补一个跟在它后面的点
        if (line.points.size() == 1) {
            line.points.push_back(line.points.front());
        }
        array.markers.push_back(line);
    }

    void MarkerNode::lights_tick()
    {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "throw";

        visualization_msgs::msg::MarkerArray array;
        add_lights(array, header);

        // 飞镖还"新鲜"的话连它一起重发，这样不同版本的客户端
        // 即使把 MarkerArray 当成"完整快照"，也不会把飞镖抹掉。
        if (has_dart_ &&
            (this->now() - last_dart_time_).seconds() < kDartKeepAlive)
        {
            add_dart(array, header, last_front_, last_back_, last_angle_deg_);
            add_trajectory(array, header);
        }

        marker_pub_->publish(array);
    }

    void MarkerNode::dart_point_callback(const DartPoint::ConstSharedPtr & msg)
    {
        const geometry_msgs::msg::Point & front = msg->dart_real_front;
        const geometry_msgs::msg::Point & back = msg->dart_real_back;

        // 无效帧（识别没找到灯条）不发 marker，画面保持上一帧，不会闪到原点
        if (!is_valid_point(front) || !is_valid_point(back)) {
            return;
        }

        // 时间倒退（切了图像源）就当重新开始
        const int64_t stamp_ns =
            static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
            static_cast<int64_t>(msg->header.stamp.nanosec);
        if (last_stamp_ns_ != 0 && stamp_ns < last_stamp_ns_) {
            trajectory_.clear();
        }
        last_stamp_ns_ = stamp_ns;

        // 中点作为轨迹点
        geometry_msgs::msg::Point mid;
        mid.x = 0.5 * (front.x + back.x);
        mid.y = 0.5 * (front.y + back.y);
        mid.z = 0.5 * (front.z + back.z);

        if (!trajectory_.empty()) {
            const geometry_msgs::msg::Point & last = trajectory_.back();
            const double dx = mid.x - last.x;
            const double dy = mid.y - last.y;
            const double dz = mid.z - last.z;
            if (std::sqrt(dx * dx + dy * dy + dz * dz) > kTeleportDistance) {
                trajectory_.clear();  // 不是同一次飞行
            }
        }
        trajectory_.push_back(mid);
        while (trajectory_.size() > kMaxTrajectory) {
            trajectory_.pop_front();
        }

        last_front_ = front;
        last_back_ = back;
        last_angle_deg_ = msg->dart_angle_degree;
        last_dart_time_ = this->now();
        has_dart_ = true;

        visualization_msgs::msg::MarkerArray array;
        add_lights(array, msg->header);
        add_dart(array, msg->header, front, back, msg->dart_angle_degree);
        add_trajectory(array, msg->header);
        marker_pub_->publish(array);
    }

}  // namespace dart_track

RCLCPP_COMPONENTS_REGISTER_NODE(dart_track::MarkerNode);