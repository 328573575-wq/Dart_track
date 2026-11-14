#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dart_point_interfaces/msg/dart_point.hpp"

using DartPoint = dart_point_interfaces::msg::DartPoint;

namespace dart_track
{

    // 纯可视化节点：把 /dart_point 转成 Foxglove 3D 面板认识的 Marker。
    //
    // Foxglove 的 3D 面板只认 visualization_msgs/Marker 这类标准 3D 类型，
    // 自定义的 DartPoint 它画不出来，所以这里做一个"翻译器"。
    // 它只订阅、只发布，不参与任何解算；删掉这个节点对测量没有任何影响。
    //
    // 发布话题 /dart_viz/markers（MarkerArray），所有 marker 都在 throw 系下：
    //   lights     四个引导灯（绿球）
    //   dart_axis  镖尾 -> 镖头的箭头（红），方向就是飞镖的朝向
    //   dart_head  镖头位置（红球）
    //   dart_angle 镖头旁边的文字，显示与 +y 轴的夹角（度）
    //   trajectory 累积的飞行轨迹（黄线）
    class MarkerNode : public rclcpp::Node
    {
    public:
        explicit MarkerNode(const rclcpp::NodeOptions & options);

    private:
        void dart_point_callback(const DartPoint::ConstSharedPtr & msg);

        // 往 array 里塞一个基础 marker（设置好 header / id / 颜色 / 尺寸）
        visualization_msgs::msg::Marker make_marker(
            int id, int type, const std_msgs::msg::Header & header,
            float r, float g, float b, float a) const;

        void add_lights(visualization_msgs::msg::MarkerArray & array,
                        const std_msgs::msg::Header & header) const;

        void add_dart(visualization_msgs::msg::MarkerArray & array,
                      const std_msgs::msg::Header & header,
                      const geometry_msgs::msg::Point & front,
                      const geometry_msgs::msg::Point & back,
                      double angle_deg) const;

        void add_trajectory(visualization_msgs::msg::MarkerArray & array,
                            const std_msgs::msg::Header & header);

        // 定时器回调：不依赖飞镖，单独把引导灯发出去。
        // 没有飞镖时 /dart_point 不存在，但引导灯还是要看得见。
        void lights_tick();

        rclcpp::Subscription<DartPoint>::SharedPtr dart_point_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr lights_timer_;

        // 四个引导灯在发射系下的坐标（和 solvepose 用同一份参数）
        std::vector<geometry_msgs::msg::Point> lights_;

        // 轨迹缓存：只留最近的一段，避免长时间运行内存一直涨
        std::deque<geometry_msgs::msg::Point> trajectory_;
        int64_t last_stamp_ns_ = 0;
        static constexpr size_t kMaxTrajectory = 600;
        // 两帧之间跳这么远，说明不是同一次飞行（仿真 loop 重来 / 重新捕获），清空轨迹
        static constexpr double kTeleportDistance = 2.0;

        // 最近一次飞镖状态，定时器刷新时如果它还新鲜，就连飞镖一起重发
        geometry_msgs::msg::Point last_front_;
        geometry_msgs::msg::Point last_back_;
        double last_angle_deg_ = 0.0;
        rclcpp::Time last_dart_time_{0, 0, RCL_ROS_TIME};
        bool has_dart_ = false;
        static constexpr double kDartKeepAlive = 1.0;  // 秒
    };

}  // namespace dart_track