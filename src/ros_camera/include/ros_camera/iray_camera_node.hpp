#pragma once

#include "ros_camera/iray_camera.hpp"
#include "dart_interfaces/srv/set_exposure.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ros_camera
{

class IRayCameraNode : public rclcpp::Node
{
public:
  explicit IRayCameraNode(const rclcpp::NodeOptions & options);
  ~IRayCameraNode() override;

private:
  void captureLoop();
  void initializeImageMessages();
  void publishCompressed(const sensor_msgs::msg::Image & image_msg);
  void setExposureCallback(
    const std::shared_ptr<dart_interfaces::srv::SetExposure::Request> request,
    std::shared_ptr<dart_interfaces::srv::SetExposure::Response> response);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
  rclcpp::Service<dart_interfaces::srv::SetExposure>::SharedPtr exposure_service_;
  std::mutex camera_mutex_;
  std::unique_ptr<IRayCamera> camera_;
  // 预分配的单条发布消息(旋转开启时宽高已互换:width=原高, height=原宽),
  // 每帧原地覆盖,稳态零分配。
  sensor_msgs::msg::Image::SharedPtr image_msg_;
  // 原始方向 BGR 中间缓冲(尺寸 = 原宽*原高*3),锁内 grabInto 写此处,
  // 锁外再旋转进 image_msg_(与海康节点的 bgr_scratch_ 同构)。
  std::vector<unsigned char> bgr_scratch_;

  std::string frame_id_{"iray_camera"};
  int fps_{30};
  double exposure_us_{2000.0};
  double gain_{2.0};
  double gamma_{1.0};
  int jpeg_quality_{80};
  int compressed_fps_{30};
  std::string jpeg_path_;
  int64_t last_compressed_ns_{0};

  std::atomic_bool running_{false};
  std::thread capture_thread_;
};

}  // namespace ros_camera
