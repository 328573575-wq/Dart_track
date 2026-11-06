#include "ros_camera/iray_camera_node.hpp"
#include "ros_camera/image_rotate.hpp"
#include "params_control/params_control.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
// 连续取流失败达到该次数就重启一次取流,应对相机拔插/链路抖动
constexpr int kMaxConsecutiveGrabFailures = 10;
// 取流失败后的退避间隔,避免 SDK 立即返回错误时形成紧密循环
constexpr int kGrabRetryBackoffMs = 5;
}  // namespace

namespace ros_camera
{

IRayCameraNode::IRayCameraNode(const rclcpp::NodeOptions & options)
: Node("iray_camera_node", options)
{
  frame_id_ = IRayParams::frame_id;
  fps_ = IRayParams::fps;
  exposure_us_ = camera_mode::Params::recognition_exposure_time;
  gain_ = IRayParams::gain;
  gamma_ = IRayParams::gamma;
  jpeg_quality_ = IRayParams::jpeg_quality;
  compressed_fps_ = IRayParams::compressed_fps;

  image_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS());
  compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "/camera/compressed", rclcpp::SensorDataQoS());
  exposure_service_ = create_service<dart_interfaces::srv::SetExposure>(
    "/camera/set_exposure",
    std::bind(&IRayCameraNode::setExposureCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  // IMV_SaveImageToFile 只能编码到文件,而这次写盘发生在相机锁内、会拖慢曝光切换。
  // 优先用 tmpfs(/dev/shm)把写盘开销降到内存级,不可写时回退 /tmp。
  const char * jpeg_dir = (access("/dev/shm", W_OK) == 0) ? "/dev/shm" : "/tmp";
  jpeg_path_ = std::string(jpeg_dir) + "/dart_iray_camera_" +
    std::to_string(static_cast<long long>(getpid())) + ".jpg";
  RCLCPP_INFO(get_logger(), "IRay JPEG temp file: %s", jpeg_path_.c_str());

  camera_ = std::make_unique<IRayCamera>(exposure_us_, gain_, gamma_, fps_);
  camera_->init();
  initializeImageMessages();

  running_.store(true);
  capture_thread_ = std::thread(&IRayCameraNode::captureLoop, this);
}

IRayCameraNode::~IRayCameraNode()
{
  running_.store(false);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (!jpeg_path_.empty()) {
    std::remove(jpeg_path_.c_str());
  }
}

void IRayCameraNode::initializeImageMessages()
{
  const int raw_w = camera_->width();   // 列数 W
  const int raw_h = camera_->height();  // 行数 H

  image_msg_ = std::make_shared<sensor_msgs::msg::Image>();
  image_msg_->header.frame_id = frame_id_;
  image_msg_->encoding = "bgr8";
  image_msg_->is_bigendian = false;
  if constexpr (ros_camera::kRotateImageCcw90) {
    // 逆时针 90° 后宽高互换,像素字节数不变(仍为 W*H*3)
    image_msg_->width = static_cast<uint32_t>(raw_h);
    image_msg_->height = static_cast<uint32_t>(raw_w);
  } else {
    image_msg_->width = static_cast<uint32_t>(raw_w);
    image_msg_->height = static_cast<uint32_t>(raw_h);
  }
  image_msg_->step = image_msg_->width * 3U;
  image_msg_->data.resize(
    static_cast<std::size_t>(image_msg_->step) * image_msg_->height);

  // 原始方向中间缓冲:与 image_msg_->data 字节数相同。两种模式都分配,
  // 保证 captureLoop 里的 grab 目标恒有效(取消旋转时也走这条路径)。
  bgr_scratch_.resize(image_msg_->data.size());

  if constexpr (ros_camera::kRotateImageCcw90) {
    RCLCPP_INFO(get_logger(), "rotate image 90 deg CCW before publish: raw %dx%d -> %ux%u",
      raw_w, raw_h, image_msg_->width, image_msg_->height);
  }
}

void IRayCameraNode::captureLoop()
{
  int fail_count = 0;
  while (rclcpp::ok() && running_.load()) {
    // grab 目标:旋转开启时先落私有中间缓冲,锁外再旋转进发布消息;关闭旋转时
    // 直接写发布缓冲。两者字节数相同(均为 W*H*3),grabInto 的尺寸校验都精确通过。
    unsigned char * grab_dst = nullptr;
    std::size_t grab_size = 0;
    if constexpr (ros_camera::kRotateImageCcw90) {
      grab_dst = bgr_scratch_.data();
      grab_size = bgr_scratch_.size();
    } else {
      grab_dst = image_msg_->data.data();
      grab_size = image_msg_->data.size();
    }

    double camera_stamp_ms = 0.0;
    bool grabbed = false;
    try {
      std::lock_guard<std::mutex> camera_lock(camera_mutex_);
      camera_->grabInto(grab_dst, grab_size, camera_stamp_ms);
      grabbed = true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "grab error: %s", e.what());
    }

    if (!grabbed) {
      // 连续失败到阈值就重启一次取流:相机拔插/链路抖动会让取流内部状态失效,
      // 不重启就再也收不到图。海康节点有同样的自愈逻辑。
      if (++fail_count >= kMaxConsecutiveGrabFailures) {
        RCLCPP_WARN(get_logger(), "grab failed %d times in a row, restarting stream", fail_count);
        {
          std::lock_guard<std::mutex> camera_lock(camera_mutex_);
          camera_->restartStream();
        }
        fail_count = 0;
      }
      // 退避:SDK 立即返回错误时,不加停顿会变成紧密循环——既空烧 CPU,
      // 又高频抢放相机锁,足以饿死曝光设置回调。sleep 必须在锁外。
      std::this_thread::sleep_for(std::chrono::milliseconds(kGrabRetryBackoffMs));
      continue;
    }
    fail_count = 0;

    if constexpr (ros_camera::kRotateImageCcw90) {
      // COW:上一条消息仍被 QoS 历史/订阅者引用时,先独占一份再写入。旋转会整帧
      // 覆盖像素,故只拷元数据、不拷旧像素,省一次全帧 memcpy。
      // 注:按当前 rclcpp 语义,publish(const T&) 会整条堆拷贝一份,image_msg_ 不会
      // 被他处持有,这段常态不触发;保留为与海康节点一致的防御性代码。
      if (!image_msg_.unique()) {
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        msg->header = image_msg_->header;
        msg->width = image_msg_->width;
        msg->height = image_msg_->height;
        msg->encoding = image_msg_->encoding;
        msg->is_bigendian = image_msg_->is_bigendian;
        msg->step = image_msg_->step;
        msg->data.resize(image_msg_->data.size());
        image_msg_ = std::move(msg);
      }
      // 旋转是纯内存操作,放到相机锁外执行,不阻塞取流与曝光设置
      const auto rotate_start = std::chrono::steady_clock::now();
      ros_camera::rotateBgr90Ccw(
        bgr_scratch_.data(), image_msg_->data.data(), camera_->width(), camera_->height());
      const double rotate_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - rotate_start).count();
      if (rotate_ms > 8.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "slow rotateBgr90Ccw: %.1f ms", rotate_ms);
      }
    }

    image_msg_->header.stamp = now();
    // 只能解引用发布:rclcpp 没有 SharedPtr 重载,只有 const T& 与带自定义 deleter 的 unique_ptr
    image_pub_->publish(*image_msg_);
    publishCompressed(*image_msg_);
  }
}

void IRayCameraNode::setExposureCallback(
  const std::shared_ptr<dart_interfaces::srv::SetExposure::Request> request,
  std::shared_ptr<dart_interfaces::srv::SetExposure::Response> response)
{
  if (!request || request->exposure_us <= 0.0) {
    response->success = false;
    response->message = "exposure_us must be positive";
    return;
  }

  std::lock_guard<std::mutex> camera_lock(camera_mutex_);
  try {
    camera_->setExpo(request->exposure_us);
    response->success = true;
    response->applied_exposure_us = camera_->getExpo();
    exposure_us_ = response->applied_exposure_us;
    response->message = "exposure updated";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = e.what();
  }
}

void IRayCameraNode::publishCompressed(const sensor_msgs::msg::Image & image_msg)
{
  if (compressed_fps_ <= 0) {
    return;
  }

  const int64_t now_ns = get_clock()->now().nanoseconds();
  const int64_t interval_ns = static_cast<int64_t>(1e9 / compressed_fps_);
  if (now_ns - last_compressed_ns_ < interval_ns) {
    return;
  }
  last_compressed_ns_ = now_ns;

  bool saved = false;
  {
    std::lock_guard<std::mutex> camera_lock(camera_mutex_);
    saved = camera_->saveJpeg(
      image_msg.data.data(), image_msg.data.size(), image_msg.width, image_msg.height,
      jpeg_quality_, jpeg_path_);
  }
  if (!saved) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "IMV_SaveImageToFile failed while encoding IRay image");
    return;
  }

  std::ifstream jpeg_file(jpeg_path_, std::ios::binary | std::ios::ate);
  if (!jpeg_file) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "failed to open IRay JPEG output");
    return;
  }
  const std::streamsize size = jpeg_file.tellg();
  if (size <= 0) {
    return;
  }
  jpeg_file.seekg(0, std::ios::beg);

  sensor_msgs::msg::CompressedImage compressed_msg;
  compressed_msg.header = image_msg.header;
  compressed_msg.format = "jpeg";
  compressed_msg.data.resize(static_cast<std::size_t>(size));
  if (!jpeg_file.read(reinterpret_cast<char *>(compressed_msg.data.data()), size)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "failed to read IRay JPEG output");
    return;
  }
  compressed_pub_->publish(compressed_msg);
}

}  // namespace ros_camera

RCLCPP_COMPONENTS_REGISTER_NODE(ros_camera::IRayCameraNode)
