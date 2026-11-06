// Copyright (c) 2026 Dart. Licensed under MIT.
//
// 海康相机节点：通过 MVS SDK 读取画面并发布到 /camera/image_raw。
// 参照 AutoAim ros_camera/hikCam.cpp 与 camera_bridge/HIK.cpp 的成熟写法，
// 取"单采集线程 + 预分配缓冲 + COW 复用"的最简高性能方案。
//
// 相机物理安装旋转 90°，故发布前把每帧内容逆时针旋转 90°（宽高互换）。
// 旋转在源头一次完成：/camera/image_raw、/camera/compressed、录包与
// /dart/target 像素坐标共享同一"正"坐标系。旋转为纯内存操作、放 SDK 锁外，
// 且只对私有中间缓冲 -> 发布消息做一遍，稳态零分配、无 OpenCV/MVS 依赖。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "MvCameraControl.h"
#include "dart_interfaces/srv/set_exposure.hpp"

#include "params_control/params_control.hpp"

// 旋转内核与 kRotateImageCcw90 由共享头提供(华睿节点同样使用)
#include "ros_camera/image_rotate.hpp"

namespace
{

// 将 MVS 错误码格式化为便于排查的十六进制串
std::string hikError(const int code)
{
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "0x%08X", code);
    return buffer;
}

}  // namespace

namespace ros_camera
{

class HKCameraNode : public rclcpp::Node
{
public:
    explicit HKCameraNode(const rclcpp::NodeOptions & options);
    ~HKCameraNode() override;

private:
    // 枚举设备并按序列号选择（camera_acsn 为空则取第一台）
    bool selectCamera();
    // 打开设备、下发分辨率/帧率/曝光/增益等配置并开始取流
    void openAndConfigure();
    // 采集循环：取帧 -> 转换(-> 旋转) -> 发布
    void captureLoop();
    // SDK 锁内：把一帧原始数据转为 bgr8，成功返回 true。
    // 旋转开启时写入私有 bgr_scratch_（原始方向），否则直写 image_msg_。
    bool convertFrame(const MV_FRAME_OUT & out_frame);
    // 按 compressed_fps_ 限频将当前帧压缩为 jpeg 发布到 /camera/compressed
    void publishCompressed();
    void setExposureCallback(
        const std::shared_ptr<dart_interfaces::srv::SetExposure::Request> request,
        std::shared_ptr<dart_interfaces::srv::SetExposure::Response> response);

    void * handle_ = nullptr;
    MV_CC_DEVICE_INFO device_info_{};

    int width_ = 1440;
    int height_ = 1080;
    double fps_ = 160.0;
    double exposure_us_ = 2000.0;
    double gain_db_ = 16.0;
    std::string camera_acsn_;
    std::string frame_id_;
    int jpeg_quality_ = 80;     // 压缩质量 (1-100)
    int compressed_fps_ = 30;   // 压缩话题发布帧率，0 表示关闭

    sensor_msgs::msg::Image::SharedPtr image_msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Service<dart_interfaces::srv::SetExposure>::SharedPtr exposure_service_;
    std::mutex sdk_mutex_;

    std::vector<unsigned char> bgr_scratch_;  // 旋转开启时 MVS 转换的中间缓冲（原始方向 BGR）
    std::vector<unsigned char> jpeg_buffer_;  // JPEG 编码输出缓冲，预分配复用
    int64_t last_compressed_ns_ = 0;          // 上次压缩发布的主机时间(ns)，用于限频

    std::atomic<bool> running_{false};
    std::thread capture_thread_;

    // 设备时间戳(10ns 单位)到主机时间(ns)的偏移，首帧计算一次
    uint64_t device_to_host_ns_ = 0;
    bool first_frame_ = true;
};

HKCameraNode::HKCameraNode(const rclcpp::NodeOptions & options)
: Node("hk_camera_node", options)
{
    // 相机参数统一从 params_control/config.yaml 的 [hk_camera] 段读取
    width_ = HKParams::width;
    height_ = HKParams::height;
    fps_ = HKParams::fps;
    exposure_us_ = camera_mode::Params::recognition_exposure_time;
    gain_db_ = HKParams::gain;
    camera_acsn_ = HKParams::camera_acsn;
    frame_id_ = HKParams::frame_id;
    jpeg_quality_ = HKParams::jpeg_quality;
    compressed_fps_ = HKParams::compressed_fps;

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS());
    compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/compressed", rclcpp::SensorDataQoS());
    exposure_service_ = create_service<dart_interfaces::srv::SetExposure>(
        "/camera/set_exposure",
        std::bind(&HKCameraNode::setExposureCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    if (!selectCamera()) {
        throw std::runtime_error("no Hikvision camera found, check cable or usbipd attach");
    }
    openAndConfigure();

    // 预分配图像消息，之后每帧原地填充，稳态零分配。
    // 旋转开启时发布消息宽高互换（内容逆时针 90°），数据字节数不变。
    image_msg_ = std::make_shared<sensor_msgs::msg::Image>();
    image_msg_->header.frame_id = frame_id_;
    image_msg_->encoding = "bgr8";
    image_msg_->is_bigendian = false;
    if constexpr (kRotateImageCcw90) {
        image_msg_->width = static_cast<uint32_t>(height_);
        image_msg_->height = static_cast<uint32_t>(width_);
    } else {
        image_msg_->width = static_cast<uint32_t>(width_);
        image_msg_->height = static_cast<uint32_t>(height_);
    }
    image_msg_->step = image_msg_->width * 3U;
    image_msg_->data.resize(image_msg_->step * image_msg_->height);
    // JPEG 输出缓冲按未压缩尺寸预留，最坏情况也足够容纳
    jpeg_buffer_.resize(image_msg_->step * image_msg_->height);
    if constexpr (kRotateImageCcw90) {
        // MVS 转换输出缓冲：相机原始方向 WxH，字节数与旋转后的消息相同
        bgr_scratch_.resize(image_msg_->data.size());
        RCLCPP_INFO(get_logger(), "rotate image 90 deg CCW before publish: raw %dx%d -> %ux%u",
                    width_, height_, image_msg_->width, image_msg_->height);
    }

    running_.store(true);
    capture_thread_ = std::thread(&HKCameraNode::captureLoop, this);
}

HKCameraNode::~HKCameraNode()
{
    running_.store(false);
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    if (handle_ != nullptr) {
        MV_CC_StopGrabbing(handle_);
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
    }
}

bool HKCameraNode::selectCamera()
{
    MV_CC_DEVICE_INFO_LIST device_list{};
    const int ret = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    if (ret != MV_OK) {
        RCLCPP_ERROR(get_logger(), "MV_CC_EnumDevices failed: %s", hikError(ret).c_str());
        return false;
    }
    if (device_list.nDeviceNum == 0) {
        return false;
    }

    RCLCPP_INFO(get_logger(), "Found %u Hikvision camera(s)", device_list.nDeviceNum);
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
        const auto * info = device_list.pDeviceInfo[i];
        std::string serial;
        if (info->nTLayerType == MV_USB_DEVICE) {
            serial = reinterpret_cast<const char *>(info->SpecialInfo.stUsb3VInfo.chSerialNumber);
        } else if (info->nTLayerType == MV_GIGE_DEVICE) {
            serial = reinterpret_cast<const char *>(info->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        if (camera_acsn_.empty() || serial == camera_acsn_) {
            device_info_ = *info;
            RCLCPP_INFO(get_logger(), "Using camera: %s", serial.c_str());
            return true;
        }
    }

    // 指定序列号未找到时回退第一台
    device_info_ = *device_list.pDeviceInfo[0];
    RCLCPP_WARN(get_logger(), "camera '%s' not found, fall back to the first device", camera_acsn_.c_str());
    return true;
}

void HKCameraNode::openAndConfigure()
{
    int ret = MV_CC_CreateHandle(&handle_, &device_info_);
    if (ret != MV_OK) {
        throw std::runtime_error("MV_CC_CreateHandle failed: " + hikError(ret));
    }
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        throw std::runtime_error("MV_CC_OpenDevice failed: " + hikError(ret));
    }

    // 分辨率、帧率、曝光、增益（单项失败不致命，与 AutoAim 的宽容策略一致）
    MV_CC_SetIntValue(handle_, "Width", width_);
    MV_CC_SetIntValue(handle_, "Height", height_);
    MV_CC_SetEnumValue(handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);  // 0x01080009
    MV_CC_SetEnumValue(handle_, "ADCBitDepth", 0);                       // 8 bit
    MV_CC_SetEnumValue(handle_, "AcquisitionMode", 2);                   // 连续采集
    MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(fps_));
    MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_us_));
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_db_));
    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", static_cast<unsigned int>(HKParams::balance_white_auto));

    // 性能相关：最小 SDK 缓存 + 快速 Bayer 插值 + 只保留最新帧
    MV_CC_SetImageNodeNum(handle_, 1);
    MV_CC_SetBayerCvtQuality(handle_, 0);
    MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly);

    // 读回实际生效的分辨率，按它分配图像缓冲
    MVCC_INTVALUE value{};
    if (MV_CC_GetIntValue(handle_, "Width", &value) == MV_OK) {
        width_ = value.nCurValue;
    }
    if (MV_CC_GetIntValue(handle_, "Height", &value) == MV_OK) {
        height_ = value.nCurValue;
    }
    RCLCPP_INFO(get_logger(), "Camera resolution: %dx%d @ %.1f fps", width_, height_, fps_);

    // 读回实际生效的曝光/增益/伽马/白平衡,便于跨机器对比排查
    MVCC_FLOATVALUE exp_value{};
    if (MV_CC_GetFloatValue(handle_, "ExposureTime", &exp_value) == MV_OK) {
        RCLCPP_INFO(get_logger(), "Camera ExposureTime readback: %.1f us", exp_value.fCurValue);
    }
    MVCC_FLOATVALUE gain_value{};
    if (MV_CC_GetFloatValue(handle_, "Gain", &gain_value) == MV_OK) {
        RCLCPP_INFO(get_logger(), "Camera Gain readback: %.2f dB (range %.1f ~ %.1f)",
                    gain_value.fCurValue, gain_value.fMin, gain_value.fMax);
    }
    MVCC_FLOATVALUE gamma_value{};
    if (MV_CC_GetFloatValue(handle_, "Gamma", &gamma_value) == MV_OK) {
        RCLCPP_INFO(get_logger(), "Camera Gamma readback: %.2f", gamma_value.fCurValue);
    }
    MVCC_ENUMVALUE wb_value{};
    if (MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &wb_value) == MV_OK) {
        RCLCPP_INFO(get_logger(), "Camera BalanceWhiteAuto readback: %u (0=off, 1=continuous, 2=once)",
                    wb_value.nCurValue);
    }

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        throw std::runtime_error("MV_CC_StartGrabbing failed: " + hikError(ret));
    }
}

void HKCameraNode::captureLoop()
{
    int fail_count = 0;
    while (rclcpp::ok() && running_.load()) {
        bool converted = false;
        {
            std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
            MV_FRAME_OUT out_frame{};
            const int ret = MV_CC_GetImageBuffer(handle_, &out_frame, 1000);
            if (ret != MV_OK) {
                ++fail_count;
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "MV_CC_GetImageBuffer failed: %s", hikError(ret).c_str());
                // 连续失败时尝试重启取流，应对拔插/链路抖动
                if (fail_count >= 10) {
                    MV_CC_StopGrabbing(handle_);
                    MV_CC_StartGrabbing(handle_);
                    fail_count = 0;
                }
                continue;
            }
            fail_count = 0;
            converted = convertFrame(out_frame);
            MV_CC_FreeImageBuffer(handle_, &out_frame);
        }
        if (!converted) {
            continue;
        }

        if constexpr (kRotateImageCcw90) {
            // COW：上一条消息仍被 QoS 历史/订阅者引用时，先独占一份再写入。
            // 旋转会整帧覆盖像素，因此只复制元数据、不复制旧像素，省一次全帧 memcpy。
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
            // 旋转是纯内存操作，放到 SDK 锁外执行，不阻塞取流与曝光设置
            const auto rotate_start = std::chrono::steady_clock::now();
            rotateBgr90Ccw(
                bgr_scratch_.data(), image_msg_->data.data(), width_, height_);
            const double rotate_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - rotate_start).count();
            if (rotate_ms > 8.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "slow rotateBgr90Ccw: %.1f ms", rotate_ms);
            }
        }

        image_pub_->publish(*image_msg_);
        publishCompressed();
    }
}

void HKCameraNode::setExposureCallback(
    const std::shared_ptr<dart_interfaces::srv::SetExposure::Request> request,
    std::shared_ptr<dart_interfaces::srv::SetExposure::Response> response)
{
    if (!request || request->exposure_us <= 0.0) {
        response->success = false;
        response->message = "exposure_us must be positive";
        return;
    }

    std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
    const int ret = MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(request->exposure_us));
    if (ret != MV_OK) {
        response->success = false;
        response->message = "MV_CC_SetFloatValue(ExposureTime) failed: " + hikError(ret);
        return;
    }

    MVCC_FLOATVALUE value{};
    if (MV_CC_GetFloatValue(handle_, "ExposureTime", &value) == MV_OK) {
        exposure_us_ = value.fCurValue;
        response->applied_exposure_us = value.fCurValue;
    } else {
        exposure_us_ = request->exposure_us;
        response->applied_exposure_us = request->exposure_us;
    }
    response->success = true;
    response->message = "exposure updated";
}

bool HKCameraNode::convertFrame(const MV_FRAME_OUT & out_frame)
{
    // 设备时间戳单位为 10ns；首帧求出与主机时钟的固定偏移
    const auto & info = out_frame.stFrameInfo;
    const uint64_t dev_ns =
        ((static_cast<uint64_t>(info.nDevTimeStampHigh) << 32) |
         static_cast<uint64_t>(info.nDevTimeStampLow)) * 10U;
    if (first_frame_) {
        device_to_host_ns_ = static_cast<uint64_t>(get_clock()->now().nanoseconds()) - dev_ns;
        first_frame_ = false;
    }
    const uint64_t stamp_ns = dev_ns + device_to_host_ns_;
    image_msg_->header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000ULL);
    image_msg_->header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000ULL);

    MV_CC_PIXEL_CONVERT_PARAM param{};
    param.nWidth = info.nWidth;
    param.nHeight = info.nHeight;
    param.pSrcData = out_frame.pBufAddr;
    param.nSrcDataLen = info.nFrameLen;
    param.enSrcPixelType = info.enPixelType;
    param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    if constexpr (kRotateImageCcw90) {
        // 旋转方案：先转进与相机同方向的私有缓冲，采集线程锁外再旋转进发布消息
        param.pDstBuffer = bgr_scratch_.data();
        param.nDstBufferSize = static_cast<unsigned int>(bgr_scratch_.size());
    } else {
        // 旧直写路径（旋转关闭）：COW 后直接转进发布消息，避免多余拷贝
        if (!image_msg_.unique()) {
            image_msg_ = std::make_shared<sensor_msgs::msg::Image>(*image_msg_);
        }
        param.pDstBuffer = image_msg_->data.data();
        param.nDstBufferSize = static_cast<unsigned int>(image_msg_->data.size());
    }
    if (MV_CC_ConvertPixelType(handle_, &param) != MV_OK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "MV_CC_ConvertPixelType failed");
        return false;
    }
    return true;
}

void HKCameraNode::publishCompressed()
{
    // 压缩限频，避免每帧 JPEG 编码拖慢 160fps 采集循环；compressed_fps_ <= 0 时关闭
    if (compressed_fps_ <= 0) {
        return;
    }
    const int64_t now_ns = get_clock()->now().nanoseconds();
    const int64_t interval_ns = static_cast<int64_t>(1e9 / compressed_fps_);
    if (now_ns - last_compressed_ns_ < interval_ns) {
        return;
    }
    last_compressed_ns_ = now_ns;

    // 用 MVS SDK 自带编码器转 JPEG：同一进程内华睿 SDK 会抢占 OpenCV 的
    // libjpeg 符号，cv::imencode 会崩在华睿的 libImageSave.so 里，故回避 OpenCV。
    MV_SAVE_IMAGE_PARAM_EX3 param{};
    param.pData = image_msg_->data.data();
    param.nDataLen = static_cast<unsigned int>(image_msg_->data.size());
    param.enPixelType = PixelType_Gvsp_BGR8_Packed;
    param.nWidth = static_cast<unsigned int>(image_msg_->width);
    param.nHeight = static_cast<unsigned int>(image_msg_->height);
    param.pImageBuffer = jpeg_buffer_.data();
    param.nBufferSize = static_cast<unsigned int>(jpeg_buffer_.size());
    param.enImageType = MV_Image_Jpeg;
    // MVS 要求质量范围 (50, 99]，越界会返回错误
    param.nJpgQuality = static_cast<unsigned int>(
        std::clamp(jpeg_quality_, 50, 99));

    int encode_ret = MV_OK;
    double encode_ms = 0.0;
    {
        std::lock_guard<std::mutex> sdk_lock(sdk_mutex_);
        const auto encode_start = std::chrono::steady_clock::now();
        encode_ret = MV_CC_SaveImageEx3(handle_, &param);
        encode_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - encode_start).count();
    }
    if (encode_ms > 10.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "slow MV_CC_SaveImageEx3: %.1f ms", encode_ms);
    }
    if (encode_ret != MV_OK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "MV_CC_SaveImageEx3 failed: %s", hikError(encode_ret).c_str());
        return;
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header = image_msg_->header;  // 复用原始帧时间戳
    msg.format = "jpeg";
    msg.data.assign(jpeg_buffer_.begin(), jpeg_buffer_.begin() + param.nImageLen);
    compressed_pub_->publish(msg);
}

}  // namespace ros_camera

#include "rclcpp_components/register_node_macro.hpp"

// 以组件形式注册，可通过 component container / standalone 方式加载
RCLCPP_COMPONENTS_REGISTER_NODE(ros_camera::HKCameraNode)
