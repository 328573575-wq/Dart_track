#include "ros_camera/iray_camera.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace ros_camera
{

IRayCamera::IRayCamera(double exposure_us, double gain, double gamma, double fps)
: exposure_us_(exposure_us), gain_(gain), gamma_(gamma), fps_(fps)
{
  ret_ = IMV_EnumDevices(&device_info_list_, interfaceTypeAll);
  if (IMV_OK != ret_) {
    throw std::runtime_error("Enum Device Failed!");
  }
  if (device_info_list_.nDevNum <= 0) {
    throw std::runtime_error("No camera detected!");
  }
  ret_ = IMV_CreateHandle(&dev_handle_, modeByIndex, reinterpret_cast<void *>(&camera_index_));
  if (IMV_OK != ret_) {
    throw std::runtime_error("Create devHandle failed!");
  }

  ret_ = IMV_Open(dev_handle_);
  if (IMV_OK != ret_) {
    throw std::runtime_error("Open camera failed!");
  }

  int64_t cam_width = 0;
  int64_t cam_height = 0;
  IMV_GetIntFeatureValue(dev_handle_, "Width", &cam_width);
  IMV_GetIntFeatureValue(dev_handle_, "Height", &cam_height);
  width_ = static_cast<int>(cam_width);
  height_ = static_cast<int>(cam_height);

  dev_info_ = &device_info_list_.pDevInfo[camera_index_];
  camera_ac_sn_ = dev_info_->serialNumber;
}

IRayCamera::~IRayCamera()
{
  uninit();
  if (dev_handle_ != nullptr) {
    IMV_Close(dev_handle_);
    IMV_DestroyHandle(dev_handle_);
    dev_handle_ = nullptr;
  }
}

void IRayCamera::init()
{
  // 特性写入统一做返回值检查:相机不支持某个特性名时 SDK 会直接返回错误。
  // 之前这些返回值全被丢弃,配置没写进去也不留痕迹(静默失败)。
  const auto set_enum = [this](const char * name, const char * value) {
    const int r = IMV_SetEnumFeatureSymbol(dev_handle_, name, value);
    if (IMV_OK != r) {
      std::fprintf(stderr, "[IRay] set enum %s=%s failed: %d\n", name, value, r);
    }
  };
  const auto set_double = [this](const char * name, double value) {
    const int r = IMV_SetDoubleFeatureValue(dev_handle_, name, value);
    if (IMV_OK != r) {
      std::fprintf(stderr, "[IRay] set double %s=%.3f failed: %d\n", name, value, r);
    }
  };
  const auto set_bool = [this](const char * name, bool value) {
    const int r = IMV_SetBoolFeatureValue(dev_handle_, name, value);
    if (IMV_OK != r) {
      std::fprintf(stderr, "[IRay] set bool %s=%d failed: %d\n", name, static_cast<int>(value), r);
    }
  };
  const auto set_string = [this](const char * name, const char * value) {
    const int r = IMV_SetStringFeatureValue(dev_handle_, name, value);
    if (IMV_OK != r) {
      std::fprintf(stderr, "[IRay] set string %s=%s failed: %d\n", name, value, r);
    }
  };

  // 自动曝光与自动增益都必须显式关闭:任一处于自动模式时,相机会按画面亮度
  // 自行调参并覆盖手动写入的值,画面亮度不再受控——而下游识别/跟踪用的是绝对
  // 像素阈值(min_green_value / min_red_value),亮度漂移会直接让阈值失效。
  set_enum("TriggerMode", "Off");
  set_enum("ExposureAuto", "Off");
  set_enum("GainAuto", "Off");
  set_enum("GainSelector", "All");

  set_double("ExposureTime", exposure_us_);
  set_double("GainRaw", gain_);
  set_double("Gamma", gamma_);
  set_bool("ReverseX", false);
  set_string("DeviceUserID", "Camera");

  // 下发采集帧率:不设帧率时相机以默认(可能远高于配置)的速率出图,
  // 取流线程追不上就会在 SDK 缓存里积压旧曝光帧。
  set_bool("AcquisitionFrameRateEnable", true);
  set_double("AcquisitionFrameRate", fps_);

  // 只保留最新帧 + 最小帧缓存:否则切曝光后队列里仍排着一批旧曝光帧,
  // 要把它们取完画面才更新,表现为"切换曝光时间过长"。
  // SetBufferCount 必须在开始取流之前调用(SDK 明确不能在拉流过程中设置)。
  const int buf_ret = IMV_SetBufferCount(dev_handle_, 1);
  if (IMV_OK != buf_ret) {
    std::fprintf(stderr, "[IRay] set buffer count failed: %d\n", buf_ret);
  }
  ret_ = IMV_StartGrabbingEx(dev_handle_, 0, grabStrartegyLatestImage);
  if (IMV_OK != ret_) {
    throw std::runtime_error("IRay: Start grabbing failed!");
  }
  std::fprintf(stderr,
    "[IRay] stream started: buffer=1, strategy=latestImage, fps=%.1f\n", fps_);
}

void IRayCamera::uninit()
{
  if (dev_handle_ != nullptr) {
    IMV_StopGrabbing(dev_handle_);
  }
}

bool IRayCamera::restartStream()
{
  if (dev_handle_ == nullptr) {
    return false;
  }
  // 与海康节点的做法一致:先停再起,重建取流状态机以应对拔插/链路抖动。
  // 取流策略与缓存个数保持不变(缓存个数只能在非取流状态下设置)。
  IMV_StopGrabbing(dev_handle_);
  ret_ = IMV_StartGrabbingEx(dev_handle_, 0, grabStrartegyLatestImage);
  if (IMV_OK != ret_) {
    std::fprintf(stderr, "[IRay] restart grabbing failed: %d\n", ret_);
    return false;
  }
  return true;
}

cv::Mat IRayCamera::grab(double & capture_time_stamp_ms)
{
  cv::Mat image(cv::Size(width_, height_), CV_8UC3);
  grabInto(image.data, image.total() * image.elemSize(), capture_time_stamp_ms);
  return image;
}

void IRayCamera::grabInto(
  uint8_t * dst_buffer, std::size_t dst_buffer_size, double & capture_time_stamp_ms)
{
  static IMV_PixelConvertParam st_pixel_convert_param;
  const std::size_t expected_size = static_cast<std::size_t>(width_) *
    static_cast<std::size_t>(height_) * 3U;
  if (dst_buffer == nullptr || dst_buffer_size < expected_size) {
    throw std::runtime_error("IRay: destination image buffer is too small");
  }

  const int get_ret = IMV_GetFrame(dev_handle_, &frame_, 100);
  if (IMV_OK != get_ret) {
    // 没借到帧就绝不能调用 IMV_ReleaseFrame:归还一个不属于自己的缓冲会破坏
    // SDK 的帧池记账(同一缓冲可能被重复发放,或池子被算空)。此刻 frame_ 里
    // 装的还是上一帧的陈旧数据(首次失败时甚至是未初始化内存)。
    throw std::runtime_error("IRay: GetFrame failed");
  }

  // 时间戳必须在释放前取走:IMV_ReleaseFrame 之后该帧归 SDK 调度,
  // frameInfo 可能马上被下一帧覆盖,释放后再读会拿到撕裂的数据。
  const double stamp_ms = frame_.frameInfo.timeStamp / (1e+6);

  memset(&st_pixel_convert_param, 0, sizeof(st_pixel_convert_param));
  st_pixel_convert_param.nWidth = frame_.frameInfo.width;
  st_pixel_convert_param.nHeight = frame_.frameInfo.height;
  st_pixel_convert_param.ePixelFormat = frame_.frameInfo.pixelFormat;
  st_pixel_convert_param.pSrcData = frame_.pData;
  st_pixel_convert_param.nSrcDataLen = frame_.frameInfo.size;
  st_pixel_convert_param.nPaddingX = frame_.frameInfo.paddingX;
  st_pixel_convert_param.nPaddingY = frame_.frameInfo.paddingY;
  st_pixel_convert_param.eBayerDemosaic = demosaicNearestNeighbor;
  st_pixel_convert_param.eDstPixelFormat = convert_format_;
  st_pixel_convert_param.pDstBuf = dst_buffer;
  st_pixel_convert_param.nDstBufSize =
    sizeof(unsigned char) * frame_.frameInfo.width * frame_.frameInfo.height * 3;

  const int convert_ret = IMV_PixelConvert(dev_handle_, &st_pixel_convert_param);

  // 这一帧是我们借到的,无论转换成功与否都必须归还
  const int release_ret = IMV_ReleaseFrame(dev_handle_, &frame_);
  if (IMV_OK != release_ret) {
    std::fprintf(stderr, "[IRay] ReleaseFrame failed: %d\n", release_ret);
  }
  if (IMV_OK != convert_ret) {
    throw std::runtime_error("IRay: PixelConvert failed");
  }

  capture_time_stamp_ms = stamp_ms;
}

void IRayCamera::setExpo(double expo_us)
{
  ret_ = IMV_SetDoubleFeatureValue(dev_handle_, "ExposureTime", expo_us);
  if (IMV_OK != ret_) {
    throw std::runtime_error("set CamExpo Error");
  }

  // 曝光已改,但队列里还排着旧曝光拍下的帧,不清掉就要等它们全部取完画面才更新。
  // 调用方(setExposureCallback)已持有相机锁,与取流线程的 IMV_GetFrame 互斥。
  const int cret = IMV_ClearFrameBuffer(dev_handle_);
  if (IMV_OK != cret) {
    std::fprintf(stderr, "[IRay] clear frame buffer failed: %d\n", cret);
  }
}

double IRayCamera::getExpo()
{
  double expo = 0.0;
  ret_ = IMV_GetDoubleFeatureValue(dev_handle_, "ExposureTime", &expo);
  if (IMV_OK != ret_) {
    throw std::runtime_error("get CamExpo Error");
  }
  return expo;
}

int IRayCamera::width() const { return width_; }
int IRayCamera::height() const { return height_; }

bool IRayCamera::saveJpeg(
  const uint8_t * bgr_data, std::size_t bgr_size, int width, int height,
  int quality, const std::string & path)
{
  if (dev_handle_ == nullptr || bgr_data == nullptr || bgr_size == 0 ||
    width <= 0 || height <= 0 || path.empty()) {
    return false;
  }
  const std::size_t expected_size = static_cast<std::size_t>(width) *
    static_cast<std::size_t>(height) * 3U;
  if (bgr_size < expected_size) {
    return false;
  }

  IMV_SaveImageToFileParam save_param{};
  save_param.nWidth = static_cast<unsigned int>(width);
  save_param.nHeight = static_cast<unsigned int>(height);
  save_param.ePixelFormat = gvspPixelBGR8;
  save_param.pSrcData = const_cast<unsigned char *>(bgr_data);
  save_param.nSrcDataLen = static_cast<unsigned int>(bgr_size);
  save_param.eImageType = typeSaveJpeg;
  save_param.pImagePath = const_cast<char *>(path.c_str());
  save_param.nQuality = static_cast<unsigned int>(std::clamp(quality, 50, 99));
  save_param.eBayerDemosaic = demosaicNearestNeighbor;
  return IMV_SaveImageToFile(dev_handle_, &save_param) == IMV_OK;
}

}  // namespace ros_camera
