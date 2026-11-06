#pragma once

#include "IMVApi.h"
#include "IMVDefines.h"

#include <opencv2/opencv.hpp>

#include <cstddef>
#include <cstdint>
#include <string>

namespace ros_camera
{

class IRayCamera
{
public:
  IRayCamera(double exposure_us, double gain, double gamma, double fps);
  ~IRayCamera();

  void init();
  cv::Mat grab(double & capture_time_stamp_ms);
  void grabInto(uint8_t * dst_buffer, std::size_t dst_buffer_size, double & capture_time_stamp_ms);
  void setExpo(double expo_us);
  double getExpo();
  void uninit();
  // 重启取流:取流连续失败(拔插/链路抖动)后重新 Stop/Start,失败返回 false
  bool restartStream();
  bool saveJpeg(const uint8_t * bgr_data, std::size_t bgr_size, int width, int height,
    int quality, const std::string & path);

  int width() const;
  int height() const;

private:
  void * dev_handle_{nullptr};
  int ret_{IMV_OK};
  IMV_DeviceList device_info_list_;
  IMV_DeviceInfo * dev_info_{nullptr};
  IMV_Frame frame_;
  IMV_EPixelType convert_format_{gvspPixelBGR8};
  unsigned int camera_index_{0};

  int width_{0};
  int height_{0};
  std::string camera_ac_sn_;

  double exposure_us_{2000.0};
  double gain_{2.0};
  double gamma_{1.0};
  double fps_{120.0};
};

}  // namespace ros_camera
