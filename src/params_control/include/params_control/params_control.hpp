#pragma once

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>

namespace params_control
{

// 统一配置文件：share/params_control/config/config.yaml
inline const std::filesystem::path config_path = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("params_control")) /
    "config" / "config.yaml";

inline const YAML::Node all_config = YAML::LoadFile(config_path.string());

}  // namespace params_control

namespace recognition_package::Params
{
inline const YAML::Node config = params_control::all_config["recognition_package"];

inline const int min_green_value = config["min_green_value"].as<int>();
inline const int min_green_excess = config["min_green_excess"].as<int>();
inline const double min_area = config["min_area"].as<double>();
inline const double min_circularity = config["min_circularity"].as<double>();
inline const int init_last_tar_center_x = config["init_last_tar_center_x"].as<int>();
inline const int init_last_tar_center_y = config["init_last_tar_center_y"].as<int>();
inline const int init_track_result_x = config["init_track_result_x"].as<int>();
inline const int track_tar_roi_w = config["track_tar_roi_w"].as<int>();
inline const int track_tar_roi_h = config["track_tar_roi_h"].as<int>();
inline const int search_tar_roi_w = config["search_tar_roi_w"].as<int>();
inline const int search_tar_roi_h = config["search_tar_roi_h"].as<int>();
inline const int debug_frame_fps = config["debug_frame_fps"].as<int>();

}  // namespace recognition_package::Params

namespace dart_tracker_package::Params
{
inline const YAML::Node config = params_control::all_config["dart_tracker"];

inline const int motion_difference_threshold = config["motion_difference_threshold"].as<int>();
inline const int track_tar_roi_w = config["track_tar_roi_w"].as<int>();
inline const int track_tar_roi_h = config["track_tar_roi_h"].as<int>();
inline const int search_tar_roi_w = config["search_tar_roi_w"].as<int>();
inline const int search_tar_roi_h = config["search_tar_roi_h"].as<int>();
inline const int debug_frame_fps = config["debug_frame_fps"].as<int>();
inline const int min_red_value = config["min_red_value"].as<int>();
inline const int min_red_excess = config["min_red_excess"].as<int>();
inline const int min_blue_value = config["min_blue_value"].as<int>();
inline const int min_blue_excess = config["min_blue_excess"].as<int>();
inline const int finish_pix_above = config["finish_pix_above"].as<int>();
inline const int init_last_tar_center_x = config["init_last_tar_center_x"].as<int>();
inline const int init_last_tar_center_y = config["init_last_tar_center_y"].as<int>();
inline const int max_track_lost_cnt = config["max_track_lost_cnt"].as<int>();
inline const double min_area = config["min_area"].as<double>();

}  // namespace dart_tracker_package::Params

namespace camera_mode::Params
{
inline const YAML::Node config = params_control::all_config["camera_mode"];

inline const double recognition_exposure_time = config["recognition_exposure_time"].as<double>();
inline const double track_exposure_time = config["track_exposure_time"].as<double>();
inline const double track_timeout_sec = config["track_timeout_sec"].as<double>();

}  // namespace camera_mode::Params

namespace ros_camera::IRayParams
{
inline const YAML::Node config = params_control::all_config["iray_camera"];

inline const std::string frame_id = config["frame_id"].as<std::string>();
inline const int fps = config["fps"].as<int>();
inline const double gain = config["gain"].as<double>();
inline const double gamma = config["gamma"].as<double>();
inline const int jpeg_quality = config["jpeg_quality"].as<int>();
inline const int compressed_fps = config["compressed_fps"].as<int>();

}  // namespace ros_camera::IRayParams

namespace video_test_package::Params
{
inline const YAML::Node config = params_control::all_config["video_test"];

inline const std::string video_path = config["video_path"].as<std::string>();
inline const std::string frame_id = config["frame_id"].as<std::string>();
inline const int fps = config["fps"].as<int>();
inline const bool loop = config["loop"].as<bool>();

}  // namespace video_test_package::Params

namespace serial_bridge::Params
{
inline const YAML::Node config = params_control::all_config["serial_bridge"];

inline const std::string serial_port = config["serial_port"].as<std::string>();
inline const int baud_rate = config["baud_rate"].as<int>();
inline const int timeout_ms = config["timeout_ms"].as<int>();
inline const int target_cmd = config["target_cmd"].as<int>();
inline const int state_cmd = config["state_cmd"].as<int>();

}  // namespace serial_bridge::Params

namespace ros_camera::HKParams
{
inline const YAML::Node config = params_control::all_config["hk_camera"];

inline const int width = config["width"].as<int>();
inline const int height = config["height"].as<int>();
inline const double fps = config["fps"].as<double>();
inline const double gain = config["gain"].as<double>();
inline const std::string camera_acsn = config["camera_acsn"].as<std::string>();
inline const int balance_white_auto = config["balance_white_auto"].as<int>();
inline const std::string frame_id = config["frame_id"].as<std::string>();
inline const int jpeg_quality = config["jpeg_quality"].as<int>();
inline const int compressed_fps = config["compressed_fps"].as<int>();

}  // namespace ros_camera::HKParams
