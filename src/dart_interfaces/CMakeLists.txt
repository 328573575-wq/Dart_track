cmake_minimum_required(VERSION 3.8)
project(dart_interfaces)

# 统一接口包：集中提供全工作区的 msg / srv
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/DetectionResult.msg"
  "msg/TrackResult.msg"
  "msg/McuState.msg"
  "msg/SerialStatus.msg"
  "srv/SetExposure.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
