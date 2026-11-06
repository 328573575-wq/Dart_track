cmake_minimum_required(VERSION 3.8)
project(ros_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(params_control REQUIRED)
find_package(dart_interfaces REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 相机 SDK 均为可选依赖：对应的节点只在 SDK 存在时参与构建，
# 保证缺任一 SDK 的环境（如仅装 MVS 的 WSL）仍可正常编译其余节点。
set(ROS_CAMERA_SOURCES)
set(ROS_CAMERA_COMPONENTS)

# ---------- Huaray (iray) SDK ----------
if(EXISTS /opt/HuarayTech/MVviewer/include/IMVApi.h)
  find_library(
    HUARAY_TECH_LIB
    NAMES MVSDK libMVSDK.so IRay libIRay.so
    PATHS
      /opt/HuarayTech/MVviewer/lib/m64x86
      /opt/HuarayTech/MVviewer/lib
    NO_DEFAULT_PATH
  )
  if(HUARAY_TECH_LIB)
    list(APPEND ROS_CAMERA_SOURCES
      src/iray_camera.cpp
      src/iray_camera_node.cpp
    )
    list(APPEND ROS_CAMERA_COMPONENTS "ros_camera::IRayCameraNode")
  else()
    message(WARNING "Huaray SDK headers found but library missing; iray camera node will not be built.")
  endif()
else()
  message(WARNING "Huaray SDK not found at /opt/HuarayTech/MVviewer; iray camera node will not be built.")
endif()

# ---------- Hikvision MVS SDK ----------
if(EXISTS /opt/MVS/include/MvCameraControl.h)
  find_library(
    MV_CAMERA_CONTROL_LIB
    NAMES MvCameraControl
    PATHS /opt/MVS/lib/64
    NO_DEFAULT_PATH
  )
  if(MV_CAMERA_CONTROL_LIB)
    list(APPEND ROS_CAMERA_SOURCES src/hk_camera_node.cpp)
    list(APPEND ROS_CAMERA_COMPONENTS "ros_camera::HKCameraNode")
  else()
    message(WARNING "MVS headers found but libMvCameraControl is missing; hk_camera_node will not be built.")
  endif()
else()
  message(WARNING "Hikvision MVS SDK not found at /opt/MVS; hk_camera_node will not be built.")
endif()

# 兜底：一个相机 SDK 都没有时，ROS_CAMERA_SOURCES 是空的，
# add_library 会以 "No SOURCES given to target" 直接失败，连带整个 colcon build 变红。
# 这里塞一个空源文件让库仍能生成（只是里面不含任何相机节点），
# 装上 /opt/MVS 或 /opt/HuarayTech/MVviewer 后本分支自动跳过。
if(NOT ROS_CAMERA_SOURCES)
  message(WARNING
    "No camera SDK found; ros_camera builds an empty component library. "
    "Install /opt/MVS (Hikvision) or /opt/HuarayTech/MVviewer (Huaray) to enable a camera node.")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/no_camera_sdk.cpp"
    "// placeholder: no camera SDK found at configure time; see CMakeLists.txt\n")
  list(APPEND ROS_CAMERA_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/no_camera_sdk.cpp")
endif()

add_library(ros_camera_component SHARED ${ROS_CAMERA_SOURCES})

ament_target_dependencies(ros_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  params_control
  dart_interfaces
)

target_link_libraries(ros_camera_component ${OpenCV_LIBS})

if(HUARAY_TECH_LIB)
  target_include_directories(ros_camera_component PRIVATE /opt/HuarayTech/MVviewer/include)
  target_link_libraries(ros_camera_component ${HUARAY_TECH_LIB})
endif()

if(MV_CAMERA_CONTROL_LIB)
  target_include_directories(ros_camera_component PRIVATE /opt/MVS/include)
  target_link_libraries(ros_camera_component ${MV_CAMERA_CONTROL_LIB})
endif()

rclcpp_components_register_nodes(ros_camera_component ${ROS_CAMERA_COMPONENTS})

install(TARGETS ros_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 运行时把 MVS SDK 库目录加入 LD_LIBRARY_PATH，见 env-hooks/ros_camera.dsv.in
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ros_camera.dsv.in")

ament_export_include_directories(include)
ament_export_libraries(ros_camera_component)
ament_package()
