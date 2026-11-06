cmake_minimum_required(VERSION 3.8)
project(dart_track)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(dart_track_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dart_point_interfaces REQUIRED)

add_library(dart_track_component SHARED
  src/recognition_node.cpp
  src/draw_screen_node.cpp
  src/solvepose_node.cpp
  src/position_node.cpp
)

target_include_directories(dart_track_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dart_track_component
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  dart_track_interfaces
  dart_point_interfaces
)

target_link_libraries(dart_track_component ${OpenCV_LIBS})
target_include_directories(dart_track_component PRIVATE ${OpenCV_INCLUDE_DIRS})

rclcpp_components_register_nodes(dart_track_component
  "dart_track::RecognitionNode"
  "dart_track::DrawScreenNode"
  "dart_track::SolvePoseNode"
  "dart_track::PositionNode"
)

install(TARGETS dart_track_component
  LIBRARY DESTINATION lib
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
