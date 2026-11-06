cmake_minimum_required(VERSION 3.8)
project(params_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# header-only 包：只安装头文件和统一配置文件
install(DIRECTORY include/
  DESTINATION include
)

install(FILES config.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_dependencies(
  ament_index_cpp
  yaml_cpp_vendor
)
ament_package()
