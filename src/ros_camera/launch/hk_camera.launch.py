# Copyright (c) 2026 Dart. Licensed under MIT.
#
# 海康相机节点启动文件：以组件方式在容器内加载 ros_camera::HKCameraNode。
# 相机参数统一从 params_control/config.yaml 的 [hk_camera] 段读取，无需额外参数文件。
#
# 用法：
#   ros2 launch ros_camera hk_camera.launch.py
#   # 关闭进程内零拷贝通信：
#   ros2 launch ros_camera hk_camera.launch.py intra_process:=false

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    intra_process = LaunchConfiguration("intra_process")

    return LaunchDescription([
        DeclareLaunchArgument(
            "intra_process",
            default_value="true",
            description="Enable intra-process zero-copy communication",
        ),
        ComposableNodeContainer(
            name="hk_camera_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="ros_camera",
                    plugin="ros_camera::HKCameraNode",
                    name="hk_camera_node",
                    extra_arguments=[{"use_intra_process_comms": intra_process}],
                ),
            ],
            output="screen",
        ),
    ])
