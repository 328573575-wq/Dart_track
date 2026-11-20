import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import ComposableNodeContainer

sys.path.append(os.path.dirname(__file__))
from composable_node import *


def generate_launch_description():
    # 这份 launch 只起"算法节点"：不含相机驱动，也不含仿真。
    #   真机：ros2 launch ros_camera hk_camera_dual.launch.py  先起相机
    #   仿真：ros2 launch dart_sim  sim.launch.py              先起仿真
    #   一条命令把两者组合起来：~/dart_ws/launch/dart_track.launch.py
    #
    # use_sim 在这里不影响启停，只用来告诉 solvepose：
    # 图像来自仿真（用仿真里的相机位置消除灯阵二义性）还是真实相机。
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='true: 图像来自 dart_sim（只影响位姿二义性的处理方式）',
    )
    plane_fallback_arg = DeclareLaunchArgument(
        'plane_fallback',
        default_value='false',
        description='true: 只有一路检测时退回平面假设（单目精度）',
    )

    hint = LogInfo(msg=(
        '只启动算法节点：识别/绘图/PnP/融合，等待 /cam1 /cam2 图像。'
        '真机请先起 ros_camera 的 hk_camera_dual.launch.py，'
        '或直接用工作空间的 launch/dart_track.launch.py（自动组合）。'))

    # 两条流水线，各自一个容器：一台相机出问题不影响另一条
    cam1_container = ComposableNodeContainer(
        name='dart_track_container_1',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=pipeline_1,
        output='screen',
    )

    cam2_container = ComposableNodeContainer(
        name='dart_track_container_2',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=pipeline_2,
        output='screen',
    )

    # 融合 + 可视化：不属于任何一条流水线
    fusion_container = ComposableNodeContainer(
        name='fusion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[position_node, marker_node],
        output='screen',
    )

    return LaunchDescription([
        use_sim_arg,
        plane_fallback_arg,
        hint,
        cam1_container,
        cam2_container,
        fusion_container,
    ])