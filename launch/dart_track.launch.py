import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer

sys.path.append(os.path.dirname(__file__))
from composable_node import *


def generate_launch_description():
    # use_sim:=false  容器内直接起海康相机
    # use_sim:=true   不起相机，改用 dart_sim 出图
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='false: 容器内起海康相机; true: 用 dart_sim 仿真出图',
    )

    dart_track_node = [
        hk_camera_node,
        recognition_node,
        draw_node,
        solvepose_node,
        position_node,
        marker_node
    ]

    dart_track_container = ComposableNodeContainer(
        name='dart_track_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=dart_track_node,
        output='screen',
    )

    return LaunchDescription([
        use_sim_arg,
        dart_track_container,
        sim_node,
    ])
