import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer

sys.path.append(os.path.dirname(__file__))
from composable_node import *


def generate_launch_description():
    # use_sim:=true  额外启动 dart_sim 仿真出图；图像源由外部提供（相机/仿真/录包）
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='true: 同时启动 dart_sim 仿真出图; false: 只起测量节点，等外部相机',
    )

    dart_track_node = [
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
