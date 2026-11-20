import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# 复用 dart_track 包里的节点定义（话题、参数都在那边维护，这里不重复写一遍）
_TRACK_LAUNCH_DIR = os.path.join(get_package_share_directory('dart_track'), 'launch')
sys.path.insert(0, _TRACK_LAUNCH_DIR)
import composable_node as track_nodes  # noqa: E402

use_sim = LaunchConfiguration('use_sim')


def _camera_node(index):
    """相机节点：只有真机模式才实例化（仿真时这个条件为假）。"""
    cam_config = os.path.join(
        get_package_share_directory('ros_camera'), 'config', f'camera_{index}.yaml')
    return ComposableNode(
        package='ros_camera',
        plugin='ros_camera::HKCameraNode',
        name=f'hk_camera_node_{index}',
        parameters=[cam_config],
        condition=UnlessCondition(use_sim),
        extra_arguments=[{'use_intra_process_comms': True}],
    )


def _pipeline_container(index, nodes):
    # 相机 + 识别 + 解算放在同一个进程里：图像走 intra-process，不序列化、不拷贝
    return ComposableNodeContainer(
        name=f'pipeline_container_{index}',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[_camera_node(index)] + nodes,
        output='screen',
    )


def generate_launch_description():
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='true: 用 dart_sim 出图（不开相机）; false: 两台海康相机',
    )
    plane_fallback_arg = DeclareLaunchArgument(
        'plane_fallback',
        default_value='false',
        description='true: 只有一路检测时退回平面假设（单目精度）',
    )

    pipelines = [track_nodes.pipeline_1, track_nodes.pipeline_2]
    containers = [_pipeline_container(i + 1, nodes) for i, nodes in enumerate(pipelines)]

    fusion_container = ComposableNodeContainer(
        name='fusion_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[track_nodes.position_node, track_nodes.marker_node],
        output='screen',
    )

    # 仿真：图像源用 dart_sim，内参和算法层保持同一份
    track_config = os.path.join(get_package_share_directory('dart_track'), 'config')
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('dart_sim'), 'launch', 'sim.launch.py')),
        launch_arguments={
            'camera_file': os.path.join(track_config, 'ost1.yaml'),
            'camera_file2': os.path.join(track_config, 'ost2.yaml'),
        }.items(),
        condition=IfCondition(use_sim),
    )

    return LaunchDescription([
        use_sim_arg,
        plane_fallback_arg,
        LogInfo(msg='每条流水线一个容器：相机 + 识别 + 解算（进程内零拷贝）',
                condition=UnlessCondition(use_sim)),
        LogInfo(msg='仿真模式：相机节点不启动，图像由 dart_sim 提供',
                condition=IfCondition(use_sim)),
        *containers,
        fusion_container,
        sim_launch,
    ])