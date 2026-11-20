"""两台海康相机的启动文件
    1 号相机 -> /cam1/image_raw   参数 ros_camera/config/camera_1.yaml
    2 号相机 -> /cam2/image_raw   参数 ros_camera/config/camera_2.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _camera_container(index, params_file):
    return ComposableNodeContainer(
        name=f'hk_camera_container_{index}',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ros_camera',
                plugin='ros_camera::HKCameraNode',
                name=f'hk_camera_node_{index}',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ros_camera'), 'config')
    return LaunchDescription([
        _camera_container(1, os.path.join(config, 'camera_1.yaml')),
        _camera_container(2, os.path.join(config, 'camera_2.yaml')),
    ])