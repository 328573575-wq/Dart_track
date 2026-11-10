import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

sys.path.append(os.path.join(get_package_share_directory('dart_track'), 'launch'))
from params import *

use_sim = LaunchConfiguration('use_sim')


hk_camera_node = ComposableNode(
    package='ros_camera',
    plugin='ros_camera::HKCameraNode',
    name='hk_camera_node',
    condition=UnlessCondition(use_sim),
    extra_arguments=[{'use_intra_process_comms': True}]
)

recognition_node = ComposableNode(
    package='dart_track',
    plugin='dart_track::RecognitionNode',
    name='recognition_node',
    parameters=[recognition_config],
    extra_arguments=[{'use_intra_process_comms': True}]
)

draw_node = ComposableNode(
    package='dart_track',
    plugin='dart_track::DrawScreenNode',
    name='draw_screen_node',
    parameters=[{'draw_max_fps': 60.0, 'draw_scale': 0.5, 'draw_det_max_age': 0.3}],
    extra_arguments=[{'use_intra_process_comms': True}]
)

solvepose_node = ComposableNode(
    package='dart_track',
    plugin='dart_track::SolvePoseNode',
    name='solvepose_node',
    parameters=[real_light_centers],
    extra_arguments=[{'use_intra_process_comms': True}]
)

position_node = ComposableNode(
    package='dart_track',
    plugin='dart_track::PositionNode',
    name='position_node',
    extra_arguments=[{'use_intra_process_comms': True}]
)

# 可视化：把 /dart_point 转成 Foxglove 3D 面板认的 Marker
marker_node = ComposableNode(
    package='dart_track',
    plugin='dart_track::MarkerNode',
    name='marker_node',
    parameters=[real_light_centers],
    extra_arguments=[{'use_intra_process_comms': True}]
)

sim_node = Node(
    package='dart_sim',
    executable='sim_world',
    name='sim_world',
    output='screen',
    condition=IfCondition(use_sim),
    parameters=[{
        'save': False,
        'loop': True,
        'time_scale': 1.0,
        'draw_truth': False,
    }],
)
