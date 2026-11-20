import os
import sys

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

sys.path.append(os.path.dirname(__file__))
from params import *

use_sim = LaunchConfiguration('use_sim')

# ---------------------------------------------------------------------------
# 分层设计：这个文件只编排"算法节点"，不含相机驱动。
#
#   图像源（ros_camera 的双相机 launch，或 dart_sim）先起来，往
#   /cam1/image_raw、/cam2/image_raw 发布图像；
#   这里每个容器装一条流水线：识别 -> 绘图 -> PnP(TF)；
#   fusion 容器里的 position 节点把两路检测做成射线交会 -> /dart_point。
#
# 一条流水线里所有节点只差"话题名、内参文件路径、灯坐标文件路径、TF 坐标系名"，
# 取值在 params/pipelines.py 里，一个配置就是一个字典，改那里就行。
# ---------------------------------------------------------------------------


def make_recognition_node(cfg):
    return ComposableNode(
        package='dart_track',
        plugin='dart_track::RecognitionNode',
        name=f'recognition_node_{cfg["index"]}',
        parameters=[recognition_config, {
            'image_topic': cfg['image_topic'],
            'compressed_topic': cfg['compressed_topic'],
            'exposure_topic': cfg['exposure_topic'],
            'use_compressed': cfg['use_compressed'],
            'detections_topic': cfg['detections_topic'],
            # 识别 + 绘图在同一个节点里，画好的图发到这个话题
            'draw_topic': cfg['draw_topic'],
            'draw_scale': cfg.get('draw_scale', 0.5),
            'draw_enable': True,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )


def make_solvepose_node(cfg, expected_position=None, condition=None):
    params = {
        'detections_topic': cfg['detections_topic'],
        'camera_file': cfg['camera_file'],
        'camera_frame': cfg['camera_frame'],
        'parent_frame': 'throw',
    }
    expected = expected_position or cfg.get('expected_position') or []
    if len(expected) == 3:
        params['expected_position'] = [float(v) for v in expected]
    return ComposableNode(
        package='dart_track',
        plugin='dart_track::SolvePoseNode',
        name=f'solvepose_node_{cfg["index"]}',
        parameters=[cfg['lights_file'], params],
        condition=condition,
        extra_arguments=[{'use_intra_process_comms': True}],
    )


def _sim_camera_eyes():
    """仿真里两台相机的真实安装位置，用来消除对称灯阵的位姿二义性。"""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('dart_sim'), 'config', 'scene.yaml')
        with open(path, 'r', encoding='utf-8') as handle:
            cfg = yaml.safe_load(handle)
        eyes = [cfg.get('camera', {}).get('eye')]
        eyes.append((cfg.get('camera2') or {}).get('eye'))
        return eyes
    except Exception:
        return [None, None]


_SIM_EYES = _sim_camera_eyes()


def make_pipeline(cfg):
    eye = _SIM_EYES[cfg['index'] - 1] if cfg['index'] - 1 < len(_SIM_EYES) else None
    return [
        make_recognition_node(cfg),
        # 真机用 yaml 里配的 expected_position；仿真用 scene.yaml 里的真实位置
        make_solvepose_node(cfg, condition=UnlessCondition(use_sim)),
        make_solvepose_node(cfg, expected_position=eye, condition=IfCondition(use_sim)),
    ]


pipeline_1 = make_pipeline(PIPELINES[0])
pipeline_2 = make_pipeline(PIPELINES[1])

# 融合节点：不属于任何一条流水线，单独放在第三个容器里
position_node = ComposableNode(
    package='dart_track',
    plugin='dart_track::PositionNode',
    name='position_node',
    parameters=[{
        'detections_topic_1': PIPELINES[0]['detections_topic'],
        'detections_topic_2': PIPELINES[1]['detections_topic'],
        'camera_file_1': PIPELINES[0]['camera_file'],
        'camera_file_2': PIPELINES[1]['camera_file'],
        'camera_frame_1': PIPELINES[0]['camera_frame'],
        'camera_frame_2': PIPELINES[1]['camera_frame'],
        'parent_frame': 'throw',
        'min_ray_angle_deg': 2.0,
        'max_stamp_diff': 0.05,
        # 仿真只出一路图，允许退回平面假设；真机保持 false，逼你发现第二台没起来
        'allow_plane_fallback': ParameterValue(
            LaunchConfiguration('plane_fallback'), value_type=bool),
    }],
    extra_arguments=[{'use_intra_process_comms': True}],
)

# 可视化：把 /dart_point 转成 Foxglove 3D 面板认的 Marker（两条流水线共用）
marker_node = ComposableNode(
    package='dart_track',
    plugin='dart_track::MarkerNode',
    name='marker_node',
    parameters=[PIPELINES[0]['lights_file']],
    extra_arguments=[{'use_intra_process_comms': True}],
)
