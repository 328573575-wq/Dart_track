import os

from ament_index_python.packages import get_package_share_directory

# 两条流水线的差异全部集中在这里。
#   图像/检测话题、内参文件、灯坐标文件、TF 坐标系名
# 相机本身的参数（序列号、分辨率、帧率）在 ros_camera/config/camera_*.yaml，
# 那边的话题名要和这里的 image_topic 对齐。
_SHARE = get_package_share_directory('dart_track')
_CONFIG = os.path.join(_SHARE, 'config')

PIPELINES = [
    {
        'index': 1,
        'camera_file': os.path.join(_CONFIG, 'ost1.yaml'),
        'lights_file': os.path.join(_CONFIG, 'real_light_centers_1.yaml'),
        'image_topic': '/cam1/image_raw',
        # 识别吃 raw（精度优先；同容器下是零拷贝，不比压缩慢）
        # 想改成压缩流调试就设 use_compressed: True
        'compressed_topic': '/cam1/compressed',
        'exposure_topic': '/cam1/exposure',
        'use_compressed': False,
        'detections_topic': '/cam1/dart_detections',
        'camera_frame': 'camera1',
        'draw_topic': '/cam1/draw',
        # 绘制结果的分辨率系数：0.5 = 一半（1440x1080 -> 720x540）。
        # 想看清细节可以调 1.0，但带宽/CPU 会翻倍。
        'draw_scale': 0.5,
        # 相机大致安装位置 [x,y,z]（throw 系），可留空。
        # 四个灯是标准矩形时，PnP 有 180° 的二义性，填了这里就能唯一确定。
        'expected_position': [],
    },
    {
        'index': 2,
        'camera_file': os.path.join(_CONFIG, 'ost2.yaml'),
        'lights_file': os.path.join(_CONFIG, 'real_light_centers_2.yaml'),
        'image_topic': '/cam2/image_raw',
        'compressed_topic': '/cam2/compressed',
        'exposure_topic': '/cam2/exposure',
        'use_compressed': False,
        'detections_topic': '/cam2/dart_detections',
        'camera_frame': 'camera2',
        'draw_topic': '/cam2/draw',
        'draw_scale': 0.5,
        'expected_position': [],
    },
]
