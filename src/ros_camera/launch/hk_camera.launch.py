"""单相机启动文件：打开"当前检测到的第一台"海康相机。

用途：标定（camera_calibration / 手眼标定）、单独看画面、录数据。
它不读 camera_1.yaml / camera_2.yaml，也不关心双相机流水线，
参数默认值直接写在节点里（1440x1080 @ 30fps，话题 /camera/image_raw）。

双相机工作时用 hk_camera_dual.launch.py，不要同时开这个
（同一台相机不能被两个进程同时打开）。

用法：
    ros2 launch ros_camera hk_camera.launch.py
    # 指定某一台（多台相机插着时，按序列号挑）
    ros2 launch ros_camera hk_camera.launch.py camera_acsn:=DA0498835
    # 换话题名 / 帧率 / 曝光
    ros2 launch ros_camera hk_camera.launch.py image_topic:=/calib/image_raw fps:=15.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_acsn = LaunchConfiguration('camera_acsn')
    image_topic = LaunchConfiguration('image_topic')
    fps = LaunchConfiguration('fps')
    exposure_us = LaunchConfiguration('exposure_us')
    intra_process = LaunchConfiguration('intra_process')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_acsn', default_value='',
            description='相机序列号；留空 = 用枚举到的第一台'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/image_raw',
            description='图像话题（标定工具默认订阅 /image_raw 之类，按需改）'),
        DeclareLaunchArgument(
            'fps', default_value='30.0', description='采集帧率'),
        DeclareLaunchArgument(
            'exposure_us', default_value='4000.0', description='曝光时间 (us)'),
        DeclareLaunchArgument(
            'intra_process', default_value='true',
            description='进程内零拷贝通信'),

        ComposableNodeContainer(
            name='hk_camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='ros_camera',
                    plugin='ros_camera::HKCameraNode',
                    name='hk_camera_node',
                    parameters=[{
                        'camera_acsn': camera_acsn,
                        'image_topic': image_topic,
                        'fps': ParameterValue(fps, value_type=float),
                        'exposure_us': ParameterValue(exposure_us, value_type=float),
                        # 单相机调试：压缩话题默认关掉，需要时在命令行打开
                        'compressed_fps': 0,
                    }],
                    extra_arguments=[{'use_intra_process_comms': intra_process}],
                ),
            ],
            output='screen',
        ),
    ])