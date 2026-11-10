"""Launch the synthetic world together with its viewer and scoring node.

``sim_world`` 自己会把显示窗口（sim_viewer）和打分（sim_eval）拉起来，
所以上层只需要关心「开不开仿真」；下面这些参数只是给调试用的开关。

Examples
--------
    ros2 launch dart_sim sim.launch.py
    ros2 launch dart_sim sim.launch.py gui:=false            # 不要窗口
    ros2 launch dart_sim sim.launch.py eval:=false           # 不要打分
    ros2 launch dart_sim sim.launch.py time_scale:=0.5 loop:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('dart_sim')
    default_scene = os.path.join(share, 'config', 'scene.yaml')

    default_lights = os.path.join(share, 'config', 'real_light_centers.yaml')
    default_camera = os.path.join(share, 'config', 'ost.yaml')

    scene_file = LaunchConfiguration('scene_file')
    lights_file = LaunchConfiguration('lights_file')
    camera_file = LaunchConfiguration('camera_file')
    save = LaunchConfiguration('save')
    gui = LaunchConfiguration('gui')
    show_truth = LaunchConfiguration('show_truth')
    eval_enabled = LaunchConfiguration('eval')
    time_scale = LaunchConfiguration('time_scale')
    loop = LaunchConfiguration('loop')
    draw_truth = LaunchConfiguration('draw_truth')

    # 只起一个进程：图像、真值、窗口、打分都在 sim_world 里
    sim_world = Node(
        package='dart_sim',
        executable='sim_world',
        name='sim_world',
        output='screen',
        parameters=[{
            'scene_file': scene_file,
            'lights_file': lights_file,
            'camera_file': camera_file,
            'save': ParameterValue(save, value_type=bool),
            'viewer': ParameterValue(gui, value_type=bool),
            'show_truth': ParameterValue(show_truth, value_type=bool),
            'eval': ParameterValue(eval_enabled, value_type=bool),
            'time_scale': ParameterValue(time_scale, value_type=float),
            'loop': ParameterValue(loop, value_type=bool),
            'draw_truth': ParameterValue(draw_truth, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('scene_file', default_value=default_scene,
                              description='scene YAML file (camera pose, dart, render)'),
        DeclareLaunchArgument('lights_file', default_value=default_lights,
                              description='guide light coordinates (same format as '
                                          'dart_track/config/real_light_centers.yaml)'),
        DeclareLaunchArgument('camera_file', default_value=default_camera,
                              description='camera intrinsics (ost.yaml format)'),
        DeclareLaunchArgument('save', default_value='false',
                              description='save PNG frames + truth to runs/<timestamp>'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='show the OpenCV viewer window'),
        DeclareLaunchArgument('show_truth', default_value='true',
                              description='overlay ground truth in the viewer'),
        DeclareLaunchArgument('eval', default_value='true',
                              description='print the per-frame score (needs dart_point_interfaces '
                                          'and the measurement nodes running)'),
        DeclareLaunchArgument('time_scale', default_value='1.0',
                              description='slow motion factor (<1 is slower)'),
        DeclareLaunchArgument('loop', default_value='true',
                              description='restart the flight when it ends'),
        DeclareLaunchArgument('draw_truth', default_value='false',
                              description='draw green truth rings into the saved images'),
        sim_world,
    ])
