"""Launch the synthetic world and (optionally) the image viewer.

Examples
--------
    ros2 launch dart_sim sim.launch.py
    ros2 launch dart_sim sim.launch.py gui:=false save:=true
    ros2 launch dart_sim sim.launch.py time_scale:=0.5 loop:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('dart_sim')
    default_scene = os.path.join(share, 'config', 'scene.yaml')

    scene_file = LaunchConfiguration('scene_file')
    save = LaunchConfiguration('save')
    gui = LaunchConfiguration('gui')
    time_scale = LaunchConfiguration('time_scale')
    loop = LaunchConfiguration('loop')
    draw_truth = LaunchConfiguration('draw_truth')

    sim_world = Node(
        package='dart_sim',
        executable='sim_world',
        name='sim_world',
        output='screen',
        parameters=[{
            'scene_file': scene_file,
            'save': ParameterValue(save, value_type=bool),
            'time_scale': ParameterValue(time_scale, value_type=float),
            'loop': ParameterValue(loop, value_type=bool),
            'draw_truth': ParameterValue(draw_truth, value_type=bool),
        }],
    )

    sim_viewer = Node(
        package='dart_sim',
        executable='sim_viewer',
        name='sim_viewer',
        output='screen',
        condition=IfCondition(gui),
        parameters=[{
            'show_truth': ParameterValue(LaunchConfiguration('show_truth'), value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('scene_file', default_value=default_scene,
                              description='scene YAML file'),
        DeclareLaunchArgument('save', default_value='false',
                              description='save PNG frames + truth to runs/<timestamp>'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='start the OpenCV viewer'),
        DeclareLaunchArgument('show_truth', default_value='true',
                              description='overlay ground truth in the viewer'),
        DeclareLaunchArgument('time_scale', default_value='1.0',
                              description='slow motion factor (<1 is slower)'),
        DeclareLaunchArgument('loop', default_value='true',
                              description='restart the flight when it ends'),
        DeclareLaunchArgument('draw_truth', default_value='false',
                              description='draw green truth rings into the saved images'),
        sim_world,
        sim_viewer,
    ])

