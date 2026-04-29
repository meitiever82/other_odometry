#!/usr/bin/env python3
"""极简腿式里程计 launch: simple_leg_odom_node。

  ros2 launch leg_odometry simple_leg_odom.launch.py \
      urdf_path:=/abs/path/to.urdf \
      effort_threshold_left:=5.0 \
      effort_threshold_right:=12.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_URDF = os.environ.get(
    'CASBOT_URDF',
    os.path.expanduser('~/casbot_ws/urdf/casbot02_7dof_shell.urdf'),
)


def _spawn(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'URDF not found: {urdf_path}')

    def _d(name):
        return float(LaunchConfiguration(name).perform(context))

    params = {
        'urdf_path': urdf_path,
        'effort_threshold_left':  _d('effort_threshold_left'),
        'effort_threshold_right': _d('effort_threshold_right'),
        'effort_hysteresis':      _d('effort_hysteresis'),
        'tilt_alpha':             _d('tilt_alpha'),
        'init_frames':            int(_d('init_frames')),
    }

    return [
        Node(
            package='leg_odometry',
            executable='simple_leg_odom_node',
            name='simple_leg_odom',
            output='screen',
            parameters=[params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value=DEFAULT_URDF),
        DeclareLaunchArgument('effort_threshold_left',  default_value='5.0'),
        DeclareLaunchArgument('effort_threshold_right', default_value='12.0',
            description='右脚静态 effort bias ~11.5Nm, 阈值调高'),
        DeclareLaunchArgument('effort_hysteresis', default_value='1.0'),
        DeclareLaunchArgument('tilt_alpha', default_value='0.01',
            description='roll/pitch 向 accel 重力收敛系数 (0=纯 gyro, 1=纯 accel)'),
        DeclareLaunchArgument('init_frames', default_value='50'),
        OpaqueFunction(function=_spawn),
    ])
