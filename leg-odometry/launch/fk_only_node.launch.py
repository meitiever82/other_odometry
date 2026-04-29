#!/usr/bin/env python3
"""Launch the FK-only leg odometry node.

State: p, R, gyro_bias.  No Kalman filter, no IMU accel in position.

Pipeline: gyro → R, FK·q̇ → v_foot_body, Mahony accel-tilt on R,
FlatZ clamp on p.z, optional heel-toe rolling compensation on v_foot_body.x.

Usage (forward walking):
    ros2 launch leg_odometry fk_only_node.launch.py \\
        urdf_path:=/abs/path/to/casbot02.urdf \\
        foot_roll_toe_offset:=0.20

Usage (in-place spinning or unknown gait):
    ros2 launch leg_odometry fk_only_node.launch.py \\
        urdf_path:=/abs/path/to/casbot02.urdf \\
        foot_roll_toe_offset:=0.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_URDF = os.environ.get(
    'CASBOT_URDF',
    os.path.expanduser(
        '~/rtabmap_ws/src/finder_lidar_mapping/glim_ros2/urdf/casbot02_7dof_shell.urdf'
    ),
)


def _spawn(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'URDF not found: {urdf_path}')

    def _d(name):
        return float(LaunchConfiguration(name).perform(context))

    def _s(name):
        return LaunchConfiguration(name).perform(context)

    def _b(name):
        return LaunchConfiguration(name).perform(context).lower() == 'true'

    params = {
        'urdf_path':              urdf_path,
        'effort_threshold_left':  _d('effort_threshold_left'),
        'effort_threshold_right': _d('effort_threshold_right'),
        'effort_hysteresis':      _d('effort_hysteresis'),
        'bias_window_sec':        _d('bias_window_sec'),
        'tilt_kp':                _d('tilt_kp'),
        'tilt_accel_band':        _d('tilt_accel_band'),
        'flatz_enabled':          _b('flatz_enabled'),
        'flatz_alpha':            _d('flatz_alpha'),
        'foot_roll_toe_offset':   _d('foot_roll_toe_offset'),
        'foot_roll_sign':         _d('foot_roll_sign'),
        'publish_tf':             _b('publish_tf'),
        'odom_frame':             _s('odom_frame'),
        'base_frame':             _s('base_frame'),
        'odom_topic':             _s('odom_topic'),
    }

    return [
        Node(
            package='leg_odometry',
            executable='fk_only_node',
            name='fk_only_odom',
            output='screen',
            parameters=[params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path',
            default_value=DEFAULT_URDF),

        # --- contact detection ---
        DeclareLaunchArgument('effort_threshold_left',  default_value='5.0'),
        DeclareLaunchArgument('effort_threshold_right', default_value='5.0',
            description='symmetric by default; set higher for a leg with static torque bias'),
        DeclareLaunchArgument('effort_hysteresis', default_value='1.0'),

        # --- init ---
        DeclareLaunchArgument('bias_window_sec', default_value='3.0',
            description='static pre-motion window (s) for gyro-bias + gravity init'),

        # --- accel-tilt Mahony ---
        DeclareLaunchArgument('tilt_kp',         default_value='1.0'),
        DeclareLaunchArgument('tilt_accel_band', default_value='0.5',
            description='quasi-static gate ||a| - 9.81| (m/s²)'),

        # --- FlatZ (indoor flat-floor prior) ---
        DeclareLaunchArgument('flatz_enabled', default_value='true'),
        DeclareLaunchArgument('flatz_alpha',   default_value='0.05',
            description='per-stance low-pass coefficient toward p.z = 0'),

        # --- heel-toe rolling compensation ---
        DeclareLaunchArgument('foot_roll_toe_offset', default_value='0.0',
            description='walking forward: ~0.20 m; spinning/unknown: 0'),
        DeclareLaunchArgument('foot_roll_sign', default_value='1.0',
            description='empirically +1 for this URDF'),

        # --- ROS output ---
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link_leg_odom'),
        DeclareLaunchArgument('odom_topic', default_value='/leg_odometry'),

        OpaqueFunction(function=_spawn),
    ])
