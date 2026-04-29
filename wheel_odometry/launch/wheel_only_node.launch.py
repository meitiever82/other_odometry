#!/usr/bin/env python3
"""Launch the FK-only swerve wheel-odometry node.

State: p, R, gyro_bias.  No Kalman filter.

Pipeline: 4-wheel LS → (vx_b, vy_b, ω_z), gyro+Mahony for R pitch/roll,
LS yaw rate (with gyro fallback on slip) for R yaw, FlatZ clamp on p.z.

Usage:
    ros2 launch wheel_odometry wheel_only_node.launch.py \\
        wheelbase:=0.6 track:=0.5 chassis_topic:=/chassis_state imu_topic:=/imu
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context, *args, **kwargs):
    def _d(name):
        return float(LaunchConfiguration(name).perform(context))

    def _s(name):
        return LaunchConfiguration(name).perform(context)

    def _b(name):
        return LaunchConfiguration(name).perform(context).lower() == 'true'

    params = {
        'wheelbase':       _d('wheelbase'),
        'track':           _d('track'),
        'wheel_radius':    _d('wheel_radius'),
        'bias_window_sec': _d('bias_window_sec'),
        'tilt_kp':         _d('tilt_kp'),
        'tilt_accel_band': _d('tilt_accel_band'),
        'yaw_source':      _s('yaw_source'),
        'slip_threshold':  _d('slip_threshold'),
        'flatz_enabled':   _b('flatz_enabled'),
        'flatz_alpha':     _d('flatz_alpha'),
        'publish_tf':      _b('publish_tf'),
        'odom_frame':      _s('odom_frame'),
        'base_frame':      _s('base_frame'),
        'odom_topic':      _s('odom_topic'),
        'chassis_topic':   _s('chassis_topic'),
        'imu_topic':       _s('imu_topic'),
        'diag_csv_path':   _s('diag_csv_path'),
    }
    return [
        Node(
            package='wheel_odometry',
            executable='wheel_only_node',
            name='wheel_only_odom',
            output='screen',
            parameters=[params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # --- geometry (calibrate per platform) ---
        DeclareLaunchArgument('wheelbase',    default_value='0.6'),
        DeclareLaunchArgument('track',        default_value='0.5'),
        DeclareLaunchArgument('wheel_radius', default_value='0.121',
            description='multiplier on ChassisState.speed (rad/s on w2 → m/s); '
                        'set 1.0 if speed field is already m/s'),

        # --- init ---
        DeclareLaunchArgument('bias_window_sec', default_value='3.0',
            description='static pre-motion window (s) for gyro-bias + gravity init'),

        # --- accel-tilt Mahony ---
        DeclareLaunchArgument('tilt_kp',         default_value='1.0'),
        DeclareLaunchArgument('tilt_accel_band', default_value='0.5'),

        # --- yaw policy ---
        DeclareLaunchArgument('yaw_source', default_value='gyro',
            description='"gyro" (drifts but slip- and geometry-immune; default) '
                        'or "ls" (LS-derived ω_z; drift-free but biased if wheelbase/track wrong)'),
        DeclareLaunchArgument('slip_threshold', default_value='0.5',
            description='LS residual (m/s) above which yaw falls back to gyro'),

        # --- FlatZ ---
        DeclareLaunchArgument('flatz_enabled', default_value='true'),
        DeclareLaunchArgument('flatz_alpha',   default_value='0.05'),

        # --- ROS output ---
        DeclareLaunchArgument('publish_tf',    default_value='true'),
        DeclareLaunchArgument('odom_frame',    default_value='odom'),
        DeclareLaunchArgument('base_frame',    default_value='base_link_wheel_odom'),
        DeclareLaunchArgument('odom_topic',    default_value='/wheel_odometry'),
        DeclareLaunchArgument('chassis_topic', default_value='/robot/wheel_status'),
        DeclareLaunchArgument('imu_topic',     default_value='/rslidar_imu_data'),
        DeclareLaunchArgument('diag_csv_path', default_value=''),

        OpaqueFunction(function=_spawn),
    ])
