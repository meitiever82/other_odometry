#!/usr/bin/env python3
"""腿式里程计 launch (C++ 版本): leg_odom_node + robot_state_publisher。

用法:
  ros2 launch leg_odometry leg_odometry.launch.py
  ros2 launch leg_odometry leg_odometry.launch.py urdf_path:=/abs/foo.urdf
  ros2 launch leg_odometry leg_odometry.launch.py params_file:=/abs/bar.yaml
  # 另一终端:
  ros2 bag play <bag_path> --clock -r 1.0

参数加载: 默认从 leg_odometry/config/ekf_params.yaml 读，
也可以用 params_file:= 临时换一组参数。yaml 是唯一真实源，
launch 不再硬编码任何噪声值。
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_URDF = os.environ.get(
    'CASBOT_URDF',
    os.path.expanduser('~/casbot_ws/urdf/casbot02_7dof_shell.urdf'),
)

# 把 ekf_params.yaml 里的 key 映射到 leg_odom_node 的 ROS 参数名。
# 只在 yaml 里出现且 C++ 节点确实 declare_parameter 过的才会被传过去；
# 缺失的 key 由 C++ 那边的 declare_parameter 默认值兜底。
_EKF_KEYS = (
    'accel_noise', 'gyro_noise',
    'accel_bias_walk', 'gyro_bias_walk',
    'foot_contact_noise', 'foot_swing_noise',
    'fk_position_noise', 'zupt_noise',
    'flat_z_noise', 'flat_vz_noise',
)

def _build_node_params(urdf_path: str, params_file: str) -> dict:
    """读 yaml 并组装成 leg_odom_node 的参数 dict。"""
    with open(params_file) as f:
        cfg = yaml.safe_load(f) or {}

    ekf = cfg.get('ekf', {}) or {}
    contact = cfg.get('contact', {}) or {}
    smoother = cfg.get('smoother', {}) or {}

    params: dict = {'urdf_path': urdf_path}
    for k in _EKF_KEYS:
        if k in ekf:
            params[k] = ekf[k]

    if 'effort_threshold' in contact:
        params['effort_threshold'] = contact['effort_threshold']
    if 'hysteresis' in contact:
        params['effort_hysteresis'] = contact['hysteresis']  # yaml→C++ 改名
    if 'effort_joint_left' in contact:
        params['effort_joint_left'] = contact['effort_joint_left']
    if 'effort_joint_right' in contact:
        params['effort_joint_right'] = contact['effort_joint_right']

    # smoother 后端 bias_walk (与前端 ekf.{accel,gyro}_bias_walk 不同)
    if 'accel_bias_walk' in smoother:
        params['smoother_accel_bias_walk'] = smoother['accel_bias_walk']
    if 'gyro_bias_walk' in smoother:
        params['smoother_gyro_bias_walk'] = smoother['gyro_bias_walk']
    return params

def _spawn(context, *args, **kwargs):
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(
            f'URDF not found: {urdf_path}\n'
            f'  use launch arg urdf_path:=... or env CASBOT_URDF=...'
        )
    if not os.path.exists(params_file):
        raise FileNotFoundError(f'params_file not found: {params_file}')

    # robot_state_publisher 需要把 URDF 内容作为字符串传入，
    # 这里顺手把 mesh 路径修正成绝对 file:// 路径。
    with open(urdf_path) as f:
        urdf_content = f.read()
    meshes_dir = os.path.abspath(
        os.path.join(os.path.dirname(urdf_path), '..', 'meshes')
    )
    urdf_content = urdf_content.replace(
        'filename="../meshes/',
        f'filename="file://{meshes_dir}/',
    )

    node_params = _build_node_params(urdf_path, params_file)

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}],
        ),
        Node(
            package='leg_odometry',
            executable='leg_odom_node',
            name='leg_odom_node',
            output='screen',
            parameters=[node_params],
        ),
    ]

def generate_launch_description():
    pkg = FindPackageShare('leg_odometry')
    default_params = PathJoinSubstitution([pkg, 'config', 'ekf_params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_path',
            default_value=DEFAULT_URDF,
            description='Absolute path to the robot URDF',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Absolute path to ekf_params.yaml (single source of truth)',
        ),
        OpaqueFunction(function=_spawn),
    ])
