"""GTSAM 滑窗平滑器：周期性批量优化，估计 IMU bias 并反馈给 ESKF。

架构:
  ESKF (200Hz) ──► 实时输出
       ↑ bias 校正
  Smoother (2-5Hz) ◄── 关键帧缓存

滑窗: 固定 N 个关键帧，LM 批量优化。
输出: 优化后的 accel_bias 和 gyro_bias。
"""

import numpy as np
import gtsam
from gtsam import symbol
from dataclasses import dataclass, field

from leg_odometry.ekf import skew

X = lambda i: symbol('x', i)
V = lambda i: symbol('v', i)
B = lambda i: symbol('b', i)
FL = lambda i: symbol('f', i)
FR = lambda i: symbol('g', i)


@dataclass
class KeyframeData:
    """一个关键帧的所有数据。"""
    pose: gtsam.Pose3           # ESKF 估计的位姿
    velocity: np.ndarray        # ESKF 估计的速度
    bias: gtsam.imuBias.ConstantBias  # 当前 bias 估计
    pim: object                 # PreintegratedImuMeasurements
    fk_left: np.ndarray         # 左脚 FK (body 系)
    fk_right: np.ndarray        # 右脚 FK (body 系)
    contact_left: bool
    contact_right: bool
    timestamp: float


def _make_fk_factor(pose_key, foot_key, fk_body: np.ndarray, noise_model):
    """FK 位置约束 CustomFactor: p + R·FK = p_foot。"""
    fk = fk_body.copy()

    def error_func(this, values, jacobians):
        pose = values.atPose3(pose_key)
        foot = np.array(values.atPoint3(foot_key))
        R = pose.rotation().matrix()
        t = np.array(pose.translation())
        r_world = R @ fk
        residual = t + r_world - foot

        if jacobians is not None:
            if len(jacobians) > 0:
                H = np.zeros((3, 6))
                H[:, 0:3] = -R @ skew(fk)
                H[:, 3:6] = R
                jacobians[0] = H
            if len(jacobians) > 1:
                jacobians[1] = -np.eye(3)
        return residual

    keys = gtsam.KeyVector([pose_key, foot_key])
    return gtsam.CustomFactor(noise_model, keys, error_func)


class SlidingWindowSmoother:
    """GTSAM 滑窗平滑器。"""

    def __init__(self, params: dict):
        self.g = np.array(params.get('gravity', [0, 0, -9.81]))

        # IMU 预积分参数
        imu_params = gtsam.PreintegrationParams(self.g)
        accel_noise = params.get('accel_noise', 0.1)
        gyro_noise = params.get('gyro_noise', 0.01)
        imu_params.setAccelerometerCovariance(np.eye(3) * accel_noise**2)
        imu_params.setGyroscopeCovariance(np.eye(3) * gyro_noise**2)
        imu_params.setIntegrationCovariance(np.eye(3) * 1e-5)
        self._imu_params = imu_params

        # 噪声参数
        self._sigma_fk = params.get('fk_position_noise', 0.005)
        self._sigma_contact = params.get('foot_contact_noise', 0.002)
        self._sigma_swing = params.get('foot_swing_noise', 1.0)
        self._sigma_flat_z = params.get('flat_z_noise', 0.001)
        self._sigma_flat_vz = params.get('flat_vz_noise', 0.001)
        self._sigma_ba = params.get('smoother_accel_bias_walk', 0.005)
        self._sigma_bg = params.get('smoother_gyro_bias_walk', 0.002)

        # 滑窗参数
        self._window_size = params.get('smoother_window_size', 60)  # 关键帧数
        self._optimize_interval = params.get('smoother_interval', 20)  # 每 N 帧优化一次

        # 关键帧缓存
        self._keyframes: list[KeyframeData] = []
        self._kf_count = 0
        self._since_last_opt = 0

        # 最新优化结果
        self.optimized_bias = None  # gtsam.imuBias.ConstantBias
        self.has_new_bias = False

    def create_preintegrator(self, current_bias):
        """为下一个关键帧创建预积分器。"""
        return gtsam.PreintegratedImuMeasurements(self._imu_params, current_bias)

    def add_keyframe(self, kf: KeyframeData):
        """添加关键帧到缓存。"""
        self._keyframes.append(kf)
        self._kf_count += 1
        self._since_last_opt += 1

        # 滑窗：保留最新 window_size 个
        if len(self._keyframes) > self._window_size:
            self._keyframes = self._keyframes[-self._window_size:]

    def should_optimize(self) -> bool:
        """是否应该触发优化。"""
        return (self._since_last_opt >= self._optimize_interval
                and len(self._keyframes) >= 10)

    def optimize(self) -> bool:
        """在滑窗上运行批量优化，估计 bias。

        Returns:
            True if optimization succeeded.
        """
        self._since_last_opt = 0
        kfs = self._keyframes
        n = len(kfs)
        if n < 3:
            return False

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()

        # 第一个关键帧: 强先验（锚定位置）
        kf0 = kfs[0]
        pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.01, 0.01, 0.01, 0.05, 0.05, 0.05]))
        vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        bias_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1]))  # accel bias 宽松，gyro 稍紧
        foot_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.05)

        graph.addPriorPose3(X(0), kf0.pose, pose_noise)
        graph.addPriorVector(V(0), kf0.velocity, vel_noise)
        graph.add(gtsam.PriorFactorConstantBias(B(0), kf0.bias, bias_noise))

        fl_world_0 = np.array(kf0.pose.translation()) + kf0.pose.rotation().matrix() @ kf0.fk_left
        fr_world_0 = np.array(kf0.pose.translation()) + kf0.pose.rotation().matrix() @ kf0.fk_right
        graph.addPriorPoint3(FL(0), gtsam.Point3(*fl_world_0), foot_noise)
        graph.addPriorPoint3(FR(0), gtsam.Point3(*fr_world_0), foot_noise)

        values.insert(X(0), kf0.pose)
        values.insert(V(0), kf0.velocity)
        values.insert(B(0), kf0.bias)
        values.insert(FL(0), gtsam.Point3(*fl_world_0))
        values.insert(FR(0), gtsam.Point3(*fr_world_0))

        fk_noise = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_fk)

        for j in range(1, n):
            i = j - 1
            kf = kfs[j]

            # IMU 因子
            if kf.pim is not None and kf.pim.deltaTij() > 0:
                graph.add(gtsam.ImuFactor(X(i), V(i), X(j), V(j), B(i), kf.pim))

            # Bias 演化
            dt = max(kf.pim.deltaTij() if kf.pim else 0.05, 1e-4)
            sigma_b = np.array([
                self._sigma_ba, self._sigma_ba, self._sigma_ba,
                self._sigma_bg, self._sigma_bg, self._sigma_bg
            ]) * np.sqrt(dt)
            sigma_b = np.maximum(sigma_b, 1e-6)
            graph.add(gtsam.BetweenFactorConstantBias(
                B(i), B(j), gtsam.imuBias.ConstantBias(),
                gtsam.noiseModel.Diagonal.Sigmas(sigma_b)))

            # FK 因子
            graph.add(_make_fk_factor(X(j), FL(j), kf.fk_left, fk_noise))
            graph.add(_make_fk_factor(X(j), FR(j), kf.fk_right, fk_noise))

            # 足端固定/摆动
            if kf.contact_left:
                cn = gtsam.noiseModel.Isotropic.Sigma(
                    3, self._sigma_contact * np.sqrt(dt))
            else:
                cn = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_swing)
            graph.add(gtsam.BetweenFactorPoint3(
                FL(i), FL(j), gtsam.Point3(0, 0, 0), cn))

            if kf.contact_right:
                cn = gtsam.noiseModel.Isotropic.Sigma(
                    3, self._sigma_contact * np.sqrt(dt))
            else:
                cn = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_swing)
            graph.add(gtsam.BetweenFactorPoint3(
                FR(i), FR(j), gtsam.Point3(0, 0, 0), cn))

            # 平面约束
            if self._sigma_flat_z > 0:
                flat_noise = gtsam.noiseModel.Diagonal.Sigmas(
                    np.array([100, 100, 100, 100, 100, self._sigma_flat_z]))
                flat_pose = gtsam.Pose3(kf.pose.rotation(),
                                        gtsam.Point3(kf.pose.x(), kf.pose.y(), 0.0))
                graph.addPriorPose3(X(j), flat_pose, flat_noise)

            # 初始值（用 ESKF 估计）
            R_j = kf.pose.rotation().matrix()
            t_j = np.array(kf.pose.translation())
            values.insert(X(j), kf.pose)
            values.insert(V(j), kf.velocity)
            values.insert(B(j), kf.bias)
            values.insert(FL(j), gtsam.Point3(*(t_j + R_j @ kf.fk_left)))
            values.insert(FR(j), gtsam.Point3(*(t_j + R_j @ kf.fk_right)))

        # LM 优化
        try:
            lm_params = gtsam.LevenbergMarquardtParams()
            lm_params.setMaxIterations(20)
            optimizer = gtsam.LevenbergMarquardtOptimizer(graph, values, lm_params)
            result = optimizer.optimize()

            # 提取最新关键帧的 bias
            latest_bias = result.atConstantBias(B(n - 1))
            self.optimized_bias = latest_bias
            self.has_new_bias = True
            return True

        except RuntimeError as e:
            return False

    def get_bias_correction(self):
        """获取优化后的 bias（取走后标记已读）。"""
        if self.has_new_bias:
            self.has_new_bias = False
            b_a = self.optimized_bias.accelerometer()
            b_g = self.optimized_bias.gyroscope()
            return np.array(b_a), np.array(b_g)
        return None, None
