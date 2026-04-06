"""GTSAM ISAM2 因子图腿式里程计。

因子:
  1. ImuFactor — IMU 预积分连接相邻关键帧
  2. FK CustomFactor — p + R·FK(q) = p_foot (几何约束)
  3. BetweenFactorPoint3 — 接触时足端世界位置不变
  4. 平面 CustomFactor — Z 位置/速度 ≈ 0
  5. BetweenFactorConstantBias — bias 随机游走

关键帧: 每 N 帧 IMU 一个 (默认 20Hz)

参考: Bloesch 2013 + Forster 2017 (IMU preintegration)
"""

import numpy as np
import gtsam
from gtsam import symbol

from leg_odometry.ekf import skew  # 复用 skew 函数

# 符号键
X = lambda i: symbol('x', i)  # Pose3
V = lambda i: symbol('v', i)  # velocity Vector3
B = lambda i: symbol('b', i)  # imuBias
FL = lambda i: symbol('f', i)  # left foot Point3
FR = lambda i: symbol('g', i)  # right foot Point3


# ============================================================
# CustomFactor 工厂函数
# ============================================================

def make_fk_factor(pose_key, foot_key, fk_body: np.ndarray,
                   noise_model):
    """FK 位置约束因子: p + R·FK_body = p_foot。

    error = p + R·fk - f  (3维)
    H_pose = [-R·skew(fk), I]  (3×6, GTSAM Pose3 local: [rot, trans])
    H_foot = -I  (3×3)
    """
    fk = fk_body.copy()

    def error_func(this, values, jacobians):
        pose = values.atPose3(pose_key)
        foot = np.array(values.atPoint3(foot_key))

        R = pose.rotation().matrix()
        t = np.array(pose.translation())
        r_world = R @ fk
        predicted = t + r_world
        residual = predicted - foot

        if jacobians is not None:
            if len(jacobians) > 0:
                # GTSAM body-frame 右扰动: T' = T * Exp([omega, v])
                H_pose = np.zeros((3, 6))
                H_pose[:, 0:3] = -R @ skew(fk)  # d(R*fk)/d(omega_body)
                H_pose[:, 3:6] = R                # d(t+R*v)/d(v_body)
                jacobians[0] = H_pose
            if len(jacobians) > 1:
                jacobians[1] = -np.eye(3)

        return residual

    keys = gtsam.KeyVector([pose_key, foot_key])
    return gtsam.CustomFactor(noise_model, keys, error_func)


def make_z_position_factor(pose_key, sigma_z: float):
    """Z 位置约束: pose.z() ≈ 0。1 维 CustomFactor。"""
    noise = gtsam.noiseModel.Isotropic.Sigma(1, sigma_z)

    def error_func(this, values, jacobians):
        pose = values.atPose3(pose_key)
        if jacobians is not None and len(jacobians) > 0:
            # GTSAM Pose3: 世界系 z = t_z, retract body-frame v → t += R*v
            # dz/d(xi) = [0,0,0, R[2,0], R[2,1], R[2,2]]
            R = pose.rotation().matrix()
            H = np.zeros((1, 6))
            H[0, 3:6] = R[2, :]
            jacobians[0] = H
        return np.array([pose.z()])

    keys = gtsam.KeyVector([pose_key])
    return gtsam.CustomFactor(noise, keys, error_func)


def make_vz_factor(vel_key, sigma_vz: float):
    """Z 速度约束: v_z ≈ 0。1 维 CustomFactor。"""
    noise = gtsam.noiseModel.Isotropic.Sigma(1, sigma_vz)

    def error_func(this, values, jacobians):
        v = values.atVector(vel_key)
        if jacobians is not None and len(jacobians) > 0:
            H = np.zeros((1, 3))
            H[0, 2] = 1.0
            jacobians[0] = H
        return np.array([v[2]])

    keys = gtsam.KeyVector([vel_key])
    return gtsam.CustomFactor(noise, keys, error_func)


# ============================================================
# GTSAM Leg Odometry 主类
# ============================================================

class GTSAMLegOdometry:
    """GTSAM ISAM2 增量优化腿式里程计。"""

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
        self._sigma_zupt = params.get('zupt_noise', 0.03)
        self._sigma_ba = params.get('accel_bias_walk', 0.0)
        self._sigma_bg = params.get('gyro_bias_walk', 0.001)

        # 关键帧间隔
        self._kf_interval = params.get('keyframe_interval', 10)

        # ISAM2
        isam_params = gtsam.ISAM2Params()
        isam_params.setRelinearizeThreshold(0.1)
        self._isam = gtsam.ISAM2(isam_params)

        # 状态
        self._kf_index = 0
        self._imu_count = 0
        self._pim = None
        self._current_bias = gtsam.imuBias.ConstantBias()

        self._last_pose = gtsam.Pose3()
        self._last_vel = np.zeros(3)
        self._last_foot_left = np.zeros(3)
        self._last_foot_right = np.zeros(3)

        # ZUPT 缓存
        self._prev_fk_left = None
        self._prev_fk_right = None
        self._last_gyro = np.zeros(3)

        self.initialized = False

    def initialize(self, accel: np.ndarray, p_fl_body: np.ndarray,
                   p_fr_body: np.ndarray):
        """用第一帧数据初始化因子图。"""
        from scipy.spatial.transform import Rotation as ScipyR

        # 估计初始姿态（对齐重力）
        g_body = -accel / np.linalg.norm(accel) * 9.81
        g_world = np.array([0, 0, -9.81])
        R0 = self._rotation_from_gravity(g_body, g_world)

        init_pose = gtsam.Pose3(gtsam.Rot3(R0), gtsam.Point3(0, 0, 0))
        init_vel = np.zeros(3)

        # 初始 bias
        b_a = accel - R0.T @ (-self.g)
        init_bias = gtsam.imuBias.ConstantBias(b_a, np.zeros(3))

        # 初始足端世界位置
        fl_world = R0 @ p_fl_body
        fr_world = R0 @ p_fr_body

        # 构建初始因子图
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()

        # 先验
        pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]))
        vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.01)
        bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        foot_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.01)

        graph.addPriorPose3(X(0), init_pose, pose_noise)
        graph.addPriorVector(V(0), init_vel, vel_noise)
        graph.add(gtsam.PriorFactorConstantBias(B(0), init_bias, bias_noise))
        graph.addPriorPoint3(FL(0), gtsam.Point3(*fl_world), foot_noise)
        graph.addPriorPoint3(FR(0), gtsam.Point3(*fr_world), foot_noise)

        values.insert(X(0), init_pose)
        values.insert(V(0), init_vel)
        values.insert(B(0), init_bias)
        values.insert(FL(0), gtsam.Point3(*fl_world))
        values.insert(FR(0), gtsam.Point3(*fr_world))

        self._isam.update(graph, values)
        self._extract_state(0)

        # 初始化预积分器
        self._pim = gtsam.PreintegratedImuMeasurements(
            self._imu_params, self._current_bias)

        self._prev_fk_left = p_fl_body.copy()
        self._prev_fk_right = p_fr_body.copy()
        self._kf_index = 0
        self._imu_count = 0
        self.initialized = True

    def add_imu(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """添加 IMU 测量到预积分器。"""
        if not self.initialized or dt <= 0 or dt > 0.1:
            return
        self._pim.integrateMeasurement(accel, gyro, dt)
        self._last_gyro = gyro - self._current_bias.gyroscope()
        self._imu_count += 1

    def add_keyframe(self, p_fl_body: np.ndarray, p_fr_body: np.ndarray,
                     contact_left: bool, contact_right: bool):
        """创建关键帧：添加因子 + ISAM2 优化。"""
        if not self.initialized or self._imu_count == 0:
            return

        i = self._kf_index
        j = i + 1
        delta_t = self._pim.deltaTij()

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()

        # --- 1. IMU 预积分因子 ---
        graph.add(gtsam.ImuFactor(X(i), V(i), X(j), V(j), B(i), self._pim))

        # --- 2. Bias 演化 ---
        sigma_b = np.array([
            self._sigma_ba, self._sigma_ba, self._sigma_ba,
            self._sigma_bg, self._sigma_bg, self._sigma_bg
        ]) * np.sqrt(max(delta_t, 1e-4))
        # 防止 sigma=0 导致 GTSAM 异常
        sigma_b = np.maximum(sigma_b, 1e-6)
        bias_noise = gtsam.noiseModel.Diagonal.Sigmas(sigma_b)
        graph.add(gtsam.BetweenFactorConstantBias(
            B(i), B(j), gtsam.imuBias.ConstantBias(), bias_noise))

        # --- 3. FK 位置 CustomFactor ---
        fk_noise = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_fk)
        graph.add(make_fk_factor(X(j), FL(j), p_fl_body, fk_noise))
        graph.add(make_fk_factor(X(j), FR(j), p_fr_body, fk_noise))

        # --- 4. 足端固定/摆动 ---
        if contact_left:
            cn = gtsam.noiseModel.Isotropic.Sigma(
                3, self._sigma_contact * np.sqrt(max(delta_t, 1e-4)))
        else:
            cn = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_swing)
        graph.add(gtsam.BetweenFactorPoint3(
            FL(i), FL(j), gtsam.Point3(0, 0, 0), cn))

        if contact_right:
            cn = gtsam.noiseModel.Isotropic.Sigma(
                3, self._sigma_contact * np.sqrt(max(delta_t, 1e-4)))
        else:
            cn = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_swing)
        graph.add(gtsam.BetweenFactorPoint3(
            FR(i), FR(j), gtsam.Point3(0, 0, 0), cn))

        # --- 5. ZUPT 速度约束 ---
        if contact_left and self._prev_fk_left is not None:
            v_exp = self._compute_zupt_velocity(p_fl_body, self._prev_fk_left)
            zupt_noise = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_zupt)
            graph.addPriorVector(V(j), v_exp, zupt_noise)

        if contact_right and self._prev_fk_right is not None:
            v_exp = self._compute_zupt_velocity(p_fr_body, self._prev_fk_right)
            zupt_noise = gtsam.noiseModel.Isotropic.Sigma(3, self._sigma_zupt)
            graph.addPriorVector(V(j), v_exp, zupt_noise)

        self._prev_fk_left = p_fl_body.copy()
        self._prev_fk_right = p_fr_body.copy()

        # --- 6. 平面约束 ---
        if self._sigma_flat_z > 0:
            graph.add(make_z_position_factor(X(j), self._sigma_flat_z))
        if self._sigma_flat_vz > 0:
            graph.add(make_vz_factor(V(j), self._sigma_flat_vz))

        # --- 初始值: IMU 预积分预测 ---
        nav_state = gtsam.NavState(self._last_pose, self._last_vel)
        pred = self._pim.predict(nav_state, self._current_bias)
        values.insert(X(j), pred.pose())
        values.insert(V(j), pred.velocity())
        values.insert(B(j), self._current_bias)

        # 足端初始值: FK 预测
        R_pred = pred.pose().rotation().matrix()
        t_pred = pred.pose().translation()
        values.insert(FL(j), gtsam.Point3(*(t_pred + R_pred @ p_fl_body)))
        values.insert(FR(j), gtsam.Point3(*(t_pred + R_pred @ p_fr_body)))

        # --- ISAM2 优化 ---
        try:
            self._isam.update(graph, values)
            self._isam.update()  # 额外迭代
            self._extract_state(j)
        except RuntimeError as e:
            # 优化失败，退回到 IMU 预测
            self._last_pose = pred.pose()
            self._last_vel = pred.velocity()

        # 重置预积分器
        self._kf_index = j
        self._pim = gtsam.PreintegratedImuMeasurements(
            self._imu_params, self._current_bias)
        self._imu_count = 0

    def _compute_zupt_velocity(self, fk_now, fk_prev):
        """计算 ZUPT 预期速度: v = -[ω]×(R·FK) - R·dFK/dt。"""
        R = self._last_pose.rotation().matrix()
        dt_fk = max(0.005 * self._kf_interval, 1e-6)  # 关键帧间隔

        R_dfk = R @ ((fk_now - fk_prev) / dt_fk)
        r_world = R @ fk_now
        omega_world = R @ self._last_gyro
        omega_cross = np.cross(omega_world, r_world)

        return -(omega_cross + R_dfk)

    def _extract_state(self, idx):
        """从 ISAM2 结果提取状态。"""
        result = self._isam.calculateEstimate()
        self._last_pose = result.atPose3(X(idx))
        self._last_vel = result.atVector(V(idx))
        self._last_foot_left = np.array(result.atPoint3(FL(idx)))
        self._last_foot_right = np.array(result.atPoint3(FR(idx)))
        self._current_bias = result.atConstantBias(B(idx))

    def should_add_keyframe(self) -> bool:
        return self._imu_count >= self._kf_interval

    def get_pose(self):
        """返回 (position [3], quaternion [x,y,z,w])。"""
        from scipy.spatial.transform import Rotation
        p = np.array(self._last_pose.translation())
        R = self._last_pose.rotation().matrix()
        quat = Rotation.from_matrix(R).as_quat()
        return p, quat

    def get_velocity(self):
        return self._last_vel.copy()

    @staticmethod
    def _rotation_from_gravity(g_body, g_world):
        g_b = g_body / np.linalg.norm(g_body)
        g_w = g_world / np.linalg.norm(g_world)
        v = np.cross(g_b, g_w)
        c = np.dot(g_b, g_w)
        if np.linalg.norm(v) < 1e-10:
            return np.eye(3) if c > 0 else -np.eye(3)
        V = skew(v)
        return np.eye(3) + V + V @ V / (1 + c)
