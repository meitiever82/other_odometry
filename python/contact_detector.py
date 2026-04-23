"""接触检测：踝关节力矩阈值法 + 滞后。"""

from collections import deque

import numpy as np


class ContactDetector:
    """基于踝关节力矩的接触检测器（带滞后）。

    当 |effort| > threshold + hysteresis 时判定为接触，
    当 |effort| < threshold - hysteresis 时判定为腾空，
    中间区域保持上一次状态。
    """

    def __init__(self, threshold: float = 5.0, hysteresis: float = 1.0,
                 fk_z_threshold: float = 0.0):
        self.threshold = threshold
        self.hysteresis = hysteresis
        self.fk_z_threshold = fk_z_threshold  # FK Z 辅助 (0=关闭)
        self._contact_left = False
        self._contact_right = False

    def update(self, effort_left: float, effort_right: float,
               fk_z_left: float = None, fk_z_right: float = None) -> tuple[bool, bool]:
        """更新接触状态。

        Args:
            effort_left:  左踝关节力矩 (Nm)
            effort_right: 右踝关节力矩 (Nm)
            fk_z_left: 左脚 FK Z 坐标 (body 系，可选)
            fk_z_right: 右脚 FK Z 坐标 (body 系，可选)

        Returns:
            (left_contact, right_contact)
        """
        self._contact_left = self._detect(
            abs(effort_left), self._contact_left, fk_z_left)
        self._contact_right = self._detect(
            abs(effort_right), self._contact_right, fk_z_right)
        return self._contact_left, self._contact_right

    def _detect(self, effort_abs: float, prev_contact: bool,
                fk_z: float = None) -> bool:
        upper = self.threshold + self.hysteresis
        lower = self.threshold - self.hysteresis

        # 基本力矩检测
        if effort_abs > upper:
            result = True
        elif effort_abs < lower:
            result = False
        else:
            result = prev_contact

        # FK Z 辅助: 脚低 = 更可能着地，滞后区间内优先判定着地
        if self.fk_z_threshold != 0 and fk_z is not None:
            if not result and prev_contact and fk_z < self.fk_z_threshold:
                # 力矩说腾空，但脚还很低 → 延迟判定为腾空
                result = True

        return result

    @property
    def left_contact(self) -> bool:
        return self._contact_left

    @property
    def right_contact(self) -> bool:
        return self._contact_right


class BiasCompensatedContactDetector:
    """偏置补偿的接触检测：|effort - rolling_median(effort)| > threshold。

    背景用因果滑动中位数估计（窗口 N 帧），真实 bag 上姿态相关的
    静态力矩 bias 随步态姿态漂移，单一固定阈值会把真实 stance 的
    尖峰切掉或漏判；减去背景后双侧可用统一阈值。
    """

    def __init__(self, threshold: float = 5.0, hysteresis: float = 1.0,
                 window_size: int = 1000):
        """
        Args:
            threshold: detrended |effort| 的阈值 (Nm)
            hysteresis: 滞后带宽 (Nm)
            window_size: rolling median 窗长 (帧). 200Hz → 1000 = 5s
        """
        self.threshold = threshold
        self.hysteresis = hysteresis
        self.window_size = window_size
        self._buf_l = deque(maxlen=window_size)
        self._buf_r = deque(maxlen=window_size)
        self._contact_left = False
        self._contact_right = False
        # 前 warmup 帧数（< window/5）用裸阈值回退，避免刚启动时中位数偏
        self._warmup = max(50, window_size // 5)

    def update(self, effort_left: float, effort_right: float,
               fk_z_left: float = None, fk_z_right: float = None):
        self._buf_l.append(effort_left)
        self._buf_r.append(effort_right)

        if len(self._buf_l) < self._warmup:
            bg_l = 0.0
            bg_r = 0.0
        else:
            bg_l = float(np.median(self._buf_l))
            bg_r = float(np.median(self._buf_r))

        det_l = abs(effort_left - bg_l)
        det_r = abs(effort_right - bg_r)

        self._contact_left = self._apply(det_l, self._contact_left)
        self._contact_right = self._apply(det_r, self._contact_right)
        return self._contact_left, self._contact_right

    def _apply(self, eabs: float, prev: bool) -> bool:
        if eabs > self.threshold + self.hysteresis:
            return True
        if eabs < self.threshold - self.hysteresis:
            return False
        return prev

    @property
    def left_contact(self) -> bool:
        return self._contact_left

    @property
    def right_contact(self) -> bool:
        return self._contact_right
