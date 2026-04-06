# CASBot02 腿式里程计 — 进展报告

## 目标

为 CASBot02 人形机器人开发腿式里程计，作为 GLIM LiDAR SLAM 的补充输入。

## 最终方案：ESKF + GTSAM 滑窗 Hybrid

```
 /imu (200Hz) ──► ┌───────────────────────────────────┐
                   │        ESKF 前端 (200Hz)           │──► /leg_odometry
 /joint_states ──► │  Predict: IMU 递推                 │    (实时位姿输出)
                   │  Update 1: FK 位置观测              │
 contact ────────► │  Update 2: ZUPT 速度约束            │
                   │  Update 3: 步级速度估计             │
                   │  Update 4: 双脚支撑 + 静止检测      │
                   │  Update 5: 平面 Z 约束              │
                   └──────────┬────────────────────────┘
                              │ 关键帧 (20Hz)
                              ▼
                   ┌───────────────────────────────────┐
                   │     GTSAM 滑窗平滑器 (2-5Hz)       │
                   │  IMU 预积分因子                     │
                   │  FK CustomFactor (解析 Jacobian)    │
                   │  足端固定因子                       │
                   │  Bias 演化因子                      │
                   │  → 估计 gyro bias → 反馈 ESKF      │
                   └───────────────────────────────────┘
```

### 状态向量（21 维 Error-State）

| 索引 | 变量 | 含义 | 在线估计 |
|------|------|------|----------|
| 0:3 | p | 位置 (世界系) | ✓ |
| 3:6 | v | 速度 (世界系) | ✓ |
| 6:9 | θ | 姿态误差 (SO3) | ✓ |
| 9:12 | b_a | 加速度计 bias | 锁定（ESKF），smoother 不注入 |
| 12:15 | b_g | 陀螺仪 bias | ESKF 小量估计 + smoother 注入 |
| 15:18 | p_fl | 左脚位置 (世界系) | ✓ |
| 18:21 | p_fr | 右脚位置 (世界系) | ✓ |

### 五层观测约束

| 层 | 约束 | 频率 | 作用 |
|----|------|------|------|
| FK 位置 | p + R·FK(q) = p_foot | 200Hz, 双脚 | 位置几何约束 |
| ZUPT | v ≈ -[ω]×(R·FK) - R·dFK/dt | 200Hz, 接触脚 | 帧级速度约束 |
| 步级速度 | v ≈ -R·ΔFK/Δt_stance | 每步 (~3Hz) | 整步平均速度 |
| 双脚支撑 | v_z ≈ 0 | 双脚着地时 | Z 速度强约束 |
| 平面先验 | p_z ≈ 0, v_z ≈ 0 | 200Hz | 消除 Z 漂移 |

### Smoother 反馈

| 参数 | 值 | 说明 |
|------|------|------|
| 窗口大小 | 60 关键帧 (3s) | 滑窗宽度 |
| 优化间隔 | 每 20 关键帧 (1s) | LM 批量优化 |
| gyro bias 注入 | alpha = 0.05 | 指数移动平均融合 |
| accel bias | 不注入 | 可观性不足，注入反而有害 |

## 评估结果

### 9 场景精度

| 场景 | 路径(m) | XY 漂移 | RMSE Z | RMSE Yaw | 评价 |
|------|---------|---------|--------|----------|------|
| slope_up_down | 11.9 | **2.6%** | 32cm | 3.9° | 优秀 |
| straight_medium | 14.9 | **3.1%** | 4cm | 3.7° | 优秀 |
| turn_in_place | 1.7 | **4.5%** | 1cm | 70.5° | 好（XY好，yaw差） |
| s_curve | 26.3 | **7.3%** | 4cm | 38.1° | 好 |
| straight_fast | 19.7 | **14.1%** | 5cm | 3.6° | 可接受 |
| straight_slow | 9.1 | **19.5%** | 3cm | 2.7° | 可接受 |
| stop_and_go | 13.6 | 39.4% | 4cm | 5.6° | 待改进 |
| long_walk | 141.7 | 42.3% | 3cm | 13.1° | 待改进 |
| curve_walk | 15.9 | 48.9% | 4cm | 86.4° | 待改进 |

### 三方对比

| 场景 | 纯 ESKF | 纯 GTSAM | **Hybrid** |
|------|---------|----------|-----------|
| straight_medium | 3.9% | 26.4% | **3.1%** |
| turn_in_place | 14.4% | 22.8% | **4.5%** |
| s_curve | 9.5% | 24.4% | **7.3%** |
| slope_up_down | **0.6%** | 72.2% | 2.6% |
| curve_walk | **48.4%** | 173.0% | 48.9% |

### 改进历程

| 版本 | straight_medium | 关键改动 |
|------|-----------------|----------|
| 原始 Bloesch EKF | 207% | 仅 FK 位置观测 |
| + ZUPT | 9.6% | dFK/dt 速度约束 |
| + 平面 Z 约束 | 6.6% | 消除 Z 漂移 |
| + gyro bias | 3.9% | 在线估计陀螺偏置 |
| + smoother gyro 注入 | 3.4% | GTSAM 后端校正 |
| + 步级速度 + 静止检测 | **3.1%** | 最终版 |

## 已知局限

| 局限 | 原因 | 影响场景 |
|------|------|----------|
| Yaw 累积漂移 | gyro bias 估计精度有限 | curve_walk, turn_in_place |
| Accel bias 不可观 | 纯腿式里程计缺少绝对位置约束 | long_walk, stop_and_go |
| 走停切换冲击 | ZUPT 的 dFK/dt 在速度突变时不适应 | stop_and_go |
| 慢速精度差 | 每米步数少，约束稀疏 | straight_slow |

## 改进方向

### 短期（算法优化，不需新传感器）

1. **C++ GTSAM 全因子图**
   - 高频关键帧（100-200Hz）解决 Python 回调性能瓶颈
   - IMU 预积分 + FK CustomFactor 全部在图内联合优化
   - 预期：全场景 < 10%

2. **自适应 ZUPT**
   - 检测走停切换，动态调节 ZUPT 噪声
   - 减速阶段逐步加紧 ZUPT，加速阶段逐步放松
   - 预期：stop_and_go 39% → 20%

3. **LVI-Q 风格足端预积分**
   - 参考 arXiv:2510.15220，将多帧 FK 观测预积分为单因子
   - 减少因子数量，提升计算效率
   - 比帧级 ZUPT 更鲁棒

4. **多步 Heading 约束**
   - 连续 N 步的足端落地位置拟合行走方向
   - 直接约束 yaw，不依赖 gyro
   - 预期：curve_walk 49% → 30%

### 中期（需要传感器融合）

5. **LiDAR 融合（GLIM 集成）**
   - GLIM 提供低频绝对位姿 → 修正 accel bias + yaw
   - Leg odometry 提供高频先验 → 帮助 GLIM 在 LiDAR 退化时存活
   - 这是最终目标

6. **视觉里程计融合**
   - 头部相机提供视觉特征 → 约束 yaw 和位置
   - 与 leg odometry 互补（视觉怕遮挡，腿怕滑动）

### 长期（系统级）

7. **Isaac Lab RL 步行策略**
   - 自主训练 CASBot02 行走（不依赖公司其他组）
   - 在 Isaac Sim 获取物理级仿真数据验证
   - DGX Spark 上已开始训练

8. **实机部署 + Sim-to-Real**
   - 真实地面行走数据验证
   - 调参适配真实 IMU 噪声特性
   - 接触检测适配真实踝关节力矩

## 代码结构

```
src/leg_odometry/
├── leg_odometry/
│   ├── ekf.py              # ESKF (21维, 5层观测, ZUPT, 静止检测)
│   ├── smoother.py          # GTSAM 滑窗平滑器 (bias 校正)
│   ├── gtsam_odom.py        # GTSAM 独立版 (对比基线)
│   ├── kinematics.py        # URDF FK (PyKDL)
│   └── contact_detector.py  # 踝关节力矩接触检测
├── scripts/
│   ├── leg_odometry_node.py  # ROS2 在线节点
│   ├── generate_sim_data.py  # 仿真数据生成 (9场景)
│   ├── evaluate_sim.py       # ESKF 评估
│   ├── evaluate_gtsam.py     # GTSAM 评估
│   └── evaluate_hybrid.py    # Hybrid 评估
├── config/
│   ├── ekf_params.yaml       # 所有噪声参数
│   └── joint_mapping.yaml    # 关节名映射
└── doc/
    ├── demo_guide.md          # RViz 演示步骤
    └── progress_report.md     # 本报告
```

## 最优参数配置

```yaml
ekf:
  accel_noise: 0.1          # IMU 加速度噪声
  gyro_noise: 0.01          # IMU 陀螺噪声
  accel_bias_walk: 0.0      # 锁定 (smoother 也不注入)
  gyro_bias_walk: 0.001     # 在线估计
  foot_contact_noise: 0.002  # 接触脚位置噪声
  foot_swing_noise: 1.0      # 摆动脚位置噪声
  fk_position_noise: 0.005   # FK 观测噪声
  zupt_noise: 0.03           # ZUPT 速度约束
  flat_z_noise: 0.001        # Z 位置约束
  flat_vz_noise: 0.001       # Z 速度约束

smoother:
  window_size: 60             # 关键帧窗口
  interval: 20                # 优化间隔
  accel_bias_walk: 0.005      # smoother 内 accel bias 噪声
  gyro_bias_walk: 0.002       # smoother 内 gyro bias 噪声
  gyro_injection_alpha: 0.05  # 融合权重
```
