# Swerve-only Wheel Odometry

## 一句话

**4 轮的 (steering angle, drive speed) 连立 8×3 最小二乘解出 body twist (vx, vy);IMU 陀螺给 yaw rate;两者积分得位姿。无 Kalman 滤波,9 维 state(p / R / b_g)。w2 实测对比 LIO 真值 APE RMSE 1.22m / 60m path = 2%。**

跟 `leg_odometry/doc/fk_only_odometry.md` 是姊妹方案,把腿 FK 模块换成 4 轮 swerve LS,信号流和"无 Kalman"哲学一致。

---

## 核心思路

### 1. 4 轮 swerve 反解 body twist(8×3 LS)

每个轮 i 在 body frame 位置 `r_i = (x_i, y_i)`(从 wheelbase L、track W 推出 FL/FR/RL/RR),给定测得的 (steering angle θ_i, signed speed v_i),轮接触点在 body 帧的速度是
```
u_i = v_i · [cos θ_i, sin θ_i]
```
刚体约束(同一刚体上各点速度一致):
```
u_i = v_body + ω_z × r_i
=>  vx − ω_z · y_i = v_i cos θ_i
    vy + ω_z · x_i = v_i sin θ_i
```
4 轮 × 2 方程 = 8 方程,3 未知 (vx, vy, ω_z),用 `colPivHouseholderQr` 一行解。残差 `||A z − b||₂` 是 slip 指标。

### 2. R 怎么来:gyro 积分 + 重力 Mahony 拉 tilt

```
R ← R · exp_so3((gyro_used − bg) · dt)
其中 gyro_used = (gx_imu, gy_imu, ω_z_LS)  if LS_residual < slip_thr  且  yaw_source=ls
                (gx_imu, gy_imu, gz_imu)   否则
```
- pitch/roll:gyro_x、gyro_y 积分,准静态时用 accel 跟重力对齐做 Mahony 修正(误差 cross product,zero out z 保留 yaw 自由度)
- yaw:理论上 LS 解出来无漂移,实测对几何标定误差极敏感,**默认走 gyro_z**(详见下一节)

### 3. IMU 装载自动校正(必须)

w2 这台车 `/rslidar_imu_data` 是雷达内置 IMU,装载方向**不竖直**:静止时 `avg_accel = [0.08, 6.89, -7.06]`,重力 ≈ 绕车头方向(+X)旋 135.65°。

节点在启动 `bias_window_sec` 静止段结束时,从 `avg_accel` 自动算
```
R_base_imu_ = FromTwoVectors(avg_accel.normalized(), [0,0,1])
```
后续每条 IMU:
```
gyro_base  = R_base_imu_ · (gyro_imu  − bg)
accel_base = R_base_imu_ · accel_imu
```
不做这步出来的轨迹是个 figure-8 鬼影。

注:`R_base_imu_` 的 yaw 分量未约束(重力对绕 z 的旋转不变)。FromTwoVectors 给最小角度旋转,等价于"yaw mount 默认 0"。实际中 ω_z 关于同一根 z 轴 frame-invariant,所以 OK。

### 4. speed 单位陷阱:wheel_radius

`navigation_interface/msg/WheelStatus.speed` spec 文档写"m/s",实测在 w2 上是**轮角速度 rad/s**。乘 `wheel_radius` 才是地速:
```
v_i = speed_field × wheel_radius
```
w2 标定值 `wheel_radius = 0.121m`(从 LIO 真值反推,对应 24.2cm 直径轮)。

---

## 为什么 yaw 默认走 gyro 而不是 LS

原始设计直觉:swerve 4 轮 LS 直接解 ω_z,无积分→无漂,应完胜 gyro。**实测反过来**。

w2 同段 bag(174.6s overlap,LIO 真值 60.4m path):

| `yaw_source` | path | path_ratio vs LIO | 备注 |
|---|---|---|---|
| `ls` (slip_thr=0.5) | 542m → 60m equiv | 错(走的是早期 r=1 错误数据) | LS 残差超 0.5 退 gyro |
| `ls` (slip_thr=100) | — | — | 永远信 LS,几何错时崩 |
| **`gyro` (默认)** | 60.5m | **1.0023** | gyro 积分 yaw,LS 只供 vx,vy |

原因:LS 解出的 ω_z 对 wheelbase / track / wheel_radius 的标定误差**极敏感**。几何错 30%,LS yaw 系统性偏几度每秒,积分 100s 就崩。gyro_z(经 mount cal 后)在 100s 量级累计漂 ~3-5°,远更鲁棒。

**结论**:理论"swerve 无 yaw drift"优势只有在**精确几何标定**时才能拿到。所以默认 `yaw_source=gyro`,LS yaw 留作"几何精确后再开"的可选项。

但 vx, vy 仍由 LS 8×3 解算 —— 4 轮一致直行场景里 vx, vy 跟 wheelbase / track 几乎无关(几何信息只对 ω_z 敏感),即使 L, W 略错也不影响线速度估计。

---

## 三个信号流

```
IMU (gyro, accel) ──► R_base_imu · (· − bg)
                         │
                         ├─► gyro_base ──► R 积分(roll/pitch/yaw)
                         └─► accel_base ─► Mahony 拉 R 的 roll/pitch 回重力

WheelStatus (4×θ, 4×v) ──► v · wheel_radius ──► 8×3 LS ──► (vx_b, vy_b, ω_z_LS, residual)
                                                              │
                                                              └─► (yaw_source=ls 时)
                                                                  替换 gyro_base 的 z 分量

                            v_world = R · (vx_b, vy_b, 0)
                                  │
                                  ▼
                            p ← p + v_world · dt   (+ FlatZ 钳 z)
```

主回调 `chassis_cb`(50Hz,每条 `/robot/wheel_status` 跑一次)8 步:

```cpp
1. 读 4 角度 + 4 速度,speed × wheel_radius
2. 8×3 LS 解 (vx_b, vy_b, ω_z_LS, residual)
3. 拼 ω_used:LS yaw 还是 gyro yaw
4. R ← R · exp_so3(ω_used · dt)
5. accel-tilt Mahony(准静态门控)
6. v_world = R · (vx_b, vy_b, 0)
7. p ← p + v_world · dt
8. FlatZ 钳 p.z
```

`imu_cb`(200Hz,只更新缓存):静止段累计 → 出 bg + R_base_imu;之后每帧 `gyro_base = R_base_imu · (g − bg)`、`accel_base = R_base_imu · a`。

---

## 实测数据(w2 platform, 2026-04-29)

**bag**:`Datasets/w2/rosbag2_2026_04_28-17_06_53`(186s, 9213 wheel_status 帧,室内/小场地慢速测试,有 LIO 真值 `traj_imu.txt`)。

**vs LIO 真值(174.6s overlap):**

| 指标 | 值 |
|---|---|
| LIO path | 60.37 m |
| wheel_odom path | 60.50 m |
| **path ratio** | **1.0023** |
| **APE RMSE** | **1.22 m**(2.0% of path)|
| APE max | 2.75 m |
| SE2 align rotation | +4.48° |
| SE2 align translation | [+1.47, +1.10] m |

参数:`wheel_radius=0.121, yaw_source=gyro, wheelbase=0.6, track=0.5(后两者对 gyro yaw 模式不敏感)`。

> 轨迹对比图:`data/w2_vs_lio_2026-04-29.png`

---

## 标定流程(新平台上线检查表)

1. **静态启动**:开机后 ≥3s 不动,看节点日志 `init done` 输出
   - `bg = [...]` 应 ≤ 0.01 rad/s 量级,否则 IMU 噪声大
   - `R_base_imu rpy = [...]` 显示装载方向,可跟机械图核对
2. **wheel_radius 标定**:跑一段已知尺寸的直行(尺测 or LIO 真值),目标 `path_ratio` ≈ 1.000
   - 错 5% 直接说明 wheel_radius 错 5%,等比例调
3. **wheelbase / track 标定**(选):做一段已知半径的转圈,看 `yaw_source=ls` 时 `ω_z` 是否准
   - 准了再考虑切回 LS yaw,拿"无漂"优势
4. **slip_threshold 调**(选):如果地面易打滑,`yaw_source=ls slip_thr=0.5` 提供回退保护

---

## 已知局限

1. **wheel slip 影响 vx, vy**:LS 残差能拒绝 ω_z(yaw 回退 gyro),但拒绝不了 vx, vy 的滑动误差。重打滑场景 path 会过估
2. **不处理 z 漂**:FlatZ 假设地面平。上下坡场景关 `flatz_enabled`,代价是 z 没约束(目前不积 IMU accel,所以也不漂太凶)
3. **gyro yaw 长漂**:当前默认配置下,无 LiDAR/视觉/磁力计闭环时 yaw 100s 量级累计 ~3-5° drift。靠几何精确后切 LS 可治
4. **假设开机前 ≥3s 静止**:用于估 gyro bias + 初始化 R_base_imu。不静止则 bg 估错,后续每秒进数千度 yaw 漂
5. **wheel_to_imu translation 不用**:translation 不影响 ω 测量,所以这套不需要;真要严格融合,需补
6. **wheel_to_imu yaw 部分未约束**:R_base_imu 只对齐 pitch/roll(重力定义);IMU yaw 多/少装 30° 对 ω_z 无影响,但 gx/gy 数值会错。Mahony tilt 每帧拉回重力 → 小 yaw mount 错位被吸收
7. **无下游 covariance**:`/wheel_odometry` 的 twist/pose covariance 留空,接 SLAM 融合时需补
8. **角度奇异**:speed ≈ 0 时该轮 angle 几乎不约束 LS(0×cos θ = 0×sin θ = 0),解算靠其他 3 轮的杠杆

---

## 使用

### Launch

```bash
ros2 launch wheel_odometry wheel_only_node.launch.py \
    wheelbase:=0.6 \
    track:=0.5 \
    wheel_radius:=0.121 \
    yaw_source:=gyro \
    chassis_topic:=/robot/wheel_status \
    imu_topic:=/rslidar_imu_data
```

### 订阅 / 发布

- 订阅:`/imu`(`sensor_msgs/Imu`)、`/chassis_state`(默认 `/robot/wheel_status`,`navigation_interface/msg/WheelStatus`)
- 发布:`/wheel_odometry`(`nav_msgs/Odometry`)、TF `odom → base_link_wheel_odom`

### Smoke test

```bash
bash src/wheel_odometry/scripts/run_smoke.sh
# env vars:BAG_DIR / WHEELBASE / TRACK / YAW_SOURCE / RATE
# 终结打印 closure error
```

### 关键参数

| 参数 | 默认 | 作用 |
|---|---|---|
| `wheelbase` | 0.6 | L,前后轮接触点纵向间距 (m),按平台标定 |
| `track` | 0.5 | W,左右轮横向间距 (m),按平台标定 |
| **`wheel_radius`** | **0.121** | speed 字段乘子;若 speed 真是 m/s 填 1.0 |
| `bias_window_sec` | 3.0 | 启动静止段 (估 bg + R_base_imu) |
| `tilt_kp` | 1.0 | Mahony pitch/roll 修正增益 |
| `tilt_accel_band` | 0.5 | accel 准静态门控 (m/s²) |
| `yaw_source` | `"gyro"` | yaw rate 源,默认 gyro,几何标定准时切 `"ls"` |
| `slip_threshold` | 0.5 | LS 残差 > 此值时 yaw 退 gyro (m/s) |
| `flatz_enabled` | true | p.z 向 0 收敛 |
| `flatz_alpha` | 0.05 | p.z 收敛比例 |

---

## 代码结构

```
src/wheel_odometry/
├── package.xml
├── CMakeLists.txt
├── include/wheel_odometry/
│   ├── so3_utils.h                    # skew, exp_so3
│   └── swerve_kinematics.h            # WheelGeometry::from_LW + solve_body_twist
├── src/
│   └── wheel_only_node.cpp            # 主节点 ~290 行
├── launch/wheel_only_node.launch.py
├── config/wheel_only_params.yaml
├── scripts/run_smoke.sh               # 端到端跑 bag,出 closure
├── doc/
│   ├── swerve_only_odometry.md        # 本文
└── data/
    └── w2_vs_lio_2026-04-29.png       # 实测对比图
```

依赖:Eigen3 + ROS2 标准包(rclcpp/sensor_msgs/nav_msgs/geometry_msgs/tf2_ros)+ `navigation_interface`(msg-only stub)。**不需要 KDL / urdf / GTSAM**。

---

## 后续可扩展

1. **Slip detector 输出**:`ls_residual` publish 成 `std_msgs/Float64`,下游 SLAM 决定是否信任
2. **Covariance 输出**:LS Jacobian + 单轮速度方差直接给 (vx, vy, ω) 协方差,接 SLAM 时补
3. **Wheel-IMU ESKF 融合**:把 IMU 加速度作 prediction、wheel 速度作 measurement,治 long-run gyro yaw drift。但需先确认 FK-only 不够用再上 —— 当前 2% RMSE 已经够多数应用
4. **wheel_radius / wheelbase / track 在线标定**:LIO 真值在线时可做 RLS 估三个 scalar
5. **支持其他 swerve msg 类型**:目前订 `navigation_interface/msg/WheelStatus`,加 bridge 节点支持其他厂商格式

---

## 一句话总结

**腿 FK-only 的车轮版,把"哪只脚 stance"换成"4 轮永远接触",把单脚 FK 换成 8×3 LS,其他都一样。w2 实测 APE 2%,够用。下一步治 yaw drift 接外部闭环。**
