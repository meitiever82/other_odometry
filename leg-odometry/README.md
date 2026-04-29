# CASBot02 Leg Odometry

**FK-only** 腿式里程计，用于 CASBot02 人形机器人。无 Kalman 滤波，9 维 state，单文件 ~380 行。

> 完整方案/推导/实测/参数详见 [`doc/fk_only_odometry.md`](doc/fk_only_odometry.md)。

## 架构

```
IMU gyro ────► R (gyro 积分) ←── accel (Mahony 拉 tilt)
                │
joint q,q̇ ────► FK: v_foot_body = J(q)·q̇
                │
joint effort ─► ContactDetector → cl, cr
                │
                ▼
         v_world = -R · v_foot_body  (按 cl, cr 选脚)
                │
                ▼
         p ← p + v · dt   (+ FlatZ 钳 z)
```

核心节点：[`src/fk_only_node.cpp`](src/fk_only_node.cpp)，每 `/joint_states` 消息跑一次 `joint_cb`（200 Hz）。

## State (9 维)

| 变量 | 维度 | 来源 |
|------|------|------|
| p | 3 | v 积分 + FlatZ 钳 z |
| R | SO(3) | gyro 积分 + accel Mahony tilt |
| b_g | 3 | 启动前 3s 静止段平均 |

无 v、无 b_a、无 foot pos。**R 的 yaw 只能 gyro 积分，无外部 heading 必漂**——这是物理硬边界。

## 实测精度（真机 bag）

| bag | 场景 | 时长 | path ratio | Kabsch RMSE | Z 漂 |
|---|---|---|---|---|---|
| 16_12_13 | 103 办公室 loop | 184s | **1.005** | 1.42m / 63m | 3.5cm |
| 17_15_31 | B1 停车场 loop | 180s | **1.006** | 5.35m / 104m | 4.5cm |
| 04_10-18:56:38 | 原地转圈 | 78s | 1.131 | 0.10m | 3.4cm |

走环路径误差 < 1 % 跨 bag 一致。原地转圈 over-count 13 % 是已知局限（脚板 pivot，linear 补偿不能同时覆盖两种）。

## 编译

```bash
cd ~/rtabmap_ws
source /opt/ros/jazzy/setup.bash    # aarch64 平台
colcon build --packages-select leg_odometry
```

依赖：Eigen3、KDL、kdl_parser、urdf、ROS2 Jazzy。**不需要 GTSAM**（FK-only 主线）。

> 如果要构建 `leg_odom_node` / `leg_odom_hybrid`（历史 ESKF+GTSAM Hybrid 方案）才需要 GTSAM 4.3。

## 使用

```bash
# 走路场景
ros2 launch leg_odometry fk_only_node.launch.py \
    urdf_path:=/abs/path/to/casbot02_7dof_shell.urdf \
    foot_roll_toe_offset:=0.20

# 原地转圈场景
ros2 launch leg_odometry fk_only_node.launch.py \
    urdf_path:=/abs/path/to/casbot02_7dof_shell.urdf \
    foot_roll_toe_offset:=0.0
```

- 订阅：`/imu` (`sensor_msgs/Imu`)、`/joint_states` (`sensor_msgs/JointState`)
- 发布：`/leg_odometry` (`nav_msgs/Odometry`)、TF `odom → base_link_leg_odom`

## 关键参数（`config/fk_only_params.yaml`）

| 参数 | 默认 | 作用 |
|---|---|---|
| `bias_window_sec` | 3.0 | 启动静止段长度（估 gyro bias + 初始 R），开机前 3s 必须静止 |
| `effort_threshold_{left,right}` | 5.0 | 接触检测阈值 (Nm) |
| `tilt_kp` | 1.0 | Mahony 补偿增益 |
| `tilt_accel_band` | 0.5 | accel 准静态门控 |
| `flatz_enabled` | true | FlatZ 钳制开关（上下坡时关掉） |
| `flatz_alpha` | 0.05 | p.z 向 0 收敛比例 |
| **`foot_roll_toe_offset`** | 0.0 | heel-toe rolling 补偿，走路 0.20，转圈 0.0 |

## 已知局限

1. **Yaw 长漂**：物理硬边界，无外部 heading 治不了
2. **原地转圈 path 过估 13 %**：foot_roll 补偿过补偿
3. **假设开机前 3 s 静止**
4. **不处理上下坡**：FlatZ 假设地面平
5. **无下游 covariance**：要接 SLAM 融合时需要补

## 代码结构

```
├── src/
│   ├── fk_only_node.cpp         # ★ 当前主线节点
│   ├── leg_odom_node.cpp        # 历史 ESKF+GTSAM Hybrid
│   └── leg_odom_hybrid.cpp      # 历史离线评估
├── include/leg_odometry/
│   ├── so3_utils.h              # skew, exp_so3
│   └── state/Kinematics.h       # LegKinematics, ContactDetector
├── launch/fk_only_node.launch.py
├── config/
│   ├── fk_only_params.yaml      # ★ 当前主线参数
│   ├── ekf_params.yaml          # 历史
│   └── joint_mapping.yaml
├── scripts/fk_only_odometry.py  # Python 离线参考实现
└── doc/
    ├── fk_only_odometry.md      # ★ 当前方案文档
    └── progress_report.md       # 历史 ESKF+GTSAM Hybrid 报告
```

Python 离线参考：`scripts/fk_only_odometry.py`（与 C++ 节点算法一致，做回归基准）。
