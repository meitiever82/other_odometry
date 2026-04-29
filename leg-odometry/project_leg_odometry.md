---
name: leg_odometry 项目状态与历史 issue 清单
description: CASBot02 腿式里程计当前方案 = FK-only (doc/fk_only_odometry.md)。本文件只保留历史 ESKF+GTSAM Hybrid 方案的 issue/工具链笔记
type: project
---

`src/leg_odometry/` 当前方案是 **FK-only**（无 Kalman，9 维 state，gyro 给姿态 + FK 给身体速度）。**架构、精度、参数读 `src/leg_odometry/doc/fk_only_odometry.md`**，节点 `src/fk_only_node.cpp`，launch `launch/fk_only_node.launch.py`，配置 `config/fk_only_params.yaml`。

旧方案 ESKF+GTSAM Hybrid（`doc/progress_report.md`、`src/leg_odom_node.cpp`、`src/leg_odom_hybrid.cpp`、`config/ekf_params.yaml`）保留在 tree 里作为对比基线，**已不是主线**。本文件下面的 issue/工具链注释都是针对旧方案的。

**Why:** 2026-04-28 用户明确指出 fk_only_odometry.md 才是当前在用的方案；FK-only 在 bag 16_12_13 上 path ratio 1.005，旧 Hybrid 同条 bag 是 48.7% drift。

**How to apply:** 用户问 leg_odometry 相关问题时，先 read `doc/fk_only_odometry.md`。如果是 GTSAM 构建报错或老 ESKF 代码层 issue 才读本文件下半部分。

---

## 实机测试结果（point-in-time, 2026-04-08，**旧方案 ESKF+GTSAM Hybrid**）

> ⚠️ FK-only 在同样 bag 上结果完全不同（path ratio 1.005，详见 `doc/fk_only_odometry.md`）。下面的数字仅作为弃用旧方案的依据保留。


第一次跑真实 bag (`rosbag2_2026_04_07-16_12_13`, 103 办公室, 起终点应重合, 334s)：

| 指标 | 值 |
|---|---|
| 估计 XY 路径长度 | **224 m**（办公室不可能这么长） |
| 起终点 XY 距离 | 109 m |
| **drift / path** | **48.7%** |
| Z range | 0.003 m（flat-z 约束生效） |

**关键观察：路径长度严重 over-count**——不只是 drift 大，是速度估计完全飞了。Sim 上同等"难度"是 curve_walk 49%，实机也是 49%——巧合地相同。

**Why:** 第一次有真机数据点。memory 没存的话，下次回来很容易把 sim 的数字当成真机表现。

**How to apply:** 下次讨论"算法精度"时区分 sim 和实机；调参方向应该先做 sim-to-real 而不是改算法结构。

## Sim-to-Real 假设（待验证）

实机 224m 路径长度的可能原因（按嫌疑排序）：

1. **接触检测阈值不匹配**：sim 调出来的 `effort_threshold=5.0` Nm。实机 LJPITCH/RJPITCH 静态都在 7.4 Nm 量级，可能让 contact 状态反常（"双脚一直接触"或"始终 swing"），ZUPT 失效
2. **IMU 噪声远高于 sim**：`accel_noise=0.1` 是 sim 假设；真实 IMU 待测
3. **关节零位/方向未在实机校准**：`config/joint_mapping.yaml` 里 `joint_offset` 全 0、`joint_sign` 全 1，可能从未在真机验证

**Why:** 不验证就改算法等于盲目。

**How to apply:** 下次工作 leg_odometry 实机精度时，先按这三条排查；写诊断脚本看 contact 状态时间序列、ZUPT 触发率、estimated velocity 大小。

---

## 构建/工具链注意事项（2026-04-08）

- **平台**：aarch64 (NVIDIA Spark/Jetson 类)，ROS **Jazzy**（不是 CLAUDE.md 写的 Humble）
- **GTSAM 4.3**：在 `/usr/local/`（aarch64 系统安装），CMakeLists 用 `find_package(GTSAM REQUIRED)` 不要加 HINTS
- **历史坑**：`~/Documents/local/gtsam-4.3/` 曾经是 x86_64 stale 二进制（已清理 2026-04-08）。如果未来 CMakeLists 又出现 GTSAM HINTS 指向某个具体路径，多半是不必要的
- **CMakeCache 污染**：跨多次失败 build 时 CMakeCache 里 `GTSAM_INCLUDE_DIR` 是 CACHE PATH，新的 `find_package` 不会覆盖，必须 `rm -rf build/leg_odometry` 完全清掉再 colcon build。GTSAMConfig.cmake 在第 9 行用 `set(... CACHE PATH)` 写 include dir
- **KDL/URDF**：用 `find_package(kdl_parser)` `find_package(urdf)` + `ament_target_dependencies(... kdl_parser urdf)`，不要硬编码 `/opt/ros/<distro>` 路径

**Why:** 这些都是今天 build leg_odometry 实际踩过的坑，下次再遇到能直接跳到解法。

## 测试基础设施教训

- **`ros2 bag record` 通过 python wrapper 启动，SIGINT 不能可靠传递到底层 recorder**。两个 recorder 同时写一个 mcap 会让文件 corrupt（mcap reader 找不到 footer）
- **替代方案**：写 rclpy subscriber 直接 dump CSV。已实现在 `scripts/dump_leg_odom_csv.py`，配套 `scripts/eval_drift.py` 算 loop-closure drift，`scripts/run_bag_test.sh` 一键串起来
- **进程清理**：测试结束后用 `pkill -KILL -f "leg_odom_node\|robot_state_publisher"` 兜底，否则 orphan recorder 会干扰下次运行

**Why:** 一晚上踩了两次"orphan recorder + corrupt mcap"，下次直接用 rclpy 路径

---

## 上次 review 发现的未修复 issue（2026-04-08 验证仍在代码里）

---

## 上次 review 发现的未修复 issue（2026-04-08 验证仍在代码里）

按优先级排序：

1. ~~【最有价值】GTSAMSmoother 没用速度类 factor~~ **【2026-04-15 验证负结果】**
   - 已接入 `FlatZVelocityFactor`（保留），`FootVelocityFactor` 试过后删除
   - sim 9 场景回归：Foot+FlatZ 7/9 变差（straight_fast +2.3pp）；仅 FlatZVelocity 均值 -0.07pp 基本中性
   - 结论：后端缺速度约束不是 drift 主因。`BetweenFactor<Point3>(FLk(i),FLk(j),0)` 已是积分版 ZUPT，再叠加速度级冗余且和 ImuFactor 打架
   - drift 根源转向前端：gyro bias 漂移 / FK 建模 / contact 误触发

2. ~~leg_odom_node.cpp init bug~~ **【2026-04-15 已修】** 未静止时清空 sum/buf/count 重来
3. ~~smoother bias_walk 写死~~ **【2026-04-15 已修】** 新增 `smoother_accel_bias_walk` / `smoother_gyro_bias_walk` 参数
4. ~~UpdaterZUPT dt_fk 硬编码~~ **【2026-04-15 已修】** 改用 `s.last_dt`（fallback 0.005）
5. ~~State::initialize b_a 表达式~~ **【2026-04-15 已修】** 改为 `b_a.setZero()`

6. **GTSAMSmoother 每次重建整张图**
   - `include/leg_odometry/smoother/GTSAMSmoother.h:78-79` 每次 optimize 从零做 LM
   - 不影响当前精度（20Hz 关键帧够），但提到 "100-200Hz" 关键帧方向时会变瓶颈
   - 升级方向：iSAM2 + marginalization

7. **FootVelocityFactor 用数值微分**
   - `include/leg_odometry/smoother/leg_factors.h:108-120` 用 `numericalDerivative21`
   - 注释写 "Numerical derivative for simplicity"
   - 当前没被 smoother 用上（见 issue 1），如果启用需要换解析雅可比

## 其他观察

- node (leg_odom_node.cpp) 和 hybrid (leg_odom_hybrid.cpp) 的 init 路径不对称：node 有 gyro_std 静止检测但有 reset bug；hybrid 直接平均前 50 帧无检测（仿真里 OK，跑实机会有问题）
- 节点里 FK 用 `noise_.sigma_zupt` 没 declare_parameter（其实 declare 了，是 declare 完但没存到 noise_，行 47 有声明）—— 这条不算 bug，已声明
