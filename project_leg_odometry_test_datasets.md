---
name: leg_odometry 真实数据集入口与测试优先级
description: CASBot02 腿式里程计真实 bag 清单、proposal 指定测试顺序、以及每个 bag 的正确使用方式
type: project
---

真实数据在 `/home/steve/Documents/Datasets/casbot/leg/`（注意是小写 `casbot`，proposal 里写的 `CASBOT` 是错的路径），带 `readme.md`。

**Why:** 测实机精度时很容易随手抓一个 loop bag 跑（memory 里 2026-04-08 48.7% drift 就是这么来的），但 proposal 阶段 2 明确指定了先测"纯直行（数据集 1）+ 纯旋转（数据集 4）"这两个最简单的模式，跳过它们直接跑 loop 会让调参找不到问题根因。

**How to apply:** 讨论实机 drift / sim-to-real 调参时，先确认测的是哪个数据集。按以下优先级：

1. **数据集 4** (`rosbag2_2026_04_10-18_56_38`) — 103办公室原地转圈，半径 ~0.5m，从静止开始。*最适合作为首测*：纯 yaw 运动 + 平移小，能单独暴露 gyro bias / yaw 漂移问题。
2. **数据集 1** (`rosbag2_2026_03_18-16_35_30`) — "直线 190m，机器人被吊着没有实际运动"。**特殊**：身体静止但关节在走路动作。**期望里程计输出 ≈ 原地不动**。用来看接触检测和 ZUPT 是否把"悬空迈步"误判成"走路"。不是真正的直行里程计精度测试。
3. **数据集 2** (`rosbag2_2026_04_07-16_12_13`) — 103办公室 loop，335s，有 `traj_imu.txt` 真值，有效段 1775549558–1775549742（184s）。**真正能算 drift 的 bag 之一**。memory 里 2026-04-08 drift 48.7% 用的就是这个。
4. **数据集 3** (`rosbag2_2026_04_07-17_15_31`) — B1 停车场 loop，436s，有真值，有效段 1775553350–1775553530（180s）。和数据集 2 一样能算 drift，场景更大。

其他 bag：`rosbag2_2026_04_10-18_55_06` 和 `rosbag2_2026_04_10-18_55_32` 在 readme 中**未说明**，用之前先确认用途，不要盲测。

## 每个 bag 的有效段处理

bag 2 和 3 的 readme 说"XXX 前是静止的，XXX 之前的轨迹数据是有效的，超过的数据没有意义"——测试脚本 (`scripts/dump_leg_odom_csv.py` / `scripts/eval_drift.py`) 需要按这个时间戳裁剪，否则 drift 计算会包含无效段。

## traj_imu.txt 真值

bag 2、3 附带 `traj_imu.txt`（IMU 坐标系轨迹），是 loop-closure 评估的参考真值。bag 1、4 没有真值文件：
- bag 1 靠"期望不动"判断
- bag 4 靠"起点终点接近 + 转 N 圈" 粗略判断
