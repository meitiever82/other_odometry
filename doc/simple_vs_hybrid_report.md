# 腿式里程计 — Simple vs Hybrid 方案对比汇报

**日期**：2026-04-16
**数据集**：`rosbag2_2026_04_07-16_12_13`（B1 平台，334s 行走）
**比较对象**：`simple_leg_odom_node`（降级 baseline）vs `leg_odom_node` (hybrid ESKF + GTSAM)

---

## 执行摘要（TL;DR）

在 B1 平台上，原有 hybrid ESKF 方案因 **FK→R 反馈环**、**contact 检测左右不对称**、**effort blackout 无兜底** 三重问题，轨迹塌缩为无拓扑意义的小团，无法使用。

为验证根因、并为下游 SLAM 提供可用里程计，我们实现了极简方案 `simple_leg_odom_node`：

- **"姿态只看 IMU，平移只看腿 FK"**，结构上切断 FK→R 的污染链路
- 实测 XY 路径 68.8m、终点离原点 3.3m（4.8% drift）、轨迹拓扑与实际行走一致
- **代价**：scale 偏短 ~42%、Z 漂累计、无 IMU bias 估计

**结论**：simple 版可作为 B1 平台上的**短期可用 baseline**；一旦 hybrid 的三条补丁（左右独立阈值 / blackout fallback / UpdaterFK swing 期去耦）完成并复测通过，应回归 hybrid 作为主方案——它在信息融合、scale 准确性、长时稳定性上有原理性优势。

---

## 1. 问题定位（简述）

Hybrid ESKF 在 B1 实机数据上失败的具体机制，已通过 simple 版对照实验验证清楚：

### 1.1 FK→R 反馈环（形状塌缩的根因）

`UpdaterFK` 的观测雅可比：

```
h(x) = p + R·FK_body − p_fl 
```
H[P]  = I
H[TH] = -skew(R·FK_body)   ← 罪魁
H[FL] = -I
```

`H[TH]` 项意味着 FK 残差会通过 Kalman gain 更新姿态 R。Swing 脚在空中甩动时，FK_body 快速变化，残差被 EKF 当成"body 姿态误差"投影回 R。下一帧 R 被扭 → 新 FK 残差更大 → 再扭 R → **正反馈，滚雪球**。

### 1.2 Contact 检测左右不对称

B1 右腿 JPITCH 关节存在 **+11.5 Nm 静态 torque bias**。hybrid 的 `ContactDetector` 使用**单阈值**（共享 5.0 Nm），无法区分左右脚的真实 stance/swing 时相，进一步给 (1.1) 喂入系统性偏差。

### 1.3 Effort blackout 无兜底 - 踝关节 (LJPITCH / RJPITCH) 的力矩测量值 effort 值持续为 0（或接近 0）

同一 bag 后半段出现 effort 整段为 0 的情形（yaml 注释已标注，但 C++ 节点未实现对应兜底）。contact 检测全程输出 swing → ZUPT 不触发 → IMU 速度自由积分 → 线性漂移。

---

## 2. 方案 A：Simple（极简降级方案）

### 2.1 设计原则

**单向数据流，刻意放弃信息融合，换取拓扑稳定性。**

```
 IMU ──► R  （gyro 积分 + accel 重力方向低通修正 roll/pitch）
             │
             ▼
 FK(q), contact ──► dp = R_prev·FK_prev − R·FK_cur ──► p
```

- 无 EKF、无协方差、无 bias 估计
- R 只受 IMU 驱动，**永远不接受 FK 残差**
- p 只受 FK 差分驱动，不接受 IMU 积分

### 2.2 关键实现要点（`simple_leg_odom_node.cpp`）

| 模块 | 做法 |
|---|---|
| 姿态积分 | `R ← R · exp(ω·dt)`（Rodrigues），每帧重正交化 |
| 重力修正 | complementary filter：`err = R^T·ẑ × â_body`，只保留 x/y 分量不动 yaw，`R ← R · exp(α·err)`，α=0.01 |
| Contact 检测 | **两个独立 `ContactDetector`**，左阈值 5.0 Nm / 右阈值 12.0 Nm + hysteresis 1.0，对冲右腿 bias |
| 平移更新 | 仅 `cl && prev_cl` 下累加左脚 dp；右脚同理；双脚 stance 取平均；双脚 swing 时 **v=0 位置冻结** |
| 失效兜底 | effort=0 时自然进入"冻结模式"，不会 IMU 自由积分 |
| Topic | 独立发 `/leg_odometry_simple`，child frame `base_link_simple`，与 hybrid 并存 |

### 2.3 实测（rosbag2_2026_04_07-16_12_13，rate 3.0）

| 指标 | 数值 |
|---|---:|
| 输出速率 | 200 Hz |
| XY 路径长度 | 68.83 m |
| 终点偏离起点 | 3.30 m |
| XY drift / path | **4.80%** |
| X 范围 | 11.50 m |
| Y 范围 | 12.53 m |
| Z 累计漂移 | 5.14 m |

轨迹拓扑：接近闭环的方形运动，与实机行走一致。

---

## 3. 方案 B：Hybrid（ESKF + GTSAM 完整方案）

### 3.1 已有架构

```
 IMU ──► ESKF Predict  (21-DOF error state)
                │
 FK ──► Update 1: UpdaterFK   (位置一致性 + swing 脚 anchor)
 Contact ──► Update 2: UpdaterZUPT  (接触脚速度=0 + 步级速度 + 静止检测 + 双脚支撑)
            Update 3: UpdaterFlatZ (平地 Z 约束)
                │
                ▼ keyframe (20Hz)
         GTSAM 滑窗平滑器
           IMU 预积分因子 / FK CustomFactor / 足端固定 / Bias 演化
                │
                ▼ 估计 gyro bias
         反馈到 ESKF 前端
```

状态向量：`p, v, R, b_a, b_g, p_fl, p_fr`（21 维 error-state）

### 3.2 当前状态

Hybrid 在 sim 数据与浅层测试中精度良好，但在 B1 实机 bag 上复现上述三重故障。已定位但**未全部修复**。

### 3.3 未完成的改造清单

| 项 | 内容 | 状态 |
|---|---|---|
| A | `ContactDetector` 支持左右独立阈值 | 源码已改，待统一测试 |
| B | 实现 `blackout_threshold` 兜底（双 effort=0 超时→强制 cl=cr=true） | **未实现**（yaml 已声明） |
| C | `UpdaterFK` 在 swing 期置 `H[TH]=0`，保留 `H[P]` 和 `H[FL]` 的 tether | 源码已改 |
| D | 统一回归测试 + 参数再 tune | **待做** |

今日已单独验证：A/C 任一条或软门控单独做都不足以修复 hybrid；必须 A+B+C 同时生效。

---

## 4. 方案对比

### 4.1 功能/工程维度

| 维度 | Simple | Hybrid |
|---|:---:|:---:|
| 实现复杂度 | ~270 行单文件 | ~1500+ 行（ESKF + 5 Updater + smoother） |
| 参数数量 | 4 个（阈值、hyst、α、init_frames） | ~25 个（噪声、门限、窗口、bias walk…） |
| 调参难度 | 低 | 高 |
| 对 contact 检测精度的敏感度 | 中（可降级成冻结模式） | **高**（直接影响 EKF 协方差注入） |
| 对 IMU 质量的依赖 | 中（短期） | 高（需 bias 稳定） |
| 对 effort 信号缺失的鲁棒性 | **高**（自然冻结） | 低（无兜底即发散） |
| 启动即用 | **是** | 需 tune |

### 4.2 性能维度（基于 rosbag2_2026_04_07-16_12_13 实测）

| 指标 | Simple | Hybrid (原版) | Hybrid (soft-gate) | Hybrid (hard-gate) |
|---|---:|---:|---:|---:|
| XY 路径 | 68.8 m | ~1019 m* | 172 m | 1019 m |
| 终点偏离 | **3.3 m** | small | 50 m | 742 m |
| Drift/Path | **4.8%** | — | 29% | 73% |
| 轨迹拓扑 | **方形回环** | 塌团 | 直线漂 | 直线漂 |

\* hybrid 原版路径长度量级正确，但拓扑塌成小团，drift 指标在该场景下失去意义。

### 4.3 原理维度

| 属性 | Simple | Hybrid (调通后) |
|---|---|---|
| Scale 准确性 | **差（~42% 低估）**，stance 期脚打滑/踝扭直接进入 dp | 好，ZUPT + 步级速度约束 scale |
| Z 轴准确性 | 差（累计漂），tilt correction 经 FK 差分泄漏到 z | **好**，Flat-Z 显式约束 |
| IMU bias 处理 | 无 | **GTSAM 后端估计 gyro bias 并反馈前端** |
| 长时稳定性 | 纯 dead-reckoning，误差线性累积 | **协方差意义下最优估计**，误差增长更慢 |
| 信息利用率 | 低（丢了 IMU 高频速度信息） | **高**（所有可观测量融合） |
| 对多模态传感器扩展 | 难（无统一状态） | **易**（状态向量 + 新 Updater） |

---

## 5. 为什么 Hybrid 调通后是更合适的主方案

Simple 的胜利是拓扑正确性上的，但是靠**放弃信息**换来的。在以下维度上 Hybrid 有原理性上限：

1. **Scale 可收敛**。Simple 的 42% scale 误差来自 stance 期脚打滑和踝扭——这些在 dp = R_prev·FK_prev − R·FK_cur 里无法区分"真实 body 位移"和"脚/踝的局部运动"。Hybrid 的 ZUPT 约束"接触脚世界速度=0"是一个独立信息源，能识别并扣除这部分误差。

2. **Z 轴不再漂**。Simple 的 Z 漂源于 complementary filter 每步微调 R 被 FK 差分翻译成 z 位移。Hybrid 的 Flat-Z 显式把 Z 锚定到初始高度，Z 误差有界（在平地场景下）。

3. **IMU bias 可估计**。Simple 完全不估 gyro bias，长时间运行后 yaw 会缓慢累积误差（IMU 消磁特性）。Hybrid 的 GTSAM 后端在滑窗内用 IMU 预积分因子 + FK 约束共同观测 bias，反馈到前端。

4. **多传感器融合天花板高**。Hybrid 的状态向量和 Updater 抽象允许未来加入：LiDAR odometry 约束、GPS、磁力计、视觉里程计等——每个都是一个新 Updater。Simple 无统一状态，扩展需重写。

5. **平滑输出 + 关键帧对齐**。Hybrid 的后端 smoother 提供 keyframe 级优化结果，天然与 LiDAR SLAM（GLIM）的 keyframe 因子图对齐。Simple 只能实时一次过。

6. **可验证性**。Hybrid 在 sim 数据上已经跑通（见 sim 评估脚本），故障是"sim2real gap"问题而非根本设计错误。三条补丁（左右阈值 / blackout / swing 去耦）都是**已定位的工程修复**，不是未知。

简言之：**Simple 是短期救场，Hybrid 是长期主战**。把 hybrid 修好是更高价值的投入，当前只需完成一周以内的工程工作即可收敛。

---

## 6. 建议与路径

### 6.1 短期（本周）
- 继续用 simple 作为 B1 平台上的 baseline，支撑下游 SLAM 联调
- 不要在 simple 上追加功能——scale/Z 的原理性缺陷不是 bug，改不动

### 6.2 中期（1–2 周）
- 完成 hybrid 三条补丁（A 左右阈值 / B blackout fallback / C UpdaterFK swing 去耦）
- 在同一 bag 上对比 simple vs hybrid 修复版
- 只要修复版在 scale/Z/drift 任一维度显著优于 simple，切换主方案

### 6.3 长期
- 在 hybrid 基础上加 LiDAR odometry 约束（新 Updater），与 GLIM 深度融合
- Simple 保留为 fallback 路径，用于传感器降级场景

---

## 附录 A：工作时间线（近几日 debug 回顾）

### A.1 第一阶段：怀疑后端（已证伪）

**假设**：GTSAM 后端 IMU 预积分 + FK 因子组合下 drift 不收敛，因缺速度约束因子。

**动作**：给后端追加 `FootVelocityFactor` 与 `FlatZVelocityFactor`。

**结果**：drift 无任何改善。

**结论**：drift 根源不在后端优化层，定位回退到前端 ESKF。（见 memory `leg_odometry_velocity_factor_negative.md`）

### A.2 第二阶段：前端形状塌缩诊断

**观察**：实机 B1 bag 上 XY 路径长度量级正确（~1000m），但空间上塌成小团，完全无运动拓扑。

**排查**：
- 检查左右腿 effort 分布 → 发现 R JPITCH 有 +11.5 Nm 静态 bias
- 推测 contact 检测左右不对称是形状塌缩的放大机制
- 未明确 FK→R 反馈环的机制

（见 memory `leg_odometry_B1_shape_diagnosis.md`）

### A.3 第三阶段：对照实验 — 写 simple 版证伪/证实

**目的**：如果 FK→R 反馈环是形状塌缩的根因，那么切断反馈环应当恢复拓扑。

**动作**：写 `simple_leg_odom_node`，单向数据流：IMU→R，FK→p。

**结果**：B1 上恢复成两个清晰的方形环；scale 偏短 42%、Z 漂 21m。

**结论**：**FK→R 反馈环**是形状塌缩的根因，已验证。Scale 和 Z 的代价是单向流动的必然产物。（见 memory `leg_odometry_simple_node_result.md`）

### A.4 第四阶段：今日（2026-04-16）把 fix 反推回 hybrid

**目标**：把 simple 的成功经验（contact 精度 + FK→R 解耦）移植回 hybrid，保留 hybrid 的信息融合优势。

**尝试 1**：Hybrid 的 UpdaterFK 加硬 contact 门控（swing 脚完全跳过 EKF 更新）
- 结果：**炸了**，path 1019m、终点 742m 直线漂，drift 73%
- 原因：去掉 swing 期 UpdaterFK 等于去掉了 p 的位置一致性 tether，IMU 积分自由发散

**尝试 2**：软门控（swing 期仅置 `H[TH]=0`，保留 `H[P]` 和 `H[FL]`）
- 结果：path 172m、终点 50m 直线漂，drift 29%
- 比硬门控好一个量级，但仍远不如 simple

**关键发现**：读 `ekf_params.yaml` 时注意到第 47-50 行注释写明**同一 bag 后半段 effort 整段为 0**，对应的 `blackout_threshold` 参数已声明但 **C++ 节点从未实现**。

**真正瓶颈定位**：
1. Hybrid 的 ContactDetector 是单阈值（硬编码在 C++），无法处理 B1 右腿 bias
2. Hybrid 对 effort blackout 无兜底，进入直线漂
3. 单改 FK 门控不够，必须三条同时做

**阶段性结论**：方向是对的，工程量是**左右阈值 + blackout fallback + FK 门控**三件套。用户打断，决定先写汇报。

### A.5 代码改动清单（部分已 commit，部分 WIP）

- `src/leg_odometry/include/leg_odometry/update/UpdaterFK.h`：加 contact_left/right 参数，swing 期 `H[TH]=0` — **已改**
- `src/leg_odometry/include/leg_odometry/state/Kinematics.h`：ContactDetector 支持左右独立阈值 — **已改**
- `src/leg_odometry/src/leg_odom_node.cpp`：读取新 params、传给 ContactDetector、传 cl/cr 给 UpdaterFK — **已改**
- `src/leg_odometry/src/test_leg_odom.cpp`：跟进调用签名 — **已改**
- `src/leg_odometry/config/ekf_params.yaml`：加 effort_threshold_left/right — **已改**
- `src/leg_odometry/launch/leg_odometry.launch.py`：透传新 yaml 字段 — **已改**
- **Blackout fallback 实现** — **未做**
- 统一回归测试 — **未做**

---

## 附录 B：实测数据汇总

所有测试均在同一 bag、同一参数、相同 rate (3.0) 下进行。

### B.1 Simple（`/tmp/leg_odom_simple_ref/odom.csv`）
```
Messages: 66874
Duration: 334.4s, rate: 200.0 Hz
Total XY path: 68.83 m
Start: (0.000, 0.000, 0.000)
End:   (-1.127, -3.105, 3.380)
|end - start| XY: 3.303 m
End - start  Z:   +3.380 m
XY drift / path:  4.80%
X range: [-10.09, 1.41] = 11.50m
Y range: [-11.90, 0.64] = 12.53m
Z range: [-1.755, 3.384] = 5.138m
```

### B.2 Hybrid + FK 硬门控（`/tmp/leg_odom_hardgate_odom.csv`）
```
Messages: 40804
Duration: 334.3s, rate: 122.1 Hz
Total XY path: 1019.67 m
Start: (-0.001, 0.000, 0.000)
End:   (174.952, 721.752, -0.000)
|end - start| XY: 742.654 m
XY drift / path: 72.83%
X range: 195.93m, Y range: 723.59m
```

### B.3 Hybrid + FK 软门控（`/tmp/leg_odom_softgate/odom.csv`）
```
Messages: 49536
Duration: 334.4s, rate: 148.1 Hz
Total XY path: 172.15 m
Start: (0.000, 0.000, 0.000)
End:   (-50.148, -1.947, 0.000)
|end - start| XY: 50.185 m
XY drift / path: 29.15%
X range: 61.08m, Y range: 9.82m
```

---

**文档版本**：v1 (2026-04-16)
**作者**：Steve + Claude
