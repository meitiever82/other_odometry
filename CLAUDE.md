# Leg Odometry — Project Context

## 架构
C++ ESKF 200Hz 前端 + GTSAM LM 滑窗后端 (gyro bias 校正)

## 代码
- **src/** — C++ 主代码 (State/Propagator/Updaters/Smoother)
  - `leg_odom_node.cpp` — ROS2 实时节点
  - `leg_odom_hybrid.cpp` — 离线评估 (CSV)
- **python/** — Python 参考实现 (ekf.py, smoother.py, symlink: leg_odometry -> python)
- **scripts/** — 仿真生成、评估、数据转换
- **config/ekf_params.yaml** — 所有噪声参数

## 精度
直线 1.8%, 斜坡 2.4%, 旋转 4-7%, 真实跑步机 6.4%
纯腿式里程计天花板已到，下一步: 接入 LiDAR (GLIM) 或视觉 (Cerberus)

## 关键决策
- accel bias / bv 速度 bias 不可观（需外部传感器）
- Jacobian 关节速度不如有限差分 dFK/dt（编码器噪声放大）
- GTSAM 独立方案不如 ESKF（keyframe 频率低）
- ESKF+GTSAM Hybrid 是最优

## 依赖
GTSAM 4.3 (~/Documents/local/gtsam-4.3/), KDL, ROS2 Humble

## 用户
Steve，中文沟通，务实风格
