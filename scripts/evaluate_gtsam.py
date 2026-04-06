#!/usr/bin/env python3
"""GTSAM 因子图腿式里程计离线评估。

回放仿真 rosbag，运行 GTSAM ISAM2，对比 ground truth。
复用 evaluate_sim.py 的指标和绘图函数。

用法:
  python3 evaluate_gtsam.py [--data-dir data/sim] [--scenario SCENARIO|all]
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from evaluate_sim import (
    read_bag, deserialize, compute_metrics, plot_results,
    JOINT_MAPPING, load_ekf_config, load_kinematics, HAS_PLT,
)
from leg_odometry.gtsam_odom import GTSAMLegOdometry
from leg_odometry.contact_detector import ContactDetector


def run_gtsam_offline(bag_dir: str):
    """回放 bag 并运行 GTSAM，返回估计轨迹和 ground truth。"""

    ekf_cfg = load_ekf_config()
    kinematics = load_kinematics()

    params = ekf_cfg.get('ekf', {})
    gtsam_odom = GTSAMLegOdometry(params)

    contact_cfg = ekf_cfg.get('contact', {})
    contact_det = ContactDetector(
        threshold=contact_cfg.get('effort_threshold', 5.0),
        hysteresis=contact_cfg.get('hysteresis', 1.0),
    )

    messages = read_bag(bag_dir)

    est_times = []
    est_positions = []
    est_orientations = []
    gt_times = []
    gt_positions = []
    gt_orientations = []

    latest_joints = {}
    latest_efforts = {}
    last_imu_ts = None
    init_count = 0
    init_accel_sum = np.zeros(3)
    INIT_FRAMES = 50

    for topic, data, ts_ns in messages:
        msg = deserialize(topic, data)
        if msg is None:
            continue

        if topic == '/ground_truth/odom':
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            gt_times.append(ts_ns * 1e-9)
            gt_positions.append([p.x, p.y, p.z])
            gt_orientations.append([o.x, o.y, o.z, o.w])
            continue

        if topic == '/joint_states':
            for i, name in enumerate(msg.name):
                if len(msg.effort) > i:
                    latest_efforts[name] = msg.effort[i]
                urdf_name = JOINT_MAPPING.get(name)
                if urdf_name and len(msg.position) > i:
                    latest_joints[urdf_name] = msg.position[i]
            continue

        if topic == '/imu':
            accel = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ])
            gyro = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ])

            # 初始化
            if not gtsam_odom.initialized:
                init_accel_sum += accel
                init_count += 1
                if init_count >= INIT_FRAMES and latest_joints:
                    avg_accel = init_accel_sum / init_count
                    p_fl = kinematics.fk_left(latest_joints)
                    p_fr = kinematics.fk_right(latest_joints)
                    gtsam_odom.initialize(avg_accel, p_fl, p_fr)
                continue

            # dt
            t_sec = ts_ns * 1e-9
            if last_imu_ts is None:
                last_imu_ts = t_sec
                continue
            dt = t_sec - last_imu_ts
            last_imu_ts = t_sec
            if dt <= 0 or dt > 0.1:
                continue

            # 接触检测
            effort_l = latest_efforts.get('LJPITCH', 0.0)
            effort_r = latest_efforts.get('RJPITCH', 0.0)
            contact_l, contact_r = contact_det.update(effort_l, effort_r)

            # 预积分 IMU
            gtsam_odom.add_imu(accel, gyro, dt)

            # 关键帧
            if gtsam_odom.should_add_keyframe() and latest_joints:
                p_fl = kinematics.fk_left(latest_joints)
                p_fr = kinematics.fk_right(latest_joints)
                gtsam_odom.add_keyframe(p_fl, p_fr, contact_l, contact_r)

                pos, quat = gtsam_odom.get_pose()
                est_times.append(t_sec)
                est_positions.append(pos.copy())
                est_orientations.append(quat.copy())

    # 按时间戳对齐 GT 到关键帧时刻（GT 200Hz, EST 20Hz）
    est_t = np.array(est_times)
    gt_t = np.array(gt_times)
    gt_pos_all = np.array(gt_positions)
    gt_ori_all = np.array(gt_orientations)

    if len(est_t) > 0 and len(gt_t) > 0:
        # 对每个 est 时刻，找最近的 gt 时刻
        gt_idx = np.searchsorted(gt_t, est_t, side='left')
        gt_idx = np.clip(gt_idx, 0, len(gt_t) - 1)
        gt_pos_aligned = gt_pos_all[gt_idx]
        gt_ori_aligned = gt_ori_all[gt_idx]
        gt_t_aligned = gt_t[gt_idx]
    else:
        gt_pos_aligned = gt_pos_all
        gt_ori_aligned = gt_ori_all
        gt_t_aligned = gt_t

    return {
        'est_t': est_t,
        'est_pos': np.array(est_positions),
        'est_ori': np.array(est_orientations),
        'gt_t': gt_t_aligned,
        'gt_pos': gt_pos_aligned,
        'gt_ori': gt_ori_aligned,
    }


def main():
    parser = argparse.ArgumentParser(description='GTSAM leg odometry 评估')
    parser.add_argument('--data-dir', '-d', default='data/sim')
    parser.add_argument('--scenario', '-s', default='all')
    parser.add_argument('--output-dir', '-o', default='data/sim/results_gtsam')
    args = parser.parse_args()

    data_dir = os.path.join(os.getcwd(), args.data_dir)
    output_dir = os.path.join(os.getcwd(), args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    if args.scenario == 'all':
        scenarios = sorted([
            d for d in os.listdir(data_dir)
            if os.path.isdir(os.path.join(data_dir, d))
            and d != 'results' and d != 'results_gtsam'
        ])
    else:
        scenarios = [args.scenario]

    print(f"[GTSAM] Evaluating {len(scenarios)} scenarios")
    print(f"Output: {output_dir}")
    print()

    all_metrics = {}

    for name in scenarios:
        bag_dir = os.path.join(data_dir, name)
        if not os.path.isdir(bag_dir):
            continue

        print(f"[{name}]")
        try:
            result = run_gtsam_offline(bag_dir)

            if len(result['est_pos']) < 2:
                print(f"  SKIP - too few estimates ({len(result['est_pos'])})")
                continue

            metrics, err_xy, err_z, err_3d, yaw_err = compute_metrics(result)
            all_metrics[name] = metrics

            print(f"  Distance:  {metrics['distance_m']:.1f} m")
            print(f"  RMSE XY:   {metrics['rmse_xy_m']*100:.2f} cm")
            print(f"  RMSE Z:    {metrics['rmse_z_m']*100:.2f} cm")
            print(f"  Final XY:  {metrics['final_err_xy_m']*100:.2f} cm")
            print(f"  Drift XY:  {metrics.get('drift_percent_xy', float('nan')):.3f} %")
            print(f"  RMSE Yaw:  {metrics['rmse_yaw_deg']:.2f}°")

            plot_results(result, metrics, err_xy, err_z, yaw_err,
                         f'{name}_gtsam', output_dir)

        except Exception as e:
            print(f"  ERROR: {e}")
            import traceback
            traceback.print_exc()
        print()

    if all_metrics:
        print("=" * 90)
        print(f"{'Scenario':<20} {'Dist(m)':>8} {'RMSE_XY(cm)':>12} {'RMSE_Z(cm)':>11} "
              f"{'Final_XY(cm)':>13} {'Drift(%)':>9} {'RMSE_Yaw(°)':>12}")
        print("-" * 90)
        for name, m in all_metrics.items():
            drift = m.get('drift_percent_xy', float('nan'))
            drift_str = f"{drift:.3f}" if not np.isnan(drift) else "N/A"
            print(f"{name:<20} {m['distance_m']:>8.1f} {m['rmse_xy_m']*100:>12.2f} "
                  f"{m['rmse_z_m']*100:>11.2f} {m['final_err_xy_m']*100:>13.2f} "
                  f"{drift_str:>9} {m['rmse_yaw_deg']:>12.2f}")
        print("=" * 90)

        csv_path = os.path.join(output_dir, 'summary_gtsam.csv')
        with open(csv_path, 'w') as f:
            keys = list(next(iter(all_metrics.values())).keys())
            f.write('scenario,' + ','.join(keys) + '\n')
            for name, m in all_metrics.items():
                vals = [str(m[k]) for k in keys]
                f.write(name + ',' + ','.join(vals) + '\n')
        print(f"\nSaved: {csv_path}")


if __name__ == '__main__':
    main()
