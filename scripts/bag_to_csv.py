#!/usr/bin/env python3
"""将仿真 rosbag 导出为 CSV，供 C++ GTSAM 程序读取。

输出列:
  timestamp,ax,ay,az,gx,gy,gz,fk_lx,fk_ly,fk_lz,fk_rx,fk_ry,fk_rz,
  contact_l,contact_r,gt_x,gt_y,gt_z,gt_qx,gt_qy,gt_qz,gt_qw
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from evaluate_sim import (
    read_bag, deserialize, JOINT_MAPPING, load_ekf_config, load_kinematics,
)
from leg_odometry.contact_detector import ContactDetector


def bag_to_csv(bag_dir: str, output_path: str):
    ekf_cfg = load_ekf_config()
    kinematics = load_kinematics()
    contact_cfg = ekf_cfg.get('contact', {})
    contact_det = ContactDetector(
        threshold=contact_cfg.get('effort_threshold', 5.0),
        hysteresis=contact_cfg.get('hysteresis', 1.0),
    )

    messages = read_bag(bag_dir)

    latest_joints = {}
    latest_efforts = {}
    gt_cache = {}  # ts_ns -> (pos, quat)

    # First pass: collect GT
    for topic, data, ts_ns in messages:
        msg = deserialize(topic, data)
        if msg is None:
            continue
        if topic == '/ground_truth/odom':
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            gt_cache[ts_ns] = ([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])

    # Second pass: write CSV
    with open(output_path, 'w') as f:
        f.write('timestamp,ax,ay,az,gx,gy,gz,'
                'fk_lx,fk_ly,fk_lz,fk_rx,fk_ry,fk_rz,'
                'contact_l,contact_r,'
                'gt_x,gt_y,gt_z,gt_qx,gt_qy,gt_qz,gt_qw\n')

        for topic, data, ts_ns in messages:
            msg = deserialize(topic, data)
            if msg is None:
                continue

            if topic == '/joint_states':
                for i, name in enumerate(msg.name):
                    if len(msg.effort) > i:
                        latest_efforts[name] = msg.effort[i]
                    urdf_name = JOINT_MAPPING.get(name)
                    if urdf_name and len(msg.position) > i:
                        latest_joints[urdf_name] = msg.position[i]
                continue

            if topic == '/imu' and latest_joints:
                accel = [msg.linear_acceleration.x,
                         msg.linear_acceleration.y,
                         msg.linear_acceleration.z]
                gyro = [msg.angular_velocity.x,
                        msg.angular_velocity.y,
                        msg.angular_velocity.z]

                fk_l = kinematics.fk_left(latest_joints)
                fk_r = kinematics.fk_right(latest_joints)

                effort_l = latest_efforts.get('LJPITCH', 0.0)
                effort_r = latest_efforts.get('RJPITCH', 0.0)
                cl, cr = contact_det.update(effort_l, effort_r)

                gt_pos, gt_quat = gt_cache.get(ts_ns, ([0, 0, 0], [0, 0, 0, 1]))

                t = ts_ns * 1e-9
                f.write(f'{t:.6f},'
                        f'{accel[0]:.6f},{accel[1]:.6f},{accel[2]:.6f},'
                        f'{gyro[0]:.6f},{gyro[1]:.6f},{gyro[2]:.6f},'
                        f'{fk_l[0]:.6f},{fk_l[1]:.6f},{fk_l[2]:.6f},'
                        f'{fk_r[0]:.6f},{fk_r[1]:.6f},{fk_r[2]:.6f},'
                        f'{int(cl)},{int(cr)},'
                        f'{gt_pos[0]:.6f},{gt_pos[1]:.6f},{gt_pos[2]:.6f},'
                        f'{gt_quat[0]:.6f},{gt_quat[1]:.6f},{gt_quat[2]:.6f},{gt_quat[3]:.6f}\n')


def main():
    parser = argparse.ArgumentParser(description='Rosbag to CSV for C++ GTSAM')
    parser.add_argument('--data-dir', '-d', default='data/sim')
    parser.add_argument('--output-dir', '-o', default='data/sim/csv')
    parser.add_argument('--scenario', '-s', default='all')
    args = parser.parse_args()

    data_dir = os.path.join(os.getcwd(), args.data_dir)
    output_dir = os.path.join(os.getcwd(), args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    if args.scenario == 'all':
        scenarios = sorted([
            d for d in os.listdir(data_dir)
            if os.path.isdir(os.path.join(data_dir, d))
            and not d.startswith('results') and d != 'csv'
        ])
    else:
        scenarios = [args.scenario]

    for name in scenarios:
        bag_dir = os.path.join(data_dir, name)
        csv_path = os.path.join(output_dir, f'{name}.csv')
        print(f'[{name}] -> {csv_path}')
        bag_to_csv(bag_dir, csv_path)

    print('Done!')


if __name__ == '__main__':
    main()
