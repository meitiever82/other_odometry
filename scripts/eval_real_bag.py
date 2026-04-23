#!/usr/bin/env python3
"""Offline EKF replay on a real robot bag with selectable contact detector.

Compares estimated XY trajectory against traj_imu.txt (the only reference we
have on the real bag — 103 office loop, start≈end).

Usage:
    python3 eval_real_bag.py <bag_dir>
                             [--contact raw|detrend|both]
                             [--t-abs-lo T --t-abs-hi T]
                             [--out DIR]
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy.serialization
from sensor_msgs.msg import JointState, Imu

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from leg_odometry.ekf import BloeSchEKF
from leg_odometry.kinematics import LegKinematics
from leg_odometry.contact_detector import (
    ContactDetector, BiasCompensatedContactDetector)

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent
CONFIG_DIR = PKG_DIR / 'config'
URDF_PATH = PKG_DIR.parent / 'finder_lidar_mapping' / 'glim_ros2' / 'urdf' / 'casbot02_7dof_shell.urdf'

JOINT_MAPPING = {
    'LJ0': 'left_leg_pelvic_pitch_joint',
    'LJ1': 'left_leg_pelvic_roll_joint',
    'LJ2': 'left_leg_pelvic_yaw_joint',
    'LJ3': 'left_leg_knee_pitch_joint',
    'LJPITCH': 'left_leg_ankle_pitch_joint',
    'LJROLL': 'left_leg_ankle_roll_joint',
    'RJ6': 'right_leg_pelvic_pitch_joint',
    'RJ7': 'right_leg_pelvic_roll_joint',
    'RJ8': 'right_leg_pelvic_yaw_joint',
    'RJ9': 'right_leg_knee_pitch_joint',
    'RJPITCH': 'right_leg_ankle_pitch_joint',
    'RJROLL': 'right_leg_ankle_roll_joint',
}


class PrecomputedContactDetector:
    """Detector that looks up stance from a precomputed (t_abs, cl, cr) timeline.

    Used for "zproxy" mode where stance is derived from FK foot-z minima — an
    independent kinematic signal — before the EKF run.
    """
    def __init__(self, t_array, cl_array, cr_array):
        self._t  = np.asarray(t_array)
        self._cl = np.asarray(cl_array, bool)
        self._cr = np.asarray(cr_array, bool)
        self._i  = 0
        self._cur_t = None

    def set_time(self, t_abs):
        self._cur_t = t_abs

    def update(self, effort_left, effort_right, fk_z_left=None, fk_z_right=None):
        t = self._cur_t
        while self._i + 1 < len(self._t) and self._t[self._i + 1] <= t:
            self._i += 1
        return bool(self._cl[self._i]), bool(self._cr[self._i])


def build_zproxy_timeline(bag_dir, t_abs_lo, t_abs_hi, kinematics, z_margin=0.02):
    """First pass: compute stance_l / stance_r from FK foot-z at every
    /joint_states message (stance ⇔ z within `z_margin` of the per-run 2-percentile)."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    latest_joints = {}
    ts_list, zl_list, zr_list = [], [], []
    while r.has_next():
        topic, data, ts = r.read_next()
        t_abs = ts * 1e-9
        if t_abs_hi is not None and t_abs > t_abs_hi:
            break
        if topic != '/joint_states':
            continue
        msg = rclpy.serialization.deserialize_message(data, JointState)
        for i, name in enumerate(msg.name):
            urdf = JOINT_MAPPING.get(name)
            if urdf and len(msg.position) > i:
                latest_joints[urdf] = msg.position[i]
        if len(latest_joints) < len(JOINT_MAPPING):
            continue
        p_l = kinematics.fk_left(latest_joints)
        p_r = kinematics.fk_right(latest_joints)
        ts_list.append(t_abs); zl_list.append(p_l[2]); zr_list.append(p_r[2])

    t  = np.array(ts_list)
    zl = np.array(zl_list); zr = np.array(zr_list)

    # only use samples in the motion window to fit the floor, but provide
    # stance decisions for the full timeline.
    if t_abs_lo is not None:
        motion = t >= t_abs_lo
    else:
        motion = np.ones_like(t, bool)
    zl_floor = np.percentile(zl[motion], 2.0)
    zr_floor = np.percentile(zr[motion], 2.0)
    stance_l = zl < zl_floor + z_margin
    stance_r = zr < zr_floor + z_margin
    print(f'[zproxy] floors  L={zl_floor:+.3f}m  R={zr_floor:+.3f}m  '
          f'margin={z_margin}m   stance L/R/both/neither='
          f'{stance_l.mean()*100:.1f}/{stance_r.mean()*100:.1f}/'
          f'{(stance_l&stance_r).mean()*100:.1f}/{(~stance_l & ~stance_r).mean()*100:.1f}%')
    return t, stance_l, stance_r


def build_detector(mode, cfg, bag_dir=None, t_abs_lo=None, t_abs_hi=None, kinematics=None):
    cc = cfg.get('contact', {})
    hyst = cc.get('hysteresis', 1.0)
    if mode == 'raw':
        # match current Python-pipeline default (symmetric threshold)
        return ContactDetector(
            threshold=cc.get('effort_threshold', 5.0),
            hysteresis=hyst)
    if mode == 'detrend':
        return BiasCompensatedContactDetector(
            threshold=5.0, hysteresis=hyst, window_size=1000)
    if mode == 'zproxy':
        t_arr, cl, cr = build_zproxy_timeline(bag_dir, t_abs_lo, t_abs_hi, kinematics)
        return PrecomputedContactDetector(t_arr, cl, cr)
    raise ValueError(mode)


def run_ekf(bag_dir, contact_mode, t_abs_lo, t_abs_hi, cfg, kinematics):
    ekf = BloeSchEKF(cfg.get('ekf', {}))
    det = build_detector(contact_mode, cfg,
                         bag_dir=bag_dir, t_abs_lo=t_abs_lo, t_abs_hi=t_abs_hi,
                         kinematics=kinematics)

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    latest_joints = {}
    latest_efforts = {}
    last_imu_ts = None
    init_count = 0
    init_accel_sum = np.zeros(3)
    INIT_FRAMES = 50

    est_t, est_pos, est_ori = [], [], []
    contact_log = []  # (t, cl, cr)

    while r.has_next():
        topic, data, ts = r.read_next()
        t_abs = ts * 1e-9
        if t_abs_hi is not None and t_abs > t_abs_hi:
            break

        if topic == '/joint_states':
            msg = rclpy.serialization.deserialize_message(data, JointState)
            for i, name in enumerate(msg.name):
                if len(msg.effort) > i:
                    latest_efforts[name] = msg.effort[i]
                urdf_name = JOINT_MAPPING.get(name)
                if urdf_name and len(msg.position) > i:
                    latest_joints[urdf_name] = msg.position[i]
            continue

        if topic == '/imu':
            msg = rclpy.serialization.deserialize_message(data, Imu)
            accel = np.array([msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z])
            gyro  = np.array([msg.angular_velocity.x,
                              msg.angular_velocity.y,
                              msg.angular_velocity.z])

            if not ekf.initialized:
                init_accel_sum += accel
                init_count += 1
                if init_count >= INIT_FRAMES and latest_joints:
                    p_fl = kinematics.fk_left(latest_joints)
                    p_fr = kinematics.fk_right(latest_joints)
                    ekf.initialize(init_accel_sum / init_count, p_fl, p_fr)
                continue

            if last_imu_ts is None:
                last_imu_ts = t_abs; continue
            dt = t_abs - last_imu_ts
            last_imu_ts = t_abs
            if dt <= 0 or dt > 0.1:
                continue

            eff_l = latest_efforts.get('LJPITCH', 0.0)
            eff_r = latest_efforts.get('RJPITCH', 0.0)
            if isinstance(det, PrecomputedContactDetector):
                det.set_time(t_abs)
            cl, cr = det.update(eff_l, eff_r)

            ekf.predict(accel, gyro, dt, cl, cr)
            if latest_joints:
                p_fl = kinematics.fk_left(latest_joints)
                p_fr = kinematics.fk_right(latest_joints)
                ekf.update(p_fl, p_fr, cl, cr)

            pos, quat = ekf.get_pose()
            est_t.append(t_abs)
            est_pos.append(pos.copy())
            est_ori.append(quat.copy())
            contact_log.append((t_abs, int(cl), int(cr)))
            # stash velocity for diagnostic
            if not hasattr(run_ekf, '_vel_log'): run_ekf._vel_log = {}
            run_ekf._vel_log.setdefault(contact_mode, []).append(
                (t_abs, *ekf.get_velocity()))

    return (np.array(est_t), np.array(est_pos), np.array(est_ori),
            np.array(contact_log))


def load_tum_traj(path, t_abs_lo=None, t_abs_hi=None):
    data = np.loadtxt(path)
    mask = np.ones(len(data), dtype=bool)
    if t_abs_lo is not None: mask &= data[:, 0] >= t_abs_lo
    if t_abs_hi is not None: mask &= data[:, 0] <= t_abs_hi
    return data[mask]


def shape_metrics(t_est, pos_est, traj_ref, t_motion_start):
    """trim to t >= motion_start, compute path len / loop err / nearest-point XY err."""
    mask = t_est >= t_motion_start
    te = t_est[mask]
    pe = pos_est[mask] - pos_est[mask][0]  # re-anchor to origin

    tr = traj_ref[:, 0]
    pr = traj_ref[:, 1:4] - traj_ref[0, 1:4]

    est_path = float(np.sum(np.linalg.norm(np.diff(pe[:, :2], axis=0), axis=1)))
    ref_path = float(np.sum(np.linalg.norm(np.diff(pr[:, :2], axis=0), axis=1)))
    est_loop = float(np.linalg.norm(pe[-1, :2] - pe[0, :2]))
    ref_loop = float(np.linalg.norm(pr[-1, :2] - pr[0, :2]))

    # interp ref at est timestamps then time-sync XY distance
    rx = np.interp(te, tr, pr[:, 0])
    ry = np.interp(te, tr, pr[:, 1])
    d_xy = np.hypot(pe[:, 0] - rx, pe[:, 1] - ry)

    return {
        'est_path_m':  est_path,
        'ref_path_m':  ref_path,
        'path_ratio':  est_path / max(ref_path, 1e-6),
        'est_loop_m':  est_loop,
        'ref_loop_m':  ref_loop,
        'mean_xy_err_m': float(np.mean(d_xy)),
        'max_xy_err_m':  float(np.max(d_xy)),
        'final_xy_err_m': float(d_xy[-1]),
    }, pe, te


def plot_compare(results, ref_traj, t_motion_start, out_path):
    """results: dict mode -> (t, pos, ori, contacts, metrics, pe_trimmed, te_trimmed)"""
    modes = list(results.keys())
    colors = {'raw': 'C3', 'detrend': 'C2'}

    fig = plt.figure(figsize=(20, 11))
    gs = fig.add_gridspec(3, 3, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)

    # === (0, :) XY overlay ===
    ax = fig.add_subplot(gs[0, :2])
    pr = ref_traj[:, 1:4] - ref_traj[0, 1:4]
    ax.plot(pr[:, 0], pr[:, 1], lw=2.0, color='k',
            label=f'traj_imu.txt  (len={np.sum(np.linalg.norm(np.diff(pr[:, :2], axis=0), axis=1)):.1f}m)')
    ax.scatter([pr[0,0]], [pr[0,1]], c='g', s=80, zorder=5, label='start')
    ax.scatter([pr[-1,0]], [pr[-1,1]], c='r', s=80, marker='x', zorder=5, label='ref end')
    for mode in modes:
        _, _, _, _, m, pe, _ = results[mode]
        ax.plot(pe[:, 0], pe[:, 1], lw=1.4, color=colors.get(mode, 'C0'), alpha=0.9,
                label=f'EKF [{mode}]  len={m["est_path_m"]:.1f}m  loop_err={m["est_loop_m"]:.2f}m')
    ax.set_title('XY trajectory — estimated vs traj_imu.txt (both anchored to origin)', fontsize=13)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=10)

    # === (0, 2) Z over time ===
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(ref_traj[:, 0] - ref_traj[0, 0], pr[:, 2], lw=2, color='k', label='ref z')
    for mode in modes:
        _, _, _, _, _, pe, te = results[mode]
        ax.plot(te - te[0], pe[:, 2], lw=1.2, color=colors.get(mode, 'C0'),
                label=f'{mode}')
    ax.set_title('z(t)'); ax.set_xlabel('t [s]'); ax.set_ylabel('z [m]')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    # === (1, :) x(t), y(t), xy_err ===
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(ref_traj[:, 0] - ref_traj[0, 0], pr[:, 0], lw=2, color='k', label='ref x')
    for mode in modes:
        _, _, _, _, _, pe, te = results[mode]
        ax.plot(te - te[0], pe[:, 0], lw=1.0, color=colors.get(mode, 'C0'), label=mode)
    ax.set_title('x(t)'); ax.set_xlabel('t [s]'); ax.grid(alpha=0.3); ax.legend(fontsize=9)

    ax = fig.add_subplot(gs[1, 1])
    ax.plot(ref_traj[:, 0] - ref_traj[0, 0], pr[:, 1], lw=2, color='k', label='ref y')
    for mode in modes:
        _, _, _, _, _, pe, te = results[mode]
        ax.plot(te - te[0], pe[:, 1], lw=1.0, color=colors.get(mode, 'C0'), label=mode)
    ax.set_title('y(t)'); ax.set_xlabel('t [s]'); ax.grid(alpha=0.3); ax.legend(fontsize=9)

    ax = fig.add_subplot(gs[1, 2])
    tref = ref_traj[:, 0]
    prxy = pr[:, :2]
    for mode in modes:
        _, _, _, _, _, pe, te = results[mode]
        rx = np.interp(te, tref, prxy[:, 0])
        ry = np.interp(te, tref, prxy[:, 1])
        d = np.hypot(pe[:, 0] - rx, pe[:, 1] - ry)
        ax.plot(te - te[0], d, lw=1.0, color=colors.get(mode, 'C0'), label=mode)
    ax.set_title('‖xy_est − xy_ref‖ over time'); ax.set_xlabel('t [s]'); ax.set_ylabel('m')
    ax.grid(alpha=0.3); ax.legend(fontsize=9)

    # === (2, :) contact state per mode ===
    for col, mode in enumerate(modes):
        ax = fig.add_subplot(gs[2, col])
        _, _, _, contacts, _, _, _ = results[mode]
        if len(contacts):
            t = contacts[:, 0] - contacts[0, 0]
            ax.fill_between(t, 0.55, 0.55 + contacts[:, 1] * 0.4, step='post',
                            alpha=0.55, color='C0', label='L stance')
            ax.fill_between(t, 0.05, 0.05 + contacts[:, 2] * 0.4, step='post',
                            alpha=0.55, color='C1', label='R stance')
            ax.set_ylim(0, 1.05)
            ax.set_yticks([0.25, 0.75]); ax.set_yticklabels(['R', 'L'])
            l_ratio = contacts[:, 1].mean() * 100
            r_ratio = contacts[:, 2].mean() * 100
            both_ratio = (contacts[:, 1].astype(bool) &
                          contacts[:, 2].astype(bool)).mean() * 100
            ax.set_title(f'contact [{mode}]   L={l_ratio:.1f}%   R={r_ratio:.1f}%   both={both_ratio:.1f}%')
        ax.set_xlabel('t [s]'); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.suptitle('leg_odometry offline replay — raw vs bias-compensated contact detection',
                 fontsize=14, y=0.995)
    fig.savefig(out_path, dpi=110, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('--contact',
                    choices=['raw', 'detrend', 'zproxy', 'both', 'all'],
                    default='both',
                    help='which contact detector(s) to run')
    ap.add_argument('--t-abs-lo', type=float, default=None,
                    help='EKF init begins from bag start; lo used for trimming ref only')
    ap.add_argument('--t-abs-hi', type=float, default=None)
    ap.add_argument('--out', default=None)
    ap.add_argument('--traj', default=None, help='TUM trajectory file')
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    bag_name = os.path.basename(bag_dir.rstrip('/'))
    out_dir = args.out or os.path.join(
        os.path.dirname(bag_dir), f'eval_{bag_name}')
    os.makedirs(out_dir, exist_ok=True)
    print(f'bag: {bag_dir}\nout: {out_dir}')

    with open(CONFIG_DIR / 'ekf_params.yaml') as f:
        cfg = yaml.safe_load(f)
    with open(URDF_PATH) as f:
        kinematics = LegKinematics(
            f.read(), base_link='base_link',
            left_foot_link='left_leg_ankle_roll_link',
            right_foot_link='right_leg_ankle_roll_link')

    traj_path = args.traj or os.path.join(bag_dir, 'traj_imu.txt')
    ref = load_tum_traj(traj_path, args.t_abs_lo, args.t_abs_hi)
    print(f'ref traj: {traj_path}  N={len(ref)}')

    t_motion_start = args.t_abs_lo if args.t_abs_lo else ref[0, 0]

    if args.contact == 'both':
        modes = ['raw', 'detrend']
    elif args.contact == 'all':
        modes = ['raw', 'detrend', 'zproxy']
    else:
        modes = [args.contact]
    results = {}
    for mode in modes:
        print(f'\n=== running EKF [{mode}] ===')
        t, pos, ori, contacts = run_ekf(
            bag_dir, mode, args.t_abs_lo, args.t_abs_hi, cfg, kinematics)
        print(f'  {len(t)} samples')
        m, pe, te = shape_metrics(t, pos, ref, t_motion_start)
        print(f'  est_path={m["est_path_m"]:.2f} m   ref_path={m["ref_path_m"]:.2f} m   '
              f'ratio={m["path_ratio"]:.2f}')
        print(f'  est_loop_err={m["est_loop_m"]:.2f} m   ref_loop_err={m["ref_loop_m"]:.2f} m')
        print(f'  mean_xy_err={m["mean_xy_err_m"]:.2f} m   max_xy_err={m["max_xy_err_m"]:.2f} m   '
              f'final_xy_err={m["final_xy_err_m"]:.2f} m')
        results[mode] = (t, pos, ori, contacts, m, pe, te)

    out_png = os.path.join(out_dir, f'compare_{"_vs_".join(modes)}.png')
    plot_compare(results, ref, t_motion_start, out_png)

    # === body velocity diagnostic ===
    if hasattr(run_ekf, '_vel_log'):
        fig_v, ax_v = plt.subplots(1, 1, figsize=(18, 5))
        for mode in modes:
            arr = np.array(run_ekf._vel_log.get(mode, []))
            if len(arr) == 0: continue
            tm = arr[:, 0] - arr[0, 0]
            sp = np.linalg.norm(arr[:, 1:4], axis=1)
            ax_v.plot(tm, sp, lw=0.6,
                      color='C3' if mode == 'raw' else ('C2' if mode == 'detrend' else 'C0'),
                      label=f'{mode}  mean={sp.mean():.2f} m/s  '
                            f'median={np.median(sp):.2f}  max={sp.max():.2f}')
        # ref speed line
        rt = ref[:, 0] - ref[0, 0]
        rp = ref[:, 1:3]
        inst = np.linalg.norm(np.diff(rp, axis=0), axis=1) / np.diff(ref[:, 0])
        inst_t = (ref[1:, 0] - ref[0, 0])
        ax_v.plot(inst_t, inst, lw=1.0, color='k', alpha=0.7,
                  label=f'ref GLIM  mean={inst.mean():.2f} m/s  '
                        f'median={np.median(inst):.2f}  max={inst.max():.2f}')
        ax_v.set_xlabel('t [s]'); ax_v.set_ylabel('|v_body| (world) [m/s]')
        ax_v.grid(alpha=0.3); ax_v.legend(loc='upper right', fontsize=9)
        ax_v.set_title('EKF estimated body speed vs reference body speed')
        out_v = os.path.join(out_dir, 'body_velocity.png')
        fig_v.savefig(out_v, dpi=120, bbox_inches='tight'); plt.close(fig_v)
        print(f'wrote {out_v}')

        # integrated path from EKF |v| to confirm it matches est_path
        for mode in modes:
            arr = np.array(run_ekf._vel_log.get(mode, []))
            if len(arr) == 0: continue
            dt = np.diff(arr[:, 0], prepend=arr[0, 0])
            dt[0] = dt[1]
            sp = np.linalg.norm(arr[:, 1:4], axis=1)
            print(f'[{mode}] Σ|v|·dt = {(sp*dt).sum():.2f} m (EKF velocity integrated)')

    # dedicated clean XY-only overlays, one per mode
    pr = ref[:, 1:4] - ref[0, 1:4]
    ref_path_len = float(np.sum(np.linalg.norm(np.diff(pr[:, :2], axis=0), axis=1)))
    ref_loop = float(np.linalg.norm(pr[-1, :2] - pr[0, :2]))
    for mode in modes:
        _, _, _, _, m, pe, _ = results[mode]
        fig, ax = plt.subplots(1, 1, figsize=(10, 9))
        ax.plot(pr[:, 0], pr[:, 1], lw=2.5, color='k',
                label=f'traj_imu.txt (reference)  len={ref_path_len:.1f}m  loop_err={ref_loop:.2f}m')
        ax.plot(pe[:, 0], pe[:, 1], lw=1.6, color='C3' if mode == 'raw' else 'C2',
                alpha=0.9,
                label=(f'leg_odometry EKF [{mode}]  len={m["est_path_m"]:.1f}m  '
                       f'loop_err={m["est_loop_m"]:.2f}m'))
        ax.scatter([pr[0, 0]], [pr[0, 1]], c='g', s=90, zorder=5, label='start')
        ax.scatter([pr[-1, 0]], [pr[-1, 1]], c='k', s=90, marker='x', zorder=5, label='ref end')
        ax.scatter([pe[-1, 0]], [pe[-1, 1]], c='r', s=90, marker='x', zorder=5, label='est end')
        tag = 'current (raw)' if mode == 'raw' else 'bias-compensated'
        ax.set_title(f'Current leg_odometry XY trajectory ({tag}) vs traj_imu.txt\n'
                     f'path_ratio={m["path_ratio"]:.2f}×   '
                     f'final_xy_err={m["final_xy_err_m"]:.2f}m   '
                     f'mean_xy_err={m["mean_xy_err_m"]:.2f}m',
                     fontsize=12)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.legend(fontsize=10, loc='best')
        p = os.path.join(out_dir, f'xy_{mode}_vs_ref.png')
        fig.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
        print(f'wrote {p}')

    # metrics summary text
    summary_path = os.path.join(out_dir, 'metrics.txt')
    with open(summary_path, 'w') as f:
        for mode in modes:
            m = results[mode][4]
            f.write(f'[{mode}]\n')
            for k, v in m.items():
                f.write(f'  {k}: {v:.4f}\n' if isinstance(v, float) else f'  {k}: {v}\n')
            f.write('\n')
    print(f'\nwrote {summary_path}')


if __name__ == '__main__':
    main()
