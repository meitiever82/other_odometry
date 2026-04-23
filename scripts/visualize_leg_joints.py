#!/usr/bin/env python3
"""Plot position / velocity / effort of the 12 leg joints leg_odometry uses.

Usage:
    python3 visualize_leg_joints.py <bag_dir> [--out DIR]
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState


LEG_JOINTS = {
    'left':  ['LJ0', 'LJ1', 'LJ2', 'LJ3', 'LJPITCH', 'LJROLL'],
    'right': ['RJ6', 'RJ7', 'RJ8', 'RJ9', 'RJPITCH', 'RJROLL'],
}
JOINT_LABEL = {
    'LJ0': 'L pelvic pitch',   'RJ6': 'R pelvic pitch',
    'LJ1': 'L pelvic roll',    'RJ7': 'R pelvic roll',
    'LJ2': 'L pelvic yaw',     'RJ8': 'R pelvic yaw',
    'LJ3': 'L knee pitch',     'RJ9': 'R knee pitch',
    'LJPITCH': 'L ankle pitch','RJPITCH': 'R ankle pitch',
    'LJROLL':  'L ankle roll', 'RJROLL':  'R ankle roll',
}
ALL_JOINTS = LEG_JOINTS['left'] + LEG_JOINTS['right']

# contact detection config (from config/ekf_params.yaml)
THRESH_L, THRESH_R = 5.0, 12.0
HYST = 1.0


def extract(bag_dir: str, t_abs_lo=None, t_abs_hi=None):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    t_list = []
    pos = {j: [] for j in ALL_JOINTS}
    vel = {j: [] for j in ALL_JOINTS}
    eff = {j: [] for j in ALL_JOINTS}

    idx_cache = None
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic != '/joint_states':
            continue
        t_abs = ts * 1e-9
        if t_abs_lo is not None and t_abs < t_abs_lo:
            continue
        if t_abs_hi is not None and t_abs > t_abs_hi:
            continue
        m = deserialize_message(data, JointState)
        if idx_cache is None:
            name2i = {n: i for i, n in enumerate(m.name)}
            idx_cache = {j: name2i.get(j) for j in ALL_JOINTS}
            missing = [j for j, i in idx_cache.items() if i is None]
            if missing:
                print(f'[WARN] missing joints in bag: {missing}', file=sys.stderr)

        t_list.append(t_abs)
        for j, i in idx_cache.items():
            if i is None:
                pos[j].append(np.nan); vel[j].append(np.nan); eff[j].append(np.nan)
                continue
            pos[j].append(m.position[i] if len(m.position) > i else np.nan)
            vel[j].append(m.velocity[i] if len(m.velocity) > i else np.nan)
            eff[j].append(m.effort[i]   if len(m.effort)   > i else np.nan)

    t = np.array(t_list)
    t_abs0 = t[0]
    t = t - t_abs0
    return (t, t_abs0,
            {j: np.array(v) for j, v in pos.items()},
            {j: np.array(v) for j, v in vel.items()},
            {j: np.array(v) for j, v in eff.items()})


def load_tum_traj(path, t_abs_lo=None, t_abs_hi=None):
    if not os.path.isfile(path):
        return None
    data = np.loadtxt(path)
    mask = np.ones(len(data), dtype=bool)
    if t_abs_lo is not None: mask &= data[:, 0] >= t_abs_lo
    if t_abs_hi is not None: mask &= data[:, 0] <= t_abs_hi
    return data[mask]


def plot_traj(traj, t_abs0, out_path):
    if traj is None or len(traj) == 0:
        print('  traj_imu.txt empty or not found, skip trajectory plot'); return
    t = traj[:, 0] - t_abs0
    x, y, z = traj[:, 1], traj[:, 2], traj[:, 3]
    path_len = float(np.sum(np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1)))
    loop_err = float(np.linalg.norm(traj[-1, 1:4] - traj[0, 1:4]))

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    ax = axes[0]
    ax.plot(x, y, lw=1.0)
    ax.scatter([x[0]], [y[0]], c='g', s=60, zorder=5, label='start')
    ax.scatter([x[-1]], [y[-1]], c='r', s=60, marker='x', zorder=5, label='end')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f'XY path   len={path_len:.2f} m   loop_err={loop_err:.3f} m')
    ax.legend()

    axes[1].plot(t, x, label='x'); axes[1].plot(t, y, label='y')
    axes[1].set_xlabel('t [s]'); axes[1].set_ylabel('m'); axes[1].grid(alpha=0.3)
    axes[1].set_title('x,y vs time'); axes[1].legend()

    axes[2].plot(t, z, color='C2')
    axes[2].set_xlabel('t [s]'); axes[2].set_ylabel('m'); axes[2].grid(alpha=0.3)
    axes[2].set_title(f'z vs time   range={z.max()-z.min():.3f} m')

    fig.suptitle(f'traj_imu.txt  (N={len(traj)})', fontsize=13)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f'  wrote {out_path}')


def plot_field(t, data, unit, title, out_path, contact_overlay=False):
    fig, axes = plt.subplots(2, 6, figsize=(22, 7), sharex=True)
    fig.suptitle(f'{title}   (bag t=0..{t[-1]:.1f}s, N={len(t)})', fontsize=14)

    for row, side in enumerate(['left', 'right']):
        for col, j in enumerate(LEG_JOINTS[side]):
            ax = axes[row, col]
            ax.plot(t, data[j], lw=0.6)
            ax.set_title(f'{j}  ({JOINT_LABEL[j]})', fontsize=9)
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=8)
            if col == 0:
                ax.set_ylabel(unit, fontsize=9)
            if row == 1:
                ax.set_xlabel('t [s]', fontsize=9)

            # stats annotation
            v = data[j][~np.isnan(data[j])]
            if len(v):
                txt = f'min={v.min():.2f}\nmax={v.max():.2f}\nmean={v.mean():.2f}\nstd={v.std():.2f}'
                ax.text(0.02, 0.97, txt, transform=ax.transAxes, fontsize=7,
                        va='top', family='monospace',
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))

            if contact_overlay and j == 'LJPITCH':
                ax.axhline(+THRESH_L, color='r', ls='--', lw=0.7, label=f'±{THRESH_L}')
                ax.axhline(-THRESH_L, color='r', ls='--', lw=0.7)
                ax.legend(fontsize=7, loc='lower right')
            if contact_overlay and j == 'RJPITCH':
                ax.axhline(+THRESH_R, color='r', ls='--', lw=0.7, label=f'±{THRESH_R}')
                ax.axhline(-THRESH_R, color='r', ls='--', lw=0.7)
                ax.legend(fontsize=7, loc='lower right')

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f'  wrote {out_path}')


def plot_contact_diagnosis(t, eff, out_path):
    """zoom-in on the contact detection signal used by leg_odometry."""
    fig, axes = plt.subplots(2, 1, figsize=(18, 7), sharex=True)

    def draw(ax, side, j, thr):
        e = eff[j]
        ax.plot(t, e, lw=0.6, color='C0', label=f'{j} effort [Nm]')
        ax.axhline(+thr,         color='r',  ls='--', lw=0.8, label=f'±threshold ({thr})')
        ax.axhline(-thr,         color='r',  ls='--', lw=0.8)
        ax.axhline(+thr + HYST,  color='orange', ls=':', lw=0.6, label=f'±(thr+hyst)')
        ax.axhline(-thr - HYST,  color='orange', ls=':', lw=0.6)
        # crude contact: |effort| > thr
        contact = (np.abs(e) > thr).astype(float)
        ax2 = ax.twinx()
        ax2.fill_between(t, 0, contact, step='post', alpha=0.15, color='g')
        ax2.set_ylim(0, 1.05)
        ax2.set_yticks([0, 1])
        ax2.set_yticklabels(['swing', 'stance'], fontsize=8)
        ax.set_title(f'{side} foot contact signal  (stance ratio = '
                     f'{contact[~np.isnan(e)].mean()*100:.1f}%)', fontsize=11)
        ax.set_ylabel('effort [Nm]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')

    draw(axes[0], 'Left',  'LJPITCH', THRESH_L)
    draw(axes[1], 'Right', 'RJPITCH', THRESH_R)
    axes[1].set_xlabel('t [s]')

    fig.suptitle('Contact detection input (|effort| > threshold ⇒ stance)', fontsize=13)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    print(f'  wrote {out_path}')


def save_csv(t, pos, vel, eff, out_path):
    header = ['t']
    for j in ALL_JOINTS: header += [f'{j}_pos']
    for j in ALL_JOINTS: header += [f'{j}_vel']
    for j in ALL_JOINTS: header += [f'{j}_eff']
    cols = [t]
    for j in ALL_JOINTS: cols.append(pos[j])
    for j in ALL_JOINTS: cols.append(vel[j])
    for j in ALL_JOINTS: cols.append(eff[j])
    arr = np.column_stack(cols)
    np.savetxt(out_path, arr, delimiter=',', header=','.join(header), comments='')
    print(f'  wrote {out_path}  shape={arr.shape}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('--out', default=None, help='output dir (default: <bag>/../viz_<bagname>[_clip])')
    ap.add_argument('--t-abs-lo', type=float, default=None,
                    help='absolute epoch lower bound (ignore samples before)')
    ap.add_argument('--t-abs-hi', type=float, default=None,
                    help='absolute epoch upper bound (ignore samples after)')
    ap.add_argument('--traj', default=None,
                    help='TUM trajectory file (default: <bag>/traj_imu.txt if exists)')
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    bag_name = os.path.basename(bag_dir.rstrip('/'))
    if args.out is None:
        suffix = '_clip' if (args.t_abs_lo or args.t_abs_hi) else ''
        out_dir = os.path.join(os.path.dirname(bag_dir), f'viz_{bag_name}{suffix}')
    else:
        out_dir = args.out
    os.makedirs(out_dir, exist_ok=True)
    print(f'bag: {bag_dir}')
    print(f'out: {out_dir}')
    if args.t_abs_lo or args.t_abs_hi:
        print(f'clip: [{args.t_abs_lo}, {args.t_abs_hi}]')

    print('reading bag ...')
    t, t_abs0, pos, vel, eff = extract(bag_dir, args.t_abs_lo, args.t_abs_hi)
    print(f'  got {len(t)} /joint_states samples, rate = {len(t)/(t[-1]-t[0]):.1f} Hz')

    traj_path = args.traj or os.path.join(bag_dir, 'traj_imu.txt')
    traj = load_tum_traj(traj_path, args.t_abs_lo, args.t_abs_hi)

    print('plotting ...')
    plot_field(t, pos, 'rad',    'Joint position',  os.path.join(out_dir, '01_position.png'))
    plot_field(t, vel, 'rad/s',  'Joint velocity',  os.path.join(out_dir, '02_velocity.png'))
    plot_field(t, eff, 'Nm',     'Joint effort',    os.path.join(out_dir, '03_effort.png'),
               contact_overlay=True)
    plot_contact_diagnosis(t, eff, os.path.join(out_dir, '04_contact_detection.png'))
    plot_traj(traj, t_abs0, os.path.join(out_dir, '05_trajectory.png'))
    save_csv(t, pos, vel, eff, os.path.join(out_dir, 'leg_joints.csv'))

    print('done.')


if __name__ == '__main__':
    main()
