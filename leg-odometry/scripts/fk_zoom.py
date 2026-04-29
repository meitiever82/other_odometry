#!/usr/bin/env python3
"""Zoomed FK foot-in-body trace over a short window, with stance shading."""
import argparse, os, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy.serialization
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from leg_odometry.kinematics import LegKinematics

PKG_DIR = Path(__file__).resolve().parent.parent
URDF_PATH = PKG_DIR.parent / 'finder_lidar_mapping' / 'glim_ros2' / 'urdf' / 'casbot02_7dof_shell.urdf'
JOINT_MAPPING = {
    'LJ0': 'left_leg_pelvic_pitch_joint', 'LJ1': 'left_leg_pelvic_roll_joint',
    'LJ2': 'left_leg_pelvic_yaw_joint',   'LJ3': 'left_leg_knee_pitch_joint',
    'LJPITCH': 'left_leg_ankle_pitch_joint', 'LJROLL': 'left_leg_ankle_roll_joint',
    'RJ6': 'right_leg_pelvic_pitch_joint', 'RJ7': 'right_leg_pelvic_roll_joint',
    'RJ8': 'right_leg_pelvic_yaw_joint',   'RJ9': 'right_leg_knee_pitch_joint',
    'RJPITCH': 'right_leg_ankle_pitch_joint', 'RJROLL': 'right_leg_ankle_roll_joint',
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t-center', type=float, default=90.0,
                    help='bag-relative seconds to center zoom')
    ap.add_argument('--window', type=float, default=6.0)
    ap.add_argument('--t-abs-lo', type=float, default=None)
    ap.add_argument('--t-abs-hi', type=float, default=None)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    out_dir = args.out or os.path.join(os.path.dirname(bag_dir),
                                       f'fkzoom_{os.path.basename(bag_dir)}')
    os.makedirs(out_dir, exist_ok=True)

    with open(URDF_PATH) as f:
        kin = LegKinematics(f.read(), base_link='base_link',
                            left_foot_link='left_leg_ankle_roll_link',
                            right_foot_link='right_leg_ankle_roll_link')

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))
    latest_joints = {}
    tl, FL, FR = [], [], []
    raw_joint_ts = []
    raw_joint_values = {k: [] for k in ['LJ0','LJ3','LJPITCH','RJ6','RJ9','RJPITCH']}
    while r.has_next():
        topic, data, ts = r.read_next()
        t_abs = ts * 1e-9
        if args.t_abs_lo and t_abs < args.t_abs_lo: continue
        if args.t_abs_hi and t_abs > args.t_abs_hi: break
        if topic != '/joint_states': continue
        msg = rclpy.serialization.deserialize_message(data, JointState)
        n2i = {n: i for i, n in enumerate(msg.name)}
        for i, name in enumerate(msg.name):
            urdf = JOINT_MAPPING.get(name)
            if urdf and len(msg.position) > i:
                latest_joints[urdf] = msg.position[i]
        if len(latest_joints) < len(JOINT_MAPPING): continue
        tl.append(t_abs); FL.append(kin.fk_left(latest_joints)); FR.append(kin.fk_right(latest_joints))
        raw_joint_ts.append(t_abs)
        for key in raw_joint_values:
            idx = n2i.get(key)
            raw_joint_values[key].append(msg.position[idx] if idx is not None else np.nan)
    t = np.array(tl); t -= t[0]
    FL = np.array(FL); FR = np.array(FR)

    # z-proxy stance
    zl_floor = np.percentile(FL[:, 2], 2.0); zr_floor = np.percentile(FR[:, 2], 2.0)
    CL = FL[:, 2] < zl_floor + 0.05
    CR = FR[:, 2] < zr_floor + 0.05

    # zoom window
    t0 = args.t_center - args.window/2
    t1 = args.t_center + args.window/2
    m = (t >= t0) & (t <= t1)
    print(f'zoom window [{t0:.1f}, {t1:.1f}] s  ({m.sum()} samples)')

    # load ref trajectory (traj_imu.txt) to overlay actual body motion
    ref_path = os.path.join(bag_dir, 'traj_imu.txt')
    ref_motion = None
    if os.path.isfile(ref_path):
        ref = np.loadtxt(ref_path)
        t_abs0 = (args.t_abs_lo if args.t_abs_lo else ref[0, 0])
        ref_motion = (ref[:, 0] - t_abs0, ref[:, 1:4] - ref[0, 1:4])
        # speed within zoom window
        m_ref = (ref_motion[0] >= t0) & (ref_motion[0] <= t1)
        if m_ref.sum() > 1:
            p_win = ref_motion[1][m_ref]
            seg_len = float(np.sum(np.linalg.norm(np.diff(p_win[:, :2], axis=0), axis=1)))
            net = float(np.linalg.norm(p_win[-1, :2] - p_win[0, :2]))
            print(f'[ref] within window: path_len={seg_len:.3f}m  net_xy={net:.3f}m  '
                  f'avg_speed={seg_len/(t1-t0):.3f} m/s')

    fig, axes = plt.subplots(3, 2, figsize=(16, 9), sharex=True)
    for col, (name, FK, C, color) in enumerate([
        ('LEFT ', FL, CL, 'C0'),
        ('RIGHT', FR, CR, 'C3'),
    ]):
        for row, lab in enumerate(['x [m]  (+forward)',
                                    'y [m]  (+left)',
                                    'z [m]  (≈ -height)']):
            ax = axes[row, col]
            ax.plot(t[m], FK[m, row], lw=1.2, color=color)
            # shade stance
            stance_mask = C[m].astype(float)
            ax2 = ax.twinx()
            ax2.fill_between(t[m], 0, stance_mask, step='post', alpha=0.15, color='g')
            ax2.set_ylim(0, 1.05); ax2.set_yticks([])
            ax.set_title(f'{name}   {lab}', fontsize=10)
            ax.grid(alpha=0.3)
            if row == 2: ax.set_xlabel('t [s]')

    # add ref body xy trace as a separate summary plot
    if ref_motion is not None:
        fig2, ax = plt.subplots(1, 1, figsize=(14, 4))
        rt, rp = ref_motion
        mr = (rt >= t0) & (rt <= t1)
        ax.plot(rt[mr], rp[mr, 0] - rp[mr, 0][0], lw=2.0, color='k', label='ref body x (re-anchored)')
        ax.plot(rt[mr], rp[mr, 1] - rp[mr, 1][0], lw=2.0, color='b', label='ref body y (re-anchored)')
        ax.set_xlabel('t [s]'); ax.set_ylabel('m'); ax.grid(alpha=0.3); ax.legend()
        ax.set_title(f'Reference (GLIM) body motion within the same zoom window  '
                     f'[{t0:.1f},{t1:.1f}]s')
        p2 = os.path.join(out_dir, f'ref_body_{int(t0)}_{int(t1)}.png')
        fig2.savefig(p2, dpi=120, bbox_inches='tight'); plt.close(fig2)
        print(f'wrote {p2}')

    fig.suptitle(f'FK zoom  [{t0:.1f}, {t1:.1f}] s   green = stance (z-proxy 5cm)', fontsize=13)
    fig.tight_layout()
    p = os.path.join(out_dir, f'fk_zoom_{int(t0)}_{int(t1)}.png')
    fig.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
    print(f'wrote {p}')

    # joint angles within zoom window
    rjt = np.array(raw_joint_ts); rjt -= rjt[0] if len(tl) == 0 else (tl[0] - (args.t_abs_lo or 0))
    rjt = np.array(raw_joint_ts) - np.array(raw_joint_ts)[0]
    fig3, axes3 = plt.subplots(3, 2, figsize=(16, 9), sharex=True)
    for col, side in enumerate(['LEFT (LJ0/LJ3/LJPITCH)', 'RIGHT (RJ6/RJ9/RJPITCH)']):
        keys = ['LJ0', 'LJ3', 'LJPITCH'] if col == 0 else ['RJ6', 'RJ9', 'RJPITCH']
        labs = ['hip pitch', 'knee pitch', 'ankle pitch']
        for row, (k, lab) in enumerate(zip(keys, labs)):
            ax = axes3[row, col]
            vals = np.array(raw_joint_values[k])
            wmask = (rjt >= t0) & (rjt <= t1)
            ax.plot(rjt[wmask], vals[wmask], lw=1.2, color='C0' if col == 0 else 'C3')
            v = vals[wmask]; v = v[~np.isnan(v)]
            if len(v):
                ax.set_title(f'{k}  ({lab})   range={v.max()-v.min():.3f} rad  '
                             f'({np.degrees(v.max()-v.min()):.1f}°)  '
                             f'peak-to-peak', fontsize=10)
            ax.grid(alpha=0.3)
            ax.set_ylabel('rad')
            if row == 2: ax.set_xlabel('t [s]')
    fig3.suptitle(f'Joint angles within zoom [{t0:.1f},{t1:.1f}]s\n'
                  f'(normal walking: hip pitch 0.3 rad p-p, knee 0.6 rad p-p, ankle 0.2 rad p-p)',
                  fontsize=12)
    fig3.tight_layout()
    p3 = os.path.join(out_dir, f'joints_{int(t0)}_{int(t1)}.png')
    fig3.savefig(p3, dpi=120, bbox_inches='tight'); plt.close(fig3)
    print(f'wrote {p3}')

    # single-stance close-up: find the first stance segment inside window
    d = np.diff(np.concatenate([[0], CL.astype(int), [0]]))
    starts = np.where(d == 1)[0]; ends = np.where(d == -1)[0]
    inwin = [(s, e) for s, e in zip(starts, ends)
             if s >= m.argmax() and e <= m[::-1].argmax() + len(m) - 1]
    if not inwin:
        print('no full stance in window'); return
    s, e = inwin[len(inwin)//2]
    tt = t[s:e]
    print(f'first stance inside window:  t=[{tt[0]:.3f}, {tt[-1]:.3f}]s  '
          f'len={e-s}  Δfoot_x={FL[e-1,0]-FL[s,0]:+.3f}m  '
          f'Δfoot_y={FL[e-1,1]-FL[s,1]:+.3f}m   '
          f'|Δxy|={np.linalg.norm(FL[e-1,:2]-FL[s,:2]):.3f}m')


if __name__ == '__main__':
    main()
