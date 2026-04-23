#!/usr/bin/env python3
"""FK sanity check: drive forward kinematics from bag's joint_states and look
at the foot positions in the body frame. If the single-step displacement is
already 2-3x a normal human/robot step, the problem is in joint sign/offset
or URDF link lengths — not in the ESKF world-frame fusion.

Usage:
    python3 fk_sanity_check.py <bag_dir> [--t-abs-lo T --t-abs-hi T]
"""
import argparse
import os
import sys
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
from leg_odometry.contact_detector import BiasCompensatedContactDetector

PKG_DIR = Path(__file__).resolve().parent.parent
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t-abs-lo', type=float, default=None)
    ap.add_argument('--t-abs-hi', type=float, default=None)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    out_dir = args.out or os.path.join(os.path.dirname(bag_dir),
                                       f'fk_{os.path.basename(bag_dir)}')
    os.makedirs(out_dir, exist_ok=True)

    with open(URDF_PATH) as f:
        kin = LegKinematics(f.read(), base_link='base_link',
                            left_foot_link='left_leg_ankle_roll_link',
                            right_foot_link='right_leg_ankle_roll_link')

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    det = BiasCompensatedContactDetector(threshold=5.0, hysteresis=1.0, window_size=1000)

    t_list = []
    fk_l = []; fk_r = []
    contact_l = []; contact_r = []
    latest_joints = {}
    latest_efforts = {}

    while r.has_next():
        topic, data, ts = r.read_next()
        t_abs = ts * 1e-9
        if args.t_abs_lo and t_abs < args.t_abs_lo: continue
        if args.t_abs_hi and t_abs > args.t_abs_hi: break
        if topic != '/joint_states': continue
        msg = rclpy.serialization.deserialize_message(data, JointState)
        for i, name in enumerate(msg.name):
            if len(msg.effort) > i:
                latest_efforts[name] = msg.effort[i]
            urdf = JOINT_MAPPING.get(name)
            if urdf and len(msg.position) > i:
                latest_joints[urdf] = msg.position[i]
        if len(latest_joints) < len(JOINT_MAPPING):
            continue
        p_l = kin.fk_left(latest_joints)
        p_r = kin.fk_right(latest_joints)
        cl, cr = det.update(latest_efforts.get('LJPITCH', 0.0),
                            latest_efforts.get('RJPITCH', 0.0))
        t_list.append(t_abs)
        fk_l.append(p_l.copy()); fk_r.append(p_r.copy())
        contact_l.append(cl); contact_r.append(cr)

    t = np.array(t_list); t -= t[0]
    FL = np.array(fk_l); FR = np.array(fk_r)
    CL = np.array(contact_l, dtype=bool); CR = np.array(contact_r, dtype=bool)
    print(f'samples={len(t)}  duration={t[-1]:.1f}s')
    print(f'FL (body-frame) range:  x=[{FL[:,0].min():+.3f},{FL[:,0].max():+.3f}]  '
          f'y=[{FL[:,1].min():+.3f},{FL[:,1].max():+.3f}]  '
          f'z=[{FL[:,2].min():+.3f},{FL[:,2].max():+.3f}]')
    print(f'FR (body-frame) range:  x=[{FR[:,0].min():+.3f},{FR[:,0].max():+.3f}]  '
          f'y=[{FR[:,1].min():+.3f},{FR[:,1].max():+.3f}]  '
          f'z=[{FR[:,2].min():+.3f},{FR[:,2].max():+.3f}]')

    # step size = stride-to-stride displacement in body frame
    # a "step" = one stance period → next stance period of the same foot
    def step_sizes(FK, contact):
        """For each contact -> no-contact -> contact cycle, compute ΔFK between
        start of previous stance and start of current stance."""
        edges = np.diff(contact.astype(int))
        stance_start = np.where(edges == 1)[0] + 1  # first contact frame
        if len(stance_start) < 2:
            return np.array([]), np.array([])
        dx = np.diff(FK[stance_start, 0])
        dy = np.diff(FK[stance_start, 1])
        dz = np.diff(FK[stance_start, 2])
        lateral = np.hypot(dx, dy)
        return lateral, np.column_stack([dx, dy, dz])

    step_l_len, step_l_xyz = step_sizes(FL, CL)
    step_r_len, step_r_xyz = step_sizes(FR, CR)
    print(f'\nleft  step count={len(step_l_len)}  '
          f'median |Δxy|={np.median(step_l_len) if len(step_l_len) else float("nan"):.3f}m  '
          f'mean={step_l_len.mean() if len(step_l_len) else float("nan"):.3f}  '
          f'max={step_l_len.max() if len(step_l_len) else float("nan"):.3f}')
    print(f'right step count={len(step_r_len)}  '
          f'median |Δxy|={np.median(step_r_len) if len(step_r_len) else float("nan"):.3f}m  '
          f'mean={step_r_len.mean() if len(step_r_len) else float("nan"):.3f}  '
          f'max={step_r_len.max() if len(step_r_len) else float("nan"):.3f}')

    # EKF body-displacement per-step = total of body displacement implied by FK
    # between consecutive contact instants of the *opposite* foot (approx).
    # Simpler proxy: accumulated |ΔFK| during one foot's full stance.
    def stance_implied_body_travel(FK, contact):
        """In stance, the world-fixed foot means body velocity = -R·J·q̇.
        Body translation during a stance period ≈ -(FK_end - FK_start) in body frame
        (rotated back, but for XY small-yaw this is ~|ΔFK_xy|).
        Summing across stance periods gives what the EKF would integrate."""
        edges = np.diff(contact.astype(int))
        starts = np.where(edges == 1)[0] + 1
        ends = np.where(edges == -1)[0] + 1
        total = 0.0
        count = 0
        for s, e in zip(starts, ends):
            if e <= s: continue
            total += np.hypot(FK[e, 0] - FK[s, 0], FK[e, 1] - FK[s, 1])
            count += 1
        return total, count

    tot_l, n_l = stance_implied_body_travel(FL, CL)
    tot_r, n_r = stance_implied_body_travel(FR, CR)
    print(f'\nimplied body XY travel from FK during stance:')
    print(f'  left:  {tot_l:.2f} m over {n_l} stance periods  (avg {tot_l/max(n_l,1):.3f} m/stance)')
    print(f'  right: {tot_r:.2f} m over {n_r} stance periods  (avg {tot_r/max(n_r,1):.3f} m/stance)')
    print(f'  sum:   {tot_l + tot_r:.2f} m')

    # === plots ===
    fig, axes = plt.subplots(3, 2, figsize=(18, 11), sharex=True)
    for col, (name, FK, contact, col_color) in enumerate([
        ('Left foot FK (body frame)', FL, CL, 'C0'),
        ('Right foot FK (body frame)', FR, CR, 'C3'),
    ]):
        for row, (axis, ylabel, rng) in enumerate([
            (0, 'x [m]  (+forward)', FK[:, 0]),
            (1, 'y [m]  (+left)',    FK[:, 1]),
            (2, 'z [m]  (≈ -height)', FK[:, 2]),
        ]):
            ax = axes[row, col]
            ax.plot(t, rng, lw=0.7, color=col_color)
            ax.set_title(f'{name}   {ylabel}', fontsize=10)
            # stance shading
            ax2 = ax.twinx()
            ax2.fill_between(t, 0, contact.astype(float), step='post', alpha=0.12, color='g')
            ax2.set_ylim(0, 1.05); ax2.set_yticks([])
            ax.grid(alpha=0.3)
            if row == 2: ax.set_xlabel('t [s]')
    fig.suptitle(f'FK sanity check  —  foot positions in body frame\n'
                 f'left median step |Δxy|={np.median(step_l_len) if len(step_l_len) else float("nan"):.2f}m   '
                 f'right median step |Δxy|={np.median(step_r_len) if len(step_r_len) else float("nan"):.2f}m   '
                 f'(normal walking: 0.30-0.50m; 2.6× overcount here implies ~1.3m)',
                 fontsize=12, y=0.995)
    fig.tight_layout()
    p = os.path.join(out_dir, 'fk_foot_body.png')
    fig.savefig(p, dpi=110, bbox_inches='tight'); plt.close(fig)
    print(f'\nwrote {p}')

    # XY trace in body frame
    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    ax.plot(FL[:, 0], FL[:, 1], lw=0.5, color='C0', alpha=0.6, label='Left foot XY')
    ax.plot(FR[:, 0], FR[:, 1], lw=0.5, color='C3', alpha=0.6, label='Right foot XY')
    # overlay stance-start points
    edges_l = np.diff(CL.astype(int)); ss_l = np.where(edges_l == 1)[0] + 1
    edges_r = np.diff(CR.astype(int)); ss_r = np.where(edges_r == 1)[0] + 1
    ax.scatter(FL[ss_l, 0], FL[ss_l, 1], c='C0', s=20, zorder=5, label='L stance-start')
    ax.scatter(FR[ss_r, 0], FR[ss_r, 1], c='C3', s=20, zorder=5, label='R stance-start')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlabel('x [m] (body +forward)'); ax.set_ylabel('y [m] (body +left)')
    ax.set_title('Foot positions in body frame (dots = stance-start points)')
    ax.legend()
    p = os.path.join(out_dir, 'fk_foot_xy.png')
    fig.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
    print(f'wrote {p}')

    # step size histogram
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.5))
    axes[0].hist(step_l_len, bins=30, color='C0', alpha=0.7, label=f'left  n={len(step_l_len)}')
    axes[0].hist(step_r_len, bins=30, color='C3', alpha=0.7, label=f'right n={len(step_r_len)}')
    axes[0].axvline(0.4, color='k', ls='--', lw=0.8, label='typical human step 0.4m')
    axes[0].set_xlabel('|Δxy| between consecutive stance-starts [m]')
    axes[0].set_ylabel('count'); axes[0].legend(); axes[0].grid(alpha=0.3)
    axes[0].set_title('Inter-stance step size (body frame)')

    all_xyz = np.vstack([step_l_xyz, step_r_xyz]) if len(step_l_xyz) and len(step_r_xyz) else (step_l_xyz if len(step_l_xyz) else step_r_xyz)
    if len(all_xyz):
        axes[1].scatter(all_xyz[:, 0], all_xyz[:, 1], s=10, alpha=0.6)
        axes[1].axhline(0, color='k', lw=0.3); axes[1].axvline(0, color='k', lw=0.3)
        axes[1].set_aspect('equal'); axes[1].grid(alpha=0.3)
        axes[1].set_xlabel('Δx [m]'); axes[1].set_ylabel('Δy [m]')
        axes[1].set_title('Step vector distribution (body frame)')
    p = os.path.join(out_dir, 'step_distribution.png')
    fig.savefig(p, dpi=110, bbox_inches='tight'); plt.close(fig)
    print(f'wrote {p}')


if __name__ == '__main__':
    main()
