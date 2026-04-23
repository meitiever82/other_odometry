#!/usr/bin/env python3
"""Diagnostic: does FK-computed ankle_roll_link z vary during stance?

If ankle z (body frame) is constant during stance, the foot acts as a point
contact at the ankle — no heel/toe rolling.  If ankle z varies by more than a
few cm during stance, there is a rolling contact that URDF does not model,
and our FK systematically under-reports body motion by the amount the contact
point rolls forward on the physical foot sole.
"""
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
from leg_odometry.contact_detector import ContactDetector

PKG = Path(__file__).resolve().parent.parent
URDF_PATH = PKG.parent / 'finder_lidar_mapping' / 'glim_ros2' / 'urdf' / 'casbot02_7dof_shell.urdf'
JMAP = {
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
    ap.add_argument('--t-abs-lo', type=float)
    ap.add_argument('--t-abs-hi', type=float)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    out_dir = args.out or os.path.join(os.path.dirname(bag_dir),
                                       f'ankle_{os.path.basename(bag_dir)}')
    os.makedirs(out_dir, exist_ok=True)

    with open(URDF_PATH) as f:
        kin = LegKinematics(f.read(), base_link='base_link',
                            left_foot_link='left_leg_ankle_roll_link',
                            right_foot_link='right_leg_ankle_roll_link')
    det = ContactDetector(threshold=5.0, hysteresis=1.0)

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    tl, FL, FR, CL, CR = [], [], [], [], []
    q, eff = {}, {}
    while r.has_next():
        topic, data, ts = r.read_next()
        t = ts * 1e-9
        if args.t_abs_lo and t < args.t_abs_lo: continue
        if args.t_abs_hi and t > args.t_abs_hi: break
        if topic != '/joint_states': continue
        m = rclpy.serialization.deserialize_message(data, JointState)
        for i, n in enumerate(m.name):
            if i < len(m.effort): eff[n] = m.effort[i]
            urdf = JMAP.get(n)
            if urdf and i < len(m.position): q[urdf] = m.position[i]
        if len(q) < len(JMAP): continue
        cl, cr = det.update(eff.get('LJPITCH', 0.0), eff.get('RJPITCH', 0.0))
        tl.append(t)
        FL.append(kin.fk_left(q)); FR.append(kin.fk_right(q))
        CL.append(cl); CR.append(cr)

    t = np.array(tl); t -= t[0]
    FL = np.array(FL); FR = np.array(FR)
    CL = np.array(CL, bool); CR = np.array(CR, bool)

    def seg_stats(FK, C, name):
        """For each stance segment, compute z range, duration, x range."""
        d = np.diff(np.concatenate([[0], C.astype(int), [0]]))
        s = np.where(d == 1)[0]; e = np.where(d == -1)[0]
        rows = []
        for a, b in zip(s, e):
            if b - a < 3: continue
            zs = FK[a:b, 2]
            xs = FK[a:b, 0]
            rows.append((t[a], t[b] - t[a],
                         zs.min(), zs.max(), zs.max() - zs.min(),
                         xs.min(), xs.max(), xs.max() - xs.min()))
        rows = np.array(rows)
        print(f'\n[{name}]  {len(rows)} stance segments (>=15 ms):')
        print('   duration:      mean={:.0f} ms  median={:.0f} ms'.format(
            float(rows[:,1].mean()*1000), float(np.median(rows[:,1])*1000)))
        print('   ankle z range during stance: mean={:.2f} cm  median={:.2f}  p90={:.2f}  max={:.2f}'.format(
            float(rows[:,4].mean()*100), float(np.median(rows[:,4])*100),
            float(np.percentile(rows[:,4],90)*100), float(rows[:,4].max()*100)))
        print('   ankle x range during stance: mean={:.2f} cm  median={:.2f}  p90={:.2f}  max={:.2f}'.format(
            float(rows[:,7].mean()*100), float(np.median(rows[:,7])*100),
            float(np.percentile(rows[:,7],90)*100), float(rows[:,7].max()*100)))
        return rows

    rows_l = seg_stats(FL, CL, 'LEFT ')
    rows_r = seg_stats(FR, CR, 'RIGHT')

    # --- plot 1: stance-z traces (overlaid, each stance normalized to [0,1]) ---
    fig, axes = plt.subplots(2, 2, figsize=(16, 9))

    def plot_traces(ax, FK, C, name, color):
        d = np.diff(np.concatenate([[0], C.astype(int), [0]]))
        s = np.where(d == 1)[0]; e = np.where(d == -1)[0]
        for a, b in zip(s, e):
            if b - a < 3: continue
            u = np.linspace(0, 1, b - a)
            ax.plot(u, (FK[a:b, 2] - FK[a:b, 2].min()) * 100, lw=0.3, color=color, alpha=0.3)
        ax.set_xlabel('stance phase [0=touch down, 1=lift off]')
        ax.set_ylabel('ankle z above stance minimum [cm]')
        ax.set_title(f'{name} ankle z trace within each stance (n={len(rows_l if name=="LEFT" else rows_r)})')
        ax.grid(alpha=0.3)

    plot_traces(axes[0, 0], FL, CL, 'LEFT',  'C0')
    plot_traces(axes[0, 1], FR, CR, 'RIGHT', 'C3')

    # --- plot 2: histograms of stance z-range and x-range ---
    ax = axes[1, 0]
    ax.hist(rows_l[:, 4] * 100, bins=40, alpha=0.6, color='C0', label=f'left (median {np.median(rows_l[:,4])*100:.1f}cm)')
    ax.hist(rows_r[:, 4] * 100, bins=40, alpha=0.6, color='C3', label=f'right (median {np.median(rows_r[:,4])*100:.1f}cm)')
    ax.set_xlabel('ankle z range within stance [cm]'); ax.set_ylabel('count')
    ax.set_title('If ankle is true contact point: peak should be near 0 cm.\n'
                 'If foot rolls heel→toe: peak at 3-6 cm.')
    ax.grid(alpha=0.3); ax.legend()

    ax = axes[1, 1]
    ax.hist(rows_l[:, 7] * 100, bins=40, alpha=0.6, color='C0', label=f'left (median {np.median(rows_l[:,7])*100:.1f}cm)')
    ax.hist(rows_r[:, 7] * 100, bins=40, alpha=0.6, color='C3', label=f'right (median {np.median(rows_r[:,7])*100:.1f}cm)')
    ax.set_xlabel('ankle x range within stance [cm]')
    ax.set_title('(x is body-forward. Large values = body translated over the foot.)')
    ax.grid(alpha=0.3); ax.legend()

    fig.suptitle('Ankle (ankle_roll_link) position within each stance segment — FK body frame',
                 fontsize=13)
    fig.tight_layout()
    p = os.path.join(out_dir, 'ankle_z_stance.png')
    fig.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
    print(f'\nwrote {p}')


if __name__ == '__main__':
    main()
