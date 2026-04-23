#!/usr/bin/env python3
"""Measure implied body XY travel from FK during stance, using z-proxy contact.

Core identity: during stance, foot is fixed in world, so body_displacement_world
equals -R · Δ(foot_in_body). Ignoring heading for a moment, XY magnitude of
body travel ≈ XY magnitude of Δ(foot_in_body).

If summing |Δfoot_in_body_xy| across all stance periods gives ≈ the reference
63 m, the FK chain is fine and drift is in world-frame fusion (rotation/init
yaw/bias).  If it gives ≈ 150 m, the FK chain itself is producing ~2.5x
inflated body displacements — pointing at joint_sign/offset or URDF link
lengths.
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

Z_MARGIN = 0.02  # default; overridden by --z-margin



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t-abs-lo', type=float, default=None)
    ap.add_argument('--t-abs-hi', type=float, default=None)
    ap.add_argument('--ref-path-m', type=float, default=62.77,
                    help='reference XY path length for comparison')
    ap.add_argument('--z-margin', type=float, default=Z_MARGIN,
                    help='height above per-foot z-floor still counted as stance')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    out_dir = args.out or os.path.join(os.path.dirname(bag_dir),
                                       f'fktravel_{os.path.basename(bag_dir)}')
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
    while r.has_next():
        topic, data, ts = r.read_next()
        t_abs = ts * 1e-9
        if args.t_abs_lo and t_abs < args.t_abs_lo: continue
        if args.t_abs_hi and t_abs > args.t_abs_hi: break
        if topic != '/joint_states': continue
        msg = rclpy.serialization.deserialize_message(data, JointState)
        for i, name in enumerate(msg.name):
            urdf = JOINT_MAPPING.get(name)
            if urdf and len(msg.position) > i:
                latest_joints[urdf] = msg.position[i]
        if len(latest_joints) < len(JOINT_MAPPING): continue
        tl.append(t_abs)
        FL.append(kin.fk_left(latest_joints))
        FR.append(kin.fk_right(latest_joints))

    t = np.array(tl); t -= t[0]
    FL = np.array(FL); FR = np.array(FR)
    print(f'samples={len(t)}  duration={t[-1]:.1f}s')

    # z-proxy stance
    zl_floor = np.percentile(FL[:, 2], 2.0)
    zr_floor = np.percentile(FR[:, 2], 2.0)
    CL = FL[:, 2] < zl_floor + args.z_margin
    CR = FR[:, 2] < zr_floor + args.z_margin
    print(f'z_margin={args.z_margin*100:.1f}cm   '
          f'L stance ratio={CL.mean()*100:.1f}%   R={CR.mean()*100:.1f}%   '
          f'both={(CL&CR).mean()*100:.1f}%')

    def stance_segments(C):
        """Return list of (start_idx, end_idx) for contiguous stance periods."""
        d = np.diff(np.concatenate([[0], C.astype(int), [0]]))
        starts = np.where(d == 1)[0]
        ends = np.where(d == -1)[0]  # exclusive
        return list(zip(starts, ends))

    def analyze(name, FK, C):
        segs = stance_segments(C)
        valid = [(s, e) for s, e in segs if e - s >= 3]  # ≥ 3 samples
        per_seg = []
        for s, e in valid:
            dxy = FK[e - 1, :2] - FK[s, :2]
            per_seg.append(np.linalg.norm(dxy))
        per_seg = np.array(per_seg)
        total = per_seg.sum()
        print(f'[{name}]  stance segments={len(valid)}   '
              f'mean seg |ΔFKxy|={per_seg.mean() if len(per_seg) else 0:.3f} m   '
              f'median={np.median(per_seg) if len(per_seg) else 0:.3f} m   '
              f'max={per_seg.max() if len(per_seg) else 0:.3f} m')
        print(f'[{name}]  Σ|ΔFKxy| over stance = {total:.2f} m   '
              f'(ref path = {args.ref_path_m:.2f} m)')
        return per_seg, total, valid

    per_l, tot_l, segs_l = analyze('LEFT ', FL, CL)
    per_r, tot_r, segs_r = analyze('RIGHT', FR, CR)
    total = tot_l + tot_r
    print(f'\nper-stance-segment  Σ|ΔFKxy| (L+R) = {total:.2f} m   '
          f'vs ref path  {args.ref_path_m:.2f} m   ratio = {total/args.ref_path_m:.2f}×')

    # === stride-to-stride: consecutive stance-starts of the SAME foot ===
    # between two stance-starts the body should have advanced by one full stride;
    # foot-in-body retreats and re-lands. Magnitude |FKxy(stance_start_i+1) -
    # FKxy(stance_start_i)| ≈ stride length for the opposite foot's motion in
    # the same frame. More importantly, the SUM across all strides must equal
    # body XY path — independent of Z_MARGIN.
    def stride_lengths(FK, C):
        segs = stance_segments(C)
        starts = [s for s, e in segs if e - s >= 3]
        if len(starts) < 2:
            return np.array([])
        vecs = np.diff(FK[starts, :2], axis=0)
        # Body moves in -Δfoot direction, so |Δbody| = |Δfoot|
        return np.linalg.norm(vecs, axis=1)

    sl_l = stride_lengths(FL, CL)
    sl_r = stride_lengths(FR, CR)
    print('\n--- stride-to-stride (same-foot consecutive stance-starts) ---')
    print(f'LEFT   strides={len(sl_l)}   mean={sl_l.mean() if len(sl_l) else 0:.3f} m   '
          f'median={np.median(sl_l) if len(sl_l) else 0:.3f} m   Σ={sl_l.sum():.2f} m')
    print(f'RIGHT  strides={len(sl_r)}   mean={sl_r.mean() if len(sl_r) else 0:.3f} m   '
          f'median={np.median(sl_r) if len(sl_r) else 0:.3f} m   Σ={sl_r.sum():.2f} m')

    # Body advance per stride = stride magnitude (single-leg), but one gait
    # cycle = two strides (L stride + R stride). Counting each foot's strides
    # separately already gives body path once per foot, so total = (Σ_L + Σ_R)/2
    body_travel_stride = (sl_l.sum() + sl_r.sum()) / 2.0
    print(f'\nimplied body XY path from strides = (Σ_L + Σ_R) / 2 = {body_travel_stride:.2f} m')
    print(f'ref path                              = {args.ref_path_m:.2f} m')
    print(f'ratio                                 = {body_travel_stride / args.ref_path_m:.2f}×')
    print('\nInterpretation:')
    print('  ratio ≈ 1.0 → FK chain correctly models body stride, drift is in world-fusion (R, bias)')
    print('  ratio ≈ 2×  → FK systematically doubles stride — likely joint_sign or URDF link length')
    print('  ratio < 0.7 → FK under-reports body motion — stance detection clipping or FK short-arm')

    # Also compute body-velocity magnitude estimated from stance foot motion,
    # compared with the reference's implied average walking speed.
    if args.t_abs_hi and args.t_abs_lo:
        dur = args.t_abs_hi - args.t_abs_lo
    else:
        dur = t[-1]
    ref_speed = args.ref_path_m / dur
    print(f'\nref walking speed = {args.ref_path_m:.2f} / {dur:.1f} = {ref_speed:.3f} m/s')

    # === plot stance segment size histogram ===
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    axes[0].hist(per_l, bins=30, color='C0', alpha=0.7, label=f'left (n={len(per_l)})')
    axes[0].hist(per_r, bins=30, color='C3', alpha=0.7, label=f'right (n={len(per_r)})')
    axes[0].axvline(0.4, color='k', ls='--', lw=0.8, label='typical human step 0.4m')
    axes[0].set_xlabel('|ΔFK_xy| across one stance segment [m]')
    axes[0].set_ylabel('count'); axes[0].grid(alpha=0.3); axes[0].legend()
    axes[0].set_title('Body displacement during each stance (FK, body frame)')

    # cumulative
    cum_l = np.cumsum(per_l); cum_r = np.cumsum(per_r)
    axes[1].plot(np.arange(len(cum_l)), cum_l, color='C0', label=f'left Σ={tot_l:.1f}m')
    axes[1].plot(np.arange(len(cum_r)), cum_r, color='C3', label=f'right Σ={tot_r:.1f}m')
    axes[1].axhline(args.ref_path_m, color='k', ls='--', lw=0.8,
                    label=f'ref path {args.ref_path_m:.1f}m')
    axes[1].set_xlabel('stance segment #'); axes[1].set_ylabel('cumulative m')
    axes[1].grid(alpha=0.3); axes[1].legend()
    axes[1].set_title(f'Cumulative Σ|ΔFK_xy|   total L+R = {total:.1f}m   ratio={total/args.ref_path_m:.2f}×')

    fig.suptitle('FK-derived body travel during stance (z-proxy contact)')
    fig.tight_layout()
    p = os.path.join(out_dir, 'fk_body_travel.png')
    fig.savefig(p, dpi=110, bbox_inches='tight'); plt.close(fig)
    print(f'\nwrote {p}')


if __name__ == '__main__':
    main()
