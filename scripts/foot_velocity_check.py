#!/usr/bin/env python3
"""Compute J·q̇ (foot linear velocity in body frame) during stance and check
whether its magnitude matches what the reference body speed implies.

If foot retreats in body at |v| ≈ ref body speed  → FK and encoders correct.
If |v| << ref body speed                           → encoders under-report speed,
                                                       or FK chain under-projects
                                                       joint motion to foot.
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
    ap.add_argument('--t-abs-lo', type=float, default=None)
    ap.add_argument('--t-abs-hi', type=float, default=None)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    out_dir = args.out or os.path.join(os.path.dirname(bag_dir),
                                       f'footvel_{os.path.basename(bag_dir)}')
    os.makedirs(out_dir, exist_ok=True)

    with open(URDF_PATH) as f:
        kin = LegKinematics(f.read(), base_link='base_link',
                            left_foot_link='left_leg_ankle_roll_link',
                            right_foot_link='right_leg_ankle_roll_link')

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    latest_q, latest_qd = {}, {}
    latest_effort = {}
    det = ContactDetector(threshold=5.0, hysteresis=1.0)
    tl, vL_jac, vR_jac, vL_num, vR_num, zL, zR = [], [], [], [], [], [], []
    CL_eff = []; CR_eff = []

    prev_FL = None; prev_FR = None; prev_t = None

    while r.has_next():
        topic, data, ts = r.read_next()
        t_abs = ts * 1e-9
        if args.t_abs_lo and t_abs < args.t_abs_lo: continue
        if args.t_abs_hi and t_abs > args.t_abs_hi: break
        if topic != '/joint_states': continue
        msg = rclpy.serialization.deserialize_message(data, JointState)
        for i, name in enumerate(msg.name):
            if len(msg.effort) > i:
                latest_effort[name] = msg.effort[i]
            urdf = JOINT_MAPPING.get(name)
            if urdf is None: continue
            if len(msg.position) > i: latest_q[urdf]  = msg.position[i]
            if len(msg.velocity) > i: latest_qd[urdf] = msg.velocity[i]
        if len(latest_q) < len(JOINT_MAPPING): continue

        # FK position (for numerical derivative as sanity)
        FL = kin.fk_left(latest_q); FR = kin.fk_right(latest_q)
        # Jacobian foot velocity in body frame
        vL = kin.foot_velocity_left(latest_q, latest_qd)
        vR = kin.foot_velocity_right(latest_q, latest_qd)

        vL_num_cur = np.zeros(3); vR_num_cur = np.zeros(3)
        if prev_FL is not None and prev_t is not None:
            dt = t_abs - prev_t
            if dt > 1e-4:
                vL_num_cur = (FL - prev_FL) / dt
                vR_num_cur = (FR - prev_FR) / dt

        cl, cr = det.update(latest_effort.get('LJPITCH', 0.0),
                            latest_effort.get('RJPITCH', 0.0))
        tl.append(t_abs); zL.append(FL[2]); zR.append(FR[2])
        vL_jac.append(vL); vR_jac.append(vR)
        vL_num.append(vL_num_cur); vR_num.append(vR_num_cur)
        CL_eff.append(cl); CR_eff.append(cr)
        prev_FL = FL; prev_FR = FR; prev_t = t_abs

    t = np.array(tl); t -= t[0]
    vL_jac = np.array(vL_jac); vR_jac = np.array(vR_jac)
    vL_num = np.array(vL_num); vR_num = np.array(vR_num)
    zL = np.array(zL); zR = np.array(zR)

    # z-proxy stance and effort-based stance
    zl_floor = np.percentile(zL, 2.0); zr_floor = np.percentile(zR, 2.0)
    CL_z = zL < zl_floor + 0.05; CR_z = zR < zr_floor + 0.05
    CL_e = np.array(CL_eff, bool); CR_e = np.array(CR_eff, bool)
    CL = CL_e; CR = CR_e   # use effort-based from here on
    print(f'effort-based stance: L={CL_e.mean()*100:.1f}%  R={CR_e.mean()*100:.1f}%  '
          f'both={(CL_e&CR_e).mean()*100:.1f}%')

    # stance-only foot speeds
    sp_l_jac = np.linalg.norm(vL_jac[:, :2], axis=1)
    sp_r_jac = np.linalg.norm(vR_jac[:, :2], axis=1)
    sp_l_num = np.linalg.norm(vL_num[:, :2], axis=1)
    sp_r_num = np.linalg.norm(vR_num[:, :2], axis=1)

    print(f'samples={len(t)}  duration={t[-1]:.1f}s')
    print('\n=== foot XY-speed in body frame during STANCE (z-proxy 5cm) ===')
    for lbl, a in [('LEFT  Jacobian J·q̇', sp_l_jac[CL]),
                   ('LEFT  numerical ΔFK ', sp_l_num[CL]),
                   ('RIGHT Jacobian J·q̇ ', sp_r_jac[CR]),
                   ('RIGHT numerical ΔFK', sp_r_num[CR])]:
        if len(a):
            print(f'  {lbl}:  mean={a.mean():.3f}  median={np.median(a):.3f}  '
                  f'p90={np.percentile(a, 90):.3f}  max={a.max():.3f}  m/s')

    # reference speed in window for comparison
    ref_path = os.path.join(bag_dir, 'traj_imu.txt')
    ref_hint = ''
    if os.path.isfile(ref_path):
        ref = np.loadtxt(ref_path)
        m = np.ones(len(ref), bool)
        if args.t_abs_lo: m &= ref[:, 0] >= args.t_abs_lo
        if args.t_abs_hi: m &= ref[:, 0] <= args.t_abs_hi
        ref_xy = ref[m, 1:3]
        ref_t  = ref[m, 0]
        path = float(np.sum(np.linalg.norm(np.diff(ref_xy, axis=0), axis=1)))
        dur  = ref_t[-1] - ref_t[0]
        ref_speed = path / dur
        inst = np.linalg.norm(np.diff(ref_xy, axis=0), axis=1) / np.diff(ref_t)
        ref_hint = f'ref path={path:.1f}m / {dur:.1f}s = {ref_speed:.3f} m/s average,   ' \
                   f'instantaneous median={np.median(inst):.3f} m/s   max={inst.max():.3f} m/s'
        print(f'\n=== reference body speed ===\n  {ref_hint}')

    # === integrated body path from foot velocity during stance ===
    dt = np.diff(t, prepend=t[0])
    dt[0] = dt[1]  # first dt same as next
    path_l_stance_jac = float(np.sum(sp_l_jac[CL] * dt[CL]))
    path_r_stance_jac = float(np.sum(sp_r_jac[CR] * dt[CR]))
    both = CL & CR
    path_dsup_jac = float(np.sum(0.5 * (sp_l_jac[both] + sp_r_jac[both]) * dt[both]))
    path_l_only_jac = float(np.sum(sp_l_jac[CL & ~CR] * dt[CL & ~CR]))
    path_r_only_jac = float(np.sum(sp_r_jac[~CL & CR] * dt[~CL & CR]))
    print('\n=== integrated body path Σ|v_foot_xy|·dt ===')
    print(f'  during LEFT stance only  (L∧¬R):  {path_l_only_jac:.2f} m')
    print(f'  during RIGHT stance only (¬L∧R):  {path_r_only_jac:.2f} m')
    print(f'  during DOUBLE support    (L∧R):   {path_dsup_jac:.2f} m '
          f'(averaged L/R to avoid double-count)')
    print(f'  total body path implied by FK:    '
          f'{path_l_only_jac + path_r_only_jac + path_dsup_jac:.2f} m')
    print(f'  ref body path:                    {path:.2f} m')
    imp = path_l_only_jac + path_r_only_jac + path_dsup_jac
    print(f'  ratio = {imp/path:.2f}×  (≈1 means FK matches ref; >1 FK inflates; <1 FK misses)')

    # zoomed comparison over 10s
    t0 = 30.0; t1 = 40.0
    m = (t >= t0) & (t <= t1)
    fig, axes = plt.subplots(2, 1, figsize=(18, 8), sharex=True)
    axes[0].plot(t[m], sp_l_jac[m], lw=0.7, color='C0', label='left  J·q̇ XY speed')
    axes[0].plot(t[m], sp_l_num[m], lw=0.7, color='C2', alpha=0.6, label='left  numerical ΔFK')
    ax2 = axes[0].twinx()
    ax2.fill_between(t[m], 0, CL[m].astype(float), step='post', alpha=0.15, color='g')
    ax2.set_ylim(0, 1.05); ax2.set_yticks([])
    axes[0].set_ylabel('m/s'); axes[0].grid(alpha=0.3)
    axes[0].legend(loc='upper right')
    axes[0].set_title(f'LEFT foot XY speed in body frame   (green=stance)\n{ref_hint}', fontsize=10)

    axes[1].plot(t[m], sp_r_jac[m], lw=0.7, color='C3', label='right J·q̇ XY speed')
    axes[1].plot(t[m], sp_r_num[m], lw=0.7, color='C4', alpha=0.6, label='right numerical ΔFK')
    ax3 = axes[1].twinx()
    ax3.fill_between(t[m], 0, CR[m].astype(float), step='post', alpha=0.15, color='g')
    ax3.set_ylim(0, 1.05); ax3.set_yticks([])
    axes[1].set_ylabel('m/s'); axes[1].set_xlabel('t [s]')
    axes[1].grid(alpha=0.3); axes[1].legend(loc='upper right')
    axes[1].set_title('RIGHT foot XY speed in body frame')

    fig.tight_layout()
    p = os.path.join(out_dir, f'foot_speed_{int(t0)}_{int(t1)}.png')
    fig.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
    print(f'\nwrote {p}')


if __name__ == '__main__':
    main()
