#!/usr/bin/env python3
"""Compare effort-based contact detection against a kinematic proxy:
FK foot z near its minimum = foot on ground = stance.

Usage:
    python3 verify_contact.py <bag_dir> [--t-abs-lo T --t-abs-hi T]
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
from leg_odometry.contact_detector import ContactDetector, BiasCompensatedContactDetector

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
STANCE_Z_MARGIN = 0.02   # m; foot within this height of its per-run minimum → stance


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--t-abs-lo', type=float, default=None)
    ap.add_argument('--t-abs-hi', type=float, default=None)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    bag_dir = os.path.abspath(args.bag)
    out_dir = args.out or os.path.join(os.path.dirname(bag_dir),
                                       f'verify_{os.path.basename(bag_dir)}')
    os.makedirs(out_dir, exist_ok=True)

    with open(URDF_PATH) as f:
        kin = LegKinematics(f.read(), base_link='base_link',
                            left_foot_link='left_leg_ankle_roll_link',
                            right_foot_link='right_leg_ankle_roll_link')

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    det_raw = ContactDetector(threshold=5.0, hysteresis=1.0)
    det_comp = BiasCompensatedContactDetector(threshold=5.0, hysteresis=1.0, window_size=1000)

    t_list, zl_list, zr_list = [], [], []
    eff_l_list, eff_r_list = [], []
    raw_cl, raw_cr = [], []
    comp_cl, comp_cr = [], []
    latest_joints, latest_efforts = {}, {}

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
        p_l = kin.fk_left(latest_joints); p_r = kin.fk_right(latest_joints)
        eff_l = latest_efforts.get('LJPITCH', 0.0)
        eff_r = latest_efforts.get('RJPITCH', 0.0)
        rl, rr = det_raw.update(eff_l, eff_r)
        cl, cr = det_comp.update(eff_l, eff_r)

        t_list.append(t_abs); zl_list.append(p_l[2]); zr_list.append(p_r[2])
        eff_l_list.append(eff_l); eff_r_list.append(eff_r)
        raw_cl.append(rl); raw_cr.append(rr)
        comp_cl.append(cl); comp_cr.append(cr)

    t  = np.array(t_list); t -= t[0]
    zl = np.array(zl_list); zr = np.array(zr_list)
    eff_l = np.array(eff_l_list); eff_r = np.array(eff_r_list)
    RL = np.array(raw_cl, bool);  RR = np.array(raw_cr, bool)
    CL = np.array(comp_cl, bool); CR = np.array(comp_cr, bool)
    print(f'samples={len(t)}  duration={t[-1]:.1f}s')

    # === z-based stance ground-truth proxy ===
    # use per-foot minimum observed, stance when z < min + MARGIN
    zl_floor = np.percentile(zl, 2.0)   # 2% tail ≈ pressed-down position
    zr_floor = np.percentile(zr, 2.0)
    ZL = zl < zl_floor + STANCE_Z_MARGIN
    ZR = zr < zr_floor + STANCE_Z_MARGIN
    print(f'foot-z floors:  L={zl_floor:+.3f} m   R={zr_floor:+.3f} m   margin={STANCE_Z_MARGIN}m')
    print(f'z-based stance ratio:  L={ZL.mean()*100:.1f}%   R={ZR.mean()*100:.1f}%   '
          f'both={(ZL&ZR).mean()*100:.1f}%   neither={(~ZL & ~ZR).mean()*100:.1f}%')

    def agree(A, B):
        tp = (A & B).sum(); tn = (~A & ~B).sum()
        fp = (~A & B).sum(); fn = (A & ~B).sum()
        n = len(A)
        return dict(acc=(tp+tn)/n*100, tpr=tp/max(tp+fn,1)*100,
                    fpr=fp/max(fp+tn,1)*100, fnr=fn/max(tp+fn,1)*100)

    print('\n=== agreement with z-based proxy (Z treated as truth) ===')
    for lbl, XL, XR in [('raw',      RL, RR),
                        ('detrend',  CL, CR)]:
        al = agree(ZL, XL); ar = agree(ZR, XR)
        print(f'[{lbl}]  LEFT  acc={al["acc"]:.1f}%  TPR(stance→stance)={al["tpr"]:.1f}%  '
              f'FNR(missed)={al["fnr"]:.1f}%  FPR={al["fpr"]:.1f}%')
        print(f'[{lbl}]  RIGHT acc={ar["acc"]:.1f}%  TPR(stance→stance)={ar["tpr"]:.1f}%  '
              f'FNR(missed)={ar["fnr"]:.1f}%  FPR={ar["fpr"]:.1f}%')

    def stats(A, B):
        return dict(
            l=A.mean()*100, r=B.mean()*100,
            both=(A & B).mean()*100, neither=(~A & ~B).mean()*100)
    print('\n=== stance ratios summary ===')
    for lbl, data in [('raw (5 Nm)', stats(RL, RR)),
                      ('detrend (5 Nm on bg-removed)', stats(CL, CR)),
                      ('z-proxy',   stats(ZL, ZR))]:
        print(f'[{lbl:30s}]  L={data["l"]:5.1f}%  R={data["r"]:5.1f}%  '
              f'both={data["both"]:5.1f}%  neither(flight)={data["neither"]:5.1f}%')

    # === plots ===
    fig, axes = plt.subplots(4, 1, figsize=(20, 13), sharex=True)

    ax = axes[0]
    ax.plot(t, zl, lw=0.6, color='C0', label='left foot z (body)')
    ax.axhline(zl_floor, color='C0', ls='--', lw=0.7, alpha=0.5, label=f'floor={zl_floor:.3f}')
    ax.axhline(zl_floor + STANCE_Z_MARGIN, color='C0', ls=':', lw=0.7, alpha=0.5,
               label=f'threshold={zl_floor+STANCE_Z_MARGIN:.3f}')
    ax2 = ax.twinx()
    ax2.fill_between(t, 0, ZL.astype(float), step='post', alpha=0.15, color='g')
    ax2.set_ylim(0, 1.05); ax2.set_yticks([])
    ax.set_ylabel('FK z  [m]'); ax.set_title('LEFT foot — z-based stance proxy (green fill)')
    ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8)

    ax = axes[1]
    ax.plot(t, zr, lw=0.6, color='C3', label='right foot z')
    ax.axhline(zr_floor, color='C3', ls='--', lw=0.7, alpha=0.5)
    ax.axhline(zr_floor + STANCE_Z_MARGIN, color='C3', ls=':', lw=0.7, alpha=0.5)
    ax2 = ax.twinx()
    ax2.fill_between(t, 0, ZR.astype(float), step='post', alpha=0.15, color='g')
    ax2.set_ylim(0, 1.05); ax2.set_yticks([])
    ax.set_ylabel('FK z  [m]'); ax.set_title('RIGHT foot — z-based stance proxy')
    ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8)

    ax = axes[2]
    ax.fill_between(t, 1.55, 1.55 + ZL.astype(float)*0.4, step='post', alpha=0.6, color='k', label='L z-proxy (truth-ish)')
    ax.fill_between(t, 1.05, 1.05 + RL.astype(float)*0.4, step='post', alpha=0.6, color='C3', label='L raw')
    ax.fill_between(t, 0.55, 0.55 + CL.astype(float)*0.4, step='post', alpha=0.6, color='C2', label='L detrend')
    ax.fill_between(t, 0.05, 0.05 + ZR.astype(float)*0.4, step='post', alpha=0.6, color='grey', label='R z-proxy')
    ax.set_ylim(0, 2.0); ax.set_yticks([0.25, 0.75, 1.25, 1.75])
    ax.set_yticklabels(['R z-proxy', 'L detrend', 'L raw', 'L z-proxy'])
    ax.set_title('LEFT stance decisions overlaid  (top = z-proxy truth, ideally ≈ below two bars)')
    ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8, ncol=4)

    ax = axes[3]
    ax.fill_between(t, 1.55, 1.55 + ZR.astype(float)*0.4, step='post', alpha=0.6, color='k', label='R z-proxy')
    ax.fill_between(t, 1.05, 1.05 + RR.astype(float)*0.4, step='post', alpha=0.6, color='C3', label='R raw')
    ax.fill_between(t, 0.55, 0.55 + CR.astype(float)*0.4, step='post', alpha=0.6, color='C2', label='R detrend')
    ax.fill_between(t, 0.05, 0.05 + ZL.astype(float)*0.4, step='post', alpha=0.6, color='grey', label='L z-proxy (ref)')
    ax.set_ylim(0, 2.0); ax.set_yticks([0.25, 0.75, 1.25, 1.75])
    ax.set_yticklabels(['L z-proxy', 'R detrend', 'R raw', 'R z-proxy'])
    ax.set_title('RIGHT stance decisions overlaid')
    ax.grid(alpha=0.3); ax.legend(loc='upper right', fontsize=8, ncol=4)
    ax.set_xlabel('t [s]')

    fig.suptitle('Contact-detection verification: effort-based vs z-kinematic proxy', fontsize=13)
    fig.tight_layout()
    p = os.path.join(out_dir, 'contact_verify.png')
    fig.savefig(p, dpi=110, bbox_inches='tight'); plt.close(fig)
    print(f'\nwrote {p}')

    # zoom on a short window
    tzoom = (t > t[-1] * 0.4) & (t < t[-1] * 0.4 + 20)
    fig, axes = plt.subplots(2, 1, figsize=(18, 7), sharex=True)
    for ax, side, z, zfloor, R, C, Zt, col in [
        (axes[0], 'LEFT',  zl, zl_floor, RL, CL, ZL, 'C0'),
        (axes[1], 'RIGHT', zr, zr_floor, RR, CR, ZR, 'C3'),
    ]:
        ax.plot(t[tzoom], z[tzoom], lw=1.0, color=col, label=f'{side} FK z')
        ax.axhline(zfloor + STANCE_Z_MARGIN, color=col, ls=':', lw=0.7)
        ax2 = ax.twinx()
        ax2.fill_between(t[tzoom], 2.05, 2.05 + Zt[tzoom].astype(float)*0.4, step='post', alpha=0.5, color='k', label='z-proxy')
        ax2.fill_between(t[tzoom], 1.55, 1.55 + R[tzoom].astype(float)*0.4,  step='post', alpha=0.5, color='C3', label='raw')
        ax2.fill_between(t[tzoom], 1.05, 1.05 + C[tzoom].astype(float)*0.4,  step='post', alpha=0.5, color='C2', label='detrend')
        ax2.set_ylim(0, 2.6); ax2.set_yticks([1.25, 1.75, 2.25]); ax2.set_yticklabels(['detrend','raw','z-proxy'])
        ax2.legend(loc='upper right', fontsize=9)
        ax.set_title(f'{side}   zoom {t[tzoom][0]:.0f}–{t[tzoom][-1]:.0f}s')
        ax.set_ylabel('FK z [m]'); ax.grid(alpha=0.3)
    axes[1].set_xlabel('t [s]')
    fig.tight_layout()
    p = os.path.join(out_dir, 'contact_verify_zoom.png')
    fig.savefig(p, dpi=110, bbox_inches='tight'); plt.close(fig)
    print(f'wrote {p}')


if __name__ == '__main__':
    main()
