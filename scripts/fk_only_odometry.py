#!/usr/bin/env python3
"""Pure FK-based leg odometry (no EKF, no IMU fusion).

For each stance sample, body_vel_world ≈ -R_yaw @ (Jxy · q̇_xy) where R_yaw is
built by integrating gyro_z from /imu. Position = integral of body_vel_world.

This bypasses the whole ESKF/ZUPT machinery — if this trajectory matches
traj_imu.txt, the EKF is making things worse than the raw FK signal, and the
right fix is to simplify the fusion rather than tune it.

Usage:
    python3 fk_only_odometry.py <bag_dir> [--t-abs-lo T --t-abs-hi T]
"""
import argparse, os, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import rclpy.serialization
from sensor_msgs.msg import JointState, Imu

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
                                       f'fkonly_{os.path.basename(bag_dir)}')
    os.makedirs(out_dir, exist_ok=True)

    with open(URDF_PATH) as f:
        kin = LegKinematics(f.read(), base_link='base_link',
                            left_foot_link='left_leg_ankle_roll_link',
                            right_foot_link='right_leg_ankle_roll_link')

    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id='sqlite3'),
           ConverterOptions('cdr', 'cdr'))

    det = ContactDetector(threshold=5.0, hysteresis=1.0)

    # state
    yaw = 0.0
    pos = np.zeros(3)
    last_t = None
    last_FL = None
    last_FR = None
    gyro_bias_z = 0.0
    n_static = 0
    accum_gyro_z = 0.0

    # storage
    t_abs_list = []
    t_list, pos_list, yaw_list, speed_list, contact_list = [], [], [], [], []
    diag_list = []   # per-step (vxy, vL, vR, cl, cr, last_gz)
    latest_q, latest_qd, latest_eff = {}, {}, {}
    last_gyro_z = 0.0
    n_fk = 0

    # first pass: estimate gyro-z bias from initial static period (first 3s of window)
    bias_window_end = args.t_abs_lo + 3.0 if args.t_abs_lo else None

    print('running FK-only odometry ...')
    while r.has_next():
        topic, data, ts = r.read_next()
        t_abs = ts * 1e-9
        if args.t_abs_lo and t_abs < args.t_abs_lo: continue
        if args.t_abs_hi and t_abs > args.t_abs_hi: break

        if topic == '/imu':
            msg = rclpy.serialization.deserialize_message(data, Imu)
            gz = msg.angular_velocity.z
            # bias estimate during initial period
            if bias_window_end and t_abs < bias_window_end:
                accum_gyro_z += gz; n_static += 1
                last_gyro_z = gz
                continue
            elif n_static > 0 and gyro_bias_z == 0.0:
                gyro_bias_z = accum_gyro_z / n_static
                print(f'gyro-z bias estimated from {n_static} samples: {gyro_bias_z:.5f} rad/s')
            last_gyro_z = gz - gyro_bias_z
            continue

        if topic != '/joint_states':
            continue

        msg = rclpy.serialization.deserialize_message(data, JointState)
        for i, name in enumerate(msg.name):
            if len(msg.effort) > i:
                latest_eff[name] = msg.effort[i]
            urdf = JOINT_MAPPING.get(name)
            if urdf is None: continue
            if len(msg.position) > i: latest_q[urdf]  = msg.position[i]
            if len(msg.velocity) > i: latest_qd[urdf] = msg.velocity[i]
        if len(latest_q) < len(JOINT_MAPPING): continue

        if last_t is None:
            last_t = t_abs
            continue
        dt = t_abs - last_t
        last_t = t_abs
        if dt <= 0 or dt > 0.1:
            continue

        # propagate yaw with gyro z
        yaw += last_gyro_z * dt

        # compute foot velocity via Jacobian
        vL_body = kin.foot_velocity_left(latest_q, latest_qd)
        vR_body = kin.foot_velocity_right(latest_q, latest_qd)

        # contact detection
        cl, cr = det.update(latest_eff.get('LJPITCH', 0.0), latest_eff.get('RJPITCH', 0.0))

        # body velocity in world:
        # During stance, foot_world ≈ fixed, so body_vel_world = -R @ v_foot_body
        # With only yaw (pitch/roll assumed 0 for this 2D model),
        # R = Rz(yaw). Project only the XY components.
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[cy, -sy], [sy, cy]])
        if cl and cr:
            v_world_xy = -0.5 * Rz @ (vL_body[:2] + vR_body[:2])
        elif cl:
            v_world_xy = -Rz @ vL_body[:2]
        elif cr:
            v_world_xy = -Rz @ vR_body[:2]
        else:
            v_world_xy = np.zeros(2)  # no stance, hold position (or could use IMU)

        pos[0] += v_world_xy[0] * dt
        pos[1] += v_world_xy[1] * dt

        t_list.append(t_abs); pos_list.append(pos.copy()); yaw_list.append(yaw)
        speed_list.append(np.linalg.norm(v_world_xy))
        contact_list.append((int(cl), int(cr)))
        diag_list.append((v_world_xy[0], v_world_xy[1],
                          vL_body[0], vL_body[1], vR_body[0], vR_body[1],
                          int(cl), int(cr), last_gyro_z))
        n_fk += 1

    t_abs_arr = np.array(t_list)
    t = t_abs_arr - t_abs_arr[0]
    P = np.array(pos_list)
    yaws = np.array(yaw_list)
    print(f'\nFK-only odom done.  {n_fk} updates.')
    est_path = float(np.sum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1)))
    loop_err = float(np.linalg.norm(P[-1, :2] - P[0, :2]))
    print(f'est_path = {est_path:.2f} m   loop_err = {loop_err:.2f} m')

    # Dump absolute-time position trace CSV (for C++ / other comparisons)
    csv_out = os.path.join(out_dir, 'fk_only_py.csv')
    with open(csv_out, 'w') as f:
        f.write('t_abs,x,y,z,yaw\n')
        for i in range(len(t_abs_arr)):
            f.write(f'{t_abs_arr[i]:.6f},{P[i,0]:.6f},{P[i,1]:.6f},{P[i,2]:.6f},{yaws[i]:.6f}\n')
    print(f'wrote {csv_out}')

    # Diagnostic CSV: per-step vxy, vL, vR, cl, cr, last_gz — for C++ parity check
    diag_out = os.path.join(out_dir, 'fk_only_py_diag.csv')
    with open(diag_out, 'w') as f:
        f.write('t_abs,x,y,yaw,vxy_x,vxy_y,vL_x,vL_y,vR_x,vR_y,cl,cr,last_gz\n')
        for i in range(len(t_abs_arr)):
            d = diag_list[i]
            f.write(f'{t_abs_arr[i]:.6f},{P[i,0]:.6f},{P[i,1]:.6f},{yaws[i]:.6f},'
                    f'{d[0]:.6f},{d[1]:.6f},{d[2]:.6f},{d[3]:.6f},{d[4]:.6f},{d[5]:.6f},'
                    f'{d[6]},{d[7]},{d[8]:.6f}\n')
    print(f'wrote {diag_out}')

    # load ref
    ref_path = os.path.join(bag_dir, 'traj_imu.txt')
    ref = np.loadtxt(ref_path)
    if args.t_abs_lo is not None or args.t_abs_hi is not None:
        m = np.ones(len(ref), bool)
        if args.t_abs_lo: m &= ref[:, 0] >= args.t_abs_lo
        if args.t_abs_hi: m &= ref[:, 0] <= args.t_abs_hi
        ref = ref[m]
    ref_xy = ref[:, 1:3] - ref[0, 1:3]
    ref_path_len = float(np.sum(np.linalg.norm(np.diff(ref_xy, axis=0), axis=1)))
    ref_loop = float(np.linalg.norm(ref_xy[-1] - ref_xy[0]))
    print(f'ref path = {ref_path_len:.2f} m   ref loop_err = {ref_loop:.2f} m')
    print(f'path ratio = {est_path / ref_path_len:.2f}×')

    # --- align FK-only to ref via 2D rigid transform (Kabsch, with optional reflection) ---
    # Resample both to a common time base
    t_abs_est = np.array(t_list)
    t_abs_ref = ref[:, 0]
    t0_common = max(t_abs_est[0], t_abs_ref[0])
    t1_common = min(t_abs_est[-1], t_abs_ref[-1])
    t_common = np.linspace(t0_common, t1_common, 500)
    est_rs = np.column_stack([
        np.interp(t_common, t_abs_est, P[:, 0]),
        np.interp(t_common, t_abs_est, P[:, 1]),
    ])
    ref_rs = np.column_stack([
        np.interp(t_common, t_abs_ref, ref[:, 1] - ref[0, 1]),
        np.interp(t_common, t_abs_ref, ref[:, 2] - ref[0, 2]),
    ])

    def kabsch(A, B, allow_reflection=False):
        """Find rigid transform (R, t) mapping A -> B. A, B are (N, 2)."""
        cA = A.mean(axis=0); cB = B.mean(axis=0)
        H = (A - cA).T @ (B - cB)
        U, S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        if allow_reflection:
            R = Vt.T @ U.T  # may include reflection
        else:
            R = Vt.T @ np.diag([1, d]) @ U.T
        t = cB - R @ cA
        return R, t

    def apply(A, R, t): return A @ R.T + t
    def rmse(A, B):     return float(np.sqrt(np.mean(np.sum((A - B)**2, axis=1))))

    R_pure, t_pure = kabsch(est_rs, ref_rs, allow_reflection=False)
    R_refl, t_refl = kabsch(est_rs, ref_rs, allow_reflection=True)
    est_aligned_pure = apply(est_rs, R_pure, t_pure)
    est_aligned_refl = apply(est_rs, R_refl, t_refl)
    rmse_pure = rmse(est_aligned_pure, ref_rs)
    rmse_refl = rmse(est_aligned_refl, ref_rs)
    det_refl = float(np.linalg.det(R_refl))
    yaw_pure = float(np.degrees(np.arctan2(R_pure[1, 0], R_pure[0, 0])))
    yaw_refl = float(np.degrees(np.arctan2(R_refl[1, 0], R_refl[0, 0])))
    print(f'\nalignment:')
    print(f'  pure rotation     yaw={yaw_pure:+.2f}°   RMSE={rmse_pure:.3f} m   t={t_pure}')
    print(f'  with reflection   yaw={yaw_refl:+.2f}°   det={det_refl:+.2f}   '
          f'RMSE={rmse_refl:.3f} m   t={t_refl}')
    use_refl = rmse_refl < rmse_pure * 0.8
    if use_refl:
        print(f'  >> reflection fits BETTER (ratio {rmse_refl/rmse_pure:.2f}) — '
              f'the two world frames are mirrored')
    est_aligned_best = est_aligned_refl if use_refl else est_aligned_pure
    rmse_best = rmse_refl if use_refl else rmse_pure
    R_best = R_refl if use_refl else R_pure
    t_best = t_refl if use_refl else t_pure
    # apply the same transform to the full-rate estimate for plotting
    P_aligned = apply(P[:, :2], R_best, t_best)

    # plot: side-by-side raw and aligned
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 9))
    for ax, (title, est_plot) in zip(
        [ax1, ax2],
        [('Raw FK-only (body-IMU frame)', P[:, :2]),
         (f'Aligned to ref frame '
          f'{"(with reflection)" if use_refl else "(pure rotation)"}\n'
          f'yaw={yaw_refl if use_refl else yaw_pure:+.1f}°  '
          f'RMSE={rmse_best:.2f}m', P_aligned)]):
        ax.plot(ref_xy[:, 0], ref_xy[:, 1], lw=2.5, color='k',
                label=f'ref (GLIM)  len={ref_path_len:.1f}m')
        ax.plot(est_plot[:, 0], est_plot[:, 1], lw=1.4, color='C2',
                label=f'FK-only  len={est_path:.1f}m  ratio={est_path/ref_path_len:.2f}×')
        ax.scatter([est_plot[0, 0]], [est_plot[0, 1]], c='g', s=80, zorder=5, label='start')
        ax.scatter([ref_xy[-1, 0]], [ref_xy[-1, 1]], c='k', s=80, marker='x', zorder=5, label='ref end')
        ax.scatter([est_plot[-1, 0]], [est_plot[-1, 1]], c='r', s=80, marker='x', zorder=5, label='est end')
        ax.set_aspect('equal'); ax.grid(alpha=0.3)
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
        ax.set_title(title)
        ax.legend(loc='best', fontsize=9)

    fig.suptitle('FK-only leg odometry (no EKF, no IMU accel) vs GLIM reference',
                 fontsize=13)
    fig.tight_layout()
    p = os.path.join(out_dir, 'fkonly_xy.png')
    fig.savefig(p, dpi=120, bbox_inches='tight'); plt.close(fig)
    print(f'wrote {p}')


if __name__ == '__main__':
    main()
