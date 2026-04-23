#!/usr/bin/env python3
"""Plot Stage 1 C++ trajectory vs GLIM reference, with Kabsch alignment."""
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def kabsch(A, B, allow_reflection=False):
    cA = A.mean(axis=0); cB = B.mean(axis=0)
    H = (A - cA).T @ (B - cB)
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    if allow_reflection:
        R = Vt.T @ U.T
    else:
        R = Vt.T @ np.diag([1, d]) @ U.T
    t = cB - R @ cA
    return R, t


def main():
    cpp_csv = sys.argv[1] if len(sys.argv) > 1 else '/tmp/fk_only_cpp.csv'
    ref_file = sys.argv[2] if len(sys.argv) > 2 else \
        '/home/steve/Documents/Datasets/casbot/leg/rosbag2_2026_04_07-16_12_13/traj_imu.txt'
    out = sys.argv[3] if len(sys.argv) > 3 else '/tmp/stage1_xy.png'
    t_lo = 1775549558.0
    t_hi = 1775549742.0

    # C++ trajectory
    cpp = np.genfromtxt(cpp_csv, delimiter=',', names=True)
    mc = (cpp['t'] >= t_lo) & (cpp['t'] <= t_hi)
    tc = cpp['t'][mc]
    xc = cpp['x'][mc] - cpp['x'][mc][0]
    yc = cpp['y'][mc] - cpp['y'][mc][0]

    # Ref trajectory (TUM: t x y z qx qy qz qw)
    ref = np.loadtxt(ref_file)
    mr = (ref[:, 0] >= t_lo) & (ref[:, 0] <= t_hi)
    tr = ref[mr, 0]
    xr = ref[mr, 1] - ref[mr, 1][0]
    yr = ref[mr, 2] - ref[mr, 2][0]

    path_cpp = float(np.sum(np.hypot(np.diff(xc), np.diff(yc))))
    path_ref = float(np.sum(np.hypot(np.diff(xr), np.diff(yr))))
    loop_cpp = float(np.hypot(xc[-1] - xc[0], yc[-1] - yc[0]))
    loop_ref = float(np.hypot(xr[-1] - xr[0], yr[-1] - yr[0]))

    # Resample both to 500 common points for Kabsch
    n = 500
    tcom = np.linspace(max(tc[0], tr[0]), min(tc[-1], tr[-1]), n)
    est = np.column_stack([np.interp(tcom, tc, xc), np.interp(tcom, tc, yc)])
    refrs = np.column_stack([np.interp(tcom, tr, xr), np.interp(tcom, tr, yr)])
    R, t_kab = kabsch(est, refrs)
    aligned = est @ R.T + t_kab
    rmse = float(np.sqrt(np.mean(np.sum((aligned - refrs)**2, axis=1))))
    yaw_off = float(np.degrees(np.arctan2(R[1, 0], R[0, 0])))

    # Apply same transform to full-rate cpp trajectory for plotting
    full = np.column_stack([xc, yc])
    full_aligned = full @ R.T + t_kab

    print(f'Stage 1 C++ leg odom trajectory:')
    print(f'  window [{t_lo}, {t_hi}]  (184 s of motion)')
    print(f'  C++  path={path_cpp:.2f} m   loop_err={loop_cpp:.2f} m')
    print(f'  ref  path={path_ref:.2f} m   loop_err={loop_ref:.2f} m')
    print(f'  ratio cpp/ref  = {path_cpp/path_ref:.3f}')
    print(f'  after Kabsch:  yaw_offset={yaw_off:+.2f}°   RMSE={rmse:.3f} m')

    fig = plt.figure(figsize=(18, 9))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1])

    # Panel 1: raw frames (each in its own world orientation)
    ax = fig.add_subplot(gs[0])
    ax.plot(xr, yr, lw=2.5, color='k',
            label=f'GLIM ref    path={path_ref:.1f}m  loop_err={loop_ref:.2f}m')
    ax.plot(xc, yc, lw=1.6, color='C2',
            label=f'leg odom C++ (Stage 1)   path={path_cpp:.1f}m  loop_err={loop_cpp:.2f}m')
    ax.scatter([0], [0], c='g', s=80, zorder=5, label='start')
    ax.scatter([xr[-1]], [yr[-1]], c='k', marker='x', s=80, zorder=5, label='ref end')
    ax.scatter([xc[-1]], [yc[-1]], c='C2', marker='x', s=80, zorder=5, label='leg end')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f'Raw frames (GLIM vs Stage 1 C++)\n'
                 f'path ratio = {path_cpp/path_ref:.3f}')
    ax.legend(loc='best', fontsize=10)

    # Panel 2: Kabsch-aligned
    ax = fig.add_subplot(gs[1])
    ax.plot(xr, yr, lw=2.5, color='k', label=f'GLIM ref   path={path_ref:.1f}m')
    ax.plot(full_aligned[:, 0], full_aligned[:, 1], lw=1.6, color='C2',
            label=f'leg odom aligned   path={path_cpp:.1f}m')
    ax.scatter([0], [0], c='g', s=80, zorder=5, label='start (ref)')
    ax.scatter([full_aligned[0, 0]], [full_aligned[0, 1]], c='C2', s=60, zorder=5, marker='o',
               alpha=0.8, label='start (leg, aligned)')
    ax.scatter([xr[-1]], [yr[-1]], c='k', marker='x', s=80, zorder=5, label='ref end')
    ax.scatter([full_aligned[-1, 0]], [full_aligned[-1, 1]], c='C2', marker='x', s=80, zorder=5,
               label='leg end (aligned)')
    ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f'After Kabsch (2D rigid alignment)\n'
                 f'yaw offset = {yaw_off:+.2f}°    RMSE = {rmse:.2f} m    path ratio = {path_cpp/path_ref:.3f}×')
    ax.legend(loc='best', fontsize=10)

    fig.suptitle('Stage 1 leg odometry (C++ fk_only_node) vs GLIM reference — ' \
                 'rosbag2_2026_04_07-16_12_13, 184 s, 103 office loop',
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
