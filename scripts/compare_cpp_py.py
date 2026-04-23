#!/usr/bin/env python3
"""Stage 1 Go/No-go check: compare C++ fk_only_node CSV vs Python fk_only_odometry.py CSV."""
import sys, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load_cpp(path):
    """t,x,y,z,qx,qy,qz,qw"""
    d = np.genfromtxt(path, delimiter=',', names=True)
    t = d['t']; x = d['x']; y = d['y']
    # yaw from quaternion z component (assuming planar)
    qw = d['qw']; qz = d['qz']
    yaw = 2.0 * np.arctan2(qz, qw)
    return t, x, y, yaw


def load_py(path):
    """t_abs,x,y,z,yaw"""
    d = np.genfromtxt(path, delimiter=',', names=True)
    return d['t_abs'], d['x'], d['y'], d['yaw']


def main():
    cpp_path = sys.argv[1]
    py_path  = sys.argv[2]
    out      = sys.argv[3] if len(sys.argv) > 3 else '/tmp/cpp_vs_py.png'
    t_lo = float(sys.argv[4]) if len(sys.argv) > 4 else 1775549558.0
    t_hi = float(sys.argv[5]) if len(sys.argv) > 5 else 1775549742.0

    tc, xc, yc, yawc = load_cpp(cpp_path)
    tp, xp, yp, yawp = load_py(py_path)

    # clip both to [t_lo, t_hi]
    mc = (tc >= t_lo) & (tc <= t_hi)
    mp = (tp >= t_lo) & (tp <= t_hi)
    tc, xc, yc, yawc = tc[mc], xc[mc], yc[mc], yawc[mc]
    tp, xp, yp, yawp = tp[mp], xp[mp], yp[mp], yawp[mp]

    # interpolate Python onto C++ timestamps
    xp_i = np.interp(tc, tp, xp)
    yp_i = np.interp(tc, tp, yp)
    yawp_i = np.interp(tc, tp, np.unwrap(yawp))
    yawc_u = np.unwrap(yawc)

    dx = xc - xp_i
    dy = yc - yp_i
    d  = np.hypot(dx, dy)
    dyaw = np.abs(yawc_u - yawp_i)

    path_cpp = float(np.sum(np.hypot(np.diff(xc), np.diff(yc))))
    path_py  = float(np.sum(np.hypot(np.diff(xp), np.diff(yp))))
    loop_cpp = float(np.hypot(xc[-1] - xc[0], yc[-1] - yc[0]))
    loop_py  = float(np.hypot(xp[-1] - xp[0], yp[-1] - yp[0]))

    print(f'clip window [{t_lo}, {t_hi}]')
    print(f'C++ samples = {len(tc)}   Python samples = {len(tp)}')
    print(f'C++ path = {path_cpp:.3f} m   loop = {loop_cpp:.3f} m')
    print(f'Py  path = {path_py:.3f} m   loop = {loop_py:.3f} m')
    print(f'path ratio (C++/Py) = {path_cpp/path_py:.4f}')
    print(f'final pos diff (C++ end - Py end): '
          f'dx={xc[-1]-xp_i[-1]:+.4f} m  dy={yc[-1]-yp_i[-1]:+.4f} m')
    print(f'per-sample |Δxy|: mean={d.mean():.4f}  p50={np.median(d):.4f}  '
          f'p90={np.percentile(d,90):.4f}  max={d.max():.4f}  m')
    print(f'per-sample Δyaw:  mean={dyaw.mean()*1e3:.3f}  max={dyaw.max()*1e3:.3f}  mrad')

    # go/no-go
    ok_pos = d.max() < 0.01
    ok_path = abs(path_cpp/path_py - 1.0) < 0.02
    print(f'\nGo/No-go:')
    print(f'  max |Δxy| < 0.01 m ?         {"YES" if ok_pos else "NO"} ({d.max():.4f} m)')
    print(f'  |path ratio - 1| < 2 % ?     {"YES" if ok_path else "NO"} ({(path_cpp/path_py-1)*100:+.2f}%)')
    overall = ok_pos and ok_path
    print(f'  overall Stage 1 passes ?     {"YES" if overall else "NO"}')

    # plot
    fig, axes = plt.subplots(2, 2, figsize=(16, 9))
    axes[0,0].plot(xp, yp, 'k-', lw=2, label=f'Python  path={path_py:.2f}m')
    axes[0,0].plot(xc, yc, 'r--', lw=1, label=f'C++    path={path_cpp:.2f}m')
    axes[0,0].set_aspect('equal'); axes[0,0].grid(alpha=0.3); axes[0,0].legend()
    axes[0,0].set_title('XY overlay'); axes[0,0].set_xlabel('x'); axes[0,0].set_ylabel('y')

    t0 = tc[0]
    axes[0,1].plot(tc - t0, d, lw=0.8, color='C3')
    axes[0,1].set_title(f'|Δxy| (C++ − Py) max={d.max():.4f}m  mean={d.mean():.4f}m')
    axes[0,1].set_xlabel('t [s]'); axes[0,1].set_ylabel('|Δxy| [m]')
    axes[0,1].grid(alpha=0.3)
    axes[0,1].axhline(0.01, color='k', ls='--', lw=0.8, label='go/no-go 0.01m')
    axes[0,1].legend()

    axes[1,0].plot(tc - t0, xc - xp_i, label='Δx', lw=0.7)
    axes[1,0].plot(tc - t0, yc - yp_i, label='Δy', lw=0.7)
    axes[1,0].set_title('Δx, Δy vs time')
    axes[1,0].set_xlabel('t [s]'); axes[1,0].grid(alpha=0.3); axes[1,0].legend()

    axes[1,1].plot(tc - t0, dyaw * 1e3, color='C4', lw=0.7)
    axes[1,1].set_title(f'|Δyaw| mean={dyaw.mean()*1e3:.2f} mrad  max={dyaw.max()*1e3:.2f} mrad')
    axes[1,1].set_xlabel('t [s]'); axes[1,1].set_ylabel('|Δyaw| [mrad]')
    axes[1,1].grid(alpha=0.3)

    fig.suptitle(f'Stage 1 — C++ fk_only_node vs Python fk_only_odometry.py\n'
                 f'path_cpp/path_py={path_cpp/path_py:.4f}  max|Δxy|={d.max():.4f}m',
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f'\nwrote {out}')

    sys.exit(0 if overall else 1)


if __name__ == '__main__':
    main()
