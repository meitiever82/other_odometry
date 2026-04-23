#!/usr/bin/env python3
"""Compare raw-threshold contact detection vs bias-compensated contact detection.

Reads the leg_joints.csv produced by visualize_leg_joints.py, estimates a slowly-
varying background (rolling median) on LJPITCH / RJPITCH effort, subtracts it,
and re-applies a unified ±THR threshold. Reports stance ratios for both schemes
and plots them side-by-side.

Usage:
    python3 contact_bias_diagnose.py <viz_clip_dir>
"""
import argparse
import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# current split thresholds (ekf_params.yaml)
THR_RAW_L = 5.0
THR_RAW_R = 12.0
# unified threshold after bias removal
THR_UNIFIED = 5.0
# rolling-median window length in seconds (≥ few gait cycles)
BG_WIN_SEC = 5.0


def rolling_median(x, win):
    """Centered rolling median; edges use shrinking window."""
    n = len(x)
    out = np.empty(n)
    half = win // 2
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = np.nanmedian(x[lo:hi])
    return out


def rolling_median_fast(x, win):
    """Prefer scipy if available — O(n log w) instead of O(n*w)."""
    try:
        from scipy.ndimage import median_filter
        return median_filter(x, size=win, mode='nearest')
    except Exception:
        return rolling_median(x, win)


def stance_ratio(mask):
    mask = mask[~np.isnan(mask)]
    return float(mask.mean()) * 100.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('viz_dir', help='dir containing leg_joints.csv from visualize_leg_joints.py')
    args = ap.parse_args()

    csv_path = os.path.join(args.viz_dir, 'leg_joints.csv')
    if not os.path.isfile(csv_path):
        sys.exit(f'{csv_path} not found')

    print(f'reading {csv_path} ...')
    data = np.genfromtxt(csv_path, delimiter=',', names=True)
    t = data['t'] - data['t'][0]
    eff_l = data['LJPITCH_eff']
    eff_r = data['RJPITCH_eff']

    dt = np.median(np.diff(t))
    win = max(3, int(round(BG_WIN_SEC / dt)))
    print(f'samples={len(t)}  dt={dt*1000:.2f}ms  rolling_win={win} ({win*dt:.1f}s)')

    bg_l = rolling_median_fast(eff_l, win)
    bg_r = rolling_median_fast(eff_r, win)
    det_l = eff_l - bg_l
    det_r = eff_r - bg_r

    # raw contact (current scheme)
    stance_raw_l = np.abs(eff_l) > THR_RAW_L
    stance_raw_r = np.abs(eff_r) > THR_RAW_R
    # bias-compensated contact
    stance_det_l = np.abs(det_l) > THR_UNIFIED
    stance_det_r = np.abs(det_r) > THR_UNIFIED

    # also-stance: both feet on ground (double support)
    both_raw = stance_raw_l & stance_raw_r
    both_det = stance_det_l & stance_det_r

    print('\n=== stance ratio ===')
    print(f'                  LEFT       RIGHT      BOTH')
    print(f'raw  (±5 / ±12):  {stance_ratio(stance_raw_l):5.1f}%     '
          f'{stance_ratio(stance_raw_r):5.1f}%     {stance_ratio(both_raw):5.1f}%')
    print(f'detrended ±{THR_UNIFIED}:    {stance_ratio(stance_det_l):5.1f}%     '
          f'{stance_ratio(stance_det_r):5.1f}%     {stance_ratio(both_det):5.1f}%')

    print('\n=== background (rolling median) stats ===')
    print(f'LJPITCH bg:  mean={np.mean(bg_l):+.2f} Nm  std={np.std(bg_l):.2f}  '
          f'min={np.min(bg_l):+.2f}  max={np.max(bg_l):+.2f}')
    print(f'RJPITCH bg:  mean={np.mean(bg_r):+.2f} Nm  std={np.std(bg_r):.2f}  '
          f'min={np.min(bg_r):+.2f}  max={np.max(bg_r):+.2f}')

    # ---- plot ----
    fig, axes = plt.subplots(2, 2, figsize=(20, 8), sharex=True)

    def draw(ax_raw, ax_det, side, raw, bg, det, thr_raw, mask_raw, mask_det):
        ax_raw.plot(t, raw, lw=0.5, color='C0', label=f'{side} effort')
        ax_raw.plot(t, bg,  lw=1.3, color='k',  label=f'rolling median ({BG_WIN_SEC:.0f}s)')
        ax_raw.axhline(+thr_raw, color='r', ls='--', lw=0.8, label=f'±{thr_raw}')
        ax_raw.axhline(-thr_raw, color='r', ls='--', lw=0.8)
        ax2 = ax_raw.twinx()
        ax2.fill_between(t, 0, mask_raw.astype(float), step='post', alpha=0.15, color='g')
        ax2.set_ylim(0, 1.05); ax2.set_yticks([0,1]); ax2.set_yticklabels(['sw','st'], fontsize=8)
        ax_raw.set_title(f'{side}  RAW threshold ±{thr_raw} Nm   '
                         f'stance={stance_ratio(mask_raw):.1f}%')
        ax_raw.set_ylabel('effort [Nm]'); ax_raw.grid(alpha=0.3)
        ax_raw.legend(loc='upper right', fontsize=8)

        ax_det.plot(t, det, lw=0.5, color='C0', label=f'{side} effort − bg')
        ax_det.axhline(+THR_UNIFIED, color='r', ls='--', lw=0.8, label=f'±{THR_UNIFIED}')
        ax_det.axhline(-THR_UNIFIED, color='r', ls='--', lw=0.8)
        ax3 = ax_det.twinx()
        ax3.fill_between(t, 0, mask_det.astype(float), step='post', alpha=0.15, color='g')
        ax3.set_ylim(0, 1.05); ax3.set_yticks([0,1]); ax3.set_yticklabels(['sw','st'], fontsize=8)
        ax_det.set_title(f'{side}  DETRENDED threshold ±{THR_UNIFIED} Nm   '
                         f'stance={stance_ratio(mask_det):.1f}%')
        ax_det.set_ylabel('effort − bg [Nm]'); ax_det.grid(alpha=0.3)
        ax_det.legend(loc='upper right', fontsize=8)

    draw(axes[0, 0], axes[0, 1], 'LEFT LJPITCH',
         eff_l, bg_l, det_l, THR_RAW_L, stance_raw_l, stance_det_l)
    draw(axes[1, 0], axes[1, 1], 'RIGHT RJPITCH',
         eff_r, bg_r, det_r, THR_RAW_R, stance_raw_r, stance_det_r)
    axes[1, 0].set_xlabel('t [s]'); axes[1, 1].set_xlabel('t [s]')

    fig.suptitle('Contact detection: raw threshold (asymmetric) vs bias-compensated', fontsize=13)
    fig.tight_layout()
    out_png = os.path.join(args.viz_dir, '06_contact_bias_diagnose.png')
    fig.savefig(out_png, dpi=110); plt.close(fig)
    print(f'\nwrote {out_png}')

    # zoomed view (middle 30 s) so individual gait cycles are visible
    t0, t1 = t[-1] * 0.4, t[-1] * 0.4 + 30
    mask = (t >= t0) & (t <= t1)
    fig, axes = plt.subplots(2, 2, figsize=(20, 7), sharex=True)
    for col, (title, yl, yr, thrl, thrr, maskl, maskr) in enumerate([
        ('RAW',       eff_l, eff_r, THR_RAW_L, THR_RAW_R, stance_raw_l, stance_raw_r),
        ('DETRENDED', det_l, det_r, THR_UNIFIED, THR_UNIFIED, stance_det_l, stance_det_r),
    ]):
        ax = axes[0, col]
        ax.plot(t[mask], yl[mask], lw=0.8, color='C0')
        ax.axhline(+thrl, color='r', ls='--', lw=0.8)
        ax.axhline(-thrl, color='r', ls='--', lw=0.8)
        ax2 = ax.twinx()
        ax2.fill_between(t[mask], 0, maskl[mask].astype(float), step='post', alpha=0.2, color='g')
        ax2.set_ylim(0, 1.05); ax2.set_yticks([])
        ax.set_title(f'LEFT  {title}  (±{thrl} Nm)'); ax.grid(alpha=0.3); ax.set_ylabel('Nm')

        ax = axes[1, col]
        ax.plot(t[mask], yr[mask], lw=0.8, color='C0')
        ax.axhline(+thrr, color='r', ls='--', lw=0.8)
        ax.axhline(-thrr, color='r', ls='--', lw=0.8)
        ax2 = ax.twinx()
        ax2.fill_between(t[mask], 0, maskr[mask].astype(float), step='post', alpha=0.2, color='g')
        ax2.set_ylim(0, 1.05); ax2.set_yticks([])
        ax.set_title(f'RIGHT {title}  (±{thrr} Nm)'); ax.grid(alpha=0.3); ax.set_ylabel('Nm')
        ax.set_xlabel('t [s]')

    fig.suptitle(f'Zoom {t0:.0f}-{t1:.0f}s: gait cycles, raw vs detrended contact', fontsize=13)
    fig.tight_layout()
    out_png = os.path.join(args.viz_dir, '07_contact_bias_zoom.png')
    fig.savefig(out_png, dpi=110); plt.close(fig)
    print(f'wrote {out_png}')


if __name__ == '__main__':
    main()
