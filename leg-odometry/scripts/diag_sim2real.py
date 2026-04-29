#!/usr/bin/env python3
"""Sim-to-Real 诊断脚本：从一个 bag 出发，验证 ekf_params.yaml 里几条
sim 假设在实机上是否还成立。

检查项 (对应 memory project_leg_odometry.md 三条假设)：
  1. IMU 噪声 — 在自动检测出的"静止段"上算 accel/gyro mean & std，
     与 ekf_params.yaml 里的 accel_noise / gyro_noise 对比
  2. 接触检测命中率 — 用 ekf_params.yaml 里的 effort_threshold/hysteresis
     回放 LJPITCH/RJPITCH effort，统计左右脚 contact / swing / both / none 的
     时间占比；任何一项 >95% 都是"detector 失效"的危险信号
  3. 关节零位 — 在第一段静止窗口内对每个关节取均值，列出 |mean|>0.1 rad
     的可疑关节（潜在零位偏移 / 方向反）

不需要启动节点、不需要 rebuild。只读 bag。

用法:
  python3 diag_sim2real.py /path/to/bag_dir
  python3 diag_sim2real.py /path/to/bag_dir --static-window 2.0 --gyro-thr 0.02
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
PKG_DIR = SCRIPT_DIR.parent

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: E402
import rclpy.serialization  # noqa: E402
from sensor_msgs.msg import JointState, Imu  # noqa: E402


def read_bag(bag_dir):
    storage = StorageOptions(uri=bag_dir, storage_id='sqlite3')
    conv = ConverterOptions(input_serialization_format='cdr',
                            output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage, conv)
    msgs = []
    while reader.has_next():
        msgs.append(reader.read_next())
    return msgs


def deserialize(topic, data):
    if topic == '/joint_states':
        return rclpy.serialization.deserialize_message(data, JointState)
    if topic == '/imu':
        return rclpy.serialization.deserialize_message(data, Imu)
    return None


# ---------------- helpers ----------------

def load_ekf_params():
    p = PKG_DIR / 'config' / 'ekf_params.yaml'
    with open(p) as f:
        return yaml.safe_load(f)


def load_joint_mapping():
    p = PKG_DIR / 'config' / 'joint_mapping.yaml'
    with open(p) as f:
        return yaml.safe_load(f)


def fmt_vec(v, prec=4):
    return '[' + ', '.join(f'{x:+.{prec}f}' for x in v) + ']'


def banner(s):
    print('\n' + '=' * 72)
    print(s)
    print('=' * 72)


def _runs(mask, times):
    """对 boolean mask 找连续 True 段，返回 [(t_start, t_end, dur_s), ...]"""
    n = len(mask)
    out = []
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and mask[j + 1]:
            j += 1
        out.append((times[i], times[j], times[j] - times[i]))
        i = j + 1
    return out


# 桶定义: (label, [low, high) seconds)
_DUR_BUCKETS = [
    ('<10ms   单帧抖动',  0.000, 0.010),
    ('10-100ms 短抖动',   0.010, 0.100),
    ('100-500ms 单步级',  0.100, 0.500),
    ('0.5-2s 多步空窗',   0.500, 2.000),
    ('>2s 大段断流',      2.000, float('inf')),
]


def _bucket_durations(runs):
    counts = [0] * len(_DUR_BUCKETS)
    totals = [0.0] * len(_DUR_BUCKETS)
    for _, _, dur in runs:
        for i, (_, lo, hi) in enumerate(_DUR_BUCKETS):
            if lo <= dur < hi:
                counts[i] += 1
                totals[i] += dur
                break
    return counts, totals


def _plot_contact(times, eff_l, eff_r, cl, cr, thr, hys, out_path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    t0 = times[0]
    t = times - t0
    abs_l = np.abs(eff_l)
    abs_r = np.abs(eff_r)
    both_swing = ~cl & ~cr

    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)

    axes[0].plot(t, abs_l, lw=0.4, color='tab:blue')
    axes[0].axhline(thr + hys, ls='--', color='gray', lw=0.7,
                    label=f'up={thr+hys}')
    axes[0].axhline(thr - hys, ls='--', color='gray', lw=0.7,
                    label=f'dn={thr-hys}')
    axes[0].set_ylabel('LEFT |effort| (Nm)')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, abs_r, lw=0.4, color='tab:orange')
    axes[1].axhline(thr + hys, ls='--', color='gray', lw=0.7)
    axes[1].axhline(thr - hys, ls='--', color='gray', lw=0.7)
    axes[1].set_ylabel('RIGHT |effort| (Nm)')
    axes[1].grid(True, alpha=0.3)

    axes[2].fill_between(t, 0, cl.astype(float), step='post', alpha=0.6,
                         color='tab:blue', label='L contact')
    axes[2].fill_between(t, 1, 1 + cr.astype(float), step='post', alpha=0.6,
                         color='tab:orange', label='R contact')
    axes[2].fill_between(t, 2, 2 + both_swing.astype(float), step='post',
                         alpha=0.8, color='red', label='BOTH swing (ZUPT off)')
    axes[2].set_yticks([0.5, 1.5, 2.5])
    axes[2].set_yticklabels(['L', 'R', 'swing'])
    axes[2].set_xlabel('time (s)')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close(fig)
    return out_path


# ---------------- 1. IMU noise on static segments ----------------

def find_static_segments(times, gyro_norms, gyro_thr, min_dur):
    """返回 [(start_idx, end_idx, duration), ...]，gyro 范数持续低于阈值。"""
    n = len(times)
    if n == 0:
        return []
    segs = []
    i = 0
    while i < n:
        if gyro_norms[i] >= gyro_thr:
            i += 1
            continue
        j = i
        while j + 1 < n and gyro_norms[j + 1] < gyro_thr:
            j += 1
        dur = times[j] - times[i]
        if dur >= min_dur:
            segs.append((i, j, dur))
        i = j + 1
    return segs


def diag_imu(messages, ekf_cfg, gyro_thr, min_dur):
    banner('1. IMU 噪声 (静止段)')

    times, accels, gyros = [], [], []
    for topic, data, _ in messages:
        if topic != '/imu':
            continue
        msg = deserialize(topic, data)
        if msg is None:
            continue
        h = msg.header.stamp
        times.append(h.sec + h.nanosec * 1e-9)
        accels.append([msg.linear_acceleration.x,
                       msg.linear_acceleration.y,
                       msg.linear_acceleration.z])
        gyros.append([msg.angular_velocity.x,
                      msg.angular_velocity.y,
                      msg.angular_velocity.z])

    if not times:
        print('  /imu 无数据，跳过')
        return None

    times = np.asarray(times)
    accels = np.asarray(accels)
    gyros = np.asarray(gyros)
    gyro_norms = np.linalg.norm(gyros, axis=1)

    rate = (len(times) - 1) / (times[-1] - times[0]) if times[-1] > times[0] else 0
    print(f'  /imu samples: {len(times)}  duration: {times[-1] - times[0]:.1f}s  rate≈{rate:.0f}Hz')

    segs = find_static_segments(times, gyro_norms, gyro_thr, min_dur)
    print(f'  静止段: {len(segs)} 个 (gyro_norm<{gyro_thr}, ≥{min_dur}s)')
    if not segs:
        print('  ⚠ 没找到静止段，无法估 IMU 噪声。')
        print(f'  gyro_norm 中位数={np.median(gyro_norms):.4f}, p10={np.percentile(gyro_norms, 10):.4f}')
        return None

    # 拼接所有静止段
    static_mask = np.zeros(len(times), dtype=bool)
    for i, j, dur in segs:
        static_mask[i:j + 1] = True
        print(f'    [{times[i] - times[0]:7.2f}, {times[j] - times[0]:7.2f}]s  dur={dur:.2f}s')

    a_static = accels[static_mask]
    g_static = gyros[static_mask]

    a_mean = a_static.mean(axis=0)
    a_std = a_static.std(axis=0)
    g_mean = g_static.mean(axis=0)
    g_std = g_static.std(axis=0)
    a_norm_mean = np.linalg.norm(a_static, axis=1).mean()

    print(f'\n  accel mean (m/s^2):  {fmt_vec(a_mean)}')
    print(f'  accel std  (m/s^2):  {fmt_vec(a_std)}')
    print(f'  |accel| mean:        {a_norm_mean:.4f}  (重力应≈9.81)')
    print(f'  gyro  mean (rad/s):  {fmt_vec(g_mean, 5)}')
    print(f'  gyro  std  (rad/s):  {fmt_vec(g_std, 5)}')

    # 与 yaml 假设比较
    sim_a = ekf_cfg['ekf']['accel_noise']
    sim_g = ekf_cfg['ekf']['gyro_noise']
    print(f'\n  yaml 假设: accel_noise={sim_a}  gyro_noise={sim_g}')

    a_max = a_std.max()
    g_max = g_std.max()
    a_ratio = a_max / sim_a
    g_ratio = g_max / sim_g
    print(f'  实测 accel std max={a_max:.4f}  比 sim x{a_ratio:.2f}')
    print(f'  实测 gyro  std max={g_max:.5f}  比 sim x{g_ratio:.2f}')

    if a_ratio > 1.5 or g_ratio > 1.5:
        print('  ❌ IMU 噪声明显高于 yaml — accel_noise/gyro_noise 需要调高')
    elif a_ratio < 0.5 and g_ratio < 0.5:
        print('  ⚠ IMU 噪声远低于 yaml — yaml 太保守，可适度调小 (信任度可提高)')
    else:
        print('  ✓ IMU 噪声大致与 yaml 一致')

    if abs(a_norm_mean - 9.81) > 0.3:
        print(f'  ⚠ |accel| 均值 {a_norm_mean:.3f} 偏离 9.81 — IMU 标定/偏置可疑')

    return {
        'static_segments': segs,
        'static_mask': static_mask,
        'times': times,
    }


# ---------------- 2. 接触检测命中率 ----------------

class ContactDetectorPy:
    """与 C++ ContactDetector 行为一致：双阈值滞后。"""
    def __init__(self, threshold, hysteresis):
        self.up = threshold + hysteresis
        self.dn = threshold - hysteresis
        self.left = False
        self.right = False

    def update(self, eff_l, eff_r):
        if abs(eff_l) > self.up:
            self.left = True
        elif abs(eff_l) < self.dn:
            self.left = False
        if abs(eff_r) > self.up:
            self.right = True
        elif abs(eff_r) < self.dn:
            self.right = False
        return self.left, self.right


def diag_contact(messages, ekf_cfg):
    banner('2. 接触检测命中率')

    cc = ekf_cfg['contact']
    thr = cc['effort_threshold']
    hys = cc['hysteresis']
    name_l = cc['effort_joint_left']
    name_r = cc['effort_joint_right']
    print(f'  threshold={thr}  hysteresis={hys}  joints=({name_l}, {name_r})')

    eff_l, eff_r, times = [], [], []
    for topic, data, _ in messages:
        if topic != '/joint_states':
            continue
        msg = deserialize(topic, data)
        if msg is None or len(msg.effort) == 0:
            continue
        h = msg.header.stamp
        t = h.sec + h.nanosec * 1e-9
        el = er = None
        for i, n in enumerate(msg.name):
            if n == name_l and i < len(msg.effort):
                el = msg.effort[i]
            elif n == name_r and i < len(msg.effort):
                er = msg.effort[i]
        if el is None or er is None:
            continue
        times.append(t)
        eff_l.append(el)
        eff_r.append(er)

    if not eff_l:
        print(f'  ⚠ 没在 /joint_states 找到 {name_l}/{name_r} 的 effort')
        return None

    eff_l = np.asarray(eff_l)
    eff_r = np.asarray(eff_r)
    abs_l = np.abs(eff_l)
    abs_r = np.abs(eff_r)

    def stats(name, x):
        print(f'  {name} |effort|:  '
              f'min={x.min():6.2f}  '
              f'p10={np.percentile(x, 10):6.2f}  '
              f'p50={np.median(x):6.2f}  '
              f'p90={np.percentile(x, 90):6.2f}  '
              f'max={x.max():6.2f}')

    stats('LEFT ', abs_l)
    stats('RIGHT', abs_r)

    # 回放双阈值滞后
    det = ContactDetectorPy(thr, hys)
    states = []
    for el, er in zip(eff_l, eff_r):
        states.append(det.update(el, er))
    cl = np.array([s[0] for s in states])
    cr = np.array([s[1] for s in states])

    pct_l = cl.mean() * 100
    pct_r = cr.mean() * 100
    pct_double = (cl & cr).mean() * 100
    pct_swing = (~cl & ~cr).mean() * 100
    pct_single = 100 - pct_double - pct_swing

    print(f'\n  时间占比 (回放当前阈值):')
    print(f'    LEFT contact:    {pct_l:5.1f}%')
    print(f'    RIGHT contact:   {pct_r:5.1f}%')
    print(f'    double support:  {pct_double:5.1f}%')
    print(f'    single support:  {pct_single:5.1f}%')
    print(f'    fully swing:     {pct_swing:5.1f}%')

    # 判断
    bad = False
    if pct_double > 95:
        print('  ❌ 几乎一直双脚接触 — ZUPT 永远生效，但前进时也会被锁住 → 速度被压制')
        bad = True
    if pct_swing > 50:
        print('  ❌ swing 占比过高 — ZUPT 大段失效，速度无约束 → 漂移放大')
        bad = True
    if pct_l > 99 or pct_r > 99:
        print('  ❌ 单脚常驻接触 — detector 没有切换，步态无法被识别')
        bad = True
    if pct_l < 5 or pct_r < 5:
        print('  ❌ 单脚几乎从未触发 — 阈值过高 / 关节力矩没传到 effort')
        bad = True
    if not bad:
        print('  ✓ 接触命中率看起来合理 (有交替支撑)')

    # 给一组建议阈值：median + safety margin
    suggested_thr = float(np.percentile(np.concatenate([abs_l, abs_r]), 60))
    print(f'\n  → 经验建议: 试 effort_threshold≈{suggested_thr:.1f} '
          f'(全体 |effort| 的 p60)')

    # ---- 连续 swing 段时长分布 (定位抖动 vs 真实信号断流) ----
    times_arr = np.asarray(times)
    both_swing = (~cl) & (~cr)
    swing_runs = _runs(both_swing, times_arr)
    counts, totals = _bucket_durations(swing_runs)
    total_swing_s = float(np.sum([d for _, _, d in swing_runs])) if swing_runs else 0.0

    print(f'\n  连续"双脚 swing"段: 共 {len(swing_runs)} 段, 累计 {total_swing_s:.1f}s')
    if total_swing_s > 0:
        for (label, _, _), c, t in zip(_DUR_BUCKETS, counts, totals):
            if c == 0:
                continue
            pct = t / total_swing_s * 100
            print(f'    {label:22s}  count={c:5d}  total={t:6.1f}s  ({pct:5.1f}% of swing time)')

        # 定性判断: 抖动 (前 2 桶) vs 断流 (后 2 桶)
        chatter_t = totals[0] + totals[1]
        blackout_t = totals[3] + totals[4]
        chatter_pct = chatter_t / total_swing_s * 100
        blackout_pct = blackout_t / total_swing_s * 100
        print(f'\n  按时间累计:  抖动 {chatter_pct:.0f}%  vs  断流 {blackout_pct:.0f}%')
        if blackout_pct > 60:
            print('  ❌ 断流为主 → 不是 detector 抖动，是 effort 信号本身大段为 0')
            print('     方向: 换接触检测信号源 (膝/髋关节力矩 / IMU 高频 / 关节速度)，')
            print('           或者只在 effort==0 且 joint velocity≈0 时才信任为 swing')
        elif chatter_pct > 60:
            print('  ⚠ 抖动为主 → detector 在阈值附近反复跳变')
            print('     方向: 加大 yaml contact.hysteresis (现在 1.0)，比如 2.0-3.0')
        else:
            print('  ⚠ 抖动+断流混合，需要先看 PNG 时间序列定位主要发生时段')

    # ---- PNG ----
    png_path = '/tmp/diag_sim2real_contact.png'
    saved = _plot_contact(times_arr, eff_l, eff_r, cl, cr, thr, hys, png_path)
    if saved:
        print(f'\n  PNG: {saved}')
    else:
        print('\n  (matplotlib 不可用，跳过 PNG)')

    return {
        'eff_l': eff_l,
        'eff_r': eff_r,
        'times': times_arr,
    }


# ---------------- 3. 关节零位 ----------------

def diag_joint_zeros(messages, imu_diag):
    banner('3. 关节零位 (静止段均值)')

    if imu_diag is None or not imu_diag['static_segments']:
        print('  没有 IMU 静止段，跳过 (joint zero 检查依赖静止时刻)')
        return None

    seg = imu_diag['static_segments'][0]
    t0 = imu_diag['times'][seg[0]]
    t1 = imu_diag['times'][seg[1]]
    print(f'  使用第一段静止窗口: [{t0:.2f}, {t1:.2f}]s')

    jm = load_joint_mapping()
    bag2urdf = jm['joint_mapping']
    of = jm.get('joint_offset', {})

    samples = {}  # bag joint name -> list of pos
    for topic, data, _ in messages:
        if topic != '/joint_states':
            continue
        msg = deserialize(topic, data)
        if msg is None:
            continue
        h = msg.header.stamp
        t = h.sec + h.nanosec * 1e-9
        if t < t0 or t > t1:
            continue
        for i, n in enumerate(msg.name):
            if i < len(msg.position):
                samples.setdefault(n, []).append(msg.position[i])

    if not samples:
        print('  ⚠ 静止窗口内没拿到 /joint_states，跳过')
        return None

    # 只挑跟 leg odom 相关的 12 个映射后的腿部关节
    leg_keys = [k for k in bag2urdf.keys()
                if k.startswith('LJ') or k.startswith('RJ')]

    print(f'  {"bag joint":12s}  {"mean":>9s}  {"std":>8s}  {"yaml_offset":>11s}  flag')
    flagged = []
    for k in leg_keys:
        if k not in samples:
            continue
        vals = np.asarray(samples[k])
        m = vals.mean()
        s = vals.std()
        off = of.get(k, 0.0)
        flag = ''
        if abs(m) > 0.1:
            flag = '⚠ |mean|>0.1 rad'
            flagged.append(k)
        print(f'  {k:12s}  {m:+9.4f}  {s:8.5f}  {off:+11.4f}  {flag}')

    if flagged:
        print(f'\n  ⚠ {len(flagged)} 个关节在静止时偏离 0 较多，'
              f'有可能需要在 joint_mapping.yaml 设 joint_offset')
        print(f'  注意：人形机器人 standing pose 本来就可能不全 0，'
              f'需要先确认这一段确实是"中性站姿"再下结论')
    else:
        print('  ✓ 所有腿部关节静止时均接近 0')

    return {'flagged': flagged}


# ---------------- main ----------------

def main():
    ap = argparse.ArgumentParser(description='Sim-to-Real 诊断')
    ap.add_argument('bag', help='rosbag2 目录路径')
    ap.add_argument('--gyro-thr', type=float, default=0.02,
                    help='静止判定阈值 (gyro norm rad/s, 默认 0.02)')
    ap.add_argument('--static-window', type=float, default=2.0,
                    help='最小静止段长度 (s, 默认 2.0)')
    args = ap.parse_args()

    if not os.path.isdir(args.bag):
        print(f'bag 路径不存在: {args.bag}')
        sys.exit(1)

    print(f'读取 bag: {args.bag}')
    messages = read_bag(args.bag)
    print(f'  共 {len(messages)} 条消息')

    ekf_cfg = load_ekf_params()

    imu_diag = diag_imu(messages, ekf_cfg, args.gyro_thr, args.static_window)
    diag_contact(messages, ekf_cfg)
    diag_joint_zeros(messages, imu_diag)

    banner('完成')


if __name__ == '__main__':
    main()
