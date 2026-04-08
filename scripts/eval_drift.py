#!/usr/bin/env python3
"""从 dump 的 CSV 算 loop-closure drift（适用于起终点重合的场景）。

用法:
  python3 eval_drift.py [csv_path]
默认输入: /tmp/leg_odom.csv
"""
import csv
import math
import sys


def main():
    path_csv = sys.argv[1] if len(sys.argv) > 1 else '/tmp/leg_odom.csv'

    xs, ys, zs, ts = [], [], [], []
    with open(path_csv) as f:
        r = csv.reader(f)
        next(r)  # header
        for row in r:
            ts.append(float(row[0]))
            xs.append(float(row[1]))
            ys.append(float(row[2]))
            zs.append(float(row[3]))

    n = len(xs)
    print(f"Messages: {n}")
    if n < 2:
        sys.exit(1)

    duration = ts[-1] - ts[0]
    print(f"Duration: {duration:.1f}s, rate: {n/duration:.1f} Hz")

    path_xy = 0.0
    for i in range(1, n):
        dx = xs[i] - xs[i-1]
        dy = ys[i] - ys[i-1]
        path_xy += math.sqrt(dx*dx + dy*dy)

    end_disp_xy = math.sqrt((xs[-1]-xs[0])**2 + (ys[-1]-ys[0])**2)
    end_disp_z = zs[-1] - zs[0]

    print()
    print(f"Total XY path: {path_xy:.2f} m")
    print(f"Start: ({xs[0]:.3f}, {ys[0]:.3f}, {zs[0]:.3f})")
    print(f"End:   ({xs[-1]:.3f}, {ys[-1]:.3f}, {zs[-1]:.3f})")
    print(f"|end - start| XY: {end_disp_xy:.3f} m")
    print(f"End - start  Z:   {end_disp_z:+.3f} m")
    print(f"XY drift / path:  {end_disp_xy/max(path_xy,1e-3)*100:.2f}%")
    print()
    print(f"X range: [{min(xs):.2f}, {max(xs):.2f}] = {max(xs)-min(xs):.2f}m")
    print(f"Y range: [{min(ys):.2f}, {max(ys):.2f}] = {max(ys)-min(ys):.2f}m")
    print(f"Z range: [{min(zs):.3f}, {max(zs):.3f}] = {max(zs)-min(zs):.3f}m")


if __name__ == '__main__':
    main()
