#!/usr/bin/env bash
# Run wheel_only_node against a recorded bag and produce a CSV trajectory
# plus a quick path / closure-error plot. Self-contained: caller just sets
# BAG_DIR if the default (w2 dataset) doesn't apply.
#
# Tested layout: bag has /robot/wheel_status (navigation_interface/WheelStatus)
# + /rslidar_imu_data + /rslidar_points.

set -euo pipefail

BAG_DIR="${BAG_DIR:-$HOME/Documents/Datasets/w2/rosbag2_2026_04_28-17_06_53}"
OUT_DIR="${OUT_DIR:-/tmp/wheel_odom_test}"
WHEELBASE="${WHEELBASE:-0.6}"
TRACK="${TRACK:-0.5}"
YAW_SOURCE="${YAW_SOURCE:-gyro}"          # gyro or ls
RATE="${RATE:-5}"

mkdir -p "$OUT_DIR"
cd "$HOME/rtabmap_ws"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

# Clean prior run.
pkill -9 -f wheel_only_node 2>/dev/null || true
pkill -9 -f "ros2 bag play"  2>/dev/null || true
sleep 1
rm -f "$OUT_DIR"/{run.csv,node.log,bag.log}

echo ">> launching wheel_only_node (L=$WHEELBASE W=$TRACK yaw=$YAW_SOURCE)"
ros2 run wheel_odometry wheel_only_node --ros-args \
    -p wheelbase:="$WHEELBASE" \
    -p track:="$TRACK" \
    -p yaw_source:="$YAW_SOURCE" \
    -p chassis_topic:=/robot/wheel_status \
    -p imu_topic:=/rslidar_imu_data \
    -p odom_topic:=/wheel_odometry \
    -p diag_csv_path:="$OUT_DIR/run.csv" \
    > "$OUT_DIR/node.log" 2>&1 &
NODE_PID=$!
sleep 2

echo ">> playing bag at ${RATE}x"
ros2 bag play "$BAG_DIR" --rate "$RATE" > "$OUT_DIR/bag.log" 2>&1
echo ">> bag done"

kill -9 "$NODE_PID" 2>/dev/null || true
wait 2>/dev/null || true

echo ">> stats:"
python3 - "$OUT_DIR/run.csv" <<'PY'
import csv, math, sys
xs, ys = [], []
with open(sys.argv[1]) as f:
    r = csv.reader(f); next(r)
    for row in r:
        try:
            xs.append(float(row[1])); ys.append(float(row[2]))
        except: pass
n = len(xs)
path = sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(n-1))
disp = math.hypot(xs[-1]-xs[0], ys[-1]-ys[0])
print(f"   rows={n}  path={path:.1f}m  disp={disp:.1f}m  "
      f"closure_err={disp/path*100:.2f}%  end=({xs[-1]:+.1f},{ys[-1]:+.1f})")
PY

echo ">> done. CSV: $OUT_DIR/run.csv  (header in line 1)"
