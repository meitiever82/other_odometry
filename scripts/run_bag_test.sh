#!/bin/bash
# 跑一个 rosbag 评估 leg_odometry 精度（loop-closure 场景）。
#
# 用法:
#   ./run_bag_test.sh <bag_path> [rate] [output_dir]
# 例:
#   ./run_bag_test.sh ~/Documents/Datasets/CASBOT/leg/rosbag2_2026_04_07-16_12_13 3.0
#
# 输出:
#   <output_dir>/odom.csv     — 节点输出的位姿
#   <output_dir>/node.log     — 节点 stdout
#   drift 评估打印到 stdout

BAG="${1:?Usage: $0 <bag_path> [rate] [output_dir]}"
RATE="${2:-3.0}"
OUT="${3:-/tmp/leg_odom_test}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

mkdir -p "$OUT"
cd "$OUT"
rm -f node.log subscriber.log play.log odom.csv

source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/../../../install/setup.bash"

# Start node (background)
ros2 launch leg_odometry leg_odometry.launch.py > node.log 2>&1 &
LAUNCH_PID=$!
sleep 3

# Start CSV subscriber (background)
python3 "$SCRIPT_DIR/dump_leg_odom_csv.py" "$OUT/odom.csv" > subscriber.log 2>&1 &
SUB_PID=$!
sleep 1

# Play bag (foreground)
echo "playing $BAG at rate $RATE..."
ros2 bag play "$BAG" -r "$RATE" --topics /imu /joint_states > play.log 2>&1
echo "play done"

sleep 2  # let trailing messages arrive

# Cleanup
kill -INT  $SUB_PID    2>/dev/null; sleep 1
kill -KILL $SUB_PID    2>/dev/null
kill -INT  $LAUNCH_PID 2>/dev/null; sleep 1
kill -KILL $LAUNCH_PID 2>/dev/null
pkill -KILL -f "leg_odom_node\|robot_state_publisher" 2>/dev/null
echo "stopped"

ls -la odom.csv
echo "---"
python3 "$SCRIPT_DIR/eval_drift.py" "$OUT/odom.csv"
