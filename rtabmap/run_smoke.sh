#!/usr/bin/env bash
# Smoke test: rtabmap RGB-D with SuperPoint + LightGlue PyMatcher
# Usage: bash /tmp/run_smoke.sh [--gdb]
set -eo pipefail

BAG_DIR="$HOME/Documents/Datasets/geoscan/outdoor/20251203"
LOG_DIR="/tmp/smoke"
mkdir -p "$LOG_DIR"

USE_GDB=false
[[ "${1:-}" == "--gdb" ]] && USE_GDB=true

# ---------- environment ----------
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CONDA_ENV=$HOME/miniforge3/envs/rtabmap
export PYTHONPATH=$CONDA_ENV/lib/python3.10/site-packages
export LD_LIBRARY_PATH=$HOME/opt/libtorch/lib:$CONDA_ENV/lib:${LD_LIBRARY_PATH:-}
export LD_PRELOAD=$CONDA_ENV/lib/libcufile.so.0

source /opt/ros/humble/setup.bash
source $HOME/rtabmap_ws/install/setup.bash

# enable core dumps
ulimit -c unlimited

cleanup() {
  echo "Cleaning up..."
  jobs -p | xargs -r kill 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

# ---------- 0. kill leftover rtabmap processes ----------
echo ">>> Cleaning up old rtabmap processes..."
pkill -9 -f rgbd_odometry 2>/dev/null || true
pkill -9 -f "rtabmap_slam" 2>/dev/null || true
pkill -9 -f rtabmap_viz 2>/dev/null || true
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f pub_camera_info 2>/dev/null || true
pkill -9 -f "ros2.bag.play" 2>/dev/null || true
sleep 3

# ---------- 1. static TF ----------
echo ">>> Starting static TF..."
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 base_link camera_color_optical_frame \
  --ros-args -p use_sim_time:=true &
sleep 1

# ---------- 2. camera_info publisher ----------
echo ">>> Starting camera_info publisher..."
python3 /tmp/pub_camera_info.py --ros-args -p use_sim_time:=true &
sleep 1

# ---------- 3. rtabmap (rgbd_odometry + rtabmap node) ----------
echo ">>> Launching rtabmap..."

# rtabmap-specific parameters go in "args" (passed as node arguments, parsed as strings)
# ROS2 node parameters (frame_id etc.) go as launch arguments
SP_LG_ARGS="--delete_db_on_start"
SP_LG_ARGS+=" --Kp/DetectorStrategy 11"
SP_LG_ARGS+=" --Vis/FeatureType 11"
SP_LG_ARGS+=" --SuperPoint/ModelPath $HOME/rtabmap_ws/models/superpoint_v1.pt"
SP_LG_ARGS+=" --SuperPoint/Cuda true"
SP_LG_ARGS+=" --Vis/CorNNType 6"
SP_LG_ARGS+=" --PyMatcher/Path $HOME/rtabmap_ws/src/rtabmap/corelib/src/python/rtabmap_lightglue.py"
SP_LG_ARGS+=" --PyMatcher/Cuda true"
SP_LG_ARGS+=" --Rtabmap/DetectionRate 1"

LAUNCH_CMD=(
  ros2 launch rtabmap_launch rtabmap.launch.py
  rgb_topic:=/camera/color/image_raw
  depth_topic:=/camera/aligned_depth_to_color/image_raw
  camera_info_topic:=/camera/color/camera_info
  frame_id:=base_link
  approx_sync:=true
  use_sim_time:=true
  qos:=2
  "database_path:=$LOG_DIR/rtabmap.db"
  rtabmap_viz:=false
  rviz:=false
  "args:=$SP_LG_ARGS"
)

if $USE_GDB; then
  echo ">>> GDB mode: launching with launch_prefix"
  LAUNCH_CMD+=(
    "launch_prefix:=gdb -batch -ex 'handle SIGSEGV stop print nopass' -ex run -ex 'thread apply all bt full 30' -ex quit --args"
  )
fi

echo ">>> Command: ${LAUNCH_CMD[*]}"
"${LAUNCH_CMD[@]}" > "$LOG_DIR/rtabmap.log" 2>&1 &

RTABMAP_PID=$!
echo ">>> rtabmap launch PID: $RTABMAP_PID"

# ---------- 4. wait for nodes to initialize ----------
echo ">>> Waiting 30s for initialization (SP model load + LG python init)..."
sleep 30

# check if still alive
if ! kill -0 $RTABMAP_PID 2>/dev/null; then
  echo "!!! rtabmap crashed during init!"
  tail -80 "$LOG_DIR/rtabmap.log"
  exit 1
fi

# ---------- 5. play bag ----------
echo ">>> Playing bag (first 60s at 0.5x)..."
ros2 bag play "$BAG_DIR" \
  --clock \
  --rate 0.5 \
  --start-offset 10 \
  --topics /camera/color/image_raw /camera/aligned_depth_to_color/image_raw \
  > "$LOG_DIR/bag_play.log" 2>&1 &
BAG_PID=$!

# Kill bag after 120s wall time (= ~60s of bag at 0.5x rate)
( sleep 120 && kill $BAG_PID 2>/dev/null ) &
TIMEOUT_PID=$!
echo ">>> Bag PID: $BAG_PID"

echo ">>> Waiting for bag to finish or rtabmap to crash..."
while kill -0 $BAG_PID 2>/dev/null && kill -0 $RTABMAP_PID 2>/dev/null; do
  sleep 5
done

echo ""
echo "=========================================="
if ! kill -0 $RTABMAP_PID 2>/dev/null; then
  echo "!!! rtabmap CRASHED! Check $LOG_DIR/rtabmap.log"
  echo "--- Last 80 lines ---"
  tail -80 "$LOG_DIR/rtabmap.log"
  echo ""
  echo "--- Core dumps ---"
  ls -lt /tmp/core.* 2>/dev/null | head -5
  coredumpctl list 2>/dev/null | tail -5
else
  echo ">>> Bag finished, rtabmap still running. Looking good!"
  echo "--- Last 30 lines ---"
  tail -30 "$LOG_DIR/rtabmap.log"
fi

echo ""
echo ">>> Full logs: $LOG_DIR/"
