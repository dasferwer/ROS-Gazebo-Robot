#!/bin/bash
set -euo pipefail
out_dir=${1:-/ws/artifacts/demo}
mkdir -p "$out_dir"
ffmpeg -y -f x11grab -framerate 15 -video_size 1600x900 -i :1 \
    -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p "$out_dir/demo.mp4" \
    > "$out_dir/video.log" 2>&1 &
video_pid=$!
ros2 bag record --use-sim-time -o "$out_dir/rosbag" \
    /clock /tf /tf_static /odom /scan /depth /depth/points /gps/fix \
    /cmd_vel_safe /route_status /plan /ground_truth /global_costmap/costmap \
    /contacts/body /contacts/left /contacts/right > "$out_dir/bag.log" 2>&1 &
bag_pid=$!
finish() {
    kill -INT "$video_pid" "$bag_pid" 2>/dev/null || true
    wait "$video_pid" "$bag_pid" || true
}
trap finish EXIT
trap 'exit 0' INT TERM
wait
