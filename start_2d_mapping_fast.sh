#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /root/rs_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

cleanup() {
  echo
  echo "[CLEANUP] stopping background nodes..."
  jobs -p | xargs -r kill
}
trap cleanup EXIT INT TERM

python3 /root/md400t_cmdvel_odom_bridge.py --ros-args \
  -r /cmd_vel:=/cmd_vel_tuned \
  -p port_right:=/dev/ttyUSB1 \
  -p port_left:=/dev/ttyUSB0 \
  -p baudrate:=57600 \
  -p id_right:=1 \
  -p id_left:=1 \
  -p wheel_radius:=0.05 \
  -p wheel_base:=0.46 \
  -p counts_per_rev_right:=260.0 \
  -p counts_per_rev_left:=260.0 \
  -p speed_scale:=300.0 \
  -p max_speed_cmd:=300 \
  -p min_effective_cmd:=80 \
  -p send_hz:=20.0 \
  -p feedback_hz:=10.0 \
  > /tmp/bridge.log 2>&1 &
sleep 2

python3 /root/cmd_vel_bias.py --ros-args -p right_correction:=0.0055 \
  > /tmp/cmd_vel_bias.log 2>&1 &
sleep 1

ros2 launch realsense2_camera rs_launch.py \
  initial_reset:=true \
  depth_module.depth_profile:=640x480x10 \
  rgb_camera.color_profile:=640x480x10 \
  align_depth.enable:=true \
  enable_sync:=true \
  > /tmp/realsense.log 2>&1 &
sleep 8

ros2 run tf2_ros static_transform_publisher 0.425 0.0 0.50 0 0 0 base_link camera_link \
  > /tmp/static_tf.log 2>&1 &
sleep 1

ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args \
  -r depth:=/camera/camera/aligned_depth_to_color/image_raw \
  -r depth_camera_info:=/camera/camera/aligned_depth_to_color/camera_info \
  -r scan:=/scan \
  -p output_frame:=camera_link \
  -p scan_height:=80 \
  -p range_min:=0.35 \
  -p range_max:=2.20 \
  > /tmp/depth_to_scan.log 2>&1 &
sleep 2

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/root/slam_d455_fast.yaml \
  use_sim_time:=false \
  > /tmp/slam_toolbox.log 2>&1 &
sleep 3

echo "[OK] 2D fast mapping stack started"
echo "bridge log      : /tmp/bridge.log"
echo "bias log        : /tmp/cmd_vel_bias.log"
echo "realsense log   : /tmp/realsense.log"
echo "static tf log   : /tmp/static_tf.log"
echo "depth->scan log : /tmp/depth_to_scan.log"
echo "slam log        : /tmp/slam_toolbox.log"
echo
echo "[INFO] keep this shell open"

wait
