**터미널 0 - 호스트에서 컨테이너 시작**

```bash
docker start yee_humble_g4
docker exec -it yee_humble_g4 bash
```

**터미널 1 - LiDAR**

```bash
source /opt/ros/humble/setup.bash
source /root/yee_g4_ws/install/setup.bash
chmod 777 /dev/serial/by-id/*
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
  --ros-args --params-file /root/ydlidar_retry.yaml
```

**터미널 2 - Cartographer**

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run cartographer_ros cartographer_node \
  -configuration_directory /root/ydlidar_slam/config \
  -configuration_basename ydlidar_2d.lua
```

**터미널 3 - /map 생성**

bash

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.05 \
  -publish_period_sec 1.0
```

**터미널 4 - base_link → laser_frame TF**

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id base_link --child-frame-id laser_frame
```

**터미널 5 - Nav2**

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=/root/my_nav2_params.yaml
```

**터미널 6 - odom + 모터 브리지**

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
python3 /root/md400t_cmdvel_odom_bridge.py --ros-args \
  -p port_right:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISNS9-if00-port0 \
  -p port_left:=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT94EQPJ-if00-port0 \
  -p baudrate:=57600 \
  -p id_right:=1 \
  -p id_left:=1 \
  -p wheel_base:=0.32 \
  -p wheel_radius:=0.04 \
  -p counts_per_rev_right:=1280.0 \
  -p counts_per_rev_left:=1280.0 \
  -p speed_scale:=300.0 \
  -p max_speed_cmd:=300 \
  -p min_effective_cmd:=80 \
  -p send_hz:=20.0 \
  -p feedback_hz:=10.0 \
  -p stale_sec:=0.5
```

**터미널 7 - RViz2 (VNC 안에서)**

xhost +
docker run -it --net=host -e DISPLAY=:1.0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged yee_humble_friendbase:latest bash
  
rviz2 -d /root/my_robot.rviz
# rviz는 로컬에서 실행 yee@_root:에다 입력
