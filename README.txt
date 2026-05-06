### 1번 터미널 — LiDAR

라이다 연결 확인

source /opt/ros/humble/setup.bash
source /root/yee_g4_ws/install/setup.bash

chmod 777 /dev/serial/by-id/*

ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node \
--ros-args --params-file /root/ydlidar_retry.yaml

#만약 정상이라면 
# ros2 pkg list | grep ydlidar → OK

-------------------
2번 터미널 — Cartographer

exportFASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run cartographer_ros cartographer_node \
-configuration_directory /root/ydlidar_slam/config \
-configuration_basename ydlidar_2d.lua

--------------
3번 터미널 — /map 생성

exportFASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run cartographer_ros cartographer_occupancy_grid_node \
-resolution0.05 \
-publish_period_sec1.0


# 여기서 -resolution 0.05는 맵 한 칸이 5cm라는 뜻이고, -publish_period_sec 1.0은 /map을 1초 주기로 갱신하겠다는 뜻이야. 이 노드는 submap을 받아 OccupancyGrid를 만든다.
------------------

4번 터미널 — base_link TF

export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher \
  --frame-id laser_frame --child-frame-id base_link

5번 터미널 — Nav2


export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false

-----
RViz2 (VNC 안에서)

xhost +
docker run -it --net=host -e DISPLAY=:1.0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged yee_humble_friendbase:latest bash
rviz2
