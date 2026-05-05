터미널 -> 도커 이동
docker start yee_humble_g4
docker exec -it yee_humble_g4 bash

rviz2용 2번 도커 실행
docker start rviz_container
docker exec -it rviz_container bash

ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args --params-file /root/ydlidar_retry.yaml

라이다 실행 

ros2 run cartographer_ros cartographer_node -configuration_directory /root/ydlidar_slam/config -configuration_basename ydlidar_2d.lua

카토그래퍼 실행 


 ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05 -publish_period_sec 1.0

그 이후 토픽 발행 


ros2 run nav2_map_server map_saver_cli -f /root/maps/my_map 

마지막 저장
