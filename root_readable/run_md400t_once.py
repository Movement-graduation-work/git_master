# 임시 모터 구동 코드 26.03.10

import subprocess
import time
import signal
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


BRIDGE_CMD = [
    "python3", "/root/md400t_cmdvel_odom_bridge.py",
    "--ros-args",
    "-p", "port_right:=/dev/ttyUSB1",
    "-p", "port_left:=/dev/ttyUSB0",
    "-p", "baudrate:=57600",
    "-p", "id_right:=1",
    "-p", "id_left:=1",
    "-p", "wheel_radius:=0.05",
    "-p", "wheel_base:=0.32",
    "-p", "counts_per_rev_right:=260.0",
    "-p", "counts_per_rev_left:=260.0",
    "-p", "speed_scale:=300.0",
    "-p", "max_speed_cmd:=300",
    "-p", "min_effective_cmd:=80",
    "-p", "send_hz:=20.0",
    "-p", "feedback_hz:=10.0",
]


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("cmdvel_test_publisher")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_for_duration(self, linear_x: float, angular_z: float, hz: float, duration_sec: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        period = 1.0 / hz
        end_time = time.time() + duration_sec

        while time.time() < end_time:
            self.pub.publish(msg)
            time.sleep(period)

    def stop_once(self):
        msg = Twist()
        self.pub.publish(msg)
        time.sleep(0.2)
        self.pub.publish(msg)


def main():
    bridge_proc = None
    try:
        print("[1] 브리지 실행")
        bridge_proc = subprocess.Popen(BRIDGE_CMD)

        print("[2] 브리지 초기화 대기")
        time.sleep(3.0)

        print("[3] ROS2 publisher 시작")
        rclpy.init()
        node = CmdVelPublisher()

        # 예시 1: 3초 전진
        print("[4] 전진 3초")
        node.publish_for_duration(linear_x=0.10, angular_z=0.0, hz=10.0, duration_sec=3.0)

        # 예시 2: 2초 정지
        print("[5] 정지")
        node.stop_once()
        time.sleep(2.0)

        # 예시 3: 2초 좌회전
        print("[6] 좌회전 2초")
        node.publish_for_duration(linear_x=0.0, angular_z=0.5, hz=10.0, duration_sec=2.0)

        # 최종 정지
        print("[7] 최종 정지")
        node.stop_once()

        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("\n[중단] Ctrl+C 입력됨")
    finally:
        if bridge_proc is not None:
            print("[종료] 브리지 프로세스 종료")
            bridge_proc.send_signal(signal.SIGINT)
            try:
                bridge_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                bridge_proc.kill()


if __name__ == "__main__":
    main()
