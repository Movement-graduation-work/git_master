#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def normalize_deg(x):
    return ((x + 180.0) % 360.0) - 180.0

def yaw_from_quat(x, y, z, w):
    return math.degrees(math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    ))

class TurnToAngle(Node):
    def __init__(self):
        super().__init__('turn_to_angle_odom')

        self.declare_parameter('target_deg', -180.0)   # 우회전=-180, 좌회전=180
        self.declare_parameter('speed', 0.18)
        self.declare_parameter('tolerance_deg', 3.0)

        self.target_deg = float(self.get_parameter('target_deg').value)
        self.speed = float(self.get_parameter('speed').value)
        self.tolerance_deg = float(self.get_parameter('tolerance_deg').value)

        self.current_yaw = None
        self.start_yaw = None

        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    def stop(self):
        msg = Twist()
        self.pub.publish(msg)

    def run(self):
        while rclpy.ok() and self.current_yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.start_yaw = self.current_yaw
        print(f"start_yaw_deg = {self.start_yaw:.2f}")

        msg = Twist()
        msg.angular.z = self.speed if self.target_deg > 0 else -self.speed

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

            delta = normalize_deg(self.current_yaw - self.start_yaw)
            error = self.target_deg - delta

            print(
                f"yaw_deg={self.current_yaw:.1f}  delta_deg={delta:.1f}  error_deg={error:.1f}",
                end='\r',
                flush=True
            )

            if abs(error) <= self.tolerance_deg:
                break

            self.pub.publish(msg)
            time.sleep(0.02)

        self.stop()
        time.sleep(0.2)
        self.stop()

        final_delta = normalize_deg(self.current_yaw - self.start_yaw)
        print()
        print(f"final_delta_deg = {final_delta:.2f}")
        print(f"target_deg      = {self.target_deg:.2f}")
        print(f"error_deg       = {self.target_deg - final_delta:.2f}")

def main():
    rclpy.init()
    node = TurnToAngle()
    try:
        node.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
