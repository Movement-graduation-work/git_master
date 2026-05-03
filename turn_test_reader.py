import math
import threading
import time
import rclpy
from nav_msgs.msg import Odometry

TOPIC = '/odometry/filtered'
latest_yaw = None

def yaw_from_quat(x, y, z, w):
    yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return math.degrees(yaw)

def cb(msg):
    global latest_yaw
    q = msg.pose.pose.orientation
    latest_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

rclpy.init()
node = rclpy.create_node('turn_test_reader')
sub = node.create_subscription(Odometry, TOPIC, cb, 10)

def spin_loop():
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

t = threading.Thread(target=spin_loop, daemon=True)
t.start()

while latest_yaw is None:
    time.sleep(0.1)

input("시작 자세를 맞춘 뒤 Enter 누르기: ")
start_yaw = latest_yaw
print(f"start_yaw_deg = {start_yaw:.2f}")

input("정확히 180도 돌린 뒤 Enter 누르기: ")
end_yaw = latest_yaw
print(f"end_yaw_deg   = {end_yaw:.2f}")

delta = ((end_yaw - start_yaw + 180.0) % 360.0) - 180.0
abs_turn = abs(delta)
error_from_180 = abs(abs_turn - 180.0)

print(f"delta_deg          = {delta:.2f}")
print(f"abs_turn_deg       = {abs_turn:.2f}")
print(f"error_from_180_deg = {error_from_180:.2f}")

node.destroy_node()
rclpy.shutdown()
