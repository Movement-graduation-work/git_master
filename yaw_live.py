import math
import rclpy
from nav_msgs.msg import Odometry

TOPIC = '/odometry/filtered'
last_printed = None

def yaw_from_quat(x, y, z, w):
    yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    return math.degrees(yaw)

def cb(msg):
    global last_printed
    q = msg.pose.pose.orientation
    yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

    # 너무 많이 찍히지 않게 0.5도 이상 바뀔 때만 출력
    if last_printed is None or abs(yaw - last_printed) >= 0.5:
        print(f"yaw_deg = {yaw:.2f}", flush=True)
        last_printed = yaw

rclpy.init()
node = rclpy.create_node('yaw_live_reader')
sub = node.create_subscription(Odometry, TOPIC, cb, 10)

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
