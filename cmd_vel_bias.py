#!/usr/bin/env python3
# (1) import
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# (2) node
class CmdVelBias(Node):
    def __init__(self):
        super().__init__('cmd_vel_bias')

        # (3) parameter
        self.declare_parameter('right_correction', 0.03)   # 좌측으로 말릴 때 오른쪽으로 살짝 보정
        self.declare_parameter('linear_threshold', 0.01)
        self.declare_parameter('angular_deadband', 0.02)

        self.right_correction = float(self.get_parameter('right_correction').value)
        self.linear_threshold = float(self.get_parameter('linear_threshold').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)

        # (4) sub/pub
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_tuned', 10)

    # (5) callback
    def cb(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z

        # (6) 직진일 때만 오른쪽 보정(ROS에서 음수 z = 오른쪽 회전)
        if msg.linear.x > self.linear_threshold and abs(msg.angular.z) < self.angular_deadband:
            out.angular.z = msg.angular.z - self.right_correction

        # (7) publish
        self.pub.publish(out)

# (8) main
def main():
    rclpy.init()
    node = CmdVelBias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
