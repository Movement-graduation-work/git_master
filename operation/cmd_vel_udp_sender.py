#!/usr/bin/env python3
import json
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelUdpSender(Node):
    def __init__(self):
        super().__init__('cmd_vel_udp_sender')

        self.declare_parameter('slave_ip', '192.168.0.140')
        self.declare_parameter('slave_port', 5005)
        self.declare_parameter('source_id', 'master')
        self.declare_parameter('wheel_base', 0.32)
        self.declare_parameter('speed_scale', 300.0)
        self.declare_parameter('max_speed_cmd', 300)
        self.declare_parameter('min_effective_cmd', 80)
        self.declare_parameter('reverse_right', False)
        self.declare_parameter('reverse_left', False)

        self.slave_ip = str(self.get_parameter('slave_ip').value)
        self.slave_port = int(self.get_parameter('slave_port').value)
        self.source_id = str(self.get_parameter('source_id').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_speed_cmd = int(self.get_parameter('max_speed_cmd').value)
        self.min_effective_cmd = int(self.get_parameter('min_effective_cmd').value)
        self.reverse_right = bool(self.get_parameter('reverse_right').value)
        self.reverse_left = bool(self.get_parameter('reverse_left').value)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.get_logger().info(
            f'sending /cmd_vel + driver_cmd UDP to {self.slave_ip}:{self.slave_port}'
        )

    def apply_min_effective(self, value: int) -> int:
        if value == 0:
            return 0
        if abs(value) < self.min_effective_cmd:
            return self.min_effective_cmd if value > 0 else -self.min_effective_cmd
        return value

    def cmd_vel_to_driver_cmd(self, linear_x: float, angular_z: float):
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)
        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)

        right_cmd = int(round(right_mps * self.speed_scale))
        left_cmd = int(round(left_mps * self.speed_scale))

        right_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, right_cmd))
        left_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, left_cmd))

        if abs(linear_x) < 0.01 or abs(angular_z) < 0.01:
            right_cmd = self.apply_min_effective(right_cmd)
            left_cmd = self.apply_min_effective(left_cmd)

        if self.reverse_right:
            right_cmd = -right_cmd
        if self.reverse_left:
            left_cmd = -left_cmd
        return right_cmd, left_cmd

    def on_cmd_vel(self, msg: Twist):
        self.seq += 1
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        right_cmd, left_cmd = self.cmd_vel_to_driver_cmd(linear_x, angular_z)
        payload = {
            'source': self.source_id,
            'seq': self.seq,
            'stamp': time.time(),
            'linear_x': linear_x,
            'angular_z': angular_z,
            'driver_right': right_cmd,
            'driver_left': left_cmd,
        }
        data = json.dumps(payload, separators=(',', ':')).encode('ascii')
        self.sock.sendto(data, (self.slave_ip, self.slave_port))

        if self.seq % 20 == 1:
            self.get_logger().info(
                f"tx seq={self.seq} v={payload['linear_x']:.3f} "
                f"w={payload['angular_z']:.3f} "
                f"right={right_cmd} left={left_cmd}"
            )

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelUdpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()