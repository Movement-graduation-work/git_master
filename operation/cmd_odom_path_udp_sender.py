#!/usr/bin/env python3
import json
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def stamp_to_dict(stamp):
    return {
        'sec': int(stamp.sec),
        'nanosec': int(stamp.nanosec),
    }


def vector3_to_dict(value):
    return {
        'x': float(value.x),
        'y': float(value.y),
        'z': float(value.z),
    }


def quaternion_to_dict(value):
    return {
        'x': float(value.x),
        'y': float(value.y),
        'z': float(value.z),
        'w': float(value.w),
    }


def twist_to_dict(msg: Twist):
    return {
        'linear': vector3_to_dict(msg.linear),
        'angular': vector3_to_dict(msg.angular),
    }


def odom_to_dict(msg: Odometry):
    return {
        'stamp': stamp_to_dict(msg.header.stamp),
        'frame_id': msg.header.frame_id,
        'child_frame_id': msg.child_frame_id,
        'pose': {
            'position': vector3_to_dict(msg.pose.pose.position),
            'orientation': quaternion_to_dict(msg.pose.pose.orientation),
        },
        'twist': twist_to_dict(msg.twist.twist),
    }


def pose_stamped_to_dict(msg):
    return {
        'stamp': stamp_to_dict(msg.header.stamp),
        'frame_id': msg.header.frame_id,
        'position': vector3_to_dict(msg.pose.position),
        'orientation': quaternion_to_dict(msg.pose.orientation),
    }


class CmdOdomPathUdpSender(Node):
    def __init__(self):
        super().__init__('cmd_odom_path_udp_sender')

        self.declare_parameter('target_ip', '192.168.0.140')
        self.declare_parameter('target_port', 5015)
        self.declare_parameter('send_hz', 5.0)
        self.declare_parameter('path_max_points', 80)
        self.declare_parameter('source_id', 'master')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/robot_path')

        self.target_ip = str(self.get_parameter('target_ip').value)
        self.target_port = int(self.get_parameter('target_port').value)
        self.send_hz = float(self.get_parameter('send_hz').value)
        self.path_max_points = int(self.get_parameter('path_max_points').value)
        self.source_id = str(self.get_parameter('source_id').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)

        self.last_cmd = None
        self.last_odom = None
        self.last_path = None
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.create_timer(1.0 / max(self.send_hz, 0.1), self.on_timer)

        self.get_logger().info(
            f'telemetry UDP sender -> {self.target_ip}:{self.target_port}, '
            f'cmd={self.cmd_topic}, odom={self.odom_topic}, path={self.path_topic}'
        )

    def on_cmd(self, msg):
        self.last_cmd = msg

    def on_odom(self, msg):
        self.last_odom = msg

    def on_path(self, msg):
        self.last_path = msg

    def path_to_dict(self, msg: Path):
        poses = msg.poses[-max(self.path_max_points, 0):]
        return {
            'stamp': stamp_to_dict(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'total_poses': len(msg.poses),
            'sent_poses': len(poses),
            'poses': [pose_stamped_to_dict(pose) for pose in poses],
        }

    def on_timer(self):
        self.seq += 1
        payload = {
            'source': self.source_id,
            'seq': self.seq,
            'stamp': time.time(),
            'cmd_vel': twist_to_dict(self.last_cmd) if self.last_cmd else None,
            'odom': odom_to_dict(self.last_odom) if self.last_odom else None,
            'path': self.path_to_dict(self.last_path) if self.last_path else None,
        }
        data = json.dumps(payload, separators=(',', ':')).encode('utf-8')
        self.sock.sendto(data, (self.target_ip, self.target_port))

        if self.seq % max(1, int(self.send_hz * 2.0)) == 1:
            path_count = 0
            if payload['path'] is not None:
                path_count = payload['path']['sent_poses']
            self.get_logger().info(
                f'tx telemetry seq={self.seq} bytes={len(data)} '
                f'cmd={payload["cmd_vel"] is not None} '
                f'odom={payload["odom"] is not None} path_points={path_count}'
            )

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdOdomPathUdpSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
