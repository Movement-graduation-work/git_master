import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ScanToPoints(Node):
    def __init__(self):
        super().__init__('scan_to_points')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(PointCloud2, '/scan_points', 10)
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.cb,
            qos
        )

    def cb(self, scan):
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append((x, y, z))
            angle += scan.angle_increment

        cloud = pc2.create_cloud_xyz32(scan.header, points)
        self.pub.publish(cloud)

def main():
    rclpy.init()
    node = ScanToPoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
