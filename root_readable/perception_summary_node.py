# (1) import
import rclpy
from rclpy.node import Node

# (2) msgs
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PerceptionSummaryNode(Node):
    def __init__(self):
        super().__init__('perception_summary_node')

        # (3) 마지막 값 저장
        self.last_state = "UNKNOWN"
        self.last_zone = "UNKNOWN"
        self.last_conf = -1.0

        self.last_vx = 0.0
        self.last_wz = 0.0
        self.got_cmd = False

        # (4) 구독
        self.sub_state = self.create_subscription(String, '/perception/state', self.on_state, 10)
        self.sub_zone  = self.create_subscription(String, '/perception/zone', self.on_zone, 10)
        self.sub_conf  = self.create_subscription(Float32, '/perception/best_conf', self.on_conf, 10)

        # (4-1) /cmd_vel도 구독해서 관제 한 줄에 넣기
        self.sub_cmd   = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        # (5) 퍼블리시
        self.pub_summary = self.create_publisher(String, '/perception/summary', 10)

        # (6) 5Hz
        self.timer = self.create_timer(0.2, self.publish_summary)

        self.get_logger().info("[perception_summary_node] started (summary includes cmd_vel)")

    def on_state(self, msg: String):
        self.last_state = msg.data

    def on_zone(self, msg: String):
        self.last_zone = msg.data

    def on_conf(self, msg: Float32):
        self.last_conf = float(msg.data)

    def on_cmd(self, msg: Twist):
        self.last_vx = float(msg.linear.x)
        self.last_wz = float(msg.angular.z)
        self.got_cmd = True

    def publish_summary(self):
        conf_txt = "NA" if self.last_conf < 0 else f"{self.last_conf:.2f}"
        cmd_txt  = "NA" if not self.got_cmd else f"vx={self.last_vx:.2f} wz={self.last_wz:.2f}"

        out = String()
        out.data = f"{self.last_state} | zone={self.last_zone} | conf={conf_txt} | cmd_vel:{cmd_txt}"
        self.pub_summary.publish(out)

def main():
    rclpy.init()
    node = PerceptionSummaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
