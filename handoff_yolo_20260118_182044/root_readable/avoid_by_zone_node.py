# (1) import
import time
import rclpy
from rclpy.node import Node

# (2) msgs
from std_msgs.msg import String          # (2-1) /perception/state, /perception/zone
from geometry_msgs.msg import Twist      # (2-2) /cmd_vel

class AvoidByZoneNode(Node):
    def __init__(self):
        super().__init__('avoid_by_zone_node')

        # (3) params (기본값)
        self.declare_parameter('forward_mps', 0.15)     # (3-1) 사람이 없을 때 전진 속도
        self.declare_parameter('turn_radps', 0.60)      # (3-2) 사람이 있을 때 회피 회전 속도
        self.declare_parameter('publish_hz', 10.0)      # (3-3) cmd_vel 발행 주기
        self.declare_parameter('stale_sec', 1.0)        # (3-4) perception 입력 끊기면 정지
        self.declare_parameter('center_action', 'STOP') # (3-5) CENTER일 때: STOP or TURN
        # TURN을 쓰면 CENTER에서도 회전(회피 시도)함. 기본은 STOP이 안전.

        self.forward_mps = float(self.get_parameter('forward_mps').value)
        self.turn_radps = float(self.get_parameter('turn_radps').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.stale_sec = float(self.get_parameter('stale_sec').value)
        self.center_action = str(self.get_parameter('center_action').value).upper()

        # (4) last perception values
        self.state = "UNKNOWN"
        self.zone = "UNKNOWN"
        self.last_rx = 0.0  # (4-1) 마지막 메시지 수신 시각(초)
        self.last_rz = 0.0

        # (5) subs
        self.sub_state = self.create_subscription(
            String, '/perception/state', self.on_state, 10
        )
        self.sub_zone = self.create_subscription(
            String, '/perception/zone', self.on_zone, 10
        )

        # (6) pub
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # (7) timer
        period = 1.0 / max(self.publish_hz, 0.1)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info(
            f"[avoid_by_zone_node] forward_mps={self.forward_mps}, turn_radps={self.turn_radps}, "
            f"publish_hz={self.publish_hz}, stale_sec={self.stale_sec}, center_action={self.center_action}"
        )

    def now_sec(self):
        # (8) 현재 시간을 초로
        return self.get_clock().now().nanoseconds / 1e9

    def on_state(self, msg: String):
        # (9) state 수신
        self.state = msg.data
        self.last_rx = self.now_sec()

    def on_zone(self, msg: String):
        # (10) zone 수신
        self.zone = msg.data
        self.last_rx = self.now_sec()

    def tick(self):
        # (11) stale 처리: perception 입력이 끊기면 정지
        t = self.now_sec()
        if (t - self.last_rx) > self.stale_sec:
            self.publish_cmd(0.0, 0.0)
            return

        # (12) 행동 결정
        # (12-1) 사람이 없으면 전진
        if self.state == "NO_HUMAN":
            self.publish_cmd(self.forward_mps, 0.0)
            return

        # (12-2) 사람이 있으면 회피/정지
        if self.state == "HUMAN_DETECTED":
            if self.zone == "LEFT":
                # 사람이 왼쪽 -> 오른쪽으로 피하려면 시계방향 회전(보통 angular_z 음수)
                self.publish_cmd(0.0, -self.turn_radps)
                return
            if self.zone == "RIGHT":
                # 사람이 오른쪽 -> 왼쪽으로 피하려면 반시계 회전(보통 angular_z 양수)
                self.publish_cmd(0.0, +self.turn_radps)
                return
            if self.zone == "CENTER":
                if self.center_action == "TURN":
                    # (옵션) 중앙이면 일단 회전으로 피해보기
                    self.publish_cmd(0.0, +self.turn_radps)
                else:
                    # (기본) 중앙이면 안전하게 정지
                    self.publish_cmd(0.0, 0.0)
                return

            # zone이 UNKNOWN이면 안전 정지
            self.publish_cmd(0.0, 0.0)
            return

        # (12-3) 그 외 상태(UNKNOWN 등) -> 정지
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, linear_x: float, angular_z: float):
        # (13) cmd_vel publish
        # (13-1) 같은 값이면 불필요하게 안 찍고 그대로 발행만(토픽은 계속 나감)
        self.last_rz = angular_z
        self.last_rx = self.last_rx  # (13-2) 의미상 유지

        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub_cmd.publish(msg)

def main():
    # (14) init
    rclpy.init()

    # (15) node
    node = AvoidByZoneNode()

    try:
        # (16) spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # (17) shutdown
        node.destroy_node()
        rclpy.shutdown()

# (18) entry
if __name__ == '__main__':
    main()
