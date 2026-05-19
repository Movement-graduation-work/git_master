# (1) import
import time
import collections
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool

class PersonStateNode(Node):
    def __init__(self):
        super().__init__('person_state_node')

        # (2) 파라미터
        self.declare_parameter('conf_threshold', 0.40)  # (2-1) 사람 conf 기준
        self.declare_parameter('publish_hz', 10.0)      # (2-2) 상태 publish 주기

        # (2-3) 관제 안정화: 프레임 히스테리시스
        self.declare_parameter('win_size', 4)          # (2-4) 최근 몇 프레임 볼지 (예: 10프레임)
        self.declare_parameter('on_count', 2)           # (2-5) 그 중 최소 몇 번 person이면 HUMAN (예: 2)
        self.declare_parameter('off_streak', 3)        # (2-6) 연속 몇 프레임 person 없으면 NO (예: 12)

        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.win_size = int(self.get_parameter('win_size').value)
        self.on_count = int(self.get_parameter('on_count').value)
        self.off_streak = int(self.get_parameter('off_streak').value)

        # (3) 상태/버퍼
        self.buf = collections.deque(maxlen=max(self.win_size, 1))  # (3-1) 최근 프레임 person 여부(True/False)
        self.no_person_run = 0                                      # (3-2) 연속 미감지 프레임 수
        self.human = False                                          # (3-3) 현재 상태

        # (4) ROS I/O
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/detections',
            self.on_detections,
            10
        )
        self.pub_state = self.create_publisher(String, '/perception/state', 10)
        self.pub_bool  = self.create_publisher(Bool,   '/perception/human_detected', 10)

        period = 1.0 / max(self.publish_hz, 0.1)
        self.timer = self.create_timer(period, self.publish_state)

        self.get_logger().info(
            f"[person_state_node] conf_threshold={self.conf_threshold}, publish_hz={self.publish_hz}, "
            f"win_size={self.win_size}, on_count={self.on_count}, off_streak={self.off_streak}"
        )

    def on_detections(self, msg: Float32MultiArray):
        # (5) 입력 수신
        data = list(msg.data)

        # (6) 기본: 이번 프레임에 person이 있는지 판단
        found_person = False

        if len(data) >= 6 and (len(data) % 6 == 0):
            for i in range(0, len(data), 6):
                class_id = int(round(data[i + 0]))   # (6-1) class_id
                conf     = float(data[i + 1])        # (6-2) conf
                if class_id == 0 and conf >= self.conf_threshold:
                    found_person = True
                    break

        # (7) 버퍼/연속미감지 업데이트
        self.buf.append(found_person)

        if found_person:
            self.no_person_run = 0
        else:
            self.no_person_run += 1

        # (8) 상태 업데이트 로직(히스테리시스)
        # (8-1) OFF는 “연속 off_streak 프레임 미감지”일 때만
        if self.human and self.no_person_run >= self.off_streak:
            self.human = False
            self.get_logger().info("[state] NO_HUMAN")

        # (8-2) ON은 “최근 win_size 프레임 중 on_count 이상 감지”면
        if (not self.human) and (sum(self.buf) >= self.on_count):
            self.human = True
            self.get_logger().info("[state] HUMAN_DETECTED")

    def publish_state(self):
        # (9) 상태 publish
        state = "HUMAN_DETECTED" if self.human else "NO_HUMAN"

        msg_state = String()
        msg_state.data = state
        self.pub_state.publish(msg_state)

        msg_bool = Bool()
        msg_bool.data = bool(self.human)
        self.pub_bool.publish(msg_bool)

def main():
    # (10) init
    rclpy.init()

    # (11) node 생성
    node = PersonStateNode()

    try:
        # (12) spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # (13) shutdown
        node.destroy_node()
        rclpy.shutdown()

# (14) entry
if __name__ == '__main__':
    main()
