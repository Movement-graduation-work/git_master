# yolo_ros2_node.py
# (1) imports
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import cv2
import torch
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_v8n_node')

        # (2) ROS I/O
        self.bridge = CvBridge()

        # (2-1) 카메라 이미지 구독 (RealSense는 보통 sensor_data QoS가 잘 맞음)
        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.cb_image,
            qos_profile_sensor_data  # (2-1) QoS
        )

        # (2-2) 사람 상태 노드가 먹는 detections (Float32MultiArray)
        self.pub_det = self.create_publisher(
            Float32MultiArray,
            '/yolo/detections',
            10
        )

        # (2-3) 디버그용 raw (동일 포맷)
        self.pub_raw = self.create_publisher(
            Float32MultiArray,
            '/yolo/detections_raw',
            10
        )

        # (2-4) 박스 그린 이미지
        self.pub_anno = self.create_publisher(
            Image,
            '/yolo/annotated/image_raw',
            qos_profile_sensor_data
        )

        # (3) GPU 체크
        dev = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"[GPU CHECK] torch={torch.__version__} cuda_available={torch.cuda.is_available()} device={dev}")

        # (4) YOLO 엔진 로드 (TensorRT engine)
        self.model = YOLO('/root/models/yolov8n_fp16.engine', task='detect')
        self.get_logger().info('YOLO engine loaded')

    def cb_image(self, msg: Image):
        # (5) ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # (6) 추론
        r = self.model(frame, verbose=False)[0]

        # (7) detections 만들기: [cls, conf, x1, y1, x2, y2] 반복
        data = []
        if r.boxes is not None and len(r.boxes) > 0:
            for b in r.boxes:
                cls = float(b.cls.item())
                conf = float(b.conf.item())
                x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                data.extend([cls, conf, x1, y1, x2, y2])

        out = Float32MultiArray()
        out.data = data

        # (8) publish (둘 다 같은 포맷으로 발행)
        self.pub_det.publish(out)
        self.pub_raw.publish(out)

        # (9) annotated 이미지 생성 + publish
        anno = frame.copy()
        # (9-1) 박스 그리기
        for i in range(0, len(data), 6):
            cls, conf, x1, y1, x2, y2 = data[i:i+6]
            p1 = (int(x1), int(y1))
            p2 = (int(x2), int(y2))
            cv2.rectangle(anno, p1, p2, (0, 255, 0), 2)
            cv2.putText(
                anno,
                f"cls={int(cls)} conf={conf:.2f}",
                (p1[0], max(0, p1[1] - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

        anno_msg = self.bridge.cv2_to_imgmsg(anno, encoding='bgr8')
        anno_msg.header = msg.header  # (9-2) 타임스탬프/프레임 유지
        self.pub_anno.publish(anno_msg)

def main():
    # (10) rclpy init
    rclpy.init()

    # (11) node 생성
    node = YoloNode()

    try:
        # (12) spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        # (13) Ctrl+C 종료
        pass
    finally:
        # (14) cleanup
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
