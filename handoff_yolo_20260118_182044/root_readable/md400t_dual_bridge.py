# (1) imports
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    # (1-1) 시리얼 통신
    import serial
except ImportError:
    serial = None


class MDMotorBridge(Node):
    def __init__(self):
        # (2) 노드 생성
        super().__init__('md400t_dual_bridge')

        # (3) 파라미터 선언
        # (3-1) 포트 2개: 오른쪽 드라이버 / 왼쪽 드라이버
        self.declare_parameter('port_right', '/dev/ttyUSB0')
        self.declare_parameter('port_left', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 57600)

        # (3-2) 드라이버 ID
        # 포트가 분리되어 있으면 둘 다 1로 시작해도 됨
        self.declare_parameter('id_right', 1)
        self.declare_parameter('id_left', 1)

        # (3-3) 차동구동 파라미터
        self.declare_parameter('wheel_base', 0.32)    # m
        self.declare_parameter('speed_scale', 300.0)  # m/s -> 드라이버 speed 단위 변환
        self.declare_parameter('max_speed_cmd', 80)   # signed int16 speed 제한
        self.declare_parameter('min_effective_cmd', 20)  # 너무 약하면 안 움직일 수 있어 최소치 부스트

        # (3-4) 전송/안전
        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('stale_sec', 0.5)
        self.declare_parameter('accel', 1)

        # (3-5) 명령 방식
        # dual_continuous: 0xCF (dir + speed + accel)
        # single_channel:  0x82 / 0x83
        self.declare_parameter('command_mode', 'dual_continuous')

        # (3-6) 방향 정의
        self.declare_parameter('forward_dir', 1)
        self.declare_parameter('reverse_dir', 0)

        # (3-7) 채널별 반전
        # 오른쪽 드라이버의 ch1/ch2, 왼쪽 드라이버의 ch1/ch2
        self.declare_parameter('reverse_right_ch1', False)
        self.declare_parameter('reverse_right_ch2', False)
        self.declare_parameter('reverse_left_ch1', False)
        self.declare_parameter('reverse_left_ch2', False)

        # (4) 파라미터 읽기
        self.port_right = str(self.get_parameter('port_right').value)
        self.port_left = str(self.get_parameter('port_left').value)
        self.baudrate = int(self.get_parameter('baudrate').value)

        self.id_right = int(self.get_parameter('id_right').value)
        self.id_left = int(self.get_parameter('id_left').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_speed_cmd = int(self.get_parameter('max_speed_cmd').value)
        self.min_effective_cmd = int(self.get_parameter('min_effective_cmd').value)

        self.send_hz = float(self.get_parameter('send_hz').value)
        self.stale_sec = float(self.get_parameter('stale_sec').value)
        self.accel = int(self.get_parameter('accel').value)

        self.command_mode = str(self.get_parameter('command_mode').value).strip().lower()

        self.forward_dir = int(self.get_parameter('forward_dir').value)
        self.reverse_dir = int(self.get_parameter('reverse_dir').value)

        self.reverse_right_ch1 = bool(self.get_parameter('reverse_right_ch1').value)
        self.reverse_right_ch2 = bool(self.get_parameter('reverse_right_ch2').value)
        self.reverse_left_ch1 = bool(self.get_parameter('reverse_left_ch1').value)
        self.reverse_left_ch2 = bool(self.get_parameter('reverse_left_ch2').value)

        # (5) 내부 상태
        self.last_cmd_time = 0.0
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

        self.prev_right_cmd = None
        self.prev_left_cmd = None

        # (6) pyserial 체크
        if serial is None:
            raise RuntimeError("pyserial 이 없습니다. pip install pyserial 필요")

        # (7) 포트 2개 열기
        self.ser_right = serial.Serial(
            port=self.port_right,
            baudrate=self.baudrate,
            timeout=0.02
        )

        self.ser_left = serial.Serial(
            port=self.port_left,
            baudrate=self.baudrate,
            timeout=0.02
        )

        # (8) /cmd_vel 구독
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.on_cmd_vel,
            10
        )

        # (9) 주기 전송
        period = 1.0 / max(self.send_hz, 0.1)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"[md400t_dual_bridge] "
            f"port_right={self.port_right} id_right={self.id_right}, "
            f"port_left={self.port_left} id_left={self.id_left}, "
            f"mode={self.command_mode}, wheel_base={self.wheel_base}, "
            f"speed_scale={self.speed_scale}, max_speed_cmd={self.max_speed_cmd}"
        )

    def on_cmd_vel(self, msg: Twist):
        # (10) /cmd_vel 저장
        self.last_linear_x = float(msg.linear.x)
        self.last_angular_z = float(msg.angular.z)
        self.last_cmd_time = time.time()

        # (10-1) 디버그 로그
        self.get_logger().info(
            f"[cmd_vel] linear_x={self.last_linear_x:.3f}, angular_z={self.last_angular_z:.3f}"
        )

    def on_timer(self):
        # (11) stale 처리
        now = time.time()
        if (now - self.last_cmd_time) > self.stale_sec:
            right_cmd = 0
            left_cmd = 0
        else:
            right_cmd, left_cmd = self.cmd_vel_to_driver_cmd(
                self.last_linear_x,
                self.last_angular_z
            )

        # (12) 디버그 로그 (값 바뀔 때만)
        if right_cmd != self.prev_right_cmd or left_cmd != self.prev_left_cmd:
            self.get_logger().info(
                f"[driver_cmd] right={right_cmd}, left={left_cmd}, mode={self.command_mode}"
            )
            self.prev_right_cmd = right_cmd
            self.prev_left_cmd = left_cmd

        # (13) 오른쪽/왼쪽 드라이버로 전송
        self.send_side_driver(
            ser=self.ser_right,
            driver_id=self.id_right,
            base_cmd=right_cmd,
            reverse_ch1=self.reverse_right_ch1,
            reverse_ch2=self.reverse_right_ch2
        )

        self.send_side_driver(
            ser=self.ser_left,
            driver_id=self.id_left,
            base_cmd=left_cmd,
            reverse_ch1=self.reverse_left_ch1,
            reverse_ch2=self.reverse_left_ch2
        )

    def cmd_vel_to_driver_cmd(self, linear_x: float, angular_z: float):
        # (14) 차동구동 계산
        # 오른쪽/왼쪽 바퀴 선속도(m/s)
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)
        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)

        # (14-1) m/s -> 드라이버 speed 단위
        right_cmd = int(round(right_mps * self.speed_scale))
        left_cmd = int(round(left_mps * self.speed_scale))

        # (14-2) 최대값 제한
        right_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, right_cmd))
        left_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, left_cmd))

        # (14-3) 너무 작은 값은 정지 마찰 때문에 안 움직일 수 있어서 최소치 보정
        if right_cmd != 0 and abs(right_cmd) < self.min_effective_cmd:
            right_cmd = self.min_effective_cmd if right_cmd > 0 else -self.min_effective_cmd

        if left_cmd != 0 and abs(left_cmd) < self.min_effective_cmd:
            left_cmd = self.min_effective_cmd if left_cmd > 0 else -self.min_effective_cmd

        return right_cmd, left_cmd

    def send_side_driver(self, ser, driver_id: int, base_cmd: int, reverse_ch1: bool, reverse_ch2: bool):
        # (15) 같은 쪽 바퀴 2개를 가진 드라이버에 전송
        ch1_cmd = -base_cmd if reverse_ch1 else base_cmd
        ch2_cmd = -base_cmd if reverse_ch2 else base_cmd

        if self.command_mode == 'dual_continuous':
            frame = self.build_dual_continuous_frame(
                driver_id=driver_id,
                ch1_cmd=ch1_cmd,
                ch2_cmd=ch2_cmd,
                accel=self.accel
            )
            ser.write(frame)

        elif self.command_mode == 'single_channel':
            frame1 = self.build_single_speed_frame(driver_id, 0x82, ch1_cmd)
            ser.write(frame1)
            time.sleep(0.002)
            frame2 = self.build_single_speed_frame(driver_id, 0x83, ch2_cmd)
            ser.write(frame2)

        else:
            self.get_logger().error(f"unknown command_mode: {self.command_mode}")

    def build_single_speed_frame(self, driver_id: int, cmd: int, speed: int) -> bytes:
        # (16) 0x82 / 0x83 방식, signed int16 little-endian
        speed_u16 = speed & 0xFFFF
        lo = speed_u16 & 0xFF
        hi = (speed_u16 >> 8) & 0xFF

        body = [driver_id & 0xFF, cmd & 0xFF, 0x02, lo, hi]
        lrc = (-sum(body)) & 0xFF

        return bytes([0xB7, 0xB8] + body + [lrc])

    def build_dual_continuous_frame(self, driver_id: int, ch1_cmd: int, ch2_cmd: int, accel: int) -> bytes:
        # (17) 0xCF 방식: dir1, spd1_L, spd1_H, dir2, spd2_L, spd2_H, accel
        dir1, spd1 = self.split_dir_and_mag(ch1_cmd)
        dir2, spd2 = self.split_dir_and_mag(ch2_cmd)

        spd1_lo = spd1 & 0xFF
        spd1_hi = (spd1 >> 8) & 0xFF
        spd2_lo = spd2 & 0xFF
        spd2_hi = (spd2 >> 8) & 0xFF

        body = [
            driver_id & 0xFF,
            0xCF,
            0x07,
            dir1, spd1_lo, spd1_hi,
            dir2, spd2_lo, spd2_hi,
            accel & 0xFF
        ]

        lrc = (-sum(body)) & 0xFF
        return bytes([0xB7, 0xB8] + body + [lrc])

    def split_dir_and_mag(self, signed_cmd: int):
        # (18) signed speed -> direction + magnitude
        if signed_cmd >= 0:
            return self.forward_dir, signed_cmd
        return self.reverse_dir, abs(signed_cmd)

    def stop_all(self):
        # (19) 정지 프레임 전송
        try:
            self.send_side_driver(self.ser_right, self.id_right, 0, self.reverse_right_ch1, self.reverse_right_ch2)
            time.sleep(0.005)
            self.send_side_driver(self.ser_left, self.id_left, 0, self.reverse_left_ch1, self.reverse_left_ch2)
        except Exception:
            pass

    def destroy_node(self):
        # (20) 종료 시 정지 + 포트 닫기
        try:
            self.stop_all()
        except Exception:
            pass

        try:
            if hasattr(self, 'ser_right') and self.ser_right and self.ser_right.is_open:
                self.ser_right.close()
        except Exception:
            pass

        try:
            if hasattr(self, 'ser_left') and self.ser_left and self.ser_left.is_open:
                self.ser_left.close()
        except Exception:
            pass

        super().destroy_node()


def main():
    # (21) ROS2 시작
    rclpy.init()

    # (22) 노드 생성
    node = MDMotorBridge()

    try:
        # (23) spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # (24) 종료 정리
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # (25) entry
    main()
