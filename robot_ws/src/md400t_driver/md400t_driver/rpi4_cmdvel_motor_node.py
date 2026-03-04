#26.03.11 jetson /cmd_vel(속도) -> rpi4 motor operate
#!/usr/bin/env python3
import math
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def lrc8(data: bytes) -> int:
    s = sum(data) & 0xFF
    return (-s) & 0xFF


def build_md400t_dual_speed_frame(dev_id: int, spd1: int, spd2: int, accel: int) -> bytes:
    """
    MD400T 듀얼 모터 속도 프레임 생성
    - spd1, spd2: -32768 ~ 32767 범위 가정
    - 부호에 따라 dir 자동 결정
    """
    dir1 = 0x01 if spd1 >= 0 else 0x00
    dir2 = 0x01 if spd2 >= 0 else 0x00

    abs_spd1 = abs(int(spd1))
    abs_spd2 = abs(int(spd2))

    abs_spd1 = max(0, min(abs_spd1, 32767))
    abs_spd2 = max(0, min(abs_spd2, 32767))
    accel = max(0, min(int(accel), 255))

    header = bytes([0xB7, 0xB8])
    pid_cf = 0xCF
    length = 0x07

    payload = bytes([
        dir1 & 0xFF,
        abs_spd1 & 0xFF, (abs_spd1 >> 8) & 0xFF,
        dir2 & 0xFF,
        abs_spd2 & 0xFF, (abs_spd2 >> 8) & 0xFF,
        accel & 0xFF
    ])

    frame_wo_chk = header + bytes([dev_id & 0xFF, pid_cf, length]) + payload
    chk = lrc8(frame_wo_chk)
    frame = frame_wo_chk + bytes([chk])
    return frame


class CmdVelMotorNode(Node):
    def __init__(self):
        super().__init__('cmdvel_motor_node')

        # ===== ROS 파라미터 =====
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('dev_id', 1)

        self.declare_parameter('wheel_radius', 0.05)      # m
        self.declare_parameter('wheel_base', 0.32)        # m
        self.declare_parameter('speed_scale', 300.0)      # rad/s -> driver cmd
        self.declare_parameter('max_speed_cmd', 300)
        self.declare_parameter('min_effective_cmd', 80)
        self.declare_parameter('accel', 20)

        self.declare_parameter('cmd_timeout', 0.5)        # sec
        self.declare_parameter('send_hz', 20.0)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.dev_id = int(self.get_parameter('dev_id').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_speed_cmd = int(self.get_parameter('max_speed_cmd').value)
        self.min_effective_cmd = int(self.get_parameter('min_effective_cmd').value)
        self.accel = int(self.get_parameter('accel').value)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.send_hz = float(self.get_parameter('send_hz').value)

        # ===== 상태 변수 =====
        self.last_cmd_time = time.time()
        self.target_left_cmd = 0
        self.target_right_cmd = 0
        self.last_sent_left = None
        self.last_sent_right = None

        # ===== Serial 연결 =====
        self.ser = None
        self.open_serial()

        # ===== Subscriber =====
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ===== Timer =====
        timer_period = 1.0 / self.send_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('cmdvel_motor_node started')
        self.get_logger().info(
            f'port={self.port}, baudrate={self.baudrate}, dev_id={self.dev_id}, '
            f'wheel_radius={self.wheel_radius}, wheel_base={self.wheel_base}'
        )

    def open_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.05
            )
            self.get_logger().info(f'Serial opened: {self.port} @ {self.baudrate}')
        except Exception as e:
            self.ser = None
            self.get_logger().error(f'Failed to open serial {self.port}: {e}')

    def clamp_cmd(self, value: int) -> int:
        return max(-self.max_speed_cmd, min(self.max_speed_cmd, value))

    def apply_min_effective(self, value: int) -> int:
        if value == 0:
            return 0
        if abs(value) < self.min_effective_cmd:
            return self.min_effective_cmd if value > 0 else -self.min_effective_cmd
        return value

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel -> 좌/우 바퀴 명령 변환
        """
        v = msg.linear.x
        w = msg.angular.z

        # differential drive
        v_left = v - (self.wheel_base / 2.0) * w
        v_right = v + (self.wheel_base / 2.0) * w

        # 선속도 -> 바퀴 각속도(rad/s)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # 각속도 -> 드라이버 명령값
        left_cmd = int(omega_left * self.speed_scale)
        right_cmd = int(omega_right * self.speed_scale)

        left_cmd = self.clamp_cmd(left_cmd)
        right_cmd = self.clamp_cmd(right_cmd)

        left_cmd = self.apply_min_effective(left_cmd)
        right_cmd = self.apply_min_effective(right_cmd)

        self.target_left_cmd = left_cmd
        self.target_right_cmd = right_cmd
        self.last_cmd_time = time.time()

        self.get_logger().info(
            f'/cmd_vel received: v={v:.3f}, w={w:.3f} -> left={left_cmd}, right={right_cmd}'
        )

    def send_motor_command(self, left_cmd: int, right_cmd: int):
        frame = build_md400t_dual_speed_frame(
            dev_id=self.dev_id,
            spd1=right_cmd,   # 오른쪽 모터
            spd2=left_cmd,    # 왼쪽 모터
            accel=self.accel
        )

        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('Serial not open, cannot send command')
            return

        try:
            self.ser.write(frame)
            self.ser.flush()
            self.get_logger().debug(
                'TX: ' + ' '.join(f'{b:02X}' for b in frame)
            )
        except Exception as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def control_loop(self):
        """
        주기적으로 모터 명령 전송
        - 최근 cmd_vel 없으면 watchdog으로 정지
        """
        now = time.time()

        if (now - self.last_cmd_time) > self.cmd_timeout:
            left_cmd = 0
            right_cmd = 0
        else:
            left_cmd = self.target_left_cmd
            right_cmd = self.target_right_cmd

        # 같은 값 계속 보내도 되지만, 로그를 줄이기 위해 비교
        if left_cmd != self.last_sent_left or right_cmd != self.last_sent_right:
            self.send_motor_command(left_cmd, right_cmd)
            self.last_sent_left = left_cmd
            self.last_sent_right = right_cmd

            if left_cmd == 0 and right_cmd == 0:
                self.get_logger().warn('Watchdog stop or zero command sent')

    def stop_motors(self):
        try:
            self.send_motor_command(0, 0)
            time.sleep(0.1)
        except Exception:
            pass

    def destroy_node(self):
        self.stop_motors()

        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
