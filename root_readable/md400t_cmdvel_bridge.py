# (1) 전역 변수 초기화
import time
import math
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# (2) 함수 정의: 헤더 포함 LRC 계산
def lrc8_with_header(frame_wo_chk: bytes) -> int:
    # (2-1) 전체 프레임 합을 8비트로 줄이기
    s = sum(frame_wo_chk) & 0xFF
    # (2-2) 2의 보수
    return (-s) & 0xFF


# (3) 함수 정의: signed int16 little-endian 2바이트 변환
def int16_le_bytes(value: int) -> bytes:
    # (3-1) 범위 체크
    if value < -32768 or value > 32767:
        raise ValueError(f"speed out of range: {value}")

    # (3-2) 16비트 2의 보수 변환
    u16 = value & 0xFFFF
    lo = u16 & 0xFF
    hi = (u16 >> 8) & 0xFF
    return bytes([lo, hi])


# (4) 함수 정의: 단일 속도 프레임 생성
#     성공 확인된 형식: B7 B8 ID 82 02 spdL spdH LRC
def build_single_speed_frame(dev_id: int, speed: int) -> bytes:
    # (4-1) 헤더
    header = bytes([0xB7, 0xB8])

    # (4-2) 속도 바이트
    spd = int16_le_bytes(speed)

    # (4-3) 체크섬 제외 전체 프레임
    frame_wo_chk = header + bytes([
        dev_id & 0xFF,
        0x82,       # (4-3-1) 성공 확인된 CMD = 130 = 0x82
        0x02        # (4-3-2) LEN = 2
    ]) + spd

    # (4-4) 헤더 포함 LRC
    chk = lrc8_with_header(frame_wo_chk)

    # (4-5) 최종 프레임
    return frame_wo_chk + bytes([chk])


class MD400TCmdVelBridge(Node):
    def __init__(self):
        # (5) 노드 생성
        super().__init__('md400t_cmdvel_bridge')

        # (6) 파라미터 선언
        # (6-1) 포트 매핑 (확정)
        self.declare_parameter('port_right', '/dev/ttyUSB0')
        self.declare_parameter('port_left', '/dev/ttyUSB1')

        # (6-2) 통신
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('id_right', 1)
        self.declare_parameter('id_left', 1)

        # (6-3) 차동구동 변환
        self.declare_parameter('wheel_base', 0.32)          # m
        self.declare_parameter('speed_scale', 300.0)        # m/s -> 드라이버 speed 값
        self.declare_parameter('max_speed_cmd', 400)        # GUI에서 300도 확인됐으니 여유 조금 둠
        self.declare_parameter('min_effective_cmd', 60)     # 너무 약하면 버벅일 수 있어서 최소치 보정

        # (6-4) 방향 반전
        self.declare_parameter('reverse_right', False)
        self.declare_parameter('reverse_left', False)

        # (6-5) 전송/안전
        self.declare_parameter('send_hz', 20.0)
        self.declare_parameter('stale_sec', 0.5)

        # (7) 파라미터 읽기
        self.port_right = str(self.get_parameter('port_right').value)
        self.port_left = str(self.get_parameter('port_left').value)

        self.baudrate = int(self.get_parameter('baudrate').value)
        self.id_right = int(self.get_parameter('id_right').value)
        self.id_left = int(self.get_parameter('id_left').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.max_speed_cmd = int(self.get_parameter('max_speed_cmd').value)
        self.min_effective_cmd = int(self.get_parameter('min_effective_cmd').value)

        self.reverse_right = bool(self.get_parameter('reverse_right').value)
        self.reverse_left = bool(self.get_parameter('reverse_left').value)

        self.send_hz = float(self.get_parameter('send_hz').value)
        self.stale_sec = float(self.get_parameter('stale_sec').value)

        # (8) 내부 상태
        self.last_cmd_time = 0.0
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

        self.prev_right_cmd = None
        self.prev_left_cmd = None

        # (9) 시리얼 포트 열기
        self.ser_right = serial.Serial(self.port_right, self.baudrate, timeout=0.02)
        self.ser_left = serial.Serial(self.port_left, self.baudrate, timeout=0.02)

        # (10) /cmd_vel 구독
        self.sub_cmd = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.on_cmd_vel,
            10
        )

        # (11) 주기 전송 타이머
        period = 1.0 / max(self.send_hz, 0.1)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"[start] right={self.port_right}(id={self.id_right}), "
            f"left={self.port_left}(id={self.id_left}), "
            f"wheel_base={self.wheel_base}, speed_scale={self.speed_scale}"
        )

    def on_cmd_vel(self, msg: Twist):
        # (12) /cmd_vel 저장
        self.last_linear_x = float(msg.linear.x)
        self.last_angular_z = float(msg.angular.z)
        self.last_cmd_time = time.time()

        # (12-1) 디버그 로그
        self.get_logger().info(
            f"[cmd_vel] linear_x={self.last_linear_x:.3f}, angular_z={self.last_angular_z:.3f}"
        )

    def on_timer(self):
        # (13) stale 처리: 최근 명령 없으면 정지
        now = time.time()
        if (now - self.last_cmd_time) > self.stale_sec:
            right_cmd = 0
            left_cmd = 0
        else:
            right_cmd, left_cmd = self.cmd_vel_to_driver_cmd(
                self.last_linear_x,
                self.last_angular_z
            )

        # (14) 방향 반전
        if self.reverse_right:
            right_cmd = -right_cmd
        if self.reverse_left:
            left_cmd = -left_cmd

        # (15) 값 바뀔 때만 로그
        if right_cmd != self.prev_right_cmd or left_cmd != self.prev_left_cmd:
            self.get_logger().info(
                f"[driver_cmd] right={right_cmd}, left={left_cmd}"
            )
            self.prev_right_cmd = right_cmd
            self.prev_left_cmd = left_cmd

        # (16) 오른쪽/왼쪽 드라이버에 각각 전송
        self.send_speed(self.ser_right, self.id_right, right_cmd)
        self.send_speed(self.ser_left, self.id_left, left_cmd)

    def cmd_vel_to_driver_cmd(self, linear_x: float, angular_z: float):
        # (17) 차동구동 계산
        # (17-1) 오른쪽/왼쪽 선속도(m/s)
        right_mps = linear_x + (angular_z * self.wheel_base / 2.0)
        left_mps = linear_x - (angular_z * self.wheel_base / 2.0)

        # (17-2) m/s -> 드라이버 speed 값
        right_cmd = int(round(right_mps * self.speed_scale))
        left_cmd = int(round(left_mps * self.speed_scale))

        # (17-3) 최대값 제한
        right_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, right_cmd))
        left_cmd = max(-self.max_speed_cmd, min(self.max_speed_cmd, left_cmd))

        # (17-4) 너무 작은 값은 최소치 보정
        if right_cmd != 0 and abs(right_cmd) < self.min_effective_cmd:
            right_cmd = self.min_effective_cmd if right_cmd > 0 else -self.min_effective_cmd

        if left_cmd != 0 and abs(left_cmd) < self.min_effective_cmd:
            left_cmd = self.min_effective_cmd if left_cmd > 0 else -self.min_effective_cmd

        return right_cmd, left_cmd

    def send_speed(self, ser, dev_id: int, speed: int):
        # (18) 단일 속도 프레임 전송
        frame = build_single_speed_frame(dev_id, speed)
        ser.write(frame)
        ser.flush()

    def stop_all(self):
        # (19) 양쪽 정지
        try:
            self.send_speed(self.ser_right, self.id_right, 0)
            time.sleep(0.01)
            self.send_speed(self.ser_left, self.id_left, 0)
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


# (21) 함수 정의: 메인 실행
def main():
    # (21-1) ROS2 시작
    rclpy.init()

    # (21-2) 노드 생성
    node = MD400TCmdVelBridge()

    try:
        # (21-3) spin
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # (21-4) 종료 정리
        node.destroy_node()
        rclpy.shutdown()


# (22) 함수 호출
if __name__ == '__main__':
    main()
