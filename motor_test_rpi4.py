# motor_test.py (Raspberry Pi 4 / macOS / Ubuntu)
# - 안전한 포트 자동선택(ttyAMA0 기본 제외)
# - 플로우컨트롤 OFF 명시
# - write_timeout 여유 확보
# - (권장) RPi GPIO UART는 /dev/serial0로 지정해서 사용

import time
import argparse
import serial
import serial.tools.list_ports


def lrc8(data: bytes) -> int:
    s = sum(data) & 0xFF
    return (-s) & 0xFF


def i16_le(v: int) -> tuple[int, int]:
    """signed int16 -> little-endian 2 bytes"""
    v = max(-32768, min(32767, v))
    u = v & 0xFFFF
    return (u & 0xFF, (u >> 8) & 0xFF)


def u8(v: int) -> int:
    return max(0, min(255, int(v)))


def build_md400t_dual_speed_frame_signed(
    dev_id: int,
    spd1: int,   # signed
    spd2: int,   # signed
    accel: int,
    dir1: int = 1,  # 일부 장비는 0이면 무효라서 기본 1 유지
    dir2: int = 1
) -> bytes:
    header = bytes([0xB7, 0xB8])
    pid_cf = 0xCF
    length = 0x07

    s1l, s1h = i16_le(spd1)
    s2l, s2h = i16_le(spd2)

    payload = bytes([
        dir1 & 0xFF,
        s1l, s1h,
        dir2 & 0xFF,
        s2l, s2h,
        u8(accel)
    ])

    frame_wo_chk = header + bytes([dev_id & 0xFF, pid_cf, length]) + payload
    return frame_wo_chk + bytes([lrc8(frame_wo_chk)])


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("인식된 시리얼 포트가 없습니다. (USB-RS485 연결/드라이버/케이블 확인)")
        return
    for p in ports:
        print(f"{p.device} | {p.description}")


def auto_pick_port(prefer_linux_serial0: bool = True) -> str | None:
    """
    - RPi에서는 /dev/serial0를 우선 권장
    - ttyAMA0는 블루투스(hciattach)나 콘솔이 잡는 경우가 많아 기본 제외
    """
    # 1) RPi 권장: /dev/serial0 존재하면 우선 사용
    if prefer_linux_serial0:
        try:
            # 존재 여부만 확인
            import os
            if os.path.exists("/dev/serial0"):
                return "/dev/serial0"
        except Exception:
            pass

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None

    cand = [p.device for p in ports]

    # 2) 위험 포트 기본 제외: ttyAMA0
    cand = [d for d in cand if "ttyAMA" not in d]

    # 3) 선호 키워드
    prefer = ("usbserial", "usbmodem", "ttyUSB", "ttyACM", "serial0", "serial1")
    for key in prefer:
        for dev in cand:
            if key in dev:
                return dev

    return cand[0] if cand else None


def open_serial(port: str, baud: int) -> serial.Serial:
    """
    - 플로우 컨트롤 OFF 명시
    - write_timeout 여유 확보
    """
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.05,
        write_timeout=2.0,   # 디버깅/현장에서는 여유있게
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def main():
    ap = argparse.ArgumentParser(description="MD400T RS-485 keep-alive (signed speed for reverse)")

    ap.add_argument("--list", action="store_true", help="포트 목록 출력 후 종료")

    # ✅ RPi 권장: /dev/serial0 (블루투스/콘솔 충돌을 피하기 쉬움)
    ap.add_argument("--port", default=None, help="mac: /dev/cu.usbserial-XXXX, linux: /dev/ttyUSB0 or /dev/serial0")

    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--id", type=int, default=1)

    # ✅ 속도는 signed: 음수면 역방향
    ap.add_argument("--spd1", type=int, default=-200, help="signed speed, negative = reverse")
    ap.add_argument("--spd2", type=int, default=200, help="signed speed, negative = reverse")
    ap.add_argument("--accel", type=int, default=5)

    # dir 바이트는 일부 장비에서 0이면 무효라서 기본 1 유지
    ap.add_argument("--dir1", type=int, default=1, help="dir byte (keep 1 if unsure)")
    ap.add_argument("--dir2", type=int, default=1, help="dir byte (keep 1 if unsure)")

    # keep-alive
    ap.add_argument("--hz", type=float, default=100.0, help="send rate (Hz)")
    ap.add_argument("--duration", type=float, default=3600.0, help="seconds to run")

    # 출력
    ap.add_argument("--print-frame", action="store_true", help="프레임 hex 출력(속도 느려질 수 있음)")

    args = ap.parse_args()

    if args.list:
        list_ports()
        return

    port = args.port or auto_pick_port(prefer_linux_serial0=True)
    if not port:
        print("포트 없음. --list로 확인하거나 --port로 지정하세요.")
        return

    if args.hz <= 0:
        print("hz는 0보다 커야 합니다.")
        return

    period = 1.0 / args.hz
    t_end = time.time() + args.duration

    ser = None
    try:
        ser = open_serial(port, args.baud)
        print(f"[INFO] port={port} baud={args.baud} hz={args.hz} duration={args.duration}s")
        print(f"[INFO] spd1={args.spd1} spd2={args.spd2} (negative = reverse) accel={args.accel}")

        # (선택) 시작 전에 한번 정지 프레임
        stop_frame = build_md400t_dual_speed_frame_signed(args.id, 0, 0, args.accel, 1, 1)
        ser.write(stop_frame)
        ser.flush()
        time.sleep(0.2)

        # 메인 프레임
        frame = build_md400t_dual_speed_frame_signed(
            args.id, args.spd1, args.spd2, args.accel, args.dir1, args.dir2
        )

        if args.print_frame:
            print("[FRAME]", frame.hex(" ").upper())

        while time.time() < t_end:
            try:
                ser.write(frame)
                ser.flush()
            except Exception as e:
                print("[WARN] write failed -> reopen:", e)
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(0.2)
                ser = open_serial(port, args.baud)

            time.sleep(period)

    finally:
        try:
            if ser:
                # 종료 시 정지 한 번
                stop_frame = build_md400t_dual_speed_frame_signed(args.id, 0, 0, args.accel, 1, 1)
                ser.write(stop_frame)
                ser.flush()
                time.sleep(0.05)
                ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
