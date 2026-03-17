# (1) imports
import time
import argparse

import serial


# (2) LRC 계산
def calc_lrc(body):
    return (-sum(body)) & 0xFF


# (3) single channel frame: 0x82 / 0x83
def build_single_speed_frame(driver_id, cmd, speed):
    speed_u16 = speed & 0xFFFF
    lo = speed_u16 & 0xFF
    hi = (speed_u16 >> 8) & 0xFF

    body = [driver_id & 0xFF, cmd & 0xFF, 0x02, lo, hi]
    lrc = calc_lrc(body)

    return bytes([0xB7, 0xB8] + body + [lrc])


# (4) dual continuous frame: 0xCF
def build_dual_frame(driver_id, ch1_cmd, ch2_cmd, accel=1, forward_dir=1, reverse_dir=0):
    def split_dir_mag(v):
        if v >= 0:
            return forward_dir, v
        return reverse_dir, abs(v)

    d1, s1 = split_dir_mag(ch1_cmd)
    d2, s2 = split_dir_mag(ch2_cmd)

    body = [
        driver_id & 0xFF,
        0xCF,
        0x07,
        d1, s1 & 0xFF, (s1 >> 8) & 0xFF,
        d2, s2 & 0xFF, (s2 >> 8) & 0xFF,
        accel & 0xFF
    ]
    lrc = calc_lrc(body)

    return bytes([0xB7, 0xB8] + body + [lrc])


def main():
    # (5) 인자
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--baudrate', type=int, default=57600)
    parser.add_argument('--id', type=int, default=1)

    parser.add_argument('--mode', choices=['single', 'dual'], default='single')
    parser.add_argument('--speed', type=int, default=30)
    parser.add_argument('--seconds', type=float, default=1.0)

    parser.add_argument('--forward_dir', type=int, default=1)
    parser.add_argument('--reverse_dir', type=int, default=0)
    parser.add_argument('--accel', type=int, default=1)

    args = parser.parse_args()

    # (6) 포트 열기
    ser = serial.Serial(args.port, args.baudrate, timeout=0.02)
    print(f"[OPEN] port={args.port} baud={args.baudrate} id={args.id} mode={args.mode}")

    try:
        # (7) 지정 시간 동안 반복 전송
        start = time.time()
        while time.time() - start < args.seconds:
            if args.mode == 'single':
                # (7-1) ch1, ch2 각각 같은 속도 전송
                f1 = build_single_speed_frame(args.id, 0x82, args.speed)
                f2 = build_single_speed_frame(args.id, 0x83, args.speed)
                ser.write(f1)
                time.sleep(0.002)
                ser.write(f2)
                print(f"[TX single] speed={args.speed}")
            else:
                # (7-2) dual continuous 전송
                f = build_dual_frame(
                    args.id,
                    args.speed,
                    args.speed,
                    accel=args.accel,
                    forward_dir=args.forward_dir,
                    reverse_dir=args.reverse_dir
                )
                ser.write(f)
                print(f"[TX dual] speed={args.speed} fwd={args.forward_dir} rev={args.reverse_dir}")

            time.sleep(0.05)

        # (8) 정지 전송
        if args.mode == 'single':
            ser.write(build_single_speed_frame(args.id, 0x82, 0))
            time.sleep(0.002)
            ser.write(build_single_speed_frame(args.id, 0x83, 0))
            print("[STOP single]")
        else:
            ser.write(build_dual_frame(args.id, 0, 0, accel=args.accel,
                                       forward_dir=args.forward_dir,
                                       reverse_dir=args.reverse_dir))
            print("[STOP dual]")

    finally:
        # (9) 포트 닫기
        ser.close()
        print("[CLOSE]")


if __name__ == '__main__':
    # (10) entry
    main()
