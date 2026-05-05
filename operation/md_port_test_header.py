# (1) 전역 변수 초기화
import time
import argparse
import serial


# (2) 함수 정의: LRC 체크섬 (헤더 B7 B8 포함)
def lrc8_with_header(frame_wo_chk: bytes) -> int:
    # (2-1) 체크섬 제외 전체 프레임 합을 8비트로 줄이기
    s = sum(frame_wo_chk) & 0xFF
    # (2-2) 2의 보수
    return (-s) & 0xFF


# (3) 함수 정의: signed int16 little-endian 2바이트로 변환
def int16_le_bytes(value: int) -> bytes:
    # (3-1) signed int16 범위 체크
    if value < -32768 or value > 32767:
        raise ValueError(f"speed out of range: {value}")

    # (3-2) 16비트 2의 보수 형태로 변환
    u16 = value & 0xFFFF
    lo = u16 & 0xFF
    hi = (u16 >> 8) & 0xFF
    return bytes([lo, hi])


# (4) 함수 정의: 단일 속도 프레임 생성
#     예) B7 B8 01 82 02 64 00 A8
def build_single_speed_frame(dev_id: int, cmd: int, speed: int) -> bytes:
    # (4-1) 헤더
    header = bytes([0xB7, 0xB8])

    # (4-2) speed 2바이트
    spd = int16_le_bytes(speed)

    # (4-3) 체크섬 제외 전체 프레임
    frame_wo_chk = header + bytes([
        dev_id & 0xFF,
        cmd & 0xFF,     # 0x82 or 0x83
        0x02            # LEN=2
    ]) + spd

    # (4-4) 헤더 포함 LRC 계산
    chk = lrc8_with_header(frame_wo_chk)

    # (4-5) 최종 프레임
    return frame_wo_chk + bytes([chk])


# (5) 함수 정의: 브레이크 프레임 생성
#     예) B7 B8 01 06 01 01 88
def build_brake_frame(dev_id: int, brake_on: int = 1) -> bytes:
    # (5-1) 헤더
    header = bytes([0xB7, 0xB8])

    # (5-2) 체크섬 제외 전체 프레임
    frame_wo_chk = header + bytes([
        dev_id & 0xFF,
        0x06,           # CMD=Brake
        0x01,           # LEN=1
        brake_on & 0xFF
    ])

    # (5-3) 헤더 포함 LRC 계산
    chk = lrc8_with_header(frame_wo_chk)

    # (5-4) 최종 프레임
    return frame_wo_chk + bytes([chk])


# (6) 함수 정의: 보기 좋게 출력
def dec_dump(b: bytes) -> str:
    # (6-1) 10진수 출력
    return " ".join(str(x) for x in b)


def hex_dump(b: bytes) -> str:
    # (6-2) 16진수 출력
    return " ".join(f"{x:02X}" for x in b)


# (7) 메인 실행
def main():
    # (7-1) 인자 받기
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="예: /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--id", type=int, default=1)

    # (7-2) mode
    # speed  : 속도 명령
    # brake  : 브레이크 ON
    # stop   : 속도 0
    ap.add_argument("--mode", choices=["speed", "brake", "stop"], default="speed")

    # (7-3) single motor command
    # 0x82 = Motor1, 0x83 = Motor2
    ap.add_argument("--cmd", type=lambda x: int(x, 0), default=0x82)

    # (7-4) speed 값
    ap.add_argument("--speed", type=int, default=100)

    # (7-5) 반복 횟수 / 주기
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--interval", type=float, default=0.05)

    # (7-6) stop 자동 전송 여부
    ap.add_argument("--auto_stop", type=int, default=1)

    args = ap.parse_args()

    # (7-7) 시리얼 열기
    ser = serial.Serial(args.port, args.baud, timeout=0.2)

    try:
        # (7-8) 반복 전송
        for i in range(args.repeat):
            if args.mode == "speed":
                frame = build_single_speed_frame(args.id, args.cmd, args.speed)
            elif args.mode == "stop":
                frame = build_single_speed_frame(args.id, args.cmd, 0)
            else:
                frame = build_brake_frame(args.id, 1)

            # (7-9) 송신 프레임 출력
            print(f"[TX #{i+1}] dec: {dec_dump(frame)}")
            print(f"[TX #{i+1}] hex: {hex_dump(frame)}")

            # (7-10) 송신
            ser.write(frame)
            ser.flush()

            # (7-11) 짧게 대기
            time.sleep(args.interval)

            # (7-12) 응답 있으면 읽기
            rx = ser.read(ser.in_waiting or 0)
            if rx:
                print(f"[RX #{i+1}] dec: {dec_dump(rx)}")
                print(f"[RX #{i+1}] hex: {hex_dump(rx)}")
            else:
                print(f"[RX #{i+1}] (none)")

        # (7-13) auto_stop이면 마지막에 정지 프레임 전송
        if args.auto_stop == 1 and args.mode == "speed":
            stop_frame = build_single_speed_frame(args.id, args.cmd, 0)
            print("[AUTO STOP] dec:", dec_dump(stop_frame))
            print("[AUTO STOP] hex:", hex_dump(stop_frame))
            ser.write(stop_frame)
            ser.flush()

    finally:
        # (7-14) 포트 닫기
        ser.close()


# (8) 함수 호출
if __name__ == "__main__":
    main()
