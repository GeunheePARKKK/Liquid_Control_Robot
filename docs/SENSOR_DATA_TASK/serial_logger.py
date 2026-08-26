import argparse
import serial
import time

def main(args):
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(2)  # ESP32 리셋 대기

    print(f"{args.port} 연결됨, {args.duration}초간 기록 후 {args.out}에 저장")
    print("Ctrl+C로 조기 종료 가능")

    lines = []
    start = time.time()
    try:
        while time.time() - start < args.duration:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                lines.append(line)
                print(line)
    except KeyboardInterrupt:
        print("\n중단됨, 지금까지 데이터 저장")

    with open(args.out, 'w', encoding='utf-8') as f:
        for line in lines:
            f.write(line + '\n')

    print(f"저장 완료: {args.out} ({len(lines)}줄)")
    ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="예: COM3 (Windows) 또는 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=3.0, help="기록할 시간(초)")
    parser.add_argument("--out", default="log_1.csv")
    main(parser.parse_args())
