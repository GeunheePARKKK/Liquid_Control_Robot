"""
arm 전후 구간을 통째로 캡처하는 진단용 스크립트.

시리얼 모니터를 닫고 실행할 것 (포트를 하나만 점유할 수 있음).

  python capture.py --port COM5                      # 5초 대기 → arm → 10초 기록
  python capture.py --port COM5 --pre 3 --post 15
  python capture.py --port COM5 --cmd t --post 8     # 시험 구동 캡처
  python capture.py --port COM5 --cmd none           # 명령 없이 관찰만

각 줄 앞에 arm 기준 상대시간(초)이 붙는다. 음수 = arm 이전.
"""
import argparse
import sys
import time
import serial

# Windows 콘솔 기본 코드페이지(cp949)는 em-dash 등을 못 찍고 예외를 낸다.
# 출력 때문에 캡처 데이터를 잃지 않도록 UTF-8 로 바꾸고, 안 되면 무시한다.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


def main(a):
    ser = serial.Serial(a.port, a.baud, timeout=0.1)
    time.sleep(2.0)                      # ESP32 리셋 대기
    ser.reset_input_buffer()

    lines = []
    t_cmd = None
    t0 = time.time()          # 명령 전 구간의 상대시간 기준

    def pump(until, tag):
        while time.time() < until:
            raw = ser.readline()
            if not raw:
                continue
            txt = raw.decode("utf-8", errors="ignore").rstrip()
            if not txt:
                continue
            # 명령 전에는 명령 시점까지 남은 시간을 음수로 표시
            rel = time.time() - (t_cmd if t_cmd else t0 + a.pre)
            line = f"{rel:+7.2f} [{tag}] {txt}"
            lines.append(line)
            try:
                print(line)
            except Exception:
                pass          # 콘솔 인코딩 문제로 캡처를 잃지 않는다

    try:
        print(f"--- {a.pre}초 관찰 (명령 전) ---")
        pump(t0 + a.pre, "PRE")

        if a.cmd.lower() != "none":
            print(f"--- 명령 전송: {a.cmd} ---")
            t_cmd = time.time()
            ser.write((a.cmd + "\n").encode())
            ser.flush()
            lines.append(f"{0.0:+7.2f} [CMD] >>> {a.cmd}")

        print(f"--- {a.post}초 기록 ---")
        pump(time.time() + a.post, "POST")

    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        # 어떤 경로로 끝나든 모터를 세우고 파일을 남긴다
        try:
            print("--- stop 전송 ---")
            ser.write(b"stop\n")
            ser.flush()
            time.sleep(0.5)
            pump(time.time() + 1.0, "END")
        except Exception:
            pass
        ser.close()

        with open(a.out, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print(f"\n저장 완료: {a.out} ({len(lines)}줄)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True, help="예: COM5")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--pre", type=float, default=5.0, help="명령 전 관찰 시간(초)")
    p.add_argument("--post", type=float, default=10.0, help="명령 후 기록 시간(초)")
    p.add_argument("--cmd", default="arm", help="보낼 명령 (arm / t / none)")
    p.add_argument("--out", default="capture_log.txt")
    main(p.parse_args())
