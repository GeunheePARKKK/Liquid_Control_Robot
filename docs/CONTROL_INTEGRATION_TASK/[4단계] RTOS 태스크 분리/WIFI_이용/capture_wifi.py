"""
capture.py 의 와이파이판. 시리얼 대신 TCP 로 보드에 붙는다.

  노트북을 보드 와이파이("Liquid_Robot")에 붙인 뒤:

  python capture_wifi.py                                   # 192.168.4.1:23, 5초 대기 → arm → 10초
  python capture_wifi.py --pre 8 --post 40 --cmd "kdp0.5,arm" --out run1.txt
  python capture_wifi.py --cmd none                        # 관찰만

capture.py 와 옵션·출력 형식이 같다. 다른 점 하나 — TCP 로 붙어도 보드가
리셋되지 않는다. 걸어둔 설정이 그대로 남으므로 --cmd 에 다시 나열할 필요가 없다.
(습관대로 나열해도 해가 없다.)
"""
import argparse
import socket
import sys
import time

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


class TcpLine:
    """소켓을 readline 있는 시리얼처럼 쓴다."""
    def __init__(self, host, port, timeout=0.1):
        self.s = socket.create_connection((host, port), timeout=5.0)
        self.s.settimeout(timeout)
        self.buf = b""

    def readline(self):
        while b"\n" not in self.buf:
            try:
                chunk = self.s.recv(4096)
            except socket.timeout:
                return b""
            if not chunk:
                return b""
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return line + b"\n"

    def write(self, b):
        self.s.sendall(b)

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.buf = b""

    def close(self):
        try:
            self.s.close()
        except Exception:
            pass


def main(a):
    ser = TcpLine(a.host, a.tcp)
    time.sleep(0.5)
    ser.reset_input_buffer()

    lines = []
    t_cmd = None
    t0 = time.time()

    def pump(until, tag):
        while time.time() < until:
            raw = ser.readline()
            if not raw:
                continue
            txt = raw.decode("utf-8", errors="ignore").rstrip()
            if not txt:
                continue
            rel = time.time() - (t_cmd if t_cmd else t0 + a.pre)
            line = f"{rel:+7.2f} [{tag}] {txt}"
            lines.append(line)
            try:
                print(line)
            except Exception:
                pass

    try:
        print(f"--- {a.pre}초 관찰 (명령 전) ---")
        pump(t0 + a.pre, "PRE")

        if a.cmd.lower() != "none":
            cmds = [c.strip() for c in a.cmd.split(",") if c.strip()]
            print("--- 명령 전송: " + " -> ".join(cmds) + " ---")
            t_cmd = time.time()
            for n, c in enumerate(cmds):
                ser.write((c + "\n").encode())
                lines.append("%+7.2f [CMD] >>> %s" % (time.time() - t_cmd, c))
                if n < len(cmds) - 1:
                    pump(time.time() + a.gap, "CMD")
        print(f"--- {a.post}초 기록 ---")
        pump(time.time() + a.post, "POST")

    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        try:
            print("--- stop 전송 ---")
            ser.write(b"stop\n")
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
    p.add_argument("--host", default="192.168.4.1", help="보드 IP (AP 모드 기본 192.168.4.1)")
    p.add_argument("--tcp", type=int, default=23, help="TCP 포트")
    p.add_argument("--pre", type=float, default=5.0)
    p.add_argument("--post", type=float, default=10.0)
    p.add_argument("--cmd", default="arm")
    p.add_argument("--gap", type=float, default=0.4)
    p.add_argument("--out", default="capture_log.txt")
    main(p.parse_args())
