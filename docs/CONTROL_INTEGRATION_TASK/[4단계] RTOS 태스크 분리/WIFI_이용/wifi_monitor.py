"""
와이파이 시리얼 모니터. 시리얼 모니터 창과 똑같이 쓴다.

  1. 노트북 와이파이를 "Liquid_Robot" 에 연결 (비밀번호 liquid1234)
     "인터넷 연결 안됨" 표시는 정상. 무시한다.
  2. python wifi_monitor.py
  3. 로그가 흐른다. 아무 때나 명령을 치고 Enter:  ?  arm  stop  kdp0.5  ...
     끝내려면 Ctrl+C  (끝날 때 stop 을 보낸다)

  다른 주소면:  python wifi_monitor.py --host 192.168.4.1 --tcp 23
"""
import argparse
import socket
import sys
import threading
import time

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


def main(a):
    print(f"접속 중  {a.host}:{a.tcp} ...")
    try:
        s = socket.create_connection((a.host, a.tcp), timeout=5.0)
    except OSError as e:
        print(f"접속 실패: {e}")
        print("노트북이 Liquid_Robot 와이파이에 붙어 있는지 확인 (인터넷 없음 표시는 정상)")
        return
    s.settimeout(0.2)
    print("접속됨. 명령 입력 후 Enter. 종료 Ctrl+C\n")

    stop = False

    def reader():
        buf = b""
        while not stop:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                print("\n[연결 끊김]")
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    print(line.decode("utf-8", errors="replace").rstrip())
                except Exception:
                    pass

    t = threading.Thread(target=reader, daemon=True)
    t.start()

    try:
        while t.is_alive():
            cmd = sys.stdin.readline()
            if not cmd:
                break
            cmd = cmd.strip()
            if cmd:
                s.sendall((cmd + "\n").encode())
    except KeyboardInterrupt:
        pass
    finally:
        stop = True
        try:
            s.sendall(b"stop\n")
            time.sleep(0.3)
        except Exception:
            pass
        s.close()
        print("\n종료 (stop 전송)")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="192.168.4.1")
    p.add_argument("--tcp", type=int, default=23)
    main(p.parse_args())
