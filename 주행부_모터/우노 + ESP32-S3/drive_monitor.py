"""
주행용 와이파이 시리얼 모니터 (wifi_monitor.py 와 같음, 와이파이 이름만 다름).

  1. 노트북 와이파이를 "Drive_Robot" 에 연결 (비밀번호 drive1234)
     "인터넷 연결 안됨" 표시는 정상. 무시한다.
  2. python drive_monitor.py
  3. 로그가 흐른다. w a s d 치고 Enter (같은 키 또 치면 한 단계 가속, w1500 = 1.5초).
     x 정지, ! 비상정지, v150 최고속도, vmin70, vstep20, ramp700, bal0/bal1, ?
     끝내려면 Ctrl+C  (끝날 때 stop 을 보낸다)

  연결이 끊기면 (보드 리셋 등) 바로 알리고 자동으로 다시 붙는다. 끊긴 동안 친 명령은 버린다.
  다시 붙었을 때 보드가 찍는 "리셋 원인: …" 줄을 보면 보드가 왜 죽었는지 알 수 있다
  (BROWNOUT = 전원 전압 부족, PANIC = 코드 예외).

  다른 주소면:  python drive_monitor.py --host 192.168.4.1 --tcp 23
"""
import argparse
import queue
import socket
import sys
import threading
import time

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# 키 입력은 프로그램 내내 한 스레드가 읽어서 큐에 넣는다. (메인이 입력에 묶이면 끊김을 못 알아챈다)
inq: "queue.Queue[str | None]" = queue.Queue()


def input_thread():
    while True:
        line = sys.stdin.readline()
        if not line:
            inq.put(None)          # EOF
            return
        inq.put(line.strip())


def connect(a, first):
    """붙을 때까지 1초마다 재시도. 입력이 끝나면(EOF) None."""
    tries = 0
    while True:
        try:
            s = socket.create_connection((a.host, a.tcp), timeout=5.0)
            s.settimeout(0.2)
            return s
        except OSError as e:
            if user_quit():
                return None
            tries += 1
            if first and tries == 1:
                print(f"접속 실패: {e}")
                print("노트북이 Drive_Robot 와이파이에 붙어 있는지 확인 (인터넷 없음 표시는 정상). 재시도 중... (Ctrl+C 로 종료)")
            elif tries % 5 == 0:
                print(f"재접속 시도 {tries}회 ...")
            time.sleep(1.0)


def session(s):
    """한 번의 연결. 반환 True = 사용자가 끝냄 (EOF), False = 연결이 끊김"""
    alive = True

    def reader():
        nonlocal alive
        buf = b""
        while alive:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    print(line.decode("utf-8", errors="replace").rstrip())
                except Exception:
                    pass
        alive = False

    threading.Thread(target=reader, daemon=True).start()

    try:
        while alive:
            try:
                cmd = inq.get(timeout=0.2)
            except queue.Empty:
                continue
            if cmd is None:
                return True
            if cmd:
                try:
                    s.sendall((cmd + "\n").encode())
                except OSError:
                    break
        return False
    finally:
        alive = False


def user_quit():
    """큐에 EOF 가 들어와 있으면 True (중간에 친 명령은 버린다)"""
    while True:
        try:
            if inq.get_nowait() is None:
                return True
        except queue.Empty:
            return False


def drain():
    while True:
        try:
            if inq.get_nowait() is None:
                inq.put(None)
                return
        except queue.Empty:
            return


def main(a):
    print(f"접속 중  {a.host}:{a.tcp} ...")
    threading.Thread(target=input_thread, daemon=True).start()
    first = True
    s = None
    try:
        while True:
            s = connect(a, first)
            if s is None:
                break
            first = False
            drain()                       # 끊긴 동안 친 명령은 버린다
            print("접속됨. 명령 입력 후 Enter. 종료 Ctrl+C\n")
            if session(s):
                break
            s.close()
            s = None
            print("\n[연결 끊김] 보드가 리셋됐거나 와이파이가 끊겼다. 모터는 보드 쪽에서 멈춘다. 다시 붙는 중...")
    except KeyboardInterrupt:
        pass
    finally:
        if s is not None:
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
