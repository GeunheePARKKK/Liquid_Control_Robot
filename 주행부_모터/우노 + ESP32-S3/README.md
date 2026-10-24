# 주행 링크 — ESP32-S3 (와이파이 시리얼 모니터) → 우노 (앞바퀴 2개)

```
노트북 PowerShell ──WiFi TCP 23──► ESP32-S3 (esp32_drive_wifi) ──UART 38400──► 우노 (uno_drive_link) ──► MDD10A ×2 ──► 앞바퀴 2개
 python drive_monitor.py                                        ◄── E 엔코더/출력 상태 ──┘
```

zv_shaping_wifi.ino 와 같은 방식. 웹 페이지 없음. WiFi `Drive_Robot` / `drive1234`, IP 192.168.4.1, TCP 23.

## ESP32-S3 ↔ 우노 배선

| ESP32-S3 | | 우노 | 비고 |
|---|---|---|---|
| **GPIO17** (TX) | → | **A4** | 직결 (3.3 V 출력, 우노가 HIGH 로 읽음) |
| **GPIO18** (RX) | ← | **A5** | 우노 5 V 출력 → **1 kΩ + 2 kΩ 분압** 필수 |
| **GND** | — | **GND** | 공통 필수 |

```
우노 A5 ──[1kΩ]──┬──► ESP32-S3 GPIO18
                 [2kΩ]
                  GND
```

- A5 선은 엔코더·실제 출력을 `?` 에서 보여주는 용도. 조작만 할 거면 GPIO17→A4, GND 두 줄로 충분.
- 우노 D0/D1(USB)을 안 쓰므로 우노 업로드·시리얼 모니터는 선 안 뽑고 그대로.
- 전원: 각자 USB, 또는 우노 5V → ESP32-S3 5V 핀 (GND 공통). 모터 전원은 MDD10A 배터리 단자로만.

## 우노 모터 배선 (주행부_모터/README.md 우노 표의 앞바퀴 부분)

| 기능 | 우노 핀 | MDD10A |
|---|---|---|
| 왼앞 PWM / DIR | **D5 / A0** | 1번 PWM1 / DIR1 |
| 오른앞 PWM / DIR | **D9 / A2** | 2번 PWM1 / DIR1 |
| 왼뒤·오른뒤 PWM (안 씀) | D6, D10 | 코드가 0 으로 고정 |
| 왼앞 엔코더 A / B | D2 / D4 | 선택 |
| 오른앞 엔코더 A / B | D3 / D7 | 선택 |

## 순서

1. **우노** `uno_drive_link.ino` 업로드 (Arduino Uno). USB 시리얼 모니터 115200, 바퀴 든 채 `f` →
   두 바퀴 다 앞으로 1초. 한쪽이 뒤로 돌면 그쪽 `L_INVERT` / `R_INVERT` = 1 로 바꿔 재업로드.
2. **ESP32-S3** `esp32_drive_wifi.ino` 업로드 (ESP32S3 Dev Module, **USB CDC On Boot = Enabled**).
3. 배선 연결 → 노트북 WiFi 를 `Drive_Robot` 에 연결 ("인터넷 없음" 표시 정상).
4. PowerShell:
   ```
   python "C:\Users\super\OneDrive\Desktop\주행_링크\drive_monitor.py"
   ```
5. `?` → `[uno] enc …` 줄이 나오면 양방향 OK (`보고 없음` 이면 A5 선 확인).

## 명령 (drive_monitor.py / ESP32 USB 시리얼 공용)

| 명령 | 동작 |
|---|---|
| `w` `a` `s` `d` | 전진 / 좌회전 / 후진 / 우회전. 다음 명령까지, 최대 `hold` 시간(기본 3초) |
| `w1500` | 1.5초 전진 후 자동 정지 (a/s/d 도 같음). 실험 사이클용 |
| `x`, `stop`, 빈 줄 | 정지 (우노 램프로 서서히) |
| `!` | 비상 정지 (램프 없이 즉시) |
| `v<0~255>` | 속도 (기본 150) |
| `hold<ms>` | `w` 만 쳤을 때 최대 주행 시간 (0 = 무제한) |
| `ramp<ms>` | 우노 램프 시간 (기본 700, 0 = 즉시). 0→255 기준 |
| `e` | 우노 엔코더 카운트 0 |
| `?` | 상태 + 우노 마지막 보고 |

램프 700 ms 는 짐벌 보상이 확실히 먹는 대역(0.7 Hz 이하)에 맞춘 값.

## 안전

1. TCP 끊김 → 정지. drive_monitor.py 를 Ctrl+C 로 끝내면 `stop` 을 보내고 닫힘
2. `w` 만 친 경우 hold 시간 지나면 정지 (노트북이 멈춰도 계속 안 달림)
3. 우노: `M` 이 300 ms 안 오면 스스로 정지 (선 빠짐·ESP32 리셋)

## 링크 프로토콜 (UART 38400, 줄 단위)

| 방향 | 메시지 | 의미 |
|---|---|---|
| ESP32 → 우노 | `M <left> <right>` | 목표 PWM −255~+255. 정지 중에도 `M 0 0` 을 50 ms 마다 |
| ESP32 → 우노 | `X` | 비상 정지 |
| ESP32 → 우노 | 그 외 줄 | 그대로 전달 (`ramp700`, `e`) |
| 우노 → ESP32 | `E <encL> <encR> <pwmL> <pwmR>` | 200 ms 마다 |
| 우노 → ESP32 | 그 외 줄 | `[uno] …` 로 표시 |
