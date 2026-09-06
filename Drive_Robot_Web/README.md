# Drive_Robot_Web — ESP32-S3 주행 로봇 웹 브라우저 리모컨

ESP32-S3가 직접 WiFi 핫스팟(AP)과 웹서버를 띄워서, 노트북/휴대폰 브라우저로
W/A/S/D 키를 눌러 실시간 주행 제어하는 리모컨 UI. 공유기·인터넷 불필요.

## 🌐 접속 주소

> ### http://192.168.4.1

⚠ **먼저 ESP32의 WiFi에 연결해야 접속됩니다.** `192.168.4.1`은 ESP32가 만든
자체 네트워크 안에서만 통하는 주소라서, 다른 WiFi나 LTE에서는 열리지 않는다.

| 항목 | 값 |
|------|-----|
| WiFi 이름 (SSID) | `Drive_Robot` |
| WiFi 비밀번호 | `drive1234` |
| 웹 UI | `http://192.168.4.1` |
| WebSocket 포트 | 81 (UI가 자동으로 연결) |

## 사용 순서

1. `drive_web_ui/drive_web_ui.ino` 를 ESP32-S3에 업로드 (보드: **ESP32S3 Dev Module**)
2. 노트북/휴대폰 WiFi 목록에서 **`Drive_Robot`** 접속 (비밀번호 `drive1234`)
3. 브라우저에서 **`http://192.168.4.1`** 열기
4. W/A/S/D — 누르고 있는 동안만 이동, 떼면 즉시 정지 (데드맨 방식)

## 기능

- **W/A/S/D 데드맨 조작** — 누르는 동안만 이동. W+A 같은 동시 입력 조합 가능
- **화면 터치 버튼** — 휴대폰에서도 리모컨으로 사용 가능
- **속도 슬라이더** (0~255)
- **시리얼 모니터 대체** — 하단 명령창에서 임의 명령 전송(`?`, `v200`, `stop` 등),
  보드 출력은 로그창에 실시간 표시. USB 시리얼(115200)도 그대로 공존
- **안전 장치 4중**
  1. 400ms 무소식이면 자동 정지 (WiFi 끊김·브라우저 멈춤 대비 워치독)
  2. 브라우저 탭 이탈/창 포커스 상실 시 정지
  3. WebSocket 연결 끊김 시 정지
  4. Space 키 / STOP 버튼 비상 정지

## 필요한 라이브러리

- **WebSockets** by Markus Sattler ([links2004/arduinoWebSockets](https://github.com/Links2004/arduinoWebSockets))
  — 라이브러리 매니저에서 `WebSockets` 검색 후 설치
- `WiFi.h` / `WebServer.h` 는 ESP32 보드 패키지 내장

## 주행 코드 연결점 (TODO)

현재 모터 출력은 스텁(로그 출력만)이라 모터 없이도 안전하게 UI 테스트 가능.
실제 주행 코드는 `drive_web_ui.ino` 의 두 함수만 채우면 된다:

| 함수 | 역할 |
|------|------|
| `setMotors(fl, fr, rl, rr)` | 4개 모터 드라이버에 PWM 출력 (−255 ~ +255, 부호 = 방향) |
| `handleLine(String)` | 시리얼/웹 명령창 공용 명령 처리 — 기존 코드의 handleLine 이식 자리 |

## 파일 구성

```
drive_web_ui/
├── drive_web_ui.ino   펌웨어 — WiFi AP + HTTP 서버 + WebSocket + 데드맨 워치독
└── web_ui.h           내장 웹 UI (HTML/CSS/JS) — 수정 시 재업로드 필요
```

## WebSocket 프로토콜 (포트 81)

| 방향 | 메시지 | 의미 |
|------|--------|------|
| 브라우저 → 보드 | `k:<mask>` | 키 상태 비트마스크 (bit0=W, bit1=A, bit2=S, bit3=D). 눌린 동안 100ms마다 재전송 |
| 브라우저 → 보드 | `v:<0~255>` | 속도 |
| 브라우저 → 보드 | `c:<text>` | 임의 명령줄 (시리얼 모니터와 동일 처리) |
| 브라우저 → 보드 | `x` | 비상 정지 |
| 보드 → 브라우저 | 텍스트 줄 | UI 로그창에 표시 |
