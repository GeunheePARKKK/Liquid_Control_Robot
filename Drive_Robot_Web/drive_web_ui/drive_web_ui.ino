/*
 * drive_web_ui.ino
 * -----------------------------------------------------------------------------
 * ESP32-S3 주행 로봇 — 웹 브라우저 리모컨 (WiFi AP + 웹서버 + WebSocket)
 *
 * zv_shaping_wifi.ino 와 같은 방식으로 보드가 직접 와이파이(AP)를 띄운다.
 * 공유기가 없어도 된다.
 *
 *   1. 노트북/휴대폰을 WiFi "Drive_Robot" (비밀번호 drive1234) 에 연결
 *   2. 브라우저에서  http://192.168.4.1  열기
 *   3. W/A/S/D 를 누르고 있는 동안만 이동, 떼면 즉시 정지 (데드맨 방식)
 *      화면의 방향 버튼을 터치해도 된다 (휴대폰용)
 *
 * USB 시리얼(115200)은 그대로 살아 있다. 출력은 USB 와 웹 UI 로그창에
 * 같이 나가고, 명령 입력은 어느 쪽에서든 받는다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 데드맨(dead-man) 안전 장치
 *
 *   브라우저는 키가 눌린 동안 100ms 마다 현재 키 상태를 재전송한다.
 *   움직이는 중에 KEY_TIMEOUT_MS 동안 아무 메시지가 없으면 (WiFi 끊김,
 *   브라우저 멈춤, 노트북 절전 등) 즉시 전체 정지한다.
 *   브라우저 탭이 백그라운드로 가거나 창 포커스를 잃어도 JS 쪽에서
 *   전체 정지를 보낸다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * WebSocket 프로토콜 (포트 81)
 *
 *   브라우저 → 보드
 *     k:<mask>   키 상태 비트마스크. bit0=W bit1=A bit2=S bit3=D
 *                (변할 때 즉시 + 눌린 동안 100ms 마다)
 *     v:<0~255>  속도 (UI 슬라이더)
 *     c:<text>   임의 명령줄 — 시리얼 모니터에 치던 것과 동일하게 처리
 *     x          비상 정지
 *
 *   보드 → 브라우저
 *     텍스트 줄 그대로 (webPrintf 로 보낸 것) → UI 로그창에 표시
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 필요한 라이브러리 (라이브러리 매니저에서 설치)
 *
 *   "WebSockets" by Markus Sattler  (links2004/arduinoWebSockets)
 *
 *   WiFi.h / WebServer.h 는 ESP32 보드 패키지에 내장.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ★ 모터 연결 자리
 *
 *   아래 setMotors() / driveFromMask() 두 함수만 채우면 된다.
 *   지금은 어떤 PWM 이 나가야 하는지 시리얼/웹 로그로 출력만 한다.
 * -----------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "web_ui.h"          // 내장 웹 UI (HTML/JS)

// =============================================================================
//  와이파이
// =============================================================================
#define WIFI_SSID  "Drive_Robot"
#define WIFI_PASS  "drive1234"       // 8자 이상

WebServer        httpServer(80);     // UI 페이지를 내려주는 HTTP 서버
WebSocketsServer wsServer(81);       // 키 입력·로그 실시간 통신

// =============================================================================
//  데드맨 / 키 상태
// =============================================================================
#define KEY_TIMEOUT_MS  400          // 움직이는 중 이 시간 동안 무소식이면 정지

// 키 비트마스크. bit0=W(전진) bit1=A(좌회전) bit2=S(후진) bit3=D(우회전)
#define KEY_W  0x01
#define KEY_A  0x02
#define KEY_S  0x04
#define KEY_D  0x08

volatile uint8_t  keyMask    = 0;    // 지금 눌려 있는 키
volatile uint32_t lastKeyMs  = 0;    // 마지막 k: 메시지 시각
uint8_t           speedVal   = 150;  // 0~255, UI 슬라이더로 조절

// =============================================================================
//  출력 — USB 시리얼과 웹 로그창에 같이 나간다
// =============================================================================
void webPrintf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  wsServer.broadcastTXT(buf);
}

// =============================================================================
//  ★ 모터 — 여기를 실제 모터 드라이버 코드로 채운다
// =============================================================================
/* 4개 모터에 PWM 을 내보낸다. 값 범위 −255 ~ +255 (부호 = 방향).
 *   fl = 앞왼쪽, fr = 앞오른쪽, rl = 뒤왼쪽, rr = 뒤오른쪽
 *
 * TODO: 실제 모터 드라이버(예: ledcWrite + 방향핀)로 교체
 */
void setMotors(int fl, int fr, int rl, int rr) {
  static int pfl = 9999, pfr, prl, prr;
  if (fl == pfl && fr == pfr && rl == prl && rr == prr) return;  // 변할 때만 출력
  pfl = fl; pfr = fr; prl = rl; prr = rr;
  webPrintf("[motor] FL:%4d FR:%4d RL:%4d RR:%4d\n", fl, fr, rl, rr);
}

/* 키 조합 → 4륜 구동 명령 (스키드 스티어 기준)
 * W+A 처럼 동시 입력도 자연스럽게 섞인다.
 */
void driveFromMask(uint8_t mask) {
  int v = speedVal;
  int fwd  = ((mask & KEY_W) ? 1 : 0) - ((mask & KEY_S) ? 1 : 0);  // 전진 −1~+1
  int turn = ((mask & KEY_D) ? 1 : 0) - ((mask & KEY_A) ? 1 : 0);  // 우회전 +

  int left  = constrain((fwd + turn) * v, -255, 255);
  int right = constrain((fwd - turn) * v, -255, 255);

  setMotors(left, right, left, right);
}

void allStop() {
  keyMask = 0;
  setMotors(0, 0, 0, 0);
}

// =============================================================================
//  명령줄 처리 — 시리얼 모니터/웹 명령창 공용
// =============================================================================
/* 시리얼 모니터에 치던 명령을 여기서 처리한다.
 * TODO: 기존 주행 코드의 handleLine 내용을 여기로 옮기면 된다.
 */
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "?") {
    webPrintf("[state] speed:%d keys:0x%02X clients:%d\n",
              speedVal, keyMask, wsServer.connectedClients());
  } else if (line == "x" || line == "stop") {
    allStop();
    webPrintf("[cmd] all stop\n");
  } else if (line.startsWith("v")) {
    speedVal = constrain(line.substring(1).toInt(), 0, 255);
    webPrintf("[cmd] speed = %d\n", speedVal);
  } else {
    webPrintf("[cmd] unknown: %s\n", line.c_str());
  }
}

// =============================================================================
//  WebSocket 이벤트
// =============================================================================
void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      webPrintf("[ws] client %u connected (%s)\n",
                num, wsServer.remoteIP(num).toString().c_str());
      break;

    case WStype_DISCONNECTED:
      webPrintf("[ws] client %u disconnected\n", num);
      allStop();               // 제어하던 창이 사라지면 무조건 정지
      break;

    case WStype_TEXT: {
      String msg((char *)payload, len);

      if (msg.startsWith("k:")) {              // 키 상태
        keyMask   = (uint8_t)msg.substring(2).toInt();
        lastKeyMs = millis();
        driveFromMask(keyMask);
      } else if (msg.startsWith("v:")) {       // 속도 슬라이더
        speedVal = constrain(msg.substring(2).toInt(), 0, 255);
        if (keyMask) driveFromMask(keyMask);   // 주행 중이면 즉시 반영
      } else if (msg.startsWith("c:")) {       // 임의 명령줄
        handleLine(msg.substring(2));
      } else if (msg == "x") {                 // 비상 정지
        allStop();
        webPrintf("[ws] EMERGENCY STOP\n");
      }
      break;
    }
    default:
      break;
  }
}

// =============================================================================
//  setup / loop
// =============================================================================
void setup() {
  Serial.begin(115200);

  // ── 와이파이 AP ──
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.printf("\n[wifi] AP \"%s\" started, IP: %s\n",
                WIFI_SSID, WiFi.softAPIP().toString().c_str());

  // ── HTTP: UI 페이지 ──
  httpServer.on("/", []() {
    httpServer.send_P(200, "text/html", WEB_UI_HTML);
  });
  httpServer.begin();

  // ── WebSocket ──
  wsServer.begin();
  wsServer.onEvent(onWsEvent);

  Serial.println("[http] open http://192.168.4.1");
  allStop();
}

void loop() {
  httpServer.handleClient();
  wsServer.loop();

  // ── 데드맨 워치독: 움직이는 중에 무소식이면 정지 ──
  if (keyMask != 0 && millis() - lastKeyMs > KEY_TIMEOUT_MS) {
    allStop();
    webPrintf("[watchdog] key timeout -> stop\n");
  }

  // ── USB 시리얼 입력도 그대로 받는다 ──
  static String serialLine;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length()) handleLine(serialLine);
      serialLine = "";
    } else {
      serialLine += c;
    }
  }
}
