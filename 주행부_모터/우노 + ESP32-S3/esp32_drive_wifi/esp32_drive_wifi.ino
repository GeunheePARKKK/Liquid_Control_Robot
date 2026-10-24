/*
 * esp32_drive_wifi.ino
 * -----------------------------------------------------------------------------
 * ESP32-S3 주행 중계 보드 — 와이파이 시리얼 모니터(TCP) 로 w/a/s/d 를 받아
 * 아두이노 우노(uno_drive_link)에 UART 로 넘긴다.
 *
 * zv_shaping_wifi.ino 와 같은 방식: 보드가 AP 를 띄우고 TCP 23번 포트를 연다.
 * 노트북은 PowerShell 에서 drive_monitor.py(= wifi_monitor.py) 로 붙어 시리얼 모니터처럼 쓴다.
 * USB 시리얼(115200)도 그대로 살아 있고, 명령은 어느 쪽에서든 받는다.
 *
 *   1. 노트북 와이파이를 "Drive_Robot" (비밀번호 drive1234) 에 연결
 *   2. python drive_monitor.py
 *   3. w / a / s / d 치고 Enter → 움직임.  x → 정지.
 *
 * 모터·엔코더는 우노에 그대로 달려 있다 (엔코더 5V 라 ESP32-S3 에 직접 못 붙임).
 * 이 보드는 "M <left> <right>" 를 50ms 마다 우노에 보내기만 한다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ESP32-S3 ↔ 우노 배선
 *
 *   ESP32-S3 GPIO17 (TX) ──────────────────── 우노 A4 (RX)
 *   ESP32-S3 GPIO18 (RX) ◄── 2kΩ ──┬── 1kΩ ── 우노 A5 (TX, 5V → 3.3V 분압)
 *                                 GND
 *   ESP32-S3 GND ──────────────────────────── 우노 GND   (공통 필수)
 *
 *   GPIO18 쪽은 우노가 보내는 엔코더/출력 상태 표시용. 조작만 할 거면 빼도 된다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 명령 (TCP / USB 공용, 줄 단위)
 *
 *   w  a  s  d        전진 / 좌회전 / 후진 / 우회전. 다음 명령이 올 때까지 (최대 hold 시간)
 *   w1500             1500ms 동안 전진 후 자동 정지 (a/s/d 도 동일). 실험 사이클용
 *   x  stop  빈 줄     정지 (우노 램프 따라 서서히)
 *   !                 비상 정지 (램프 없이 즉시)
 *   v<0~255>          속도 (기본 150)
 *   hold<ms>          w 만 쳤을 때 최대 주행 시간 (기본 3000, 0 = 무제한)
 *   ramp<ms>          우노 램프 시간 (기본 700, 0 = 즉시) — 우노로 전달
 *   ?                 상태 + 우노 마지막 보고
 *   e                 우노 엔코더 카운트 0 (우노로 전달)
 *   그 외              우노로 그대로 전달
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 안전
 *
 *   - TCP 접속이 끊기면 정지
 *   - w 만 친 경우 hold 시간(기본 3초)이 지나면 정지 — 노트북이 멈춰도 계속 달리지 않게
 *   - 우노는 M 이 300ms 안 오면 스스로 정지 (선 빠짐·이 보드 리셋)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 우노 링크 프로토콜 (Serial1, 38400, 줄 단위)
 *
 *   보드 → 우노   M <left> <right>   −255~+255. 50ms 마다 항상 (정지 중엔 M 0 0)
 *                X                  비상 정지
 *                그 외 줄            그대로 전달
 *   우노 → 보드   E <encL> <encR> <pwmL> <pwmR>   200ms 마다 → '?' 에서 표시
 *                그 외 줄                          "[uno] …" 로 표시
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 보드 설정: ESP32S3 Dev Module, Tools → USB CDC On Boot = Enabled
 * 라이브러리: WiFi.h (보드 패키지 내장) 뿐
 * -----------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <lwip/sockets.h>

// =============================================================================
//  와이파이 / TCP
// =============================================================================
#define WIFI_SSID   "Drive_Robot"
#define WIFI_PASS   "drive1234"      // 8자 이상
#define TCP_PORT    23

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// =============================================================================
//  우노 링크 (Serial1)
// =============================================================================
#define UNO_TX_PIN   17              // → 우노 A4
#define UNO_RX_PIN   18              // ← 우노 A5 (분압 거쳐서)
#define UNO_BAUD     38400
#define UNO_SEND_MS  50              // M 전송 주기 (20Hz)

// =============================================================================
//  주행 상태
// =============================================================================
#define SPEED_DEFAULT   150
#define HOLD_MS_DEFAULT 3000         // w 만 쳤을 때 최대 주행 시간 (0 = 무제한)

int      speedVal   = SPEED_DEFAULT;
uint32_t holdMs     = HOLD_MS_DEFAULT;
int      cmdL = 0, cmdR = 0;         // 우노에 보낼 목표 PWM
char     moving     = 0;             // 'w' 'a' 's' 'd' 또는 0
uint32_t moveEndMs  = 0;             // 0 = 끝 시각 없음
uint32_t lastUnoSendMs = 0;
uint32_t lastUnoRxMs   = 0;
long     unoEncL = 0, unoEncR = 0;
int      unoPwmL = 0, unoPwmR = 0;

// =============================================================================
//  출력 — USB 와 TCP 에 같이
// =============================================================================
void outPrintf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  Serial.write((uint8_t *)buf, n);
  if (tcpClient && tcpClient.connected()) send(tcpClient.fd(), buf, n, MSG_DONTWAIT);
}

// =============================================================================
//  우노로
// =============================================================================
void sendUnoCmd() {
  Serial1.printf("M %d %d\n", cmdL, cmdR);
  lastUnoSendMs = millis();
}

void setCmd(int l, int r) {
  l = constrain(l, -255, 255); r = constrain(r, -255, 255);
  if (l == cmdL && r == cmdR) return;
  cmdL = l; cmdR = r;
  sendUnoCmd();
  outPrintf("[motor] L:%4d R:%4d\n", cmdL, cmdR);
}

void stopDrive(const char *why) {
  moving = 0; moveEndMs = 0;
  setCmd(0, 0);
  if (why) outPrintf("[stop] %s\n", why);
}

void emergencyStop() {
  moving = 0; moveEndMs = 0;
  cmdL = cmdR = 0;
  sendUnoCmd();
  Serial1.println("X");
  outPrintf("[stop] EMERGENCY\n");
}

/* key: 'w' 'a' 's' 'd', durMs: 0 이면 hold 설정 적용 */
void startDrive(char key, uint32_t durMs) {
  int fwd = 0, turn = 0;
  switch (key) {
    case 'w': fwd =  1; break;
    case 's': fwd = -1; break;
    case 'a': turn = -1; break;
    case 'd': turn =  1; break;
  }
  int left  = (fwd + turn) * speedVal;
  int right = (fwd - turn) * speedVal;
  moving = key;
  uint32_t dur = durMs ? durMs : holdMs;
  moveEndMs = dur ? millis() + dur : 0;
  setCmd(left, right);
  if (dur) outPrintf("[drive] %c  %lu ms\n", key, (unsigned long)dur);
  else     outPrintf("[drive] %c  (x 로 정지)\n", key);
}

// =============================================================================
//  우노 → 보드
// =============================================================================
void handleUnoLine(char *line) {
  if (strncmp(line, "E ", 2) == 0) {
    long a, b; int c, d;
    if (sscanf(line + 2, "%ld %ld %d %d", &a, &b, &c, &d) == 4) {
      unoEncL = a; unoEncR = b; unoPwmL = c; unoPwmR = d;
      lastUnoRxMs = millis();
    }
    return;
  }
  outPrintf("[uno] %s\n", line);
}

// =============================================================================
//  명령 처리 — TCP / USB 공용
// =============================================================================
void handleLine(char *line) {
  // 앞뒤 공백 제거
  while (*line == ' ' || *line == '\t') line++;
  size_t n = strlen(line);
  while (n && (line[n - 1] == ' ' || line[n - 1] == '\r' || line[n - 1] == '\t')) line[--n] = 0;

  if (n == 0 || strcmp(line, "x") == 0 || strcmp(line, "stop") == 0) {
    stopDrive(n == 0 ? "빈 줄" : line);
    return;
  }
  if (strcmp(line, "!") == 0) { emergencyStop(); return; }

  char k = line[0];
  if ((k == 'w' || k == 'a' || k == 's' || k == 'd') && (n == 1 || isDigit(line[1]))) {
    uint32_t dur = (n > 1) ? (uint32_t)atol(line + 1) : 0;
    startDrive(k, dur);
    return;
  }
  if (k == 'v' && n > 1 && isDigit(line[1])) {
    speedVal = constrain(atoi(line + 1), 0, 255);
    outPrintf("[cmd] speed = %d\n", speedVal);
    if (moving) startDrive(moving, moveEndMs ? (moveEndMs - millis()) : 0);  // 주행 중이면 즉시 반영
    return;
  }
  if (strncmp(line, "hold", 4) == 0 && isDigit(line[4])) {
    holdMs = (uint32_t)atol(line + 4);
    outPrintf("[cmd] hold = %lu ms%s\n", (unsigned long)holdMs, holdMs ? "" : " (무제한)");
    return;
  }
  if (strcmp(line, "?") == 0) {
    outPrintf("[state] speed %d  hold %lu ms  cmd L:%d R:%d  moving:%c  tcp:%s\n",
              speedVal, (unsigned long)holdMs, cmdL, cmdR, moving ? moving : '-',
              (tcpClient && tcpClient.connected()) ? tcpClient.remoteIP().toString().c_str() : "미접속");
    if (lastUnoRxMs == 0)
      outPrintf("[uno]   보고 없음 (A5 선 미연결?)\n");
    else
      outPrintf("[uno]   enc L:%ld R:%ld  out L:%d R:%d  (%lu ms 전)\n",
                unoEncL, unoEncR, unoPwmL, unoPwmR, (unsigned long)(millis() - lastUnoRxMs));
    return;
  }
  // 나머지 (ramp700, e …) 는 우노로
  Serial1.println(line);
  outPrintf("[cmd] -> uno: %s\n", line);
}

// 줄 버퍼
struct LineBuf {
  char buf[96];
  size_t len = 0;
  bool feed(char c, char *&out) {
    if (c == '\n' || c == '\r') {
      if (len == 0) return false;
      buf[len] = 0; len = 0; out = buf; return true;
    }
    if (len < sizeof(buf) - 1) buf[len++] = c; else len = 0;
    return false;
  }
};
LineBuf usbBuf, tcpBuf, unoBuf;

// =============================================================================
//  setup / loop
// =============================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  tcpServer.begin();
  tcpServer.setNoDelay(true);

  Serial.printf("\n=== 와이파이 AP  SSID %s  비밀번호 %s  IP %s  TCP %d ===\n",
                WIFI_SSID, WIFI_PASS, WiFi.softAPIP().toString().c_str(), TCP_PORT);
  Serial.printf("=== 우노 링크 Serial1 TX=%d RX=%d %d baud ===\n", UNO_TX_PIN, UNO_RX_PIN, UNO_BAUD);
  Serial.println("명령: w a s d (w1500 = 1.5초) / x 정지 / ! 비상 / v150 / hold3000 / ramp700 / ?");
  emergencyStop();
}

void loop() {
  uint32_t now = millis();
  char *line;

  // ── TCP 접속 수락 (한 명만) ──
  if (tcpServer.hasClient()) {
    if (tcpClient && tcpClient.connected()) {
      tcpServer.available().stop();
    } else {
      tcpClient = tcpServer.available();
      tcpClient.setNoDelay(true);
      outPrintf(">>> TCP 접속 %s\n", tcpClient.remoteIP().toString().c_str());
      outPrintf("명령: w a s d (w1500 = 1.5초) / x 정지 / ! 비상 / v150 / hold3000 / ramp700 / ?\n");
    }
  }
  // ── TCP 끊김 → 정지 ──
  static bool tcpWas = false;
  bool tcpNow = tcpClient && tcpClient.connected();
  if (tcpWas && !tcpNow) {
    if (moving) stopDrive("TCP 끊김");
    Serial.println(">>> TCP 끊김");
    tcpBuf.len = 0;
  }
  tcpWas = tcpNow;

  // ── 입력: TCP / USB ──
  while (tcpNow && tcpClient.available()) {
    if (tcpBuf.feed((char)tcpClient.read(), line)) handleLine(line);
  }
  while (Serial.available()) {
    if (usbBuf.feed((char)Serial.read(), line)) handleLine(line);
  }
  // ── 우노에서 오는 줄 ──
  while (Serial1.available()) {
    if (unoBuf.feed((char)Serial1.read(), line)) handleUnoLine(line);
  }

  // ── 주행 시간 만료 ──
  if (moving && moveEndMs && (int32_t)(now - moveEndMs) >= 0) stopDrive("시간 만료");

  // ── 우노에 M 을 50ms 마다 항상 (우노 쪽 링크 워치독 유지) ──
  if (now - lastUnoSendMs >= UNO_SEND_MS) sendUnoCmd();
}
