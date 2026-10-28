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
 *   ESP32-S3 GPIO17 (TX) ──────────────────── 우노 D10 / 쉴드 S10 (RX)
 *   ESP32-S3 GPIO18 (RX) ◄── 2kΩ ──┬── 1kΩ ── 우노 D11 / 쉴드 S11 (TX, 5V → 3.3V 분압)
 *                                 GND
 *   ESP32-S3 GND ──────────────────────────── 우노 GND   (공통 필수)
 *
 *   GPIO18 쪽은 우노가 보내는 엔코더/출력 상태 표시용. 조작만 할 거면 빼도 된다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 명령 (TCP / USB 공용, 줄 단위)
 *
 *   w  a  s  d        전진 / 좌회전 / 후진 / 우회전. x 를 칠 때까지 계속 (시간 제한 없음)
 *                     처음 누르면 vmin(기본 70) 으로 천천히, 같은 키를 또 누를 때마다
 *                     vstep(기본 20) 씩 빨라져 v(기본 255, 하드웨어 최대) 까지.
 *                     다른 키·정지 뒤엔 다시 vmin 부터
 *   w1500             1500ms 동안 전진 후 자동 정지 (a/s/d 도 동일). 실험 사이클용. 단계 올리기는 같음
 *   x  stop  빈 줄     정지 (우노 램프 따라 서서히)
 *   !                 비상 정지 (램프 없이 즉시)
 *   v<0~255>          최고 속도 (기본 255 = PWM 최대). 주행 중이면 현재 속도를 이 값으로 바로 바꿈
 *   vmin<0~255>       첫 단계 속도 (기본 70)
 *   vstep<1~255>      한 번 더 누를 때 올라가는 양 (기본 20)
 *   hold<ms>          w 만 쳤을 때 최대 주행 시간 (기본 0 = 무제한. 혼자 멈추는 게 싫어서 끔)
 *   ramp<ms>          우노 가속 램프 (기본 700, 0→255 기준, 0 = 즉시) — 우노로 전달
 *   rampd<ms>         우노 감속 램프 (기본 1500, 255→0 기준). x 를 쳐도 이 시간에 걸쳐 선다 — 우노로 전달
 *                     실험(출발 세게, 정지 서서히): vmin255 ramp0 rampd2000 뒤 w1500
 *   ?                 상태 + 우노 마지막 보고
 *   e                 우노 엔코더 카운트 0 (우노로 전달)
 *   그 외              우노로 그대로 전달
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 안전
 *
 *   - TCP 접속이 끊기면 정지
 *   - hold<ms> 를 주면 w 만 친 경우 그 시간 뒤 정지 (기본 0 = 안 멈춤)
 *   - 우노는 M 이 300ms 안 오면 스스로 정지 (선 빠짐·이 보드 리셋)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 우노 링크 프로토콜 (Serial1, 9600, 줄 단위)
 *
 *   보드 → 우노   M <left> <right>   −255~+255. 50ms 마다 항상 (정지 중엔 M 0 0)
 *                X                  비상 정지
 *                그 외 줄            그대로 전달
 *   우노 → 보드   E <encL> <encR> <pwmL> <pwmR>   우노 SEND_STATUS 1 + S11 연결 시만. '?' 에서 표시
 *                그 외 줄                          "[uno] …" 로 표시
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 보드 설정: ESP32S3 Dev Module. COM 포트(CH343)에 꽂으면 Tools → USB CDC On Boot = Disabled (보드 1 과 동일).
 *            네이티브 USB 포트에 꽂을 때만 Enabled.
 * 라이브러리: WiFi.h (보드 패키지 내장) 뿐
 * -----------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <lwip/sockets.h>
#include <esp_system.h>

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
#define UNO_TX_PIN   17              // → 우노 D10 (쉴드 S10)
#define UNO_RX_PIN   18              // ← 우노 D11 (쉴드 S11, 분압 거쳐서)
#define UNO_BAUD     9600
#define UNO_SEND_MS  50              // M 전송 주기 (20Hz)

// =============================================================================
//  주행 상태
// =============================================================================
#define SPEED_DEFAULT   255          // 최고 속도 (v). PWM 상한 = 255 가 하드웨어 최대
#define SPEED_MIN_DEFAULT  70        // 첫 단계 속도 (vmin)
#define SPEED_STEP_DEFAULT 20        // 같은 키 한 번 더 → 이만큼 증가 (vstep). 70 90 110 130 150
#define HOLD_MS_DEFAULT 0            // w 만 쳤을 때 최대 주행 시간 (0 = 무제한). 3000 이었으나 혼자 멈춰서 끔

int      speedVal   = SPEED_DEFAULT; // 최고 속도
int      speedMin   = SPEED_MIN_DEFAULT;
int      speedStep  = SPEED_STEP_DEFAULT;
int      speedCur   = 0;             // 지금 단계의 속도 (정지 중 0)
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

// 리셋 원인 — 접속할 때와 ? 에 찍는다. 주행 중 TCP 가 끊겼으면 이걸로 보드가 죽었는지 본다
const char *resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "전원 켬";
    case ESP_RST_SW:       return "소프트 리셋 (업로드·재시작)";
    case ESP_RST_PANIC:    return "PANIC (코드 예외)";
    case ESP_RST_INT_WDT:  return "인터럽트 워치독";
    case ESP_RST_TASK_WDT: return "태스크 워치독";
    case ESP_RST_WDT:      return "워치독";
    case ESP_RST_BROWNOUT: return "BROWNOUT (전원 전압 떨어짐)";
    case ESP_RST_DEEPSLEEP:return "딥슬립";
    case ESP_RST_EXT:      return "외부 리셋 핀";
    default:               return "알 수 없음";
  }
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
  moving = 0; moveEndMs = 0; speedCur = 0;
  setCmd(0, 0);
  if (why) outPrintf("[stop] %s\n", why);
}

void emergencyStop() {
  moving = 0; moveEndMs = 0; speedCur = 0;
  cmdL = cmdR = 0;
  sendUnoCmd();
  Serial1.println("X");
  outPrintf("[stop] EMERGENCY\n");
}

/* key: 'w' 'a' 's' 'd', durMs: 0 이면 hold 설정 적용
 * bump: true 면 키 입력에 의한 호출 — 같은 키로 달리는 중이면 한 단계 올리고,
 *       아니면(처음·다른 키·정지 뒤) vmin 부터. false 면 속도 설정만 바뀐 것이라 단계 유지 */
void startDrive(char key, uint32_t durMs, bool bump) {
  int fwd = 0, turn = 0;
  switch (key) {
    case 'w': fwd =  1; break;
    case 's': fwd = -1; break;
    case 'a': turn = -1; break;
    case 'd': turn =  1; break;
  }
  if (bump) {
    if (moving == key && speedCur > 0) speedCur += speedStep;   // 같은 키 또 누름 → 한 단계 위
    else                               speedCur  = speedMin;    // 처음 / 방향 바꿈 / 정지 뒤
  }
  if (speedCur > speedVal) speedCur = speedVal;                 // v 가 상한
  int stage  = (speedCur <= speedMin) ? 1 : 1 + (speedCur - speedMin + speedStep - 1) / speedStep;
  int stages = (speedVal <= speedMin) ? 1 : 1 + (speedVal - speedMin + speedStep - 1) / speedStep;
  int left  = (fwd + turn) * speedCur;
  int right = (fwd - turn) * speedCur;
  moving = key;
  uint32_t dur = durMs ? durMs : holdMs;
  moveEndMs = dur ? millis() + dur : 0;
  setCmd(left, right);
  if (dur) outPrintf("[drive] %c  %d단/%d단 속도 %d/%d  %lu ms\n", key, stage, stages, speedCur, speedVal, (unsigned long)dur);
  else     outPrintf("[drive] %c  %d단/%d단 속도 %d/%d  (x 로 정지)\n", key, stage, stages, speedCur, speedVal);
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
    startDrive(k, dur, true);
    return;
  }
  if (strncmp(line, "vmin", 4) == 0 && isDigit(line[4])) {
    speedMin = constrain(atoi(line + 4), 0, 255);
    outPrintf("[cmd] vmin = %d (첫 단계)\n", speedMin);
    return;
  }
  if (strncmp(line, "vstep", 5) == 0 && isDigit(line[5])) {
    speedStep = constrain(atoi(line + 5), 1, 255);
    outPrintf("[cmd] vstep = %d (한 단계)\n", speedStep);
    return;
  }
  if (k == 'v' && n > 1 && isDigit(line[1])) {
    speedVal = constrain(atoi(line + 1), 0, 255);
    outPrintf("[cmd] v = %d (최고)\n", speedVal);
    if (moving) {                                    // 주행 중이면 지금 속도를 이 값으로
      speedCur = speedVal;
      startDrive(moving, moveEndMs ? (moveEndMs - millis()) : 0, false);
    }
    return;
  }
  if (strncmp(line, "hold", 4) == 0 && isDigit(line[4])) {
    holdMs = (uint32_t)atol(line + 4);
    outPrintf("[cmd] hold = %lu ms%s\n", (unsigned long)holdMs, holdMs ? "" : " (무제한)");
    return;
  }
  if (strcmp(line, "?") == 0) {
    outPrintf("[state] 가동 %lu s  리셋 원인: %s\n", (unsigned long)(millis() / 1000), resetReasonStr());
    outPrintf("[state] v %d  vmin %d  vstep %d  지금 %d  hold %lu ms  cmd L:%d R:%d  moving:%c  tcp:%s\n",
              speedVal, speedMin, speedStep, speedCur, (unsigned long)holdMs, cmdL, cmdR, moving ? moving : '-',
              (tcpClient && tcpClient.connected()) ? tcpClient.remoteIP().toString().c_str() : "미접속");
    if (lastUnoRxMs == 0)
      outPrintf("[uno]   보고 없음 (S11 선 미연결?)\n");
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
  Serial.printf("=== 리셋 원인: %s ===\n", resetReasonStr());
  Serial.println("명령: w a s d (또 누르면 가속, w1500 = 1.5초) / x 정지 / ! 비상 / v255 vmin70 vstep20 / hold0 / ramp700 rampd1500 / ?");
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
      outPrintf("보드 가동 %lu s  리셋 원인: %s\n", (unsigned long)(millis() / 1000), resetReasonStr());
      outPrintf("명령: w a s d (또 누르면 가속, w1500 = 1.5초) / x 정지 / ! 비상 / v255 vmin70 vstep20 / hold0 / ramp700 rampd1500 / ?\n");
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
