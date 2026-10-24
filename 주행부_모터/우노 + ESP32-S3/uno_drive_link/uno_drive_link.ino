/*
 * uno_drive_link.ino
 * -----------------------------------------------------------------------------
 * 아두이노 우노 — 주행 보드 (앞바퀴 2개, MDD10A 2개)
 *
 * ESP32-S3(esp32_drive_wifi)가 와이파이 TCP 로 받은 w/a/s/d 를
 * UART 한 줄로 넘겨주면, 이 보드가 실제로 모터를 돌린다.
 * 엔코더가 5V 라서 ESP32-S3 에 직접 못 붙이므로 모터·엔코더는 우노에 그대로 둔다.
 *
 * 핀은 주행부_모터/README.md "우노 배선과의 대응" 표의 앞바퀴 2개 그대로.
 * 뒷바퀴 채널(D6/A1, D10/A3)은 안 쓰지만 선이 아직 꽂혀 있을 수 있어 PWM 0 으로 묶는다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ESP32-S3 ↔ 우노 배선
 *
 *   ESP32-S3 GPIO17 (TX) ──────────────────── 우노 A4 (RX, SoftwareSerial)
 *   ESP32-S3 GPIO18 (RX) ◄── 2kΩ ──┬── 1kΩ ── 우노 A5 (TX, 5V → 3.3V 분압)
 *                                 GND
 *   ESP32-S3 GND ──────────────────────────── 우노 GND   (공통 필수)
 *
 *   우노 A5 쪽 선은 엔코더 값을 웹에 보여주는 용도라 조작만 할 거면 빼도 된다.
 *   D0/D1(USB 시리얼)을 안 쓰므로 우노 업로드·시리얼 모니터는 선 안 뽑고 그대로 된다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 링크 프로토콜 (38400, 줄 단위)
 *
 *   ESP32 → 우노
 *     M <left> <right>   −255~+255. 부호 = 방향. 키가 눌려 있든 아니든 50ms 마다 온다
 *     X                  비상 정지 (램프 없이 즉시 0)
 *     ramp<ms>           램프 시간 변경 (0 = 램프 없음)
 *     ramp<ms> e ?       램프 변경 / 엔코더 0 / 상태
 *     (f b l r 테스트는 USB 전용 — ESP32 가 보내는 M 에 바로 덮인다)
 *   우노 → ESP32
 *     E <encL> <encR> <pwmL> <pwmR>   200ms 마다 상태
 *     그 외 텍스트 줄                   ESP32 가 "[uno] ..." 로 웹 로그에 표시
 *
 *   300ms 동안 유효한 M 이 안 오면 즉시 정지 (선 빠짐·ESP32 리셋 대비).
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 램프
 *
 *   목표 PWM 이 바뀌면 RAMP_MS 동안 서서히 따라간다 (0→255 기준).
 *   짐벌 보상이 확실히 먹는 대역이 0.7Hz 아래라 기본 700ms. 키를 떼도 같은 램프로 선다.
 *   X(비상 정지)와 링크 타임아웃만 램프 없이 즉시 0.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * USB 시리얼(115200) 테스트 명령 — 우노만 USB 로 꽂고 바퀴 들고 확인
 *
 *   f / b / l / r   1초 동안 전진/후진/좌회전/우회전 (PWM 100)
 *   s               정지
 *   ?               상태
 *   e               엔코더 카운트 0 으로
 *   ramp<ms>        램프 시간 (예: ramp0, ramp700)
 *
 *   'f' 를 쳤는데 한쪽 바퀴가 뒤로 돌면 그쪽 *_INVERT 를 1 로 바꾼다.
 * -----------------------------------------------------------------------------
 */

#include <SoftwareSerial.h>

// =============================================================================
//  핀
// =============================================================================
#define L_PWM    5     // 왼앞   PWM  (MDD10A 1 · PWM1)
#define L_DIR    A0    // 왼앞   DIR  (MDD10A 1 · DIR1)
#define R_PWM    9     // 오른앞 PWM  (MDD10A 2 · PWM1)
#define R_DIR    A2    // 오른앞 DIR  (MDD10A 2 · DIR1)
#define RL_PWM   6     // 왼뒤   PWM  — 안 씀, 0 고정
#define RR_PWM   10    // 오른뒤 PWM  — 안 씀, 0 고정

#define L_INVERT 0     // 'f' 테스트에서 왼쪽이 뒤로 돌면 1
#define R_INVERT 0     // 'f' 테스트에서 오른쪽이 뒤로 돌면 1

#define ENC_L_A  2     // 왼앞  엔코더 A (INT0)
#define ENC_L_B  4     // 왼앞  엔코더 B
#define ENC_R_A  3     // 오른앞 엔코더 A (INT1)
#define ENC_R_B  7     // 오른앞 엔코더 B

#define LINK_RX  A4    // ← ESP32 GPIO17
#define LINK_TX  A5    // → 분압 → ESP32 GPIO18

// =============================================================================
//  설정
// =============================================================================
#define LINK_BAUD        38400
#define LINK_TIMEOUT_MS  300    // M 이 이만큼 안 오면 정지
#define RAMP_MS_DEFAULT  700    // 0→255 램프 시간
#define RAMP_TICK_MS     10
#define STATUS_MS        200    // E 상태 전송 주기
#define TEST_PWM         100
#define TEST_HOLD_MS     1000

SoftwareSerial link(LINK_RX, LINK_TX);

volatile long encL = 0, encR = 0;
int      tgtL = 0, tgtR = 0;      // 목표 PWM (−255~255)
int      curL = 0, curR = 0;      // 램프 적용 후 실제 출력
uint16_t rampMs      = RAMP_MS_DEFAULT;
uint32_t lastCmdMs   = 0;         // 마지막 유효 M
uint32_t lastRampMs  = 0;
uint32_t lastStatMs  = 0;
uint32_t testUntilMs = 0;         // USB 테스트 명령 유지 시각 (0 = 테스트 아님)
bool     linkAlive   = false;

// =============================================================================
//  엔코더 ISR — A 상승 에지에서 B 로 방향
// =============================================================================
void isrEncL() { if (digitalRead(ENC_L_B)) encL++; else encL--; }
void isrEncR() { if (digitalRead(ENC_R_B)) encR++; else encR--; }

// =============================================================================
//  모터 출력
// =============================================================================
void outMotor(uint8_t pwmPin, uint8_t dirPin, int v, bool inv) {
  bool fwd = (v >= 0);
  if (inv) fwd = !fwd;
  digitalWrite(dirPin, fwd ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(v), 0, 255));
}

void applyOutput() {
  outMotor(L_PWM, L_DIR, curL, L_INVERT);
  outMotor(R_PWM, R_DIR, curR, R_INVERT);
}

void hardStop() {
  tgtL = tgtR = curL = curR = 0;
  applyOutput();
}

int stepToward(int cur, int tgt, int step) {
  if (cur < tgt) return min(cur + step, tgt);
  if (cur > tgt) return max(cur - step, tgt);
  return cur;
}

void rampTick() {
  uint32_t now = millis();
  if (now - lastRampMs < RAMP_TICK_MS) return;
  lastRampMs = now;
  if (rampMs == 0) {
    curL = tgtL; curR = tgtR;
  } else {
    int step = max(1, (int)((255L * RAMP_TICK_MS + rampMs / 2) / rampMs));
    curL = stepToward(curL, tgtL, step);
    curR = stepToward(curR, tgtR, step);
  }
  applyOutput();
}

// =============================================================================
//  명령 처리 — 링크(ESP32)와 USB 공용
// =============================================================================
void setTarget(int l, int r) {
  tgtL = constrain(l, -255, 255);
  tgtR = constrain(r, -255, 255);
}

void startTest(int l, int r, const __FlashStringHelper *name, Stream &out) {
  setTarget(l, r);
  testUntilMs = millis() + TEST_HOLD_MS;
  out.print(F("[test] ")); out.println(name);
}

void printState(Stream &out) {
  out.print(F("[state] tgt "));  out.print(tgtL); out.print(' '); out.print(tgtR);
  out.print(F("  out "));        out.print(curL); out.print(' '); out.print(curR);
  out.print(F("  enc "));        out.print(encL); out.print(' '); out.print(encR);
  out.print(F("  ramp "));       out.print(rampMs);
  out.print(F("ms  link "));     out.println(linkAlive ? F("up") : F("down"));
}

void handleLine(char *line, Stream &out, bool fromLink) {
  while (*line == ' ') line++;
  size_t n = strlen(line);
  while (n && (line[n - 1] == ' ' || line[n - 1] == '\r')) line[--n] = 0;
  if (n == 0) return;

  if (line[0] == 'M' && line[1] == ' ') {              // M <l> <r>
    char *end;
    long l = strtol(line + 2, &end, 10);
    if (end == line + 2) return;                       // 숫자 아님 → 무시
    long r = strtol(end, &end, 10);
    setTarget((int)l, (int)r);
    if (fromLink) lastCmdMs = millis();
    testUntilMs = 0;
    return;
  }
  if (strcmp(line, "X") == 0 || strcmp(line, "x") == 0) {
    hardStop();
    testUntilMs = 0;
    out.println(F("[cmd] emergency stop"));
    return;
  }
  if (strncmp(line, "ramp", 4) == 0) {
    rampMs = (uint16_t)constrain(atol(line + 4), 0, 10000);
    out.print(F("[cmd] ramp = ")); out.print(rampMs); out.println(F(" ms"));
    return;
  }
  if (n == 1) {
    switch (line[0]) {
      case 'f': startTest( TEST_PWM,  TEST_PWM, F("forward"),  out); return;
      case 'b': startTest(-TEST_PWM, -TEST_PWM, F("backward"), out); return;
      case 'l': startTest(-TEST_PWM,  TEST_PWM, F("left"),     out); return;
      case 'r': startTest( TEST_PWM, -TEST_PWM, F("right"),    out); return;
      case 's': setTarget(0, 0); testUntilMs = 0; out.println(F("[test] stop")); return;
      case '?': printState(out); return;
      case 'e': noInterrupts(); encL = encR = 0; interrupts(); out.println(F("[cmd] enc zero")); return;
    }
  }
  out.print(F("[cmd] unknown: ")); out.println(line);
}

// 줄 단위 수신 버퍼
struct LineBuf {
  char buf[32];
  uint8_t len = 0;
  bool feed(char c, char *&outLine) {
    if (c == '\n' || c == '\r') {
      if (len == 0) return false;
      buf[len] = 0; len = 0; outLine = buf; return true;
    }
    if (len < sizeof(buf) - 1) buf[len++] = c;
    else len = 0;                                     // 넘치면 버림
    return false;
  }
};
LineBuf linkBuf, usbBuf;

// =============================================================================
//  setup / loop
// =============================================================================
void setup() {
  pinMode(L_PWM, OUTPUT);  pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT);  pinMode(R_DIR, OUTPUT);
  pinMode(RL_PWM, OUTPUT); pinMode(RR_PWM, OUTPUT);
  analogWrite(RL_PWM, 0);  analogWrite(RR_PWM, 0);   // 뒷바퀴 채널 0 고정
  hardStop();

  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, RISING);

  Serial.begin(115200);
  link.begin(LINK_BAUD);
  Serial.println(F("[uno] drive link ready (front 2WD). f/b/l/r/s/?/e/ramp<ms>"));
  link.println(F("[uno] drive link ready"));
}

void loop() {
  uint32_t now = millis();
  char *line;

  // ── ESP32 링크 수신 ──
  while (link.available()) {
    if (linkBuf.feed((char)link.read(), line)) handleLine(line, link, true);
  }
  // ── USB 시리얼 수신 ──
  while (Serial.available()) {
    if (usbBuf.feed((char)Serial.read(), line)) handleLine(line, Serial, false);
  }

  // ── 링크 상태 전이 ──
  bool alive = (lastCmdMs != 0) && (now - lastCmdMs < LINK_TIMEOUT_MS);
  if (alive != linkAlive) {
    linkAlive = alive;
    Serial.println(alive ? F("[link] up") : F("[link] down"));
  }

  // ── USB 테스트 1초 종료 ──
  bool testing = (testUntilMs != 0);
  if (testing && (int32_t)(now - testUntilMs) >= 0) {
    testUntilMs = 0; testing = false; setTarget(0, 0);
  }

  // ── 워치독: 링크 끊기면 즉시 정지 (USB 테스트 중은 예외) ──
  if (!testing && !alive && (tgtL || tgtR || curL || curR)) {
    hardStop();
    Serial.println(F("[wd] link timeout -> stop"));
  }

  rampTick();

  // ── 상태 전송 ──
  if (now - lastStatMs >= STATUS_MS) {
    lastStatMs = now;
    long l, r;
    noInterrupts(); l = encL; r = encR; interrupts();
    link.print(F("E ")); link.print(l); link.print(' '); link.print(r);
    link.print(' ');     link.print(curL); link.print(' '); link.println(curR);
  }
}
