/*
 * uno_drive_link.ino
 * -----------------------------------------------------------------------------
 * 아두이노 우노 — 주행 보드 (앞바퀴 2개, MDD10A 2개)
 *
 * ESP32-S3(esp32_drive_wifi)가 와이파이 TCP 로 받은 w/a/s/d 를
 * UART 한 줄로 넘겨주면, 이 보드가 실제로 모터를 돌린다.
 * 엔코더가 5V 라서 ESP32-S3 에 직접 못 붙이므로 모터·엔코더는 우노에 그대로 둔다.
 *
 * 배선 (2026-10-24 실물 + 예전 2WD 코드 기준, 쉴드 S핀 번호 = 우노 D핀 번호):
 *   MDD10A 한 장의 두 채널로 앞바퀴 2개 구동
 *   앞왼(FL)  PWM / DIR = D6 / D4   (MDD10A PWM2 / DIR2, 쉴드 S6 / S4)  전진 = DIR LOW
 *   앞오른(FR) PWM / DIR = D9 / D8   (MDD10A PWM1 / DIR1, 쉴드 S9 / S8)  전진 = DIR HIGH
 *   엔코더  FL B상 D3 (인터럽트), FR B상 D7 (폴링). A상(D2, D5)은 불량이라 안 씀
 *   예전 코드의 시작 스위치 A5 — 이 코드에선 안 쓰지만 꽂혀 있어도 됨
 *   ※ 주행부_모터/README.md 의 "우노 배선과의 대응" 표(D5/A0, D9/A2 …)와 다르다. 실물이 기준.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ESP32-S3 ↔ 우노 배선
 *
 *   ESP32-S3 GPIO17 (TX) ──────────────────── 우노 D10 / 쉴드 S10 (RX, SoftwareSerial)
 *   ESP32-S3 GPIO18 (RX) ◄── 2kΩ ──┬── 1kΩ ── 우노 D11 / 쉴드 S11 (TX, 5V → 3.3V 분압)
 *                                 GND
 *   ESP32-S3 GND ──────────────────────────── 우노 GND   (공통 필수)
 *
 *   우노 S11 쪽 선은 엔코더 값을 ESP32 에 보여주는 용도라 조작만 할 거면 빼도 된다.
 *   D0/D1(USB 시리얼)을 안 쓰므로 우노 업로드·시리얼 모니터는 선 안 뽑고 그대로 된다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 링크 프로토콜 (9600, 줄 단위)
 *
 *   ESP32 → 우노
 *     M <left> <right>   −255~+255. 부호 = 방향. 키가 눌려 있든 아니든 50ms 마다 온다
 *     X                  비상 정지 (램프 없이 즉시 0)
 *     ramp<ms>           램프 시간 변경 (0 = 램프 없음)
 *     ramp<ms> e ?       램프 변경 / 엔코더 0 / 상태
 *     bal0 / bal1        엔코더 좌우 균형 보정 끄기 / 켜기 (기본 켬)
 *     (f b l r 테스트는 USB 전용 — ESP32 가 보내는 M 에 바로 덮인다)
 *   우노 → ESP32 (SEND_STATUS 1 일 때만. SoftwareSerial 은 보내는 동안 못 받으므로 기본 0)
 *     E <encL> <encR> <pwmL> <pwmR>   200ms 마다 상태
 *     그 외 텍스트 줄                   ESP32 가 "[uno] ..." 로 표시
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
 * USB 시리얼(115200) 테스트 명령 — 바퀴 들고 확인. ESP32 가 붙어 있어도 된다 (1초 동안 링크 M 무시)
 *
 *   f / b / l / r   1초 동안 전진/후진/좌회전/우회전 (PWM 150). 반드시 Enter(새 줄)로 끝내야 한다
 *   s               정지
 *   ?               상태
 *   e               엔코더 카운트 0 으로
 *   ramp<ms>        램프 시간 (예: ramp0, ramp700)
 *   bal0 / bal1     엔코더 좌우 균형 보정 끄기 / 켜기
 *
 *   'f' 를 쳤는데 한쪽 바퀴가 뒤로 돌면 그쪽 *_INVERT 를 뒤집는다.
 * -----------------------------------------------------------------------------
 */

#include <SoftwareSerial.h>

// =============================================================================
//  핀
// =============================================================================
// 예전 2WD 코드(FL D6/D4, FR D9/D8)와 동일
#define L_PWM    6     // 앞왼(FL)  PWM  (MDD10A PWM2, 쉴드 S6)
#define L_DIR    4     // 앞왼(FL)  DIR  (MDD10A DIR2, 쉴드 S4)
#define R_PWM    9     // 앞오른(FR) PWM (MDD10A PWM1, 쉴드 S9)
#define R_DIR    8     // 앞오른(FR) DIR (MDD10A DIR1, 쉴드 S8)

#define L_INVERT 1     // 예전 코드: FL 전진 = DIR LOW
#define R_INVERT 0     // 예전 코드: FR 전진 = DIR HIGH

// 엔코더: 예전 코드와 같이 B상만 센다 (A상 불량). FL B = D3 인터럽트, FR B = D7 폴링.
// 방향은 그때 내고 있는 PWM 부호로 정한다.
#define USE_ENCODERS 1
#define ENC_L_B  3     // FL 엔코더 B상 (INT1)
#define ENC_R_B  7     // FR 엔코더 B상 (폴링)

#define LINK_RX  10    // ← ESP32 GPIO17          (쉴드 S10)
#define LINK_TX  11    // → 분압 → ESP32 GPIO18   (쉴드 S11, 선택)

// =============================================================================
//  설정
// =============================================================================
#define LINK_BAUD        9600   // SoftwareSerial 은 느릴수록 안정. 20Hz×10바이트면 9600 으로 충분
#define LINK_TIMEOUT_MS  300    // M 이 이만큼 안 오면 정지
#define RAMP_MS_DEFAULT  700    // 0→255 램프 시간
#define RAMP_TICK_MS     10
#define SEND_STATUS      0      // 1 이면 E 상태를 S11 로 보냄. 보내는 동안 수신이 막히므로 S11 안 쓰면 0
#define STATUS_MS        200    // E 상태 전송 주기
#define TEST_PWM         150
#define TEST_HOLD_MS     1000

// 엔코더 피드백 — 좌우 속도 균형 (직진 보정)
//   50ms 마다 두 바퀴 속도(cps)를 재고, "PWM 1 당 cps" 가 좌우 같아지도록 trim 을 적분한다.
//   출력 = 램프값 × (1 ∓ trim).  명령 비율(직진 1:1, 제자리 회전 1:−1)을 그대로 유지하면서
//   빠른 쪽을 줄이고 느린 쪽을 올린다.  bal0 / bal1 로 끄고 켠다.
#define SPEED_MS         50     // 속도 측정 주기
#define SPEED_LPF        0.4f   // 속도 저역 필터 (0~1, 클수록 빠름)
#define BAL_DEFAULT      1      // 1 = 균형 제어 켜고 시작
#define BAL_KP           0.3f   // 비례
#define BAL_KI           1.0f   // 적분 (/s)
#define BAL_TRIM_MAX     0.35f  // trim 한계 (±35 %)
#define BAL_MIN_CPS      20     // 두 바퀴 속도 합이 이보다 작으면 판단 안 함 (정지·출발 직후)

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

float    spdL = 0, spdR = 0;      // 측정 속도 (counts/s, 필터)
float    balI = 0, balP = 0;      // 균형 trim 의 적분·비례 성분
bool     balOn = BAL_DEFAULT;
int      outL = 0, outR = 0;      // 균형 보정 뒤 실제 모터에 간 값
uint32_t lastSpeedMs = 0;
long     lastEncL = 0, lastEncR = 0;

// =============================================================================
//  엔코더 — B상 상승 에지만 센다. 부호는 현재 출력 방향
// =============================================================================
volatile int8_t encSignL = 1, encSignR = 1;
void isrEncL() { encL += encSignL; }
void pollEncR() {
  static int last = HIGH;
  int b = digitalRead(ENC_R_B);
  if (b == HIGH && last == LOW) encR += encSignR;
  last = b;
}

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
  float t = balOn ? constrain(balI + balP, -BAL_TRIM_MAX, BAL_TRIM_MAX) : 0.0f;
  outL = constrain((int)lroundf(curL * (1.0f - t)), -255, 255);
  outR = constrain((int)lroundf(curR * (1.0f + t)), -255, 255);
  outMotor(L_PWM, L_DIR, outL, L_INVERT);
  outMotor(R_PWM, R_DIR, outR, R_INVERT);
  if (curL) encSignL = (curL > 0) ? 1 : -1;
  if (curR) encSignR = (curR > 0) ? 1 : -1;
}

// =============================================================================
//  속도 측정 + 좌우 균형
// =============================================================================
void speedTick() {
  uint32_t now = millis();
  if (now - lastSpeedMs < SPEED_MS) return;
  float dt = (now - lastSpeedMs) * 1e-3f;
  lastSpeedMs = now;

  long l, r;
  noInterrupts(); l = encL; r = encR; interrupts();
  float vL = (l - lastEncL) / dt, vR = (r - lastEncR) / dt;
  lastEncL = l; lastEncR = r;
  spdL += SPEED_LPF * (vL - spdL);
  spdR += SPEED_LPF * (vR - spdR);

  if (!balOn || curL == 0 || curR == 0) { balP = 0; return; }   // 한쪽이 0 이면 비율이 없다
  if (fabsf(spdL) + fabsf(spdR) < BAL_MIN_CPS) { balP = 0; return; }

  float sL = spdL / curL, sR = spdR / curR;      // PWM 1 당 cps (방향 맞으면 양수)
  float e  = (sL - sR) / (fabsf(sL) + fabsf(sR) + 1e-6f);   // −1 ~ +1. 왼쪽이 빠르면 +
  balP  = BAL_KP * e;
  balI += BAL_KI * e * dt;
  balI  = constrain(balI, -BAL_TRIM_MAX, BAL_TRIM_MAX);
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
  out.print(F("  spd "));        out.print((int)spdL); out.print(' '); out.print((int)spdR);
  out.print(F("cps  bal "));     out.print(balOn ? F("on ") : F("off "));
  out.print((int)lroundf((balI + balP) * 100)); out.print(F("%  motor "));
  out.print(outL); out.print(' '); out.print(outR);
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
    if (fromLink) {
      lastCmdMs = millis();
      if (testUntilMs) return;          // USB 테스트(f/b/l/r) 중엔 ESP32 의 M 0 0 이 덮어쓰지 않게
    } else {
      testUntilMs = 0;                  // USB 로 직접 친 M 은 테스트 취소
    }
    setTarget((int)l, (int)r);
    return;
  }
  if (strcmp(line, "X") == 0 || strcmp(line, "x") == 0) {
    hardStop();
    testUntilMs = 0;
    out.println(F("[cmd] emergency stop"));
    return;
  }
  if (strncmp(line, "bal", 3) == 0 && (line[3] == '0' || line[3] == '1')) {
    balOn = (line[3] == '1');
    balI = balP = 0;
    out.print(F("[cmd] balance ")); out.println(balOn ? F("on") : F("off"));
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
      case 'e': noInterrupts(); encL = encR = 0; interrupts(); lastEncL = lastEncR = 0; out.println(F("[cmd] enc zero")); return;
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
  hardStop();

#if USE_ENCODERS
  pinMode(ENC_L_B, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrEncL, RISING);
#endif

  Serial.begin(115200);
  link.begin(LINK_BAUD);
  Serial.println(F("[uno] drive link ready (front 2WD, encoder balance). f/b/l/r/s/?/e/ramp<ms>/bal0/bal1"));
}

void loop() {
  uint32_t now = millis();
  char *line;

#if USE_ENCODERS
  pollEncR();
#endif

  // ── ESP32 링크 수신 ──
  while (link.available()) {
    if (linkBuf.feed((char)link.read(), line)) handleLine(line, Serial, true);   // 응답은 USB 로만
  }
  // ── USB 시리얼 수신 ──
  while (Serial.available()) {
    if (usbBuf.feed((char)Serial.read(), line)) handleLine(line, Serial, false);
  }

  // ── 링크 상태 전이 ──
  // now 는 입력 처리 뒤에 다시 읽는다. 위에서 읽은 값을 쓰면 handleLine 안에서 찍은
  // lastCmdMs 가 now 보다 커져 (now - lastCmdMs) 가 뒤집히고 워치독이 매 루프 오작동한다.
  now = millis();
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

#if USE_ENCODERS
  speedTick();          // 속도 측정 + 좌우 균형 trim
#endif
  rampTick();

  // ── 상태 전송 (S11 연결 시) ──
#if SEND_STATUS
  if (now - lastStatMs >= STATUS_MS) {
    lastStatMs = now;
    long l, r;
    noInterrupts(); l = encL; r = encR; interrupts();
    link.print(F("E ")); link.print(l); link.print(' '); link.print(r);
    link.print(' ');     link.print(outL); link.print(' '); link.println(outR);
  }
#endif
}
