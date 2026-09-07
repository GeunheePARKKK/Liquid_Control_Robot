/*
 * drv8825_serial_control.ino
 *
 * ESP32-S3 + DRV8825 ×2 스텝모터 시리얼 명령 제어 (저속 안전 설정)
 * 두 모터를 같은 방향·같은 속도로 동시에 돌린다.
 *
 *   1 -> 시계 방향 연속 회전
 *   2 -> 반시계 방향 연속 회전
 *   0 -> 즉시 정지
 *   - -> 더 느리게
 *   + -> 더 빠르게
 *   ? -> 현재 상태 출력
 *
 * 시리얼 모니터 115200 bps
 *
 * 기본 속도는 6 RPM (1회전에 10초). 손으로 만져도 위험하지 않은 수준으로 낮춰 둠.
 *
 * ---- 배선 ----------------------------------------------------------------
 *   #1 STEP -> GPIO 15    #1 DIR -> GPIO 16      (4·5 는 CAN 이 쓴다)
 *   #2 STEP -> GPIO 14    #2 DIR -> GPIO 21
 *   두 드라이버 모두:
 *   DRV8825 RESET  -> ESP32-S3 3V3      (필수)
 *   DRV8825 SLEEP  -> ESP32-S3 3V3      (필수)
 *   DRV8825 GND    -> ESP32-S3 GND      (전원 (-)와 공통 필수)
 *   DRV8825 VMOT   -> 18V(+)  + 100uF/35V 이상 커패시터 병렬
 *   A1/A2, B1/B2   -> 모터 코일 1 / 코일 2
 */

#define STEP1_PIN 15
#define DIR1_PIN  16
#define STEP2_PIN 14
#define DIR2_PIN  21

// 두 모터가 서로 반대로 돌면 1로. (#2 만 방향을 뒤집는다)
#define DIR2_INVERT 0

// ENABLE 핀을 GPIO에 연결했다면 1로 바꾼다.
// 정지 중 코일 전류가 차단되어 발열이 줄고, 축이 자유롭게 돌아간다.
#define USE_ENABLE 0
#define EN_PIN     6      // DRV8825 ENABLE (LOW = 동작, HIGH = 출력 차단)

const int STEPS_PER_REV = 200;    // 1.8도 풀스텝 기준

// STEP 펄스 폭. DRV8825 최소 사양은 1.9us이므로 5us면 충분하다.
// 속도는 이 값이 아니라 아래 stepIntervalUs로 조절한다.
const unsigned int PULSE_WIDTH_US = 5;

// 스텝과 스텝 사이 간격(us). 클수록 느리다.
//   50000us -> 1회전 10초 (6 RPM)   <- 기본값, 안전
//   25000us -> 1회전  5초 (12 RPM)
//   10000us -> 1회전  2초 (30 RPM)
//    3000us -> 1회전 0.6초 (100 RPM)
unsigned long stepIntervalUs = 50000;

const unsigned long MIN_INTERVAL_US = 3000;      // 상한 속도 제한
const unsigned long MAX_INTERVAL_US = 300000;    // 하한 속도 (1회전 60초)

// 방향 전환 시 잠깐 멈추는 시간(ms)
const unsigned int REVERSE_PAUSE_MS = 100;

int mode = 0;                     // 0 = 정지, 1 = 시계, 2 = 반시계
unsigned long lastStepUs = 0;

float currentRpm() {
  return 60000000.0f / ((float)stepIntervalUs * STEPS_PER_REV);
}

void printSpeed() {
  Serial.printf("속도: %.1f RPM (1회전 %.1f초, 간격 %luus)\n",
                currentRpm(), 60.0f / currentRpm(), stepIntervalUs);
}

void setMotorEnabled(bool on) {
#if USE_ENABLE
  digitalWrite(EN_PIN, on ? LOW : HIGH);
#else
  (void)on;
#endif
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(STEP1_PIN, OUTPUT);  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);  pinMode(DIR2_PIN, OUTPUT);
  digitalWrite(STEP1_PIN, LOW);  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(STEP2_PIN, LOW);  digitalWrite(DIR2_PIN, LOW);

#if USE_ENABLE
  pinMode(EN_PIN, OUTPUT);
#endif
  setMotorEnabled(false);         // 시작은 정지 상태

  Serial.println();
  Serial.println("=== DRV8825 x2 serial control (모터 2개 동시) ===");
  Serial.println("  1 : 시계 방향   2 : 반시계 방향   0 : 정지");
  Serial.println("  - : 느리게      + : 빠르게        ? : 상태");
  printSpeed();
  Serial.println("현재 상태: STOP");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '0') {
      if (mode != 0) {
        mode = 0;
        digitalWrite(STEP1_PIN, LOW);
        digitalWrite(STEP2_PIN, LOW);
        setMotorEnabled(false);
        Serial.println("STOP");
      }
    }
    else if (c == '1' || c == '2') {
      int newMode = (c == '1') ? 1 : 2;
      if (newMode == mode) continue;

      // 회전 중 반대 방향 명령이면 잠깐 멈췄다가 전환 (탈조 방지)
      if (mode != 0) {
        digitalWrite(STEP1_PIN, LOW);
        digitalWrite(STEP2_PIN, LOW);
        delay(REVERSE_PAUSE_MS);
      }

      mode = newMode;
      bool cw = (mode == 1);
      digitalWrite(DIR1_PIN, cw ? HIGH : LOW);
      digitalWrite(DIR2_PIN, (cw != (bool)DIR2_INVERT) ? HIGH : LOW);
      delayMicroseconds(10);      // DIR 셋업 시간
      setMotorEnabled(true);
      lastStepUs = micros();

      Serial.println((mode == 1) ? "CW  (시계 방향)" : "CCW (반시계 방향)");
      printSpeed();
    }
    else if (c == '-') {
      stepIntervalUs = stepIntervalUs * 2;
      if (stepIntervalUs > MAX_INTERVAL_US) stepIntervalUs = MAX_INTERVAL_US;
      printSpeed();
    }
    else if (c == '+' || c == '=') {
      stepIntervalUs = stepIntervalUs / 2;
      if (stepIntervalUs < MIN_INTERVAL_US) stepIntervalUs = MIN_INTERVAL_US;
      printSpeed();
    }
    else if (c == '?') {
      Serial.printf("상태: %s\n",
                    mode == 0 ? "STOP" : (mode == 1 ? "CW" : "CCW"));
      printSpeed();
    }
    // 그 외 문자(개행 등)는 무시
  }
}

void loop() {
  handleSerial();                 // 매 루프마다 확인 -> 0 입력 시 즉시 정지

  if (mode == 0) {
    delay(1);                     // 정지 중에는 그냥 쉰다
    return;
  }

  // mode가 1 또는 2인 동안에는 계속 회전한다.
  // 0(또는 반대 방향)을 입력하기 전까지 스스로 멈추지 않는다.
  unsigned long now = micros();
  unsigned long elapsed = now - lastStepUs;      // micros() 오버플로우에도 안전

  if (elapsed >= stepIntervalUs) {
    lastStepUs = now;
    // 두 STEP 을 같은 순간에 올리고 내린다 → 두 모터가 같은 스텝을 동시에 밟는다
    digitalWrite(STEP1_PIN, HIGH);
    digitalWrite(STEP2_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(STEP1_PIN, LOW);
    digitalWrite(STEP2_PIN, LOW);
  }
  else if (stepIntervalUs - elapsed > 2000) {
    // 다음 스텝까지 2ms 넘게 남았으면 1ms 양보한다.
    // 이걸 안 하면 저속에서 loop가 계속 CPU를 점유해
    // Task Watchdog에 걸려 보드가 리셋될 수 있다.
    delay(1);
  }
}
