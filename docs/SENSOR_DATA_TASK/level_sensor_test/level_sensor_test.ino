/*
 * level_sensor_test.ino — 저항식 수위 센서 단독 동작 확인 (ESP32-S3)
 * -----------------------------------------------------------------------------
 * 루트 Liquid_Control_Robot.ino 의 readLevel() 과 똑같은 핀·방식으로 읽는다.
 * 센서를 꽂은 ESP32 를 USB 로 연결하고 시리얼 모니터 115200 으로 보면 된다.
 *
 * 배선 (메카솔루션 저항식 수위센서, 60x20 mm, 핀 3개)
 *
 *   센서 S (OUT, 신호)  ──  ESP32-S3 GPIO 1  (ADC)
 *   센서 + (VCC)        ──  ESP32-S3 GPIO 2  (읽을 때만 300 µs HIGH. 3V3 에 직결하지 말 것)
 *   센서 − (GND)        ──  ESP32-S3 GND
 *
 *   VCC 를 GPIO 로 주는 이유: 전원이 계속 걸린 채 물에 두면 빗살이 전기분해로
 *   며칠 만에 죽는다. 읽는 순간에만 켠다.
 *
 * 기준값 (2026-09-05 단독 테스트)
 *   마름 ~1070   /   다 잠김 ~2050   /   정지 수면은 1400~1600 이 되게 높이를 맞춘다
 *
 * 출력 (초당 10줄)
 *   lvl 1523  [##########..........]  물  (1초 최소 1498 최대 1540, 폭 42)
 *   값이 안 변하면: 센서 S 선 / GND 를 본다.  마른데 1070 보다 한참 높으면: 센서 표면에 물기.
 *
 * 보드 설정: ESP32S3 Dev Module. COM 포트(CH343)에 꽂으면 USB CDC On Boot = Disabled,
 *            네이티브 USB 포트에 꽂으면 Enabled.
 * -----------------------------------------------------------------------------
 */

#define PIN_LVL_OUT  1      // ADC
#define PIN_LVL_PWR  2      // 읽을 때만 HIGH

#define LVL_DRY      1070   // 마름 기준
#define LVL_FULL     2050   // 다 잠김 기준

static int lvl_hist[3] = {0, 0, 0};
int lvl_med = 0;

// 루트 코드와 동일
void readLevel() {
  digitalWrite(PIN_LVL_PWR, HIGH);
  delayMicroseconds(300);              // 분압 정착
  int v = analogRead(PIN_LVL_OUT);
  digitalWrite(PIN_LVL_PWR, LOW);
  lvl_hist[0] = lvl_hist[1]; lvl_hist[1] = lvl_hist[2]; lvl_hist[2] = v;
  int a = lvl_hist[0], b = lvl_hist[1], c = lvl_hist[2];
  lvl_med = max(min(a, b), min(max(a, b), c));
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(PIN_LVL_PWR, OUTPUT);
  digitalWrite(PIN_LVL_PWR, LOW);
  analogReadResolution(12);            // 0 ~ 4095

  Serial.println();
  Serial.println("=== 수위 센서 단독 테스트 ===");
  Serial.printf("S -> GPIO%d (ADC)   VCC -> GPIO%d (읽을 때만 HIGH)   GND -> GND\n", PIN_LVL_OUT, PIN_LVL_PWR);
  Serial.printf("마름 ~%d / 잠김 ~%d / 정지 수면은 1400~1600 이 되게\n", LVL_DRY, LVL_FULL);
  Serial.println("100 Hz 로 읽고 초당 10줄 출력. 손가락을 적셔 대보거나 물컵에 담가 본다.");
  Serial.println();
}

void loop() {
  static uint32_t lastRead = 0, lastPrint = 0, lastSec = 0;
  static int secMin = 9999, secMax = -1;
  uint32_t now = millis();

  if (now - lastRead >= 10) {          // 100 Hz — 루트 코드와 같은 주기
    lastRead = now;
    readLevel();
    if (lvl_med < secMin) secMin = lvl_med;
    if (lvl_med > secMax) secMax = lvl_med;
  }

  if (now - lastPrint >= 100) {        // 초당 10줄
    lastPrint = now;
    int n = constrain((lvl_med - LVL_DRY) * 20 / (LVL_FULL - LVL_DRY), 0, 20);
    char bar[22];
    for (int i = 0; i < 20; i++) bar[i] = (i < n) ? '#' : '.';
    bar[20] = 0;
    const char *state = (lvl_med < LVL_DRY + 60) ? "마름" :
                        (lvl_med > LVL_FULL - 80) ? "다 잠김" :
                        (lvl_med >= 1400 && lvl_med <= 1600) ? "물 (정지 수면 적정)" : "물";
    Serial.printf("lvl %4d  [%s]  %s", lvl_med, bar, state);
    if (now - lastSec >= 1000) {
      Serial.printf("   (1초 최소 %d 최대 %d, 폭 %d)", secMin, secMax, secMax - secMin);
      lastSec = now; secMin = 9999; secMax = -1;
    }
    Serial.println();
  }
}
