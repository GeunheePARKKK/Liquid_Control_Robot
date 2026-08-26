/*
 * IMU_Reading.ino
 * -----------------------------------------------------------------------------
 * 트레이 IMU (LSM6DSOX) 단독 기울기 측정 — 시리얼 출력
 *
 *   MCU     : ESP32-S3
 *   통신    : SPI 4-wire, 1MHz, MODE0
 *   센서    : LSM6DSOX  (±4g / ±2000dps, ODR 416Hz)
 *   출력    : 100Hz 제어 루프 / 10Hz 시리얼 출력 (시리얼 플로터 호환 포맷)
 *
 * 배선 (hardware/배선도.md 참고)
 *   IMU 보드 라벨 → ESP32-S3
 *     SDA  → GPIO 11  (MOSI)
 *     SCL  → GPIO 12  (SCK)
 *     SDO  → GPIO 13  (MISO)
 *     CS   → GPIO 10  (10kΩ 풀업 → 3V3)
 *     VIN  → 3V3
 *     GND  → GND
 *
 * 포함된 방어 로직
 *   1) 부팅 시 레지스터 덤프로 배선/설정 자체 진단
 *   2) 센서 리셋(브라운아웃) 감지 및 재초기화 — 즉시 + 1초 주기 이중 확인
 *   3) |a| 기반 가속도 신뢰도 가중 — 1g 에서 멀수록 가속도계 반영을 줄임
 *      → 깨진 SPI 샘플 차단 + 선형 가속도로 인한 기울기 오염 방지
 *   4) 연속 무보정 구간 추적 — 아래 참고
 *
 * "연속" 을 보는 이유
 *   가속도 보정이 빠지면 그 순간은 자이로 단독 적분이 된다.
 *   자이로는 짧게는 정확하고 길어지면 틀어지므로, 무보정 샘플의 총 개수보다
 *   연속으로 몇 개가 이어졌는가가 중요하다. (1샘플 = 0.01초)
 *
 *     연속 ≤ 5    → 0.05초. 무해 ✅
 *     연속 ~50    → 0.5초.  드리프트 발생 ❌
 *
 *   "무보정" 에는 실제 가속(로봇 주행, 손으로 흔들기)으로 인한 정상 감쇠도
 *   포함된다. 통신 오류만 보려면 센서를 완전히 고정한 상태에서 측정할 것.
 *
 * 측정 이력 (2026-08-26, 벤치 테스트)
 *
 *   [1차 — 하드 컷 / ±2g]
 *     정지          : 오류 0 / 304 샘플  → 배선 정상 확인
 *     가볍게 움직임 : 최대연속 7~8  (0.08초)
 *     격하게 흔듦   : 최대연속 44   (0.44초), |a| 2.77g 로 ±2g 포화
 *                     → ±4g 상향 + 하드 컷 대신 신뢰도 가중으로 변경
 *
 *   [2차 — 신뢰도 가중 / ±4g]
 *     정지          : 무보정 0, pitch_acc 흔들림 0.25°
 *     가볍게 움직임 : 무보정 0     (1차의 최대연속 14 → 0)
 *     격하게 흔듦   : 최대연속 12  (0.44초 → 0.12초, 3.7배 단축)
 *                     |a| 3.21g 까지 포화 없이 측정됨
 *
 * 진동 주의
 *   센서를 손에 들고 측정하면 정지 상태인데도 손 떨림 때문에 pitch_acc 흔들림이
 *   0.25° → 9.56° 로 악화된다 (통신 오류는 0). 측정은 반드시 바닥에 내려놓고 할 것.
 *   옆방향 진동 0.087g 는 각도를 5° 흔들지만 |a| 는 1.004g 라 신뢰도 가중을
 *   그대로 통과하므로, 진동은 이 로직으로 걸러지지 않는다.
 *   → 실장 후 모터 진동이 확인되면 방진 마운트를 검토할 것 (영향은 아직 미측정).
 * -----------------------------------------------------------------------------
 */

#include <SPI.h>

// ── 핀 (IMU 보드 라벨 기준) ──
#define PIN_MOSI  11    // IMU 보드 SDA
#define PIN_SCK   12    // IMU 보드 SCL
#define PIN_MISO  13    // IMU 보드 SDO
#define CS_TRAY   10    // IMU 보드 CS

#define SPI_SPEED 1000000
#define SPI_MD    SPI_MODE0

// ── LSM6DSOX 레지스터 ──
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_STATUS     0x1E
#define REG_OUTX_L_G   0x22    // 자이로 X부터 12바이트 (자이로 → 가속도 순)

#define WHO_AM_I_VAL   0x6C
#define CTRL1_XL_VAL   0x68    // 가속도 416Hz, ±4g  (±2g 는 0x60 — 주행 중 포화함)
#define CTRL2_G_VAL    0x6C    // 자이로  416Hz, ±2000dps
#define CTRL3_C_VAL    0x44    // BDU=1, IF_INC=1

// ── 감도 ──
const float G_LSB    = 8192.0;    // ±4g   : 1g 당 LSB   (±2g 이면 16384.0)
const float GYRO_LSB = 14.286;    // ±2000dps : 1dps 당 LSB (70 mdps/LSB)
                                  // 주의: ICM/MPU 계열의 16.4가 아님

// ── 상보 필터 ──
const float dt       = 0.01;      // 100Hz
const float alpha_cf = 0.98;      // 정지 시 자이로 신뢰도 98%

/* ── 가속도 신뢰도 (정지 시 |a| = 1.0g) ──
 * 예전에는 밴드를 벗어나면 샘플을 통째로 버렸는데, 그러면 가속 구간에서
 * 수십 샘플이 연속으로 빠져 그동안 자이로 단독 적분이 되어 각도가 무너졌다.
 * 지금은 1g 에서 멀어질수록 가속도계 반영 비중을 서서히 줄인다.
 *   |a| = 1.0g              → trust 1.00 (평소대로 보정)
 *   |a| = 1.25g             → trust 0.50 (절반만 반영)
 *   |a| = 1.5g 이상         → trust 0.00 (자이로 단독)
 */
const float ACC_TRUST_FALLOFF = 0.5;
const float ACC_MAG_DEAD      = 0.05;  // 이보다 작으면 센서 파워다운으로 간주

// ── 상태 변수 ──
float pitch_filtered = 0, roll_filtered = 0;
float gyro_bias_x = 0, gyro_bias_y = 0;
float g_scale = 1.0;              // 정지 시 측정한 1g 크기 (setup 에서 결정)

unsigned long lastLoop = 0, lastPrint = 0, lastCheck = 0;
uint32_t n_win = 0, err_win = 0, err_reset = 0;

/* 연속 무보정 구간 추적 (trust = 0 인 샘플이 몇 개나 이어졌는가)
 * 흩어진 거부는 무해하지만 연속으로 뭉치면 각도가 틀어진다. (1샘플 = 0.01초)
 *   ≤ 5   (0.05초) → 무해
 *   ~50   (0.5초)  → 드리프트 발생
 * "최근" 은 직전 1초 구간, "전체" 는 부팅 이후 최고 기록.
 */
uint16_t rej_run = 0, rej_run_win = 0, rej_run_max = 0;


// =============================================================================
//  SPI 저수준
// =============================================================================
void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(CS_TRAY, LOW);
  delayMicroseconds(20);
  SPI.transfer(reg & 0x7F);            // 쓰기: MSB = 0
  SPI.transfer(val);
  digitalWrite(CS_TRAY, HIGH);
  SPI.endTransaction();
}

uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(CS_TRAY, LOW);
  delayMicroseconds(20);
  SPI.transfer(reg | 0x80);            // 읽기: MSB = 1
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(CS_TRAY, HIGH);
  SPI.endTransaction();
  return v;
}

// 자이로 3축 + 가속도 3축 연속 읽기 (LSM6DSOX = 리틀엔디안)
void readIMU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[12];
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(CS_TRAY, LOW);
  delayMicroseconds(20);
  SPI.transfer(REG_OUTX_L_G | 0x80);
  for (int i = 0; i < 12; i++) b[i] = SPI.transfer(0x00);
  digitalWrite(CS_TRAY, HIGH);
  SPI.endTransaction();

  gx = (int16_t)((b[1]  << 8) | b[0]);
  gy = (int16_t)((b[3]  << 8) | b[2]);
  gz = (int16_t)((b[5]  << 8) | b[4]);
  ax = (int16_t)((b[7]  << 8) | b[6]);
  ay = (int16_t)((b[9]  << 8) | b[8]);
  az = (int16_t)((b[11] << 8) | b[10]);
}


// =============================================================================
//  센서 설정
// =============================================================================
void configSensor() {
  writeReg(REG_CTRL3_C, 0x01);         // 소프트 리셋
  delay(50);
  writeReg(REG_CTRL3_C,  CTRL3_C_VAL); // BDU=1, IF_INC=1
  writeReg(REG_CTRL1_XL, CTRL1_XL_VAL);
  writeReg(REG_CTRL2_G,  CTRL2_G_VAL);
  delay(100);
}

/*
 * 부팅 진단 — 배선 문제를 한 번에 판별
 *
 *   WHO_AM_I = 0x00           → 읽기 경로 사망 (SDO / VIN / GND 확인)
 *   WHO_AM_I = 0xFF           → MISO 플로팅 or CS 안 내려감
 *   WHO_AM_I OK, CTRL1 = 0x00 → 쓰기가 안 먹음 (SDA 확인)
 *   raw 12B 전부 같은 바이트  → IF_INC 미동작
 *   raw 12B 전부 00           → 센서 미가동
 */
void dumpDiag() {
  Serial.println("\n===== 진단 =====");
  Serial.printf("WHO_AM_I   (0x0F): 0x%02X   기대 0x%02X\n", readReg(REG_WHO_AM_I), WHO_AM_I_VAL);
  Serial.printf("CTRL1_XL   (0x10): 0x%02X   기대 0x%02X\n", readReg(REG_CTRL1_XL), CTRL1_XL_VAL);
  Serial.printf("CTRL2_G    (0x11): 0x%02X   기대 0x%02X\n", readReg(REG_CTRL2_G),  CTRL2_G_VAL);
  Serial.printf("CTRL3_C    (0x12): 0x%02X   기대 0x%02X\n", readReg(REG_CTRL3_C),  CTRL3_C_VAL);
  Serial.printf("STATUS_REG (0x1E): 0x%02X   기대 0x03 (XLDA|GDA)\n", readReg(REG_STATUS));

  uint8_t b[12];
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(CS_TRAY, LOW);
  delayMicroseconds(20);
  SPI.transfer(REG_OUTX_L_G | 0x80);
  for (int i = 0; i < 12; i++) b[i] = SPI.transfer(0x00);
  digitalWrite(CS_TRAY, HIGH);
  SPI.endTransaction();

  Serial.print("raw 12B: ");
  for (int i = 0; i < 12; i++) Serial.printf("%02X ", b[i]);
  Serial.println("\n================\n");
}


// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);                         // USB CDC 안정화

  pinMode(CS_TRAY, OUTPUT);
  digitalWrite(CS_TRAY, HIGH);         // SPI 모드 진입 전 반드시 HIGH
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  delay(100);

  uint8_t id = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I: 0x%02X (기대값 0x%02X)\n", id, WHO_AM_I_VAL);
  if (id != WHO_AM_I_VAL) {
    Serial.println("!! IMU 응답 없음 — 0xFF: MISO/CS 확인, 0x00: VIN/GND 확인");
  }

  configSensor();
  dumpDiag();

  /* ── 자이로 영점 + 중력 스케일 (정지 상태 유지) ──
   * 가속도계는 개체 편차로 1g 를 정확히 1.000 으로 읽지 않는다 (실측 0.97).
   * 그대로 두면 정지 상태에서도 trust 가 1.00 에 못 미치고, 신뢰도 감쇠 기준도
   * 실제 1g 가 아닌 값에서 출발하므로 정지 시 측정값으로 정규화한다.
   */
  Serial.println(">>> 캘리브레이션 중... 움직이지 마세요");
  long  sgx = 0, sgy = 0;
  float smag = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    sgx += gx;  sgy += gy;
    float fax = ax, fay = ay, faz = az;
    smag += sqrtf(fax * fax + fay * fay + faz * faz) / G_LSB;
    delay(5);
  }
  gyro_bias_x = (float)sgx / 200.0;
  gyro_bias_y = (float)sgy / 200.0;

  float measured = smag / 200.0;
  // 값이 터무니없으면(센서 이상/캘리브 중 움직임) 정규화를 포기하고 1.0 유지
  if (measured > 0.5f && measured < 1.5f) {
    g_scale = measured;
  } else {
    Serial.printf("!! 중력 스케일 이상 (%.3f) — 정규화 생략\n", measured);
  }
  Serial.printf("gyro bias  x:%.1f  y:%.1f   g_scale:%.4f\n",
                gyro_bias_x, gyro_bias_y, g_scale);

  // ── 가속도 기준 초기값 (수렴 시간 단축) ──
  int16_t ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);
  pitch_filtered = atan2((float)ay,  sqrt(pow((float)ax, 2) + pow((float)az, 2))) * (180.0 / PI);
  roll_filtered  = atan2((float)-ax, sqrt(pow((float)ay, 2) + pow((float)az, 2))) * (180.0 / PI);

  lastLoop = micros();
  Serial.println("start");
}


// =============================================================================
//  loop
// =============================================================================
void loop() {
  if ((long)(micros() - lastLoop) >= 10000) {     // 100Hz
    lastLoop += 10000;                             // 누적 방식 (dt 오차 방지)

    int16_t ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);

    // ── 자이로 → dps ──
    float rate_pitch = ((float)gx - gyro_bias_x) / GYRO_LSB;
    float rate_roll  = ((float)gy - gyro_bias_y) / GYRO_LSB;

    // ── 가속도 기반 각도 (노이즈 있지만 절대 기준) ──
    float pitch_acc = atan2((float)ay,  sqrt(pow((float)ax, 2) + pow((float)az, 2))) * (180.0 / PI);
    float roll_acc  = atan2((float)-ax, sqrt(pow((float)ay, 2) + pow((float)az, 2))) * (180.0 / PI);

    /* ── 가속도 신뢰도 가중 ────────────────────────────────────────────
     * 정지 상태에서 |a| 는 반드시 1.0g. 벗어나면 두 경우 중 하나다.
     *   (1) SPI 통신 오류로 값이 깨짐
     *   (2) 실제 선형 가속 중 → 중력 방향 추정 불가
     * 어느 쪽이든 가속도계를 기울기 기준으로 그대로 쓸 수 없다.
     * 다만 통째로 버리면 가속 구간에서 수십 샘플이 연속으로 빠지므로,
     * 버리는 대신 1g 에서 멀어진 만큼 반영 비중만 줄인다.
     * (2)는 짐벌 제어 진입 후에도 계속 필요한 로직이므로 유지할 것.
     */
    float fax = ax, fay = ay, faz = az;
    // g_scale 로 나눠 정지 시 정확히 1.000 이 되도록 정규화
    float a_mag = sqrtf(fax * fax + fay * fay + faz * faz) / G_LSB / g_scale;

    // 1g 에서 벗어난 만큼 가속도계 반영 비중을 줄인다 (0.0 ~ 1.0)
    float dev   = fabsf(a_mag - 1.0f);
    float trust = constrain(1.0f - dev / ACC_TRUST_FALLOFF, 0.0f, 1.0f);

    // trust 만큼만 가속도 보정이 섞이도록 alpha 를 끌어올린다
    //   trust = 1 → alpha = alpha_cf (평소)
    //   trust = 0 → alpha = 1.0      (자이로 단독)
    float alpha = 1.0f - (1.0f - alpha_cf) * trust;

    pitch_filtered = alpha * (pitch_filtered + rate_pitch * dt) + (1.0f - alpha) * pitch_acc;
    roll_filtered  = alpha * (roll_filtered  + rate_roll  * dt) + (1.0f - alpha) * roll_acc;

    // 진단: 완전히 무보정(trust = 0)인 샘플이 연속으로 몇 개나 이어지는지
    n_win++;
    if (trust <= 0.0f) {
      err_win++;
      if (++rej_run > rej_run_win) rej_run_win = rej_run;
      if (rej_run > rej_run_max)   rej_run_max = rej_run;
    } else {
      rej_run = 0;
    }

    /* ── 센서 리셋 즉시 감지 ──
     * |a| 가 0에 가까우면 출력 레지스터가 전부 0이라는 뜻 = 파워다운 상태.
     * 아래 1초 주기 점검을 기다리지 않고 그 자리에서 복구한다.
     */
    if (a_mag < ACC_MAG_DEAD && readReg(REG_CTRL1_XL) != CTRL1_XL_VAL) {
      err_reset++;
      Serial.println("!! 센서 파워다운 감지 — 즉시 재설정");
      configSensor();
    }

    // ── 주기 점검 (즉시 감지에 걸리지 않는 설정 변형 대비) ──
    if (millis() - lastCheck > 1000) {
      lastCheck = millis();
      uint8_t c1 = readReg(REG_CTRL1_XL);
      if (c1 != CTRL1_XL_VAL) {
        err_reset++;
        Serial.printf("!! 센서 리셋 감지 CTRL1_XL=0x%02X — 재설정\n", c1);
        configSensor();
      }
      // 누적이 아니라 직전 1초 구간 기준으로 출력 (누적은 한 번 튀면 계속 남아 헷갈림)
      Serial.printf("[stat] reset:%lu  무보정:%lu/%lu  연속:%u최근/%u전체  |a|=%.2f trust=%.2f\n",
                    err_reset, err_win, n_win,
                    rej_run_win, rej_run_max, a_mag, trust);
      n_win = 0;  err_win = 0;  rej_run_win = 0;
    }

    // ── 시리얼 플로터 출력 (10Hz) ──
    if (millis() - lastPrint > 100) {
      lastPrint = millis();
      Serial.printf("pitch:%.2f,pitch_acc:%.2f,roll:%.2f,roll_acc:%.2f\n",
                    pitch_filtered, pitch_acc, roll_filtered, roll_acc);
    }
  }
}

/* -----------------------------------------------------------------------------
 * 검증 방법
 *
 *  1) 정지 : pitch / roll 이 0 근처, 수 분간 드리프트 없어야 함
 *  2) 기울임 : 30° 기울이면 30° 출력
 *  3) 축·부호 : 빠르게 흔들 때 pitch 와 pitch_acc 가 같이 움직여야 정상.
 *               반대로 튀거나 발산하면 rate_pitch / rate_roll 부호를 반전.
 *  4) [stat] 로그 — 센서를 책상에 고정하고 30초간 손대지 말 것
 *       reset 증가       → 전원 문제 (VIN/GND 접촉, 디커플링 100nF 추가)
 *       무보정 0/N       → 정상. 제어 단계로 진행 가능
 *       무보정 계속 증가 → 신호선 접촉 불량 (SDO/SCL/SDA 점퍼 확인, 납땜 검토)
 *       ※ 움직이는 중이면 무보정 카운트는 정상적으로 올라간다.
 *         "연속" 은 최근 1초 구간(앞)과 부팅 이후 최고 기록(뒤)을 함께 표시한다.
 *  5) 포화 : 격한 동작에서도 |a| 가 3.9g 를 넘으면 ±4g 포화.
 *            CTRL1_XL_VAL → 0x6C (±8g), G_LSB → 4096.0 으로 변경.
 * ---------------------------------------------------------------------------*/
