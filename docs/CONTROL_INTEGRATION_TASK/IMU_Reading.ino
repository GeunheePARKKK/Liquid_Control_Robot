/*
 * IMU_Reading.ino
 * -----------------------------------------------------------------------------
 * 트레이 IMU (LSM6DSOX) 단독 기울기 측정 — 시리얼 출력
 *
 *   MCU     : ESP32-S3
 *   통신    : SPI 4-wire, 1MHz, MODE0
 *   센서    : LSM6DSOX  (±2g / ±2000dps, ODR 416Hz)
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
 *   3) 가속도 크기(|a|) 검사로 불량 샘플 제거
 *      → 깨진 SPI 샘플 차단 + 선형 가속도로 인한 기울기 오염 방지
 *   4) 연속 거부 구간(최대연속) 추적 — 아래 참고
 *
 * "최대연속" 을 보는 이유
 *   샘플을 버리면 대체값이 생기는 게 아니라 그 순간 가속도 보정이 빠진다.
 *   즉 자이로 단독 적분 구간이 되는데, 자이로는 짧게는 정확하고 길어지면 틀어진다.
 *   따라서 버린 총 개수보다 연속 길이가 중요하다. (1샘플 = 0.01초)
 *
 *     최대연속 ≤ 5   → 0.05초. 흩어진 오류. 무해 ✅
 *     최대연속 ~50   → 0.5초.  드리프트 발생 ❌
 *     최대연속 100+  → 1초 이상. 배선 수리 필요 ❌
 *
 *   bit_err 에는 실제 가속(로봇 주행 등)으로 인한 정상 거부도 포함되므로,
 *   센서를 완전히 고정한 상태에서 측정해야 통신 오류만 구분된다.
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
#define CTRL1_XL_VAL   0x60    // 가속도 416Hz, ±2g
#define CTRL2_G_VAL    0x6C    // 자이로  416Hz, ±2000dps
#define CTRL3_C_VAL    0x44    // BDU=1, IF_INC=1

// ── 감도 ──
const float G_LSB    = 16384.0;   // ±2g   : 1g 당 LSB
const float GYRO_LSB = 14.286;    // ±2000dps : 1dps 당 LSB (70 mdps/LSB)
                                  // 주의: ICM/MPU 계열의 16.4가 아님

// ── 상보 필터 ──
const float dt       = 0.01;      // 100Hz
const float alpha_cf = 0.98;      // 자이로 신뢰도 98%

// ── 가속도 신뢰 구간 (정지 시 |a| = 1.0g) ──
const float ACC_MAG_MIN  = 0.85;
const float ACC_MAG_MAX  = 1.15;
const float ACC_MAG_DEAD = 0.05;   // 이보다 작으면 센서 파워다운으로 간주

// ── 상태 변수 ──
float pitch_filtered = 0, roll_filtered = 0;
float gyro_bias_x = 0, gyro_bias_y = 0;

unsigned long lastLoop = 0, lastPrint = 0, lastCheck = 0;
uint32_t n_total = 0, err_bit = 0, err_reset = 0;

/* 연속 거부 구간 추적
 * 샘플을 버리면 그 순간 가속도 보정 없이 자이로만 적분한다.
 * 흩어진 거부는 무해하지만 연속으로 뭉치면 각도가 틀어지므로,
 * 버린 총 개수보다 "최대 몇 개가 연속으로 버려졌는가"가 중요하다.
 *   최대연속 ≤ 5   (0.05초) → 무해
 *   최대연속 ~50  (0.5초)  → 드리프트 발생, 배선 점검
 */
uint16_t rej_run = 0, rej_run_max = 0;


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

  // ── 자이로 영점 (정지 상태 유지) ──
  Serial.println(">>> 캘리브레이션 중... 움직이지 마세요");
  long sgx = 0, sgy = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    sgx += gx;  sgy += gy;
    delay(5);
  }
  gyro_bias_x = (float)sgx / 200.0;
  gyro_bias_y = (float)sgy / 200.0;
  Serial.printf("gyro bias  x:%.1f  y:%.1f\n", gyro_bias_x, gyro_bias_y);

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
    n_total++;

    // ── 자이로 → dps ──
    float rate_pitch = ((float)gx - gyro_bias_x) / GYRO_LSB;
    float rate_roll  = ((float)gy - gyro_bias_y) / GYRO_LSB;

    // ── 가속도 기반 각도 (노이즈 있지만 절대 기준) ──
    float pitch_acc = atan2((float)ay,  sqrt(pow((float)ax, 2) + pow((float)az, 2))) * (180.0 / PI);
    float roll_acc  = atan2((float)-ax, sqrt(pow((float)ay, 2) + pow((float)az, 2))) * (180.0 / PI);

    /* ── 가속도 크기 검사 ──────────────────────────────────────────────
     * 정지 상태에서 |a| 는 반드시 1.0g.
     * 벗어나면 두 경우 중 하나이며, 어느 쪽이든 가속도계를
     * 기울기 기준으로 쓸 수 없으므로 이번 샘플은 자이로만 적분한다.
     *   (1) SPI 통신 오류로 값이 깨짐
     *   (2) 실제 선형 가속 중 → 중력 방향 추정 불가
     * (2)는 짐벌 제어 진입 후에도 계속 필요한 로직이므로 유지할 것.
     */
    float fax = ax, fay = ay, faz = az;
    float a_mag = sqrtf(fax * fax + fay * fay + faz * faz) / G_LSB;
    bool  acc_ok = (a_mag > ACC_MAG_MIN && a_mag < ACC_MAG_MAX);
    if (!acc_ok) err_bit++;

    if (acc_ok) {
      rej_run = 0;
      pitch_filtered = alpha_cf * (pitch_filtered + rate_pitch * dt) + (1.0 - alpha_cf) * pitch_acc;
      roll_filtered  = alpha_cf * (roll_filtered  + rate_roll  * dt) + (1.0 - alpha_cf) * roll_acc;
    } else {
      if (++rej_run > rej_run_max) rej_run_max = rej_run;
      pitch_filtered += rate_pitch * dt;           // 자이로만
      roll_filtered  += rate_roll  * dt;
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
      Serial.printf("[stat] reset:%lu  bit_err:%lu / %lu (%.1f%%)  최대연속:%u  (|a|=%.3f)\n",
                    err_reset, err_bit, n_total,
                    100.0f * err_bit / (n_total ? n_total : 1),
                    rej_run_max, a_mag);
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
 *       최대연속 ≤ 5     → 정상. 제어 단계로 진행 가능
 *       최대연속 50 이상 → 신호선 접촉 불량 (SDO/SCL/SDA 점퍼 확인, 납땜 검토)
 *       (움직이는 중이면 bit_err 는 정상적으로 올라가므로 반드시 정지 상태에서 볼 것)
 *  5) 포화 : 주행 중 pitch_acc 가 끊기면 ±2g 포화.
 *            CTRL1_XL_VAL → 0x68 (±4g), G_LSB → 8192.0 으로 변경.
 * ---------------------------------------------------------------------------*/
