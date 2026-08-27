/*
 * 모터_수평제어.ino
 * -----------------------------------------------------------------------------
 * 1단계 — 순수 수평제어 (ZV / 합력벡터 없음)
 *
 * 베이스(차체) IMU 가 읽은 기울기의 반대로 트레이를 돌려 절대 수평을 유지한다.
 * 차체가 +20° 기울면 트레이는 차체 기준 −20° 로 돌아 결과적으로 수평.
 *
 *   제어식 :  motor_cmd = −θ_base
 *
 *   목표각(θ_ref)은 항상 0. 가속도에 의한 슬로싱 대응(합력 벡터)과
 *   ZV 입력성형은 이 단계에서 넣지 않는다. 다음 단계에서 추가.
 *
 * 구성
 *   IMU_Reading.ino  — SPI IMU 읽기, 상보필터, 신뢰도 가중, 진단
 *   motor_test.ino   — CAN(TWAI) 짐벌모터 2축 구동
 *   두 코드를 합치고 그 사이에 제어식을 넣은 것.
 *
 * 하드웨어
 *   MCU     : ESP32-S3
 *   IMU     : LSM6DSOX, SPI 1MHz MODE0, CS = GPIO10
 *             (현재는 벤치용. 최종적으로 로봇 차체 하단에 부착할 센서)
 *   모터    : CAN 짐벌모터 2축, 1Mbit/s, TX=GPIO4 / RX=GPIO5
 *
 * 축 규약 (docs/CONTROL_INTEGRATION_TASK/README.md)
 *   pitch = X축 중심 회전,  +pitch = 보드 +Y축 쪽이 위로
 *   roll  = Y축 중심 회전,  +roll  = 보드 +X축 쪽이 아래로
 *
 *   모터는 X/Y 가 아니라 담당 각도로 이름 붙였다. 논문과 한이음 보고서의
 *   Motor_X / Motor_Y 대응이 서로 반대로 적혀 있어 혼동을 피하기 위함.
 *
 * 안전
 *   - 부팅 시 모터를 활성화하지 않는다. 시리얼로 arm 을 쳐야 움직인다.
 *   - 각도 제한, 속도 제한, 슬루(변화율) 제한 적용
 *   - GAIN 을 0.3 부터 올려가며 시험 (기본 0.3)
 *   - IMU 이상 또는 모터 오류 시 자동 비활성화
 *
 * 시리얼 명령 (115200)
 *   arm       모터 활성화 (트레이를 중립, 차체를 수평에 두고 실행할 것)
 *   stop      모터 비활성화
 *   z         현재 위치를 모터 영점으로 재설정
 *   g<값>     제어 게인 변경   예) g0.5
 *   ?         현재 상태 출력
 * -----------------------------------------------------------------------------
 */

#include <SPI.h>
#include "driver/twai.h"

// =============================================================================
//  설정
// =============================================================================

// ── IMU 핀 (보드 라벨 → GPIO) ──
#define PIN_MOSI  11    // SDA
#define PIN_SCK   12    // SCL
#define PIN_MISO  13    // SDO
#define CS_BASE   10    // CS  — 베이스(차체) IMU

#define SPI_SPEED 1000000
#define SPI_MD    SPI_MODE0

// ── CAN ──
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

/* 모터 담당 축.
 * 어느 물리 모터가 pitch 를 담당하는지 확인되면 두 값을 맞바꾸면 된다.
 */
#define CAN_ID_PITCH  0x01
#define CAN_ID_ROLL   0x02

/* 모터 회전 방향 부호.
 * 모터 + 방향과 트레이 각도 + 방향이 반대면 −1 로 뒤집는다.
 * 시운전 첫 항목에서 반드시 확인할 것 (아래 "시운전 순서" 참고).
 */
float DIR_PITCH = +1.0f;
float DIR_ROLL  = +1.0f;

// ── 제어 파라미터 ──
float GAIN            = 0.3f;    // 0.3 부터 시작해 1.0 까지 올린다
const float LIMIT_DEG = 25.0f;   // 모터 각도 제한
const float MAX_RATE  = 60.0f;   // 슬루 제한 [deg/s]
const float VEL_LIMIT = 2.0f;    // 모터 속도 제한 [rad/s]

// ── LSM6DSOX 레지스터 ──
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_STATUS     0x1E
#define REG_OUTX_L_G   0x22

#define WHO_AM_I_VAL   0x6C
#define CTRL1_XL_VAL   0x68    // 가속도 416Hz, ±4g
#define CTRL2_G_VAL    0x6C    // 자이로  416Hz, ±2000dps
#define CTRL3_C_VAL    0x44    // BDU=1, IF_INC=1

// ── 감도 / 필터 ──
const float G_LSB    = 8192.0;    // ±4g
const float GYRO_LSB = 14.286;    // ±2000dps
const float dt       = 0.01;      // 100Hz
const float alpha_cf = 0.98;

const float ACC_TRUST_FALLOFF = 0.5;
const float ACC_MAG_DEAD      = 0.05;

/* IMU 워치독 — 가속도 보정 없이 이만큼 연속되면 자세 추정을 믿을 수 없다.
 * 100 샘플 = 1초.
 */
const uint16_t REJ_RUN_FAULT = 100;

/* 모터 피드백 data[0] 상위니블은 오류코드가 아니라 상태코드다.
 *   0 = Disable, 1 = Enable  ← 정상
 *   8 이상       = 이상 (과전압/저전압/과전류/과열/통신끊김/과부하)
 * hardware/MOTOR_HANDOFF.md 참고.
 */
const uint8_t ST_FAULT_MIN = 0x8;

const char* statusName(uint8_t st) {
  switch (st) {
    case 0x0: return "Disable";
    case 0x1: return "Enable";
    case 0x8: return "과전압";
    case 0x9: return "저전압";
    case 0xA: return "과전류";
    case 0xB: return "MOS과열";
    case 0xC: return "권선과열";
    case 0xD: return "통신끊김";
    case 0xE: return "과부하";
    default:  return "미정의";
  }
}

// =============================================================================
//  상태 변수
// =============================================================================
float pitch_filtered = 0, roll_filtered = 0;
float gyro_bias_x = 0, gyro_bias_y = 0;
float g_scale = 1.0;

float cmd_pitch = 0, cmd_roll = 0;      // 실제로 내보낸 모터 지령 [deg]

bool  armed      = false;
bool  imu_ok     = false;
uint16_t rej_run = 0;

uint8_t  last_status[4] = {0xFF, 0xFF, 0xFF, 0xFF};   // 모터별 최근 상태코드
uint32_t fb_count = 0;                                 // 피드백 수신 횟수

unsigned long lastLoop = 0, lastPrint = 0, lastCheck = 0;
uint32_t n_win = 0, err_win = 0, err_reset = 0;


// =============================================================================
//  IMU — SPI 저수준
// =============================================================================
void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(CS_BASE, LOW);
  delayMicroseconds(20);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(CS_BASE, HIGH);
  SPI.endTransaction();
}

uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(CS_BASE, LOW);
  delayMicroseconds(20);
  SPI.transfer(reg | 0x80);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(CS_BASE, HIGH);
  SPI.endTransaction();
  return v;
}

// 자이로 3축 + 가속도 3축 (리틀엔디안, 자이로 먼저)
void readIMU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[12];
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(CS_BASE, LOW);
  delayMicroseconds(20);
  SPI.transfer(REG_OUTX_L_G | 0x80);
  for (int i = 0; i < 12; i++) b[i] = SPI.transfer(0x00);
  digitalWrite(CS_BASE, HIGH);
  SPI.endTransaction();

  gx = (int16_t)((b[1]  << 8) | b[0]);
  gy = (int16_t)((b[3]  << 8) | b[2]);
  gz = (int16_t)((b[5]  << 8) | b[4]);
  ax = (int16_t)((b[7]  << 8) | b[6]);
  ay = (int16_t)((b[9]  << 8) | b[8]);
  az = (int16_t)((b[11] << 8) | b[10]);
}

void configSensor() {
  writeReg(REG_CTRL3_C, 0x01);          // 소프트 리셋
  delay(50);
  writeReg(REG_CTRL3_C,  CTRL3_C_VAL);
  writeReg(REG_CTRL1_XL, CTRL1_XL_VAL);
  writeReg(REG_CTRL2_G,  CTRL2_G_VAL);
  delay(100);
}


// =============================================================================
//  CAN 모터
// =============================================================================
void sendPosVel(uint8_t id, float pos, float vel) {
  twai_message_t m = {};
  m.identifier = 0x100 | id;         // 위치/속도 모드
  m.data_length_code = 8;
  memcpy(&m.data[0], &pos, 4);       // 목표 위치 [rad] float LE
  memcpy(&m.data[4], &vel, 4);       // 속도 제한 [rad/s] float LE
  twai_transmit(&m, pdMS_TO_TICKS(5));
}

void sendUniversal(uint8_t id, uint8_t last) {  // FB=에러해제 FC=enable FD=disable FE=영점
  twai_message_t m = {};
  m.identifier = 0x100 | id;
  m.data_length_code = 8;
  memset(m.data, 0xFF, 7);
  m.data[7] = last;
  twai_transmit(&m, pdMS_TO_TICKS(5));
}

void broadcastUniversal(uint8_t last) {
  sendUniversal(CAN_ID_PITCH, last);
  sendUniversal(CAN_ID_ROLL,  last);
}


// =============================================================================
//  arm / disarm
// =============================================================================
void motorsArm() {
  if (!imu_ok) {
    Serial.println("!! IMU 이상 — arm 거부");
    return;
  }
  Serial.println(">>> arm: 트레이 중립 / 차체 수평 상태여야 합니다");
  broadcastUniversal(0xFB);  delay(50);   // 에러 해제
  broadcastUniversal(0xFE);  delay(50);   // 현재 위치를 영점으로
  broadcastUniversal(0xFC);  delay(100);  // enable

  cmd_pitch = 0;  cmd_roll = 0;           // 슬루 기준점 초기화
  armed = true;
  Serial.printf(">>> ARMED  (GAIN=%.2f, 제한 ±%.0f°)\n", GAIN, LIMIT_DEG);
}

void motorsDisarm(const char *reason) {
  if (armed) Serial.printf(">>> DISARM — %s\n", reason);
  broadcastUniversal(0xFD);               // disable
  armed = false;
  cmd_pitch = 0;  cmd_roll = 0;
}


// =============================================================================
//  유틸
// =============================================================================
// 변화율 제한 — 지령이 한 번에 튀지 않게
float slew(float target, float prev, float maxStep) {
  float d = target - prev;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  return prev + d;
}

void printStatus() {
  Serial.printf("[상태] %s  GAIN=%.2f  DIR(p/r)=%+.0f/%+.0f  IMU=%s\n",
                armed ? "ARMED" : "STOP", GAIN, DIR_PITCH, DIR_ROLL,
                imu_ok ? "OK" : "FAULT");
  Serial.printf("       pitch=%.2f roll=%.2f  cmd(p/r)=%.2f/%.2f\n",
                pitch_filtered, roll_filtered, cmd_pitch, cmd_roll);
  Serial.printf("       모터 피드백 %lu회  P(id%u)=%s  R(id%u)=%s\n",
                fb_count,
                CAN_ID_PITCH, statusName(last_status[CAN_ID_PITCH & 0x03]),
                CAN_ID_ROLL,  statusName(last_status[CAN_ID_ROLL  & 0x03]));
  if (fb_count == 0) {
    Serial.println("       !! 피드백 0회 — CAN 배선/종단저항/모터 전원 확인");
  }
}

/* CAN 경로만 따로 확인하는 시험 구동.
 * motor_test.ino 와 동일하게 ±0.5 rad 을 왕복시킨다. IMU 와 제어식을 거치지
 * 않으므로, 이게 움직이면 CAN 은 정상이고 문제는 제어 쪽에 있다는 뜻이다.
 */
void motorSweep() {
  Serial.println(">>> 시험 구동 6초 (±0.5 rad). 손을 치우세요.");
  broadcastUniversal(0xFB);  delay(50);
  broadcastUniversal(0xFE);  delay(50);
  broadcastUniversal(0xFC);  delay(100);

  bool dir = true;
  uint32_t t0 = millis(), last = 0;
  while (millis() - t0 < 6000) {
    if (millis() - last >= 500) { last = millis(); dir = !dir; }
    sendPosVel(CAN_ID_PITCH, dir ? 0.5f : -0.5f, 1.0f);
    sendPosVel(CAN_ID_ROLL,  dir ? 0.5f : -0.5f, 1.0f);

    twai_message_t rx;
    while (twai_receive(&rx, 0) == ESP_OK) {
      uint8_t mid = rx.data[0] & 0x0F;
      uint8_t st  = rx.data[0] >> 4;
      last_status[mid & 0x03] = st;
      fb_count++;
    }
    delay(20);
  }
  broadcastUniversal(0xFD);
  Serial.printf(">>> 시험 종료. 피드백 %lu회  P=%s  R=%s\n", fb_count,
                statusName(last_status[CAN_ID_PITCH & 0x03]),
                statusName(last_status[CAN_ID_ROLL  & 0x03]));
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;

  if (s.equalsIgnoreCase("arm")) {
    motorsArm();
  } else if (s.equalsIgnoreCase("stop") || s.equalsIgnoreCase("s")) {
    motorsDisarm("사용자 명령");
  } else if (s.equalsIgnoreCase("z")) {
    broadcastUniversal(0xFE);
    cmd_pitch = 0;  cmd_roll = 0;
    Serial.println(">>> 모터 영점 재설정");
  } else if (s.startsWith("g") || s.startsWith("G")) {
    float v = s.substring(1).toFloat();
    if (v >= 0.0f && v <= 1.5f) {
      GAIN = v;
      Serial.printf(">>> GAIN = %.2f\n", GAIN);
    } else {
      Serial.println("!! GAIN 범위 0.0 ~ 1.5");
    }
  } else if (s.equalsIgnoreCase("t")) {
    motorsDisarm("시험 구동 진입");
    motorSweep();
  } else if (s == "?") {
    printStatus();
  } else {
    Serial.println("명령: arm / stop / z / t / g<값> / ?");
  }
}


// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // ── IMU ──
  pinMode(CS_BASE, OUTPUT);
  digitalWrite(CS_BASE, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  delay(100);

  uint8_t id = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I: 0x%02X (기대값 0x%02X)\n", id, WHO_AM_I_VAL);
  imu_ok = (id == WHO_AM_I_VAL);
  if (!imu_ok) {
    Serial.println("!! IMU 응답 없음 — 0xFF: MISO/CS 확인, 0x00: VIN/GND 확인");
  }
  configSensor();

  // ── 자이로 영점 + 중력 스케일 ──
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
  if (measured > 0.5f && measured < 1.5f) g_scale = measured;
  Serial.printf("gyro bias x:%.1f y:%.1f   g_scale:%.4f\n",
                gyro_bias_x, gyro_bias_y, g_scale);

  // 가속도 기준 초기값
  {
    int16_t ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    pitch_filtered = atan2((float)ay,  sqrt(pow((float)ax, 2) + pow((float)az, 2))) * (180.0 / PI);
    roll_filtered  = atan2((float)-ax, sqrt(pow((float)ay, 2) + pow((float)az, 2))) * (180.0 / PI);
  }

  // ── CAN ──
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
  delay(100);
  broadcastUniversal(0xFD);        // 부팅 시에는 비활성 상태로 둔다

  lastLoop = micros();
  Serial.println("\n==================================================");
  Serial.println(" 모터는 비활성 상태입니다. 'arm' 을 입력해야 움직입니다.");
  Serial.println("--------------------------------------------------");
  Serial.println("  arm    모터 활성화 (차체 수평 / 트레이 중립에서)");
  Serial.println("  stop   비활성화");
  Serial.println("  t      시험 구동 — CAN 경로만 확인 (IMU 무관)");
  Serial.println("  z      모터 영점 재설정");
  Serial.println("  g<값>  게인 변경   예) g0.5");
  Serial.println("  ?      상태 출력");
  Serial.println("==================================================\n");
}


// =============================================================================
//  loop
// =============================================================================
void loop() {
  handleSerial();

  if ((long)(micros() - lastLoop) < 10000) return;   // 100Hz
  lastLoop += 10000;

  // ─────────────────────────────────────────────────────────────────
  //  1. 자세 추정  (IMU_Reading.ino 와 동일)
  // ─────────────────────────────────────────────────────────────────
  int16_t ax, ay, az, gx, gy, gz;
  readIMU(ax, ay, az, gx, gy, gz);

  float rate_pitch = ((float)gx - gyro_bias_x) / GYRO_LSB;
  float rate_roll  = ((float)gy - gyro_bias_y) / GYRO_LSB;

  float pitch_acc = atan2((float)ay,  sqrt(pow((float)ax, 2) + pow((float)az, 2))) * (180.0 / PI);
  float roll_acc  = atan2((float)-ax, sqrt(pow((float)ay, 2) + pow((float)az, 2))) * (180.0 / PI);

  float fax = ax, fay = ay, faz = az;
  float a_mag = sqrtf(fax * fax + fay * fay + faz * faz) / G_LSB / g_scale;

  // 1g 에서 벗어난 만큼 가속도계 반영 비중을 줄인다
  float dev   = fabsf(a_mag - 1.0f);
  float trust = constrain(1.0f - dev / ACC_TRUST_FALLOFF, 0.0f, 1.0f);
  float alpha = 1.0f - (1.0f - alpha_cf) * trust;

  pitch_filtered = alpha * (pitch_filtered + rate_pitch * dt) + (1.0f - alpha) * pitch_acc;
  roll_filtered  = alpha * (roll_filtered  + rate_roll  * dt) + (1.0f - alpha) * roll_acc;

  n_win++;
  if (trust <= 0.0f) {
    err_win++;
    rej_run++;
  } else {
    rej_run = 0;
  }

  // ─────────────────────────────────────────────────────────────────
  //  2. 이상 감지
  // ─────────────────────────────────────────────────────────────────
  // 센서 파워다운 즉시 복구
  if (a_mag < ACC_MAG_DEAD && readReg(REG_CTRL1_XL) != CTRL1_XL_VAL) {
    err_reset++;
    Serial.println("!! 센서 파워다운 감지 — 즉시 재설정");
    configSensor();
    motorsDisarm("센서 리셋");
  }

  // 자세 추정을 믿을 수 없을 만큼 오래 무보정이면 정지
  if (rej_run > REJ_RUN_FAULT) {
    imu_ok = false;
    motorsDisarm("IMU 무보정 구간 과다");
  }

  // 모터 피드백 수신 및 오류 확인
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    uint8_t mid = rx.data[0] & 0x0F;
    uint8_t st  = rx.data[0] >> 4;      // 상태코드 (오류코드가 아님)

    last_status[mid & 0x03] = st;
    fb_count++;

    if (st >= ST_FAULT_MIN && armed) {
      Serial.printf("!! 모터 %u 이상 — %s\n", mid, statusName(st));
      motorsDisarm("모터 이상");
    }
  }

  // ─────────────────────────────────────────────────────────────────
  //  3. 제어  —  motor_cmd = −θ_base
  // ─────────────────────────────────────────────────────────────────
  /* 트레이가 절대 수평이 되려면 차체 기울기의 반대로 돌려야 한다.
   * 목표각(θ_ref)은 이 단계에서 항상 0 이므로 (0 − θ_base) = −θ_base.
   * 합력 벡터와 ZV 는 다음 단계에서 θ_ref 자리에 들어간다.
   */
  float want_pitch = -pitch_filtered * GAIN * DIR_PITCH;
  float want_roll  = -roll_filtered  * GAIN * DIR_ROLL;

  want_pitch = constrain(want_pitch, -LIMIT_DEG, LIMIT_DEG);
  want_roll  = constrain(want_roll,  -LIMIT_DEG, LIMIT_DEG);

  // 슬루 제한 — 한 주기당 최대 변화량
  const float maxStep = MAX_RATE * dt;
  cmd_pitch = slew(want_pitch, cmd_pitch, maxStep);
  cmd_roll  = slew(want_roll,  cmd_roll,  maxStep);

  if (armed) {
    sendPosVel(CAN_ID_PITCH, cmd_pitch * DEG_TO_RAD, VEL_LIMIT);
    sendPosVel(CAN_ID_ROLL,  cmd_roll  * DEG_TO_RAD, VEL_LIMIT);
  }

  // ─────────────────────────────────────────────────────────────────
  //  4. 출력
  // ─────────────────────────────────────────────────────────────────
  if (millis() - lastPrint > 100) {         // 10Hz
    lastPrint = millis();
    Serial.printf("pitch:%.2f,cmd_pitch:%.2f,roll:%.2f,cmd_roll:%.2f\n",
                  pitch_filtered, cmd_pitch, roll_filtered, cmd_roll);
  }

  if (millis() - lastCheck > 2000) {        // 2초
    lastCheck = millis();
    uint8_t c1 = readReg(REG_CTRL1_XL);
    if (c1 != CTRL1_XL_VAL) {
      err_reset++;
      Serial.printf("!! 센서 리셋 CTRL1_XL=0x%02X — 재설정\n", c1);
      configSensor();
      motorsDisarm("센서 리셋");
    }
    Serial.printf("[stat] %s 모터P=%s R=%s 피드백:%lu | reset:%lu 무보정:%lu/%lu trust=%.2f\n",
                  armed ? "ARMED" : "STOP",
                  statusName(last_status[CAN_ID_PITCH & 0x03]),
                  statusName(last_status[CAN_ID_ROLL  & 0x03]),
                  fb_count, err_reset, err_win, n_win, trust);
    n_win = 0;  err_win = 0;
  }
}

/* -----------------------------------------------------------------------------
 * 시운전 순서
 *
 *  0) 준비
 *     트레이 위에 아무것도 올리지 말고, 주변에 손·물체를 치운다.
 *     전원공급기 전류 제한을 낮게 (1~2A) 걸어둔다.
 *
 *  1) 모터 없이 부호 확인  ← 가장 먼저
 *     arm 하지 않은 상태로 차체를 기울이며 cmd_pitch / cmd_roll 을 본다.
 *     pitch 가 +10 일 때 cmd_pitch 가 −3 (GAIN 0.3) 이면 정상.
 *     같은 부호로 나오면 제어식이 아니라 출력이 뒤집힌 것이므로 코드를 확인한다.
 *
 *  2) 모터 방향 확인
 *     차체를 수평, 트레이를 중립에 두고 arm.
 *     차체를 한쪽으로 천천히 기울인다.
 *       트레이가 반대로 돌아 수평을 유지 → DIR 정상
 *       트레이가 같은 방향으로 더 기울어짐 → 즉시 stop, 해당 축 DIR 을 −1 로
 *     ★ 이 확인 전에는 절대 GAIN 을 올리지 말 것. 방향이 반대면 발산한다.
 *
 *  3) 게인 상승
 *     g0.5 → g0.7 → g1.0 순으로 올리며 각 단계에서 진동 여부를 본다.
 *     떨림이 생기면 한 단계 내리고, VEL_LIMIT / MAX_RATE 를 조정한다.
 *
 *  4) 정량 확인
 *     차체를 20° 기울였을 때 GAIN=1.0 에서 cmd 가 −20° 근처인지,
 *     트레이가 실제로 수평인지 각도기 또는 상단 IMU 로 확인한다.
 *
 * 다음 단계
 *   이 수평제어가 안정되면 θ_ref 자리에 합력 벡터 목표각을 넣고,
 *   그 다음 ZV 입력성형을 추가한다.
 * ---------------------------------------------------------------------------*/
