/*
 * 합력벡터_제어.ino
 * -----------------------------------------------------------------------------
 * 2단계 — 합력 벡터 목표각 (ZV 입력성형은 아직 없음)
 *
 * 1단계 [모터_수평제어_I2C.ino](../[1단계]%20수평제어/모터_수평제어_I2C.ino) 에서
 * 목표각 θ_ref 만 0 이 아니게 바뀐 것이다. 나머지는 전부 같다.
 *
 *   1단계 :  θ_ref = 0                    →  motor_cmd = −θ_base   (수평 유지)
 *   2단계 :  θ_ref = atan(a_linear / g)   →  motor_cmd = θ_ref − θ_base
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 왜 수평이 정답이 아닌가
 *
 * 가속하는 차 안의 물컵을 생각하면 된다. 컵을 수평으로 붙잡고 있으면 물은
 * 뒤로 쏠린다. 액체 입장에서는 중력과 관성력의 **합력**이 진짜 아래쪽이므로,
 * 컵 바닥을 그 합력에 수직으로 놓아야 수면이 컵 기준으로 평평해진다.
 *
 *     정지·등속  :  합력 = 중력        →  θ_ref = 0      (수평이 정답)
 *     가속 중    :  합력 = 중력+관성력  →  θ_ref ≠ 0      (기울여야 정답)
 *
 * 그래서 수평제어와 안티슬로싱이 별개의 모드가 아니다. 아래 한 식이 두 경우를
 * 모두 덮는다. 경사면에 서 있으면 a_linear 가 0 이라 θ_ref 도 0 이 되어
 * 자동으로 절대 수평이 된다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 계산 순서
 *
 *   1. 상보필터로 차체 절대 자세 θ_base 를 얻는다            (1단계와 동일)
 *   2. 측정 가속도에서 중력 성분을 뺀다
 *
 *        a_lin_y = ay − g·sin(pitch)
 *        a_lin_x = ax − g·sin(roll)
 *
 *      가속도계는 기울어져 있기만 해도 중력을 읽는다. 그 몫을 빼야 순수한
 *      선형 가속만 남는다. 뺄 때 쓰는 각도가 1단계에서 만든 θ_base 다.
 *
 *   3. 합력이 수직에서 벗어난 각도
 *
 *        θ_ref = atan(a_lin / g)
 *
 *   4. 모터는 차체 기준으로 도므로 차체 자세를 뺀다
 *
 *        motor_cmd = θ_ref − θ_base
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ⚠ 부호를 확인하기 전에는 켜지 마라
 *
 * ACC_GAIN 기본값이 **0** 이라 처음 올리면 1단계와 완전히 같게 동작한다.
 * θ_ref 는 계산되어 화면에만 나오고 모터로는 가지 않는다.
 *
 * 부호가 반대면 액체를 **더** 흔드는 방향으로 기울인다. 아래 절차로 확인한 뒤
 * ag 명령으로 조금씩 올릴 것. 파일 끝의 "부호 확인 절차" 참고.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 배선·모터는 1단계와 동일
 *   IMU  : LSM6DSOX, I2C 0x6A (CS→3V3, SDO→GND), SDA=11 SCL=12
 *   모터 : GL60II ×2, MIT 모드, CAN TX=4 / RX=5
 *          바깥축 0x01 (Master 0x00) / 안쪽축 0x02 (Master 0x11)
 *
 * 시리얼 명령 (115200)
 *   arm / stop / z / t / q / ?          1단계와 동일
 *   g<값>     수평 게인      예) g1.0
 *   ag<값>    합력 게인      예) ag0.3   ★ 2단계에서 추가. 기본 0
 *   ad<값>    합력 저역통과  예) ad0.2   (작을수록 부드럽고 느림)
 *   kp / kd / f / d                      1단계와 동일
 * -----------------------------------------------------------------------------
 */

#include <Wire.h>
#include "driver/twai.h"

// =============================================================================
//  설정
// =============================================================================

// ── IMU (I2C) ──
#define PIN_SDA    11
#define PIN_SCL    12
#define I2C_HZ     400000
#define IMU_ADDR   0x6A      // SDO→GND. 3V3 이면 0x6B

// ── CAN ──
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

/* 제어모드 접두.  CAN ID = (모드 << 8) | 모터ID
 *   0x000 = MIT,  0x100 = 위치/속도,  0x200 = 속도
 */
#define CAN_MODE  0x000

#define CAN_ID_PITCH  0x02
#define CAN_ID_ROLL   0x01

/* 모터 회전 방향 부호. 트레이가 반대로 돌면 해당 축을 −1 로 뒤집는다. */
float DIR_PITCH = +1.0f;
float DIR_ROLL  = +1.0f;

// ── 제어 파라미터 ──
float GAIN            = 0.3f;    // 0.3 부터 올린다
const float LIMIT_DEG = 25.0f;   // 지령 각도 제한
float MAX_RATE        = 30.0f;   // 슬루 제한 [deg/s]
float CMD_LPF         = 0.15f;   // 지령 저역통과
int   CAN_DIV         = 2;       // 100Hz / 2 = 50Hz 전송

/* 모터 임피던스 게인 — 실측값 (2026-08-27)
 *   Kp 1     트레이를 잡는다. Kp 3 은 Kd 0 에서 발산했다.
 *   Kd 0.1   밀었다 놓으면 왕복 없이 복귀. 0.05 는 1회 오버슛.
 *
 * ⚠ MIT 모드에는 속도 제한이 없다. 목표가 튀면 Kp × 오차 가 즉시 토크로 나간다.
 *   Kp 20 에 0.5 rad 오차면 10 N·m 로 TMAX 포화다. 실제로 기구가 튕긴 적이 있다.
 * ⚠ Kd 가 0 이 아니면 미세 진동이 생긴다. 엔코더 속도 추정 노이즈에 곱해지는
 *   것이라 원리적으로 못 없앤다.
 */
float MOTOR_KP = 1.0f;
float MOTOR_KD = 0.1f;

const float TORQUE_WARN = 3.0f;

/* ── 합력 벡터 (2단계) ────────────────────────────────────────────────────
 * ACC_GAIN 0 이면 θ_ref 가 항상 0 이라 1단계(수평 유지)와 완전히 같다.
 * 부호를 확인한 뒤 0.3 → 0.6 → 1.0 으로 올린다.
 *
 * DIR_ACC 는 합력 목표각의 부호. 가속 방향과 기울여야 할 방향의 관계는
 * IMU 장착 방향에 따라 달라지므로 실측으로 정한다.
 * 반대면 액체를 더 흔든다. 파일 끝 "부호 확인 절차" 참고.
 *
 * ACC_LPF 는 θ_ref 전용 저역통과. 선형 가속 추정은 노이즈가 크므로 그대로
 * 쓰면 지령이 떨린다. 다만 필터가 세면 지연이 늘어 슬로싱 억제에 불리하다.
 */
float ACC_GAIN = 0.0f;      // ★ 기본 0 — 확인 전까지 1단계와 동일하게 동작
float DIR_ACC  = +1.0f;     // 부호. 실측 후 −1 이 될 수 있다
float ACC_LPF  = 0.2f;      // θ_ref 저역통과
const float ACC_REF_LIMIT = 20.0f;   // θ_ref 자체의 상한 [deg]

// 계산된 목표각 (표시·기록용)
float ref_pitch = 0, ref_roll = 0;

/* 폭주 차단 — 지령은 ±LIMIT_DEG 를 넘지 않으므로 실제 위치가 이 값을 넘으면
 * 우리가 시킨 움직임이 아니다.
 */
const float ACT_LIMIT_DEG = 50.0f;
const uint32_t FB_TIMEOUT_MS = 300;
unsigned long lastFb = 0;

// ── MIT 변환 범위 — 드라이버 설정과 일치해야 한다 ──
const float P_MIN  = -12.5f, P_MAX  =  12.5f;   // rad
const float V_MIN  = -30.0f, V_MAX  =  30.0f;   // rad/s (VMAX 30)
const float KP_MIN =   0.0f, KP_MAX = 500.0f;
const float KD_MIN =   0.0f, KD_MAX =   5.0f;
const float T_MIN  = -10.0f, T_MAX  =  10.0f;

// ── LSM6DSOX 레지스터 (I2C·SPI 공통) ──
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_OUTX_L_G   0x22

#define WHO_AM_I_VAL   0x6C
#define CTRL1_XL_VAL   0x68    // 416Hz, ±4g
#define CTRL2_G_VAL    0x6C    // 416Hz, ±2000dps
#define CTRL3_C_VAL    0x44    // BDU=1, IF_INC=1

const float G_LSB    = 8192.0;
const float GYRO_LSB = 14.286;
const float dt       = 0.01;
const float alpha_cf = 0.98;

const float ACC_TRUST_FALLOFF = 0.5;
const float ACC_MAG_DEAD      = 0.05;
const uint16_t REJ_RUN_FAULT  = 100;

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

float cmd_pitch = 0, cmd_roll = 0;

bool  armed  = false;
bool  imu_ok = false;
bool  stream = true;
uint16_t rej_run = 0;

uint8_t  last_status[4] = {0xFF, 0xFF, 0xFF, 0xFF};
float    act_deg[4]     = {0, 0, 0, 0};
uint32_t fb_count = 0;
uint32_t i2c_err  = 0;             // I2C 읽기 실패 누적

unsigned long lastLoop = 0, lastPrint = 0, lastCheck = 0;
uint32_t n_win = 0, err_win = 0, err_reset = 0;


// =============================================================================
//  IMU — I2C
// =============================================================================
/* SPI 판과 다른 부분은 여기뿐이다.
 * I2C 에는 SPI 의 "읽기는 MSB=1" 같은 규칙이 없다. 레지스터 주소를 쓰고
 * repeated start 로 곧바로 읽는다. 연속 읽기는 CTRL3_C 의 IF_INC 가 1 이어야
 * 주소가 자동 증가한다 (CTRL3_C_VAL 0x44 에 포함).
 */
bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { i2c_err++; return false; }   // repeated start
  if (Wire.requestFrom((int)IMU_ADDR, (int)n) != n) { i2c_err++; return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

uint8_t readReg(uint8_t reg) {
  uint8_t v = 0;
  i2cRead(reg, &v, 1);        // 실패하면 0 — 파워다운 감지가 이를 잡는다
  return v;
}

void writeReg(uint8_t reg, uint8_t val) { i2cWrite(reg, val); }

// 자이로 3축 + 가속도 3축 (리틀엔디안, 자이로 먼저)
void readIMU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[12] = {0};
  i2cRead(REG_OUTX_L_G, b, 12);   // 실패하면 전부 0 → 아래 검사에 걸린다

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
//  MIT 모드 CAN  (SPI 판과 동일)
// =============================================================================
uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits) {
  float span = xmax - xmin;
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  uint32_t maxv = (1UL << bits) - 1;
  uint32_t v = (uint32_t)((x - xmin) * ((float)(1UL << bits) / span));
  return (uint16_t)(v > maxv ? maxv : v);
}

float uint_to_float(uint16_t xi, float xmin, float xmax, uint8_t bits) {
  float span = xmax - xmin;
  return ((float)xi) * span / ((float)((1UL << bits) - 1)) + xmin;
}

/* MIT 제어 프레임 (매뉴얼 42p)
 *   D0 pos[15:8]  D1 pos[7:0]  D2 vel[11:4]  D3 vel[3:0]|kp[11:8]
 *   D4 kp[7:0]    D5 kd[11:4]  D6 kd[3:0]|t[11:8]        D7 t[7:0]
 */
void sendMIT(uint8_t id, float pos, float vel, float kp, float kd, float tff) {
  uint16_t p   = float_to_uint(pos, P_MIN,  P_MAX,  16);
  uint16_t v   = float_to_uint(vel, V_MIN,  V_MAX,  12);
  uint16_t kpi = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
  uint16_t kdi = float_to_uint(kd,  KD_MIN, KD_MAX, 12);
  uint16_t t   = float_to_uint(tff, T_MIN,  T_MAX,  12);

  twai_message_t m = {};
  m.identifier = CAN_MODE | id;
  m.data_length_code = 8;
  m.data[0] = p >> 8;
  m.data[1] = p & 0xFF;
  m.data[2] = v >> 4;
  m.data[3] = ((v & 0x0F) << 4) | (kpi >> 8);
  m.data[4] = kpi & 0xFF;
  m.data[5] = kdi >> 4;
  m.data[6] = ((kdi & 0x0F) << 4) | (t >> 8);
  m.data[7] = t & 0xFF;
  twai_transmit(&m, pdMS_TO_TICKS(5));
}

void sendUniversal(uint8_t id, uint8_t last) {  // FB=에러해제 FC=enable FD=disable FE=영점
  twai_message_t m = {};
  m.identifier = CAN_MODE | id;
  m.data_length_code = 8;
  memset(m.data, 0xFF, 7);
  m.data[7] = last;
  twai_transmit(&m, pdMS_TO_TICKS(5));
}

void broadcastUniversal(uint8_t last) {
  sendUniversal(CAN_ID_PITCH, last);
  sendUniversal(CAN_ID_ROLL,  last);
}

void sendBoth() {
  sendMIT(CAN_ID_PITCH, cmd_pitch * DEG_TO_RAD, 0.0f, MOTOR_KP, MOTOR_KD, 0.0f);
  sendMIT(CAN_ID_ROLL,  cmd_roll  * DEG_TO_RAD, 0.0f, MOTOR_KP, MOTOR_KD, 0.0f);
}


// =============================================================================
//  arm / disarm
// =============================================================================
void motorsDisarm(const char *reason) {
  if (armed) Serial.printf(">>> DISARM — %s\n", reason);
  broadcastUniversal(0xFD);
  armed = false;
  cmd_pitch = 0;  cmd_roll = 0;
}

void motorsArm() {
  // 저장된 플래그를 믿지 말고 직접 읽는다 (워치독이 내린 뒤 스스로 안 올라온다)
  uint8_t id = readReg(REG_WHO_AM_I);
  imu_ok = (id == WHO_AM_I_VAL);
  if (!imu_ok) {
    Serial.printf("!! IMU 응답 0x%02X (기대 0x%02X) — arm 거부\n", id, WHO_AM_I_VAL);
    return;
  }
  rej_run = 0;

  Serial.println(">>> arm: 트레이 중립 / 차체 수평 상태여야 합니다");
  broadcastUniversal(0xFB);  delay(50);   // 에러 해제
  broadcastUniversal(0xFE);  delay(50);   // 현재 위치를 영점으로

  // enable 전에 목표를 현재 위치(0)로 박아둔다. 빼면 마지막 목표로 확 튄다.
  cmd_pitch = 0;  cmd_roll = 0;
  sendBoth();
  delay(50);

  broadcastUniversal(0xFC);  delay(100);  // enable

  for (int i = 0; i < 4; i++) act_deg[i] = 0;   // 영점 후 낡은 위치값 제거
  lastFb = millis();
  armed = true;
  Serial.printf(">>> ARMED  GAIN=%.2f  ACC_GAIN=%.2f  Kp=%.1f  Kd=%.2f  제한 ±%.0f°\n",
                GAIN, ACC_GAIN, MOTOR_KP, MOTOR_KD, LIMIT_DEG);
}


// =============================================================================
//  유틸
// =============================================================================
float slew(float target, float prev, float maxStep) {
  float d = target - prev;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  return prev + d;
}

void printStatus() {
  Serial.printf("[상태] %s  GAIN=%.2f  Kp=%.1f Kd=%.2f  DIR(p/r)=%+.0f/%+.0f  IMU=%s\n",
                armed ? "ARMED" : "STOP", GAIN, MOTOR_KP, MOTOR_KD,
                DIR_PITCH, DIR_ROLL, imu_ok ? "OK" : "FAULT");
  Serial.printf("       ACC_GAIN=%.2f  DIR_ACC=%+.0f  ACC_LPF=%.2f   ref(p/r)=%.2f/%.2f\n",
                ACC_GAIN, DIR_ACC, ACC_LPF, ref_pitch, ref_roll);
  Serial.printf("       pitch=%.2f roll=%.2f  cmd(p/r)=%.2f/%.2f  act(p/r)=%.2f/%.2f\n",
                pitch_filtered, roll_filtered, cmd_pitch, cmd_roll,
                act_deg[CAN_ID_PITCH & 0x03], act_deg[CAN_ID_ROLL & 0x03]);
  Serial.printf("       피드백 %lu회  I2C오류 %lu회  P(id%u)=%s  R(id%u)=%s\n",
                fb_count, i2c_err,
                CAN_ID_PITCH, statusName(last_status[CAN_ID_PITCH & 0x03]),
                CAN_ID_ROLL,  statusName(last_status[CAN_ID_ROLL  & 0x03]));
  if (fb_count == 0) {
    Serial.println("       !! 피드백 0회 — ControlMode 가 MIT 인지, CAN 배선 확인");
  }
}

void drainCAN() {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    uint8_t mid = rx.data[0] & 0x0F;
    uint8_t st  = rx.data[0] >> 4;
    uint8_t i   = mid & 0x03;

    uint16_t praw = ((uint16_t)rx.data[1] << 8) | rx.data[2];
    act_deg[i] = uint_to_float(praw, P_MIN, P_MAX, 16) * RAD_TO_DEG;

    last_status[i] = st;
    fb_count++;
    lastFb = millis();

    if (st >= ST_FAULT_MIN && armed) {
      Serial.printf("!! 모터 %u 이상 — %s\n", mid, statusName(st));
      motorsDisarm("모터 이상");
    }
  }

  if (!armed) return;

  float ap = act_deg[CAN_ID_PITCH & 0x03];
  float ar = act_deg[CAN_ID_ROLL  & 0x03];
  if (fabsf(ap) > ACT_LIMIT_DEG || fabsf(ar) > ACT_LIMIT_DEG) {
    Serial.printf("!! 폭주 차단 — 실제 위치 P=%.1f° R=%.1f° (한계 ±%.0f°)\n",
                  ap, ar, ACT_LIMIT_DEG);
    motorsDisarm("위치 한계 초과");
    return;
  }

  if (lastFb != 0 && millis() - lastFb > FB_TIMEOUT_MS) {
    Serial.printf("!! 피드백 %lums 끊김\n", millis() - lastFb);
    motorsDisarm("피드백 두절");
  }
}

/* CAN 경로만 확인하는 시험 구동. IMU·제어식을 거치지 않는다.
 * MIT 는 속도 제한이 없으므로 계단이 아니라 램프로 움직인다.
 */
void motorSweep() {
  const float SWEEP_DEG  = 8.0f;
  const float SWEEP_RATE = 20.0f;

  Serial.printf(">>> 시험 구동 8초 (±%.0f°, %.0f°/s). 손을 치우세요.\n",
                SWEEP_DEG, SWEEP_RATE);
  broadcastUniversal(0xFB);  delay(50);
  broadcastUniversal(0xFE);  delay(50);
  sendMIT(CAN_ID_PITCH, 0, 0, MOTOR_KP, MOTOR_KD, 0);
  sendMIT(CAN_ID_ROLL,  0, 0, MOTOR_KP, MOTOR_KD, 0);
  delay(50);
  broadcastUniversal(0xFC);  delay(100);

  float p = 0;
  int   dir = +1;
  uint32_t t0 = millis(), last = millis();
  while (millis() - t0 < 8000) {
    uint32_t now = millis();
    p += dir * SWEEP_RATE * (now - last) * 0.001f;
    last = now;
    if (p >=  SWEEP_DEG) { p =  SWEEP_DEG; dir = -1; }
    if (p <= -SWEEP_DEG) { p = -SWEEP_DEG; dir = +1; }

    sendMIT(CAN_ID_PITCH, p * DEG_TO_RAD, 0, MOTOR_KP, MOTOR_KD, 0);
    sendMIT(CAN_ID_ROLL,  p * DEG_TO_RAD, 0, MOTOR_KP, MOTOR_KD, 0);
    drainCAN();
    delay(20);
  }
  while (fabsf(p) > 0.5f) {           // 기울어진 채로 끄지 않는다
    p += (p > 0 ? -0.4f : 0.4f);
    sendMIT(CAN_ID_PITCH, p * DEG_TO_RAD, 0, MOTOR_KP, MOTOR_KD, 0);
    sendMIT(CAN_ID_ROLL,  p * DEG_TO_RAD, 0, MOTOR_KP, MOTOR_KD, 0);
    delay(20);
  }
  broadcastUniversal(0xFD);
  Serial.printf(">>> 시험 종료. 피드백 %lu회\n", fb_count);
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;
  String u = s;  u.toLowerCase();

  if (u == "arm") {
    motorsArm();
  } else if (u == "stop" || u == "s") {
    motorsDisarm("사용자 명령");
  } else if (u == "z") {
    broadcastUniversal(0xFE);
    cmd_pitch = 0;  cmd_roll = 0;
    Serial.println(">>> 모터 영점 재설정");
  } else if (u == "t") {
    motorsDisarm("시험 구동 진입");
    motorSweep();
  } else if (u.startsWith("kp")) {          // kd 보다 먼저 검사
    float v = s.substring(2).toFloat();
    if (v >= KP_MIN && v <= KP_MAX) {
      MOTOR_KP = v;
      float tmax = MOTOR_KP * LIMIT_DEG * DEG_TO_RAD;
      Serial.printf(">>> Kp = %.1f N/rad  (%.0f° 오차에서 %.2f N·m)\n",
                    MOTOR_KP, LIMIT_DEG, tmax);
      if (tmax > TORQUE_WARN)
        Serial.printf("   !! %.1f N·m 초과 — 목표가 튀면 그만큼 나갑니다\n", TORQUE_WARN);
    } else Serial.printf("!! Kp 범위 %.0f ~ %.0f\n", KP_MIN, KP_MAX);
  } else if (u.startsWith("kd")) {
    float v = s.substring(2).toFloat();
    if (v >= KD_MIN && v <= KD_MAX) {
      MOTOR_KD = v;
      Serial.printf(">>> Kd = %.2f N·s/rad\n", MOTOR_KD);
      if (v == 0.0f) Serial.println("   !! Kd=0 은 매뉴얼상 발진·폭주 조건입니다");
    } else Serial.printf("!! Kd 범위 %.0f ~ %.0f\n", KD_MIN, KD_MAX);
  } else if (u.startsWith("ag")) {          // g 보다 먼저 검사할 것
    float v = s.substring(2).toFloat();
    if (v >= 0.0f && v <= 1.5f) {
      ACC_GAIN = v;
      Serial.printf(">>> ACC_GAIN = %.2f  (합력 벡터 %s)\n",
                    ACC_GAIN, ACC_GAIN == 0.0f ? "꺼짐 — 1단계와 동일" : "켜짐");
      if (ACC_GAIN > 0.0f)
        Serial.println("   부호를 확인했습니까? 반대면 액체를 더 흔듭니다");
    } else Serial.println("!! ACC_GAIN 범위 0 ~ 1.5");
  } else if (u.startsWith("ad")) {
    float v = s.substring(2).toFloat();
    if (v > 0.0f && v <= 1.0f) { ACC_LPF = v; Serial.printf(">>> ACC_LPF = %.2f\n", ACC_LPF); }
    else Serial.println("!! ACC_LPF 범위 0 ~ 1");
  } else if (u.startsWith("g")) {
    float v = s.substring(1).toFloat();
    if (v >= 0.0f && v <= 1.5f) { GAIN = v; Serial.printf(">>> GAIN = %.2f\n", GAIN); }
    else Serial.println("!! GAIN 범위 0 ~ 1.5");
  } else if (u.startsWith("f")) {
    float v = s.substring(1).toFloat();
    if (v > 0.0f && v <= 1.0f) { CMD_LPF = v; Serial.printf(">>> CMD_LPF = %.2f\n", CMD_LPF); }
    else Serial.println("!! CMD_LPF 범위 0 ~ 1");
  } else if (u.startsWith("d")) {
    int v = s.substring(1).toInt();
    if (v >= 1 && v <= 20) { CAN_DIV = v; Serial.printf(">>> CAN_DIV = %d (%dHz)\n", v, 100 / v); }
    else Serial.println("!! CAN_DIV 범위 1 ~ 20");
  } else if (u == "q") {
    stream = !stream;
    Serial.printf(">>> 스트림 출력 %s\n", stream ? "ON" : "OFF");
  } else if (u == "?") {
    printStatus();
  } else {
    Serial.println("명령: arm / stop / z / t / q / g / kp / kd / f / d / ?");
  }
}


// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // ── I2C ──
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire.setTimeOut(50);            // 응답 없을 때 멈추지 않게
  delay(100);

  uint8_t id = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I: 0x%02X (기대값 0x%02X)  주소 0x%02X\n",
                id, WHO_AM_I_VAL, IMU_ADDR);
  imu_ok = (id == WHO_AM_I_VAL);
  if (!imu_ok) {
    Serial.println("!! IMU 응답 없음 — CS→3V3, SDO→GND 인지, SDA/SCL 풀업 확인");
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
  Serial.printf("gyro bias x:%.1f y:%.1f   g_scale:%.4f   I2C오류 %lu회\n",
                gyro_bias_x, gyro_bias_y, g_scale, i2c_err);

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
  broadcastUniversal(0xFD);        // 부팅 시 비활성

  lastLoop = micros();
  Serial.println("\n==================================================");
  Serial.println(" 2단계 — 합력 벡터 목표각");
  Serial.println(" IMU 는 I2C, 모터는 MIT 모드입니다.");
  Serial.println(" 모터는 비활성 상태입니다. 'arm' 을 입력해야 움직입니다.");
  Serial.println("--------------------------------------------------");
  Serial.println("  arm     활성화        stop  비활성화");
  Serial.println("  t       시험 구동     z     영점 재설정");
  Serial.println("  g<값>   수평 게인     예) g1.0");
  Serial.printf ("  ag<값>  합력 게인     예) ag0.3   현재 %.2f%s\n",
                 ACC_GAIN, ACC_GAIN == 0.0f ? "  ← 0 이면 1단계와 동일" : "");
  Serial.println("  ad<값>  합력 필터     예) ad0.2");
  Serial.println("  kp<값>  모터 강성     kd<값>  모터 감쇠");
  Serial.println("  f<값>   지령 필터     d<값>   CAN 분주     ?  상태");
  Serial.println("==================================================\n");
}


// =============================================================================
//  loop
// =============================================================================
void loop() {
  handleSerial();

  long behind = (long)(micros() - lastLoop);
  if (behind < 10000) return;                    // 100Hz
  if (behind > 100000) lastLoop = micros();      // 크게 밀리면 재동기
  else                 lastLoop += 10000;

  // ── 1. 자세 추정 ──
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
  if (trust <= 0.0f) { err_win++; rej_run++; } else rej_run = 0;

  // ── 2. 이상 감지 ──
  if (a_mag < ACC_MAG_DEAD && readReg(REG_CTRL1_XL) != CTRL1_XL_VAL) {
    err_reset++;
    Serial.println("!! 센서 응답 없음 — 재설정");
    configSensor();
    motorsDisarm("센서 리셋");
  }
  if (rej_run > REJ_RUN_FAULT) {
    imu_ok = false;
    motorsDisarm("IMU 무보정 구간 과다");
  }

  drainCAN();

  // ── 3. 합력 벡터 목표각 ──
  /* 측정 가속도에서 중력 성분을 뺀다. 가속도계는 기울어져 있기만 해도
   * 중력을 읽으므로, θ_base 로 그 몫을 계산해 제거해야 순수 선형 가속이 남는다.
   * 1g 에 해당하는 LSB 는 G_LSB × g_scale (부팅 시 정규화한 값).
   *
   * ⚠ 두 축의 부호가 다르다. 각도 정의가 다르기 때문이다.
   *
   *     pitch_acc = atan2( ay, ...)   →   ay = +g·sin(pitch)
   *     roll_acc  = atan2(-ax, ...)   →   ax = −g·sin(roll)     ★ 음수
   *
   * roll 은 중력 성분이 이미 음수라 빼는 게 아니라 더해야 상쇄된다.
   * 처음에 둘 다 빼도록 썼다가 roll 만 중력이 두 배로 남았다. 평평한데
   * roll 2.5° 에서 ref_roll 이 −5° 로, 조금만 기울이면 상한에 포화됐다.
   */
  float one_g   = G_LSB * g_scale;
  float a_lin_y = (float)ay - one_g * sinf(pitch_filtered * DEG_TO_RAD);
  float a_lin_x = (float)ax + one_g * sinf(roll_filtered  * DEG_TO_RAD);

  // 합력이 수직에서 벗어난 각도.  θ = atan(a_linear / g)
  float raw_ref_p = atan2f(a_lin_y, one_g) * RAD_TO_DEG * DIR_ACC;
  float raw_ref_r = atan2f(a_lin_x, one_g) * RAD_TO_DEG * DIR_ACC;

  raw_ref_p = constrain(raw_ref_p, -ACC_REF_LIMIT, ACC_REF_LIMIT);
  raw_ref_r = constrain(raw_ref_r, -ACC_REF_LIMIT, ACC_REF_LIMIT);

  // 선형 가속 추정은 노이즈가 크므로 목표각을 따로 저역통과
  ref_pitch = ACC_LPF * raw_ref_p + (1.0f - ACC_LPF) * ref_pitch;
  ref_roll  = ACC_LPF * raw_ref_r + (1.0f - ACC_LPF) * ref_roll;

  // ── 4. 제어  motor_cmd = θ_ref − θ_base ──
  /* ACC_GAIN 이 0 이면 θ_ref 항이 사라져 1단계(−θ_base)와 완전히 같아진다.
   * 모터는 차체 기준으로 돌므로 절대 목표각에서 차체 자세를 뺀다.
   */
  float want_pitch = (ACC_GAIN * ref_pitch - pitch_filtered * GAIN) * DIR_PITCH;
  float want_roll  = (ACC_GAIN * ref_roll  - roll_filtered  * GAIN) * DIR_ROLL;

  want_pitch = constrain(want_pitch, -LIMIT_DEG, LIMIT_DEG);
  want_roll  = constrain(want_roll,  -LIMIT_DEG, LIMIT_DEG);

  static float lpf_pitch = 0, lpf_roll = 0;
  lpf_pitch = CMD_LPF * want_pitch + (1.0f - CMD_LPF) * lpf_pitch;
  lpf_roll  = CMD_LPF * want_roll  + (1.0f - CMD_LPF) * lpf_roll;

  const float maxStep = MAX_RATE * dt;
  cmd_pitch = slew(lpf_pitch, cmd_pitch, maxStep);
  cmd_roll  = slew(lpf_roll,  cmd_roll,  maxStep);

  static int div_cnt = 0;
  if (armed && (++div_cnt >= CAN_DIV)) {
    div_cnt = 0;
    sendBoth();
  }

  // ── 5. 출력 ──
  if (stream && millis() - lastPrint > 20) {      // 50Hz
    lastPrint = millis();
    Serial.printf("pitch:%.2f,ref_pitch:%.2f,cmd_pitch:%.2f,act_pitch:%.2f,"
                  "roll:%.2f,ref_roll:%.2f,cmd_roll:%.2f,act_roll:%.2f\n",
                  pitch_filtered, ref_pitch, cmd_pitch, act_deg[CAN_ID_PITCH & 0x03],
                  roll_filtered,  ref_roll,  cmd_roll,  act_deg[CAN_ID_ROLL  & 0x03]);
  }

  if (millis() - lastCheck > 2000) {
    lastCheck = millis();
    uint8_t c1 = readReg(REG_CTRL1_XL);
    if (c1 != CTRL1_XL_VAL) {
      err_reset++;
      Serial.printf("!! 센서 리셋 CTRL1_XL=0x%02X — 재설정\n", c1);
      configSensor();
      motorsDisarm("센서 리셋");
    }
    Serial.printf("[stat] %s Kp=%.1f Kd=%.2f 모터P=%s R=%s 피드백:%lu | 무보정:%lu/%lu I2C오류:%lu\n",
                  armed ? "ARMED" : "STOP", MOTOR_KP, MOTOR_KD,
                  statusName(last_status[CAN_ID_PITCH & 0x03]),
                  statusName(last_status[CAN_ID_ROLL  & 0x03]),
                  fb_count, err_win, n_win, i2c_err);
    n_win = 0;  err_win = 0;
  }
}

/* -----------------------------------------------------------------------------
 * 부호 확인 절차 — ACC_GAIN 을 올리기 전에 반드시
 *
 * 합력 목표각의 부호가 반대면 가속할 때 액체를 **더** 흔드는 쪽으로 기울인다.
 * 수평제어는 틀려도 트레이가 반대로 기우는 것으로 금방 보이지만, 이건
 * 가속하는 순간에만 나타나므로 눈으로 알아채기 어렵다. 숫자로 확인한다.
 *
 * ── 1) 모터 없이 부호만 본다 ────────────────────────────────────────────
 *
 * arm 하지 않은 상태로 IMU 를 책상 위에서 한 방향으로 쭉 민다.
 * 출력의 ref_pitch / ref_roll 을 본다. ACC_GAIN 이 0 이어도 계산되어 찍힌다.
 *
 *   pitch:0.10,ref_pitch:-4.20,cmd_pitch:0.00,...
 *                    ↑ 여기
 *
 * 판정 기준 — 액체가 쏠리는 반대쪽으로 기울여야 한다.
 *
 *   보드의 +Y 축 방향으로 밀었을 때
 *     ref_pitch 가 음수  →  +Y 쪽이 내려감  →  맞다   (DIR_ACC = +1 유지)
 *     ref_pitch 가 양수  →  틀리다. DIR_ACC 를 −1 로
 *
 * 왜 음수여야 하는가. 컵을 +Y 로 가속하면 물은 −Y 쪽으로 쏠린다. 수면을
 * 컵 기준으로 평평하게 두려면 컵의 +Y 쪽을 내려야 한다. 우리 규약에서
 * +pitch 는 +Y 가 올라가는 것이므로 목표는 음수가 된다.
 *
 * roll 도 같은 방식으로 +X 방향으로 밀어 확인한다.
 * (+roll 은 +X 가 내려가는 것이므로 부호 방향이 pitch 와 반대다)
 *
 * ── 2) 정지 상태에서 0 근처인지 ────────────────────────────────────────
 *
 * 책상에 가만히 두면 ref 가 0 근처여야 한다. 계속 치우쳐 있으면 중력 성분을
 * 덜 뺀 것이므로 g_scale 과 θ_base 를 먼저 의심한다.
 *
 * 기울여 놓고 가만히 두었을 때도 0 근처여야 한다. 기울기만으로 ref 가
 * 생긴다면 중력 제거가 잘못된 것이다. 이 경우 ACC_GAIN 을 올리면 안 된다.
 *
 * ── 3) 게인을 조금씩 ───────────────────────────────────────────────────
 *
 *   ag0.3   →  ag0.6  →  ag1.0
 *
 * 각 단계에서 IMU 를 밀었다 세웠다 하며 트레이가 가속 반대쪽으로 기우는지 본다.
 * 소리·진동이 커지면 한 단계 내린다.
 *
 * ── 되돌리기 ───────────────────────────────────────────────────────────
 *
 *   ag0     합력 항이 사라져 1단계(수평 유지)와 완전히 같아진다.
 *
 * 무슨 일이 생기면 stop 보다 ag0 이 먼저다. 수평제어는 계속 살아 있다.
 *
 * -----------------------------------------------------------------------------
 * 아직 없는 것 — 3단계에서
 *
 * ZV 입력성형이 없다. 지금은 θ_ref 가 바뀌는 즉시 그대로 따라가므로, 목표각의
 * 급변 자체가 슬로싱을 새로 만들 수 있다. 3단계에서 θ_ref 에 Convolved ZV 를
 * 걸어 그 성분을 없앤다.
 *
 * 그리고 지연이 150~180ms 로 측정돼 있다. 슬로싱 주기 420ms 의 36% 라
 * 위상 여유가 부족하다. CMD_LPF·슬루·Kp 를 손봐야 할 수 있다.
 * ---------------------------------------------------------------------------*/
