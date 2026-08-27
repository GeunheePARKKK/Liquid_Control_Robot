/*
 * 모터_수평제어_MIT.ino
 * -----------------------------------------------------------------------------
 * 1단계 수평제어 — MIT 모드판 (ZV / 합력벡터 없음)
 *
 * [모터_수평제어.ino](모터_수평제어.ino) 와 제어식은 같다.
 *
 *   motor_cmd = −θ_base
 *
 * 다른 점은 모터에 보내는 방식뿐이다. 위치/속도 모드 대신 MIT 모드를 쓴다.
 *
 * 왜 MIT 모드인가
 *   위치/속도 모드에서 바깥축 모터가 1.75Hz 로 약 38° 왕복하며 멈추지 못했다.
 *   지령은 0.09° 이내로 고정이었고 두 드라이버의 게인 설정도 동일했으므로,
 *   원인은 바깥축의 큰 관성이다. 같은 게인이 무거운 축에서 감쇠 부족이 된 것.
 *
 *   위치/속도 모드의 내부 구조는 Pdes → PI → Vdes → PI → iqref 인 캐스케이드로
 *   **감쇠(D)항이 없다.** 게인도 드라이버 플래시에 있어 코드로 못 바꾼다.
 *
 *   MIT 모드는 매 프레임에 Kp 와 Kd 를 실어 보내며 아래 임피던스 제어를 한다.
 *
 *       τ = Kp·(p_des − p) + Kd·(v_des − v) + t_ff
 *
 *   v_des = 0 으로 두면 Kd 가 "움직이면 반대로 제동"하는 점성 감쇠가 되어
 *   오버슛을 직접 잡는다. 적분항이 없어 예전 서보 구조에서 I 를 쓰면 발산하던
 *   문제도 없다.
 *
 *   매뉴얼 42p 경고: "When controlling position, kd cannot be assigned 0,
 *   otherwise it will cause the motor to oscillate and even go out of control."
 *
 * ⚠ 드라이버 사전 설정 필요
 *   CubeMars 툴에서 두 모터 모두 ControlMode 를 MIT 로 바꿔야 한다.
 *   위치/속도 모드인 상태로 이 스케치를 돌리면 아무 반응이 없다.
 *   되돌리는 절차는 모터_수평제어_MIT.ino_변경이력.md 참고.
 *
 * 하드웨어
 *   MCU  : ESP32-S3
 *   IMU  : LSM6DSOX, SPI 1MHz MODE0, CS = GPIO10
 *   모터 : GL60II ×2, CAN 1Mbit/s, TX=GPIO4 / RX=GPIO5
 *
 * 시리얼 명령 (115200)
 *   arm       모터 활성화 (트레이 중립 / 차체 수평에서)
 *   stop      비활성화
 *   z         현재 위치를 모터 영점으로
 *   g<값>     제어 게인       예) g0.5
 *   kp<값>    모터 강성       예) kp30    [N/rad]   0~500
 *   kd<값>    모터 감쇠       예) kd1.5   [N·s/rad] 0~5   ← 발진 잡는 값
 *   f<값>     지령 필터       예) f0.1
 *   d<값>     CAN 분주        예) d2  (2=50Hz)
 *   ?         상태 출력
 * -----------------------------------------------------------------------------
 */

#include <SPI.h>
#include "driver/twai.h"

// =============================================================================
//  설정
// =============================================================================

// ── IMU 핀 ──
#define PIN_MOSI  11    // 보드 SDA
#define PIN_SCK   12    // 보드 SCL
#define PIN_MISO  13    // 보드 SDO
#define CS_BASE   10    // 보드 CS — 베이스(차체) IMU

#define SPI_SPEED 1000000
#define SPI_MD    SPI_MODE0

// ── CAN ──
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

/* 제어모드 접두.  CAN ID = (모드 << 8) | 모터ID
 *   0x000 = MIT,  0x100 = 위치/속도,  0x200 = 속도
 * 위치/속도 모드로 되돌릴 때는 이 값만 0x100 으로 바꾸고
 * sendMIT() 대신 sendPosVel() 을 쓰면 된다.
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
float CMD_LPF         = 0.15f;   // 지령 저역통과 (손 떨림 억제)
int   CAN_DIV         = 2;       // 100Hz / 2 = 50Hz 전송

/* 모터 임피던스 게인 — MIT 모드의 핵심.
 *   MOTOR_KP : 강성. 클수록 목표를 세게 따라가지만 오버슛도 커진다.
 *   MOTOR_KD : 감쇠. 발진을 잡는 값. 0 이면 매뉴얼 경고대로 폭주한다.
 * 임계감쇠 근처는  Kd ≈ 2·√(Kp·J).  낮게 시작해 올려가며 찾는다.
 */
float MOTOR_KP = 20.0f;
float MOTOR_KD = 1.0f;

// ── MIT 모드 변환 범위 — 드라이버 설정값과 반드시 일치해야 한다 ──
const float P_MIN  = -12.5f, P_MAX  =  12.5f;   // rad      (PMAX 12.5)
const float V_MIN  = -30.0f, V_MAX  =  30.0f;   // rad/s    (VMAX 30 — 매뉴얼 기본 200 아님)
const float KP_MIN =   0.0f, KP_MAX = 500.0f;   // N/rad
const float KD_MIN =   0.0f, KD_MAX =   5.0f;   // N·s/rad
const float T_MIN  = -10.0f, T_MAX  =  10.0f;   // N·m      (TMAX 10)

// ── LSM6DSOX ──
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

/* 피드백 data[0] 상위니블은 상태코드 (오류코드가 아님).
 * 0=Disable, 1=Enable 이 정상이고 8 이상이 이상.
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

float cmd_pitch = 0, cmd_roll = 0;      // 내보낸 지령 [deg]

bool  armed  = false;
bool  imu_ok = false;
uint16_t rej_run = 0;

uint8_t  last_status[4] = {0xFF, 0xFF, 0xFF, 0xFF};
float    act_deg[4]     = {0, 0, 0, 0};   // 모터 실제 위치 [deg]
float    act_vel[4]     = {0, 0, 0, 0};   // 모터 실제 속도 [rad/s]
uint32_t fb_count = 0;

unsigned long lastLoop = 0, lastPrint = 0, lastCheck = 0;
uint32_t n_win = 0, err_win = 0, err_reset = 0;


// =============================================================================
//  IMU — SPI
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
  writeReg(REG_CTRL3_C, 0x01);
  delay(50);
  writeReg(REG_CTRL3_C,  CTRL3_C_VAL);
  writeReg(REG_CTRL1_XL, CTRL1_XL_VAL);
  writeReg(REG_CTRL2_G,  CTRL2_G_VAL);
  delay(100);
}


// =============================================================================
//  MIT 모드 CAN
// =============================================================================
/* 매뉴얼 43p 의 float_to_uint 를 그대로 옮긴 것.
 * 매뉴얼 식은 (1<<bits)/span 이라 상한에서 1 만큼 넘칠 수 있어 클램프를 더했다.
 */
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
  uint16_t p  = float_to_uint(pos, P_MIN,  P_MAX,  16);
  uint16_t v  = float_to_uint(vel, V_MIN,  V_MAX,  12);
  uint16_t kpi= float_to_uint(kp,  KP_MIN, KP_MAX, 12);
  uint16_t kdi= float_to_uint(kd,  KD_MIN, KD_MAX, 12);
  uint16_t t  = float_to_uint(tff, T_MIN,  T_MAX,  12);

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

// 현재 지령을 두 모터에 전송
void sendBoth() {
  sendMIT(CAN_ID_PITCH, cmd_pitch * DEG_TO_RAD, 0.0f, MOTOR_KP, MOTOR_KD, 0.0f);
  sendMIT(CAN_ID_ROLL,  cmd_roll  * DEG_TO_RAD, 0.0f, MOTOR_KP, MOTOR_KD, 0.0f);
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

  // enable 전에 목표를 현재 위치(0)로 박아둔다. 빼면 마지막 목표로 확 튄다.
  cmd_pitch = 0;  cmd_roll = 0;
  sendBoth();
  delay(50);

  broadcastUniversal(0xFC);  delay(100);  // enable
  armed = true;
  Serial.printf(">>> ARMED  GAIN=%.2f  Kp=%.1f  Kd=%.2f  제한 ±%.0f°\n",
                GAIN, MOTOR_KP, MOTOR_KD, LIMIT_DEG);
}

void motorsDisarm(const char *reason) {
  if (armed) Serial.printf(">>> DISARM — %s\n", reason);
  broadcastUniversal(0xFD);
  armed = false;
  cmd_pitch = 0;  cmd_roll = 0;
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
  Serial.printf("       pitch=%.2f roll=%.2f  cmd(p/r)=%.2f/%.2f  act(p/r)=%.2f/%.2f\n",
                pitch_filtered, roll_filtered, cmd_pitch, cmd_roll,
                act_deg[CAN_ID_PITCH & 0x03], act_deg[CAN_ID_ROLL & 0x03]);
  Serial.printf("       피드백 %lu회  P(id%u)=%s  R(id%u)=%s\n", fb_count,
                CAN_ID_PITCH, statusName(last_status[CAN_ID_PITCH & 0x03]),
                CAN_ID_ROLL,  statusName(last_status[CAN_ID_ROLL  & 0x03]));
  if (fb_count == 0) {
    Serial.println("       !! 피드백 0회 — ControlMode 가 MIT 인지, CAN 배선을 확인");
  }
}

// 수신 프레임 파싱 (MIT 피드백 포맷)
//   D0 ID|ERR<<4  D1~2 POS  D3 VEL[11:4]  D4 VEL[3:0]|T[11:8]  D5 T[7:0]
//   D6 T_MOS      D7 T_Rotor
void drainCAN() {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    uint8_t mid = rx.data[0] & 0x0F;
    uint8_t st  = rx.data[0] >> 4;
    uint8_t i   = mid & 0x03;

    uint16_t praw = ((uint16_t)rx.data[1] << 8) | rx.data[2];
    uint16_t vraw = ((uint16_t)rx.data[3] << 4) | (rx.data[4] >> 4);
    act_deg[i] = uint_to_float(praw, P_MIN, P_MAX, 16) * RAD_TO_DEG;
    act_vel[i] = uint_to_float(vraw, V_MIN, V_MAX, 12);

    last_status[i] = st;
    fb_count++;

    if (st >= ST_FAULT_MIN && armed) {
      Serial.printf("!! 모터 %u 이상 — %s\n", mid, statusName(st));
      motorsDisarm("모터 이상");
    }
  }
}

/* CAN 경로만 확인하는 시험 구동. IMU·제어식을 거치지 않는다. */
void motorSweep() {
  Serial.println(">>> 시험 구동 6초 (±0.5 rad). 손을 치우세요.");
  broadcastUniversal(0xFB);  delay(50);
  broadcastUniversal(0xFE);  delay(50);
  sendMIT(CAN_ID_PITCH, 0, 0, MOTOR_KP, MOTOR_KD, 0);
  sendMIT(CAN_ID_ROLL,  0, 0, MOTOR_KP, MOTOR_KD, 0);
  delay(50);
  broadcastUniversal(0xFC);  delay(100);

  bool dir = true;
  uint32_t t0 = millis(), last = 0;
  while (millis() - t0 < 6000) {
    if (millis() - last >= 500) { last = millis(); dir = !dir; }
    float p = dir ? 0.5f : -0.5f;
    sendMIT(CAN_ID_PITCH, p, 0, MOTOR_KP, MOTOR_KD, 0);
    sendMIT(CAN_ID_ROLL,  p, 0, MOTOR_KP, MOTOR_KD, 0);
    drainCAN();
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
  } else if (u.startsWith("kp")) {          // kd 보다 먼저 검사할 것
    float v = s.substring(2).toFloat();
    if (v >= KP_MIN && v <= KP_MAX) {
      MOTOR_KP = v;
      Serial.printf(">>> Kp = %.1f N/rad\n", MOTOR_KP);
    } else Serial.printf("!! Kp 범위 %.0f ~ %.0f\n", KP_MIN, KP_MAX);
  } else if (u.startsWith("kd")) {
    float v = s.substring(2).toFloat();
    if (v >= KD_MIN && v <= KD_MAX) {
      MOTOR_KD = v;
      Serial.printf(">>> Kd = %.2f N·s/rad\n", MOTOR_KD);
      if (v == 0.0f) Serial.println("   !! Kd=0 은 매뉴얼상 발진·폭주 조건입니다");
    } else Serial.printf("!! Kd 범위 %.0f ~ %.0f\n", KD_MIN, KD_MAX);
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
  } else if (u == "?") {
    printStatus();
  } else {
    Serial.println("명령: arm / stop / z / t / g / kp / kd / f / d / ?");
  }
}


// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(CS_BASE, OUTPUT);
  digitalWrite(CS_BASE, HIGH);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  delay(100);

  uint8_t id = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I: 0x%02X (기대값 0x%02X)\n", id, WHO_AM_I_VAL);
  imu_ok = (id == WHO_AM_I_VAL);
  if (!imu_ok) Serial.println("!! IMU 응답 없음 — MISO/CS/VIN/GND 확인");
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
  Serial.println(" MIT 모드.  드라이버 ControlMode 가 MIT 여야 동작합니다.");
  Serial.println(" 모터는 비활성 상태입니다. 'arm' 을 입력해야 움직입니다.");
  Serial.println("--------------------------------------------------");
  Serial.println("  arm     활성화        stop  비활성화");
  Serial.println("  t       시험 구동     z     영점 재설정");
  Serial.println("  g<값>   제어 게인     예) g0.5");
  Serial.println("  kp<값>  모터 강성     예) kp30   [0~500]");
  Serial.println("  kd<값>  모터 감쇠     예) kd1.5  [0~5]  ← 발진 잡는 값");
  Serial.println("  f<값>   지령 필터     d<값>  CAN 분주     ?  상태");
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
    Serial.println("!! 센서 파워다운 감지 — 즉시 재설정");
    configSensor();
    motorsDisarm("센서 리셋");
  }
  if (rej_run > REJ_RUN_FAULT) {
    imu_ok = false;
    motorsDisarm("IMU 무보정 구간 과다");
  }

  drainCAN();

  // ── 3. 제어  motor_cmd = −θ_base ──
  float want_pitch = -pitch_filtered * GAIN * DIR_PITCH;
  float want_roll  = -roll_filtered  * GAIN * DIR_ROLL;

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

  // ── 4. 출력 ──
  if (millis() - lastPrint > 20) {          // 50Hz — 진동 관찰용
    lastPrint = millis();
    Serial.printf("pitch:%.2f,cmd_pitch:%.2f,act_pitch:%.2f,"
                  "roll:%.2f,cmd_roll:%.2f,act_roll:%.2f\n",
                  pitch_filtered, cmd_pitch, act_deg[CAN_ID_PITCH & 0x03],
                  roll_filtered,  cmd_roll,  act_deg[CAN_ID_ROLL  & 0x03]);
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
    Serial.printf("[stat] %s Kp=%.1f Kd=%.2f 모터P=%s R=%s 피드백:%lu | 무보정:%lu/%lu\n",
                  armed ? "ARMED" : "STOP", MOTOR_KP, MOTOR_KD,
                  statusName(last_status[CAN_ID_PITCH & 0x03]),
                  statusName(last_status[CAN_ID_ROLL  & 0x03]),
                  fb_count, err_win, n_win);
    n_win = 0;  err_win = 0;
  }
}

/* -----------------------------------------------------------------------------
 * 튜닝 순서
 *
 *  1) 드라이버가 MIT 모드인지 먼저 확인
 *     t 를 쳐서 두 모터가 왕복하면 통신 정상. 반응 없으면 ControlMode 확인.
 *
 *  2) arm 후 Kd 를 올려가며 발진을 잡는다  ← 이번 문제의 핵심
 *     kd0.5 → kd1.0 → kd2.0 ...
 *     떨림이 멎는 최솟값을 찾는다. 너무 키우면 끈적하게 느려진다.
 *
 *  3) Kp 로 강성을 맞춘다
 *     목표에 못 미치면(처짐) kp 를 올린다. 올리면 Kd 도 같이 올려야 한다.
 *     임계감쇠 근처가  Kd ≈ 2·√(Kp·J).
 *
 *  4) GAIN 을 1.0 까지 올린다
 *     g0.5 → g0.7 → g1.0. 이때 비로소 기운 만큼 전부 보상한다.
 *
 *  두 축의 관성이 다르므로 최종적으로는 축별로 Kp/Kd 를 나눠야 할 수 있다.
 *  지금은 두 축에 같은 값을 보낸다.
 *
 * 위치/속도 모드로 되돌리려면
 *   모터_수평제어_MIT.ino_변경이력.md 의 "되돌리는 방법" 참고.
 * ---------------------------------------------------------------------------*/
