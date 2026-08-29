/*
 * motor_test_MIT.ino
 * -----------------------------------------------------------------------------
 * MIT 모드 짐벌모터 2축 연속 왕복 테스트.
 *
 * [motor_test.ino](motor_test.ino) 의 MIT 판. 그쪽은 위치/속도 모드용이라
 * 드라이버 ControlMode 가 MIT 인 지금은 동작하지 않는다.
 *
 * run 을 치면 stop 을 칠 때까지 계속 왕복한다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ⚠ MIT 모드에는 속도 제한이 없다 — 계단 명령 금지
 *
 *   위치/속도 모드의 v_des 는 "이 속도를 넘지 마라" 였지만 MIT 의 v_des 는
 *   감쇠항의 기준 속도일 뿐이다. 목표를 한 번에 크게 주면 그 오차에 Kp 를
 *   곱한 토크가 그 자리에서 전부 나간다.
 *
 *       τ = Kp × 오차          (TMAX 10 N·m 에서 포화)
 *
 *   motor_test.ino 의 ±0.5 rad 계단을 그대로 옮겼다가 Kp 20 에서 10 N·m,
 *   즉 최대 토크가 걸려 기구가 튕겼다.
 *
 *   그래서 이 스케치는 목표를 매 주기 rate×dt 만큼만 움직인다. 진폭이나
 *   속도를 바꿔도 계단이 될 수 없는 구조다.
 *
 * ⚠ 드라이버 사전 설정
 *   두 모터 모두 CubeMars 에서 ControlMode = MIT 여야 한다.
 *   Master ID 도 MIT 명령 주소(0x001 / 0x002)와 겹치면 안 된다.
 *     바깥축 0x01 → Master 0x00,  안쪽축 0x02 → Master 0x11
 *   겹치면 한쪽 모터가 다른 쪽 피드백을 명령으로 읽어 폭주한다.
 *   자세한 내용은 docs/CONTROL_INTEGRATION_TASK/모터_수평제어_MIT.ino_변경이력.md
 *
 * 하드웨어
 *   ESP32-S3 / CAN 1Mbit/s / TX=GPIO4, RX=GPIO5 / 24V, 3A 이상
 *
 * 시리얼 명령 (115200)
 *   run       왕복 시작 (stop 칠 때까지 계속)
 *   stop      0° 로 되돌린 뒤 정지
 *   z         현재 위치를 영점으로
 *   a<값>     진폭 [deg]      예) a12    (상한 30)
 *   r<값>     속도 [deg/s]    예) r60    (상한 150)
 *   kp<값>    강성 [N/rad]    예) kp2    (상한 5)
 *   kd<값>    감쇠 [N·s/rad]  예) kd0.1  (상한 1, 기본 0.05)
 *             올리면 부드럽고 시끄럽다. 내리면 조용하고 끊긴다.
 *   ?         상태 출력
 * -----------------------------------------------------------------------------
 */

#include "driver/twai.h"

#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

#define CAN_MODE       0x000    // MIT (위치/속도는 0x100)
#define CAN_ID_OUTER   0x01     // 바깥축
#define CAN_ID_INNER   0x02     // 안쪽축

// MIT 변환 범위 — 드라이버 설정과 일치해야 한다
const float P_MIN  = -12.5f, P_MAX  =  12.5f;   // rad
const float V_MIN  = -30.0f, V_MAX  =  30.0f;   // rad/s  (VMAX 30, 매뉴얼 기본 200 아님)
const float KP_MIN =   0.0f, KP_MAX = 500.0f;
const float KD_MIN =   0.0f, KD_MAX =   5.0f;
const float T_MIN  = -10.0f, T_MAX  =  10.0f;

// ── 시험 파라미터 (시리얼로 조정, 상한은 코드에 박음) ──
float SWEEP_DEG  = 8.0f;    float SWEEP_DEG_MAX  = 30.0f;
float SWEEP_RATE = 20.0f;   float SWEEP_RATE_MAX = 150.0f;
/* Kp / Kd 트레이드오프 — 부드러움 vs 소음
 *
 * 이 두 값이 서로 반대 방향으로 작용한다.
 *
 *   Kd = 0     조용하지만 뚝뚝 끊긴다 (스틱슬립).
 *              토크가 정지마찰을 넘을 때까지 오차가 쌓였다가 훅 미끄러지고,
 *              오차가 줄면 다시 멈춘다. 이게 반복된다.
 *
 *   Kd > 0     부드럽게 움직인다. 대신 드라이버가 엔코더 속도 추정값에
 *              이 값을 곱해 토크를 만드는데, 그 추정값이 정지 상태에서도
 *              양자화 때문에 떨려서 "위이잉" 소리가 난다.
 *
 * motor_test.ino(위치/속도 모드)가 조용하면서도 매끄러웠던 것은 드라이버
 * 내부 캐스케이드가 Position KP 54 에 적분항까지 갖고 훨씬 세게 밀기
 * 때문이다. MIT 모드에는 그 내부 루프가 없어 우리가 대신 잡아야 한다.
 *
 * 아래 기본값은 그 중간이다. 끊기면 kd 를 올리고, 시끄러우면 내린다.
 * 토크가 부족해 끊기는 것 같으면 kp 를 올려도 된다 (Kd 와 같이 올릴 것).
 * 잘 안 잡히면 진폭·속도(a / r)를 바꿔보는 것도 방법이다.
 */
float MOTOR_KP   = 1.0f;    float MOTOR_KP_MAX   = 5.0f;
float MOTOR_KD   = 0.05f;   float MOTOR_KD_MAX   = 1.0f;

// 실제 위치가 진폭보다 이만큼 더 벗어나면 폭주로 보고 정지
const float ACT_MARGIN_DEG = 25.0f;

// 피드백이 이만큼 끊기면 실제 위치를 모르므로 정지
const uint32_t FB_TIMEOUT_MS = 300;

// ── 상태 ──
bool  running  = false;
float target   = 0;         // 현재 목표 [deg] — 여기서만 조금씩 움직인다
int   dir      = +1;
uint8_t  last_status[4] = {0xFF, 0xFF, 0xFF, 0xFF};
float    act_deg[4]     = {0, 0, 0, 0};
uint32_t fb_count = 0;
unsigned long lastStep = 0, lastPrint = 0, lastFb = 0;

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
void sendMIT(uint8_t id, float pos_deg) {
  uint16_t p  = float_to_uint(pos_deg * DEG_TO_RAD, P_MIN, P_MAX, 16);
  uint16_t v  = float_to_uint(0.0f,     V_MIN,  V_MAX,  12);   // v_des = 0
  uint16_t kp = float_to_uint(MOTOR_KP, KP_MIN, KP_MAX, 12);
  uint16_t kd = float_to_uint(MOTOR_KD, KD_MIN, KD_MAX, 12);
  uint16_t t  = float_to_uint(0.0f,     T_MIN,  T_MAX,  12);   // t_ff = 0

  twai_message_t m = {};
  m.identifier = CAN_MODE | id;
  m.data_length_code = 8;
  m.data[0] = p >> 8;
  m.data[1] = p & 0xFF;
  m.data[2] = v >> 4;
  m.data[3] = ((v & 0x0F) << 4) | (kp >> 8);
  m.data[4] = kp & 0xFF;
  m.data[5] = kd >> 4;
  m.data[6] = ((kd & 0x0F) << 4) | (t >> 8);
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

void bothUniversal(uint8_t last) {
  sendUniversal(CAN_ID_OUTER, last);
  sendUniversal(CAN_ID_INNER, last);
}

void sendBoth(float pos_deg) {
  sendMIT(CAN_ID_OUTER, pos_deg);
  sendMIT(CAN_ID_INNER, pos_deg);
}

void testStop(const char *reason);   // 아래에서 정의

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

    if (st >= 0x8 && running) {
      Serial.printf("!! 모터 %u 이상 — %s\n", mid, statusName(st));
      testStop("모터 이상");
      return;
    }
  }

  if (!running) return;

  // 목표는 ±SWEEP_DEG 안에서만 움직인다. 그보다 크게 벗어나면 우리가 시킨 게 아니다.
  float lim = SWEEP_DEG + ACT_MARGIN_DEG;
  float ao = act_deg[CAN_ID_OUTER & 0x03];
  float ai = act_deg[CAN_ID_INNER & 0x03];
  if (fabsf(ao) > lim || fabsf(ai) > lim) {
    Serial.printf("!! 폭주 차단 — 실제 위치 바깥=%.1f° 안쪽=%.1f° (한계 ±%.0f°)\n", ao, ai, lim);
    testStop("위치 한계 초과");
    return;
  }

  if (lastFb != 0 && millis() - lastFb > FB_TIMEOUT_MS) {
    Serial.printf("!! 피드백 %lums 끊김\n", millis() - lastFb);
    testStop("피드백 두절");
  }
}

void testStart() {
  Serial.println(">>> 시작 준비 — 손을 치우세요");
  bothUniversal(0xFB);  delay(50);      // 에러 해제
  bothUniversal(0xFE);  delay(50);      // 현재 위치를 영점으로

  /* enable 전에 목표를 현재 위치(0)로 먼저 박아둔다.
   * 빼면 드라이버가 마지막으로 기억하던 목표로 확 달려간다.
   */
  target = 0;  dir = +1;
  for (int i = 0; i < 4; i++) act_deg[i] = 0;
  sendBoth(0);
  delay(50);

  bothUniversal(0xFC);  delay(100);     // enable
  lastFb = millis();
  lastStep = millis();
  running = true;
  Serial.printf(">>> RUN  진폭 ±%.0f°  속도 %.0f°/s  Kp %.1f  Kd %.2f\n",
                SWEEP_DEG, SWEEP_RATE, MOTOR_KP, MOTOR_KD);
  Serial.println("    멈추려면 stop");
}

void testStop(const char *reason) {
  if (!running) { bothUniversal(0xFD); return; }
  running = false;

  // 기울어진 채로 끄지 않도록 0 까지 되돌린 뒤 disable
  while (fabsf(target) > 0.5f) {
    target += (target > 0 ? -0.5f : 0.5f);
    sendBoth(target);
    delay(20);
  }
  sendBoth(0);
  delay(50);
  bothUniversal(0xFD);
  target = 0;
  Serial.printf(">>> STOP — %s  (피드백 %lu회)\n", reason, fb_count);
}

void printStatus() {
  Serial.printf("[상태] %s  진폭±%.0f°  속도%.0f°/s  Kp%.1f Kd%.2f\n",
                running ? "RUN " : "STOP", SWEEP_DEG, SWEEP_RATE, MOTOR_KP, MOTOR_KD);
  Serial.printf("       목표 %+.2f°   바깥(id%u) %s %+.2f°   안쪽(id%u) %s %+.2f°   피드백 %lu\n",
                target,
                CAN_ID_OUTER, statusName(last_status[CAN_ID_OUTER & 0x03]),
                act_deg[CAN_ID_OUTER & 0x03],
                CAN_ID_INNER, statusName(last_status[CAN_ID_INNER & 0x03]),
                act_deg[CAN_ID_INNER & 0x03], fb_count);
  if (fb_count == 0) {
    Serial.println("       !! 피드백 0회 — ControlMode 가 MIT 인지, CAN 배선 확인");
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;
  String u = s;  u.toLowerCase();

  if (u == "run") {
    if (running) Serial.println("이미 돌고 있습니다");
    else testStart();
  } else if (u == "stop") {
    testStop("사용자 명령");
  } else if (u == "z") {
    bothUniversal(0xFE);
    target = 0;
    for (int i = 0; i < 4; i++) act_deg[i] = 0;
    Serial.println(">>> 영점 재설정");
  } else if (u.startsWith("kp")) {              // kd 보다 먼저
    MOTOR_KP = constrain(s.substring(2).toFloat(), 0.0f, MOTOR_KP_MAX);
    Serial.printf(">>> Kp = %.2f  (상한 %.0f)  진폭에서 약 %.2f N·m\n",
                  MOTOR_KP, MOTOR_KP_MAX, MOTOR_KP * SWEEP_DEG * DEG_TO_RAD);
  } else if (u.startsWith("kd")) {
    MOTOR_KD = constrain(s.substring(2).toFloat(), 0.0f, MOTOR_KD_MAX);
    Serial.printf(">>> Kd = %.2f  (상한 %.1f)\n", MOTOR_KD, MOTOR_KD_MAX);
    if (MOTOR_KD == 0.0f) Serial.println("   (Kd 0 — 조용하지만 스틱슬립으로 끊긴다)");
  } else if (u.startsWith("a")) {
    SWEEP_DEG = constrain(s.substring(1).toFloat(), 1.0f, SWEEP_DEG_MAX);
    Serial.printf(">>> 진폭 = ±%.0f°  (상한 %.0f)\n", SWEEP_DEG, SWEEP_DEG_MAX);
  } else if (u.startsWith("r")) {
    SWEEP_RATE = constrain(s.substring(1).toFloat(), 1.0f, SWEEP_RATE_MAX);
    Serial.printf(">>> 속도 = %.0f°/s  (상한 %.0f)\n", SWEEP_RATE, SWEEP_RATE_MAX);
  } else if (u == "?") {
    printStatus();
  } else {
    Serial.println("명령: run / stop / z / a<값> / r<값> / kp<값> / kd<값> / ?");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
  delay(100);
  bothUniversal(0xFD);                  // 부팅 시 비활성

  Serial.println("\n==================================================");
  Serial.println(" MIT 모드 모터 왕복 테스트");
  Serial.println(" 드라이버 ControlMode 가 MIT 여야 동작합니다.");
  Serial.println(" 모터는 비활성 상태입니다. 'run' 을 입력해야 움직입니다.");
  Serial.println("--------------------------------------------------");
  Serial.println("  run     왕복 시작 (stop 까지 계속)");
  Serial.println("  stop    0° 로 되돌린 뒤 정지");
  Serial.println("  z       영점 재설정");
  Serial.println("  ?       상태 출력");
  Serial.println("--------------------------------------------------");
  Serial.println("  값 조절            현재값    범위       예");
  Serial.printf ("  a<값>  진폭 [deg]  %6.1f    1 ~ %-3.0f    a15\n",  SWEEP_DEG,  SWEEP_DEG_MAX);
  Serial.printf ("  r<값>  속도 [deg/s]%6.1f    1 ~ %-3.0f    r60\n",  SWEEP_RATE, SWEEP_RATE_MAX);
  Serial.printf ("  kp<값> 강성        %6.2f    0 ~ %-3.0f    kp2\n",  MOTOR_KP,   MOTOR_KP_MAX);
  Serial.printf ("  kd<값> 감쇠        %6.2f    0 ~ %-3.0f    kd0.1\n", MOTOR_KD,   MOTOR_KD_MAX);
  Serial.println("--------------------------------------------------");
  Serial.printf ("  지금 설정이면 한 번 왕복에 %.1f 초\n", 4.0f * SWEEP_DEG / SWEEP_RATE);
  Serial.println("  빠르게: r60    크게: a15    부드럽게(시끄러움): kd0.1");
  Serial.println("==================================================\n");
}

void loop() {
  handleSerial();
  drainCAN();

  if (!running) return;

  // 50Hz 로 목표를 조금씩만 움직인다 — 여기가 계단을 막는 지점
  unsigned long now = millis();
  if (now - lastStep < 20) return;
  float step = SWEEP_RATE * (now - lastStep) * 0.001f;
  lastStep = now;

  target += dir * step;
  if (target >=  SWEEP_DEG) { target =  SWEEP_DEG; dir = -1; }
  if (target <= -SWEEP_DEG) { target = -SWEEP_DEG; dir = +1; }

  sendBoth(target);

  if (now - lastPrint >= 100) {         // 10Hz
    lastPrint = now;
    Serial.printf("target:%.2f,outer:%.2f,inner:%.2f\n",
                  target, act_deg[CAN_ID_OUTER & 0x03], act_deg[CAN_ID_INNER & 0x03]);
  }
}

/* -----------------------------------------------------------------------------
 * 사용 순서
 *
 *  1) 손을 치우고 run
 *     두 모터가 ±8° 를 20°/s 로 천천히 왕복해야 한다.
 *
 *  2) target 과 outer / inner 를 비교한다
 *     붙어서 따라오면 정상. 벌어지면 Kp 를 조금 올린다 (kp2, kp3).
 *
 *  3) 조건을 바꿔가며 본다
 *     a12 r30   더 크고 빠르게
 *     kd0.05    감쇠를 줄이면 왕복(오버슛)이 생기는지
 *
 *  4) stop
 *
 * 무슨 일이 생기면 stop 을 치면 0° 로 되돌린 뒤 멈춘다.
 * 위치가 한계를 넘거나 피드백이 끊기면 코드가 알아서 멈춘다.
 * ---------------------------------------------------------------------------*/
