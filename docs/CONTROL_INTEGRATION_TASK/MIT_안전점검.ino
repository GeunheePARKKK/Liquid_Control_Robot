/*
 * MIT_안전점검.ino
 * -----------------------------------------------------------------------------
 * MIT 모드 주소 충돌이 해소됐는지 확인하는 진단 전용 스케치.
 *
 * ★ 모터는 절대 힘을 쓰지 않는다 ★
 *
 *   보내는 값이  Kp = 0, Kd = 0, t_ff = 0  으로 고정이다.
 *
 *       τ = Kp·(p_des − p) + Kd·(v_des − v) + t_ff
 *         =  0·(...)      +  0·(...)      + 0
 *         =  0
 *
 *   enable 은 되지만 토크가 0 이므로 손으로 자유롭게 돌아간다.
 *   게인을 바꾸는 명령 자체를 넣지 않았다. 제어 루프도 없다.
 *
 * 왜 필요한가
 *   MIT 모드로 전환한 뒤 모터가 3바퀴 돌며 배선이 끊어졌다. 원인은 CAN 주소 충돌.
 *
 *     CAN ID = (제어모드 << 8) | 모터ID
 *       위치/속도 모드(1) : 명령이 0x101 / 0x102 → 피드백 0x000 / 0x001 과 안 겹침
 *       MIT 모드(0)       : 명령이 0x001 / 0x002 → 모터2 피드백(Master ID 0x01)과 충돌
 *
 *   모터1 이 모터2 의 피드백을 자기 MIT 명령으로 해석했다. 피드백 data[0] 은
 *   ID|ERR<<4 인데 MIT 에서는 p_des[15:8] 자리라, p_des 가 약 −12.3 rad
 *   (−700° 이상) 으로 읽히고 Kp 자리에도 쓰레기가 들어갔다.
 *
 *   조치: 모터2 의 Master ID 를 0x01 → 0x11 로 변경 (CubeMars).
 *   이 스케치는 그 조치가 실제로 먹었는지를 무동력 상태에서 확인한다.
 *
 * 확인할 것
 *   1. 두 모터의 피드백이 모두 들어오는가
 *   2. 피드백이 도착한 CAN ID 가 명령 주소(0x001 / 0x002)와 겹치지 않는가
 *      → 겹치면 화면에 경고가 뜬다
 *   3. 손으로 축을 돌렸을 때 해당 모터의 위치값만 따라 움직이는가
 *      → 어느 모터가 어느 물리 축을 담당하는지도 여기서 확정된다
 *
 * 하드웨어
 *   ESP32-S3 / CAN 1Mbit/s / TX=GPIO4, RX=GPIO5
 *   드라이버 두 대 모두 ControlMode = MIT 여야 한다.
 *
 * 시리얼 명령 (115200)
 *   e    enable  (토크는 계속 0 이다)
 *   d    disable
 *   z    현재 위치를 영점으로
 *   c    카운터 초기화
 * -----------------------------------------------------------------------------
 */

#include "driver/twai.h"

#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

#define CAN_MODE      0x000     // MIT
#define CAN_ID_A      0x01      // 바깥축 모터
#define CAN_ID_B      0x02      // 안쪽축 모터

// MIT 변환 범위 — 드라이버 설정과 일치해야 한다
const float P_MIN  = -12.5f, P_MAX  =  12.5f;
const float V_MIN  = -30.0f, V_MAX  =  30.0f;   // VMAX 30
const float KP_MIN =   0.0f, KP_MAX = 500.0f;
const float KD_MIN =   0.0f, KD_MAX =   5.0f;
const float T_MIN  = -10.0f, T_MAX  =  10.0f;

struct MotorInfo {
  uint32_t frames  = 0;       // 받은 피드백 수
  uint32_t rxid    = 0xFFFF;  // 마지막으로 도착한 CAN ID
  uint8_t  status  = 0xFF;
  float    pos_deg = 0;
  float    vel     = 0;
};
MotorInfo mot[16];

uint32_t collide = 0;         // 명령 주소로 들어온 프레임 수 (0 이어야 정상)
bool     enabled = false;
unsigned long lastSend = 0, lastPrint = 0;

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

/* 무동력 MIT 프레임.
 * Kp / Kd / t_ff 는 인자로 받지 않는다 — 실수로라도 토크가 나갈 수 없게.
 * p_des 와 v_des 도 0 이지만, Kp=Kd=0 이므로 어떤 값이든 토크는 0 이다.
 */
void sendLimp(uint8_t id) {
  uint16_t p  = float_to_uint(0.0f, P_MIN,  P_MAX,  16);
  uint16_t v  = float_to_uint(0.0f, V_MIN,  V_MAX,  12);
  uint16_t kp = float_to_uint(0.0f, KP_MIN, KP_MAX, 12);   // 0
  uint16_t kd = float_to_uint(0.0f, KD_MIN, KD_MAX, 12);   // 0
  uint16_t t  = float_to_uint(0.0f, T_MIN,  T_MAX,  12);   // 0 N·m

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

void sendUniversal(uint8_t id, uint8_t last) {
  twai_message_t m = {};
  m.identifier = CAN_MODE | id;
  m.data_length_code = 8;
  memset(m.data, 0xFF, 7);
  m.data[7] = last;
  twai_transmit(&m, pdMS_TO_TICKS(5));
}

void bothUniversal(uint8_t last) {
  sendUniversal(CAN_ID_A, last);
  sendUniversal(CAN_ID_B, last);
}

void drainCAN() {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {

    /* 명령 주소로 프레임이 들어오면 아직 충돌 상태다.
     * 우리가 보낸 프레임은 루프백되지 않으므로, 여기 잡히면 남의 피드백이다.
     */
    if (rx.identifier == (CAN_MODE | CAN_ID_A) ||
        rx.identifier == (CAN_MODE | CAN_ID_B)) {
      collide++;
      if (collide <= 5) {
        Serial.printf("!! 충돌: 명령 주소 0x%03X 로 프레임 수신 — Master ID 확인 필요\n",
                      (unsigned)rx.identifier);
      }
    }

    uint8_t mid = rx.data[0] & 0x0F;
    uint16_t praw = ((uint16_t)rx.data[1] << 8) | rx.data[2];
    uint16_t vraw = ((uint16_t)rx.data[3] << 4) | (rx.data[4] >> 4);

    mot[mid].frames++;
    mot[mid].rxid    = rx.identifier;
    mot[mid].status  = rx.data[0] >> 4;
    mot[mid].pos_deg = uint_to_float(praw, P_MIN, P_MAX, 16) * RAD_TO_DEG;
    mot[mid].vel     = uint_to_float(vraw, V_MIN, V_MAX, 12);
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;

  if (s == "e") {
    bothUniversal(0xFB);  delay(50);       // 에러 해제
    sendLimp(CAN_ID_A);  sendLimp(CAN_ID_B);
    delay(50);
    bothUniversal(0xFC);  delay(50);       // enable — 토크는 여전히 0
    enabled = true;
    Serial.println(">>> ENABLE (토크 0 — 손으로 돌아갑니다)");
  } else if (s == "d") {
    bothUniversal(0xFD);
    enabled = false;
    Serial.println(">>> DISABLE");
  } else if (s == "z") {
    bothUniversal(0xFE);
    Serial.println(">>> 영점 재설정");
  } else if (s == "c") {
    for (int i = 0; i < 16; i++) mot[i].frames = 0;
    collide = 0;
    Serial.println(">>> 카운터 초기화");
  } else {
    Serial.println("명령: e(enable) / d(disable) / z(영점) / c(카운터 초기화)");
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
  bothUniversal(0xFD);                      // 부팅 시 비활성

  Serial.println("\n==================================================");
  Serial.println(" MIT 주소 충돌 점검 — 무동력 (Kp=0, Kd=0, t_ff=0)");
  Serial.println(" 모터는 힘을 쓰지 않습니다. 손으로 돌려보세요.");
  Serial.println("--------------------------------------------------");
  Serial.printf ("  명령 주소 : 0x%03X (모터 0x%02X) / 0x%03X (모터 0x%02X)\n",
                 CAN_MODE | CAN_ID_A, CAN_ID_A, CAN_MODE | CAN_ID_B, CAN_ID_B);
  Serial.println("  이 주소로 프레임이 들어오면 아직 충돌 상태입니다.");
  Serial.println("--------------------------------------------------");
  Serial.println("  e  enable (토크 0)   d  disable");
  Serial.println("  z  영점 재설정        c  카운터 초기화");
  Serial.println("==================================================\n");
}

void loop() {
  handleSerial();

  // 20Hz 로 무동력 프레임 전송 — 피드백을 받아내기 위한 것
  if (millis() - lastSend >= 50) {
    lastSend = millis();
    sendLimp(CAN_ID_A);
    sendLimp(CAN_ID_B);
  }

  drainCAN();

  if (millis() - lastPrint >= 200) {        // 5Hz
    lastPrint = millis();
    Serial.printf("[%s] 충돌:%lu | ", enabled ? "EN " : "DIS", collide);
    for (uint8_t id : {CAN_ID_A, CAN_ID_B}) {
      MotorInfo &m = mot[id];
      Serial.printf("모터%02X(rx 0x%03X %s %lu회) pos=%+7.2f°  ",
                    id, (unsigned)m.rxid, statusName(m.status), m.frames, m.pos_deg);
    }
    Serial.println();
  }
}

/* -----------------------------------------------------------------------------
 * 점검 순서
 *
 *  1) 전원 켜고 업로드. 아직 e 를 치지 않은 상태로 출력만 본다.
 *     두 모터의 피드백 횟수가 늘어나면 통신 정상.
 *
 *  2) rx 주소를 확인한다.
 *       모터01 rx 0x000   ← Master ID 0x00
 *       모터02 rx 0x011   ← Master ID 0x11 로 변경한 결과
 *     "충돌:0" 이 유지되어야 한다. 숫자가 올라가면 Master ID 를 다시 확인한다.
 *
 *  3) e 를 쳐서 enable 한다. 토크가 0 이므로 아무 일도 일어나지 않아야 한다.
 *     ★ 이때 조금이라도 움직이면 즉시 d 를 치고 전원을 내린다 ★
 *
 *  4) 손으로 각 축을 천천히 돌려본다.
 *     돌린 축에 해당하는 모터의 pos 만 변해야 한다.
 *     여기서 어느 모터가 어느 물리 축인지 확정된다. 결과를 기록할 것.
 *
 *  5) 여기까지 통과해야 제어 스케치로 넘어간다.
 *     넘어갈 때도 Kp 는 아주 작은 값(예: 1)부터 올린다.
 * ---------------------------------------------------------------------------*/
