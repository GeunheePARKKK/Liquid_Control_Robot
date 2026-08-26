/*
 * motor_test.ino
 * -----------------------------------------------------------------------------
 * 짐벌모터 2축 동시 구동 테스트
 *
 * 같은 CAN 버스에 물린 모터 2개에 동일한 목표를 보내 ±POS_RAD 왕복시킨다.
 * 두 모터가 같은 동작을 하는지, 피드백이 정상적으로 올라오는지 확인용.
 *
 *   MCU     : ESP32-S3 (TWAI)
 *   버스    : CAN 1Mbit/s, TX=GPIO4 / RX=GPIO5, 종단저항 100Ω
 *   모터    : CubeMars 툴에서 아래와 같이 설정
 *
 *       ┌────────┬────────┬───────────┐
 *       │ 모터   │ CAN ID │ Master ID │
 *       ├────────┼────────┼───────────┤
 *       │  1축   │  0x01  │   0x00    │
 *       │  2축   │  0x02  │   0x01    │
 *       └────────┴────────┴───────────┘
 *
 * 프로토콜
 *   송신 ID   : 0x100 | CAN_ID       (위치/속도 모드)
 *   송신 데이터: [0:3] 목표 위치 float LE, [4:7] 속도 제한 float LE
 *   유니버설  : 0xFF × 7 + 명령바이트
 *               0xFB=에러해제  0xFC=enable  0xFD=disable  0xFE=현재위치 영점
 *   수신 데이터: data[0] 상위4bit=에러, 하위4bit=모터 CAN ID
 *               data[1:2] 위치(±12.5rad), data[6]=MOS온도, data[7]=로터온도
 *
 * 확인 사항
 *   - 로그의 rxid 로 Master ID 가 피드백 식별자로 쓰이는지 확인
 *   - 한쪽만 응답하면: 종단저항 / 모터 CAN ID 저장 여부 / CANH·CANL 분기 배선
 *
 * 배선 상세는 hardware/배선도.md 참고.
 * -----------------------------------------------------------------------------
 */

#include "driver/twai.h"

#define N_MOTOR 2

// CubeMars 툴에서 설정한 값 — {CAN_ID, MASTER_ID}
const uint8_t CAN_ID[N_MOTOR]    = { 0x01, 0x02 };
const uint8_t MASTER_ID[N_MOTOR] = { 0x00, 0x01 };

#define POS_RAD    0.5f    // ±0.5 rad 왕복 (범위 ±12.5 rad)
#define VEL_LIMIT  1.0f    // 속도 제한 1 rad/s

void sendPosVel(uint8_t id, float pos, float vel) {
  twai_message_t m = {};
  m.identifier = 0x100 | id;         // 위치/속도 모드
  m.data_length_code = 8;
  memcpy(&m.data[0], &pos, 4);       // 목표 위치 (float LE)
  memcpy(&m.data[4], &vel, 4);       // 속도 제한 (float LE)
  twai_transmit(&m, pdMS_TO_TICKS(10));
}

void sendUniversal(uint8_t id, uint8_t last) {  // 0xFC=enable, 0xFD=disable, 0xFB=err clear, 0xFE=영점
  twai_message_t m = {};
  m.identifier = 0x100 | id;
  m.data_length_code = 8;
  memset(m.data, 0xFF, 7);
  m.data[7] = last;
  twai_transmit(&m, pdMS_TO_TICKS(10));
}

// 전체 모터에 같은 유니버설 명령 전송
void broadcastUniversal(uint8_t last) {
  for (int i = 0; i < N_MOTOR; i++) sendUniversal(CAN_ID[i], last);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
  delay(100);

  broadcastUniversal(0xFB);  delay(50);   // 에러 해제
  broadcastUniversal(0xFE);  delay(50);   // 현재 위치 영점 (왕복 기준점)
  broadcastUniversal(0xFC);  delay(100);  // enable

  Serial.printf("모터 %d개 초기화 완료 (CAN ID: 0x%02X, 0x%02X)\n",
                N_MOTOR, CAN_ID[0], CAN_ID[1]);
}

void loop() {
  static bool dir = true;
  static uint32_t last = 0;
  if (millis() - last >= 500) { last = millis(); dir = !dir; }

  // 두 모터에 동일한 목표 전송
  float target = dir ? POS_RAD : -POS_RAD;
  for (int i = 0; i < N_MOTOR; i++) sendPosVel(CAN_ID[i], target, VEL_LIMIT);

  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    uint8_t  mid = rx.data[0] & 0x0F;          // 피드백 프레임 안의 모터 CAN ID
    uint8_t  err = rx.data[0] >> 4;
    uint16_t p   = (rx.data[1] << 8) | rx.data[2];
    float    pos = ((float)p) * 25.0f / 65535.0f - 12.5f;

    Serial.printf("[모터%u] rxid=0x%03X 상태=%X pos=%+.2frad temp=%d/%d\n",
                  mid, (unsigned)rx.identifier, err, pos,
                  (int8_t)rx.data[6], (int8_t)rx.data[7]);
  }
  delay(20);
}
