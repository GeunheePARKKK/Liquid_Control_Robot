#include "driver/twai.h"

#define MOTOR_ID   0x01
#define POS_RAD    0.5f    // ±3 rad 왕복 (범위 ±12.5 rad)
#define VEL_LIMIT  1.0f    // 최대 속도 6 rad/s

void sendPosVel(float pos, float vel) {
  twai_message_t m = {};
  m.identifier = 0x100 | MOTOR_ID;   // 위치/속도 모드 = 0x101
  m.data_length_code = 8;
  memcpy(&m.data[0], &pos, 4);       // 목표 위치 (float LE)
  memcpy(&m.data[4], &vel, 4);       // 속도 제한 (float LE)
  twai_transmit(&m, pdMS_TO_TICKS(10));
}

void sendUniversal(uint8_t last) {   // 0xFC=enable, 0xFD=disable, 0xFB=err clear, 0xFE=영점
  twai_message_t m = {};
  m.identifier = 0x100 | MOTOR_ID;
  m.data_length_code = 8;
  memset(m.data, 0xFF, 7);
  m.data[7] = last;
  twai_transmit(&m, pdMS_TO_TICKS(10));
}

void setup() {
  Serial.begin(115200);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
  delay(100);
  sendUniversal(0xFB);  delay(50);   // 에러 해제
  sendUniversal(0xFE);  delay(50);   // 현재 위치 영점 (왕복 기준점)
  sendUniversal(0xFC);  delay(100);  // enable
}

void loop() {
  static bool dir = true;
  static uint32_t last = 0;
  if (millis() - last >= 500) { last = millis(); dir = !dir; }
  sendPosVel(dir ? POS_RAD : -POS_RAD, VEL_LIMIT);

  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    uint8_t err = rx.data[0] >> 4;
    uint16_t p  = (rx.data[1] << 8) | rx.data[2];
    float pos = ((float)p) * 25.0f / 65535.0f - 12.5f;
    Serial.printf("상태=%X pos=%.2frad temp=%d/%d\n", err, pos, (int8_t)rx.data[6], (int8_t)rx.data[7]);
  }
  delay(20);
}