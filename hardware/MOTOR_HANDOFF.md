# GL60II 모터 구동 핸드오프 문서 (검증 완료: 2026-08-22)

## 1. 하드웨어 구성 (확정)
- MCU: **ESP32-S3** (Arduino IDE, 내장 TWAI 컨트롤러 사용)
- CAN 트랜시버: **SN65HVD230** (3.3V, VCC→3V3)
- 모터: **CubeMars GL60II KV28 + 전용 드라이버**
- 배선: 트랜시버 TXD→**GPIO4**, RXD→**GPIO5**, CANH↔흰색, CANL↔파랑,
  종단저항 100Ω CANH-CANL 사이, **GND 공통 필수** (전원− ↔ 트랜시버 GND ↔ ESP32 GND)
- 전원: 24V, 전류 제한 **최소 2A** (1A는 방향 전환 피크에 컷오프됨 — 실측)

## 2. CAN 프로토콜 (매뉴얼 `hardware/gl-ii-gimbal-motor-drive-user-manual.pdf` p.41~49에서 추출, 실기 검증됨)
- **표준 11-bit 프레임, 1Mbps 고정** (`TWAI_TIMING_CONFIG_1MBITS()`)
- CAN ID = `(제어모드 << 8) | 모터ID`
  - 모드 0 = MIT, 모드 1 = 위치/속도, 모드 2 = 속도
- **⚠️ 제어 모드는 CAN으로 변경 불가** — 드라이버 플래시에 저장, 상위 PC 툴(시리얼)로만 변경
- **현재 우리 드라이버 설정 (스캔으로 확정): 모드 1 (위치/속도), 모터 ID = 1, 피드백 ID(Master ID) = 0x0**
  - → 모든 명령은 **ID `0x101`** 로 전송

### 공통 명령 (DLC=8, ID=0x101)
| 명령 | 데이터 |
|---|---|
| Enable | `FF FF FF FF FF FF FF FC` |
| Disable | `FF FF FF FF FF FF FF FD` |
| 현재 위치 영점 | `FF FF FF FF FF FF FF FE` |
| 에러 해제 | `FF FF FF FF FF FF FF FB` |

### 위치/속도 명령 (DLC=8, ID=0x101)
- Data[0..3] = 목표 위치 (IEEE754 **float, little-endian**, 범위 ±12.5 rad)
- Data[4..7] = 속도 제한 (float LE, 범위 ±200 rad/s)

### 피드백 프레임 (ID=0x0)
- Data[0]: 상위니블=상태코드, 하위니블=모터ID
  - 0=Disable, 1=Enable, 8=과전압, 9=저전압, A=과전류, B=MOS과열, C=권선과열, D=통신끊김, E=과부하
- Data[1..2]: 위치 16bit → `raw*25.0/65535 - 12.5` rad
- Data[3..4]: 속도 12bit(±200), Data[4하위..5]: 토크 12bit(±10)
- Data[6]/Data[7]: 드라이버/모터 온도 (int8, °C)

## 3. 검증된 최소 동작 코드 (0.5초 왕복)
```cpp
#include "driver/twai.h"
#define MOTOR_ID 0x01
#define POS_RAD  0.5f   // 살살: ±0.5 rad
#define VEL_LIMIT 1.0f  // 살살: 1 rad/s

void sendPosVel(float pos, float vel) {
  twai_message_t m = {};
  m.identifier = 0x100 | MOTOR_ID;
  m.data_length_code = 8;
  memcpy(&m.data[0], &pos, 4);
  memcpy(&m.data[4], &vel, 4);
  twai_transmit(&m, pdMS_TO_TICKS(10));
}
void sendUniversal(uint8_t last) {
  twai_message_t m = {};
  m.identifier = 0x100 | MOTOR_ID;
  m.data_length_code = 8;
  memset(m.data, 0xFF, 7); m.data[7] = last;
  twai_transmit(&m, pdMS_TO_TICKS(10));
}
void setup() {
  Serial.begin(115200);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f); twai_start(); delay(100);
  sendUniversal(0xFB); delay(50);  // 에러 해제
  sendUniversal(0xFE); delay(50);  // 영점
  sendUniversal(0xFC); delay(100); // enable
}
void loop() {
  static bool dir = true; static uint32_t last = 0;
  if (millis() - last >= 500) { last = millis(); dir = !dir; }
  sendPosVel(dir ? POS_RAD : -POS_RAD, VEL_LIMIT);
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK)
    Serial.printf("상태=%X pos=%.2f\n", rx.data[0]>>4,
                  ((float)((rx.data[1]<<8)|rx.data[2]))*25.0f/65535.0f-12.5f);
  delay(20);
}
```

## 4. 삽질 로그 (같은 실수 반복 금지)
1. ❌ AK시리즈용 **확장 29-bit + ERPM int32** 프로토콜 → GL II는 완전히 다름 (표준 11-bit + float)
2. ❌ RMD식 `0x140+ID / 0xA1/0xA2` 프로토콜 → GL II 아님
3. ❌ 속도 모드(`0x201`) 명령 → 드라이버가 위치/속도 모드로 설정돼 있어 무시됨
4. ✅ **모드/ID 불명확할 땐 스캔**: 모드 0~2 × ID 0~127에 enable을 쏘고 피드백이 오는 조합을 찾으면 확정됨
5. 진단 순서: TWAI `state=1`+에러0 → 물리층 OK / `twai_transmit`이 ESP_OK(ACK) → 비트레이트 OK / 피드백 무응답 → 프레임 내용(모드/ID) 문제

## 5. 다음 단계
- 이 프로토콜을 팀 규약(`docs/ai-handoff/MOTOR_CAN_TASK.md`)대로 `firmware/gimbal_control/Gimbal.h/.cpp`의 `GimbalBus` 클래스로 이식
- 핀/ID/제한값은 `Config.h`에서 읽기 (하드코딩 금지)
- 두 번째 모터 추가 시 CAN ID 분리 필요 (PC 툴로 설정)
