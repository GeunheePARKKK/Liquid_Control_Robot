// ============================================================================
// serial_bridge.ino — PC ↔ 모터 드라이버 시리얼 양방향 중계
//
// 목적: ESP32를 USB-TTL 변환기 대신 써서, PC의 상위 SW(gl-ii-upper-computer)가
//       모터 드라이버와 직접 대화할 수 있게 한다. S-link 없이 CAN ID를 바꾸는 용도.
//
// 이 스케치는 serial_probe.ino 로 baud 를 찾은 다음에 쓴다.
// 절차 문서: ../../모터_CAN_ID_변경_가이드.md  (9장 화면 조작)
//
// ---------------------------------------------------------------------------
// 배선 (serial_probe 와 동일)
//   모터 드라이버 3핀           ESP32-S3
//   -----------------          --------
//   1  GND  ------------------  GND
//   2  TX   -----------------> GPIO18   (모터 -> PC 방향)
//   3  RX   <----------------- GPIO17   (PC -> 모터 방향)   * 이 스케치에서 실제로 쓰임
//
//   * 모터에 24V 전원 필수
//   * serial_probe 는 받는 방향만 검증한다. 이 스케치에서 처음으로 보내는 방향을 쓴다.
//     상위 SW가 응답을 못 받으면 GPIO17 <-> 3번 핀 연결을 먼저 의심할 것.
//
// ---------------------------------------------------------------------------
// ⚠ 반드시 할 설정 두 가지
//
// 1) 아래 MOTOR_BAUD 를 serial_probe 로 찾은 값으로 바꿀 것
//
// 2) Arduino IDE:  Tools -> USB CDC On Boot -> Enabled
//
//    이유: CDC(네이티브 USB)를 쓰면 PC쪽 baud 설정이 무의미해진다.
//          상위 SW가 몇으로 포트를 열든 그대로 통과한다.
//          이 설정을 끄면 Serial 이 UART0(브리지 칩)로 가고, 그때는 상위 SW가 여는
//          baud 와 아래 Serial.begin() 값이 정확히 같아야 해서 실패하기 쉽다.
//
// ---------------------------------------------------------------------------
// 사용법
//   1. MOTOR_BAUD 수정 후 업로드
//   2. 상위 SW 실행 -> 이 ESP32의 COM 포트 선택 -> Open     <-- 먼저!
//   3. 모터 전원 ON                                        <-- 나중에!
//   4. 화면에 드라이버 정보가 출력되면 연결 성공
//   5. "Set parameters" -> "Read Parameters" -> CAN ID 변경 -> "Write Parameters"
//   6. 모터 전원 OFF -> ON 후 값이 바뀐 것 확인
// ============================================================================

#define MOTOR_RX   18       // ESP32가 받는 핀  <- 모터 TX (2번 핀)
#define MOTOR_TX   17       // ESP32가 보내는 핀 -> 모터 RX (3번 핀)

#define MOTOR_BAUD 921600   // CubeMars 통신 설정에 표시된 값 (2026-08-23 확인)

void setup() {
  Serial.begin(MOTOR_BAUD);   // PC 쪽. CDC 사용 시 이 값은 무시된다
  Serial1.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
}

void loop() {
  // PC -> 모터
  while (Serial.available())  Serial1.write(Serial.read());
  // 모터 -> PC
  while (Serial1.available()) Serial.write(Serial1.read());
}
