/*
 * drv8825_s3_test.ino
 *
 * ESP32-S3 + DRV8825 스텝모터 기본 동작 확인 테스트
 * 정회전 1바퀴 -> 역회전 1바퀴 -> 정회전 5바퀴 반복
 *
 * ---- 배선 ----------------------------------------------------------------
 *   DRV8825 STEP   -> ESP32-S3 GPIO 5
 *   DRV8825 DIR    -> ESP32-S3 GPIO 4
 *   DRV8825 RESET  -> ESP32-S3 3V3      (필수. 미연결 시 드라이버가 리셋에 잠김)
 *   DRV8825 SLEEP  -> ESP32-S3 3V3      (필수)
 *   DRV8825 GND    -> ESP32-S3 GND      (18V(-)와 공통 필수)
 *   DRV8825 VMOT   -> 18V(+)
 *   DRV8825 A1/A2  -> 모터 코일 1
 *   DRV8825 B1/B2  -> 모터 코일 2
 *   M0/M1/M2 미연결 = 풀스텝(200 스텝/회전)
 *
 *   * VMOT-GND 사이에 전해 커패시터 100uF / 35V 이상을 "병렬로" 필수 연결
 *   * DRV8825에는 VDD 핀이 없다. A4988의 VDD 자리가 FAULT 출력이다.
 *
 * ---- 실행 전 확인 --------------------------------------------------------
 *   1. 모터 4선을 뺀 상태로 전원을 인가하고 VREF를 설정할 것
 *        전류 제한(A) = VREF(V) x 2      (첫 테스트는 VREF 0.25V = 0.5A 권장)
 *   2. 전원을 끄고 모터를 연결한 뒤 다시 인가
 *   3. 전원이 인가된 상태에서 모터선을 탈착하면 드라이버가 파괴된다
 *
 * ---- 주의 ----------------------------------------------------------------
 *   gimbal_control이 CAN TX/RX로 GPIO4 / GPIO5를 사용한다.
 *   한 보드에 통합할 때는 STEP/DIR을 GPIO 15 / 16 등으로 옮길 것.
 *
 * 보드: ESP32S3 Dev Module / 시리얼 모니터 115200 bps
 */

#define STEP_PIN 5
#define DIR_PIN  4

// 1.8도 풀스텝 모터 기준. 0.9도 모터면 400.
// 마이크로스텝을 켜면 배수만큼 곱할 것 (1/2 스텝 -> 400)
const int STEPS_PER_REV = 200;

// 펄스 폭(us). 클수록 느리다.
// 1500 = 1회전 약 0.6초. 첫 테스트는 이 값으로 시작하고
// 800 -> 400 순으로 줄여가며 탈조 지점을 찾는다.
unsigned int pulseUs = 1500;

void stepMotor(int steps, bool clockwise, unsigned int us) {
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
  delayMicroseconds(10);              // DIR 셋업 시간

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(us);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(us);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  Serial.println();
  Serial.println("=== DRV8825 stepper test ===");
  Serial.printf("STEP=GPIO%d, DIR=GPIO%d, %d steps/rev, pulse=%uus\n",
                STEP_PIN, DIR_PIN, STEPS_PER_REV, pulseUs);
  Serial.println("3초 후 시작합니다.");
  delay(3000);
}

void loop() {
  Serial.println("[1] 시계 방향 1회전");
  stepMotor(STEPS_PER_REV, true, pulseUs);
  delay(1000);

  Serial.println("[2] 반시계 방향 1회전");
  stepMotor(STEPS_PER_REV, false, pulseUs);
  delay(1000);

  Serial.println("[3] 시계 방향 5회전");
  stepMotor(STEPS_PER_REV * 5, true, pulseUs);
  delay(1500);

  Serial.println("--- 한 사이클 완료, 2초 후 반복 ---");
  Serial.println();
  delay(2000);
}
