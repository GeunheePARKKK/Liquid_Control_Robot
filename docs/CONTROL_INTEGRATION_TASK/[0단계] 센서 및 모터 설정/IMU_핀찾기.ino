/*
 * IMU_핀찾기.ino
 * -----------------------------------------------------------------------------
 * IMU 가 WHO_AM_I 에 0x00 만 돌려줄 때, 어느 선이 잘못됐는지 찾는 스케치.
 *
 * 모터는 전혀 건드리지 않는다. CAN 을 아예 초기화하지 않는다.
 *
 * 하는 일
 *   1. MISO 핀이 실제로 무언가에 연결돼 있는지 검사 (풀업/풀다운 토글)
 *   2. CS 후보 핀을 하나씩 바꿔가며 WHO_AM_I 를 읽어본다
 *   3. 0x6C 가 나오는 조합을 찾으면 알려준다
 *
 * 배선이 맞다면 CS=10 에서 0x6C 가 나와야 한다.
 * 다른 핀에서 나오면 그 핀에 CS 가 꽂혀 있는 것이다.
 * 어느 핀에서도 안 나오면 CS 가 아니라 SDA/SCL/SDO/전원 쪽 문제다.
 * -----------------------------------------------------------------------------
 */

#include <SPI.h>

#define PIN_MOSI  11    // 보드 SDA
#define PIN_SCK   12    // 보드 SCL
#define PIN_MISO  13    // 보드 SDO

#define SPI_SPEED 1000000
#define SPI_MD    SPI_MODE0

#define REG_WHO_AM_I  0x0F
#define WHO_AM_I_VAL  0x6C

// CS 가 꽂혀 있을 만한 곳. 사용 중이거나 금지된 핀은 뺐다.
const int CS_CANDIDATES[] = {10, 9, 14, 15, 16, 21, 1, 2, 6, 7, 8, 47, 48};
const int N_CS = sizeof(CS_CANDIDATES) / sizeof(CS_CANDIDATES[0]);

uint8_t readWho(int cs) {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MD));
  digitalWrite(cs, LOW);
  delayMicroseconds(20);
  SPI.transfer(REG_WHO_AM_I | 0x80);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v;
}

/* MISO 핀이 무언가에 연결돼 있는지 본다.
 * 내부 풀업/풀다운을 걸었을 때 값이 그대로 따라오면 = 아무것도 안 붙은 것.
 * 한쪽으로 고정돼 있으면 = 무언가가 그 핀을 잡고 있다.
 */
void checkMiso() {
  Serial.println("\n── MISO(GPIO13) 연결 검사 ──");
  pinMode(PIN_MISO, INPUT_PULLUP);
  delay(20);
  int up = digitalRead(PIN_MISO);
  pinMode(PIN_MISO, INPUT_PULLDOWN);
  delay(20);
  int dn = digitalRead(PIN_MISO);

  Serial.printf("  풀업 %d / 풀다운 %d  →  ", up, dn);
  if (up == 1 && dn == 0) {
    Serial.println("핀이 떠 있다. SDO 선이 안 꽂혔거나 끊겼다. ★");
  } else if (up == 0 && dn == 0) {
    Serial.println("항상 LOW. SDO 가 GND 에 물렸거나, IMU 가 I2C 모드로 SDO 를 눌러 잡고 있다.");
  } else if (up == 1 && dn == 1) {
    Serial.println("항상 HIGH. SDO 가 3V3 에 물렸을 수 있다.");
  }
  pinMode(PIN_MISO, INPUT);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n===== IMU 핀 찾기 =====");
  Serial.printf("고정: SDA=GPIO%d  SCL=GPIO%d  SDO=GPIO%d\n", PIN_MOSI, PIN_SCK, PIN_MISO);

  checkMiso();

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // 후보 핀 전부 HIGH 로 올려둔다. 하나씩만 내려서 시험하기 위함.
  for (int i = 0; i < N_CS; i++) {
    pinMode(CS_CANDIDATES[i], OUTPUT);
    digitalWrite(CS_CANDIDATES[i], HIGH);
  }
  delay(100);

  Serial.println("\n── CS 후보 스캔 ──");
  int found = -1;
  for (int i = 0; i < N_CS; i++) {
    int cs = CS_CANDIDATES[i];
    uint8_t v1 = readWho(cs);
    delay(5);
    uint8_t v2 = readWho(cs);          // 두 번 읽어 우연 배제
    Serial.printf("  GPIO%-2d → 0x%02X 0x%02X %s\n", cs, v1, v2,
                  (v1 == WHO_AM_I_VAL && v2 == WHO_AM_I_VAL) ? "  ★★★ 찾음" : "");
    if (v1 == WHO_AM_I_VAL && v2 == WHO_AM_I_VAL) found = cs;
  }

  Serial.println("\n===== 결과 =====");
  if (found == 10) {
    Serial.println("  CS = GPIO10.  배선 정상. 원래 스케치로 돌아가면 된다.");
  } else if (found > 0) {
    Serial.printf("  CS 가 GPIO%d 에 꽂혀 있다. GPIO10 으로 옮기거나\n", found);
    Serial.printf("  코드의 CS_BASE 를 %d 으로 바꿀 것.\n", found);
  } else {
    Serial.println("  어느 핀에서도 응답이 없다. CS 문제가 아니다.");
    Serial.println("  아래 순서로 확인할 것:");
    Serial.println("    1) 위 MISO 검사 결과");
    Serial.println("    2) SDA=GPIO11, SCL=GPIO12 두 선이 제자리인지");
    Serial.println("    3) IMU VIN 에 3.3V 가 실제로 걸리는지 (멀티미터)");
    Serial.println("    4) IMU GND 와 ESP32 GND 도통");
  }
  Serial.println("=================\n");
}

void loop() {
  delay(1000);
}
