/*
 * IMU_I2C확인.ino
 * -----------------------------------------------------------------------------
 * SPI 로 응답하지 않는 LSM6DSOX 가 I2C 로는 살아있는지 확인한다.
 *
 * 목적은 칩 손상과 SPI 경로 문제를 가르는 것.
 *
 *   I2C 로 0x6C 가 나온다  →  칩은 살아있다. SPI 쪽만 문제
 *   I2C 로도 응답 없음      →  칩 손상 쪽
 *
 * 모터는 건드리지 않는다. CAN 을 초기화하지 않는다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 배선 (SPI 에서 두 선만 바꾸면 된다)
 *
 *   IMU        연결            비고
 *   ---        ----            ----
 *   VIN   →   3V3
 *   GND   →   GND
 *   SDA   →   GPIO 11         그대로
 *   SCL   →   GPIO 12         그대로
 *   CS    →   3V3      ★변경   HIGH 로 두면 I2C 모드로 들어간다
 *   SDO   →   GND      ★변경   주소 하위비트. GND 면 0x6A, 3V3 이면 0x6B
 *
 * I2C 는 SDA·SCL 에 풀업이 필요하다. 브레이크아웃에 내장된 경우가 많고
 * ESP32 내부 풀업으로도 짧은 거리면 대개 동작한다. 안 되면 4.7kΩ 을
 * SDA·SCL 각각에서 3V3 으로 달 것.
 *
 * 하는 일
 *   1. 0x08~0x77 전체 주소를 훑어 응답하는 장치를 찾는다
 *   2. 찾은 주소에서 WHO_AM_I(0x0F) 를 읽어 0x6C 인지 본다
 *   3. 1초마다 반복 — 접촉이 불안정해도 간헐적으로 잡히는지 볼 수 있다
 * -----------------------------------------------------------------------------
 */

#include <Wire.h>

#define PIN_SDA  11
#define PIN_SCL  12

#define REG_WHO_AM_I  0x0F
#define WHO_AM_I_VAL  0x6C

// LSM6DSOX 는 SDO 핀 레벨로 두 주소 중 하나가 된다
const uint8_t ADDR_SDO_LOW  = 0x6A;
const uint8_t ADDR_SDO_HIGH = 0x6B;

uint32_t round_no = 0;

// 해당 주소에서 레지스터 한 바이트 읽기. 실패하면 false.
bool readReg(uint8_t addr, uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  out = Wire.read();
  return true;
}

void scanBus() {
  int found = 0;
  Serial.print("  응답 주소:");
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      found++;
    }
  }
  if (!found) Serial.print(" (없음)");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n===== IMU I2C 확인 =====");
  Serial.printf("SDA=GPIO%d  SCL=GPIO%d  (100kHz)\n", PIN_SDA, PIN_SCL);
  Serial.println("배선: CS→3V3, SDO→GND 로 바꿔야 한다");
  Serial.println("------------------------------------");

  Wire.begin(PIN_SDA, PIN_SCL, 100000);   // 100kHz — 느릴수록 잘 잡힌다
  delay(100);
}

void loop() {
  Serial.printf("\n[%lu회]\n", ++round_no);
  scanBus();

  bool ok = false;
  for (uint8_t addr : {ADDR_SDO_LOW, ADDR_SDO_HIGH}) {
    uint8_t v;
    if (readReg(addr, REG_WHO_AM_I, v)) {
      Serial.printf("  0x%02X → WHO_AM_I 0x%02X %s\n", addr, v,
                    v == WHO_AM_I_VAL ? "  ★★★ 칩 살아있음" : "  (기대 0x6C)");
      if (v == WHO_AM_I_VAL) ok = true;
    } else {
      Serial.printf("  0x%02X → 응답 없음\n", addr);
    }
  }

  if (ok) {
    Serial.println("  판정: 칩 정상. SPI 경로만 문제다.");
  } else {
    Serial.println("  판정: I2C 로도 응답 없음. 아래를 확인할 것.");
    Serial.println("        CS 가 3V3 에, SDO 가 GND 에 연결됐는지");
    Serial.println("        SDA·SCL 풀업 (4.7kΩ → 3V3)");
    Serial.println("        전원이 실제로 칩까지 가는지");
  }

  delay(1000);
}

/* -----------------------------------------------------------------------------
 * 결과 해석
 *
 *  ① 0x6A 또는 0x6B 에서 WHO_AM_I = 0x6C
 *     → 칩은 멀쩡하다. SPI 인터페이스나 그 경로만 죽은 것.
 *       CS·SDO 배선을 다시 보거나, 최악의 경우 I2C 로 계속 쓰는 방법도 있다.
 *       (다만 두 IMU 를 같은 버스에 붙이려면 주소가 달라야 하므로
 *        한쪽 SDO 를 3V3 에 두어 0x6B 로 쓰면 된다)
 *
 *  ② 스캔에 아무 주소도 안 잡힘
 *     → 풀업이 없거나 배선이 잘못됐을 수 있다. 4.7kΩ 을 달고 다시 볼 것.
 *       그래도 안 잡히면 칩 손상 쪽으로 본다.
 *
 *  ③ 주소는 잡히는데 WHO_AM_I 가 0x6C 가 아님
 *     → 다른 칩이 붙어 있다. 그 값으로 칩 종류를 다시 확인할 것.
 * ---------------------------------------------------------------------------*/
