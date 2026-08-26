#include <SPI.h>

#define LSM_CS 9
#define REG_WHOAMI    0x0F
#define REG_CTRL1_XL  0x10
#define REG_CTRL2_G   0x11
#define REG_OUTX_L_XL 0x28

SPIClass mySPI = SPIClass(FSPI);
SPISettings spiSettings(500000, MSBFIRST, SPI_MODE0);

const unsigned long SAMPLE_INTERVAL_MS = 10;
unsigned long lastSample = 0;
bool headerPrinted = false;

float CURRENT_VOLUME_PCT = 100;
String CURRENT_DISTURBANCE = "accel";
int CURRENT_TRIAL_ID = 1;

const float ACCEL_SENSITIVITY_G = 0.000244;  // ±8g 레인지, g/LSB
const float G_TO_MS2 = 9.80665;

uint8_t readRegister(uint8_t reg) {
  digitalWrite(LSM_CS, LOW);
  mySPI.transfer(reg | 0x80);
  uint8_t val = mySPI.transfer(0x00);
  digitalWrite(LSM_CS, HIGH);
  return val;
}

void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(LSM_CS, LOW);
  mySPI.transfer(reg & 0x7F);
  mySPI.transfer(value);
  digitalWrite(LSM_CS, HIGH);
}

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  uint8_t buf[6];
  digitalWrite(LSM_CS, LOW);
  mySPI.transfer(REG_OUTX_L_XL | 0x80);
  for (int i = 0; i < 6; i++) buf[i] = mySPI.transfer(0x00);
  digitalWrite(LSM_CS, HIGH);
  ax = (int16_t)((buf[1] << 8) | buf[0]);
  ay = (int16_t)((buf[3] << 8) | buf[2]);
  az = (int16_t)((buf[5] << 8) | buf[4]);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(LSM_CS, OUTPUT);
  digitalWrite(LSM_CS, HIGH);
  mySPI.begin(12, 13, 11, LSM_CS);

  mySPI.beginTransaction(spiSettings);
  uint8_t whoami = readRegister(REG_WHOAMI);
  if (whoami != 0x6C) {
    Serial.print("LSM6DSOX 연결 실패, WHO_AM_I=0x");
    Serial.println(whoami, HEX);
    mySPI.endTransaction();
    while (1) delay(10);
  }
  Serial.println("LSM6DSOX 연결됨 (raw SPI)");

  writeRegister(REG_CTRL1_XL, 0x4C); // ODR 104Hz, ±4g
  writeRegister(REG_CTRL2_G, 0x40);  // ODR 104Hz, ±250dps
  mySPI.endTransaction();
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;
    if (!headerPrinted) {
      Serial.println("timestamp_ms,ax,ay,az,volume_pct,disturbance,trial_id");
      headerPrinted = true;
    }

    int16_t rawX, rawY, rawZ;
    mySPI.beginTransaction(spiSettings);
    readAccel(rawX, rawY, rawZ);
    mySPI.endTransaction();

    float ax = rawX * ACCEL_SENSITIVITY_G * G_TO_MS2;
    float ay = rawY * ACCEL_SENSITIVITY_G * G_TO_MS2;
    float az = rawZ * ACCEL_SENSITIVITY_G * G_TO_MS2;

    Serial.printf("%lu,%.4f,%.4f,%.4f,%.1f,%s,%d\n",
      now, ax, ay, az, CURRENT_VOLUME_PCT, CURRENT_DISTURBANCE.c_str(), CURRENT_TRIAL_ID);
  }
}
