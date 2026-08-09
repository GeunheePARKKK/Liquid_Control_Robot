#include <SPI.h>
#include <PID_v1.h>

// --- 핀 설정 ---
const int CS_ACTIVE = 5;  // 상단 트레이에 붙은 IMU 센서의 선택(Chip Select) 핀 (BMI160으로 교체)
const int CS_FIXED = 4;   // 하단 베이스에 붙은 IMU 센서의 선택(Chip Select) 핀 (ICM-42688 유지)
const int MOTOR_Y = 22;   // Y축 제어용 모터(서보 또는 PWM) 출력 핀
const int MOTOR_X = 16;   // X축 제어용 모터(서보 또는 PWM) 출력 핀
const int SCK_PIN = 18;   // SPI 통신용 시계(Clock) 핀
const int SDO_PIN = 19;   // SPI 데이터 입력(MISO) 핀
const int SDI_PIN = 23;   // SPI 데이터 출력(MOSI) 핀


// --- PWM 및 수평 설정 ---
const int PWM_FREQ = 333;  // 모터에 보내는 PWM 신호의 주파수 (333Hz)
const int PWM_RES = 10;    // PWM 해상도 (10비트 = 0~1023 단계로 정밀 제어)
int HORIZ_X = 460;         // X축이 수평일 때 모터에 전달할 기본값 (중점)
int HORIZ_Y = 510;         // Y축이 수평일 때 모터에 전달할 기본값 (중점)

// 모터의 기계적 파손을 막기 위한 이동 범위 제한값
const int LIMIT_X_MIN = 300, LIMIT_X_MAX = 650;
const int LIMIT_Y_MIN = 400, LIMIT_Y_MAX = 800;

// --- 물리 상수 및 매핑 비율 ---
const float G_LSB = 16384.0;  // 가속도 센서가 1G(중력)를 감지했을 때 뱉는 Raw 데이터 값
const float GYRO_LSB = 16.4;  // 자이로 센서가 초당 1도 회전할 때 뱉는 Raw 데이터 값

// 계산된 각도(degree)를 모터 제어용 수치(LSB)로 바꿀 때 곱해주는 비율 (1도당 약 286단위)
const float SCALE_FACTOR_X = 286.0;
const float SCALE_FACTOR_Y = 286.0;

// --- 영점(Bias) 변수 ---
double static_bias_X = 0;  // X축 시스템 전체의 정적 오프셋
double static_bias_Y = 0;  // Y축 시스템 전체의 정적 오프셋
float gyro_bias_x = 0;     // 하단 센서의 자이로 X축 센서 오차 (정지 상태인데 0이 아닐 경우)
float gyro_bias_y = 0;     // 하단 센서의 자이로 Y축 센서 오차

float gyro_bias_x_a = 0;  // 상단 센서의 자이로 X축 센서 오차
float gyro_bias_y_a = 0;  // 상단 센서의 자이로 Y축 센서 오차
float acc_z_error = 0;    // 가속도 센서 Z축이 완벽한 1G가 아닐 때의 오차

// --- 상보 필터 변수 ---
float pitch_filtered = 0;        // 하단 베이스의 필터링된 피치(앞뒤 기울기) 각도
float roll_filtered = 0;         // 하단 베이스의 필터링된 롤(좌우 기울기) 각도
float tray_pitch_filtered = 0;  // 상단 트레이의 필터링된 피치 각도
float tray_roll_filtered = 0;   // 상단 트레이의 필터링된 롤 각도

const float dt = 0.01;        // 루프가 한 번 도는 시간 (10ms = 0.01초)
const float alpha_cf = 0.98;  // 상보 필터 계수. 자이로 데이터에 98%의 신뢰도를 줌

//ZV
const int ZV_BUFFER_SIZE = 40;
const int DELTA_IDX = 21;  // 1차 모드 지연
const int delta_idx = 11;  // 3차 모드 지연

// X축, Y축 각각 독립적인 메모리 공간 할당
float zv_buffer_x[ZV_BUFFER_SIZE] = {
  0,
};
float zv_buffer_y[ZV_BUFFER_SIZE] = {
  0,
};
int zv_head_x = 0;
int zv_head_y = 0;

// --- PID 및 제어 변수 ---
float filteredRelDiffX = 0, filteredRelDiffY = 0;  // 필터링된 상대 오차값 (사용처 확인 필요)
const float alpha_XY = 0.2;                        // LPF(저주파 통과 필터) 계수
const int deadzone = 0;                            // 불감대 설정 (0이면 사용 안 함)

double Kp_X = 0.01, Ki_X = 0.00, Kd_X = 0.000;  // X축 PID 게인값
double Kp_Y = 0.0051, Ki_Y = 0.00, Kd_Y = 0.000;  // Y축 PID 게인값

// --- X축 비선형 Gain Scheduling 파라미터 ---
const double Kp_X_MAX = 0.007;   // 오차 클 때 (기존값 유지)
const double Kp_X_MIN = 0.002;  // 오차 0에 가까울 때
const double ERROR_MAX_DEG = 23.0; // 이 각도 이상이면 Kp_MAX 그대로 사용

double setpointX = 0, setpointY = 0;      // 우리가 가고자 하는 목표 각도
double inputX, outputX, inputY, outputY;  // PID의 입력(현재각)과 출력(모터힘)

// PID 객체 초기화: 입력값, 출력값 저장처, 목표값, 게인, 제어방향(REVERSE) 설정
PID pidX(&inputX, &outputX, &setpointX, Kp_X, Ki_X, Kd_X, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp_Y, Ki_Y, Kd_Y, REVERSE);

#define SPI_SPEED 1000000
#define SPI_MODE SPI_MODE0

// -------------------------------------------------------------
// [하단 ICM-42688용 기존 함수 유지]
// -------------------------------------------------------------
void writeReg(int cs, uint8_t reg, uint8_t data) {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(cs, LOW);
  delayMicroseconds(20);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void readIMU(int cs, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[12];
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(cs, LOW);
  delayMicroseconds(20);
  SPI.transfer(0x1F | 0x80);
  for (int i = 0; i < 12; i++) b[i] = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  az = (int16_t)((b[4] << 8) | b[5]);
  gx = (int16_t)((b[6] << 8) | b[7]);
  gy = (int16_t)((b[8] << 8) | b[9]);
  gz = (int16_t)((b[10] << 8) | b[11]);
}

// -------------------------------------------------------------
// [상단 BMI160용 전용 함수 추가]
// -------------------------------------------------------------
void writeReg_BMI(int cs, uint8_t reg, uint8_t data) {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(cs, LOW);
  delayMicroseconds(20);
  SPI.transfer(reg & 0x7F); // BMI Write: MSB 0
  SPI.transfer(data);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void readIMU_BMI(int cs, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[12];
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(cs, LOW);
  delayMicroseconds(20);
  SPI.transfer(0x0C | 0x80); // BMI Data 주소 | Read: MSB 1
  for (int i = 0; i < 12; i++) b[i] = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  // BMI160 Little Endian 처리
  gx = (int16_t)((b[1] << 8) | b[0]);
  gy = (int16_t)((b[3] << 8) | b[2]);
  gz = (int16_t)((b[5] << 8) | b[4]);
  ax = (int16_t)((b[7] << 8) | b[6]);
  ay = (int16_t)((b[9] << 8) | b[8]);
  az = (int16_t)((b[11] << 8) | b[10]);
}

// 목표값(target)이 들어오면 진동을 줄이기 위해 과거값들과 섞는 함수
float applyConvolvedZV(float target, float *buffer, int &head) {
  buffer[head] = target;  // 현재 들어온 목표값을 버퍼의 현재 위치에 저장

  // 현재 시점(0), 지연 시점 1, 지연 시점 2, 결합 시점 3의 인덱스를 계산 (버퍼 오버플로우 방지 처리 포함)
  int idx_0 = head;
  int idx_1 = (head - DELTA_IDX + ZV_BUFFER_SIZE) % ZV_BUFFER_SIZE;
  int idx_2 = (head - delta_idx + ZV_BUFFER_SIZE) % ZV_BUFFER_SIZE;
  int idx_3 = (head - (DELTA_IDX + delta_idx) + ZV_BUFFER_SIZE) % ZV_BUFFER_SIZE;

  // 4개 시점의 목표값을 각각 0.25씩 나누어 합침 (입력을 부드럽게 쪼갬)
  float shaped_target = 0.25 * buffer[idx_0] + 0.25 * buffer[idx_1] + 0.25 * buffer[idx_2] + 0.25 * buffer[idx_3];

  head = (head + 1) % ZV_BUFFER_SIZE;  // 다음 데이터 저장을 위해 인덱스 한 칸 이동
  return shaped_target;                // 진동이 제거된 목표 각도 반환
}

void setup() {
    // SPI.begin() 이전에 추가
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);
  delay(1000);
  Serial.println("CS_ACTIVE(27) HIGH 설정됨");

  // 일부러 LOW로 내렸다가 다시 HIGH
  digitalWrite(27, LOW);
  delay(100);
  digitalWrite(27, HIGH);
  Serial.println("CS 토글 완료");

  Serial.begin(115200);
  ledcAttach(MOTOR_X, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_Y, PWM_FREQ, PWM_RES);
  ledcWrite(MOTOR_X, HORIZ_X);  // 시작 시 X축 수평 위치로 이동
  ledcWrite(MOTOR_Y, HORIZ_Y);  // 시작 시 Y축 수평 위치로 이동

  pinMode(SDO_PIN, INPUT);
  pinMode(CS_ACTIVE, OUTPUT);
  pinMode(CS_FIXED, OUTPUT);
  digitalWrite(CS_ACTIVE, HIGH);
  digitalWrite(CS_FIXED, HIGH);
  SPI.begin(SCK_PIN, SDO_PIN, SDI_PIN);
  delay(500);

  // setup() 안, SPI.begin() 이후 ~ writeReg 호출 이전에 추가

// ── WHO_AM_I 확인 함수 (상단 BMI160 규격에 맞게 0x00 주소 확인) ──
  auto checkWhoAmI_BMI = [](int cs) {
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
    digitalWrite(cs, LOW);
    delayMicroseconds(20);
    SPI.transfer(0x00 | 0x80); 
    uint8_t id = SPI.transfer(0x00);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    return id;
  };
// 하단 ICM용 WHOAMI
  auto checkWhoAmI_ICM = [](int cs) {
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
    digitalWrite(cs, LOW);
    delayMicroseconds(20);
    SPI.transfer(0x75 | 0x80); 
    uint8_t id = SPI.transfer(0x00);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();
    return id;
  };

  uint8_t id_active = checkWhoAmI_BMI(CS_ACTIVE);
  uint8_t id_fixed  = checkWhoAmI_ICM(CS_FIXED);
  Serial.printf("CS_ACTIVE Sensor ID: 0x%02X (Expected BMI: 0xD1)\n", id_active);
  Serial.printf("CS_FIXED  Sensor ID: 0x%02X (Expected ICM: 0x47)\n", id_fixed);

  // 상단 BMI160 전원 켜기 및 측정 설정
  writeReg_BMI(CS_ACTIVE, 0x7E, 0xB6); delay(50);

  // 변경 후 (안전)
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(CS_ACTIVE, LOW);
  delayMicroseconds(10);
  digitalWrite(CS_ACTIVE, HIGH);
  SPI.endTransaction();
  delay(10);

  writeReg_BMI(CS_ACTIVE, 0x7E, 0x11); delay(50);
  writeReg_BMI(CS_ACTIVE, 0x7E, 0x15); delay(50);
  writeReg_BMI(CS_ACTIVE, 0x41, 0x03); 
  writeReg_BMI(CS_ACTIVE, 0x40, 0x03); 
  
  writeReg_BMI(CS_ACTIVE, 0x42, 0x08);  // Accel ODR 1600Hz
  writeReg_BMI(CS_ACTIVE, 0x40, 0x0C);  // Accel +-2g
  writeReg_BMI(CS_ACTIVE, 0x43, 0x00);  // Gyro ODR 3200Hz
  writeReg_BMI(CS_ACTIVE, 0x41, 0x00);  // Gyro +-2000dps
  delay(100);

  // 하단 ICM-42688 전원 켜기 및 측정 설정 (기존 로직)
  writeReg(CS_FIXED, 0x11, 0x01);
  delay(100);

  writeReg(CS_FIXED, 0x4E, 0x0F);
  delay(100);
  writeReg(CS_FIXED, 0x50, 0x69);  // 가속도 측정 켬
  writeReg(CS_FIXED, 0x4F, 0x09);  // 자이로 측정 켬

  

  // 초기화 직후 확인
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE));
  digitalWrite(CS_ACTIVE, LOW);
  delayMicroseconds(10);
  SPI.transfer(0x03 | 0x80);  // PMU_STATUS 레지스터
  uint8_t pmu = SPI.transfer(0x00);
  digitalWrite(CS_ACTIVE, HIGH);
  SPI.endTransaction();
  Serial.printf("PMU_STATUS: 0x%02X (정상: 0x14)\n", pmu);

  // 절대 영점 및 자이로 오프셋 측정
  Serial.println("\n>>> CALIBRATING BASE SENSORS...");
  long sumX = 0, sumY = 0, sumGx = 0, sumGy = 0, sumZ = 0, sumGx_a = 0, sumGy_a = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax_a, ay_a, az_a, gx_a, gy_a, gz_a;
    int16_t ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
    readIMU_BMI(CS_ACTIVE, ax_a, ay_a, az_a, gx_a, gy_a, gz_a);

    delayMicroseconds(200);
    readIMU(CS_FIXED, ax_f, ay_f, az_f, gx_f, gy_f, gz_f);


    sumGx += gx_f;  // 하단 자이로 누적
    sumGy += gy_f;

    sumZ += az_f;
    sumGx_a += gx_a;
    sumGy_a += gy_a;
    delay(5);
  }

  gyro_bias_x = (float)sumGx / 200.0;
  gyro_bias_y = (float)sumGy / 200.0;
  gyro_bias_x_a = (float)sumGx_a / 200.0;
  gyro_bias_y_a = (float)sumGy_a / 200.0;

  acc_z_error = (float)sumZ / 200.0;
  acc_z_error = (acc_z_error - 16384);

  setpointX = static_bias_X;
  setpointY = static_bias_Y;

  pidX.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-150, 150);
  pidX.SetSampleTime(10);
  pidY.SetMode(AUTOMATIC);
  pidY.SetOutputLimits(-150, 150);
  pidY.SetSampleTime(10);
}

void loop() {
  // [멘토 수정] 시리얼 입력을 통한 실시간 튜닝 로직 추가
   if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase(); // 소문자 입력 허용

    // 1. X축 Kp만 수정 (예: KP_X_0.005)
    if (input.startsWith("KP_X_")) {
        double newKpX = input.substring(5).toDouble();
        if (newKpX >= 0) {
            Kp_X = newKpX;
            pidX.SetTunings(Kp_X, Ki_X, Kd_X); // X축 PID 객체에만 반영
            
            Serial.print(">>> [TUNING] Kp_X가 ");
            Serial.print(Kp_X, 6);
            Serial.println(" 으로 변경되었습니다.");
        }
    }
    // 2. Y축 Kp도 따로 바꿀 경우를 대비 (예: KP_Y_0.005)
    else if (input.startsWith("KP_Y_")) {
        double newKpY = input.substring(5).toDouble();
        if (newKpY >= 0) {
            Kp_Y = newKpY;
            pidY.SetTunings(Kp_Y, Ki_Y, Kd_Y); // Y축 PID 객체에만 반영
            
            Serial.print(">>> [TUNING] Kp_Y가 ");
            Serial.print(Kp_Y, 6);
            Serial.println(" 으로 변경되었습니다.");
        }
    }

    if (input.startsWith("KI_X_")) {
    Ki_X = input.substring(5).toDouble();
    pidX.SetTunings(Kp_X, Ki_X, Kd_X);
    Serial.printf(">>> [TUNING] Ki_X: %.6f\n", Ki_X);
    }
    else if (input.startsWith("KD_X_")) {
        Kd_X = input.substring(5).toDouble();
        pidX.SetTunings(Kp_X, Ki_X, Kd_X);
        Serial.printf(">>> [TUNING] Kd_X: %.6f\n", Kd_X);
    }

    if (input.length() > 2) {
      char axis = toupper(input.charAt(0));  // 첫 글자(축) 추출 후 대문자로 변환
      int commaIndex = input.indexOf(',');

      if (commaIndex > 0) {
        int newValue = input.substring(commaIndex + 1).toInt();  // 콤마 뒤의 숫자 추출

        if (axis == 'X') {
          HORIZ_X = constrain(newValue, LIMIT_X_MIN, LIMIT_X_MAX);  // 한계값 내로 제한
          Serial.printf(">>> [TUNING] HORIZ_X 값이 %d 로 변경되었습니다.\n", HORIZ_X);
        } else if (axis == 'Y') {
          HORIZ_Y = constrain(newValue, LIMIT_Y_MIN, LIMIT_Y_MAX);
          Serial.printf(">>> [TUNING] HORIZ_Y 값이 %d 로 변경되었습니다.\n", HORIZ_Y);
        } else {
          Serial.println(">>> [ERROR] 축 이름이 틀렸습니다. X 또는 Y를 입력하세요.");
        }
      }
    }
  }

  static unsigned long lastLoop = 0;
  if (micros() - lastLoop >= 10000) {  // 100Hz 루프 보장
    lastLoop = micros();

    int16_t ax_a, ay_a, az_a, gx_a, gy_a, gz_a;
    int16_t ax_f, ay_f, az_f, gx_f, gy_f, gz_f;
    readIMU_BMI(CS_ACTIVE, ax_a, ay_a, az_a, gx_a, gy_a, gz_a);
    delayMicroseconds(50);
    readIMU(CS_FIXED, ax_f, ay_f, az_f, gx_f, gy_f, gz_f);
    az_f = -az_f - (int16_t)acc_z_error;

    // -------------------------------------------------------------
    // [Phase 1: 상보 필터를 통한 '진짜 기울기' 산출]
    // -------------------------------------------------------------
    // 1-1. 자이로 데이터 오프셋 제거 및 dps(초당 회전각) 변환
    float rate_pitch = ((float)gx_f - gyro_bias_x) / GYRO_LSB;
    float rate_roll = ((float)gy_f - gyro_bias_y) / GYRO_LSB;

    // 1-2. 가속도계 기반의 거친 각도 (노이즈, 관성력 포함)
    float pitch_acc = atan2((float)-ay_f, sqrt(pow((float)ax_f, 2) + pow((float)az_f, 2))) * (180.0 / PI);
    float roll_acc = atan2((float)ax_f, (float)-az_f) * (180.0 / PI);

    // 1-3. 상보 필터 결합: 자이로의 민첩성 + 가속도의 영점 유지력
    pitch_filtered = alpha_cf * (pitch_filtered + rate_pitch * dt) + (1.0 - alpha_cf) * pitch_acc;
    roll_filtered = alpha_cf * (roll_filtered + rate_roll * dt) + (1.0 - alpha_cf) * roll_acc;

    // -------------------------------------------------------------
    // [Phase 2: 중력 보상 및 목표 각도 매핑]
    // -------------------------------------------------------------
    // 2-1. 진짜 기울기를 이용한 중력 성분 제거 -> 순수 선형 가속도 추출
    float a_linear_y = (float)ay_f - (G_LSB * sin(pitch_filtered * PI / 180.0));
    float a_linear_x = (float)ax_f - (G_LSB * sin(roll_filtered * PI / 180.0));

    // 2-2. 목표 합력 각도 산출X
    float raw_target_pitch = atan2(a_linear_y, G_LSB) * (180.0 / PI);
    float raw_target_roll = atan2(a_linear_x, G_LSB) * (180.0 / PI);

    // 2-3. 동적 Setpoint 충격 방지 필터 (기억과 출력의 분리)
    static float internal_pitch = 0, internal_roll = 0;

    // ① 필터 연산: 데드존의 방해 없이 무조건 과거 데이터를 누적하여 기억합니다.
    internal_pitch = (0.3 * raw_target_pitch) + (0.7 * internal_pitch);
    internal_roll = (0.1 * raw_target_roll) + (0.9 * internal_roll);

    // ② 출력 변수 생성: 누적된 값을 임시 변수에 복사합니다.
    float final_target_pitch = internal_pitch;
    float final_target_roll = internal_roll;

    // ③ 각도 데드존: 출력할 값이 2.0도보다 작으면 노이즈로 간주하고 0으로 묵살합니다.
    if (abs(final_target_pitch) < 2.0) final_target_pitch = 0.0;
    if (abs(final_target_roll) < 3.0) final_target_roll = 0.0;


    // 위에서 결정된 최종 타겟 각도(도 단위)를 ZV 필터에 통과시킵니다.
    float shaped_pitch = applyConvolvedZV(final_target_pitch, zv_buffer_y, zv_head_y);  // pitch는 기존 유지
    float shaped_roll = -roll_filtered;

    // -------------------------------------------------------------
    // [Phase 3: 상단 트레이 상보 필터 (가속도 채찍 효과 완벽 차단)]
    // -------------------------------------------------------------
    // 3-1. 상단 자이로 데이터 오프셋 제거 및 dps 변환
    float rate_pitch_a = ((float)gx_a - gyro_bias_x_a) / GYRO_LSB;
    float rate_roll_a = ((float)gy_a - gyro_bias_y_a) / GYRO_LSB;

    // 3-2. 상단 가속도계 기반 거친 각도 (접선 가속도로 인해 노이즈가 극심한 상태)
    float pitch_acc_a = atan2((float)ay_a, sqrt(pow((float)ax_a, 2) + pow((float)az_a, 2))) * (180.0 / PI);
    // 수정된 BMI160용 roll 수식 적용
    float roll_acc_a = atan2((float)-ax_a, sqrt(pow((float)ay_a, 2) + pow((float)az_a, 2))) * (180.0 / PI);

    // 3-3. 상보 필터 융합 (자이로 98% 신뢰, 가속도 2% 보정)
    tray_pitch_filtered = alpha_cf * (tray_pitch_filtered + rate_pitch_a * dt) + (1.0 - alpha_cf) * pitch_acc_a;
    tray_roll_filtered = alpha_cf * (tray_roll_filtered + rate_roll_a * dt) + (1.0 - alpha_cf) * roll_acc_a;

  
    // -------------------------------------------------------------
    // [Phase 4: 순수 각도 기반 제어기 직결]
    // -------------------------------------------------------------
    // 4-1. ZV 성형 목표값과 트레이의 실제 절대 각도를 LSB 단위로 스케일링하여 제어기 입력에 직결합니다.
    inputX = (double)(tray_roll_filtered * SCALE_FACTOR_X);
    inputY = (double)(tray_pitch_filtered * SCALE_FACTOR_Y);

    setpointX = static_bias_X + (shaped_roll * SCALE_FACTOR_X);  // roll만 + 로 변경
    setpointY = static_bias_Y - (shaped_pitch * SCALE_FACTOR_Y); // pitch는 기존 유지

    // 4-2. 디버깅 및 시각화를 위한 순수 각도 오차 산출
    float error_deg_x = tray_roll_filtered;
    float error_deg_y = shaped_pitch - tray_pitch_filtered;

    /*
    // --- X축 비선형 Gain Scheduling ---
    float abs_error_x = abs(error_deg_x); // error_deg_x는 이미 위에서 계산됨
    float ratio = constrain(abs_error_x / ERROR_MAX_DEG, 0.0, 1.0);
    double Kp_X_dynamic = Kp_X_MIN + (Kp_X_MAX - Kp_X_MIN) * ratio * ratio;
    pidX.SetTunings(Kp_X_dynamic, Ki_X, Kd_X);
    */
    
    // 제어기 연산 실행
    pidX.Compute();
    pidY.Compute();

    // loop() 함수 내부, pidX.Compute() 바로 다음에 넣으세요.
    static double filtered_outputX = 0; // 이전 값을 기억하기 위한 정적 변수
    float alpha = 0.1; // 0.0 ~ 1.0 사이값 (작을수록 더 부드러워지지만 반응은 느려짐)

    // LPF 연산
    filtered_outputX = (alpha * outputX) + (1.0 - alpha) * filtered_outputX;


      // 데드존을 Compute() 이후로 이동
    if (abs(error_deg_x) < 2.0) {
        outputX = 0;
    }


    int finalX = HORIZ_X + (int)filtered_outputX;
    int finalY = HORIZ_Y + (int)outputY;

    ledcWrite(MOTOR_X, constrain(finalX, LIMIT_X_MIN, LIMIT_X_MAX));
    ledcWrite(MOTOR_Y, constrain(finalY, LIMIT_Y_MIN, LIMIT_Y_MAX));

    // 시리얼 출력 로직 (물리적 각도 단위로 통일)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100) {
      Serial.print("base_pitch:"); Serial.print(pitch_filtered);       Serial.print(",");
      Serial.print("base_roll:");  Serial.print(roll_filtered);         Serial.print(",");
      Serial.print("tray_pitch:"); Serial.print(tray_pitch_filtered);   Serial.print(",");
      Serial.print("tray_roll:");  Serial.print(tray_roll_filtered);    Serial.print(",");
      Serial.print("target_roll:"); Serial.print(shaped_roll);          Serial.print(",");
      Serial.print("error_roll:");  Serial.println(error_deg_x);
      // 추가된 Kp_X 출력 (소수점 6자리까지)
      lastPrint = millis();
    }
  }
}