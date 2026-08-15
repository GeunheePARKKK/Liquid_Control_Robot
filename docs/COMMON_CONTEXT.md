# Liquid Control Robot 공통 작업 규약

이 문서는 세 명의 팀원과 각 팀원이 사용하는 AI가 공유하는 프로젝트 기준입니다. 역할별 작업을 시작하기 전에 이 문서와 본인의 역할 문서를 순서대로 제공합니다.

## 프로젝트 목표

하부 IMU로 이동 베이스의 자세와 가속도를 측정하여 액체에 작용하는 합력 방향을 계산하고, 트레이가 합력 벡터에 수직하도록 목표 자세를 생성합니다. 상부 IMU는 실제 트레이 자세를 측정하며 외부 P 제어기는 추종 오차를 보정합니다. ZV 입력성형기는 이동 명령으로 발생하는 잔류 슬로싱을 줄입니다.

1D CNN은 제어기가 아니라 슬로싱 주파수 추정기입니다. 학습 데이터와 검증된 모델이 준비되기 전에는 고정 주파수 ZV를 사용합니다.

## 시스템 경계

```text
하부·상부 IMU
    ↓
Sensor: 센서 읽기와 물리 단위 변환
    ↓
Control: 합력 목표각 → P → ZV
    ↓
Gimbal: GL60II CAN 명령·피드백
    ↓
GL60II 내장 드라이버: 위치·속도·전류 제어
```

- MCU는 ESP32-S3-DevKitC-1-N16R8V를 사용합니다.
- 모터는 GL60II 두 대를 사용합니다.
- ESP32-S3의 TWAI와 외부 CAN 트랜시버로 모터 버스에 연결합니다.
- GL60II의 실제 모드와 CAN ID는 초기 설정 과정에서 확인한 뒤 `Config`에 반영합니다.
- 두 IMU의 모델과 핀 배치는 확정 후 `Config`에 반영합니다.

## Config 단일 설정 원칙

핀, 센서 종류, 통신 설정, 제어주기, 축 방향과 안전 제한은 `Config.h/.cpp`에서만 정의합니다. 다른 파일은 값을 직접 선언하거나 숫자로 하드코딩하지 않고 `Config.h`를 포함하여 사용합니다.

```cpp
#include "Config.h"
```

Config가 제공해야 할 설정 범주는 다음과 같습니다. 실제 이름과 자료형은 공용 인터페이스를 확정할 때 정합니다.

```text
CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE
SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO
BASE_IMU_CS_GPIO, TRAY_IMU_CS_GPIO
BASE_IMU_TYPE, TRAY_IMU_TYPE
MOTOR_X_CAN_ID, MOTOR_Y_CAN_ID, MASTER_CAN_ID
CONTROL_PERIOD_US
MOTOR_POSITION_LIMIT_RAD, MOTOR_VELOCITY_LIMIT_RAD_S
FEEDBACK_TIMEOUT_MS, SENSOR_TIMEOUT_US
MOTOR_X_DIRECTION, MOTOR_Y_DIRECTION
MOTOR_X_ZERO_OFFSET_RAD, MOTOR_Y_ZERO_OFFSET_RAD
```

- 모터·센서 담당자는 필요한 설정항목을 통합 담당자에게 제안합니다.
- `Config.h/.cpp`는 통합 담당자만 수정합니다.
- 설정값이 확정되지 않은 경우 담당 파일에 임시 숫자를 넣지 않고 미확정 사항으로 보고합니다.
- 데이터시트로 확인되는 통신 규격과 실험으로 정해야 하는 배선·축 설정을 구분합니다.

## 파일 소유권

| 담당 | 수정 가능 파일 | 책임 |
|---|---|---|
| 모터·CAN | `Gimbal.h/.cpp` | TWAI, GL60II 명령·피드백 |
| 센서·데이터 | `Sensor.h/.cpp`, 승인된 `data/` 파일 | 이중 IMU, 물리 단위 데이터 |
| 제어·통합 | `Control.h/.cpp`, `Config.h/.cpp`, `gimbal_control.ino` | 목표각, P, ZV, 상태기계 |

- 담당자는 자신의 전담 파일만 수정합니다.
- 공용 자료형이나 공개 함수 변경은 세 담당자가 합의한 뒤 별도 변경으로 처리합니다.
- 다른 모듈의 내부 변수에 직접 접근하지 않고 공개 함수만 사용합니다.
- 새 파일 생성, 공용 파일 변경, Git 작업은 팀의 변경 절차에 따라 별도로 합의합니다.

## 공용 데이터 계약

다음은 모듈 사이에서 교환해야 하는 최소 정보입니다. 실제 공용 헤더를 만들기 전까지 역할별 AI가 서로 다른 단위나 중복 자료형을 임의로 만들지 않습니다.

```cpp
struct MotorFeedback {
    float positionRad;
    float velocityRadS;
    float torqueNm;
    int8_t driverTemperatureC;
    int8_t motorTemperatureC;
    uint8_t errorCode;
    uint32_t receivedAtMs;
    bool valid;
};

struct ImuSample {
    float axMps2;
    float ayMps2;
    float azMps2;
    float gxRadS;
    float gyRadS;
    float gzRadS;
    uint32_t sampledAtUs;
    bool valid;
};

struct ImuPair {
    ImuSample base;
    ImuSample tray;
};

struct ControlOutput {
    float xPositionRad;
    float yPositionRad;
    float maxVelocityRadS;
    bool enable;
};
```

내부 각도는 rad, 각속도는 rad/s, 가속도는 m/s², 토크는 N·m를 사용합니다. 사람이 읽는 출력에서 degree를 사용할 때는 필드명에 단위를 명시합니다.

## 안전 원칙

- 부팅 직후 모터를 자동 활성화하거나 자동 영점 설정하지 않습니다.
- 초기 모터 시험은 무부하 소각도에서 수행합니다.
- 위치·속도·제어 출력은 Config의 안전 제한을 적용합니다.
- 모터 피드백이나 IMU 값이 유효하지 않으면 신규 이동 명령을 생성하지 않습니다.
- 통신 모듈은 상태를 보고하고, 전체 비활성화 정책은 통합 상태기계가 결정합니다.
- 센서 축과 모터 방향은 추측하지 않고 축별 시험 후 Config에 기록합니다.
- 데이터시트에 없는 프로토콜, 레지스터, 핀 특성을 추측하지 않습니다.

## 개발 순서

1. GL60II 한 대 CAN 통신
2. GL60II 두 대 독립 통신
3. 이중 IMU 단독·동시 읽기
4. 합력 방향과 목표각 생성
5. 외부 P 제어
6. 고정 주파수 ZV
7. 데이터 수집과 1D CNN 검증
8. 검증된 CNN 주파수로 ZV 갱신

이번 구현에서 ESO, LQR, MPC는 제외합니다.

## AI 작업 공통 지시

- 제공된 자료를 먼저 읽고 현재 상태와 불확실한 부분을 보고합니다.
- 변경 전에 대상 파일, 변경 내용, 이유와 영향을 설명하고 팀원의 승인을 받습니다.
- 맡지 않은 파일은 수정하지 않습니다.
- Config에 있어야 할 값을 담당 파일에 하드코딩하지 않습니다.
- 컴파일·시험·Git 작업은 팀원의 승인을 받은 범위에서만 수행합니다.
- 종료 시 변경 파일, 검증 결과, 실제 장비에서 확인할 항목을 보고합니다.
