# 담당 3: 제어 알고리즘 및 통합

## 목표

하부 IMU에서 액체에 작용하는 합력 방향을 계산하고, 상부 IMU의 트레이 자세 피드백을 이용해 GL60II 목표 위치를 생성합니다. P 제어와 고정 주파수 ZV를 단계적으로 활성화하며 전체 시스템의 설정과 안전 상태를 관리합니다.

## AI에 제공할 자료

1. `docs/ai-handoff/COMMON_CONTEXT.md`
2. 이 문서
3. `firmware/gimbal_control/Control.h`
4. `firmware/gimbal_control/Control.cpp`
5. `firmware/gimbal_control/Config.h`
6. `firmware/gimbal_control/Config.cpp`
7. `firmware/gimbal_control/gimbal_control.ino`
8. 프로젝트의 논문 또는 합력 방향·ZV 알고리즘 설명 자료
9. 병합된 `Gimbal.h`와 `Sensor.h` — 공개 인터페이스 확인용

## 파일 소유권

- 수정 가능: `Control.h/.cpp`, `Config.h/.cpp`, `gimbal_control.ino`
- 읽기 전용: `Gimbal.*`, `Sensor.*`
- 새 공용 타입 또는 데이터 파일이 필요하면 팀의 승인을 받음

## Config 책임

이 담당자만 `Config.h/.cpp`를 수정합니다. 모터·센서 담당자가 요청한 설정을 검토하고 다음 범주를 한곳에서 관리합니다.

- CAN·SPI·CS 핀
- CAN bitrate와 모터 ID
- 상부·하부 IMU 종류
- 제어주기와 통신·센서 타임아웃
- 모터 위치·속도 제한
- 축 방향과 기구 영점
- P 게인, ZV 기본 주파수와 안전 범위

다른 모듈에서 요구하는 설정이 추가되면 공개 설정 이름과 단위를 먼저 합의합니다.

## 제어 데이터 흐름

```text
하부 IMU
  → 베이스 자세 추정
  → 중력 성분과 선형가속도 분리
  → 합력 방향과 트레이 목표 roll/pitch
  → 저역통과·데드존·각도 제한
  → ZV 입력성형
  → 상부 IMU 실제 트레이 자세와 비교
  → 외부 P 보정
  → Config의 축 방향·영점·제한 적용
  → GL60II Position/Velocity 목표
```

ZV는 모터에 보내기 전 목표각에만 적용합니다. 모터 피드백이나 P 보정 출력을 다시 ZV 입력으로 사용하지 않습니다.

## 구현 책임

- Config 기반 하드웨어·제어 설정 제공
- 실제 경과시간을 사용하는 주기 제어
- 하부 IMU 기반 합력 방향과 목표각 계산
- 목표각 저역통과, 데드존과 물리 범위 제한
- 상부 IMU 자세에 대한 외부 P 보정
- 고정 주파수 Convolved ZV와 버퍼 초기화
- 시작, 대기, 활성, 고장 상태기계
- 센서와 모터가 정상일 때만 모터 명령 활성화
- 시리얼 로그에 센서, 목표각, 성형각, 모터 명령·피드백과 상태 기록
- 추후 CNN이 주파수와 신뢰도를 전달할 입력 인터페이스 제공
- CNN 값이 유효하지 않으면 Config의 고정 ZV 값으로 복귀

## 공개 인터페이스 목표

```cpp
bool initControl();
ControlOutput runControlStep(
    const ImuPair& imu,
    const MotorFeedback& xMotor,
    const MotorFeedback& yMotor,
    float dtSeconds);

void resetZV();
bool setEstimatedSloshFrequency(
    float xFrequencyHz,
    float yFrequencyHz,
    float confidence);
```

실제 CNN 모델과 검증 데이터가 없으면 입력 검증과 고정값 폴백까지만 구현합니다.

## 상태기계

```text
BOOT → SENSOR_CHECK → MOTOR_CHECK → READY → ACTIVE
                    ↘ FAULT ←───────────────┘
```

- `BOOT`: 모터 비활성, 버퍼 초기화
- `SENSOR_CHECK`: 두 IMU 유효성 확인
- `MOTOR_CHECK`: 두 모터 피드백과 오류 확인
- `READY`: 사용자의 시작 명령 대기
- `ACTIVE`: 주기적으로 제어 명령 전송
- `FAULT`: 신규 이동 명령 중단, 가능한 경우 비활성화, 오류 출력

자동으로 `READY`에서 `ACTIVE`로 전환하지 않습니다.

## 구현 순서

1. Config 설정 인터페이스와 공용 단위 확정
2. 상태기계와 안전 제한
3. 가짜 센서 입력으로 합력 목표각 검증
4. ZV 없이 목표각 직접 명령
5. 상부 IMU 외부 P 보정
6. 고정 주파수 ZV 추가
7. 로깅과 P/P+ZV 비교 실험
8. 검증된 주파수 입력 연결

## 안전 및 금지사항

- CAN 바이트를 Control 또는 `.ino`에서 직접 만들지 않습니다.
- IMU 레지스터를 Control 또는 `.ino`에서 직접 읽지 않습니다.
- 부팅 시 모터 활성화나 영점 설정을 자동 실행하지 않습니다.
- 한 번에 P, ZV와 CNN을 모두 활성화하지 않습니다.
- 주기 누락, 센서 오류, 모터 오류 또는 타임아웃 시 정상 명령을 계속 보내지 않습니다.
- 이번 단계에 ESO, LQR과 MPC를 추가하지 않습니다.

## 완료 기준

- 가짜 입력으로 합력 방향과 목표각의 축·부호 검증 가능
- 센서 또는 모터가 유효하지 않으면 ACTIVE에 진입하지 않음
- P 단독으로 트레이가 목표각에 안정적으로 수렴함
- P 단독과 P+ZV 실험을 같은 조건과 로그로 비교 가능
- ZV 초기화와 주파수 변경 시 불연속 명령을 방지함
- 제어주기와 통신 상태가 로그에 기록됨

## AI 전달용 프롬프트

```text
COMMON_CONTEXT.md, CONTROL_INTEGRATION_TASK.md와 제공된 제어 소스 및 알고리즘 자료를 모두 읽어라.
너는 제어 및 통합 담당자이며 Control.*, Config.*와 gimbal_control.ino만 소유한다.
먼저 알고리즘의 데이터 흐름, 단위, 축 방향과 안전 상태를 정리하고 변경 계획과 이유를 설명한 뒤 승인을 요청하라.
승인 전에는 파일을 수정하지 마라.
모든 핀, 센서 종류, ID, 제한값과 제어 설정은 Config에서 관리하라.
Gimbal과 Sensor의 공개 인터페이스만 사용하고 해당 담당 파일을 수정하지 마라.
P를 먼저 검증하고 ZV를 별도로 활성화하라. 실제 모델이 없으면 CNN을 구현한 것처럼 꾸미지 마라.
ESO, LQR, MPC는 추가하지 마라.
작업 후 제어식, 단위, 안전 동작, 시험 결과와 장비 검증 항목을 보고하라.
```
