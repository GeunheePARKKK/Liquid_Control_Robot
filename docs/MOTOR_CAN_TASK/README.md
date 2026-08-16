# 담당 1: GL60II 및 CAN 통신

## 목표

ESP32-S3의 TWAI 컨트롤러와 외부 CAN 트랜시버를 사용하여 GL60II 두 대에 Position/Velocity 명령을 보내고, 모터 피드백과 통신 상태를 상위 모듈에 제공합니다.

## AI에 제공할 자료

1. `docs/ai-handoff/COMMON_CONTEXT.md`
2. 이 문서
3. `firmware/gimbal_control/Gimbal.h`
4. `firmware/gimbal_control/Gimbal.cpp`
5. `firmware/gimbal_control/Config.h` — 읽기 전용
6. `Gimbal Motor Drive User Manual.pdf`
7. `ESP32-S3-DEVKITC-1-N16R8V_Espressif_Systems.pdf`

## 파일 소유권

- 수정 가능: `Gimbal.h`, `Gimbal.cpp`
- 읽기 전용: `Config.h/.cpp`
- 수정 금지: `Sensor.*`, `Control.*`, `gimbal_control.ino`

새 테스트 파일이나 공용 헤더가 필요하면 팀의 승인을 먼저 받습니다.

## 구현 책임

- Config에서 CAN 핀, 속도, 모터 ID와 타임아웃을 읽어 TWAI 초기화
- GL60II Position/Velocity 표준 11-bit CAN 프레임 생성
- 활성화, 비활성화, 현재 위치 영점, 오류 해제 명령 제공
- 목표 위치 rad와 최대속도 rad/s를 GL60II 형식으로 변환
- 피드백에서 모터 ID, 위치, 속도, 토크, 온도와 오류 코드 해석
- Config에 설정된 두 모터의 최신 피드백을 각각 보관
- 마지막 정상 피드백 시각과 통신 건강 상태 제공
- 잘못된 CAN ID, DLC, 범위 또는 오래된 피드백 거부

## 공개 인터페이스 목표

```cpp
class GimbalBus {
public:
    bool begin();
    bool enable(uint8_t motorId);
    bool disable(uint8_t motorId);
    bool clearError(uint8_t motorId);
    bool setCurrentPositionZero(uint8_t motorId);
    bool commandPositionVelocity(
        uint8_t motorId,
        float positionRad,
        float maxVelocityRadS);
    void poll();
    bool getFeedback(uint8_t motorId, MotorFeedback& out) const;
    bool healthy(uint8_t motorId, uint32_t nowMs) const;
};
```

`GimbalBus`는 IMU, 합력 계산, P와 ZV를 알지 못합니다. 설정값은 Config에서 읽고 프로토콜 계층에서 명령 범위를 검증합니다.

## 구현 순서

1. 매뉴얼에서 Position/Velocity 프레임과 공통 명령 확인
2. ID와 float 바이트 순서 인코딩·디코딩 함수 작성
3. TWAI 초기화와 송수신 처리
4. 한 모터 활성화와 무부하 소각도 명령
5. 두 모터 피드백 분리
6. 타임아웃과 오류 상태 제공

## 안전 및 금지사항

- `begin()`에서 모터를 자동 활성화하지 않습니다.
- 영점 설정을 부팅 시 자동 호출하지 않습니다.
- 피드백을 받지 못했을 때 임의의 값을 만들지 않습니다.
- MIT와 Velocity 모드는 현재 범위에 추가하지 않습니다.
- 통신 타임아웃을 감지하되 시스템 전체 정책은 통합 담당자에게 맡깁니다.
- 핀, ID, 제한값을 `Gimbal` 파일에 하드코딩하지 않습니다.

## 완료 기준

- 매뉴얼 예제와 생성된 CAN ID 및 데이터 바이트가 일치함
- 한 모터가 무부하 소각도 명령을 수행하고 피드백이 갱신됨
- Config로 지정된 두 모터의 피드백이 서로 섞이지 않음
- 잘못된 DLC와 알 수 없는 ID가 거부됨
- 타임아웃 후 `healthy()`가 false를 반환함

## AI 전달용 프롬프트

```text
COMMON_CONTEXT.md, MOTOR_CAN_TASK.md와 제공된 소스 및 데이터시트를 모두 읽어라.
너는 GL60II/TWAI 담당자이며 Gimbal.h와 Gimbal.cpp만 소유한다.
먼저 현재 상태, 데이터시트에서 확인한 프로토콜과 구현 계획을 설명하고 변경 승인을 요청하라.
승인 전에는 파일을 수정하지 마라.
Position/Velocity 모드만 구현하고 센서 및 P/ZV 제어를 추가하지 마라.
핀, CAN ID, 속도와 제한값은 Config에서 읽고 Gimbal 파일에 하드코딩하지 마라.
다른 담당자의 파일은 수정하지 마라.
작업 후 변경 API, 프레임 검증, 하드웨어 시험 절차와 남은 위험을 보고하라.
```
