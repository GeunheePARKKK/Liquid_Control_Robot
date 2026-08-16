# Documents

제어 알고리즘, 실험 계획, 논문 및 설계 문서를 보관합니다.

## AI 협업 문서

세 명이 역할을 나누어 각자의 AI와 작업할 때 다음 순서로 자료를 제공합니다.

1. [공통 작업 규약](COMMON_CONTEXT.md)
2. 본인의 역할 문서
3. 역할 문서에 지정된 소스와 데이터시트

역할별 문서:

- [담당 1: GL60II 및 CAN 통신](MOTOR_CAN_TASK/README.md)
- [담당 2: 이중 IMU 및 데이터](SENSOR_DATA_TASK/SENSOR_DATA_TASK.md)
- [담당 3: 제어 알고리즘 및 통합](CONTROL_INTERGRATION_TASK/CONTROL_INTEGRATION_TASK.md)

각 담당자는 AI가 제안한 변경사항을 검토하고 승인한 뒤 자신의 브랜치에서 작업합니다. 핀, 센서 종류, 통신 설정과 안전 제한은 `Config.h/.cpp`에서만 정의하며, 다른 모듈은 이를 읽어서 사용합니다.
