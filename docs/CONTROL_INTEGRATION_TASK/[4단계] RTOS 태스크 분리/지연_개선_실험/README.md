# 4단계 합력 직접보상 지연 개선 실험

이 폴더는 `zv_shaping_rtos.ino`의 **합력 목표 생성부터 모터 엔코더 응답까지**
구간별 지연을 분리하고, 필터·CAN 주기·모터 게인이 응답성과 안정성에 미치는 영향을
실기에서 비교한 자료다.

현재 실물 시험의 목표는 ZV 성능 검증이 아니다. 단일 베이스 IMU가 계산한 합력 목표를
모터가 가능한 한 빠르고 안정적으로 추종하게 만드는 **직접보상 경로**를 먼저 확정한다.
ZV는 전 시험에서 껐다.

상세 수치와 판단은 [`결과_분석.md`](%EA%B2%B0%EA%B3%BC_%EB%B6%84%EC%84%9D.md)에 있다.

## 폴더 구성

| 경로 | 내용 |
|---|---|
| `tools/capture.py` | ESP32-S3 시리얼을 arm 전후로 캡처한다 |
| `tools/check_tuning.py` | 드룹·슬루·지연·추종률·정지 떨림을 계산한다 |
| `logs/` | 이 문서의 판단에 사용한 원본 로그 15개 |
| `requirements.txt` | PC 분석 도구 의존성 |

`capture.py`와 `check_tuning.py`는 실험 당시 사용한 버전을 그대로 보존했다.

## 실험 환경

```text
상자 바닥               LSM6DSOX IMU 1개
상자 위                 2축 GL60II 짐벌
안쪽축 PITCH            CAN 0x02
바깥축 ROLL             CAN 0x01, 안쪽 짐벌과 트레이 전체를 함께 회전
제어 태스크             100 Hz
로그 스트림             약 33 Hz
```

이 시험에는 트레이 IMU가 없다. `act_pitch/act_roll`은 모터 엔코더 관절각이지 트레이
절대각이 아니다. 강체·영점·유격이 이상적이라는 가정 아래에서만 트레이 움직임의
대용값으로 쓴다.

또한 앞뒤 방향은 바퀴가 굴러가지만 좌우 방향은 상자를 바닥에서 끈다. 좌우 시험에는
정지마찰·stick-slip·상자 회전·바닥 충격이 섞일 수 있으므로 PITCH와 ROLL의 절대 성능을
그대로 비교하지 않는다. 같은 방향, 비슷한 입력 대역의 로그끼리만 A/B 비교한다.

## 로그 신호

현행 펌웨어는 다음 필드를 출력한다.

```text
t
raw_pitch/raw_roll
pre_pitch/pre_roll
pitch/roll
ref_pitch/ref_roll
zv_pitch/zv_roll
cmd_pitch/cmd_roll
act_pitch/act_roll
```

신호 사슬은 다음처럼 해석한다.

```text
물리 가속
  → raw
  → ACC_LPF
  → ref
  → ZV
  → want = ACC_GAIN·zv − GAIN·base_angle
  → CMD_LPF
  → pre
  → slew
  → cmd
  → CAN/MIT 모터
  → act
```

`want`는 로그에 직접 없으며 분석기가 `zv`, 베이스 자세, `GAIN`, `ACC_GAIN`, 각도
제한으로 재구성한다. 재구성한 `want`에 동일한 `CMD_LPF`를 적용하면 `pre`가 약 1%
오차로 재현되는 것을 확인했다.

## PC 준비

Python 3 환경에서:

```powershell
python -m pip install -r requirements.txt
```

시리얼 모니터와 `capture.py`는 같은 포트를 동시에 열 수 없다. 캡처 전에 Arduino IDE
시리얼 모니터를 닫는다.

## 안전 절차

1. 펌웨어 업로드 때 24 V 모터 전원을 끈다.
2. ESP32 부팅 뒤 24 V를 켠다.
3. 트레이가 중립이고 걸리는 물체가 없는지 확인한다.
4. 24 V 차단 스위치에 손을 둔 상태에서 `arm`한다.
5. 발진·충격·이상음이 나면 즉시 전원을 차단한다.
6. 이 시험에는 물을 올리지 않는다.
7. `t` 시험구동은 보호 로직이 꺼지는 구간이 있으므로 사용하지 않는다.

포트를 열면 ESP32가 리셋되어 이전 시리얼 설정이 모두 사라진다. 비교에 필요한 설정은
반드시 `capture.py --cmd`에 전부 적는다.

## 표준 수동 입력

완전한 정현파 입력 장치를 사용할 수 없어 다음 절차로 반복성을 높였다.

1. 메트로놈을 60 BPM으로 켠다.
2. 네 박자를 한 동작 구간으로 사용한다.
3. 두 손으로 상자 중심을 잡아 앞뒤로 밀고 정지한다.
4. 같은 방식으로 좌우로 밀고 정지한다.
5. 회전이 섞이지 않도록 양손 힘을 대칭으로 준다.
6. 모든 후보에서 같은 박자·거리·순서로 26초를 기록한다.

수동 입력은 완전히 같지 않다. 분석 결과의 `흔든 대역`이 약 0.15~0.20 Hz 이상 다르면
시간지연을 직접 비교하지 않는다. 동일 설정도 최소 2회, 기준 설정은 3회 반복한다.

## 캡처

현행 잠정 설정의 예다.

```powershell
cd tools

python capture.py --port COM8 `
  --cmd "zv0,vf1,ad0.20,f0.30,r120,d1,kpp2.0,kdp0.13,kpr2.0,kdr0.40,arm" `
  --pre 6 --post 26 --out run_01.txt
```

핵심 설정:

| 명령 | 의미 |
|---|---|
| `zv0` | ZV 끔. 직접보상 경로만 시험 |
| `vf1` | MIT `v_des` 속도 피드포워드 켬 |
| `ad0.20` | ACC 목표각 EMA의 현재값 비율 20% |
| `f0.30` | 최종 명령 EMA의 현재값 비율 30% |
| `r120` | 슬루 상한 120 deg/s |
| `d1` | CAN 명령 100 Hz |
| `kpp/kdp` | 안쪽 PITCH 게인 |
| `kpr/kdr` | 바깥 ROLL 게인 |

## 분석

한 파일:

```powershell
python check_tuning.py ..\logs\CAN100_01.txt
```

여러 파일 비교:

```powershell
python check_tuning.py `
  ..\logs\baseline_01.txt `
  ..\logs\baseline_02.txt `
  ..\logs\baseline_03.txt `
  ..\logs\CAN100_01.txt `
  ..\logs\CAN100_02.txt
```

중요 지표:

| 지표 | 해석 |
|---|---|
| `cmd→act` | CAN·모터·기구의 동적 응답 |
| `want→act − cmd→act` | CMD_LPF 기여분의 근사 |
| `raw→act − want→act` | ACC 전처리 기여분의 근사 |
| 추종률 | `std(act)/std(cmd)`. 1 초과는 동적 증폭 |
| 외란 몫 | 모터 명령으로 설명되지 않는 출력 분산 |
| 떨림 | 정지 구간 엔코더의 고주파 표준편차 |
| 흔든 대역 | 입력 스펙트럼 누적파워 50% 지점 |

`check_tuning.py`의 지연은 광대역 수동 입력의 상호상관 최대점이다. 고정된 물리적
전달지연이 아니며 입력 주파수에 따라 달라진다. 출력되는 `슬로싱 위상`도 계산 주기
456 ms에 지연을 단순 환산한 참고값일 뿐, 실제 액체 위상이나 ZV 효과의 측정값이 아니다.

## 현재 잠정값

```text
zv0, vf1, ad0.20, f0.30, r120, d1
kpp2.0, kdp0.13, kpr2.0, kdr0.40
```

이 값은 펌웨어 기본값으로 확정한 것이 아니다. 캡처 명령으로 적용한 현재 최선 후보다.
ROLL은 여전히 `cmd→act` 약 85 ms, 추종률 약 1.4라 추가 검증이 필요하다.

