/*
 * 합력벡터_제어.ino
 * -----------------------------------------------------------------------------
 * 3단계 — ZV 입력성형 (ZV 입력성형은 아직 없음)
 *
 * 1단계 [모터_수평제어_I2C.ino](../[1단계]%20수평제어/모터_수평제어_I2C.ino) 에서
 * 목표각 θ_ref 만 0 이 아니게 바뀐 것이다. 나머지는 전부 같다.
 *
 *   1단계 :  θ_ref = 0                    →  motor_cmd = −θ_base   (수평 유지)
 *   2단계 :  θ_ref = atan(a_linear / g)   →  motor_cmd = θ_ref − θ_base
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 왜 수평이 정답이 아닌가
 *
 * 가속하는 차 안의 물컵을 생각하면 된다. 컵을 수평으로 붙잡고 있으면 물은
 * 뒤로 쏠린다. 액체 입장에서는 중력과 관성력의 **합력**이 진짜 아래쪽이므로,
 * 컵 바닥을 그 합력에 수직으로 놓아야 수면이 컵 기준으로 평평해진다.
 *
 *     정지·등속  :  합력 = 중력        →  θ_ref = 0      (수평이 정답)
 *     가속 중    :  합력 = 중력+관성력  →  θ_ref ≠ 0      (기울여야 정답)
 *
 * 그래서 수평제어와 안티슬로싱이 별개의 모드가 아니다. 아래 한 식이 두 경우를
 * 모두 덮는다. 경사면에 서 있으면 a_linear 가 0 이라 θ_ref 도 0 이 되어
 * 자동으로 절대 수평이 된다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 계산 순서
 *
 *   1. 상보필터로 차체 절대 자세 θ_base 를 얻는다            (1단계와 동일)
 *   2. 측정 가속도에서 중력 성분을 뺀다
 *
 *        a_lin_y = ay − g·sin(pitch)
 *        a_lin_x = ax − g·sin(roll)
 *
 *      가속도계는 기울어져 있기만 해도 중력을 읽는다. 그 몫을 빼야 순수한
 *      선형 가속만 남는다. 뺄 때 쓰는 각도가 1단계에서 만든 θ_base 다.
 *
 *   3. 합력이 수직에서 벗어난 각도
 *
 *        θ_ref = atan(a_lin / g)
 *
 *   4. 모터는 차체 기준으로 도므로 차체 자세를 뺀다
 *
 *        motor_cmd = θ_ref − θ_base
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * ⚠ 켠 채로 부팅한다
 *
 * 부호를 실측으로 확정했으므로 (2026-08-29, DIR_ACC = +1) GAIN 1.0 / ACC_GAIN 1.0
 * 으로 부팅한다. arm 하는 순간 수평제어와 합력 항이 함께 살아 있다.
 *
 * 되돌릴 때는 ag0 → g0.3 → stop 순서다. 합력 항만 끄면 1단계와 같아진다.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 배선·모터는 1단계와 동일
 *   IMU  : LSM6DSOX, I2C 0x6A (CS→3V3, SDO→GND), SDA=11 SCL=12
 *   모터 : GL60II ×2, MIT 모드, CAN TX=4 / RX=5
 *          바깥축 0x01 (Master 0x00) / 안쪽축 0x02 (Master 0x11)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * 시리얼 명령 (115200)  —  ★ 부팅 기본값
 *
 *   동작
 *     arm            모터 활성화 (현재 위치를 영점으로 잡고 켠다)
 *     stop  / s      비활성화
 *     z              모터 영점 재설정 — 지금 트레이 자세를 0 으로
 *     t              시험 구동 8초 (IMU·제어식을 거치지 않는 CAN 경로 확인)
 *     q              스트림 출력 켜기/끄기
 *     ?              현재 상태 전부 출력
 *
 *   제어 게인                        기본값      범위        올릴 때
 *     g<값>   수평 게인              ★ 1.00     0 ~ 1.5     1.0 = 완전 수평
 *     ag<값>  합력 게인              ★ 1.00     0 ~ 1.5     ag0 이면 1단계와 동일
 *     ad<값>  3축 합력 벡터 필터     ★ 0.20     0 ~ 1       각도가 아니라 벡터를 거른다
 *
 *   지령 응답                        기본값      범위        올릴 때
 *     r<값>   슬루 제한 [°/s]        ★ 120      5 ~ 400     실측 0% 클리핑. 병목 아님
 *     f<값>   지령 저역통과          ★ 0.30     0 ~ 1       실측 확정. 0.35 부터 3.85Hz 진동
 *     d<값>   CAN 분주               ★ 1        1 ~ 20      1 = 100Hz 전송. 실측 확정
 *
 *   ZV 입력성형 (3단계)              기본값      범위        비고
 *     zv0 / zv1  끄기 / 켜기          ★ 켜짐                 zv0 이면 2단계와 동일
 *     zf<값>  슬로싱 주파수 [Hz]      ★ 2.40     0.8 ~ 8     ★ 실측해서 넣을 것
 *     zd<값>  감쇠비 ζ                ★ 0.02     0 ~ 0.9     액체는 보통 0.01~0.05
 *     zm<값>  임펄스 수               ★ 2        2 또는 3    2=ZV, 3=ZVD
 *
 *   가속 종료 소프트 복귀            기본값      범위        비고
 *     sr0 / sr1  끄기 / 켜기          ★ 꺼짐                 켜면 ZV 가 자동으로 꺼진다
 *     sra<값> 가속 인정 문턱 [°]      ★ 1.50     0.2 ~ 10    sre 보다 커야 한다
 *     sre<값> 종료 판정 문턱 [°]      ★ 0.75     0.05 ~      sra 보다 작아야 한다
 *     srd<값> 종료 확인 시간 [ms]     ★ 30       10 ~ 200    잠깐 흔들린 것을 걸러낸다
 *     srh<값> 유지 시간 [ms]          ★ 100      10 ~ 500    ★ 기울어진 채 머무는 시간
 *     srr<값> 복귀 시간 [ms]          ★ 120      20 ~ 1000   최소저크로 0 까지
 *
 *       세 시간이 순서대로 이어진다.
 *
 *         가속 중 ── raw_ref < sre ──> srd 확인 ──> srh 유지 ──> srr 복귀 ──> 수평
 *
 *       문턱을 둘로 나눈 것은 히스테리시스다. 하나면 문턱 근처에서 켜졌다
 *       꺼졌다 한다. 판정은 ACC_LPF 를 걸기 전의 raw_ref 로 하고, 유지할
 *       값은 지금 실제로 나가는 ref 를 잡는다 — 그래서 진입에 계단이 없다.
 *
 *       슬로싱 주기(11x11 용기 350mL 에서 456ms) 대비 srh+srr 을 얼마로
 *       둘지는 물을 올려 재야 한다. 기본값은 실측 전 출발점이다.
 *
 *   모터                             기본값      범위        올릴 때
 *     kp<값>  강성 — 두 축 함께      ★ 2.0      0 ~ 500     3 은 Kd 0 에서 발산했다
 *     kd<값>  감쇠 — 두 축 함께      ★ 0.13     0 ~ 5       0 은 매뉴얼상 금지
 *     kpp/kpr 강성 — 안쪽/바깥 따로  ★ 4.0/2.0              안쪽 6 은 물이 요동. 4 가 상한
 *     kdp/kdr 감쇠 — 안쪽/바깥 따로  ★ 0.20/0.13            예) kdr0.22
 *
 *       바깥축이 안쪽보다 81ms 느리다 (189 vs 108ms, 2026-09-01 실측).
 *       관성이 커서 같은 Kp 로는 못 따라온다. kpr 을 올려 지연을 맞출 것.
 *       Kp 를 올리면 Kd 도 sqrt 비율로 같이 올려야 발진하지 않는다.
 *     dp / dr 회전 방향 뒤집기       ★ −1 / +1  토글        안쪽 / 바깥
 *
 *   코드에만 있는 값 (명령 없음)
 *     DIR_ACC        ★ +1     합력 목표각 부호. 실측 확정
 *     LIMIT_DEG      ★ 25°    지령 각도 상한
 *     ACC_REF_LIMIT  ★ 20°    합력 목표각 자체의 상한
 *     ACT_LIMIT_DEG  ★ 50°    실제 위치가 넘으면 폭주로 보고 끈다
 *
 * 값을 바꿔도 저장되지 않는다. 전원을 껐다 켜면 위 기본값으로 돌아온다.
 * 좋은 값을 찾으면 알려줄 것 — 코드의 기본값으로 박는다.
 *
 * 되돌릴 때 순서:  ag0  →  g0.3  →  stop
 *   수평제어는 살려둔 채 합력 항만 끄는 것이 먼저다.
 * -----------------------------------------------------------------------------
 */

#include <Wire.h>
#include "driver/twai.h"

// =============================================================================
//  설정
// =============================================================================

// ── IMU (I2C) ──
#define PIN_SDA    11
#define PIN_SCL    12
#define I2C_HZ     400000
#define IMU_ADDR   0x6A      // SDO→GND. 3V3 이면 0x6B

// ── CAN ──
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5

/* 제어모드 접두.  CAN ID = (모드 << 8) | 모터ID
 *   0x000 = MIT,  0x100 = 위치/속도,  0x200 = 속도
 */
#define CAN_MODE  0x000

#define CAN_ID_PITCH  0x02
#define CAN_ID_ROLL   0x01

/* 모터 회전 방향 부호. CAN 으로 나갈 때만 곱해진다 (sendBoth 참고).
 * 제어식에는 들어가지 않으므로 cmd_pitch / cmd_roll 은 이 값과 무관하게
 * 항상 물리적으로 맞는 값이다. +30° 기울면 −30° 가 찍힌다.
 *
 * ★ 두 값이 다른 것이 정상이다. 짐벌의 두 모터가 서로 마주보게 장착돼 있어
 *   같은 부호를 주면 한 축이 기울기를 키우는 쪽으로 돈다. 카단 구조에서는
 *   흔한 배치다. 2026-08-29 실측 확인 (dp 명령으로 뒤집어 확정).
 *
 * 이 부호가 흔들렸던 이유는 부호 자체가 아니라 배정이었다.
 *
 *     모터_수평제어.ino       CAN_ID_PITCH 0x01 (바깥축)   DIR +1  → 정상
 *     모터_수평제어_MIT.ino   CAN_ID_PITCH 0x02 (안쪽축)   DIR +1  → 반대
 *
 *   MIT 모드로 포팅하면서 0x01 과 0x02 가 뒤바뀌었다. 모터 장착 방향이
 *   서로 반대라 같은 +1 인데 물리 방향이 뒤집혔다. 배정을 되돌리는 대신
 *   현재 배정에 맞춰 부호를 잡았다.
 *
 * ⚠ 배정 자체는 아직 잠정이다 (README 의 "미확정" 절). IMU 를 손에 들고
 *   있어 짐벌 축과 고정된 관계가 없다. 차체에 실장한 뒤 dp / dr 로 확정할 것.
 */
volatile float DIR_PITCH = -1.0f;    // [dp] 기본 −1   안쪽축 0x02
volatile float DIR_ROLL  = +1.0f;    // [dr] 기본 +1   바깥축 0x01

// ── 제어 파라미터 ──
volatile float GAIN            = 1.0f;    // [g]  기본 1.00   완전 수평 보상
const float LIMIT_DEG = 25.0f;   //      기본 25°    지령 각도 상한 (명령 없음)
volatile float MAX_RATE        = 120.0f;  // [r]  실측 확정 120  요구 변화율 p95 가 109~112
                                 //      30 이면 지령의 31% 가 잘려 ZV 성형이 무너진다
volatile float CMD_LPF         = 0.30f;   // [f]  실측 확정 0.30  지연 40→15ms, 상관 0.99 유지 (2026-09-04)
volatile int   CAN_DIV         = 1;       // [d]  실측 확정 1     100Hz 전송. 모터 지연 75→60ms (2026-09-04)

/* 모터 임피던스 게인 — 실측값 (2026-08-27)
 *   Kp 1     트레이를 잡는다. Kp 3 은 Kd 0 에서 발산했다.
 *   Kd 0.1   밀었다 놓으면 왕복 없이 복귀. 0.05 는 1회 오버슛.
 *
 * ⚠ MIT 모드에는 속도 제한이 없다. 목표가 튀면 Kp × 오차 가 즉시 토크로 나간다.
 *   Kp 20 에 0.5 rad 오차면 10 N·m 로 TMAX 포화다. 실제로 기구가 튕긴 적이 있다.
 * ⚠ Kd 가 0 이 아니면 미세 진동이 생긴다. 엔코더 속도 추정 노이즈에 곱해지는
 *   것이라 원리적으로 못 없앤다.
 */
/* ── 축별로 나눈 이유 (2026-09-01 실측) ──────────────────────────────────
 * 지령→실제 지연을 상호상관으로 쟀더니 두 축이 크게 달랐다.
 *
 *     안쪽 (PITCH, 0x02)   108 ms   corr +0.961
 *     바깥 (ROLL,  0x01)   189 ms   corr +0.925
 *
 * 지연을 보정하면 둘 다 지령을 잘 따라간다. 부호도 배선도 정상이고 바깥축이
 * 느릴 뿐이다. 그런데 지연 0 에서의 상관은 안쪽 +0.681 / 바깥 +0.192 다.
 * 지금 이 순간의 트레이 자세가 지금 필요한 자세와 거의 무관하다는 뜻이라,
 * 눈으로 보면 바깥축이 엉뚱한 방향으로 도는 것처럼 보인다.
 *
 * 189 ms 는 슬로싱 주기 420 ms 의 45% 로 거의 역위상이다. 이대로 액체를
 * 올리면 바깥축은 슬로싱을 억제하지 않고 키운다.
 *
 * 힘이 모자란 것이 아니다. 추종률이 0.94 로 크기는 충분히 간다. 대역폭이
 * 문제고, 원인은 바깥축이 안쪽 짐벌과 트레이를 통째로 이고 있어 관성이
 * 크기 때문이다.  ω ≈ √(Kp / J)  이므로 같은 Kp 로는 느릴 수밖에 없다.
 *
 * 189 → 108 ms 로 줄이려면 ω 를 1.75배, 즉 Kp 를 3배(2.0 → 6.0) 로 올려야
 * 한다. 그러면 ζ = Kd / 2√(Kp·J) 가 √3 배 떨어지므로 Kd 도 1.7배
 * (0.13 → 0.22) 를 같이 올린다. Kd 를 안 올리면 예전 바깥축에서 났던
 * 1.75Hz 한계진동이 돌아온다.
 *
 * ★ 기본값은 두 축 같게 두었다. 계산값은 출발점일 뿐이라 kpr/kdr 로
 *   올려가며 지연을 다시 재서 확정할 것. 바깥축은 발진 이력이 있는 축이다.
 *
 * ── 원래 주석 (2026-08-27 실측) ──
 *   Kp 1     트레이를 잡는다. Kp 3 은 Kd 0 에서 발산했다.
 *   Kd 0.1   밀었다 놓으면 왕복 없이 복귀. 0.05 는 1회 오버슛.
 *   Kp 2.0   드룹 54~82% -> 90%
 *   Kd 0.13  Kd ~ sqrt(Kp+k), k=0.28. 떨림 0.010~0.018도로 여유 있음
 */
volatile float KP_PITCH = 4.0f;    // [kpp] 안쪽 0x02  실측 확정 4.0. 6 은 토크 계단으로 물이 요동 (2026-09-04)
volatile float KD_PITCH = 0.20f;   // [kdp]           실측 확정 0.20. 더 올리면 엔코더 잡음이 토크로
volatile float KP_ROLL  = 2.0f;    // [kpr] 바깥 0x01 — 올려야 할 쪽
volatile float KD_ROLL  = 0.13f;   // [kdr]

const float TORQUE_WARN = 3.0f;

/* ── 합력 벡터 (2단계) ────────────────────────────────────────────────────
 * ACC_GAIN 0 이면 θ_ref 가 항상 0 이라 1단계(수평 유지)와 완전히 같다.
 * 부호를 확인한 뒤 0.3 → 0.6 → 1.0 으로 올린다.
 *
 * DIR_ACC 는 합력 목표각의 부호. 가속 방향과 기울여야 할 방향의 관계는
 * IMU 장착 방향에 따라 달라지므로 실측으로 정한다.
 * 반대면 액체를 더 흔든다. 파일 끝 "부호 확인 절차" 참고.
 *
 * ACC_LPF 는 θ_ref 전용 저역통과. 선형 가속 추정은 노이즈가 크므로 그대로
 * 쓰면 지령이 떨린다. 다만 필터가 세면 지연이 늘어 슬로싱 억제에 불리하다.
 */
volatile float ACC_GAIN = 1.0f;      // [ag] 기본 1.00  ag0 으로 끄면 1단계와 동일
volatile float DIR_ACC  = +1.0f;     //      기본 +1    실측 확정 (명령 없음)
volatile float ACC_LPF  = 0.2f;      // [ad] 기본 0.20  ★ 3축 합력 벡터 전용
const float ACC_REF_LIMIT = 20.0f;   //      기본 20°   θ_ref 상한 (명령 없음)

/* 합력 벡터를 성분 상태로 들고 있는다. 각도로 바꾼 뒤 거르지 않고, 벡터를
 * 거른 뒤 한 번만 각도로 바꾼다. 축별로 각도를 따로 거르면 두 축이 동시에
 * 움직일 때 벡터의 길이와 방향이 서로 어긋난다.
 */
float force_ax = 0, force_ay = 0, force_az = 1;
float force_pitch = 0, force_roll = 0;

// 계산된 목표각 (표시·기록용)
float ref_pitch = 0, ref_roll = 0;

/* 수평 선형가속을 차체 기울기로 학습하지 않게 하는 자세 보정 게이트.
 *
 * 상보필터는 가속도계가 읽은 각도로 자이로 적분을 보정한다. 그런데 수평으로
 * 가속하면 가속도계는 그것을 기울기로 읽는다. 그대로 보정하면 θ_base 가
 * 가속을 기울기로 흡수해버린다 (경사/가속 모호성).
 *
 * 자이로 예측각과 가속도계 각이 크게 벌어지면 그 차이는 선형가속일 수 있다.
 * 그동안 해당 축은 자이로 예측만 쓰고, 다시 가까워진 상태가 지속되면 보정을
 * 재개한다. 합력 목표 계산에는 가속도계를 계속 쓴다.
 *
 * ⚠ 이 게이트는 모터 지령을 바꾸지 않는다. want = ref − θ_base 에서
 *   ref = θ_base − θ_force 이므로 θ_base 가 소거되기 때문이다.
 *   바뀌는 것은 (1) 로그의 pitch/ref 값과 (2) SR 의 종료 판정이다.
 *   게이트가 없으면 가속 중 raw_ref 가 씻겨나가 SR 이 "끝났다" 고 조기
 *   오판한다. 그것을 막는 것이 실질적 이득이다.
 *
 * 문턱은 SR 것을 그대로 쓴다. SR 이 꺼져 있어도 게이트는 돈다 — 로그의
 * θ_base 를 정직하게 유지하기 위해서다.
 */
struct AccelCorrectionGate {
  bool active;
  uint16_t quietTicks;
};

AccelCorrectionGate accGatePitch = {false, 0};
AccelCorrectionGate accGateRoll  = {false, 0};

/* ── ZV 입력성형 (3단계) ──────────────────────────────────────────────────
 * 목표각을 반씩 나눠 반주기 간격으로 보낸다. 앞 임펄스가 만든 출렁임을 뒤
 * 임펄스가 반대 위상으로 때려 지운다.
 *
 * ★ theta_ref 에만 건다. 수평 항에는 걸지 않는다.
 *
 *   theta_ref 는 우리가 트레이를 어디 두고 싶은지(기준값)다. 성형은 원래
 *   기준값에 거는 기법이다. 반면 theta_base 는 차체가 이미 기울어버린 것을
 *   잰 값(외란)이다. 여기에 성형을 걸면 반주기 동안 수평을 못 지키고, 그러면
 *   유효중력 방향 자체가 트레이 기준으로 기운다. 성형으로 막으려던 진동보다
 *   훨씬 큰 1차 외란이다.
 *
 * ⚠ 지연을 산다. 2임펄스면 평균 Td/4, 3임펄스면 Td/2 만큼 늦어진다.
 *   2.4Hz 기준 각각 104ms / 208ms 다. 1단계에서 잰 지연이 이미 150~180ms 라
 *   슬로싱 주기 420ms 의 상당 부분을 지연으로 쓰게 된다. r 로 슬루 지연을
 *   먼저 줄여놓고 켤 것.
 *
 * ⚠ ZV_FREQ 2.40 은 이전 서보 코드의 파라미터에서 역산한 값이다. 슬로싱
 *   주파수는 용기 형상과 액체 높이에 따라 변하므로 실측해서 넣어야 한다.
 *     원통 1차 모드 :  w^2 = 1.841 (g/R) tanh(1.841 h/R)
 */
volatile bool  ZV_ON   = true;       // [zv] 기본 켜짐  ★ 실측 확정 (2026-09-04). zv0 이면 2단계와 동일
volatile float ZV_FREQ = 2.40f;      // [zf] 기본 2.40Hz  ★ 실측값으로 바꿀 것
volatile float ZV_ZETA = 0.02f;      // [zd] 기본 0.02    액체는 보통 0.01~0.05
volatile int   ZV_MODE = 2;          // [zm] 기본 2       2=ZV, 3=ZVD

/* 100Hz 기준 1.28초. ZVD 는 2배 지연을 쓰므로 최저 0.79Hz 까지 담긴다. */
#define ZV_BUF 128
float zv_p[ZV_BUF] = {0}, zv_r[ZV_BUF] = {0};
int   zv_head = 0;

int   zv_n1 = 21;                       // 반주기 [샘플]
float zv_A1 = 0.5f, zv_A2 = 0.5f, zv_A3 = 0.0f;
float shaped_pitch = 0, shaped_roll = 0;

/* ── 가속 종료 소프트 복귀 ────────────────────────────────────────────────
 * 액체는 과거 가속도의 상태를 갖지만 직접보상 목표각은 현재 가속도가 0 이면
 * 곧바로 0 으로 사라진다. 가속 종료 때 남아 있는 ref 를 잠시 유지한 뒤
 * 최소저크 곡선으로 0 까지 보내, 트레이의 급정지가 새 슬로싱을 만드는 것을
 * 줄인다. 시작 구간은 건드리지 않으므로 직접보상의 빠른 반응은 그대로다.
 *
 * ZV 와 목적이 겹치므로 동시에 사용하지 않는다. 기본은 OFF 이고 sr1 로 켠다.
 * 각 축은 독립적으로 종료를 검출한다.
 */
volatile bool     SOFT_RETURN_ON = false;   // [sr]  기본 꺼짐
volatile float    SR_ACTIVE_DEG  = 1.50f;   // [sra] 이 이상을 실제 가속으로 인정
volatile float    SR_END_DEG     = 0.75f;   // [sre] 이 아래가 지속되면 종료
volatile uint16_t SR_DWELL_MS    = 30;      // [srd] 종료 확인 시간
volatile uint16_t SR_HOLD_MS     = 100;     // [srh] 남은 반대 목표각 유지 시간
volatile uint16_t SR_RETURN_MS   = 120;     // [srr] 최소저크 수평 복귀 시간

enum SoftReturnPhase : uint8_t { SR_DIRECT, SR_CONFIRM, SR_HOLD, SR_RETURN, SR_LEVEL };

struct SoftReturnAxis {
  SoftReturnPhase phase;
  bool sawActive;
  uint16_t quietTicks;
  uint16_t phaseTicks;
  float holdAngle;
  float lastActiveAngle;      // 문턱을 넘기 직전의 유효 목표각
};

SoftReturnAxis srPitch = {SR_DIRECT, false, 0, 0, 0, 0};
SoftReturnAxis srRoll  = {SR_DIRECT, false, 0, 0, 0, 0};
volatile bool srResetRequest = false;

/* 폭주 차단 — 지령은 ±LIMIT_DEG 를 넘지 않으므로 실제 위치가 이 값을 넘으면
 * 우리가 시킨 움직임이 아니다.
 */
const float ACT_LIMIT_DEG = 50.0f;   //      기본 50°   폭주 차단 (명령 없음)
const uint32_t FB_TIMEOUT_MS = 300;
unsigned long lastFb = 0;

// ── MIT 변환 범위 — 드라이버 설정과 일치해야 한다 ──
const float P_MIN  = -12.5f, P_MAX  =  12.5f;   // rad
const float V_MIN  = -30.0f, V_MAX  =  30.0f;   // rad/s (VMAX 30)
const float KP_MIN =   0.0f, KP_MAX = 500.0f;
const float KD_MIN =   0.0f, KD_MAX =   5.0f;
const float T_MIN  = -10.0f, T_MAX  =  10.0f;

// ── LSM6DSOX 레지스터 (I2C·SPI 공통) ──
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_OUTX_L_G   0x22

#define WHO_AM_I_VAL   0x6C
#define CTRL1_XL_VAL   0x68    // 416Hz, ±4g
#define CTRL2_G_VAL    0x6C    // 416Hz, ±2000dps
#define CTRL3_C_VAL    0x44    // BDU=1, IF_INC=1

const float G_LSB    = 8192.0;
const float GYRO_LSB = 14.286;
const float dt       = 0.01;
const float alpha_cf = 0.98;

const float ACC_TRUST_FALLOFF = 0.5;
const float ACC_MAG_DEAD      = 0.05;
const uint16_t REJ_RUN_FAULT  = 100;

const uint8_t ST_FAULT_MIN = 0x8;

const char* statusName(uint8_t st) {
  switch (st) {
    case 0x0: return "Disable";
    case 0x1: return "Enable";
    case 0x8: return "과전압";
    case 0x9: return "저전압";
    case 0xA: return "과전류";
    case 0xB: return "MOS과열";
    case 0xC: return "권선과열";
    case 0xD: return "통신끊김";
    case 0xE: return "과부하";
    default:  return "미정의";
  }
}

// =============================================================================
//  상태 변수
// =============================================================================
float pitch_filtered = 0, roll_filtered = 0;
float gyro_bias_x = 0, gyro_bias_y = 0;
float g_scale = 1.0;

float cmd_pitch = 0, cmd_roll = 0;

volatile bool  armed  = false;
bool  imu_ok = false;
volatile bool  stream = true;
uint16_t rej_run = 0;

uint8_t  last_status[4] = {0xFF, 0xFF, 0xFF, 0xFF};
float    act_deg[4]     = {0, 0, 0, 0};
uint32_t fb_count = 0;
uint32_t i2c_err  = 0;             // I2C 읽기 실패 누적

unsigned long lastLoop = 0, lastPrint = 0, lastCheck = 0;
uint32_t n_win = 0, err_win = 0, err_reset = 0;


// =============================================================================
//  RTOS 기반 시설
// =============================================================================
/* 태스크 두 개로 나눈다.
 *
 *   ctrlTask (코어1, 우선순위6)  100Hz 제어. 기존 loop() 의 내용 그대로.
 *   ioTask   (코어0, 우선순위2)  시리얼 명령 읽기 + 모든 출력.
 *
 * 나눈 이유: USB CDC 는 PC 가 안 읽어가면 Serial.printf 가 그 자리에서 막힌다.
 * 지금까지는 그 순간 제어 루프가 통째로 섰다. 코어를 나누면 물리적으로 무관해진다.
 *
 * ── 규칙 1. Serial 은 ioTask 만 만진다 ──
 * 두 태스크가 동시에 쓰면 줄 중간이 섞여 로그가 깨진다. check_tuning.py 는
 * 한 줄 안에서 pre_pitch 와 cmd_pitch 를 짝지어 읽으므로 치명적이다.
 * ctrlTask 는 큐에 넣기만 하고, ioTask 가 꺼내서 찍는다.
 * 큐를 하나만 두는 것이 중요하다 — 둘로 나누면 이벤트와 스트림 순서가 뒤집힌다.
 *
 * 스트림은 숫자만 담아 보낸다. %.2f 13개를 포맷하는 비용(수백 us)을 ioTask 가 진다.
 */
struct Snap {
  uint32_t t;
  float raw_p, raw_r;              // ACC_LPF 를 걸기 전의 합력 목표각
  float pre_p, pre_r, pitch, ref_p, zv_p, cmd_p, act_p;
  float roll, ref_r, zv_r, cmd_r, act_r;
};

/* 큐를 둘로 나눈다.
 *
 *   qEvt   중요 이벤트 — 모터 이상, 폭주 차단, 피드백 두절, 센서 리셋, DISARM 이유.
 *          샘플과 한 줄에 세우면 PC 가 멈춘 순간 샘플이 자리를 먼저 차지해서
 *          정작 원인이 되는 메시지가 버려진다. 제어는 돌아도 사후 분석이 막힌다.
 *   qSnap  33Hz 스트림. 가득 차면 버린다. 이쪽은 버려도 된다.
 *
 * ioTask 는 qEvt 를 먼저 비운다. 그래서 이벤트 줄이 샘플 줄보다 최대 몇 ms
 * 앞당겨 나올 수 있다 — 줄 단위로 읽는 분석에는 영향이 없다.
 */
QueueHandle_t qEvt  = NULL;      // 텍스트 12통
QueueHandle_t qSnap = NULL;      // 샘플 32통 (약 1초치)
#define EVT_LEN 192              // [stat] 줄이 한글 포함 약 140바이트

/* 명령도 큐로 받는다. 변수 하나로 두면 ctrlTask 가 읽고 지우는 사이에 들어온
 * 명령이 통째로 사라지고, 연속 입력은 앞의 것을 덮어쓴다.
 */
QueueHandle_t qCmd = NULL;       // 명령 6통

/* arm(250ms)·t(8초) 실행 중임을 알린다. 원본에서는 그동안 handleSerial 이
 * 아예 불리지 않아 kp·dr·d 같은 설정이 바뀔 수 없었다. 특히 sweep 도중 dr 이
 * 들어오면 복귀 직후 roll 방향이 뒤집힌다 — 출력이 아니라 모터가 달라진다.
 * 그래서 이 깃발이 서 있는 동안 파서를 멈춘다. 입력은 UART 버퍼에 그대로
 * 쌓였다가 끝난 뒤 처리되므로 원본과 동작이 같다.
 */
volatile bool ctrlBusy = false;

/* 계측. 0 으로 두면 출력이 원본과 완전히 같아진다.
 * 33Hz 스트림의 t: 간격만으로는 100Hz 누락을 못 잡는다 — 틱 하나가 빠져도
 * 앞뒤 출력이 30ms 로 보인다. 그래서 틱 자체를 재는 수밖에 없다.
 */
#define RTOS_DIAG 1
volatile uint32_t dgMaxUs = 0, dgOver = 0, dgResync = 0;
volatile uint32_t dgTxFail = 0, dgSnapDrop = 0, dgEvtDrop = 0, dgCmdDrop = 0;

/* ZV 계수는 zv_n1/A1/A2/A3 네 개가 한 세트라 따로 읽으면 섞인 조합이 나온다.
 * 링버퍼(zv_p/zv_r/zv_head)도 zvClear 가 통째로 지우므로 같은 문제가 있다.
 * 잠그는 시간은 float 256개 처리 수준(수 us)이라 제어에 영향이 없다.
 *
 * GAIN·KP_ROLL·MAX_RATE 같은 홑겹 변수는 32비트 단일 접근이 원자적이라
 * volatile 만으로 충분하다. 옛 값 아니면 새 값이지 섞인 값은 안 나온다.
 */
portMUX_TYPE zvMux = portMUX_INITIALIZER_UNLOCKED;
#define ZV_LOCK()    taskENTER_CRITICAL(&zvMux)
#define ZV_UNLOCK()  taskEXIT_CRITICAL(&zvMux)

uint16_t msToCtrlTicks(uint16_t ms) {
  uint16_t ticks = (uint16_t)((ms + 9U) / 10U);
  return ticks > 0 ? ticks : 1;
}

void accelGateReset() {
  accGatePitch = {false, 0};
  accGateRoll  = {false, 0};
}

/* 자이로 예측과 가속도계 각의 차이를 보고 보정을 막을지 정한다.
 * 켜지는 문턱과 꺼지는 문턱이 다르다 (히스테리시스). 하나면 문턱 근처에서
 * 켜졌다 꺼졌다 하며 자세 추정이 흔들린다.
 */
bool accelGateStep(float angleResidual, AccelCorrectionGate &g) {
  float a = fabsf(angleResidual);
  uint16_t dwellTicks = msToCtrlTicks(SR_DWELL_MS);

  if (!g.active) {
    if (a >= SR_ACTIVE_DEG) {
      g.active = true;
      g.quietTicks = 0;
    }
  } else if (a <= SR_END_DEG) {
    if (g.quietTicks < 0xFFFF) g.quietTicks++;
    if (g.quietTicks >= dwellTicks) {
      g.active = false;
      g.quietTicks = 0;
    }
  } else {
    g.quietTicks = 0;
  }
  return g.active;
}

void softReturnResetAxis(SoftReturnAxis &s) {
  s.phase = SR_DIRECT;
  s.sawActive = false;
  s.quietTicks = 0;
  s.phaseTicks = 0;
  s.holdAngle = 0;
  s.lastActiveAngle = 0;
}

void softReturnReset() {
  softReturnResetAxis(srPitch);
  softReturnResetAxis(srRoll);
}

const char *softReturnPhaseName(SoftReturnPhase phase) {
  switch (phase) {
    case SR_CONFIRM: return "CONFIRM";
    case SR_HOLD:   return "HOLD";
    case SR_RETURN: return "RETURN";
    case SR_LEVEL:  return "LEVEL";
    default:        return "DIRECT";
  }
}

/* 종료 임계값을 처음 통과한 순간 직전 유효 목표를 붙잡는다.
 *
 * 종료 확인(dwell)을 마친 뒤에 잡으면 그 30ms 동안 ACC_LPF 가 이미 목표각을
 * 0 쪽으로 깎아놓아 HOLD 가 실제 피크보다 약해진다. 그래서 문턱을 넘는 순간
 * 직전 값(lastActiveAngle)을 잡아두고, 확인은 CONFIRM 단계에서 따로 한다.
 * CONFIRM 중에 가속이 다시 올라오면 DIRECT 로 되돌아간다 — 오판을 물릴 수 있다.
 */
float softReturnStep(float ref, float rawRef, SoftReturnAxis &s) {
  float a = fabsf(rawRef);
  uint16_t dwellTicks  = msToCtrlTicks(SR_DWELL_MS);
  uint16_t holdTicks   = msToCtrlTicks(SR_HOLD_MS);
  uint16_t returnTicks = msToCtrlTicks(SR_RETURN_MS);

  /* 새 가속은 꼬리 파형보다 우선한다. CMD_LPF 와 slew 가 전환 계단을 제한한다. */
  if (s.phase != SR_DIRECT && a >= SR_ACTIVE_DEG) {
    softReturnResetAxis(s);
    s.sawActive = true;
    s.lastActiveAngle = ref;
    return ref;
  }

  if (s.phase == SR_DIRECT) {
    if (a >= SR_ACTIVE_DEG) {
      s.sawActive = true;
      s.quietTicks = 0;
      s.lastActiveAngle = ref;
    } else if (s.sawActive) {
      if (a > SR_END_DEG) {
        s.lastActiveAngle = ref;      // 아직 유효 구간. 계속 갱신한다
      } else {
        s.phase = SR_CONFIRM;
        s.phaseTicks = 0;
        s.holdAngle = s.lastActiveAngle;   // 깎이기 전 값을 잡는다
        if (fabsf(s.holdAngle) < 0.15f) {
          s.phase = SR_LEVEL;
          s.sawActive = false;
        }
        return s.holdAngle;
      }
    }
    return ref;
  }

  if (s.phase == SR_CONFIRM) {
    if (a > SR_END_DEG) {              // 아직 끝난 게 아니었다
      s.phase = SR_DIRECT;
      s.phaseTicks = 0;
      s.lastActiveAngle = ref;
      return ref;
    }
    if (++s.phaseTicks >= dwellTicks) {
      s.phase = SR_HOLD;
      s.phaseTicks = 0;
    }
    return s.holdAngle;
  }

  if (s.phase == SR_HOLD) {
    if (++s.phaseTicks >= holdTicks) {
      s.phase = SR_RETURN;
      s.phaseTicks = 0;
    }
    return s.holdAngle;
  }

  if (s.phase == SR_RETURN) {
    if (s.phaseTicks < returnTicks) s.phaseTicks++;
    float x = constrain((float)s.phaseTicks / (float)returnTicks, 0.0f, 1.0f);
    float smooth = x * x * x * (10.0f + x * (-15.0f + 6.0f * x));
    float out = s.holdAngle * (1.0f - smooth);
    if (s.phaseTicks >= returnTicks) {
      s.phase = SR_LEVEL;
      s.sawActive = false;
      s.holdAngle = 0;
      out = 0;
    }
    return out;
  }

  /* LEVEL 동안 ref 는 백그라운드에서 거의 0 으로 수렴한다. */
  if (a >= SR_ACTIVE_DEG) {
    softReturnResetAxis(s);
    s.sawActive = true;
  }
  return ref;
}

/* ctrlTask 쪽 출력. 이벤트 전용 큐라 샘플에 밀리지 않는다. 드문 일이므로
 * 5ms 까지는 기다린다 — 원본도 printf 가 막혔으므로 동작이 같다.
 * 태스크 생성 전에는 직접 찍는다.
 */
void qprintf(const char *fmt, ...) {
  char buf[EVT_LEN];
  va_list ap;  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (!qEvt) { Serial.print(buf); return; }
  /* 절대 기다리지 않는다. PC 가 안 읽어서 이벤트 큐까지 차면 로그 한 줄마다
   * 제어가 멈춘다 — 그걸 없애려고 태스크를 나눈 것이라 앞뒤가 맞지 않는다.
   * 잃은 줄은 dgEvtDrop 으로 센다.
   */
  if (xQueueSend(qEvt, buf, 0) != pdTRUE) dgEvtDrop = dgEvtDrop + 1;
}

void qprintln(const char *t) {
  qprintf("%s\n", t);
}

/* stop 은 잃으면 안 된다. 자리가 없으면 앞의 명령을 하나 버려서라도 넣는다.
 * 넣었으면 true. 실패했을 때 호출자가 ctrlBusy 를 되돌려야 하므로 결과를 준다.
 */
bool ctrlSend(uint8_t c) {
  if (xQueueSend(qCmd, &c, 0) == pdTRUE) return true;
  if (c == 2) {
    uint8_t junk;
    xQueueReceive(qCmd, &junk, 0);
    if (xQueueSend(qCmd, &c, 0) == pdTRUE) return true;
  }
  dgCmdDrop = dgCmdDrop + 1;
  Serial.println("!! 명령 큐 가득 — 무시됨");   // 조용히 잃는 것보다 낫다
  return false;
}

// =============================================================================
//  IMU — I2C
// =============================================================================
/* SPI 판과 다른 부분은 여기뿐이다.
 * I2C 에는 SPI 의 "읽기는 MSB=1" 같은 규칙이 없다. 레지스터 주소를 쓰고
 * repeated start 로 곧바로 읽는다. 연속 읽기는 CTRL3_C 의 IF_INC 가 1 이어야
 * 주소가 자동 증가한다 (CTRL3_C_VAL 0x44 에 포함).
 */
bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { i2c_err++; return false; }   // repeated start
  if (Wire.requestFrom((int)IMU_ADDR, (int)n) != n) { i2c_err++; return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

uint8_t readReg(uint8_t reg) {
  uint8_t v = 0;
  i2cRead(reg, &v, 1);        // 실패하면 0 — 파워다운 감지가 이를 잡는다
  return v;
}

void writeReg(uint8_t reg, uint8_t val) { i2cWrite(reg, val); }

// 자이로 3축 + 가속도 3축 (리틀엔디안, 자이로 먼저)
void readIMU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t b[12] = {0};
  i2cRead(REG_OUTX_L_G, b, 12);   // 실패하면 전부 0 → 아래 검사에 걸린다

  gx = (int16_t)((b[1]  << 8) | b[0]);
  gy = (int16_t)((b[3]  << 8) | b[2]);
  gz = (int16_t)((b[5]  << 8) | b[4]);
  ax = (int16_t)((b[7]  << 8) | b[6]);
  ay = (int16_t)((b[9]  << 8) | b[8]);
  az = (int16_t)((b[11] << 8) | b[10]);
}

void configSensor() {
  writeReg(REG_CTRL3_C, 0x01);          // 소프트 리셋
  delay(50);
  writeReg(REG_CTRL3_C,  CTRL3_C_VAL);
  writeReg(REG_CTRL1_XL, CTRL1_XL_VAL);
  writeReg(REG_CTRL2_G,  CTRL2_G_VAL);
  delay(100);
}


// =============================================================================
//  MIT 모드 CAN  (SPI 판과 동일)
// =============================================================================
uint16_t float_to_uint(float x, float xmin, float xmax, uint8_t bits) {
  float span = xmax - xmin;
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  uint32_t maxv = (1UL << bits) - 1;
  uint32_t v = (uint32_t)((x - xmin) * ((float)(1UL << bits) / span));
  return (uint16_t)(v > maxv ? maxv : v);
}

float uint_to_float(uint16_t xi, float xmin, float xmax, uint8_t bits) {
  float span = xmax - xmin;
  return ((float)xi) * span / ((float)((1UL << bits) - 1)) + xmin;
}

/* MIT 제어 프레임 (매뉴얼 42p)
 *   D0 pos[15:8]  D1 pos[7:0]  D2 vel[11:4]  D3 vel[3:0]|kp[11:8]
 *   D4 kp[7:0]    D5 kd[11:4]  D6 kd[3:0]|t[11:8]        D7 t[7:0]
 */
void sendMIT(uint8_t id, float pos, float vel, float kp, float kd, float tff) {
  uint16_t p   = float_to_uint(pos, P_MIN,  P_MAX,  16);
  uint16_t v   = float_to_uint(vel, V_MIN,  V_MAX,  12);
  uint16_t kpi = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
  uint16_t kdi = float_to_uint(kd,  KD_MIN, KD_MAX, 12);
  uint16_t t   = float_to_uint(tff, T_MIN,  T_MAX,  12);

  twai_message_t m = {};
  m.identifier = CAN_MODE | id;
  m.data_length_code = 8;
  m.data[0] = p >> 8;
  m.data[1] = p & 0xFF;
  m.data[2] = v >> 4;
  m.data[3] = ((v & 0x0F) << 4) | (kpi >> 8);
  m.data[4] = kpi & 0xFF;
  m.data[5] = kdi >> 4;
  m.data[6] = ((kdi & 0x0F) << 4) | (t >> 8);
  m.data[7] = t & 0xFF;
  /* 대기 0. 이건 50Hz 로 계속 나가는 주기 지령이라 한 장 버려도 20ms 뒤에
   * 다음 장이 간다. 반면 5ms 를 기다리면 두 축이 10ms — 한 틱 예산 전부다.
   *
   * 실측: 모터 전원을 끈 채 arm 하면 CAN ACK 가 없어 송신 큐가 막히는데,
   * 5ms 일 때 17틱이 10ms 를 넘었다 (TX실패 36). 0 이면 그런 상황에서도
   * 주기가 유지된다. 정상 연결에서는 큐가 안 차므로 동작이 같다.
   *
   * enable/disable/영점 같은 비주기 명령은 sendUniversal 이고, 거기는
   * 한 장이 중요하므로 5ms 를 그대로 둔다.
   */
  if (twai_transmit(&m, 0) != ESP_OK) dgTxFail = dgTxFail + 1;
}

void sendUniversal(uint8_t id, uint8_t last) {  // FB=에러해제 FC=enable FD=disable FE=영점
  twai_message_t m = {};
  m.identifier = CAN_MODE | id;
  m.data_length_code = 8;
  memset(m.data, 0xFF, 7);
  m.data[7] = last;
  if (twai_transmit(&m, pdMS_TO_TICKS(5)) != ESP_OK) dgTxFail = dgTxFail + 1;
}

void broadcastUniversal(uint8_t last) {
  sendUniversal(CAN_ID_PITCH, last);
  sendUniversal(CAN_ID_ROLL,  last);
}

/* DIR 은 여기서만 곱한다. 모터 배선·장착 방향을 고쳐도 제어식과 화면 숫자는
 * 흔들리지 않는다. 두 관심사를 한 노브에 묶으면 부호를 뒤집을 때마다 로그의
 * 의미가 같이 바뀌어 무엇을 보고 있는지 알 수 없게 된다.
 */
void sendBoth() {
  sendMIT(CAN_ID_PITCH, cmd_pitch * DIR_PITCH * DEG_TO_RAD, 0.0f, KP_PITCH, KD_PITCH, 0.0f);
  sendMIT(CAN_ID_ROLL,  cmd_roll  * DIR_ROLL  * DEG_TO_RAD, 0.0f, KP_ROLL,  KD_ROLL,  0.0f);
}


// =============================================================================
//  arm / disarm
// =============================================================================
void motorsDisarm(const char *reason) {
  if (armed) qprintf(">>> DISARM — %s\n", reason);
  broadcastUniversal(0xFD);
  armed = false;
  cmd_pitch = 0;  cmd_roll = 0;
  softReturnReset();
  accelGateReset();
}

void motorsArm() {
  // 저장된 플래그를 믿지 말고 직접 읽는다 (워치독이 내린 뒤 스스로 안 올라온다)
  uint8_t id = readReg(REG_WHO_AM_I);
  imu_ok = (id == WHO_AM_I_VAL);
  if (!imu_ok) {
    qprintf("!! IMU 응답 0x%02X (기대 0x%02X) — arm 거부\n", id, WHO_AM_I_VAL);
    return;
  }
  rej_run = 0;

  qprintln(">>> arm: 트레이 중립 / 차체 수평 상태여야 합니다");
  broadcastUniversal(0xFB);  delay(50);   // 에러 해제
  broadcastUniversal(0xFE);  delay(50);   // 현재 위치를 영점으로

  // enable 전에 목표를 현재 위치(0)로 박아둔다. 빼면 마지막 목표로 확 튄다.
  cmd_pitch = 0;  cmd_roll = 0;
  softReturnReset();
  accelGateReset();
  sendBoth();
  delay(50);

  broadcastUniversal(0xFC);  delay(100);  // enable

  for (int i = 0; i < 4; i++) act_deg[i] = 0;   // 영점 후 낡은 위치값 제거
  lastFb = millis();
  armed = true;
  qprintf(">>> ARMED  GAIN=%.2f  ACC_GAIN=%.2f  Kp=%.1f  Kd=%.2f  제한 ±%.0f°\n",
                GAIN, ACC_GAIN, KP_PITCH, KD_PITCH, LIMIT_DEG);
}


// =============================================================================
//  유틸
// =============================================================================
/* 임펄스 시각과 크기를 다시 계산한다. 파라미터가 바뀔 때만 부르면 된다.
 *
 *   K   = exp(-z*pi / sqrt(1-z^2))        한 주기 뒤 진폭비
 *   Td  = 1 / (f * sqrt(1-z^2))           감쇠 주기
 *   ZV  : [1, K] / (1+K)          at 0, Td/2
 *   ZVD : [1, 2K, K^2] / (1+K)^2  at 0, Td/2, Td
 *
 * ZVD 는 주파수를 잘못 잡아도 잘 듣지만 지연이 두 배다. 주파수를 실측으로
 * 확정하기 전까지는 ZVD 가 안전하고, 확정한 뒤에는 ZV 가 낫다.
 */
void zvRecalc() {
  float z  = constrain(ZV_ZETA, 0.0f, 0.9f);
  float wd = sqrtf(1.0f - z * z);
  float K  = expf(-z * PI / wd);
  float Td = 1.0f / (ZV_FREQ * wd);

  zv_n1 = (int)(Td * 0.5f / dt + 0.5f);
  int maxn = (ZV_MODE == 3) ? (ZV_BUF - 1) / 2 : (ZV_BUF - 1);
  zv_n1 = constrain(zv_n1, 1, maxn);

  if (ZV_MODE == 3) {
    float d = (1.0f + K) * (1.0f + K);
    zv_A1 = 1.0f / d;  zv_A2 = 2.0f * K / d;  zv_A3 = K * K / d;
  } else {
    zv_A1 = 1.0f / (1.0f + K);  zv_A2 = K / (1.0f + K);  zv_A3 = 0.0f;
  }
}

// 성형이 사는 평균 지연 [ms] — 켤지 말지 판단하는 숫자
float zvDelayMs() {
  return (zv_A2 * zv_n1 + zv_A3 * 2 * zv_n1) * dt * 1000.0f;
}

void zvClear() {
  for (int i = 0; i < ZV_BUF; i++) { zv_p[i] = 0; zv_r[i] = 0; }
  zv_head = 0;
}

float slew(float target, float prev, float maxStep) {
  float d = target - prev;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  return prev + d;
}

/* 게인을 바꿀 때마다 두 축을 함께 찍는다. 한쪽만 올리고 다른 쪽을 잊는 것이
 * 이 구조에서 가장 하기 쉬운 실수라서, 항상 나란히 보여준다.
 */
void reportKp() {
  float tp = KP_PITCH * LIMIT_DEG * DEG_TO_RAD;
  float tr = KP_ROLL  * LIMIT_DEG * DEG_TO_RAD;
  Serial.printf(">>> Kp  안쪽 %.2f  바깥 %.2f  [N/rad]\n", KP_PITCH, KP_ROLL);
  Serial.printf("    %.0f도 오차에서 토크  안쪽 %.2f  바깥 %.2f N·m\n", LIMIT_DEG, tp, tr);
  if (tp > TORQUE_WARN || tr > TORQUE_WARN)
    Serial.printf("    !! %.1f N·m 초과 — 목표가 튀면 그만큼 나갑니다\n", TORQUE_WARN);
}

void reportKd(float changed) {
  Serial.printf(">>> Kd  안쪽 %.3f  바깥 %.3f  [N·s/rad]\n", KD_PITCH, KD_ROLL);
  if (changed == 0.0f) Serial.println("    !! Kd=0 은 매뉴얼상 발진·폭주 조건입니다");
  /* Kp 를 올리면 감쇠비가 sqrt(Kp) 만큼 떨어진다. 같이 안 올리면 발진한다. */
  if (KP_ROLL > 3.0f && KD_ROLL < 0.17f)
    Serial.println("    !! 바깥축 Kp 대비 Kd 가 낮습니다 (1.75Hz 한계진동 이력)");
}

void printStatus() {
  Serial.printf("[상태] %s  GAIN=%.2f  DIR(p/r)=%+.0f/%+.0f  IMU=%s\n",
                armed ? "ARMED" : "STOP", GAIN,
                DIR_PITCH, DIR_ROLL, imu_ok ? "OK" : "FAULT");
  Serial.printf("       Kp 안쪽 %.2f / 바깥 %.2f    Kd 안쪽 %.3f / 바깥 %.3f\n",
                KP_PITCH, KP_ROLL, KD_PITCH, KD_ROLL);
  Serial.printf("       ACC_GAIN=%.2f  DIR_ACC=%+.0f  ACC_LPF=%.2f   ref(p/r)=%.2f/%.2f\n",
                ACC_GAIN, DIR_ACC, ACC_LPF, ref_pitch, ref_roll);
  /* 어느 성형기가 켜져 있는지 한눈에. 둘은 동시에 켜지지 않는다. */
  Serial.printf("       ZV %s   %.2fHz  z=%.3f  %d임펄스  지연 +%.0fms\n",
                ZV_ON ? "ON " : "OFF", ZV_FREQ, ZV_ZETA, ZV_MODE, zvDelayMs());
  /* 값마다 어느 명령으로 바꾸는지 이름을 붙여 찍는다. 표를 다시 찾아보지
   * 않고 바로 고칠 수 있다.
   */
  Serial.printf("       합력각(p/r) %.2f/%.2f°   자세보정게이트 %s/%s\n",
                force_pitch, force_roll,
                accGatePitch.active ? "BLOCK" : "TRACK",
                accGateRoll.active  ? "BLOCK" : "TRACK");
  Serial.printf("       소프트복귀 %s   sra %.2f°  sre %.2f°\n",
                SOFT_RETURN_ON ? "ON " : "OFF", SR_ACTIVE_DEG, SR_END_DEG);
  Serial.printf("                       srd %ums(확인)  srh %ums(유지)  srr %ums(복귀)\n",
                SR_DWELL_MS, SR_HOLD_MS, SR_RETURN_MS);
  Serial.printf("       소프트복귀 상태  안쪽 %s / 바깥 %s\n",
                softReturnPhaseName(srPitch.phase), softReturnPhaseName(srRoll.phase));
  Serial.printf("       pitch=%.2f roll=%.2f  cmd(p/r)=%.2f/%.2f  act(p/r)=%.2f/%.2f\n",
                pitch_filtered, roll_filtered, cmd_pitch, cmd_roll,
                act_deg[CAN_ID_PITCH & 0x03], act_deg[CAN_ID_ROLL & 0x03]);
  Serial.printf("       피드백 %lu회  I2C오류 %lu회  P(id%u)=%s  R(id%u)=%s\n",
                fb_count, i2c_err,
                CAN_ID_PITCH, statusName(last_status[CAN_ID_PITCH & 0x03]),
                CAN_ID_ROLL,  statusName(last_status[CAN_ID_ROLL  & 0x03]));
  if (fb_count == 0) {
    Serial.println("       !! 피드백 0회 — ControlMode 가 MIT 인지, CAN 배선 확인");
  }
}

void drainCAN() {
  twai_message_t rx;
  while (twai_receive(&rx, 0) == ESP_OK) {
    uint8_t mid = rx.data[0] & 0x0F;
    uint8_t st  = rx.data[0] >> 4;
    uint8_t i   = mid & 0x03;

    uint16_t praw = ((uint16_t)rx.data[1] << 8) | rx.data[2];
    act_deg[i] = uint_to_float(praw, P_MIN, P_MAX, 16) * RAD_TO_DEG;

    last_status[i] = st;
    fb_count++;
    lastFb = millis();

    if (st >= ST_FAULT_MIN && armed) {
      qprintf("!! 모터 %u 이상 — %s\n", mid, statusName(st));
      motorsDisarm("모터 이상");
    }
  }

  if (!armed) return;

  float ap = act_deg[CAN_ID_PITCH & 0x03];
  float ar = act_deg[CAN_ID_ROLL  & 0x03];
  if (fabsf(ap) > ACT_LIMIT_DEG || fabsf(ar) > ACT_LIMIT_DEG) {
    qprintf("!! 폭주 차단 — 실제 위치 P=%.1f° R=%.1f° (한계 ±%.0f°)\n",
                  ap, ar, ACT_LIMIT_DEG);
    motorsDisarm("위치 한계 초과");
    return;
  }

  if (lastFb != 0 && millis() - lastFb > FB_TIMEOUT_MS) {
    qprintf("!! 피드백 %lums 끊김\n", millis() - lastFb);
    motorsDisarm("피드백 두절");
  }
}

/* CAN 경로만 확인하는 시험 구동. IMU·제어식을 거치지 않는다.
 * MIT 는 속도 제한이 없으므로 계단이 아니라 램프로 움직인다.
 */
void motorSweep() {
  const float SWEEP_DEG  = 8.0f;
  const float SWEEP_RATE = 20.0f;

  qprintf(">>> 시험 구동 8초 (±%.0f°, %.0f°/s). 손을 치우세요.\n",
                SWEEP_DEG, SWEEP_RATE);
  broadcastUniversal(0xFB);  delay(50);
  broadcastUniversal(0xFE);  delay(50);
  sendMIT(CAN_ID_PITCH, 0, 0, KP_PITCH, KD_PITCH, 0);
  sendMIT(CAN_ID_ROLL,  0, 0, KP_ROLL,  KD_ROLL,  0);
  delay(50);
  broadcastUniversal(0xFC);  delay(100);

  float p = 0;
  int   dir = +1;
  uint32_t t0 = millis(), last = millis();
  while (millis() - t0 < 8000) {
    uint32_t now = millis();
    p += dir * SWEEP_RATE * (now - last) * 0.001f;
    last = now;
    if (p >=  SWEEP_DEG) { p =  SWEEP_DEG; dir = -1; }
    if (p <= -SWEEP_DEG) { p = -SWEEP_DEG; dir = +1; }

    sendMIT(CAN_ID_PITCH, p * DEG_TO_RAD, 0, KP_PITCH, KD_PITCH, 0);
    sendMIT(CAN_ID_ROLL,  p * DEG_TO_RAD, 0, KP_ROLL,  KD_ROLL,  0);
    drainCAN();
    delay(20);
  }
  while (fabsf(p) > 0.5f) {           // 기울어진 채로 끄지 않는다
    p += (p > 0 ? -0.4f : 0.4f);
    sendMIT(CAN_ID_PITCH, p * DEG_TO_RAD, 0, KP_PITCH, KD_PITCH, 0);
    sendMIT(CAN_ID_ROLL,  p * DEG_TO_RAD, 0, KP_ROLL,  KD_ROLL,  0);
    delay(20);
  }
  broadcastUniversal(0xFD);
  qprintf(">>> 시험 종료. 피드백 %lu회\n", fb_count);
}

void handleLine(const char *raw) {
  String s = raw;
  s.trim();
  if (s.length() == 0) return;
  String u = s;  u.toLowerCase();

  /* arm 은 delay 만 250ms, t(sweep) 는 수 초 걸린다. 여기(ioTask)서 돌리면
   * 그동안 ctrlTask 가 CAN 지령을 계속 쏜다 — 영점 잡는 중에 지령이 끼어드는
   * 셈이라 지금은 물리적으로 불가능한 상황이 된다. 또 motorsArm 은 I2C 를
   * 쓰는데 Wire 는 두 태스크가 동시에 만지면 깨진다.
   *
   * 그래서 쪽지만 남기고 ctrlTask 가 직접 실행한다. 결과적으로 지금과 똑같이
   * 같은 코드가 같은 것을 멈춘 채로 돈다.
   */
  /* 큐에 못 넣었으면 ctrlBusy 를 되돌린다. 안 그러면 풀어줄 사람이 없어
   * 파서가 영영 멈춘 채로 남는다.
   */
  if (u == "arm") {
    ctrlBusy = true;  if (!ctrlSend(1)) ctrlBusy = false;   // 250ms 파서 정지
  } else if (u == "stop" || u == "s") {
    ctrlSend(2);
  } else if (u == "z") {
    ctrlSend(4);
  } else if (u == "t") {
    ctrlBusy = true;  if (!ctrlSend(3)) ctrlBusy = false;   // 8초 파서 정지
  /* kpp/kpr/kdp/kdr 은 kp/kd 보다 반드시 먼저 검사한다.
   * ag 를 g 보다 먼저 보는 것과 같은 이유로, 순서가 뒤바뀌면 "kpr6" 이
   * kp 접두에 걸려 substring(2)="r6" -> toFloat()=0 으로 조용히 먹힌다.
   */
  } else if (u.startsWith("kpp") || u.startsWith("kpr")) {
    bool isP = u.startsWith("kpp");
    float v = s.substring(3).toFloat();
    if (v >= KP_MIN && v <= KP_MAX) {
      if (isP) KP_PITCH = v; else KP_ROLL = v;
      reportKp();
    } else Serial.printf("!! Kp 범위 %.0f ~ %.0f\n", KP_MIN, KP_MAX);
  } else if (u.startsWith("kdp") || u.startsWith("kdr")) {
    bool isP = u.startsWith("kdp");
    float v = s.substring(3).toFloat();
    if (v >= KD_MIN && v <= KD_MAX) {
      if (isP) KD_PITCH = v; else KD_ROLL = v;
      reportKd(v);
    } else Serial.printf("!! Kd 범위 %.0f ~ %.0f\n", KD_MIN, KD_MAX);
  } else if (u.startsWith("kp")) {          // 두 축 함께. kd 보다 먼저 검사
    float v = s.substring(2).toFloat();
    if (v >= KP_MIN && v <= KP_MAX) {
      KP_PITCH = v;  KP_ROLL = v;
      reportKp();
    } else Serial.printf("!! Kp 범위 %.0f ~ %.0f\n", KP_MIN, KP_MAX);
  } else if (u.startsWith("kd")) {          // 두 축 함께
    float v = s.substring(2).toFloat();
    if (v >= KD_MIN && v <= KD_MAX) {
      KD_PITCH = v;  KD_ROLL = v;
      reportKd(v);
    } else Serial.printf("!! Kd 범위 %.0f ~ %.0f\n", KD_MIN, KD_MAX);
  } else if (u.startsWith("ag")) {          // g 보다 먼저 검사할 것
    float v = s.substring(2).toFloat();
    if (v >= 0.0f && v <= 1.5f) {
      ACC_GAIN = v;
      Serial.printf(">>> ACC_GAIN = %.2f  (합력 벡터 %s)\n",
                    ACC_GAIN, ACC_GAIN == 0.0f ? "꺼짐 — 1단계와 동일" : "켜짐");
      if (ACC_GAIN > 0.0f)
        Serial.println("   부호를 확인했습니까? 반대면 액체를 더 흔듭니다");
    } else Serial.println("!! ACC_GAIN 범위 0 ~ 1.5");
  } else if (u.startsWith("ad")) {
    float v = s.substring(2).toFloat();
    if (v > 0.0f && v <= 1.0f) { ACC_LPF = v; Serial.printf(">>> ACC_LPF = %.2f\n", ACC_LPF); }
    else Serial.println("!! ACC_LPF 범위 0 ~ 1");
  } else if (u == "dp" || u == "dr") {     // d 보다 먼저 검사할 것
    /* 부호를 뒤집으면 지령이 반대편으로 간다. 슬루(MAX_RATE)가 걸려 있어
     * 계단이 아니라 램프로 넘어가지만, 기울어져 있을수록 이동량이 크다.
     */
    bool isP = (u == "dp");
    volatile float &d = isP ? DIR_PITCH : DIR_ROLL;
    d = -d;
    Serial.printf(">>> DIR_%s = %+.0f  (%s축)\n",
                  isP ? "PITCH" : "ROLL", d, isP ? "안쪽" : "바깥");
    Serial.println("    기울였을 때 트레이가 반대로 돌면 맞습니다");
    if (armed) Serial.println("    !! ARMED 상태 — 지령이 반대편으로 이동합니다");
  } else if (u.startsWith("sra")) {
    float v = s.substring(3).toFloat();
    if (v >= 0.2f && v <= 10.0f && v > SR_END_DEG) {
      SR_ACTIVE_DEG = v;  srResetRequest = true;
      Serial.printf(">>> SOFT_RETURN active = %.2f°\n", SR_ACTIVE_DEG);
    } else Serial.println("!! sra 범위 0.2 ~ 10°, sre 보다 커야 합니다");
  } else if (u.startsWith("sre")) {
    float v = s.substring(3).toFloat();
    if (v >= 0.05f && v < SR_ACTIVE_DEG) {
      SR_END_DEG = v;  srResetRequest = true;
      Serial.printf(">>> SOFT_RETURN end = %.2f°\n", SR_END_DEG);
    } else Serial.println("!! sre 범위 0.05° 이상, sra 보다 작아야 합니다");
  } else if (u.startsWith("srd")) {
    int v = s.substring(3).toInt();
    if (v >= 10 && v <= 200) {
      SR_DWELL_MS = (uint16_t)v;  srResetRequest = true;
      Serial.printf(">>> SOFT_RETURN 종료 확인 = %ums\n", SR_DWELL_MS);
    } else Serial.println("!! srd 범위 10 ~ 200ms");
  } else if (u.startsWith("srh")) {
    int v = s.substring(3).toInt();
    if (v >= 10 && v <= 500) {
      SR_HOLD_MS = (uint16_t)v;  srResetRequest = true;
      Serial.printf(">>> SOFT_RETURN 유지 = %ums\n", SR_HOLD_MS);
    } else Serial.println("!! srh 범위 10 ~ 500ms");
  } else if (u.startsWith("srr")) {
    int v = s.substring(3).toInt();
    if (v >= 20 && v <= 1000) {
      SR_RETURN_MS = (uint16_t)v;  srResetRequest = true;
      Serial.printf(">>> SOFT_RETURN 복귀 = %ums\n", SR_RETURN_MS);
    } else Serial.println("!! srr 범위 20 ~ 1000ms");
  } else if (u.startsWith("sr")) {
    bool srOn = (s.substring(2).toInt() != 0);
    ZV_LOCK();
    SOFT_RETURN_ON = srOn;
    if (srOn) {
      ZV_ON = false;               // 두 성형기를 겹치지 않는다
      zvClear();
    }
    ZV_UNLOCK();
    srResetRequest = true;
    Serial.printf(">>> SOFT_RETURN %s  active/end=%.2f/%.2f°  dwell/hold/return=%u/%u/%ums%s\n",
                  SOFT_RETURN_ON ? "켜짐" : "꺼짐", SR_ACTIVE_DEG, SR_END_DEG,
                  SR_DWELL_MS, SR_HOLD_MS, SR_RETURN_MS,
                  srOn ? "  (ZV 자동 꺼짐)" : "");
  } else if (u.startsWith("zv")) {
    /* 문자열 변환을 락 밖에서 먼저 끝낸다. 임계구역 안에서 String 을 만들면
  } else if (u.startsWith("zv")) {
    /* 문자열 변환을 락 밖에서 먼저 끝낸다. 임계구역 안에서 String 을 만들면
     * 인터럽트가 막힌 채로 힙을 건드리게 된다.
     */
    bool zvOn = (s.substring(2).toInt() != 0);
    ZV_LOCK();
    ZV_ON = zvOn;
    if (zvOn) {
      SOFT_RETURN_ON = false;     // 두 성형기를 겹치지 않는다
      srResetRequest = true;
      zvClear();                  // 낡은 값이 섞여 첫 순간 튀는 것을 막는다
    }
    ZV_UNLOCK();
    Serial.printf(">>> ZV %s   %.2fHz  z=%.3f  %d임펄스  지연 +%.0fms\n",
                  ZV_ON ? "켜짐" : "꺼짐 — 2단계와 동일",
                  ZV_FREQ, ZV_ZETA, ZV_MODE, zvDelayMs());
  } else if (u.startsWith("zf")) {
    float v = s.substring(2).toFloat();
    if (v >= 0.8f && v <= 8.0f) {
      ZV_LOCK();  ZV_FREQ = v;  zvRecalc();  zvClear();  ZV_UNLOCK();
      Serial.printf(">>> ZV_FREQ = %.2f Hz  (주기 %.0fms, 반주기 %d샘플, 지연 +%.0fms)\n",
                    ZV_FREQ, 1000.0f / ZV_FREQ, zv_n1, zvDelayMs());
    } else Serial.println("!! ZV_FREQ 범위 0.8 ~ 8 Hz");
  } else if (u.startsWith("zd")) {
    float v = s.substring(2).toFloat();
    if (v >= 0.0f && v <= 0.9f) {
      ZV_LOCK();  ZV_ZETA = v;  zvRecalc();  ZV_UNLOCK();
      Serial.printf(">>> ZV_ZETA = %.3f  (A1=%.3f A2=%.3f A3=%.3f)\n",
                    ZV_ZETA, zv_A1, zv_A2, zv_A3);
    } else Serial.println("!! ZV_ZETA 범위 0 ~ 0.9");
  } else if (u.startsWith("zm")) {
    int v = s.substring(2).toInt();
    if (v == 2 || v == 3) {
      ZV_LOCK();  ZV_MODE = v;  zvRecalc();  zvClear();  ZV_UNLOCK();
      Serial.printf(">>> ZV_MODE = %d (%s)  지연 +%.0fms\n",
                    ZV_MODE, ZV_MODE == 3 ? "ZVD — 주파수 오차에 강하고 두 배 느리다"
                                          : "ZV — 빠르지만 주파수를 맞춰야 한다",
                    zvDelayMs());
    } else Serial.println("!! ZV_MODE 는 2 또는 3");
  } else if (u.startsWith("r")) {          // 슬루 제한 [deg/s]
    /* 가속 목표각은 0.11초에 17° 까지 오른다 (155°/s). 30°/s 로는 3.3° 밖에
     * 못 따라가므로 ACC_GAIN 을 아무리 올려도 결과가 같다. 실측으로 확인됐다.
     * 지령 변화율의 27% 가 한계에 붙어 있었다 — 수평제어만 켰을 때도 그렇다.
     */
    float v = s.substring(1).toFloat();
    if (v >= 5.0f && v <= 400.0f) {
      MAX_RATE = v;
      Serial.printf(">>> MAX_RATE = %.0f °/s  (%.0fms 안에 %.1f° 이동 가능)\n",
                    MAX_RATE, 110.0f, MAX_RATE * 0.11f);
      if (v > 200.0f)
        Serial.println("   !! 지령이 모터보다 빨라집니다. 드룹·진동을 보세요");
    } else Serial.println("!! MAX_RATE 범위 5 ~ 400");
  } else if (u.startsWith("g")) {
    float v = s.substring(1).toFloat();
    if (v >= 0.0f && v <= 1.5f) { GAIN = v; Serial.printf(">>> GAIN = %.2f\n", GAIN); }
    else Serial.println("!! GAIN 범위 0 ~ 1.5");
  } else if (u.startsWith("f")) {
    float v = s.substring(1).toFloat();
    if (v > 0.0f && v <= 1.0f) { CMD_LPF = v; Serial.printf(">>> CMD_LPF = %.2f\n", CMD_LPF); }
    else Serial.println("!! CMD_LPF 범위 0 ~ 1");
  } else if (u.startsWith("d")) {
    int v = s.substring(1).toInt();
    if (v >= 1 && v <= 20) { CAN_DIV = v; Serial.printf(">>> CAN_DIV = %d (%dHz)\n", v, 100 / v); }
    else Serial.println("!! CAN_DIV 범위 1 ~ 20");
  } else if (u == "q") {
    stream = !stream;
    Serial.printf(">>> 스트림 출력 %s\n", stream ? "ON" : "OFF");
  } else if (u == "?") {
    printStatus();
  } else {
    Serial.println("명령: arm/stop/z/t/q/g/ag/ad/kp/kd/f/r/d/dp/dr/zv/zf/zd/zm/sr/sra/sre/srd/srh/srr/?");
  }
}


void ctrlTask(void *arg);
void ioTask(void *arg);

// =============================================================================
//  setup
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // ── I2C ──
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire.setTimeOut(50);            // 응답 없을 때 멈추지 않게
  delay(100);

  uint8_t id = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I: 0x%02X (기대값 0x%02X)  주소 0x%02X\n",
                id, WHO_AM_I_VAL, IMU_ADDR);
  imu_ok = (id == WHO_AM_I_VAL);
  if (!imu_ok) {
    Serial.println("!! IMU 응답 없음 — CS→3V3, SDO→GND 인지, SDA/SCL 풀업 확인");
  }
  configSensor();

  // ── 자이로 영점 + 중력 스케일 ──
  Serial.println(">>> 캘리브레이션 중... 움직이지 마세요");
  long  sgx = 0, sgy = 0;
  float smag = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    sgx += gx;  sgy += gy;
    float fax = ax, fay = ay, faz = az;
    smag += sqrtf(fax * fax + fay * fay + faz * faz) / G_LSB;
    delay(5);
  }
  gyro_bias_x = (float)sgx / 200.0;
  gyro_bias_y = (float)sgy / 200.0;
  float measured = smag / 200.0;
  if (measured > 0.5f && measured < 1.5f) g_scale = measured;
  Serial.printf("gyro bias x:%.1f y:%.1f   g_scale:%.4f   I2C오류 %lu회\n",
                gyro_bias_x, gyro_bias_y, g_scale, i2c_err);

  {
    int16_t ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);
    pitch_filtered = atan2((float)ay,  sqrt(pow((float)ax, 2) + pow((float)az, 2))) * (180.0 / PI);
    roll_filtered  = atan2((float)-ax, sqrt(pow((float)ay, 2) + pow((float)az, 2))) * (180.0 / PI);
    force_ax = (float)ax;  force_ay = (float)ay;  force_az = (float)az;
    force_pitch = pitch_filtered;
    force_roll  = roll_filtered;
  }
  accelGateReset();

  // ── CAN ──
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g, &t, &f);
  twai_start();
  delay(100);
  broadcastUniversal(0xFD);        // 부팅 시 비활성

  zvRecalc();
  lastLoop = micros();
  Serial.println("\n==================================================");
  Serial.println(" 2단계 — 합력 벡터 목표각");
  Serial.println(" IMU 는 I2C, 모터는 MIT 모드입니다.");
  Serial.println(" 모터는 비활성 상태입니다. 'arm' 을 입력해야 움직입니다.");
  Serial.println("--------------------------------------------------");
  Serial.println("  arm     활성화        stop  비활성화");
  Serial.println("  t       시험 구동     z     영점 재설정");
  Serial.println("  g<값>   수평 게인     예) g1.0");
  Serial.printf ("  ag<값>  합력 게인     예) ag0.3   현재 %.2f%s\n",
                 ACC_GAIN, ACC_GAIN == 0.0f ? "  ← 0 이면 1단계와 동일" : "");
  Serial.println("  ad<값>  합력 필터     예) ad0.2");
  Serial.println("  kp<값>  모터 강성     kd<값>  모터 감쇠   (두 축 함께)");
  Serial.printf ("  kpp/kpr kdp/kdr 축별   현재 Kp %.2f/%.2f  Kd %.3f/%.3f (안/바깥)\n",
                 KP_PITCH, KP_ROLL, KD_PITCH, KD_ROLL);
  Serial.println("  f<값>   지령 필터     d<값>   CAN 분주     ?  상태");
  Serial.printf ("  r<값>   슬루 제한     예) r120    현재 %.0f °/s\n", MAX_RATE);
  Serial.printf ("  zv1/zv0 ZV 켜기/끄기  zf<값> 주파수  zd<값> 감쇠비  zm<2|3>\n");
  Serial.printf ("          현재 %s  %.2fHz  z=%.3f  %d임펄스  지연 +%.0fms\n",
                 ZV_ON ? "켜짐" : "꺼짐 ← 꺼두면 2단계와 동일",
                 ZV_FREQ, ZV_ZETA, ZV_MODE, zvDelayMs());
  Serial.printf ("  dp / dr 부호 뒤집기   현재 %+.0f / %+.0f  (안쪽 / 바깥)\n",
                 DIR_PITCH, DIR_ROLL);
  Serial.println("==================================================\n");

  /* 여기부터 두 태스크가 돈다. 큐를 먼저 만들어야 qprintf 가 큐로 간다.
   * ctrlTask 를 코어1, ioTask 를 코어0 에 붙인다. WiFi 를 안 쓰므로 코어0 이
   * 통째로 비어 있다. 시리얼이 아무리 막혀도 코어1 의 제어는 영향을 안 받는다.
   */
  qEvt  = xQueueCreate(12, EVT_LEN);
  qSnap = xQueueCreate(32, sizeof(Snap));
  qCmd  = xQueueCreate(6,  sizeof(uint8_t));
  /* 스택 6144: qprintf 안의 vsnprintf 가 %f 와 한글을 다루느라 1KB 남짓 쓴다.
   * 넘치면 재현이 어려운 간헐적 리셋으로 나타나므로 여유를 둔다. RAM 은 7% 뿐이다.
   */
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 6144, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(ioTask,   "io",   6144, NULL, 2, NULL, 0);
}


// =============================================================================
//  loop
// =============================================================================
void ctrlTask(void *arg) {
  /* vTaskDelayUntil 은 "마지막에 깬 시각 + 10ms" 에 깨운다. 앞의 일이
   * 오래 걸려도 주기가 밀리지 않는다. dt = 0.01 이 이제 가정이 아니라
   * OS 가 지키는 약속이 된다.
   */
  TickType_t xLastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(10));

    /* 긴 블로킹 명령은 여기서 실행한다. ioTask 에서 돌리면 arm 이 걸리는 250ms
     * 동안 이 태스크가 CAN 지령을 계속 쏜다. 여기서 돌리면 원본처럼 함께 멈춘다.
     * 큐로 받으므로 연속 입력이 덮어써지거나 사라지지 않는다.
     */
    uint8_t req;
    while (xQueueReceive(qCmd, &req, 0) == pdTRUE) {
      /* ctrlBusy 는 arm 과 t 가 건 것이므로 그 둘이 끝날 때만 푼다.
       * 공통으로 풀면 큐에 [z, arm] 이 들어 있을 때 z 가 먼저 풀어버려서
       * arm 이 도는 250ms 동안 파서가 다시 열린다.
       */
      switch (req) {
        case 1: motorsArm();  ctrlBusy = false;  break;
        case 2: motorsDisarm("사용자 명령"); break;
        case 3: motorsDisarm("시험 구동 진입");
                motorSweep();  ctrlBusy = false;  break;
        case 4: broadcastUniversal(0xFE);
                cmd_pitch = 0;  cmd_roll = 0;
                qprintln(">>> 모터 영점 재설정");
                break;
      }
    }

    /* ioTask 는 설정만 바꾸고, 축별 상태는 ctrlTask 하나만 초기화·갱신한다. */
    if (srResetRequest || !SOFT_RETURN_ON) {
      softReturnReset();
      srResetRequest = false;
    }

    /* 여기서부터 잰다. arm 의 delay 250ms 는 원본에도 있는 의도된 정지라
     * 같이 세면 "제어 틱이 늦었나"를 볼 수가 없다 (실측 270ms 로 찍혔다).
     */
#if RTOS_DIAG
    uint32_t dgT0 = micros();
#endif

    // ── 1. 자세 추정 ──
    int16_t ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);

    float rate_pitch = ((float)gx - gyro_bias_x) / GYRO_LSB;
    float rate_roll  = ((float)gy - gyro_bias_y) / GYRO_LSB;

    float pitch_acc = atan2((float)ay,  sqrt(pow((float)ax, 2) + pow((float)az, 2))) * (180.0 / PI);
    float roll_acc  = atan2((float)-ax, sqrt(pow((float)ay, 2) + pow((float)az, 2))) * (180.0 / PI);

    /* 합력 벡터를 성분 상태에서 한 번만 거른 뒤 각도로 바꾼다. 각 축의 각도를
     * 따로 거르면 두 축이 동시에 움직일 때 벡터 길이와 방향이 어긋난다.
     */
    force_ax = ACC_LPF * (float)ax + (1.0f - ACC_LPF) * force_ax;
    force_ay = ACC_LPF * (float)ay + (1.0f - ACC_LPF) * force_ay;
    force_az = ACC_LPF * (float)az + (1.0f - ACC_LPF) * force_az;
    force_pitch = atan2f(force_ay,  sqrtf(force_ax * force_ax + force_az * force_az)) * RAD_TO_DEG;
    force_roll  = atan2f(-force_ax, sqrtf(force_ay * force_ay + force_az * force_az)) * RAD_TO_DEG;

    float fax = ax, fay = ay, faz = az;
    float a_mag = sqrtf(fax * fax + fay * fay + faz * faz) / G_LSB / g_scale;

    // 1g 에서 벗어난 만큼 가속도계 반영 비중을 줄인다
    float dev   = fabsf(a_mag - 1.0f);
    float trust = constrain(1.0f - dev / ACC_TRUST_FALLOFF, 0.0f, 1.0f);
    float alpha = 1.0f - (1.0f - alpha_cf) * trust;

    /* 자이로 예측과 가속도계 합력각이 크게 다르면 그 차이는 선형가속일 수 있다.
     * 그동안 가속도계로 자세를 보정하면 수평가속을 차체 기울기로 학습하므로,
     * 해당 축은 자이로 예측만 사용한다.
     */
    float predicted_pitch = pitch_filtered + rate_pitch * dt;
    float predicted_roll  = roll_filtered  + rate_roll  * dt;
    bool block_acc_pitch = accelGateStep(predicted_pitch - pitch_acc, accGatePitch);
    bool block_acc_roll  = accelGateStep(predicted_roll  - roll_acc,  accGateRoll);

    float alpha_pitch = block_acc_pitch ? 1.0f : alpha;
    float alpha_roll  = block_acc_roll  ? 1.0f : alpha;
    pitch_filtered = alpha_pitch * predicted_pitch + (1.0f - alpha_pitch) * force_pitch;
    roll_filtered  = alpha_roll  * predicted_roll  + (1.0f - alpha_roll)  * force_roll;

    n_win++;
    if (trust <= 0.0f) { err_win++; rej_run++; } else rej_run = 0;

    // ── 2. 이상 감지 ──
    if (a_mag < ACC_MAG_DEAD && readReg(REG_CTRL1_XL) != CTRL1_XL_VAL) {
      err_reset++;
      qprintln("!! 센서 응답 없음 — 재설정");
      configSensor();
      motorsDisarm("센서 리셋");
    }
    if (rej_run > REJ_RUN_FAULT) {
      imu_ok = false;
      motorsDisarm("IMU 무보정 구간 과다");
    }

    drainCAN();

    // ── 3. 합력 벡터 목표각 ──
    /* 합력각을 직접 목표로 쓴다. 중력을 빼서 선형가속을 복원한 뒤 다시 각도로
     * 바꾸던 방식을 대체한다.
     *
     *   want = ACC_GAIN·ref − GAIN·θ_base,   ref = θ_base − θ_force
     *        = −θ_force                       (GAIN = ACC_GAIN 일 때)
     *
     * θ_base 가 소거되므로 자세 추정 오차가 지령에 닿지 않는다. 예전 방식도
     * 1차 근사로는 같은 결과였지만, one_g 를 상수로 쓰기 때문에 가속 중
     * 비력의 크기가 g 가 아니라는 점에서 오차가 남았다.
     *
     *   차체 20° 기울고 0.3g 가속 :  정답 −16.70°,  예전 −15.74°  (0.96° 부족)
     *   차체 30° 기울고 0.3g 가속 :  정답 −16.70°,  예전 −14.56°  (2.14° 부족)
     *
     * 합력각 방식은 크기를 쓰지 않고 방향만 보므로 모든 각도에서 정확하고,
     * g_scale 오차에도 영향을 받지 않는다.
     *
     * raw 는 필터 전(pitch_acc), ref 는 3축 벡터 필터 뒤(force_pitch)를 쓴다.
     * SR 의 종료 판정은 빨라야 하므로 필터 전 값을 본다.
     */
    float raw_ref_p = (pitch_filtered - pitch_acc) * DIR_ACC;
    float raw_ref_r = (roll_filtered  - roll_acc)  * DIR_ACC;
    ref_pitch = (pitch_filtered - force_pitch) * DIR_ACC;
    ref_roll  = (roll_filtered  - force_roll)  * DIR_ACC;

    raw_ref_p = constrain(raw_ref_p, -ACC_REF_LIMIT, ACC_REF_LIMIT);
    raw_ref_r = constrain(raw_ref_r, -ACC_REF_LIMIT, ACC_REF_LIMIT);
    ref_pitch = constrain(ref_pitch, -ACC_REF_LIMIT, ACC_REF_LIMIT);
    ref_roll  = constrain(ref_roll,  -ACC_REF_LIMIT, ACC_REF_LIMIT);

    /* ── 3.5. ZV 입력성형 ──
     * 링 버퍼에 목표각을 쌓고, 반주기 전 값을 섞는다. 꺼져 있으면 그대로 통과.
     * 두 축 모두에 건다 — 이전 서보 구조는 pitch 만 성형하고 roll 버퍼는
     * 선언만 해둔 채 쓰지 않았다.
     */
    /* 잠그는 이유: zf/zd/zm 명령이 ioTask 에서 zv_n1/A1/A2/A3 를 묶어서 바꾸고,
     * zvClear 는 링버퍼를 통째로 지운다. 중간에 읽으면 섞인 조합이 나온다.
     * 계산식 자체는 원본 그대로다.
     */
    ZV_LOCK();
    zv_head = (zv_head + 1) % ZV_BUF;
    zv_p[zv_head] = ref_pitch;
    zv_r[zv_head] = ref_roll;

    if (SOFT_RETURN_ON) {
      shaped_pitch = softReturnStep(ref_pitch, raw_ref_p, srPitch);
      shaped_roll  = softReturnStep(ref_roll,  raw_ref_r, srRoll);
    } else if (ZV_ON) {
      int i1 = (zv_head - zv_n1       + ZV_BUF) % ZV_BUF;
      int i2 = (zv_head - 2 * zv_n1   + ZV_BUF) % ZV_BUF;
      shaped_pitch = zv_A1 * zv_p[zv_head] + zv_A2 * zv_p[i1] + zv_A3 * zv_p[i2];
      shaped_roll  = zv_A1 * zv_r[zv_head] + zv_A2 * zv_r[i1] + zv_A3 * zv_r[i2];
    } else {
      shaped_pitch = ref_pitch;
      shaped_roll  = ref_roll;
    }
    ZV_UNLOCK();

    // ── 4. 제어  motor_cmd = θ_ref − θ_base ──
    /* ACC_GAIN 이 0 이면 θ_ref 항이 사라져 1단계(−θ_base)와 완전히 같아진다.
     * 모터는 차체 기준으로 돌므로 절대 목표각에서 차체 자세를 뺀다.
     */
    float want_pitch = ACC_GAIN * shaped_pitch - pitch_filtered * GAIN;
    float want_roll  = ACC_GAIN * shaped_roll  - roll_filtered  * GAIN;

    want_pitch = constrain(want_pitch, -LIMIT_DEG, LIMIT_DEG);
    want_roll  = constrain(want_roll,  -LIMIT_DEG, LIMIT_DEG);

    static float lpf_pitch = 0, lpf_roll = 0;
    lpf_pitch = CMD_LPF * want_pitch + (1.0f - CMD_LPF) * lpf_pitch;
    lpf_roll  = CMD_LPF * want_roll  + (1.0f - CMD_LPF) * lpf_roll;

    const float maxStep = MAX_RATE * dt;
    cmd_pitch = slew(lpf_pitch, cmd_pitch, maxStep);
    cmd_roll  = slew(lpf_roll,  cmd_roll,  maxStep);

    static int div_cnt = 0;
    if (armed && (++div_cnt >= CAN_DIV)) {
      div_cnt = 0;
      sendBoth();
    }

    // ── 5. 출력 ──
    /* t   : 펌웨어 시각. PC 수신 시각에는 USB 지터가 섞여 변화율 계산에 못 쓴다.
     * pre : 슬루 제한을 걸기 전의 지령(slew 의 입력).
     *       cmd 와 같으면 안 걸린 것, 다르면 걸린 것이다. 차이가 잘려나간 양이고
     *       pre 의 변화율이 실제로 필요한 MAX_RATE 다.
     *       ZV 성형은 계단을 만드니 이 값이 특히 중요하다 — 슬루에 걸리면
     *       성형이 무너진다 (비선형이라 상쇄가 깨진다).
     * 주기는 조건이 > 20 이라 10ms 루프에서 30ms 마다 나간다. 50Hz 가 아니라 33Hz.
     */
    if (stream && millis() - lastPrint > 20) {      // 33Hz
      lastPrint = millis();
      /* 숫자만 담아 넘긴다. 포맷(%.2f 13개, 수백 us)은 ioTask 가 대신 한다.
       * 큐가 가득 차면 버린다 — 로그를 지키자고 제어를 세우지 않는다.
       * 버려지면 t 값이 30ms 씩 건너뛴 것으로 로그에 그대로 보인다.
       */
      Snap sn;
      sn.t     = millis();
      /* raw : ACC_LPF 를 걸기 전. 이게 없으면 지연을 모터 구간(cmd→act)밖에
       * 못 재는데, 실측하니 앞단 필터가 ACC_LPF 40ms + CMD_LPF 50ms 로
       * 모터(85ms)보다 크다. 슬로싱 위상은 물리 가속부터 재야 뜻이 있으므로
       * 사슬의 원점을 로그에 남긴다.
       * raw_ref_p/r 은 같은 틱 안의 지역변수라 여기서 그대로 보인다.
       */
      sn.raw_p = raw_ref_p;       sn.raw_r = raw_ref_r;
      sn.pre_p = lpf_pitch;       sn.pre_r = lpf_roll;
      sn.pitch = pitch_filtered;  sn.ref_p = ref_pitch;
      sn.zv_p  = shaped_pitch;    sn.cmd_p = cmd_pitch;
      sn.act_p = act_deg[CAN_ID_PITCH & 0x03] * DIR_PITCH;
      sn.roll  = roll_filtered;   sn.ref_r = ref_roll;
      sn.zv_r  = shaped_roll;     sn.cmd_r = cmd_roll;
      sn.act_r = act_deg[CAN_ID_ROLL  & 0x03] * DIR_ROLL;
      if (xQueueSend(qSnap, &sn, 0) != pdTRUE) dgSnapDrop = dgSnapDrop + 1;
    }

    /* configSensor 는 정상 실행만으로 150ms 를 쓰고, I2C 는 응답이 없으면
     * 거기에 최대 50ms 를 더 멈춘다. arm/sweep 뒤에만 시계를 맞추면 이런 경우를
     * 놓쳐서, vTaskDelayUntil 이 밀린 틱을 10ms 간격 없이 연속으로 따라잡는다.
     * 그러면 제어 계산과 CAN 전송이 한꺼번에 몰린다. 그래서 매 틱 끝에서 잰다.
     */
    if ((TickType_t)(xTaskGetTickCount() - xLastWake) >= pdMS_TO_TICKS(20)) {
      xLastWake = xTaskGetTickCount();
      dgResync = dgResync + 1;
    }
#if RTOS_DIAG
    uint32_t dgEl = micros() - dgT0;
    if (dgEl > dgMaxUs) dgMaxUs = dgEl;
    if (dgEl > 10000)   dgOver = dgOver + 1;
#endif

    if (millis() - lastCheck > 2000) {
      lastCheck = millis();
      uint8_t c1 = readReg(REG_CTRL1_XL);
      if (c1 != CTRL1_XL_VAL) {
        err_reset++;
        qprintf("!! 센서 리셋 CTRL1_XL=0x%02X — 재설정\n", c1);
        configSensor();
        motorsDisarm("센서 리셋");
      }
      qprintf("[stat] %s Kp=%.1f Kd=%.2f 모터P=%s R=%s 피드백:%lu | 무보정:%lu/%lu I2C오류:%lu\n",
                    armed ? "ARMED" : "STOP", KP_PITCH, KD_PITCH,
                    statusName(last_status[CAN_ID_PITCH & 0x03]),
                    statusName(last_status[CAN_ID_ROLL  & 0x03]),
                    fb_count, err_win, n_win, i2c_err);
#if RTOS_DIAG
      qprintf("[rtos] 최대틱 %luus  10ms초과 %lu  재동기 %lu | "
              "TX실패 %lu 샘플드롭 %lu 이벤트드롭 %lu 명령드롭 %lu\n",
              dgMaxUs, dgOver, dgResync, dgTxFail, dgSnapDrop, dgEvtDrop, dgCmdDrop);
      dgMaxUs = 0;
#endif
      n_win = 0;  err_win = 0;
    }
  }
}

void ioTask(void *arg) {
  char line[96];
  size_t n = 0;
  char txt[EVT_LEN];
  Snap sn;

  for (;;) {
    // 이벤트를 먼저 비운다 — 중요 메시지가 샘플 뒤에 줄서지 않게
    while (xQueueReceive(qEvt, txt, 0) == pdTRUE) Serial.print(txt);

    /* 샘플이 없으면 5ms 기다린다. while(!Serial.available()){} 같은 바쁜
     * 대기를 만들면 IDLE 태스크가 굶어 워치독이 보드를 리셋시킨다.
     */
    if (xQueueReceive(qSnap, &sn, pdMS_TO_TICKS(5)) == pdTRUE) {
      Serial.printf("t:%lu,raw_pitch:%.2f,raw_roll:%.2f,"
                    "pre_pitch:%.2f,pre_roll:%.2f,"
                    "pitch:%.2f,ref_pitch:%.2f,zv_pitch:%.2f,cmd_pitch:%.2f,act_pitch:%.2f,"
                    "roll:%.2f,ref_roll:%.2f,zv_roll:%.2f,cmd_roll:%.2f,act_roll:%.2f\n",
                    sn.t, sn.raw_p, sn.raw_r, sn.pre_p, sn.pre_r,
                    sn.pitch, sn.ref_p, sn.zv_p, sn.cmd_p, sn.act_p,
                    sn.roll,  sn.ref_r, sn.zv_r, sn.cmd_r, sn.act_r);
    }

    /* 문자 단위로 모은다. readStringUntil 은 개행이 올 때까지 기본 1초를
     * 기다리므로, 터미널이 글자를 나눠 보내기만 해도 그동안 큐를 못 비운다.
     * 32칸 샘플 큐는 33Hz 에서 1초면 가득 찬다. 명령·응답 내용은 원본과 같다.
     */
    while (!ctrlBusy && Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (n) { line[n] = 0;  n = 0;  handleLine(line); }
      } else if (n < sizeof(line) - 1) {
        line[n++] = c;
      }
    }
  }
}

void loop() {
  /* 아두이노 loopTask 는 태스크 워치독에게 살아있다고 보고하는 역할도 겸한다.
   * 지우면 리셋이 걸릴 수 있어 껍데기로 남긴다.
   */
  vTaskDelay(pdMS_TO_TICKS(1000));
}

/* -----------------------------------------------------------------------------
 * 부호 확인 절차 — ACC_GAIN 을 올리기 전에 반드시
 *
 * 합력 목표각의 부호가 반대면 가속할 때 액체를 **더** 흔드는 쪽으로 기울인다.
 * 수평제어는 틀려도 트레이가 반대로 기우는 것으로 금방 보이지만, 이건
 * 가속하는 순간에만 나타나므로 눈으로 알아채기 어렵다. 숫자로 확인한다.
 *
 * ── 1) 모터 없이 부호만 본다 ────────────────────────────────────────────
 *
 * arm 하지 않은 상태로 IMU 를 책상 위에서 한 방향으로 쭉 민다.
 * 출력의 ref_pitch / ref_roll 을 본다. ACC_GAIN 이 0 이어도 계산되어 찍힌다.
 *
 *   pitch:0.10,ref_pitch:-4.20,cmd_pitch:0.00,...
 *                    ↑ 여기
 *
 * 판정 기준 — 액체가 쏠리는 반대쪽으로 기울여야 한다.
 *
 *   보드의 +Y 축 방향으로 밀었을 때
 *     ref_pitch 가 음수  →  +Y 쪽이 내려감  →  맞다   (DIR_ACC = +1 유지)
 *     ref_pitch 가 양수  →  틀리다. DIR_ACC 를 −1 로
 *
 * 왜 음수여야 하는가. 컵을 +Y 로 가속하면 물은 −Y 쪽으로 쏠린다. 수면을
 * 컵 기준으로 평평하게 두려면 컵의 +Y 쪽을 내려야 한다. 우리 규약에서
 * +pitch 는 +Y 가 올라가는 것이므로 목표는 음수가 된다.
 *
 * roll 도 같은 방식으로 +X 방향으로 밀어 확인한다.
 * (+roll 은 +X 가 내려가는 것이므로 부호 방향이 pitch 와 반대다)
 *
 * ── 2) 정지 상태에서 0 근처인지 ────────────────────────────────────────
 *
 * 책상에 가만히 두면 ref 가 0 근처여야 한다. 계속 치우쳐 있으면 중력 성분을
 * 덜 뺀 것이므로 g_scale 과 θ_base 를 먼저 의심한다.
 *
 * 기울여 놓고 가만히 두었을 때도 0 근처여야 한다. 기울기만으로 ref 가
 * 생긴다면 중력 제거가 잘못된 것이다. 이 경우 ACC_GAIN 을 올리면 안 된다.
 *
 * ── 3) 게인을 조금씩 ───────────────────────────────────────────────────
 *
 *   ag0.3   →  ag0.6  →  ag1.0
 *
 * 각 단계에서 IMU 를 밀었다 세웠다 하며 트레이가 가속 반대쪽으로 기우는지 본다.
 * 소리·진동이 커지면 한 단계 내린다.
 *
 * ── 되돌리기 ───────────────────────────────────────────────────────────
 *
 *   ag0     합력 항이 사라져 1단계(수평 유지)와 완전히 같아진다.
 *
 * 무슨 일이 생기면 stop 보다 ag0 이 먼저다. 수평제어는 계속 살아 있다.
 *
 * -----------------------------------------------------------------------------
 * 아직 없는 것 — 3단계에서
 *
 * ZV 입력성형이 없다. 지금은 θ_ref 가 바뀌는 즉시 그대로 따라가므로, 목표각의
 * 급변 자체가 슬로싱을 새로 만들 수 있다. 3단계에서 θ_ref 에 Convolved ZV 를
 * 걸어 그 성분을 없앤다.
 *
 * 그리고 지연이 150~180ms 로 측정돼 있다. 슬로싱 주기 420ms 의 36% 라
 * 위상 여유가 부족하다. CMD_LPF·슬루·Kp 를 손봐야 할 수 있다.
 * ---------------------------------------------------------------------------*/
