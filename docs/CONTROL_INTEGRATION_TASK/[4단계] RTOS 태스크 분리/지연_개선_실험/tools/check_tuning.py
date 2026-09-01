"""
r / kp / kd 가 잘 잡혔는지 로그로 판정한다.

합력벡터_제어.ino + capture.py 로 뜬 로그를 읽어 아래만 본다.
모델도 격자탐색도 없다. 단순 통계다.

    드룹        act 가 cmd 의 몇 %까지 가는가        -> kp 가 충분한가
    슬루 클리핑  지령이 잘려나간 시간 비율            -> r 이 충분한가
    요구 변화율  실제로 필요했던 도/초                -> r 을 얼마로 올려야 하나
    추종오차    cmd 와 act 의 차이 (RMS / 최대)
    떨림        정지 구간에서 act 가 흔들리는 정도    -> kd 가 충분한가

여러 로그를 한 번에 주면 표로 비교한다. 그게 이 도구의 핵심이다.

---------------------------------------------------------------------------
쓰는 법

    python check_tuning.py log_kp10_r60.txt
    python check_tuning.py log_*.txt                 <- 설정별 비교
    python check_tuning.py log_*.txt --axis pitch

로그 뜨는 법

    시리얼 모니터에서 설정을 바꾼다      예: r60  kp2.0  kd0.14
    모니터를 닫는다                      (포트를 하나만 점유할 수 있다)
    python capture.py --port COM? --cmd arm --post 20 --out log_kp20_r60.txt
    기록되는 20초 동안 차체(또는 IMU)를 손으로 흔든다

    파일명에 설정을 적어두면 표에서 알아보기 쉽다.

---------------------------------------------------------------------------
pre_* 가 있어야 슬루를 정확히 본다

    합력벡터_제어.ino 의 출력에 t / pre_pitch / pre_roll 이 추가돼 있어야 한다
    (stream_log_추가.patch 참고). 없으면 드룹·떨림만 보고 슬루는 건너뛴다.

    pre 는 슬루 제한을 걸기 전의 지령이다.
        cmd == pre  ->  안 걸림
        cmd != pre  ->  걸림. 차이가 잘려나간 양
    이 판정은 표본 주기와 무관해서 정확하다.
"""

import argparse
import glob
import math
import os
import re
import sys

try:
    import numpy as np
except ImportError:
    sys.exit("numpy 가 필요합니다:  pip install numpy")

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


# capture.py 형식:  "+12.34 [POST] t:123,pre_pitch:...."
# 시리얼 모니터 형식: "t:123,pre_pitch:...."          (앞에 아무것도 없음)
# 둘 다 받는다. 모니터에서 그냥 복사해 붙여넣은 파일도 그대로 분석된다.
RE_LINE = re.compile(r"^\s*([+-]?\d+\.\d+)\s+\[(\w+)\]\s+(.*)$")
RE_PLAIN = re.compile(r"^\s*(t:\d+,.*)$")
RE_PAIR = re.compile(r"([A-Za-z_]+)\s*:\s*(-?\d+(?:\.\d+)?)")
RE_STAT = re.compile(r"Kp\s*=\s*([\d.]+)\s+Kd\s*=\s*([\d.]+)")
# 축별 게인. 4단계 펌웨어는 kpp/kpr/kdp/kdr 로 두 축을 따로 두고, 바꿀 때
# ">>> Kp  안쪽 2.00  바깥 6.00  [N/rad]" 처럼 나란히 찍는다.
# 2초마다 나오는 [stat] 줄에는 **안쪽 값만** 들어 있어서, 그것만 읽으면
# 바깥축 게인을 안쪽 것으로 잘못 표시한다 (실제로 kdr0.25 캡처가 0.13 으로 찍혔다).
RE_KP2 = re.compile(r"Kp\s+안쪽\s+([\d.]+)\s*/?\s*바깥\s+([\d.]+)")
# arm 배너에서 게인을 뽑는다. want 를 재구성하려면 GAIN·ACC_GAIN·상한이 필요하다.
#   >>> ARMED  GAIN=1.00  ACC_GAIN=1.00  Kp=2.0  Kd=0.13  제한 ±25°
RE_ARM = re.compile(r"ARMED\s+GAIN=([\d.]+)\s+ACC_GAIN=([\d.]+).*?±([\d.]+)")
RE_KD2 = re.compile(r"Kd\s+안쪽\s+([\d.]+)\s*/?\s*바깥\s+([\d.]+)")

# 판정 기준
DROOP_OK   = 0.90     # act/cmd 가 이보다 낮으면 kp 부족
CLIP_OK    = 0.05     # 슬루에 걸린 비율이 이보다 크면 r 부족
JITTER_OK  = 0.10     # 정지 구간 act 표준편차 [deg]. 넘으면 kd 부족
STATIC_WIN = 0.30     # 정지로 판정할 창 [s]
STATIC_TOL = 0.20     # 그 창 안에서 cmd 변동이 이보다 작으면 정지 [deg]
STATIC_MIN_RUN = 0.6  # 이보다 짧은 정지 구간은 쓰지 않는다 [s]
SETTLE_SKIP = 0.6     # 정지 구간의 앞 60% 는 정착 중이라 버린다
JITTER_HP_S = 0.30    # 떨림 측정용 고역통과 창 [s]. 이보다 느린 것은 떨림이 아니다
CLIP_TOL   = 0.02     # cmd 와 pre 의 차이가 이보다 크면 잘린 것 (출력이 소수 2자리)
MIN_SPAN   = 1.0      # 지령 진폭이 이보다 작으면 판정 불가 [deg]
DROOP_MIN_HOLD = 5.0  # 유지 각도가 한 가지뿐일 때, 그 각도가 이보다 작으면 드룹 무효
LAG_GRID_MS = 5.0     # 지연 탐색 격자. 로그가 30ms 라 보간해야 이 해상도가 나온다
LAG_MAX_MS  = 400.0   # 탐색 상한
LAG_OK_MS   = 100.0   # 이보다 크면 슬로싱 주기 대비 위상이 위험하다
SLOSH_MS    = 456.0   # 슬로싱 1차 모드 주기. 용기 11x11cm, 물 350mL, f=2.19Hz


def parse(path):
    """로그 한 개를 읽어 {채널: 배열} 과 Kp/Kd 를 돌려준다.

    capture.py 는 구간마다 태그를 붙인다. POST 만 써야 한다.

      PRE  : arm 전이다. 모터가 꺼져 있어 act 가 0 으로 고정된다
      POST : arm 후 기록 구간                                <- 이것만 쓴다
      END  : stop 후다. 명령이 끊기면 피드백도 끊겨서
             act 가 마지막 값에 얼어붙는다

    섞으면 드룹이 완전히 틀린 값으로 나온다. 실제로 이 로그에서
    걸러내기 전 61%, 걸러낸 뒤 값이 달라졌다.
    """
    rows, pc_t, tags = [], [], []
    kp = kd = None                # [stat] 에서 온 값 = 안쪽(pitch)
    kp2 = kd2 = None              # (안쪽, 바깥) 쌍. 축별 명령을 쓴 로그에만 있다
    g_lv = g_acc = lim = None     # arm 배너의 GAIN / ACC_GAIN / 상한
    armed = False                 # 모니터 로그용. ARMED ~ DISARM 사이만 쓴다
    saw_arm = False
    with open(path, encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.rstrip("\n")

            # 모터가 켜진 구간 표시 (모니터 로그에는 태그가 없으므로 이것으로 자른다)
            if ">>> ARMED" in line:
                armed, saw_arm = True, True
                ma = RE_ARM.search(line)
                if ma:
                    g_lv, g_acc, lim = (float(ma.group(1)), float(ma.group(2)),
                                        float(ma.group(3)))
                continue
            if "DISARM" in line or "arm 거부" in line:
                armed = False
                continue

            m = RE_LINE.match(line)
            if m:
                tag, body = m.group(2), m.group(3)
                t_pc = float(m.group(1))
            else:
                mp = RE_PLAIN.match(line)
                if not mp:
                    continue
                tag, body, t_pc = "MON", mp.group(1), 0.0

            m2 = RE_KP2.search(body)
            if m2:
                kp2 = (float(m2.group(1)), float(m2.group(2)))
                continue
            m2 = RE_KD2.search(body)
            if m2:
                kd2 = (float(m2.group(1)), float(m2.group(2)))
                continue
            ms = RE_STAT.search(body)
            if ms:
                kp, kd = float(ms.group(1)), float(ms.group(2))
                continue
            pairs = RE_PAIR.findall(body)
            if len(pairs) < 4:
                continue
            rows.append({k: float(v) for k, v in pairs})
            pc_t.append(t_pc)
            tags.append(tag if tag != "MON" else ("MON_ON" if armed else "MON_OFF"))

    if not rows:
        return None

    seen = sorted(set(tags))
    if "POST" in seen:                       # capture.py 로그: POST 만
        keep = [i for i, g in enumerate(tags) if g == "POST"]
    elif saw_arm:                            # 모니터 로그: ARMED~DISARM 만
        keep = [i for i, g in enumerate(tags) if g == "MON_ON"]
    else:
        keep = list(range(len(rows)))
    rows = [rows[i] for i in keep]
    pc_t = [pc_t[i] for i in keep]
    tags = [tags[i] for i in keep]
    if not rows:
        return None
    # 깨진 줄 하나 때문에 채널 전체를 버리면 안 된다.
    # 예전에는 모든 줄의 교집합을 썼는데, USB 로 5줄이 깨지자 t·pre_*·roll 이
    # 통째로 사라져 1111줄짜리 로그에서 드룹 하나밖에 못 냈다.
    # 가장 흔한 필드 조합을 정상으로 보고, 그걸 다 갖춘 줄만 남긴다.
    from collections import Counter
    keys = set(Counter(frozenset(r) for r in rows).most_common(1)[0][0])
    good = [i for i, r in enumerate(rows) if keys <= set(r)]
    dropped = len(rows) - len(good)
    rows = [rows[i] for i in good]
    pc_t = [pc_t[i] for i in good]
    chan = {k: np.array([r[k] for r in rows], float) for k in keys}

    # 시간축 — 펌웨어 t(ms) 가 있으면 그것을 쓴다. PC 시각에는 USB 지터가 섞인다.
    if "t" in chan:
        t = (chan["t"] - chan["t"][0]) / 1000.0
        src = "펌웨어"
    else:
        t = np.array(pc_t) - pc_t[0]
        src = "PC(지터 있음)"
    return dict(t=t, chan=chan, kp=kp, kd=kd, kp2=kp2, kd2=kd2,
                gain=g_lv, acc_gain=g_acc, limit=lim,
                tsrc=src, n=len(rows), tags=seen, dropped=dropped)


def gains(d, axis):
    """그 축의 (Kp, Kd). 축별 값이 로그에 있으면 그것을 쓴다."""
    i = 0 if axis == "pitch" else 1        # pitch = 안쪽, roll = 바깥
    kp = d["kp2"][i] if d.get("kp2") else d.get("kp")
    kd = d["kd2"][i] if d.get("kd2") else d.get("kd")
    return kp, kd


def static_runs(t, cmd):
    """지령이 거의 안 변하는 연속 구간들을 (시작, 끝) 로 돌려준다.

    구간의 앞부분은 아직 정착 중이라 버린다. 이걸 안 하면 흔들리는 중의
    값이 섞여서 드룹은 실제보다 좋게, 떨림은 크게 나온다.
    """
    m = np.zeros(len(t), bool)
    j = 0
    for i in range(len(t)):
        while t[i] - t[j] > STATIC_WIN:
            j += 1
        if i - j >= 3:
            seg = cmd[j:i + 1]
            m[i] = (seg.max() - seg.min()) < STATIC_TOL

    runs, i = [], 0
    while i < len(m):
        if not m[i]:
            i += 1
            continue
        s = i
        while i < len(m) and m[i]:
            i += 1
        if t[i - 1] - t[s] >= STATIC_MIN_RUN:
            runs.append((s, i))
    return runs


def settled_idx(t, runs):
    """각 정지 구간에서 앞 SETTLE_SKIP 비율을 버린 나머지 인덱스."""
    out = []
    for s, e in runs:
        k = s + int((e - s) * SETTLE_SKIP)
        if e - k >= 3:
            out.extend(range(k, e))
    return np.array(out, int)


def lag(t, u, y):
    """u 가 y 로 나타나기까지의 지연 [ms] 을 상호상관으로 찾는다.

    2차계의 위상 지연은 주파수에 따라 커진다. 그래서 **절대값은 어떻게 흔들었나에
    달려 있다.** 같은 방식으로 흔든 캡처끼리만 비교할 것. 그래서 지령 대역도 함께
    돌려준다 — 무게중심이 비슷하면 비교해도 된다.

    되돌리는 값
        tau   최적 지연 [ms]
        r     그 지연에서의 상관
        r0    지연 0 에서의 상관. 낮으면 "지금 자세가 지금 필요한 자세와 무관"
        gain  그 지연에서의 진폭비(추종률). 1 보다 크면 지나친다
        fc    지령 스펙트럼 무게중심 [Hz]
    """
    if u is None or len(t) < 60:
        return None
    g = np.arange(t[0], t[-1], LAG_GRID_MS / 1000.0)
    if len(g) < 60:
        return None
    a = np.interp(g, t, u);  a = a - a.mean()
    b = np.interp(g, t, y);  b = b - b.mean()
    if a.std() < 1e-9 or b.std() < 1e-9:
        return None

    best, r0 = (0.0, -2.0), None
    for k in range(int(LAG_MAX_MS / LAG_GRID_MS) + 1):
        x, z = a[:len(a) - k], b[k:]
        if len(x) < 50:
            break
        sx, sz = x.std(), z.std()
        if sx < 1e-9 or sz < 1e-9:
            continue
        rr = float(np.mean(x * z) / (sx * sz))
        if r0 is None:
            r0 = rr
        if rr > best[1]:
            best = (k * LAG_GRID_MS, rr)

    k = int(best[0] / LAG_GRID_MS)
    x, z = a[:len(a) - k], b[k:]
    # 추종률은 진폭비 std(act)/std(cmd) 로 낸다.
    # 회귀기울기(cov/var)는 r 배만큼 과소평가된다 — 실측에서 상관 0.904 인 축이
    # 회귀 1.35 / 진폭비 1.50 으로 11% 어긋났다. "얼마나 지나치나" 를 묻는
    # 것이므로 진폭비가 맞다.
    gain = float(z.std() / x.std()) if x.std() > 1e-9 else float("nan")
    slope = float(np.polyfit(x, z, 1)[0]) if len(x) > 10 else float("nan")

    # 지령 대역 — 누적 파워 50% 지점(중앙주파수).
    # 무게중심은 잡음 바닥에 끌려 올라간다 (실측 0.41 vs 중앙 0.27Hz).
    # 캡처끼리 비교 가능한지 판단하는 용도라 잡음에 강한 쪽을 쓴다.
    w = a * np.hanning(len(a))
    Pw = np.abs(np.fft.rfft(w)) ** 2
    f = np.fft.rfftfreq(len(a), LAG_GRID_MS / 1000.0)
    m = (f > 0) & (f <= 15.0)
    fc = float("nan")
    if Pw[m].sum() > 0:
        cum = np.cumsum(Pw[m]) / Pw[m].sum()
        fc = float(f[m][np.searchsorted(cum, 0.5)])

    # 해상도 — 로그 표본 간격의 절반. 5ms 격자는 보간이라 그보다 정밀할 수 없다.
    res = float(np.median(np.diff(t)) * 1000.0 / 2.0)
    return dict(tau=best[0], r=best[1], r0=r0, gain=gain, slope=slope,
                fc=fc, res=res)


def rate(t, y):
    """도/초. 표본 간격이 0 인 지점은 버린다."""
    dt = np.diff(t)
    ok = dt > 1e-4
    return np.abs(np.diff(y)[ok] / dt[ok])


def analyse(d, axis):
    cn, an, pn = "cmd_" + axis, "act_" + axis, "pre_" + axis
    ch, t = d["chan"], d["t"]
    if cn not in ch or an not in ch:
        return None
    cmd, act = ch[cn], ch[an]
    pre = ch.get(pn)

    r = dict(axis=axis, span=float(cmd.max() - cmd.min()))
    if r["span"] < MIN_SPAN:
        r["skip"] = "지령 진폭 %.2f° — 흔들림 부족" % r["span"]
        return r

    runs = static_runs(t, cmd)
    idx = settled_idx(t, runs)
    r["n_hold"] = len(runs)
    r["static_frac"] = float(len(idx)) / len(t)

    # ── 드룹 : 정착된 정지 구간에서 act = a*cmd + b 회귀 ───────────────────
    # 정지 구간만, 그중에서도 정착된 뒷부분만 쓴다. 움직이는 중의 뒤처짐(동적)과
    # 도달 못 함(정적)은 원인이 다르므로 섞으면 안 된다.
    if len(idx) >= 10 and (cmd[idx].max() - cmd[idx].min()) > 0.5:
        A = np.vstack([cmd[idx], np.ones(len(idx))]).T
        a, b = np.linalg.lstsq(A, act[idx], rcond=None)[0]
        r["droop"], r["offset"] = float(a), float(b)
    elif len(idx) >= 10:
        # 유지 각도가 한 가지뿐이면 기울기를 못 구한다. 비율로 대신할 수는 있지만
        # 그 각도가 0 에 가까우면 안 된다. act = a·cmd + b 에서 오프셋 b 가
        # 실측 ±0.5° 라, cmd 가 1~2° 면 b 가 결과를 지배해 무의미한 값이 나온다.
        # 실제로 밀기 시험(유지 중 cmd ≈ −1°)에서 드룹 50% / −25% 라는
        # 쓰레기가 나왔다. 그래서 5° 미만이면 아예 내놓지 않는다.
        #
        # 밀기는 멈추면 상자가 수평이라 cmd ≈ 0 이다. 즉 드룹은 원리적으로
        # 기울이기 시험에서만 측정된다.
        mc = float(np.mean(cmd[idx]))
        if abs(mc) >= DROOP_MIN_HOLD:
            r["droop"] = float(np.mean(act[idx]) / mc)
            r["droop_rough"] = "유지 각도가 한 가지뿐 (%.0f° 부근)" % mc
        else:
            r["droop"] = float("nan")
            r["droop_rough"] = ("유지 중 지령이 %.1f° 뿐 — 0 에 너무 가까워 측정 불가. "
                                "기울인 채로 멈추는 구간이 필요합니다" % mc)
        r["offset"] = float("nan")
    else:
        r["droop"] = float("nan")
        r["offset"] = float("nan")
        r["droop_rough"] = "정착된 유지 구간이 없음 — 흔들다가 2초쯤 멈추는 구간을 넣으세요"

    # ── 추종오차 ──────────────────────────────────────────────────────────
    e = act - cmd
    r["err_rms"] = float(np.sqrt(np.mean(e ** 2)))
    r["err_max"] = float(np.max(np.abs(e)))

    # ── 떨림 : 정착된 구간에서 act 의 잔여 흔들림 ─────────────────────────
    # 구간마다 따로 계산한다. 떨어진 구간을 이어붙이면 그 이음매가 흔들림으로 잡힌다.
    # 이동평균을 빼서 고역만 남긴다. 직선만 빼면 느린 잔여 진동(정착 중)이
    # 그대로 남아 떨림으로 오인된다. 우리가 보려는 떨림은 빠른 진동이다.
    js = []
    w = max(5, int(round(JITTER_HP_S / max(np.median(np.diff(t)), 1e-3))) | 1)
    for s0, e0 in runs:
        k = s0 + int((e0 - s0) * SETTLE_SKIP)
        seg = act[k:e0]
        if len(seg) >= w + 4:
            base = np.convolve(seg, np.ones(w) / w, "same")
            h = w // 2
            js.append(float(np.std((seg - base)[h:-h])))
    r["jitter"] = float(np.median(js)) if js else float("nan")

    # ── 슬루 : pre 가 있어야 정확하다 ─────────────────────────────────────
    if pre is not None:
        clip = np.abs(pre - cmd) > CLIP_TOL
        r["clip_frac"] = float(clip.mean())
        r["clip_max"] = float(np.max(np.abs(pre - cmd)))
        rr = rate(t, pre)
        if len(rr):
            r["want_p95"] = float(np.percentile(rr, 95))
            r["want_max"] = float(rr.max())
        rc = rate(t, cmd)
        if len(rc):
            # 슬루가 걸렸을 때 cmd 는 정확히 한계 속도로 움직인다 -> 관측 최대가 곧 r
            r["r_eff"] = float(np.percentile(rc, 99))
    else:
        r["no_pre"] = True
        rc = rate(t, cmd)
        if len(rc):
            r["r_eff"] = float(np.percentile(rc, 99))

    # ── 지연 : cmd→act 가 모터만의 지연, pre→act 는 슬루까지 포함 ──────────
    # ref→act 는 재지 않는다. want = ACC_GAIN·ref − GAIN·θ_base 라 ref 는
    # 사슬의 입력이 아니고, 실제로 상관이 −0.1 까지 나와 아무 의미가 없었다.
    r["lag_motor"] = lag(t, cmd, act)
    if pre is not None:
        r["lag_slew"] = lag(t, pre, act)

    # ── 전체 사슬 ─────────────────────────────────────────────────────────
    # cmd→act 는 모터만이라 앞단 필터를 통째로 빼놓는다. 실측에서 CMD_LPF 가
    # 45~50ms, ACC_LPF 가 계산상 40ms 로 둘을 합치면 모터(85ms)보다 크다.
    # 슬로싱 위상을 판단하려면 물리 가속부터 재야 한다.
    #
    #   raw → [ACC_LPF] → ref → want → [CMD_LPF] → pre → [슬루] → cmd → [모터] → act
    #
    # want 는 로그에 없지만 want = ACC_GAIN·zv − GAIN·θ_base 로 재구성된다.
    # 검증: 재구성한 want 에 CMD_LPF 를 걸면 pre 가 오차 1% 로 재현됐다.
    zv = ch.get("zv_" + axis)
    base = ch.get(axis)
    if zv is not None and base is not None and d.get("acc_gain") is not None:
        want = np.clip(d["acc_gain"] * zv - d["gain"] * base,
                       -(d["limit"] or 25.0), (d["limit"] or 25.0))
        r["lag_want"] = lag(t, want, act)

    # raw_* 가 있으면 앞단까지 실측된다 (펌웨어 출력에 추가해야 나온다)
    raw = ch.get("raw_" + axis)
    if raw is not None:
        r["lag_raw"] = lag(t, raw, act)
    return r


def fmt(v, unit="", nd=2, pct=False):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    return ("%.0f%%" % (v * 100)) if pct else ("%.*f%s" % (nd, v, unit))


def detail(name, d, res):
    print()
    print("=" * 78)
    print(" %s" % name)
    print("=" * 78)
    if d.get("kp2") or d.get("kd2"):
        kpp, kpr = d.get("kp2") or (d.get("kp"), d.get("kp"))
        kdp, kdr = d.get("kd2") or (d.get("kd"), d.get("kd"))
        print("  표본 %d개  %.1f초  시간축 %s  |  Kp 안쪽 %s / 바깥 %s   Kd 안쪽 %s / 바깥 %s"
              % (d["n"], d["t"][-1] - d["t"][0], d["tsrc"],
                 fmt(kpp, nd=2), fmt(kpr, nd=2), fmt(kdp, nd=3), fmt(kdr, nd=3)))
    else:
        print("  표본 %d개  %.1f초  시간축 %s  |  Kp=%s Kd=%s"
              % (d["n"], d["t"][-1] - d["t"][0], d["tsrc"],
                 fmt(d["kp"], nd=2), fmt(d["kd"], nd=3)))
    if d.get("dropped"):
        print("  깨진 줄 %d개 버림 (%.2f%%) — USB 전송 중 유실"
              % (d["dropped"], 100.0 * d["dropped"] / (d["n"] + d["dropped"])))
    tg = d.get("tags") or []
    print("  로그 구간 %s%s" % (", ".join(tg) if tg else "(태그 없음)",
                              "  → POST 만 사용" if "POST" in tg else ""))

    for r in res:
        print()
        print("  ── %s ──" % r["axis"])
        if "skip" in r:
            print("     건너뜀: %s" % r["skip"])
            continue
        print("     지령 진폭 %.2f°   유지 구간 %d개   정착 표본 %s"
              % (r["span"], r.get("n_hold", 0), fmt(r["static_frac"], pct=True)))

        d_ = r["droop"]
        tag = ("  (%s)" % r["droop_rough"]) if r.get("droop_rough") else ""
        v = "-" if math.isnan(d_) else ("부족 → kp 올리기" if d_ < DROOP_OK else "충분")
        print("     드룹        %s   %s%s" % (fmt(d_, pct=True), v, tag))
        if not math.isnan(r["offset"]):
            print("       오프셋    %s   (0 에 가까우면 스프링, 크면 마찰·코깅)"
                  % fmt(r["offset"], "°"))

        print("     추종오차    RMS %s   최대 %s" % (fmt(r["err_rms"], "°"), fmt(r["err_max"], "°")))
        lm = r.get("lag_motor")
        if lm:
            v = "느림 → kp 올리기" if lm["tau"] > LAG_OK_MS else "양호"
            print("     지연        %s ±%s  %s   (모터. cmd→act)"
                  % (fmt(lm["tau"], "ms", 0), fmt(lm["res"], "ms", 0), v))
            print("       추종률    %s   1 보다 크면 지나칩니다" % fmt(lm["gain"], "", 2))
            # act = G·cmd + n 으로 갈라 본다. 진폭비는 외란 n 까지 포함하므로
            # kd 를 올려도 안 줄어드는 몫이 섞인다. 회귀 G 가 cmd 로 설명되는 몫이다.
            print("         지령 몫  %s   외란 몫 %s   (설명 안 되는 분산)"
                  % (fmt(lm.get("slope"), "", 2),
                     fmt(1.0 - lm["r"] ** 2, "", 3)))
            print("       상관      최적 %s / 지연0 %s"
                  % (fmt(lm["r"], "", 3), fmt(lm["r0"], "", 3)))
            if lm["r0"] < 0.5:
                print("         !! 지연0 상관이 낮습니다 — 지금 자세가 지금 필요한 "
                      "자세와 거의 무관합니다")
            ls = r.get("lag_slew")
            if ls:
                print("       슬루 포함 %s   (pre→act. 차이가 슬루 몫)"
                      % fmt(ls["tau"], "ms", 0))
            lw = r.get("lag_want")
            if lw:
                print("       LPF 포함  %s   (want→act. 차이가 CMD_LPF 몫)"
                      % fmt(lw["tau"], "ms", 0))
            lr = r.get("lag_raw")
            if lr:
                print("       ★ 전체    %s   (raw→act. 물리 가속부터)"
                      % fmt(lr["tau"], "ms", 0))
            # 슬로싱 위상 — 이게 최종 판단 근거다
            full = (lr or lw or lm)["tau"]
            src = "raw→act" if lr else ("want→act (ACC_LPF 제외)" if lw else "cmd→act (필터 전부 제외)")
            ph = 360.0 * full / SLOSH_MS
            v = ("역위상 — 슬로싱을 키운다" if ph > 120 else
                 "부담 큼" if ph > 90 else "허용" if ph > 45 else "양호")
            print("       슬로싱 위상 %.0f°   %s   [%s, 주기 %.0fms 기준]"
                  % (ph, v, src, SLOSH_MS))
            print("       흔든 대역 %s   ★ 이 값이 비슷한 캡처끼리만 비교하세요"
                  % fmt(lm["fc"], "Hz", 2))

        j = r.get("jitter")
        if j is not None and not math.isnan(j):
            v = "큼 → kd 올리기" if j > JITTER_OK else "양호"
            print("     떨림        %s   %s" % (fmt(j, "°", 3), v))

        if r.get("no_pre"):
            print("     슬루        pre_* 가 없어 판정 불가")
            print("                 (stream_log_추가.patch 를 적용하면 나옵니다)")
            if "r_eff" in r:
                print("       참고: 관측된 최대 지령 변화율 %s" % fmt(r["r_eff"], "°/s", 0))
        else:
            v = "잦음 → r 올리기" if r["clip_frac"] > CLIP_OK else "양호"
            print("     슬루 걸림   %s   %s   (최대 %s 잘림)"
                  % (fmt(r["clip_frac"], pct=True), v, fmt(r["clip_max"], "°")))
            if "want_p95" in r:
                print("       요구 변화율  p95 %s   최대 %s"
                      % (fmt(r["want_p95"], "°/s", 0), fmt(r["want_max"], "°/s", 0)))
                if (r.get("clip_frac") or 0) < 0.01:
                    print("       슬루는 걸리지 않았습니다 — 위 값은 관측된 최대일 뿐 한계가 아닙니다")
                else:
                    print("       유효 한계    약 %s" % fmt(r.get("r_eff"), "°/s", 0))


def advise(rows):
    print()
    print("=" * 78)
    print(" 판정")
    print("=" * 78)
    say = []
    for name, kp, kd, r in rows:
        if "skip" in r:
            continue
        if not math.isnan(r["droop"]) and r["droop"] < DROOP_OK:
            say.append("  · %s %s : 드룹 %s → kp %s 에서 한 단계 올리세요"
                       % (name, r["axis"], fmt(r["droop"], pct=True), fmt(kp, nd=1)))
        if r.get("clip_frac", 0) > CLIP_OK and "want_p95" in r:
            say.append("  · %s %s : 슬루에 %s 걸림. 요구 변화율 p95 가 %s 이니 "
                       "r 을 그 근처까지 올리면 대부분 사라집니다"
                       % (name, r["axis"], fmt(r["clip_frac"], pct=True),
                          fmt(r["want_p95"], "°/s", 0)))
        j = r.get("jitter")
        if j is not None and not math.isnan(j) and j > JITTER_OK:
            say.append("  · %s %s : 떨림 %s → kd %s 를 한 단계 올리거나 kp 를 내리세요"
                       % (name, r["axis"], fmt(j, "°", 3), fmt(kd, nd=2)))
    if say:
        print("\n".join(say))
    else:
        print("  모든 항목이 기준 안에 있습니다. 이 설정으로 진행해도 됩니다.")
    print()
    print("  기준: 드룹 %.0f%% 이상 / 슬루 걸림 %.0f%% 이하 / 떨림 %.2f° 이하"
          % (DROOP_OK * 100, CLIP_OK * 100, JITTER_OK))


def main():
    p = argparse.ArgumentParser(description="r / kp / kd 조정 상태를 로그로 판정")
    p.add_argument("logs", nargs="+", help="capture.py 로 뜬 로그 (여러 개 가능)")
    p.add_argument("--axis", default="both", choices=["pitch", "roll", "both"])
    p.add_argument("--quiet", action="store_true", help="비교표만 보기")
    a = p.parse_args()

    files = []
    for pat in a.logs:
        got = sorted(glob.glob(pat))
        files += got if got else [pat]

    axes = ["pitch", "roll"] if a.axis == "both" else [a.axis]
    table, flat = [], []
    for f in files:
        if not os.path.exists(f):
            print("!! 없는 파일: %s" % f)
            continue
        d = parse(f)
        if d is None:
            print("!! 스트림 줄이 없습니다: %s" % f)
            continue
        res = [r for r in (analyse(d, ax) for ax in axes) if r]
        if not a.quiet:
            detail(os.path.basename(f), d, res)
        for r in res:
            g_kp, g_kd = gains(d, r["axis"])
            table.append((os.path.basename(f), g_kp, g_kd, r))
            flat.append((os.path.basename(f), g_kp, g_kd, r))

    if not table:
        return 1

    print()
    print("=" * 78)
    print(" 비교표")
    print("=" * 78)
    print("  %-24s %-6s %5s %5s %6s %7s %7s %6s %6s %6s"
          % ("파일", "축", "Kp", "Kd", "드룹", "슬루걸림", "지연", "추종", "떨림", "대역"))
    print("  " + "-" * 76)
    for name, kp, kd, r in table:
        if "skip" in r:
            print("  %-26s %-6s %5s %5s   %s" % (name[:26], r["axis"],
                  fmt(kp, nd=1), fmt(kd, nd=2), r["skip"]))
            continue
        lm = r.get("lag_motor") or {}
        print("  %-24s %-6s %5s %5s %6s %7s %7s %6s %6s %6s"
              % (name[:24], r["axis"], fmt(kp, nd=1), fmt(kd, nd=2),
                 fmt(r["droop"], pct=True),
                 fmt(r.get("clip_frac"), pct=True) if not r.get("no_pre") else "-",
                 fmt(lm.get("tau"), "ms", 0),
                 fmt(lm.get("gain"), "", 2),
                 fmt(r.get("jitter"), "", 3),
                 fmt(lm.get("fc"), "", 2)))

    advise(flat)
    return 0


if __name__ == "__main__":
    sys.exit(main())
