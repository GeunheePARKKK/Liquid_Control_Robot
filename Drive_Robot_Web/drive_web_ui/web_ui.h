/*
 * web_ui.h — 펌웨어에 내장되는 웹 UI (HTML/CSS/JS 한 덩어리)
 *
 * 브라우저가 http://192.168.4.1 을 열면 이 문자열이 그대로 내려간다.
 * UI 를 고치면 펌웨어를 다시 업로드해야 한다 (OTA 가능).
 *
 * 동작 요약
 *   - keydown/keyup 으로 W/A/S/D 상태를 비트마스크로 만들어 "k:<mask>" 전송
 *   - 눌린 동안 100ms 마다 재전송 (보드 쪽 데드맨 워치독의 심장박동)
 *   - 탭이 숨겨지거나 포커스를 잃으면 즉시 전체 정지
 *   - 화면 버튼은 터치(pointerdown/up)로 같은 동작 (휴대폰용)
 *   - 하단 입력창: 시리얼 모니터처럼 임의 명령 전송 ("c:<text>")
 *   - 보드가 보내는 텍스트는 로그창에 표시
 *   - 연결이 끊기면 1초마다 자동 재접속
 */
#pragma once

static const char WEB_UI_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>Drive Robot</title>
<style>
  :root { --bg:#111418; --panel:#1b2027; --key:#2a323d; --key-on:#2f81f7;
          --txt:#dbe4ee; --dim:#8b98a9; --ok:#3fb950; --bad:#f85149; }
  * { box-sizing:border-box; margin:0; -webkit-tap-highlight-color:transparent; }
  body { background:var(--bg); color:var(--txt);
         font-family:system-ui,'Segoe UI',sans-serif;
         display:flex; flex-direction:column; align-items:center;
         min-height:100vh; padding:16px; gap:14px; user-select:none; }

  h1 { font-size:20px; letter-spacing:1px; }
  #status { font-size:14px; color:var(--bad); font-weight:600; }
  #status.ok { color:var(--ok); }

  /* ── 방향 패드 ── */
  #pad { display:grid; grid-template-columns:repeat(3,86px);
         grid-template-rows:repeat(2,86px); gap:10px; touch-action:none; }
  .key { background:var(--key); border:1px solid #3a4552; border-radius:12px;
         display:flex; flex-direction:column; justify-content:center; align-items:center;
         font-size:26px; font-weight:700; cursor:pointer;
         transition:background .06s, transform .06s; }
  .key small { font-size:11px; font-weight:400; color:var(--dim); margin-top:2px; }
  .key.on { background:var(--key-on); transform:scale(.95); }
  .key.on small { color:#e8f0ff; }
  #kw { grid-column:2; grid-row:1; }
  #ka { grid-column:1; grid-row:2; }
  #ks { grid-column:2; grid-row:2; }
  #kd { grid-column:3; grid-row:2; }

  #estop { width:278px; padding:13px; border:none; border-radius:12px;
           background:var(--bad); color:#fff; font-size:17px; font-weight:800;
           letter-spacing:2px; cursor:pointer; }
  #estop:active { transform:scale(.97); }

  /* ── 속도 ── */
  #speedbox { width:278px; background:var(--panel); border-radius:12px; padding:12px 16px; }
  #speedbox label { font-size:13px; color:var(--dim); display:flex; justify-content:space-between; }
  #speed { width:100%; margin-top:8px; accent-color:var(--key-on); }

  /* ── 로그 + 명령 입력 ── */
  #logbox { width:min(560px,94vw); background:var(--panel); border-radius:12px;
            padding:10px; display:flex; flex-direction:column; gap:8px; }
  #log { height:180px; overflow-y:auto; background:#0d1117; border-radius:8px;
         padding:8px 10px; font:12px/1.5 Consolas,monospace; color:#9fb3c8;
         white-space:pre-wrap; word-break:break-all; user-select:text; }
  #cmdrow { display:flex; gap:8px; }
  #cmd { flex:1; background:#0d1117; border:1px solid #3a4552; border-radius:8px;
         padding:8px 10px; color:var(--txt); font:13px Consolas,monospace; outline:none; }
  #cmd:focus { border-color:var(--key-on); }
  #send { background:var(--key-on); border:none; border-radius:8px; color:#fff;
          padding:0 18px; font-weight:700; cursor:pointer; }
  .hint { font-size:11px; color:var(--dim); }
</style>
</head>
<body>

<h1>🤖 Drive Robot</h1>
<div id="status">연결 안 됨…</div>

<div id="pad">
  <div class="key" id="kw" data-bit="1">W<small>전진</small></div>
  <div class="key" id="ka" data-bit="2">A<small>좌회전</small></div>
  <div class="key" id="ks" data-bit="4">S<small>후진</small></div>
  <div class="key" id="kd" data-bit="8">D<small>우회전</small></div>
</div>

<button id="estop">STOP (Space)</button>

<div id="speedbox">
  <label>속도 <span id="speedval">150</span></label>
  <input type="range" id="speed" min="0" max="255" value="150">
</div>

<div id="logbox">
  <div id="log"></div>
  <div id="cmdrow">
    <input id="cmd" placeholder="명령 입력 (시리얼 모니터처럼) — 예: ?  v200  stop"
           autocomplete="off">
    <button id="send">전송</button>
  </div>
  <div class="hint">키보드 W/A/S/D = 누르는 동안 이동 · Space = 비상 정지 · 명령창 포커스 중엔 주행키 무시</div>
</div>

<script>
"use strict";
let ws = null, mask = 0, hbTimer = null;
const $ = id => document.getElementById(id);
const statusEl = $("status"), logEl = $("log"), cmdEl = $("cmd");
const KEYBIT = { w:1, a:2, s:4, d:8 };

/* ── WebSocket 연결 (끊기면 1초 후 재시도) ── */
function connect() {
  try {
    ws = new WebSocket("ws://" + location.hostname + ":81/");
  } catch (e) {
    statusEl.textContent = "WebSocket 생성 실패 — 재시도 중…";
    setTimeout(connect, 1000);
    return;
  }
  ws.onopen = () => { statusEl.textContent = "연결됨 ✓"; statusEl.className = "ok"; };
  ws.onclose = () => {
    statusEl.textContent = "연결 끊김 — 재접속 중…"; statusEl.className = "";
    setMask(0);
    setTimeout(connect, 1000);
  };
  ws.onmessage = e => addLog(e.data);
}
function wsSend(s) { if (ws && ws.readyState === 1) ws.send(s); }

/* ── 키 상태 전송 + 심장박동 ── */
function sendMask() { wsSend("k:" + mask); }
function setMask(m) {
  if (m === mask) return;
  mask = m;
  sendMask();
  for (const k in KEYBIT)
    $("k" + k).classList.toggle("on", !!(mask & KEYBIT[k]));
  // 눌린 동안 100ms 마다 재전송 → 보드 데드맨 워치독용
  if (mask && !hbTimer) hbTimer = setInterval(sendMask, 100);
  if (!mask && hbTimer) { clearInterval(hbTimer); hbTimer = null; }
}

/* ── 키보드 ── */
addEventListener("keydown", e => {
  if (e.repeat) return;
  if (document.activeElement === cmdEl) return;      // 명령창 입력 중엔 무시
  const k = e.key.toLowerCase();
  if (k === " ") { estop(); e.preventDefault(); return; }
  if (KEYBIT[k]) { setMask(mask | KEYBIT[k]); e.preventDefault(); }
});
addEventListener("keyup", e => {
  const k = e.key.toLowerCase();
  if (KEYBIT[k]) setMask(mask & ~KEYBIT[k]);
});

/* ── 화면 버튼 (터치/마우스) ── */
for (const k in KEYBIT) {
  const el = $("k" + k), bit = KEYBIT[k];
  el.addEventListener("pointerdown", e => { e.preventDefault(); setMask(mask | bit); });
  el.addEventListener("pointerup",     () => setMask(mask & ~bit));
  el.addEventListener("pointerleave",  () => setMask(mask & ~bit));
  el.addEventListener("pointercancel", () => setMask(mask & ~bit));
}

/* ── 비상 정지 ── */
function estop() { setMask(0); wsSend("x"); }
$("estop").addEventListener("click", estop);

/* ── 탭 숨김·포커스 상실 → 전체 정지 ── */
addEventListener("blur", () => setMask(0));
document.addEventListener("visibilitychange", () => { if (document.hidden) estop(); });

/* ── 속도 슬라이더 ── */
$("speed").addEventListener("input", e => {
  $("speedval").textContent = e.target.value;
  wsSend("v:" + e.target.value);
});

/* ── 명령 입력창 ── */
function sendCmd() {
  const t = cmdEl.value.trim();
  if (!t) return;
  wsSend("c:" + t);
  addLog("> " + t + "\n");
  cmdEl.value = "";
}
$("send").addEventListener("click", sendCmd);
cmdEl.addEventListener("keydown", e => { if (e.key === "Enter") sendCmd(); });

/* ── 로그창 ── */
function addLog(s) {
  logEl.textContent += s;
  if (logEl.textContent.length > 20000)               // 너무 길어지면 앞을 자른다
    logEl.textContent = logEl.textContent.slice(-15000);
  logEl.scrollTop = logEl.scrollHeight;
}

connect();
</script>
</body>
</html>
)rawliteral";
