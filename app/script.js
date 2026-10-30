"use strict";

/* Configuración por defecto; el firmware la confirma con la línea CFG */
const cfg = {
  T: 0.01, u0p: 42.1, u0n: 55.3, umax: 57.9, umin: -44.7, dz: true, lim: 40,
  coef: {
    1: { q0: 36.208476, q1: -63.095738, q2: 27.591053, s0: 0 },
    2: { q0: 8.523371, q1: -14.371015, q2: 6.123475, s0: -0.262282 },
  },
};
const DESIGN = structuredClone(cfg.coef);
const MODE_NAME = { 0: "Apagado", 1: "Continuo–discreto", 2: "Discreto–discreto" };
const R_MAX = 10;
const HIST_SECONDS = 30;
const MM_PER_COUNT = 0.0671;

/* Simulación en Simulink (saturación + encoder), escalones 0→+5 y +5→−5 mm */
const SIM = {
  1: { ts1: 0.157, ts2: 0.329, mp1: 1.68, mp2: 2.89, ise: 0.861 + 3.975 },
  2: { ts1: 0.382, ts2: 0.360, mp1: 0.12, mp2: 0.04, ise: 1.270 + 5.373 },
};

let port = null, writer = null, reader = null, connected = false;
let lastMsg = 0, sending = false, pendingRef = null;
let hist = [], last = null, dirty = true;
let winSec = 10, paused = false, pausedEnd = 0, cursorT = null;
let iseWin = 5, iseQueue = [], iseRun = 0, iseStepAcc = 0, lastR = null;
let rec = null, capture = null, abortTest = false, testing = false;
let sqTimer = null, sqSign = 1;
let coefTab = 1;
const results = { 1: null, 2: null };

const $ = (id) => document.getElementById(id);
const sl = $("sl"), bubble = $("bubble"), banner = $("banner"), inMm = $("inMm");
const cvPos = $("cvPos"), cvErr = $("cvErr"), cvU = $("cvU"), cvIse = $("cvIse");
const ctx = {
  pos: cvPos.getContext("2d"), err: cvErr.getContext("2d"),
  u: cvU.getContext("2d"), ise: cvIse.getContext("2d"),
};
const sleep = (ms) => new Promise((res) => setTimeout(res, ms));
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const fmtNum = (v, d, unit) => `${v.toFixed(d)}<span>${unit}</span>`;

/* ============================================================
 * Avisos
 * ============================================================ */
function toast(msg, type = "error") {
  const el = document.createElement("div");
  el.className = `toast ${type}`;
  el.textContent = msg;
  $("toasts").appendChild(el);
  setTimeout(() => el.remove(), 5000);
}
function showBanner(msg) { banner.textContent = msg; banner.classList.remove("hidden"); }
function hideBanner() { banner.classList.add("hidden"); }

if (!("serial" in navigator)) {
  showBanner("Este navegador no soporta Web Serial. Usa Chrome, Edge u Opera.");
  $("btnConn").disabled = true;
}

/* ============================================================
 * Conexión serial
 * ============================================================ */
function setLink(ok) {
  connected = ok;
  $("dot").classList.toggle("on", ok);
  $("st").textContent = ok ? "Conectado" : "Desconectado";
  $("btnConn").textContent = ok ? "Desconectar" : "Conectar";
  $("btnConn").classList.toggle("on", ok);
  if (ok) hideBanner();
}

$("btnConn").addEventListener("click", async () => {
  if (connected) return disconnectSerial();
  try {
    port = await navigator.serial.requestPort();
  } catch (e) {
    if (e.name !== "NotFoundError") toast("No se pudo abrir el selector de puertos: " + e.message);
    return;
  }
  try {
    await port.open({ baudRate: 115200, bufferSize: 65536 });
    writer = port.writable.getWriter();
    setLink(true);
    toast("Conectado al ESP32", "ok");
    readLoop();
    watchTelemetry();
    setTimeout(() => sendLine("CFG"), 300);
  } catch (e) {
    toast("No se pudo abrir el puerto (¿está en uso por idf.py monitor?): " + e.message);
    port = null;
  }
});

async function disconnectSerial() {
  stopSquare();
  abortTest = true;
  try {
    if (reader) { await reader.cancel().catch(() => {}); reader = null; }
    if (writer) { writer.releaseLock(); writer = null; }
    if (port) { await port.close(); port = null; }
  } catch (e) { toast("Error al desconectar: " + e.message); }
  setLink(false);
}

navigator.serial?.addEventListener("disconnect", () => {
  if (connected) { toast("Se desconectó el cable USB"); setLink(false); }
});

function watchTelemetry() {
  const t0 = Date.now();
  const iv = setInterval(() => {
    if (!connected) return clearInterval(iv);
    if (Date.now() - t0 > 3000 && lastMsg < t0) {
      toast("No llega telemetría. Revisa que el firmware esté corriendo y que ningún otro programa tenga el puerto abierto.");
      clearInterval(iv);
    }
  }, 500);
}

setInterval(() => {
  if (connected && lastMsg !== 0 && Date.now() - lastMsg > 1500) {
    lastMsg = 0;
    toast("Se dejó de recibir telemetría del ESP32");
  }
}, 500);

async function readLoop() {
  const decoder = new TextDecoderStream();
  reader = port.readable.pipeThrough(decoder).getReader();
  let buf = "";
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      buf += value;
      const lines = buf.split("\n");
      buf = lines.pop();
      for (const line of lines) handleLine(line.trim());
    }
  } catch (e) {
    if (connected) toast("Se perdió la conexión: " + e.message);
  } finally {
    try { reader.releaseLock(); } catch (_) {}
    setLink(false);
  }
}

async function sendLine(text) {
  if (!connected || !writer) { toast("Conéctate primero."); return false; }
  try { await writer.write(new TextEncoder().encode(text + "\n")); return true; }
  catch (e) { toast("Error enviando comando: " + e.message); return false; }
}

/* ============================================================
 * Líneas del ESP32
 * ============================================================ */
function handleLine(line) {
  if (line.startsWith("D,")) return handleData(line);
  if (line.startsWith("CFG,")) return handleCfg(line);
  if (line.startsWith("ERR,")) return toast(line.slice(4));
  if (line.startsWith("ACK,")) return handleAck(line.slice(4).split(","));
}

/* CFG,T_ms,q0,q1,q2,s0 (CD),q0,q1,q2,s0 (DD),u0+,u0-,u_ef_max,u_ef_min,dz,lim */
function handleCfg(line) {
  const p = line.split(",").map(Number);
  if (p.length < 16 || p.slice(1).some(Number.isNaN)) return;
  cfg.T = p[1] / 1000;
  cfg.coef[1] = { q0: p[2], q1: p[3], q2: p[4], s0: p[5] };
  cfg.coef[2] = { q0: p[6], q1: p[7], q2: p[8], s0: p[9] };
  [cfg.u0p, cfg.u0n, cfg.umax, cfg.umin] = p.slice(10, 14);
  cfg.dz = p[14] === 1;
  cfg.lim = p[15];
  $("chkDz").checked = cfg.dz;
  $("pillCfg").textContent = `T ${p[1]} ms · u_ef ${cfg.umin.toFixed(1)}…${cfg.umax.toFixed(1)} % · lím ±${cfg.lim} mm`;
  $("pillCfg").className = "pill ok";
  fillCoefs();
}

function handleAck(p) {
  const msgs = {
    ZERO: "Posición actual = 0 mm",
    STOP: "Motor detenido · control apagado",
    C: `Controlador: ${MODE_NAME[p[1]] ?? p[1]}`,
    Q: `Coeficientes del ${MODE_NAME[p[1]]} actualizados`,
    DZ: p[1] === "1" ? "Compensación de zona muerta activada" : "Compensación de zona muerta desactivada",
    LIM: `Límite de seguridad ±${p[1]} mm`,
  };
  if (msgs[p[0]] && !testing) toast(msgs[p[0]], "ok");
}

/* D,t_ms,r,y,e,u_ef,u_pwm,v,y_mod,modo */
function handleData(line) {
  const p = line.split(",");
  if (p.length < 10) return;
  const n = p.slice(1).map(Number);
  if (n.some(Number.isNaN)) return;
  const [tMs, r, y, e, uef, upwm, v, ym, mode] = n;
  const s = { t: tMs / 1000, r, y, e, uef, upwm, v, ym, mode };

  if (hist.length && s.t < hist[hist.length - 1].t) { hist = []; iseQueue = []; iseRun = 0; } // reinicio del ESP32
  lastMsg = Date.now();

  /* ISE: ventana deslizante y desde el último cambio de referencia */
  const de = e * e * cfg.T;
  iseQueue.push({ t: s.t, de });
  iseRun += de;
  while (iseQueue.length && s.t - iseQueue[0].t >= iseWin) iseRun -= iseQueue.shift().de;
  if (lastR === null || Math.abs(r - lastR) > 1e-6) iseStepAcc = 0;
  iseStepAcc += de;
  lastR = r;
  s.ise = Math.max(0, iseRun);

  last = s;
  hist.push(s);
  while (hist.length && s.t - hist[0].t > HIST_SECONDS) hist.shift();

  if (rec && rec.active) rec.rows.push(s);
  if (capture) capture.push(s);
  dirty = true;
}

/* ============================================================
 * Tarjetas
 * ============================================================ */
function centerBar(el, value, full) {
  const w = clamp(Math.abs(value) / full, 0, 1) * 50;
  el.style.left = `${value >= 0 ? 50 : 50 - w}%`;
  el.style.width = `${w}%`;
}

function renderStats() {
  const s = last;
  if (!s) return;
  $("y").innerHTML = fmtNum(s.y, 3, "mm");
  centerBar($("by"), s.y, R_MAX);
  $("mr").style.left = `${50 + clamp(s.r / R_MAX, -1, 1) * 50}%`;
  $("yfoot").textContent = `modelo G(z): ${s.ym.toFixed(3)} mm · v ${s.v.toFixed(1)} mm/s`;

  $("r").innerHTML = fmtNum(s.r, 3, "mm");
  centerBar($("br"), s.r, R_MAX);
  $("rfoot").textContent = s.r > 0 ? "hacia la derecha" : s.r < 0 ? "hacia la izquierda" : "centro";

  const eEl = $("e");
  eEl.innerHTML = fmtNum(s.e, 3, "mm");
  eEl.classList.toggle("ok", Math.abs(s.e) <= MM_PER_COUNT * 1.6);
  centerBar($("be"), s.e, 5);
  $("efoot").textContent = `${(s.e / MM_PER_COUNT).toFixed(1)} cuentas del encoder`;

  $("u").innerHTML = fmtNum(s.uef, 2, "%");
  centerBar($("bu"), s.uef, 60);
  const sat = s.uef >= cfg.umax - 0.05 || s.uef <= cfg.umin + 0.05;
  $("ufoot").textContent = `PWM aplicado ${s.upwm.toFixed(1)} %${sat && s.mode ? " · saturado" : ""}`;

  $("ise").innerHTML = fmtNum(s.ise, 3, "mm²·s");
  $("bise").style.width = `${clamp(s.ise / 10, 0, 1) * 100}%`;
  $("isefoot").textContent = `últimos ${iseWin} s · ${MODE_NAME[s.mode]}`;
  $("iseW").innerHTML = fmtNum(s.ise, 3, "mm²·s");
  $("iseStep").innerHTML = fmtNum(iseStepAcc, 3, "mm²·s");

  setModeUI(s.mode);
  if (rec && rec.active) {
    $("recN").innerHTML = fmtNum(rec.rows.length, 0, "muestras");
    const d = rec.rows.length ? rec.rows[rec.rows.length - 1].t - rec.rows[0].t : 0;
    $("recT").textContent = `${d.toFixed(2)} s · T = ${(cfg.T * 1000).toFixed(0)} ms`;
  }
}

/* ============================================================
 * Modo, cero, stop, zona muerta
 * ============================================================ */
const modeToggle = $("modeToggle");
function setModeUI(mode) {
  if (modeToggle.dataset.mode === String(mode)) return;
  modeToggle.dataset.mode = String(mode);
  modeToggle.querySelectorAll(".toggle-opt").forEach((b) => b.classList.toggle("active", b.dataset.mode === String(mode)));
  $("law").classList.toggle("dim", mode === 0);
  if (mode !== 0 && !testing) { coefTab = mode; fillCoefs(); }
}
modeToggle.querySelectorAll(".toggle-opt").forEach((b) =>
  b.addEventListener("click", async () => {
    if (testing) return toast("Hay una prueba en curso. Pulsa STOP para cancelarla.");
    const m = b.dataset.mode;
    if (m !== "0" && last && Math.abs(last.r - last.y) > 3)
      toast(`Error grande (${(last.r - last.y).toFixed(1)} mm): el controlador saturará al arrancar`, "ok");
    await sendLine(`C ${m}`);
  })
);

$("btnZero").addEventListener("click", () => { sendLine("ZERO"); hist = []; iseQueue = []; iseRun = 0; });
async function stopAll() {
  abortTest = true;
  stopSquare();
  await sendLine("S");
}
$("btnStop").addEventListener("click", stopAll);
$("chkDz").addEventListener("change", (e) => { cfg.dz = e.target.checked; sendLine(`DZ ${cfg.dz ? 1 : 0}`); });

/* ============================================================
 * Referencia
 * ============================================================ */
function paintSlider(mm) {
  const pct = (mm + R_MAX) / (2 * R_MAX) * 100;
  const a = Math.min(50, pct), b = Math.max(50, pct);
  sl.style.background = `linear-gradient(90deg, transparent ${a}%, oklch(78% 0.15 65) ${a}%, oklch(66% 0.18 40) ${b}%, transparent ${b}%), oklch(100% 0 0 / .10)`;
  $("sv").innerHTML = fmtNum(mm, 2, "mm");
  $("svSide").textContent = mm > 0 ? "derecha →" : mm < 0 ? "← izquierda" : "centro";
  bubble.textContent = `${mm.toFixed(2)} mm`;
  const thumb = 26, trackWidth = sl.clientWidth - thumb;
  bubble.style.left = `${thumb / 2 + (pct / 100) * trackWidth}px`;
}

function setTarget(mm) {
  mm = clamp(Math.round(mm * 100) / 100, -R_MAX, R_MAX);
  sl.value = mm;
  inMm.value = mm.toFixed(2);
  paintSlider(mm);
  return mm;
}

/* Envía R sin saturar el puerto: si hay uno en camino, guarda solo el último */
function sendRef(mm) {
  if (sending) { pendingRef = mm; return; }
  sending = true;
  sendLine(`R ${mm.toFixed(3)}`).finally(() => {
    sending = false;
    if (pendingRef !== null) { const r = pendingRef; pendingRef = null; sendRef(r); }
  });
}
function applyRef(mm) {
  mm = setTarget(mm);
  if (connected) sendRef(mm);
  if (connected && last && last.mode === 0) toast("La referencia se guardó, pero el controlador está apagado", "ok");
}

sl.addEventListener("input", () => {
  const mm = setTarget(parseFloat(sl.value));
  if ($("chkLive").checked && connected) sendRef(mm);
});
sl.addEventListener("change", () => { if (!$("chkLive").checked) applyRef(parseFloat(sl.value)); });
inMm.addEventListener("keydown", (e) => { if (e.key === "Enter") $("btnApply").click(); });
$("btnApply").addEventListener("click", () => {
  const mm = parseFloat(inMm.value);
  if (Number.isNaN(mm)) return toast("Escribe una referencia en mm.");
  if (Math.abs(mm) > R_MAX) toast(`Se limita a ±${R_MAX} mm`);
  applyRef(mm);
});
document.querySelectorAll(".presets button").forEach((b) =>
  b.addEventListener("click", () => applyRef(parseFloat(b.dataset.mm)))
);
window.addEventListener("resize", () => paintSlider(parseFloat(sl.value)));

/* Onda cuadrada */
function stopSquare() {
  if (!sqTimer) return;
  clearInterval(sqTimer);
  sqTimer = null;
  $("btnSq").textContent = "▶ Iniciar";
  $("btnSq").classList.remove("run");
}
$("btnSq").addEventListener("click", () => {
  if (sqTimer) return stopSquare();
  if (!connected) return toast("Conéctate primero.");
  if (!last || last.mode === 0) return toast("Activa un controlador antes de la onda cuadrada.");
  const A = clamp(parseFloat($("inSqA").value) || 4, 0.5, R_MAX);
  const P = clamp(parseFloat($("inSqP").value) || 3, 1, 20);
  sqSign = 1;
  applyRef(A);
  sqTimer = setInterval(() => { sqSign = -sqSign; applyRef(sqSign * A); }, P * 500);
  $("btnSq").textContent = "■ Detener";
  $("btnSq").classList.add("run");
});

/* Teclado */
document.addEventListener("keydown", (e) => {
  if (e.target.matches("input, select, textarea")) return;
  if (e.key === "Escape") { e.preventDefault(); stopAll(); }
  else if (e.key === "ArrowRight") { e.preventDefault(); applyRef(parseFloat(sl.value) + 0.5); }
  else if (e.key === "ArrowLeft") { e.preventDefault(); applyRef(parseFloat(sl.value) - 0.5); }
  else if (e.key === "0") applyRef(0);
  else if (e.key === " ") { e.preventDefault(); $("btnPause").click(); }
});

/* ============================================================
 * ISE
 * ============================================================ */
$("selIseWin").addEventListener("change", (e) => {
  iseWin = parseFloat(e.target.value);
  iseQueue = []; iseRun = 0;
  const tEnd = hist.length ? hist[hist.length - 1].t : 0;
  for (const s of hist) if (tEnd - s.t < iseWin) { const de = s.e * s.e * cfg.T; iseQueue.push({ t: s.t, de }); iseRun += de; }
  dirty = true;
});

/* ============================================================
 * Registro de datos
 * ============================================================ */
$("btnRec").addEventListener("click", () => {
  if (rec && rec.active) {
    rec.active = false;
    $("btnRec").textContent = "● Grabar";
    $("btnRec").classList.remove("rec");
    $("btnCsv").disabled = rec.rows.length < 2;
    toast(`Grabación lista: ${rec.rows.length} muestras`, "ok");
    return;
  }
  if (!connected) return toast("Conéctate primero.");
  rec = { active: true, rows: [], started: new Date() };
  $("btnRec").textContent = "■ Detener";
  $("btnRec").classList.add("rec");
  $("btnCsv").disabled = true;
});

function download(name, text) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type: "text/csv" }));
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
const stamp = () => new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-");

$("btnCsv").addEventListener("click", () => {
  if (!rec || rec.rows.length < 2) return;
  const t0 = rec.rows[0].t;
  const c1 = cfg.coef[1], c2 = cfg.coef[2];
  const lines = [
    "# Lab 5 - Control discreto de un sistema mecanico (pinon-cremallera)",
    `# T_s = ${cfg.T.toFixed(4)} s`,
    `# inicio = ${rec.started.toISOString()}`,
    `# CD: q0=${c1.q0} q1=${c1.q1} q2=${c1.q2} s0=${c1.s0}`,
    `# DD: q0=${c2.q0} q1=${c2.q1} q2=${c2.q2} s0=${c2.s0}`,
    `# zona muerta u0+=${cfg.u0p} u0-=${cfg.u0n} compensacion=${cfg.dz ? 1 : 0}`,
    "# modo: 0 apagado, 1 continuo-discreto, 2 discreto-discreto",
    "k,t_s,t_esp32_ms,r_mm,y_mm,e_mm,u_ef_pct,u_pwm_pct,v_mm_s,y_modelo_mm,modo",
  ];
  rec.rows.forEach((p, k) => lines.push([k, (p.t - t0).toFixed(3), Math.round(p.t * 1000), p.r.toFixed(3), p.y.toFixed(3),
    p.e.toFixed(3), p.uef.toFixed(2), p.upwm.toFixed(1), p.v.toFixed(1), p.ym.toFixed(3), p.mode].join(",")));
  download(`lab5_datos_${stamp()}.csv`, lines.join("\n"));
});

/* ============================================================
 * Prueba de escalón automática
 * ============================================================ */
function segMetrics(seg, r0, rf) {
  const A = rf - r0, sg = Math.sign(A), absA = Math.abs(A), t0 = seg[0].t;
  const peak = Math.max(...seg.map((p) => sg * (p.y - rf)));
  const mp = Math.max(0, peak) / absA * 100;
  const band = Math.max(0.02 * absA, MM_PER_COUNT * 1.01);
  let lastOut = -1;
  seg.forEach((p, i) => { if (Math.abs(p.y - rf) > band) lastOut = i; });
  const ts = lastOut === seg.length - 1 ? null : (lastOut < 0 ? 0 : seg[lastOut + 1].t - t0);
  const tail = seg.slice(-Math.max(5, Math.round(seg.length * 0.2)));
  const ess = rf - tail.reduce((a, p) => a + p.y, 0) / tail.length;
  const ise = seg.reduce((a, p) => a + p.e * p.e * cfg.T, 0);
  const umax = Math.max(...seg.map((p) => Math.abs(p.uef)));
  return { r0, rf, ts, mp, ess, ise, umax };
}

async function waitOrAbort(ms, onTick) {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) {
    if (abortTest || !connected) throw new Error("abort");
    onTick?.();
    await sleep(50);
  }
}

async function runStepTest(mode, A, D, progBase, progSpan) {
  const refs = [A, -A, 0];
  const total = 1.5 + 3 * D;
  const tick = (t0) => () => {
    const f = (Date.now() - t0) / 1000 / total;
    $("stBar").style.width = `${(progBase + clamp(f, 0, 1) * progSpan) * 100}%`;
  };
  const t0 = Date.now();
  $("stInfo").textContent = `${MODE_NAME[mode]}: llevando a 0 mm…`;
  setTarget(0);
  await sendLine("R 0");
  await sendLine(`C ${mode}`);
  await waitOrAbort(1500, tick(t0));
  capture = [];
  for (const r of refs) {
    $("stInfo").textContent = `${MODE_NAME[mode]}: escalón a ${r} mm…`;
    setTarget(r);
    await sendLine(`R ${r}`);
    await waitOrAbort(D * 1000, tick(t0));
  }
  const data = capture;
  capture = null;

  /* corta por cambios de r reportados por el ESP32 */
  const segs = [];
  let start = data.findIndex((p) => Math.abs(p.r - A) < 1e-6);
  if (start < 0) throw new Error("no se detectó el escalón en la telemetría");
  let prevR = 0;
  for (let i = start; i < data.length;) {
    let j = i;
    while (j < data.length && Math.abs(data[j].r - data[i].r) < 1e-6) j++;
    segs.push(segMetrics(data.slice(i, j), prevR, data[i].r));
    prevR = data[i].r;
    i = j;
  }
  return { mode, A, D, segs, data, ise: segs.reduce((a, s) => a + s.ise, 0) };
}

async function startTests(modes) {
  if (testing) return;
  if (!connected) return toast("Conéctate primero.");
  const A = clamp(parseFloat($("inStA").value) || 5, 1, R_MAX);
  const D = clamp(parseFloat($("inStD").value) || 2.5, 1, 10);
  stopSquare();
  testing = true; abortTest = false;
  $("stProg").classList.remove("hidden");
  $("btnStepCur").disabled = $("btnStepBoth").disabled = true;
  try {
    for (let i = 0; i < modes.length; i++) {
      results[modes[i]] = await runStepTest(modes[i], A, D, i / modes.length, 1 / modes.length);
      renderCmp();
    }
    setTarget(0);
    await sendLine("R 0");
    $("stInfo").textContent = `Listo: escalones de ±${A} mm, ${D} s por tramo. El controlador queda activo en 0 mm.`;
    toast("Prueba de escalón terminada", "ok");
  } catch (e) {
    capture = null;
    $("stInfo").textContent = e.message === "abort" ? "Prueba cancelada." : `Error: ${e.message}`;
  } finally {
    testing = false;
    $("stProg").classList.add("hidden");
    $("btnStepCur").disabled = $("btnStepBoth").disabled = false;
  }
}
$("btnStepCur").addEventListener("click", () => {
  const m = last ? last.mode : 0;
  if (m === 0) return toast("Selecciona primero un controlador (continuo–discreto o discreto–discreto).");
  startTests([m]);
});
$("btnStepBoth").addEventListener("click", () => startTests([1, 2]));

/* Tabla de comparación */
const CMP_ROWS = [
  { label: "t<sub>s</sub> 0 → +A", get: (r) => r.segs[0]?.ts, sim: (s) => s.ts1, fmt: (v) => v === null ? "no estable" : `${v.toFixed(2)} s`, lower: true },
  { label: "t<sub>s</sub> +A → −A", get: (r) => r.segs[1]?.ts, sim: (s) => s.ts2, fmt: (v) => v === null ? "no estable" : `${v.toFixed(2)} s`, lower: true },
  { label: "M<sub>p</sub> 0 → +A", get: (r) => r.segs[0]?.mp, sim: (s) => s.mp1, fmt: (v) => `${v.toFixed(1)} %`, lower: true },
  { label: "M<sub>p</sub> +A → −A", get: (r) => r.segs[1]?.mp, sim: (s) => s.mp2, fmt: (v) => `${v.toFixed(1)} %`, lower: true },
  { label: "|e<sub>ss</sub>| máx", get: (r) => Math.max(...r.segs.map((s) => Math.abs(s.ess))), fmt: (v) => `${v.toFixed(3)} mm`, lower: true },
  { label: "ISE total", get: (r) => r.ise, fmt: (v) => `${v.toFixed(3)}`, lower: true },
  { label: "ISE 0 → +A → −A", get: (r) => (r.segs[0]?.ise ?? 0) + (r.segs[1]?.ise ?? 0), sim: (s) => s.ise, fmt: (v) => `${v.toFixed(3)}`, lower: true },
  { label: "|u<sub>ef</sub>| máx", get: (r) => Math.max(...r.segs.map((s) => s.umax)), fmt: (v) => `${v.toFixed(1)} %` },
];

function renderCmp() {
  const tb = $("tblCmp").querySelector("tbody");
  tb.innerHTML = "";
  const showSim = [results[1], results[2]].some((r) => r && Math.abs(r.A - 5) < 1e-6) || (!results[1] && !results[2]);
  for (const row of CMP_ROWS) {
    const tr = document.createElement("tr");
    const v = [1, 2].map((m) => (results[m] ? row.get(results[m]) : undefined));
    const both = v.every((x) => x !== undefined && x !== null);
    const best = both && row.lower ? (v[0] < v[1] ? 0 : v[1] < v[0] ? 1 : -1) : -1;
    tr.innerHTML = `<td>${row.label}</td>` + [1, 2].map((m, i) => {
      const val = v[i] === undefined ? "—" : row.fmt(v[i]);
      const sim = row.sim && showSim ? `<span class="sim">sim ${row.fmt(row.sim(SIM[m]))}</span>` : "";
      return `<td class="${best === i ? "win" : ""}">${val}${sim}</td>`;
    }).join("");
    tb.appendChild(tr);
  }
  $("btnCmpCsv").disabled = !results[1] && !results[2];
}
renderCmp();

$("btnCmpCsv").addEventListener("click", () => {
  const lines = [`# Lab 5 - pruebas de escalon, T_s = ${cfg.T.toFixed(4)} s`,
    "controlador,A_mm,tramo,r0_mm,rf_mm,ts_s,Mp_pct,ess_mm,ISE_mm2s,u_ef_max_pct"];
  for (const m of [1, 2]) {
    const r = results[m];
    if (!r) continue;
    r.segs.forEach((s, i) => lines.push([MODE_NAME[m], r.A, i + 1, s.r0, s.rf, s.ts === null ? "" : s.ts.toFixed(3),
      s.mp.toFixed(2), s.ess.toFixed(4), s.ise.toFixed(4), s.umax.toFixed(2)].join(",")));
  }
  lines.push("", "# muestras", "controlador,t_s,r_mm,y_mm,e_mm,u_ef_pct,u_pwm_pct");
  for (const m of [1, 2]) {
    const r = results[m];
    if (!r) continue;
    const t0 = r.data[0].t;
    for (const p of r.data) lines.push([MODE_NAME[m], (p.t - t0).toFixed(3), p.r, p.y.toFixed(3), p.e.toFixed(3), p.uef.toFixed(2), p.upwm.toFixed(1)].join(","));
  }
  download(`lab5_escalones_${stamp()}.csv`, lines.join("\n"));
});

/* ============================================================
 * Coeficientes
 * ============================================================ */
const fmtC = (v) => (Math.abs(v) < 1e-12 ? "0" : (+v.toPrecision(7)).toString());

function renderDiffEq() {
  const q0 = parseFloat($("cq0").value), q1 = parseFloat($("cq1").value), q2 = parseFloat($("cq2").value), s0 = parseFloat($("cs0").value);
  if ([q0, q1, q2, s0].some(Number.isNaN)) { $("diffeq").textContent = "—"; return; }
  const arg = (lag) => `(<i>k</i>${lag ? `−${lag}` : ""})`;
  const term = (c, v, lag, cls, first = false) =>
    `<span class="tm ${cls}"><span class="op">${first ? "" : c < 0 ? "−" : "+"}</span>` +
    `<b>${fmtC(first ? c : Math.abs(c))}</b><i>${v}</i>${arg(lag)}</span>`;
  let html = `<span class="eq-lhs"><i>u</i>(<i>k</i>)</span><span class="op">=</span>` + term(1 - s0, "u", 1, "mem", true);
  if (Math.abs(s0) > 1e-12) html += term(s0, "u", 2, "mem");
  html += term(q0, "e", 0, "err") + term(q1, "e", 1, "err") + term(q2, "e", 2, "err");
  $("diffeq").innerHTML = html;
}
function fillCoefs(src = cfg.coef[coefTab]) {
  $("cq0").value = fmtC(src.q0); $("cq1").value = fmtC(src.q1);
  $("cq2").value = fmtC(src.q2); $("cs0").value = fmtC(src.s0);
  document.querySelectorAll("#coefTabs button").forEach((b) => b.classList.toggle("active", b.dataset.c === String(coefTab)));
  renderDiffEq();
}
document.querySelectorAll("#coefTabs button").forEach((b) =>
  b.addEventListener("click", () => { coefTab = parseInt(b.dataset.c, 10); fillCoefs(); })
);
["cq0", "cq1", "cq2", "cs0"].forEach((id) => $(id).addEventListener("input", renderDiffEq));
$("btnCoef").addEventListener("click", () => {
  const v = ["cq0", "cq1", "cq2", "cs0"].map((id) => parseFloat($(id).value));
  if (v.some(Number.isNaN)) return toast("Completa los cuatro coeficientes.");
  const sumq = v[0] + v[1] + v[2];
  if (Math.abs(sumq) < 1e-9) toast("q0 + q1 + q2 = 0: el controlador no tendrá acción integral efectiva");
  sendLine(`Q ${coefTab} ${v.join(" ")}`);
});
$("btnCoefDef").addEventListener("click", () => fillCoefs(DESIGN[coefTab]));

/* Lazo abierto */
document.querySelectorAll(".bench-presets button").forEach((b) =>
  b.addEventListener("click", () => {
    const u = parseFloat(b.dataset.u), ms = parseInt(b.dataset.ms, 10);
    sendLine(u === 0 ? "S" : `P ${u} ${ms}`);
  })
);

/* ============================================================
 * Gráficas
 * ============================================================ */
const COL = {
  grid: "rgba(255,255,255,.07)", zero: "rgba(255,255,255,.24)", label: "rgba(220,224,240,.55)",
  y: "#f0b35e", ref: "rgba(240,242,250,.8)", mod: "#c9a0ff", err: "#ff8a7a",
  uef: "#62d0b8", pwm: "rgba(98,208,184,.5)", lim: "rgba(255,138,122,.35)", cursor: "rgba(255,255,255,.35)",
  ise: "#8b5cf6",
};
const MONO = "10.5px 'JetBrains Mono', ui-monospace, Consolas, monospace";
const PAD = { l: 40, r: 10 };

function fitCanvas(cv, c) {
  const k = window.devicePixelRatio || 1;
  cv.width = cv.clientWidth * k; cv.height = cv.clientHeight * k;
  c.setTransform(k, 0, 0, k, 0, 0);
  dirty = true;
}
new ResizeObserver(() => {
  fitCanvas(cvPos, ctx.pos); fitCanvas(cvErr, ctx.err); fitCanvas(cvU, ctx.u); fitCanvas(cvIse, ctx.ise);
}).observe(document.body);

function niceStep(range, target) {
  const raw = range / target, mag = 10 ** Math.floor(Math.log10(raw));
  for (const m of [1, 2, 2.5, 5, 10]) if (m * mag >= raw) return m * mag;
  return 10 * mag;
}

function timeWindow() {
  const tEnd = paused ? pausedEnd : (hist.length ? hist[hist.length - 1].t : 0);
  return { t1: tEnd, t0: tEnd - winSec };
}

function drawAxes(c, W, H, top, bottom, lo, hi, t0, t1, showTime, col = COL) {
  const py = (v) => top + (hi - v) / (hi - lo) * (H - top - bottom);
  const px = (t) => PAD.l + (t - t0) / (t1 - t0) * (W - PAD.l - PAD.r);
  c.font = MONO;
  c.lineWidth = 1;
  const ys = niceStep(hi - lo, 4);
  for (let v = Math.ceil(lo / ys) * ys; v <= hi + 1e-9; v += ys) {
    const y = Math.round(py(v)) + 0.5;
    c.strokeStyle = Math.abs(v) < 1e-9 ? col.zero : col.grid;
    c.beginPath(); c.moveTo(PAD.l, y); c.lineTo(W - PAD.r, y); c.stroke();
    c.fillStyle = col.label; c.textAlign = "right";
    c.fillText(+v.toFixed(2), PAD.l - 6, y + 3);
  }
  const ts = winSec <= 5 ? 1 : winSec <= 10 ? 2 : 5;
  for (let t = Math.ceil(t0 / ts) * ts; t <= t1; t += ts) {
    const x = Math.round(px(t)) + 0.5;
    c.strokeStyle = col.grid;
    c.beginPath(); c.moveTo(x, top); c.lineTo(x, H - bottom); c.stroke();
    if (showTime) {
      c.fillStyle = col.label; c.textAlign = "center";
      c.fillText(`${(t - t1).toFixed(0)} s`, x, H - 4);
    }
  }
  return { px, py };
}

function drawSeries(c, pts, key, px, py, color, { dash = [], fillTo = null, glow = true, step = false, width = 2 } = {}) {
  if (pts.length < 2) return;
  c.save();
  const path = () => {
    c.beginPath();
    pts.forEach((p, i) => {
      const x = px(p.t), y = py(p[key]);
      if (!i) return c.moveTo(x, y);
      if (step) c.lineTo(x, py(pts[i - 1][key]));
      c.lineTo(x, y);
    });
  };
  if (fillTo !== null) {
    const yBase = py(fillTo);
    const g = c.createLinearGradient(0, py(Math.max(...pts.map((p) => Math.abs(p[key])))), 0, yBase);
    g.addColorStop(0, color + "44"); g.addColorStop(1, color + "00");
    path();
    c.lineTo(px(pts[pts.length - 1].t), yBase); c.lineTo(px(pts[0].t), yBase); c.closePath();
    c.fillStyle = g; c.fill();
  }
  c.setLineDash(dash);
  c.strokeStyle = color; c.lineWidth = width; c.lineJoin = "round"; c.lineCap = "round";
  if (glow) { c.shadowColor = color; c.shadowBlur = 8; }
  path(); c.stroke();
  c.restore();
}

function hline(c, A, W, v, color) {
  const y = Math.round(A.py(v)) + 0.5;
  c.save(); c.setLineDash([3, 4]); c.strokeStyle = color; c.lineWidth = 1;
  c.beginPath(); c.moveTo(PAD.l, y); c.lineTo(W - PAD.r, y); c.stroke(); c.restore();
}

function shadeModes(c, pts, A, top, H, bottom) {
  /* banda tenue cuando el control está activo, con color por controlador */
  let i = 0;
  while (i < pts.length) {
    const m = pts[i].mode;
    let j = i;
    while (j < pts.length && pts[j].mode === m) j++;
    if (m) {
      c.fillStyle = m === 1 ? "rgba(240,179,94,.05)" : "rgba(98,208,184,.06)";
      c.fillRect(A.px(pts[i].t), top, A.px(pts[j - 1].t) - A.px(pts[i].t), H - top - bottom);
    }
    i = j;
  }
}

function drawCharts() {
  const { t0, t1 } = timeWindow();
  const pts = hist.filter((p) => p.t >= t0 - 0.05 && p.t <= t1 + 1e-9);
  const showMod = $("chkMod").checked;

  /* posición */
  let W = cvPos.clientWidth, H = cvPos.clientHeight;
  ctx.pos.clearRect(0, 0, W, H);
  let lo = -2, hi = 2;
  for (const p of pts) {
    lo = Math.min(lo, p.y, p.r); hi = Math.max(hi, p.y, p.r);
    if (showMod) { lo = Math.min(lo, p.ym); hi = Math.max(hi, p.ym); }
  }
  const padY = (hi - lo) * 0.08;
  const A = drawAxes(ctx.pos, W, H, 6, 6, lo - padY, hi + padY, t0, t1, false);
  shadeModes(ctx.pos, pts, A, 6, H, 6);
  drawSeries(ctx.pos, pts, "r", A.px, A.py, COL.ref, { dash: [6, 5], glow: false, step: true, width: 1.6 });
  if (showMod) drawSeries(ctx.pos, pts, "ym", A.px, A.py, COL.mod, { dash: [2, 4], glow: false, width: 1.8 });
  drawSeries(ctx.pos, pts, "y", A.px, A.py, COL.y);
  if (pts.length && !paused) {
    const p = pts[pts.length - 1];
    const cx = A.px(p.t), cy = A.py(p.y);
    ctx.pos.beginPath(); ctx.pos.arc(cx, cy, 8, 0, 6.3); ctx.pos.fillStyle = COL.y + "33"; ctx.pos.fill();
    ctx.pos.beginPath(); ctx.pos.arc(cx, cy, 4, 0, 6.3); ctx.pos.fillStyle = COL.y; ctx.pos.fill();
  }

  /* error */
  W = cvErr.clientWidth; H = cvErr.clientHeight;
  ctx.err.clearRect(0, 0, W, H);
  let em = 0.5;
  for (const p of pts) em = Math.max(em, Math.abs(p.e) * 1.1);
  const B = drawAxes(ctx.err, W, H, 6, 6, -em, em, t0, t1, false);
  drawSeries(ctx.err, pts, "e", B.px, B.py, COL.err, { fillTo: 0, step: true });

  /* control */
  W = cvU.clientWidth; H = cvU.clientHeight;
  ctx.u.clearRect(0, 0, W, H);
  const C = drawAxes(ctx.u, W, H, 6, 18, -105, 105, t0, t1, true);
  hline(ctx.u, C, W, cfg.umax, COL.lim); hline(ctx.u, C, W, cfg.umin, COL.lim);
  drawSeries(ctx.u, pts, "upwm", C.px, C.py, COL.pwm, { dash: [4, 4], glow: false, step: true, width: 1.4 });
  drawSeries(ctx.u, pts, "uef", C.px, C.py, COL.uef, { step: true });

  /* ISE (panel claro) */
  W = cvIse.clientWidth; H = cvIse.clientHeight;
  ctx.ise.clearRect(0, 0, W, H);
  let im = 0.5;
  for (const p of pts) im = Math.max(im, p.ise * 1.15);
  const light = { grid: "rgba(30,30,60,.06)", zero: "rgba(30,30,60,.15)", label: "rgba(60,60,90,.55)" };
  const D = drawAxes(ctx.ise, W, H, 6, 6, 0, im, t0, t1, false, light);
  drawSeries(ctx.ise, pts, "ise", D.px, D.py, COL.ise, { fillTo: 0, glow: false });

  /* cursor */
  const read = $("cursorRead");
  if (cursorT === null || !pts.length) { read.classList.add("hidden"); return; }
  let best = pts[0];
  for (const p of pts) if (Math.abs(p.t - cursorT) < Math.abs(best.t - cursorT)) best = p;
  for (const [c, P, cv, b] of [[ctx.pos, A, cvPos, 6], [ctx.err, B, cvErr, 6], [ctx.u, C, cvU, 18]]) {
    const x = Math.round(P.px(best.t)) + 0.5;
    c.strokeStyle = COL.cursor; c.beginPath(); c.moveTo(x, 6); c.lineTo(x, cv.clientHeight - b); c.stroke();
  }
  ctx.pos.beginPath(); ctx.pos.arc(A.px(best.t), A.py(best.y), 4, 0, 6.3); ctx.pos.fillStyle = COL.y; ctx.pos.fill();
  read.innerHTML = `t  ${(best.t - t1).toFixed(2)} s<br>r  ${best.r.toFixed(3)} mm<br>y  ${best.y.toFixed(3)} mm<br>` +
    `e  ${best.e.toFixed(3)} mm<br>u  ${best.uef.toFixed(2)} %<br>PWM ${best.upwm.toFixed(1)} %<br>${MODE_NAME[best.mode]}`;
  read.classList.remove("hidden");
}

for (const cv of [cvPos, cvErr, cvU]) {
  cv.addEventListener("pointermove", (e) => {
    const { t0, t1 } = timeWindow();
    const rect = cv.getBoundingClientRect();
    const f = (e.clientX - rect.left - PAD.l) / (rect.width - PAD.l - PAD.r);
    cursorT = f < 0 || f > 1 ? null : t0 + f * (t1 - t0);
    dirty = true;
  });
  cv.addEventListener("pointerleave", () => { cursorT = null; dirty = true; });
}

document.querySelectorAll("#segWin button").forEach((b) =>
  b.addEventListener("click", () => {
    document.querySelectorAll("#segWin button").forEach((o) => o.classList.toggle("active", o === b));
    winSec = parseInt(b.dataset.win, 10);
    dirty = true;
  })
);
$("btnPause").addEventListener("click", () => {
  paused = !paused;
  pausedEnd = hist.length ? hist[hist.length - 1].t : 0;
  $("btnPause").textContent = paused ? "▶" : "❚❚";
  $("btnPause").classList.toggle("on", paused);
  $("btnPause").title = paused ? "Reanudar gráfica" : "Pausar gráfica";
  dirty = true;
});
$("chkMod").addEventListener("change", () => { dirty = true; });

/* ============================================================
 * Bucle de dibujo
 * ============================================================ */
function frame() {
  if (dirty) {
    dirty = false;
    renderStats();
    drawCharts();
  }
  requestAnimationFrame(frame);
}

setTarget(0);
fillCoefs();
document.fonts?.ready.then(() => { paintSlider(parseFloat(sl.value)); dirty = true; });
requestAnimationFrame(frame);
