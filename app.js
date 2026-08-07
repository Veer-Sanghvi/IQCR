// IQCR dashboard: UI wiring, SVG charts, IK demo orchestration.
import { createRobotScene } from "./scene.js";

const K = window.IQCR_KIN;
const $ = (id) => document.getElementById(id);
const NS = "http://www.w3.org/2000/svg";
const deg2rad = (d) => (d * Math.PI) / 180;
const rad2deg = (r) => (r * 180) / Math.PI;
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

// ---------------- paper's actual results (embedded, static) ----------------
const PAPER_RESULTS = [
  [1, 2.56840872523143e-10, 5, 1], [2, 2.96280235187255e-11, 5, 1],
  [3, 1.29606111380682e-10, 5, 1], [4, 2.00172864791411e-10, 5, 1],
  [5, 1.68627285884926e-11, 7, 1], [6, 2.53891625088414e-11, 7, 1],
  [7, 6.356876055618e-13, 5, 1], [8, 1.36187151732781e-12, 6, 1],
];
const RST_RESULTS = [
  [1, 9.92833020929689e-07, 28, 1], [2, 4.82689411074375e-07, 25, 1],
  [3, 3.97715063606403e-06, 27, 1], [4, 1.4208057327512e-05, 23, 1],
  [5, 1.40913029711185e-05, 26, 1], [6, 5.73109690567023e-06, 29, 1],
  [7, 7.01640592362468e-06, 27, 1], [8, 32.6579568741827, 1500, 0],
];

function fillStaticTable(bodyId, rows) {
  const tbody = $(bodyId);
  tbody.innerHTML = rows.map(([c, pos, it, ok]) =>
    `<tr class="${ok ? "" : "fail"}"><td>${c}</td><td>${pos < 1e-3 ? pos.toExponential(2) : pos.toFixed(3)}</td><td>${it}</td><td>${ok ? "✓" : "✗"}</td></tr>`
  ).join("");
}
fillStaticTable("paper-table", PAPER_RESULTS);
fillStaticTable("rst-table", RST_RESULTS);

// ---------------- SVG chart helpers ----------------
function svgEl(tag, attrs = {}) { const el = document.createElementNS(NS, tag); for (const k in attrs) el.setAttribute(k, attrs[k]); return el; }
function clearSvg(svg) { while (svg.firstChild) svg.removeChild(svg.firstChild); }
function frame(svg, { x0, y0, w, h, xDomain, yDomain, xTicks = 5, yTicks = 5, xFmt = (v) => v, yFmt = (v) => v, xLabel, yLabel }) {
  const xs = (v) => x0 + ((v - xDomain[0]) / (xDomain[1] - xDomain[0])) * w;
  const ys = (v) => y0 + h - ((v - yDomain[0]) / (yDomain[1] - yDomain[0])) * h;
  if (yTicks > 0) for (let i = 0; i <= yTicks; i++) {
    const v = yDomain[0] + (i / yTicks) * (yDomain[1] - yDomain[0]);
    const y = ys(v);
    svg.appendChild(svgEl("line", { x1: x0, x2: x0 + w, y1: y, y2: y, class: "grid-line" }));
    const t = svgEl("text", { x: x0 - 8, y: y + 3, "text-anchor": "end", class: "axis-label" }); t.textContent = yFmt(v); svg.appendChild(t);
  }
  if (xTicks > 0) for (let i = 0; i <= xTicks; i++) {
    const v = xDomain[0] + (i / xTicks) * (xDomain[1] - xDomain[0]);
    const x = xs(v);
    const t = svgEl("text", { x, y: y0 + h + 18, "text-anchor": "middle", class: "axis-label" }); t.textContent = xFmt(v); svg.appendChild(t);
  }
  svg.appendChild(svgEl("line", { x1: x0, x2: x0 + w, y1: y0 + h, y2: y0 + h, class: "baseline" }));
  if (xLabel) { const t = svgEl("text", { x: x0 + w / 2, y: y0 + h + 38, "text-anchor": "middle", class: "axis-label" }); t.textContent = xLabel; svg.appendChild(t); }
  if (yLabel) { const t = svgEl("text", { x: 14, y: y0 + h / 2, "text-anchor": "middle", class: "axis-label", transform: `rotate(-90 14 ${y0 + h / 2})` }); t.textContent = yLabel; svg.appendChild(t); }
  return { xs, ys };
}
function pathFrom(pts, xs, ys) { return pts.map((p, i) => `${i === 0 ? "M" : "L"} ${xs(p[0]).toFixed(2)} ${ys(p[1]).toFixed(2)}`).join(" "); }
const CURVE_COLORS = ["#e07a3f", "#4c93e8", "#0ca30c", "#e66767", "#c98500", "#9085e9", "#199e70", "#d55181"];

// ---------------- scene ----------------
const scene = createRobotScene($("canvas-host"));

function qFromSliders() {
  return [0, 1, 2, 3, 4, 5].map((i) => deg2rad(+$(`q${i}`).value));
}
function updateEEReadout(pos) {
  $("ee-x").textContent = pos[0].toFixed(0);
  $("ee-y").textContent = pos[1].toFixed(0);
  $("ee-z").textContent = pos[2].toFixed(0);
}
function renderManual() {
  const q = qFromSliders();
  [0, 1, 2, 3, 4, 5].forEach((i) => { $(`v-q${i}`).textContent = `${(+$(`q${i}`).value).toFixed(0)}°`; });
  const { frames } = K.fkChain(q);
  const pos = scene.updatePose(frames);
  updateEEReadout(pos);
}
[0, 1, 2, 3, 4, 5].forEach((i) => $(`q${i}`).addEventListener("input", renderManual));
$("reset-manual").addEventListener("click", () => {
  const defaults = [0, 20, 20, 0, 29, 0];
  defaults.forEach((v, i) => { $(`q${i}`).value = v; });
  renderManual();
});

// ---------------- mode tabs ----------------
let mode = "manual";
function setMode(next) {
  mode = next;
  $("tab-manual").classList.toggle("active", mode === "manual");
  $("tab-ik").classList.toggle("active", mode === "ik");
  $("panel-manual").style.display = mode === "manual" ? "flex" : "none";
  $("panel-ik").style.display = mode === "ik" ? "flex" : "none";
  $("mode-pill").textContent = mode === "manual" ? "MANUAL POSE" : "INVERSE KINEMATICS";
  scene.setTarget(null, mode === "ik" && currentTarget ? true : false);
  if (mode === "ik" && currentTarget) scene.setTarget(currentTarget, $("show-target").checked);
}
$("tab-manual").addEventListener("click", () => setMode("manual"));
$("tab-ik").addEventListener("click", () => setMode("ik"));

// ---------------- IK demo ----------------
let currentTarget = null, currentQInit = null, caseCounter = 0;
const liveCurves = []; // {label, errNorms, color}

function renderConvergenceChart() {
  const svg = $("chart-convergence");
  clearSvg(svg);
  const x0 = 54, y0 = 14, w = 380, h = 190;
  if (liveCurves.length === 0) {
    frame(svg, { x0, y0, w, h, xDomain: [0, 30], yDomain: [-3, 3], xTicks: 3, yTicks: 0, xLabel: "iteration", yLabel: "log₁₀ |pos err| (mm)" });
    const t = svgEl("text", { x: x0 + w / 2, y: y0 + h / 2, "text-anchor": "middle", class: "axis-label" });
    t.textContent = "solve a target to see it converge"; svg.appendChild(t);
    return;
  }
  const allIters = liveCurves.flatMap((c) => c.errNorms.map((_, i) => i));
  const allLogErr = liveCurves.flatMap((c) => c.errNorms.map((e) => Math.log10(Math.max(e, 1e-13))));
  const { xs, ys } = frame(svg, {
    x0, y0, w, h,
    xDomain: [0, Math.max(10, ...allIters)], yDomain: [Math.min(...allLogErr) - 0.5, Math.max(...allLogErr) + 0.5],
    xTicks: 5, yTicks: 4, xFmt: (v) => v.toFixed(0), yFmt: (v) => `10^${v.toFixed(0)}`,
    xLabel: "iteration", yLabel: "|pos err| mm, log",
  });
  liveCurves.forEach((c) => {
    const pts = c.errNorms.map((e, i) => [i, Math.log10(Math.max(e, 1e-13))]);
    svg.appendChild(svgEl("path", { d: pathFrom(pts, xs, ys), fill: "none", stroke: c.color, "stroke-width": 2, "stroke-linecap": "round" }));
  });
}
renderConvergenceChart();

function appendLiveRow(caseNum, posErr, oriErr, iters, converged) {
  const tbody = document.querySelector("#live-table tbody");
  const tr = document.createElement("tr");
  if (!converged) tr.classList.add("fail");
  tr.innerHTML = `<td>${caseNum}</td><td>${posErr < 1e-3 ? posErr.toExponential(2) : posErr.toFixed(3)}</td><td>${oriErr.toFixed(4)}</td><td>${iters}</td><td>${converged ? "✓" : "✗"}</td>`;
  tbody.prepend(tr);
}

function currentPerturbDeg() { return +$("perturb").value; }
function currentLambda() { return Math.pow(10, +$("lambda").value); }

function generateTarget() {
  const qTrue = K.randomQ(Math.random);
  currentTarget = K.fk(qTrue);
  const perturb = deg2rad(currentPerturbDeg());
  currentQInit = qTrue.map((v, i) => clamp(v + perturb * (Math.random() - 0.5) * 2, K.QMIN[i], K.QMAX[i]));
  const { frames } = K.fkChain(currentQInit);
  const pos = scene.updatePose(frames);
  updateEEReadout(pos);
  scene.setTarget(currentTarget, $("show-target").checked);
  $("solve-ik").disabled = false;
  $("ik-result").innerHTML = `<p class="small" style="margin-top:8px;">Target generated. Arm placed at a perturbed starting guess. Click Solve.</p>`;
}

function solveIK(onDone) {
  if (!currentTarget || !currentQInit) { if (onDone) onDone(); return; }
  $("solve-ik").disabled = true;
  const result = K.ikSolve(currentQInit, currentTarget, { lambda: currentLambda() });
  // animate through the trace
  let i = 0;
  const stepEvery = Math.max(15, Math.min(60, Math.floor(600 / Math.max(result.trace.length, 1))));
  const timer = setInterval(() => {
    const q = result.trace[Math.min(i, result.trace.length - 1)];
    const { frames } = K.fkChain(q);
    const pos = scene.updatePose(frames);
    updateEEReadout(pos);
    i++;
    if (i >= result.trace.length) {
      clearInterval(timer);
      caseCounter++;
      const color = CURVE_COLORS[(caseCounter - 1) % CURVE_COLORS.length];
      liveCurves.push({ label: `#${caseCounter}`, errNorms: result.errNorms, color });
      if (liveCurves.length > 8) liveCurves.shift();
      renderConvergenceChart();
      appendLiveRow(caseCounter, result.posErrFinal, result.oriErrFinalDeg, result.iterations, result.converged);
      $("ik-result").innerHTML = result.converged
        ? `<span class="badge good">converged</span> <span class="small">${result.posErrFinal.toExponential(2)} mm, ${result.iterations} iterations</span>`
        : `<span class="badge bad">did not converge</span> <span class="small">${result.posErrFinal.toFixed(2)} mm off after ${result.iterations} iterations. This happens; see analysis below.</span>`;
      $("solve-ik").disabled = false;
      if (onDone) onDone();
    }
  }, stepEvery);
}

$("gen-target").addEventListener("click", generateTarget);
$("show-target").addEventListener("change", () => { if (currentTarget) scene.setTarget(currentTarget, $("show-target").checked); });
$("solve-ik").addEventListener("click", () => solveIK());

// ---------------- auto-demo (plays until the visitor touches anything) ----------------
let autoDemoActive = true;
let autoDemoTimer = null;

function stopAutoDemo() {
  if (!autoDemoActive) return;
  autoDemoActive = false;
  clearTimeout(autoDemoTimer);
  const badge = document.getElementById("auto-demo-badge");
  if (badge) badge.classList.add("hidden");
  // clear the demo's own runs so the visitor's own solves start from a clean slate
  liveCurves.length = 0;
  caseCounter = 0;
  document.querySelector("#live-table tbody").innerHTML = "";
  renderConvergenceChart();
}

function autoDemoLoop() {
  if (!autoDemoActive) return;
  setMode("ik");
  generateTarget();
  autoDemoTimer = setTimeout(() => {
    if (!autoDemoActive) return;
    solveIK(() => {
      if (!autoDemoActive) return;
      autoDemoTimer = setTimeout(autoDemoLoop, 3200);
    });
  }, 1100);
}

document.getElementById("viewer").addEventListener("pointerdown", stopAutoDemo, { once: true });
autoDemoTimer = setTimeout(autoDemoLoop, 1400);

// ---------------- IK solver-parameter sliders ----------------
$("perturb").addEventListener("input", () => { $("v-perturb").textContent = `±${currentPerturbDeg()}°`; });
$("lambda").addEventListener("input", () => { $("v-lambda").textContent = currentLambda().toExponential(0); });

// ---------------- workspace envelope ----------------
const envelopePts = K.workspaceEnvelope(60);
scene.setEnvelopePoints(envelopePts.map(([x, z]) => [x, 0, z]));
$("show-envelope").addEventListener("change", (e) => scene.setEnvelopeVisible(e.target.checked));

function renderEnvelopeChart() {
  const svg = $("chart-envelope");
  clearSvg(svg);
  const x0 = 64, y0 = 20, w = 500, h = 400;
  const xs_ = envelopePts.map((p) => p[0]), zs_ = envelopePts.map((p) => p[1]);
  const xMin = Math.min(...xs_), xMax = Math.max(...xs_);
  const zMin = Math.min(...zs_), zMax = Math.max(...zs_);
  const pad = 30;
  const { xs, ys } = frame(svg, {
    x0, y0, w, h,
    xDomain: [xMin - pad, xMax + pad], yDomain: [zMin - pad, zMax + pad],
    xTicks: 5, yTicks: 5, xFmt: (v) => v.toFixed(0), yFmt: (v) => v.toFixed(0),
    xLabel: "X (mm)", yLabel: "Z (mm)",
  });
  envelopePts.forEach(([x, z]) => {
    svg.appendChild(svgEl("circle", { cx: xs(x), cy: ys(z), r: 1.4, fill: "var(--amber)", opacity: 0.55 }));
  });
  // base marker
  svg.appendChild(svgEl("circle", { cx: xs(0), cy: ys(0), r: 5, fill: "none", stroke: "var(--blue)", "stroke-width": 2 }));
  const lbl = svgEl("text", { x: xs(0) + 10, y: ys(0) - 8, class: "axis-label", fill: "var(--blue)" });
  lbl.textContent = "base joint";
  svg.appendChild(lbl);
}
renderEnvelopeChart();

// ---------------- init ----------------
renderManual();
