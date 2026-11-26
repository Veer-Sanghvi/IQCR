// Forward-kinematics check for kinematics.js against the IRB 120 numbers the
// repo already holds from MATLAB's Robotics System Toolbox (ABB's own model).
// Run: node tests/fk-check.js (npm test runs it before Playwright).
"use strict";
const fs = require("fs");
const path = require("path");
const assert = require("assert");

require("../kinematics.js");
const K = globalThis.IQCR_KIN;
const deg = (v) => (v * Math.PI) / 180;
const pos = (q) => K.posOf(K.fk(q));
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
let checks = 0;
const ok = (cond, msg) => { assert(cond, msg); checks++; };

// 1. Upright zero pose, ABB datasheet: TCP at x 374, y 0, z 630 mm.
const p0 = pos([0, 0, 0, 0, 0, 0]);
ok(dist(p0, [374, 0, 630]) < 1e-6, `q=0 gives ${p0.map((v) => v.toFixed(1))}`);

// 2. Target positions the toolbox recorded for specific joint vectors
//    (conference_paper/matlab_source/rst_v3_failure_forensics.txt, 0.1 mm print precision).
const forensics = [
  [[0, 62.98, 75.67, -24.59, -63.92, 70.88], [75.1, 26.9, 95.5]],
  [[0, -111.77, -38.17, -14.64, -84.59, -11.91], [-587.8, 18.1, 223.9]],
  [[0, -110.13, -14.13, -69.15, 73.71, -44.26], [-472.4, -64.6, 437.8]],
  [[0, -107.38, 76.33, 23.69, 26.62, -75.89], [35.3, 13.0, 433.0]],
  [[0, -110.7, -28.32, 1.3, -76.51, 69.51], [-585.1, -1.6, 297.9]],
];
for (const [qDeg, target] of forensics) {
  const p = pos(qDeg.map(deg));
  // the printed qTrue is rounded to 0.01 deg, so allow 0.5 mm
  ok(dist(p, target) < 0.5, `forensics ${qDeg} -> ${p.map((v) => v.toFixed(1))}, expected ${target}`);
}

// 3. Every (qTrue, qSol) pair in the toolbox CSVs: our FK must reproduce the
//    recorded |TCP(qTrue) - TCP(qSol)| in mm, including the stuck cases.
function csvRows(file) {
  const [head, ...lines] = fs.readFileSync(path.join(__dirname, "..", "conference_paper", "matlab_source", file), "utf8").trim().split(/\r?\n/);
  const cols = head.split(",");
  return lines.map((l) => Object.fromEntries(l.split(",").map((v, i) => [cols[i], Number(v)])));
}
for (const file of ["rst_v3_eight.csv", "rst_v3_legacy100.csv"]) {
  for (const r of csvRows(file)) {
    const qt = [1, 2, 3, 4, 5, 6].map((i) => r[`qTrue${i}`]);
    const qs = [1, 2, 3, 4, 5, 6].map((i) => r[`qSol${i}`]);
    const d = dist(pos(qt), pos(qs));
    // the CSV holds 15 significant digits; 1e-3 mm absorbs tool0 vs DH frame rounding
    ok(Math.abs(d - r.PosErr_mm) < 1e-3, `${file} case ${r.Case}: FK distance ${d} vs recorded ${r.PosErr_mm}`);
  }
}

// 4. Limits are the ABB datasheet values the toolbox model carries.
ok(JSON.stringify(K.QMIN.map((v) => Math.round(v * 180 / Math.PI))) === "[-165,-110,-110,-160,-120,-400]", "QMIN");
ok(JSON.stringify(K.QMAX.map((v) => Math.round(v * 180 / Math.PI))) === "[165,110,70,160,120,400]", "QMAX");

console.log(`fk-check: ${checks} checks passed; q=0 TCP = (${p0.map((v) => v.toFixed(1)).join(", ")}) mm`);
