// ABB IRB120 forward/inverse kinematics — direct JS port of iqcr_kinematics.m.
// Row-major 4x4 matrices as plain arrays: T[row][col], translation in T[i][3].
"use strict";

const DH_A     = [0, 270, 70, 0, 0, 0];               // mm
const DH_ALPHA = [0, -90, 0, -90, 90, -90].map(d => d * Math.PI / 180); // rad
const DH_D     = [290, 0, 0, 302, 0, 72];              // mm

const QMIN = [-150, -100, -90, -150, -100, -150].map(d => d * Math.PI / 180);
const QMAX = [150, 100, 50, 150, 100, 150].map(d => d * Math.PI / 180);
const HOME_Q = [0, 0.35, 0.35, 0, 0.5, 0]; // a visually pleasant, fully-in-limits starting pose

function mat4Identity() {
  return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
}

function dhMatrix(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}

function matMul4(A, B) {
  const C = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += A[i][k] * B[k][j];
    C[i][j] = s;
  }
  return C;
}

// Forward kinematics: q is an array of 6 joint angles (rad). Returns {T, frames}
// where frames[i] is the world transform after joint i (frames[0] = base).
function fkChain(q) {
  let T = mat4Identity();
  const frames = [T];
  for (let i = 0; i < 6; i++) {
    T = matMul4(T, dhMatrix(q[i], DH_D[i], DH_A[i], DH_ALPHA[i]));
    frames.push(T);
  }
  return { T, frames };
}

function fk(q) { return fkChain(q).T; }

function transpose3(R) {
  return [[R[0][0], R[1][0], R[2][0]], [R[0][1], R[1][1], R[2][1]], [R[0][2], R[1][2], R[2][2]]];
}
function matMul3(A, B) {
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let s = 0;
    for (let k = 0; k < 3; k++) s += A[i][k] * B[k][j];
    C[i][j] = s;
  }
  return C;
}
function rotOf(T) { return [[T[0][0], T[0][1], T[0][2]], [T[1][0], T[1][1], T[1][2]], [T[2][0], T[2][1], T[2][2]]]; }
function posOf(T) { return [T[0][3], T[1][3], T[2][3]]; }
function vsub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function vnorm(v) { return Math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2); }
function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

// axis-angle (log map) orientation error, small-angle safe — mirrors the MATLAB block exactly.
function orientationError(Ttarget, Tcur) {
  const Rt = rotOf(Ttarget), Rc = rotOf(Tcur);
  const Rerr = matMul3(Rt, transpose3(Rc));
  const cosang = clamp((Rerr[0][0] + Rerr[1][1] + Rerr[2][2] - 1) / 2, -1, 1);
  const ang = Math.acos(cosang);
  if (ang < 1e-9) return { oriErr: [0, 0, 0], angDeg: 0 };
  const axis = [
    (Rerr[2][1] - Rerr[1][2]) / (2 * Math.sin(ang)),
    (Rerr[0][2] - Rerr[2][0]) / (2 * Math.sin(ang)),
    (Rerr[1][0] - Rerr[0][1]) / (2 * Math.sin(ang)),
  ];
  return { oriErr: axis.map((a) => a * ang), angDeg: ang * 180 / Math.PI };
}

// 6x6 linear solve via Gaussian elimination with partial pivoting.
function solve6(Ain, bin) {
  const n = 6;
  const A = Ain.map((r) => r.slice());
  const b = bin.slice();
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    if (piv !== col) { [A[col], A[piv]] = [A[piv], A[col]]; [b[col], b[piv]] = [b[piv], b[col]]; }
    const pivVal = A[col][col] || 1e-12;
    for (let r = col + 1; r < n; r++) {
      const f = A[r][col] / pivVal;
      for (let c = col; c < n; c++) A[r][c] -= f * A[col][c];
      b[r] -= f * b[col];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r];
    for (let c = r + 1; c < n; c++) s -= A[r][c] * x[c];
    x[r] = s / (A[r][r] || 1e-12);
  }
  return x;
}

// Damped least-squares (Levenberg-Marquardt) numerical IK — same algorithm and
// tolerances as the paper's MATLAB script (lambda=1e-3, central-difference
// Jacobian with h=1e-6, maxIter=200, tol=1e-9).
function ikSolve(q0, Ttarget, { lambda = 1e-3, maxIter = 200, tol = 1e-9 } = {}) {
  let q = q0.slice();
  const trace = [q.slice()];
  const errNorms = [];
  let iter = 0;
  const h = 1e-6;

  for (let it = 1; it <= maxIter; it++) {
    const Tcur = fk(q);
    const posErr = vsub(posOf(Ttarget), posOf(Tcur));
    const { oriErr } = orientationError(Ttarget, Tcur);
    const e = [...posErr, ...oriErr];
    errNorms.push(vnorm(posErr));
    iter = it;
    const eNorm = Math.sqrt(e.reduce((s, v) => s + v * v, 0));
    if (eNorm < tol) break;

    const J = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]];
    for (let j = 0; j < 6; j++) {
      const qp = q.slice(); qp[j] += h;
      const qm = q.slice(); qm[j] -= h;
      const Tp = fk(qp), Tm = fk(qm);
      const dpos = [0, 1, 2].map((k) => (Tp[k][3] - Tm[k][3]) / (2 * h));
      const Rp = rotOf(Tp), Rm = rotOf(Tm);
      const dR = [[0,0,0],[0,0,0],[0,0,0]];
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) dR[a][b] = (Rp[a][b] - Rm[a][b]) / (2 * h);
      const S = matMul3(dR, transpose3(Rm));
      const w = [S[2][1], S[0][2], S[1][0]];
      const col = [...dpos, ...w];
      for (let r = 0; r < 6; r++) J[r][j] = col[r];
    }
    // (J'J + lambda I) dq = J' e
    const JT = J[0].map((_, c) => J.map((row) => row[c])); // transpose 6x6
    const JTJ = JT.map((row, r) => JT[0].map((_, c) => {
      let s = 0; for (let k = 0; k < 6; k++) s += JT[r][k] * J[k][c]; return s + (r === c ? lambda : 0);
    }));
    const JTe = JT.map((row) => row.reduce((s, v, k) => s + v * e[k], 0));
    const dq = solve6(JTJ, JTe);
    q = q.map((v, i) => v + dq[i]);
    trace.push(q.slice());
  }

  const Tfinal = fk(q);
  const posErrFinal = vnorm(vsub(posOf(Ttarget), posOf(Tfinal)));
  const { angDeg: oriErrFinalDeg } = orientationError(Ttarget, Tfinal);
  return { q, trace, iterations: iter, posErrFinal, oriErrFinalDeg, converged: posErrFinal < 1e-3, errNorms };
}

function randomQ(rng) {
  return QMIN.map((lo, i) => lo + rng() * (QMAX[i] - lo));
}

function mulberry32(seed) {
  return function () {
    seed |= 0; seed = (seed + 0x6D2B79F5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// Reachable workspace envelope in the X-Z plane, base joint fixed at 0 —
// same sweep as the paper's fig_workspace.png (60x60 grid over joints 2 & 3).
function workspaceEnvelope(steps = 60) {
  const pts = [];
  for (let i = 0; i < steps; i++) {
    const th2 = QMIN[1] + (i / (steps - 1)) * (QMAX[1] - QMIN[1]);
    for (let j = 0; j < steps; j++) {
      const th3 = QMIN[2] + (j / (steps - 1)) * (QMAX[2] - QMIN[2]);
      const Tw = fk([0, th2, th3, 0, 0, 0]);
      pts.push([Tw[0][3], Tw[2][3]]);
    }
  }
  return pts;
}

window.IQCR_KIN = {
  DH_A, DH_ALPHA, DH_D, QMIN, QMAX, HOME_Q,
  fk, fkChain, ikSolve, randomQ, mulberry32, workspaceEnvelope, posOf, rotOf,
};
