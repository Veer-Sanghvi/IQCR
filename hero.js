// Showpiece hero: the same IRB 120 chain as scene.js (frames from kinematics.js's
// fkChain) dressed as solid hardware: painted links, graphite joint housings,
// a lime sensor at the flange, studio lighting from a procedural room
// environment, a soft contact shadow and a calibration grid on the floor.
// The model itself lives in arm-model.js (shared with scene.js) and is
// re-exported here so og/og-scene.html can render the identical arm as a still.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { buildArm, configureRenderer, jointAnchors, lowCapability, makeEnvironment, makeStudio } from "./arm-model.js";
export { buildArm, jointAnchors, makeEnvironment, makeStudio } from "./arm-model.js";

const Z_AXIS = new THREE.Vector3(0, 0, 1);

// Position-only damped-least-squares step toward a 3D point, using the same
// numerical Jacobian the page's IK solver uses. Orientation is left free so
// the wrist settles naturally while the tool point tracks the pointer.
function ikStep(K, q, target, gain = 0.35, lambda = 60) {
  const T = K.fk(q);
  const p = K.posOf(T);
  const e = [target.x - p[0], target.y - p[1], target.z - p[2]];
  const J = K.jacobian(q).slice(0, 3);
  // A = J J^T + lambda^2 I (3x3), solve A y = e, dq = J^T y
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) {
    let s = 0; for (let k = 0; k < 6; k++) s += J[r][k] * J[c][k];
    A[r][c] = s + (r === c ? lambda * lambda : 0);
  }
  const [[a, b, c], [d, e2, f], [g, h, i]] = A;
  const det = a * (e2 * i - f * h) - b * (d * i - f * g) + c * (d * h - e2 * g);
  if (Math.abs(det) < 1e-9) return q;
  const inv = [
    [(e2 * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e2) / det],
    [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
    [(d * h - e2 * g) / det, (b * g - a * h) / det, (a * e2 - b * d) / det],
  ];
  const y = inv.map((row) => row[0] * e[0] + row[1] * e[1] + row[2] * e[2]);
  return q.map((v, k) => {
    const dq = J[0][k] * y[0] + J[1][k] * y[1] + J[2][k] * y[2];
    return Math.max(K.QMIN[k], Math.min(K.QMAX[k], v + gain * dq));
  });
}

export function createHero(host) {
  const K = window.IQCR_KIN;
  const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)");
  const canvas = host.querySelector("canvas") || host.appendChild(document.createElement("canvas"));
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: "high-performance" });
  // shared settings (no synchronous shader-error queries, DPR, shadows, tone
  // mapping); low-capability machines get no shadows, no PMREM and DPR 1
  const low = lowCapability(renderer);
  configureRenderer(renderer, low);
  renderer.setClearColor(0x000000, 0);
  host.dataset.quality = low ? "low" : "full";

  const scene = new THREE.Scene();
  makeStudio(renderer, scene);
  const arm = buildArm(K);
  scene.add(arm.group);

  const camera = new THREE.PerspectiveCamera(28, 1, 10, 6000);
  camera.up.copy(Z_AXIS);
  camera.position.set(1150, -820, 420);
  const controls = new OrbitControls(camera, host);
  controls.target.set(140, -40, 330);
  controls.enableZoom = false;
  controls.enablePan = false;
  controls.enableDamping = true;
  controls.dampingFactor = 0.07;
  controls.rotateSpeed = 0.55;
  controls.minPolarAngle = 0.55;
  controls.maxPolarAngle = 1.42;
  controls.update();

  // ---- pointer target ----
  const CENTER = new THREE.Vector3(360, -150, 330);
  const SHOULDER = new THREE.Vector3(0, 0, K.DH_D[0]);
  const REACH = 540;
  const targetRaw = CENTER.clone();
  const target = CENTER.clone();
  let q = K.HOME_Q.slice();
  let lastPointer = -Infinity;
  let dragging = false;
  let pointerDown = null;
  const ray = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  const plane = new THREE.Plane();
  const hit = new THREE.Vector3();

  function pointerToTarget(ev) {
    const r = host.getBoundingClientRect();
    ndc.set(((ev.clientX - r.left) / r.width) * 2 - 1, -((ev.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    const n = camera.getWorldDirection(new THREE.Vector3()).negate();
    plane.setFromNormalAndCoplanarPoint(n, CENTER);
    if (!ray.ray.intersectPlane(plane, hit)) return;
    const off = hit.clone().sub(SHOULDER);
    if (off.length() > REACH) off.setLength(REACH);
    hit.copy(SHOULDER).add(off);
    hit.z = Math.max(70, hit.z);
    targetRaw.copy(hit);
    lastPointer = performance.now();
  }
  host.addEventListener("pointermove", (ev) => {
    if (reduceMotion.matches) return;
    if (ev.pointerType === "touch" || dragging) return;
    pointerToTarget(ev);
  });
  host.addEventListener("pointerdown", (ev) => { pointerDown = { x: ev.clientX, y: ev.clientY, t: performance.now() }; });
  host.addEventListener("pointerup", (ev) => {
    if (!pointerDown) return;
    const moved = Math.hypot(ev.clientX - pointerDown.x, ev.clientY - pointerDown.y);
    if (moved < 6 && !reduceMotion.matches) pointerToTarget(ev); // a tap places the target
    pointerDown = null;
  });
  host.addEventListener("pointerleave", () => { lastPointer = performance.now() - 1800; });
  controls.addEventListener("start", () => { dragging = true; });
  controls.addEventListener("end", () => { dragging = false; });

  // ---- annotation overlay: J1 to J6 callouts and live angles ----
  // Each tag sits in a column on its joint's side of the arm (the three
  // leftmost joints on screen go left, the other three right) at its joint's
  // own height, pushed apart just enough not to overlap and damped so it
  // glides. Leaders are therefore short and near-horizontal, and two leaders
  // on one side can only cross if a tag is shoved past its neighbour's joint,
  // which the spread never does. On narrow screens the CSS lays the tags out
  // as a static row and hides the leaders, and the transforms set here are
  // overridden.
  const tags = Array.from(host.querySelectorAll("[data-joint]"));
  const lines = Array.from(host.querySelectorAll("[data-leader]"));
  const dots = Array.from(host.querySelectorAll("[data-leader-dot]"));
  const tcp = host.querySelector("[data-tcp]");
  const copy = host.querySelector(".showpiece-copy");
  const tagPos = tags.map(() => ({ x: NaN, y: NaN }));
  const fmtDeg = (rad) => { const d = (rad * 180) / Math.PI; return ((d >= 0 ? "+" : "−") + Math.abs(d).toFixed(1)).padStart(6, " ") + "°"; };
  function updateOverlay(frames) {
    const W = host.clientWidth, H = host.clientHeight;
    const pts = frames.map((T) => new THREE.Vector3(T[0][3], T[1][3], T[2][3]));
    const anchors = jointAnchors(frames);
    const v = new THREE.Vector3();
    const screen = anchors.map((a) => { v.copy(a).project(camera); return [(v.x * 0.5 + 0.5) * W, (-v.y * 0.5 + 0.5) * H]; });
    tags.forEach((tag, j) => { tag.lastElementChild.textContent = fmtDeg(q[j]); });
    if (tcp) {
      const p = pts[6];
      tcp.textContent = `TCP x ${p.x.toFixed(0)} · y ${p.y.toFixed(0)} · z ${p.z.toFixed(0)} mm`;
    }
    const pad = copy ? copy.offsetLeft : 24;
    const tagW = tags[0].offsetWidth, tagH = tags[0].offsetHeight, pitch = tagH + 6;
    const top = pad * 0.75, bottom = (copy ? copy.offsetTop : H) - 12 - tagH;
    const byX = screen.map((p, j) => [p[0], j]).sort((a, b) => a[0] - b[0]).map(([, j]) => j);
    [byX.slice(0, 3), byX.slice(3)].forEach((side, s) => {
      const left = s === 0;
      side.sort((a, b) => screen[a][1] - screen[b][1]);
      const ys = side.map((j) => Math.min(bottom, Math.max(top, screen[j][1] - tagH / 2)));
      for (let k = 1; k < ys.length; k++) ys[k] = Math.max(ys[k], ys[k - 1] + pitch);
      ys[ys.length - 1] = Math.min(ys[ys.length - 1], bottom);
      for (let k = ys.length - 2; k >= 0; k--) ys[k] = Math.min(ys[k], ys[k + 1] - pitch);
      side.forEach((j, k) => {
        const p = tagPos[j], tx = left ? pad : W - pad - tagW, ty = ys[k];
        if (Number.isNaN(p.x)) { p.x = tx; p.y = ty; } else { p.x += (tx - p.x) * 0.22; p.y += (ty - p.y) * 0.22; }
        tags[j].style.transform = `translate(${p.x.toFixed(1)}px,${p.y.toFixed(1)}px)`;
        const [x, y] = screen[j], line = lines[j], dot = dots[j];
        if (line) line.setAttribute("x1", left ? p.x + tagW + 4 : p.x - 4), line.setAttribute("y1", p.y + tagH / 2), line.setAttribute("x2", x), line.setAttribute("y2", y);
        if (dot) dot.setAttribute("cx", x), dot.setAttribute("cy", y);
      });
    });
  }

  // ---- loop state (declared before resize() uses it) ----
  let visible = true, hidden = document.hidden, running = false, needsRender = true, viewerActive = false;

  // ---- sizing without layout shift: the host already has its CSS height ----
  function resize() {
    const w = host.clientWidth, h = host.clientHeight;
    if (!w || !h) return;
    camera.aspect = w / h;
    // portrait phones: wider lens, camera pulled back and aimed lower so the
    // whole arm sits above the copy; desktop keeps the long-lens studio look
    const portrait = camera.aspect < 0.9;
    camera.fov = portrait ? 50 : camera.aspect < 1.4 ? 34 : 28;
    controls.target.set(portrait ? 120 : 140, portrait ? -60 : -40, portrait ? 340 : 330);
    const dir = camera.position.clone().sub(controls.target).normalize();
    camera.position.copy(controls.target).addScaledVector(dir, portrait ? 1550 : 1500);
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
    needsRender = true;
  }
  new ResizeObserver(resize).observe(host);
  resize();

  // ---- loop: idle path when untouched, paused offscreen, in a hidden tab or
  // while the lab viewer below is animating. The idle sweep is capped at 30
  // frames a second (15 on low-capability machines); a pointer or an orbit
  // still gets every frame. ----
  const t0 = performance.now();
  const idleMs = low ? 1000 / 15 : 1000 / 30;
  let lastDraw = -Infinity;
  function frame(now) {
    running = false;
    if (!visible || hidden || viewerActive) return;
    const engaged = dragging || now - lastPointer < 2600;
    if (!engaged && !needsRender && now - lastDraw < idleMs) { schedule(); return; }
    lastDraw = now;
    const t = (now - t0) / 1000;
    if (!reduceMotion.matches) {
      if (now - lastPointer > 2600 && !dragging) {
        // slow idle sweep, a Lissajous drift around the working point
        targetRaw.set(CENTER.x + 150 * Math.sin(t * 0.37), CENTER.y + 120 * Math.sin(t * 0.29 + 1.2), CENTER.z + 95 * Math.sin(t * 0.23 + 2.1));
      }
      target.lerp(targetRaw, 0.085);
      q = ikStep(K, q, target);
      arm.glow.material.opacity = 0.48 + 0.1 * Math.sin(t * 2.4);
      needsRender = true;
    }
    controls.update();
    const { frames } = K.fkChain(q);
    arm.update(frames);
    updateOverlay(frames);
    if (needsRender || !reduceMotion.matches) renderer.render(scene, camera);
    needsRender = false;
    schedule();
  }
  function schedule() {
    if (running || !visible || hidden) return;
    running = true;
    requestAnimationFrame(frame);
  }
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; schedule(); }, { threshold: 0.02 }).observe(host);
  document.addEventListener("visibilitychange", () => { hidden = document.hidden; schedule(); });
  document.addEventListener("iqcr:viewer-active", (e) => { viewerActive = !!e.detail; schedule(); });
  controls.addEventListener("change", () => { needsRender = true; schedule(); });
  reduceMotion.addEventListener("change", () => { q = K.HOME_Q.slice(); target.copy(CENTER); needsRender = true; schedule(); });

  // first frame synchronously so the canvas is never blank once it exists
  const { frames } = K.fkChain(q);
  arm.update(frames);
  updateOverlay(frames);
  renderer.render(scene, camera);
  host.dataset.hero = "ready";
  schedule();
  // the reflection environment is the single most expensive setup step, so it
  // lands in its own idle task after the first frame is already on screen
  const addEnv = () => { makeEnvironment(renderer, scene); needsRender = true; schedule(); };
  if ("requestIdleCallback" in window) requestIdleCallback(addEnv, { timeout: 2000 }); else setTimeout(addEnv, 250);
  return { renderer, scene, camera, arm };
}
