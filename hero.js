// Showpiece hero: the same IRB 120 chain as scene.js (frames from kinematics.js's
// fkChain) dressed as solid hardware: painted links, graphite joint housings,
// a lime sensor at the flange, studio lighting from a procedural room
// environment, a soft contact shadow and a calibration grid on the floor.
// Exports buildArm() and makeStudio() so og/og-scene.html can render the
// identical arm as a still, and createHero() for the live page.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

const LIME = 0xC6F432;
const PAINT = 0xEDECE8;
const GRAPHITE = 0x242427;
const X_AXIS = new THREE.Vector3(1, 0, 0);
const Y_AXIS = new THREE.Vector3(0, 1, 0);
const Z_AXIS = new THREE.Vector3(0, 0, 1);

function radialTexture(inner, outer, size = 256, hold = 0) {
  const c = document.createElement("canvas");
  c.width = c.height = size;
  const g = c.getContext("2d");
  const grad = g.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  grad.addColorStop(0, inner);
  if (hold > 0) grad.addColorStop(hold, inner);
  grad.addColorStop(1, outer);
  g.fillStyle = grad;
  g.fillRect(0, 0, size, size);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// The arm. Link i lives in a group whose matrix is frames[i + 1]; in that frame
// the link runs from P_i = Rx(-alpha_i) * (-a_i, 0, -d_i) to the origin, which is
// pose independent, so only the six group matrices change per frame.
// Proportions follow the IRB 120: a rounded base plate with a turret, a chunky
// shoulder housing hung off one side of the turret, a tapered upper arm, the
// J4 motor housing behind the elbow, a slim forearm and a compact wrist.
export function buildArm(K) {
  const group = new THREE.Group();
  const paint = new THREE.MeshPhysicalMaterial({ color: PAINT, roughness: 0.42, metalness: 0.0, clearcoat: 0.5, clearcoatRoughness: 0.3, envMapIntensity: 0.6 });
  const graphite = new THREE.MeshStandardMaterial({ color: GRAPHITE, roughness: 0.34, metalness: 0.85, envMapIntensity: 0.9 });
  const rubber = new THREE.MeshStandardMaterial({ color: 0x151517, roughness: 0.9, metalness: 0.1 });
  const limeLit = new THREE.MeshBasicMaterial({ color: LIME });

  const add = (parent, geo, mat, { pos, quat, rotX, castShadow = true } = {}) => {
    const m = new THREE.Mesh(geo, mat);
    if (pos) m.position.copy(pos);
    if (quat) m.quaternion.copy(quat);
    if (rotX) m.rotation.x = rotX;
    m.castShadow = castShadow;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  };
  // a link body: rounded at both ends, tapering from r0 at `from` to r1 at `to`
  const linkBody = (parent, from, to, r0, r1, mat) => {
    const dir = to.clone().sub(from).normalize();
    const len = from.distanceTo(to);
    const q = new THREE.Quaternion().setFromUnitVectors(Y_AXIS, dir);
    add(parent, new THREE.CylinderGeometry(r1, r0, len, 40), mat, { pos: from.clone().addScaledVector(dir, len / 2), quat: q });
    add(parent, new THREE.SphereGeometry(r0, 32, 20), mat, { pos: from.clone() });
    add(parent, new THREE.SphereGeometry(r1, 32, 20), mat, { pos: to.clone() });
    return { dir, q };
  };
  // a joint housing: a cast graphite drum on the local z axis, chamfered rims,
  // rubber end caps. Callers size it so every link root it covers (the rounded
  // link ends sit on the joint axis) is fully inside the drum, so no sliver of
  // paint ever pokes out of a face and nothing z-fights with the caps.
  const housing = (parent, r, h, zOff = 0) => {
    const c = Math.min(8, r * 0.12);
    const drum = new THREE.LatheGeometry([
      new THREE.Vector2(0, -h / 2), new THREE.Vector2(r - c, -h / 2), new THREE.Vector2(r, -h / 2 + c),
      new THREE.Vector2(r, h / 2 - c), new THREE.Vector2(r - c, h / 2), new THREE.Vector2(0, h / 2),
    ], 48);
    add(parent, drum, graphite, { pos: new THREE.Vector3(0, 0, zOff), rotX: Math.PI / 2 });
    add(parent, new THREE.CylinderGeometry(r * 0.62, r * 0.62, h + 6, 32), rubber, { pos: new THREE.Vector3(0, 0, zOff), rotX: Math.PI / 2 });
  };

  // static base in the world frame: a rounded plate, a lip, the turret foot
  const plate = new THREE.LatheGeometry([
    new THREE.Vector2(0, 0), new THREE.Vector2(150, 0), new THREE.Vector2(158, 10), new THREE.Vector2(158, 24),
    new THREE.Vector2(146, 36), new THREE.Vector2(0, 36),
  ], 56);
  add(group, plate, graphite, { rotX: Math.PI / 2 });
  const foot = new THREE.LatheGeometry([
    new THREE.Vector2(0, 36), new THREE.Vector2(122, 36), new THREE.Vector2(118, 70), new THREE.Vector2(96, 112), new THREE.Vector2(0, 112),
  ], 56);
  add(group, foot, paint, { rotX: Math.PI / 2 });
  add(group, new THREE.TorusGeometry(92, 4, 10, 48), rubber, { pos: new THREE.Vector3(0, 0, 112) });

  const links = [];
  for (let i = 0; i < 6; i++) {
    const g = new THREE.Group();
    g.matrixAutoUpdate = false;
    group.add(g);
    const P = new THREE.Vector3(-K.DH_A[i], 0, -K.DH_D[i]).applyAxisAngle(X_AXIS, -K.DH_ALPHA[i]);
    const O = new THREE.Vector3();
    if (i === 0) {
      // turret: from the foot up to the shoulder, slightly tapered, then the
      // shoulder housing on the J2 axis, a touch longer on one side. Its
      // 170 mm length encloses the turret's top sphere (r 62) and the upper
      // arm's root sphere (r 50), which both sit on that axis.
      const top = P.clone().addScaledVector(P.clone().negate().normalize(), K.DH_D[0]);
      const dir = top.clone().sub(P).normalize();
      const q = new THREE.Quaternion().setFromUnitVectors(Y_AXIS, dir);
      const from = P.clone().addScaledVector(dir, 112);
      const len = from.distanceTo(top);
      add(g, new THREE.CylinderGeometry(62, 76, len, 40), paint, { pos: from.clone().addScaledVector(dir, len / 2), quat: q });
      add(g, new THREE.SphereGeometry(62, 32, 20), paint, { pos: top });
      housing(g, 80, 170, 12);
    } else if (i === 1) {
      // upper arm: tapers from the shoulder to the elbow, seam just outside the
      // shoulder drum (the arm leaves the drum's side 62 to 80 mm from the axis)
      const { dir, q } = linkBody(g, P, O, 50, 40, paint);
      add(g, new THREE.TorusGeometry(47.5, 2.4, 8, 40), rubber, { pos: P.clone().addScaledVector(dir, 92), quat: q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(X_AXIS, Math.PI / 2)) });
      housing(g, 56, 118);
    } else if (i === 2) {
      // elbow offset link (70 mm); the J4 drum on the next link covers its far end
      linkBody(g, P, O, 40, 40, paint);
    } else if (i === 3) {
      // forearm: the J4 motor drum behind the elbow, then a slim tube to the wrist
      const dir = O.clone().sub(P).normalize();
      const q = new THREE.Quaternion().setFromUnitVectors(Y_AXIS, dir);
      add(g, new THREE.CylinderGeometry(42, 46, 110, 40), paint, { pos: P.clone().addScaledVector(dir, 55), quat: q });
      add(g, new THREE.SphereGeometry(46, 32, 20), paint, { pos: P.clone() });
      add(g, new THREE.TorusGeometry(43, 2.4, 8, 40), rubber, { pos: P.clone().addScaledVector(dir, 110), quat: q.clone().multiply(new THREE.Quaternion().setFromAxisAngle(X_AXIS, Math.PI / 2)) });
      linkBody(g, P.clone().addScaledVector(dir, 110), O, 32, 30, paint);
      housing(g, 36, 84);
    } else if (i === 4) {
      // zero-length wrist link: the J6 housing sits on the J5 drum
      housing(g, 30, 64);
    } else {
      // flange: a short graphite stub to the tool point
      linkBody(g, P, O, 24, 22, graphite);
    }
    links.push(g);
  }

  // sensor head at the tool point: graphite puck, lime emitter ring, glow, scan cone
  const tool = links[5];
  const toolDir = new THREE.Vector3(-K.DH_A[5], 0, -K.DH_D[5]).applyAxisAngle(X_AXIS, -K.DH_ALPHA[5]).negate().normalize();
  const toolQ = new THREE.Quaternion().setFromUnitVectors(Y_AXIS, toolDir);
  add(tool, new THREE.CylinderGeometry(30, 26, 26, 36), graphite, { pos: toolDir.clone().multiplyScalar(6), quat: toolQ });
  add(tool, new THREE.TorusGeometry(19, 3, 8, 40), limeLit, { pos: toolDir.clone().multiplyScalar(20), quat: toolQ.clone().multiply(new THREE.Quaternion().setFromAxisAngle(X_AXIS, Math.PI / 2)), castShadow: false });
  add(tool, new THREE.CircleGeometry(14, 32), limeLit, { pos: toolDir.clone().multiplyScalar(20), quat: toolQ.clone().multiply(new THREE.Quaternion().setFromAxisAngle(X_AXIS, -Math.PI / 2)), castShadow: false });
  const glow = new THREE.Sprite(new THREE.SpriteMaterial({ map: radialTexture("rgba(198,244,50,0.9)", "rgba(198,244,50,0)"), transparent: true, blending: THREE.AdditiveBlending, depthWrite: false, opacity: 0.55 }));
  glow.scale.set(150, 150, 1);
  glow.position.copy(toolDir.clone().multiplyScalar(22));
  tool.add(glow);
  const cone = add(tool, new THREE.ConeGeometry(74, 190, 40, 1, true),
    new THREE.MeshBasicMaterial({ color: LIME, transparent: true, opacity: 0.07, side: THREE.DoubleSide, depthWrite: false, blending: THREE.AdditiveBlending }),
    { pos: toolDir.clone().multiplyScalar(22 + 95), quat: toolQ.clone().multiply(new THREE.Quaternion().setFromAxisAngle(X_AXIS, Math.PI)), castShadow: false });
  cone.receiveShadow = false;

  const tmp = new THREE.Matrix4();
  function update(frames) {
    for (let i = 0; i < 6; i++) {
      const T = frames[i + 1];
      tmp.set(
        T[0][0], T[0][1], T[0][2], T[0][3],
        T[1][0], T[1][1], T[1][2], T[1][3],
        T[2][0], T[2][1], T[2][2], T[2][3],
        0, 0, 0, 1
      );
      links[i].matrix.copy(tmp);
      links[i].matrixWorldNeedsUpdate = true;
    }
  }
  return { group, update, glow, cone };
}

// Where each joint label points: J1 on the turret, J2 the shoulder, J3 the
// elbow, J4 the middle of the forearm, J5 the wrist centre, J6 the flange.
export function jointAnchors(frames) {
  const pts = frames.map((T) => new THREE.Vector3(T[0][3], T[1][3], T[2][3]));
  return [new THREE.Vector3(0, 0, 200), pts[1], pts[2], pts[3].clone().lerp(pts[4], 0.5), pts[4], pts[6]];
}

// Three-point studio: warm key with a soft shadow, cool rim, low fill, a
// procedural room for reflections, a fading dark floor with a contact shadow
// and a lime calibration grid.
export function makeEnvironment(renderer, scene) {
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture;
  pmrem.dispose();
}

export function makeStudio(renderer, scene) {

  const key = new THREE.DirectionalLight(0xfff0dc, 2.6);
  key.position.set(620, -560, 980);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  key.shadow.radius = 4;
  key.shadow.bias = -0.0005;
  Object.assign(key.shadow.camera, { left: -700, right: 700, top: 700, bottom: -700, near: 200, far: 2600 });
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xbcd0ff, 1.6);
  rim.position.set(-760, 640, 420);
  scene.add(rim);
  const fill = new THREE.DirectionalLight(0xffffff, 0.45);
  fill.position.set(-260, -860, 260);
  scene.add(fill);

  const floor = new THREE.Mesh(
    new THREE.CircleGeometry(2400, 72),
    new THREE.MeshStandardMaterial({ color: 0x0d0d0f, roughness: 0.96, metalness: 0.0, transparent: true, alphaMap: radialTexture("#ffffff", "#000000", 512, 0.18), envMapIntensity: 0.08 })
  );
  floor.receiveShadow = true;
  scene.add(floor);

  const contact = new THREE.Mesh(
    new THREE.PlaneGeometry(520, 520),
    new THREE.MeshBasicMaterial({ map: radialTexture("rgba(0,0,0,0.85)", "rgba(0,0,0,0)"), transparent: true, depthWrite: false })
  );
  contact.position.z = 0.6;
  scene.add(contact);

  // calibration grid: 100 mm cells, reach rings every 200 mm, all very faint
  const pts = [];
  for (let v = -1200; v <= 1200; v += 100) pts.push(v, -1200, 1.2, v, 1200, 1.2, -1200, v, 1.2, 1200, v, 1.2);
  const gridGeo = new THREE.BufferGeometry();
  gridGeo.setAttribute("position", new THREE.Float32BufferAttribute(pts, 3));
  scene.add(new THREE.LineSegments(gridGeo, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.045 })));
  const ringPts = [];
  for (let r = 200; r <= 800; r += 200) {
    for (let k = 0; k < 96; k++) {
      const a0 = (k / 96) * Math.PI * 2, a1 = ((k + 1) / 96) * Math.PI * 2;
      ringPts.push(r * Math.cos(a0), r * Math.sin(a0), 1.4, r * Math.cos(a1), r * Math.sin(a1), 1.4);
    }
  }
  const ringGeo = new THREE.BufferGeometry();
  ringGeo.setAttribute("position", new THREE.Float32BufferAttribute(ringPts, 3));
  scene.add(new THREE.LineSegments(ringGeo, new THREE.LineBasicMaterial({ color: LIME, transparent: true, opacity: 0.11 })));
}

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
  // no synchronous link-status queries: shader compiles stay off the main thread
  renderer.debug.checkShaderErrors = false;
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.setClearColor(0x000000, 0);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;

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
      tcp.textContent = `TCP ${p.x.toFixed(0).padStart(5, " ")} ${p.y.toFixed(0).padStart(5, " ")} ${p.z.toFixed(0).padStart(5, " ")} mm`;
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
  let visible = true, hidden = document.hidden, running = false, needsRender = true;

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

  // ---- loop: idle path when untouched, paused offscreen or in a hidden tab ----
  const t0 = performance.now();
  function frame(now) {
    running = false;
    if (!visible || hidden) return;
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
