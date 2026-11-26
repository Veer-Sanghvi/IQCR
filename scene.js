// Lab viewer (section 01): the shared IRB 120 model from arm-model.js posed by
// kinematics.js's fkChain(q), with the IK target shown as a physical part to
// inspect: a machined flange hub held in front of the target tool pose on a
// drop line to the floor, a lime-ringed target frame with an xyz triad, a
// mono TARGET label, and an optional translucent ghost of the arm at the
// target pose. Same materials, lights and floor as the hero.
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { LIME, buildArm, configureRenderer, ghostArm, lowCapability, makeEnvironment, makeStudio } from "./arm-model.js";

// the part sits this far in front of the target TCP, along the tool axis, so
// the sensor cone (22 to 212 mm from the flange) covers it at convergence
const STANDOFF = 120;

export function createRobotScene(hostEl) {
  const K = window.IQCR_KIN;
  const reduceMotion = matchMedia("(prefers-reduced-motion: reduce)");
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x141415);
  scene.fog = new THREE.Fog(0x141415, 1800, 4200);

  const camera = new THREE.PerspectiveCamera(40, 1, 1, 6000);
  camera.up.set(0, 0, 1);

  // camera fly-in on load (about 1.2 s, eased); skipped entirely under reduced motion
  const CAM_FINAL = new THREE.Vector3(1020, -1236, 990);
  const CAM_START = new THREE.Vector3(2000, -2450, 1700);
  const FLY_MS = 1200;
  let flyStart = reduceMotion.matches ? null : performance.now();
  camera.position.copy(flyStart === null ? CAM_FINAL : CAM_START);
  reduceMotion.addEventListener("change", (e) => {
    if (e.matches && flyStart !== null) { flyStart = null; camera.position.copy(CAM_FINAL); }
  });

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
  const low = lowCapability(renderer);
  configureRenderer(renderer, low);
  hostEl.dataset.quality = low ? "low" : "full";
  hostEl.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 420);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 500;
  controls.maxDistance = 3200;
  controls.update();
  // the visitor grabbing the scene ends the fly-in where it is
  controls.addEventListener("start", () => { flyStart = null; });

  // lights, fading floor, contact shadow, calibration grid: identical to the hero
  makeStudio(renderer, scene);

  // ---- the arm, and its translucent ghost at the target pose ----
  const arm = buildArm(K);
  scene.add(arm.group);
  const ghost = ghostArm(arm);
  ghost.group.visible = false;
  scene.add(ghost.group);
  let ghostWanted = true, ghostPosed = false;

  // ---- target: machined flange hub, target frame, drop line, label ----
  const target = new THREE.Group();
  target.visible = false;
  hostEl.dataset.target = "none";
  scene.add(target);
  const alu = new THREE.MeshStandardMaterial({ color: 0xb9bcc1, roughness: 0.32, metalness: 0.92, envMapIntensity: 1.0 });
  const graphite = new THREE.MeshStandardMaterial({ color: 0x242427, roughness: 0.4, metalness: 0.8 });
  const rimMat = new THREE.MeshBasicMaterial({ color: LIME, transparent: true, opacity: 0.3 });
  const limeLine = new THREE.LineBasicMaterial({ color: LIME, transparent: true, opacity: 0.55 });

  // the part: a stepped hub, hub face toward the sensor, bore through, six bolt heads, lime rim
  const part = new THREE.Group();
  part.position.z = STANDOFF;
  target.add(part);
  const hub = new THREE.Mesh(new THREE.LatheGeometry([
    new THREE.Vector2(12, -12), new THREE.Vector2(52, -12), new THREE.Vector2(55, -9), new THREE.Vector2(55, -3),
    new THREE.Vector2(52, 0), new THREE.Vector2(30, 0), new THREE.Vector2(30, 24), new THREE.Vector2(27, 27),
    new THREE.Vector2(12, 27), new THREE.Vector2(12, -12),
  ], 48), alu);
  hub.rotation.x = -Math.PI / 2; // lathe axis (y) onto the tool axis, hub face toward the sensor (-z)
  hub.castShadow = true;
  hub.receiveShadow = true;
  part.add(hub);
  const boltGeo = new THREE.CylinderGeometry(4.5, 4.5, 5, 6); // hex heads
  for (let k = 0; k < 6; k++) {
    const a = (k / 6) * Math.PI * 2;
    const b = new THREE.Mesh(boltGeo, graphite);
    b.rotation.x = Math.PI / 2;
    b.position.set(42 * Math.cos(a), 42 * Math.sin(a), -2.5);
    b.castShadow = true;
    part.add(b);
  }
  const rim = new THREE.Mesh(new THREE.TorusGeometry(55.6, 1.6, 8, 64), rimMat);
  rim.position.z = 6;
  part.add(rim);

  // the target frame at the TCP: lime ring in the tool plane plus an xyz triad
  const triad = new THREE.Group();
  target.add(triad);
  triad.add(new THREE.Mesh(new THREE.TorusGeometry(30, 1.2, 8, 48), rimMat));
  const axisGeo = new THREE.CylinderGeometry(1.5, 1.5, 70, 10);
  [[0xff6b6b, [1, 0, 0]], [0x6bff9b, [0, 1, 0]], [0x6bb6ff, [0, 0, 1]]].forEach(([color, d]) => {
    const m = new THREE.Mesh(axisGeo, new THREE.MeshBasicMaterial({ color }));
    const dir = new THREE.Vector3(...d);
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    m.position.copy(dir).multiplyScalar(35);
    triad.add(m);
  });

  // drop line from the part straight down to the floor, with a floor ring
  const dropGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
  const drop = new THREE.Line(dropGeo, limeLine);
  drop.visible = false;
  scene.add(drop);
  const floorRing = new THREE.Mesh(new THREE.RingGeometry(16, 19, 40), new THREE.MeshBasicMaterial({ color: LIME, transparent: true, opacity: 0.5, side: THREE.DoubleSide }));
  floorRing.visible = false;
  floorRing.position.z = 1.6;
  scene.add(floorRing);

  // mono label in the DOM, projected to the part every frame
  const label = document.createElement("div");
  label.className = "target-tag";
  label.dataset.targetLabel = "";
  label.hidden = true;
  label.innerHTML = `<span>TARGET</span><b></b><small>part at ${STANDOFF} mm standoff</small>`;
  hostEl.appendChild(label);
  const leader = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  leader.setAttribute("class", "target-leader");
  leader.innerHTML = '<line x1="0" y1="0" x2="0" y2="0"></line><circle r="3"></circle>';
  leader.setAttribute("hidden", "");
  hostEl.appendChild(leader);
  const leaderLine = leader.firstElementChild, leaderDot = leader.lastElementChild;
  const tcpWorld = new THREE.Vector3();
  const partWorld = new THREE.Vector3();
  let flash = 0; // 1 right after convergence, decays to 0; rim brightness follows it

  // workspace envelope point cloud
  const envelopeGeo = new THREE.BufferGeometry();
  const envelopePoints = new THREE.Points(envelopeGeo, new THREE.PointsMaterial({ color: LIME, size: 5, transparent: true, opacity: 0.35 }));
  envelopePoints.visible = false;
  scene.add(envelopePoints);

  // render on demand: anything that changes what is on screen marks the scene
  // dirty and wakes the loop; the loop stops again once nothing is animating
  let dirty = true;
  let visible = false, hidden = document.hidden, running = false, last = performance.now(), envQueued = false, active = false;
  const invalidate = () => { dirty = true; schedule(); };
  function setEnvelopePoints(pts) {
    invalidate();
    const arr = new Float32Array(pts.length * 3);
    pts.forEach((p, i) => { arr[i * 3] = p[0]; arr[i * 3 + 1] = p[1] || 0; arr[i * 3 + 2] = p[2]; });
    envelopeGeo.setAttribute("position", new THREE.BufferAttribute(arr, 3));
  }
  function setEnvelopeVisible(v) { envelopePoints.visible = v; invalidate(); }

  const tmpM = new THREE.Matrix4();
  function mat4ToThree(T) {
    // T is row-major [row][col]; THREE.Matrix4.set takes row-major args directly.
    return tmpM.set(
      T[0][0], T[0][1], T[0][2], T[0][3],
      T[1][0], T[1][1], T[1][2], T[1][3],
      T[2][0], T[2][1], T[2][2], T[2][3],
      0, 0, 0, 1
    );
  }

  function updatePose(frames) {
    invalidate();
    // frames: array of 7 row-major 4x4 matrices from fkChain(q)
    arm.update(frames);
    const T = frames[6];
    return [T[0][3], T[1][3], T[2][3]];
  }

  // T: target tool pose (null hides everything); q: the joint vector behind it, for the ghost
  function setTarget(T, q) {
    invalidate();
    target.visible = !!T;
    hostEl.dataset.target = T ? "set" : "none";
    label.hidden = !T;
    leader.toggleAttribute("hidden", !T);
    if (!T) { ghost.group.visible = false; drop.visible = floorRing.visible = false; desiredDist = baseDist; return; }
    const m = mat4ToThree(T);
    target.position.setFromMatrixPosition(m);
    target.quaternion.setFromRotationMatrix(m);
    target.updateMatrixWorld(true);
    part.getWorldPosition(partWorld);
    const pos = dropGeo.attributes.position;
    pos.setXYZ(0, partWorld.x, partWorld.y, partWorld.z);
    pos.setXYZ(1, partWorld.x, partWorld.y, 1.6);
    pos.needsUpdate = true;
    floorRing.position.set(partWorld.x, partWorld.y, 1.6);
    drop.visible = floorRing.visible = partWorld.z > 40;
    tcpWorld.copy(target.position);
    // the label states the IK target (where the TCP goes); the part is STANDOFF mm further along the tool axis
    label.children[1].textContent = `${tcpWorld.x.toFixed(0)} ${tcpWorld.y.toFixed(0)} ${tcpWorld.z.toFixed(0)} mm`;
    fitTarget();
    ghostPosed = Array.isArray(q);
    if (ghostPosed) ghost.update(K.fkChain(q).frames);
    ghost.group.visible = ghostWanted && ghostPosed;
    flash = 0;
  }
  function setGhostVisible(v) { ghostWanted = !!v; ghost.group.visible = ghostWanted && ghostPosed && target.visible; invalidate(); }
  // lime rim lights up on convergence and dims back over about a second; a
  // reduced-motion visitor gets no pulse at all
  function flashTarget() { ghost.group.visible = false; if (!reduceMotion.matches) flash = 1; invalidate(); }

  // the label sits in the margin on the part's side of the canvas (away from
  // the arm, whose base projects near the centre) with a leader to the part
  const screen = new THREE.Vector3();
  function placeLabel() {
    if (label.hidden) return;
    screen.copy(partWorld).project(camera);
    const W = hostEl.clientWidth, H = hostEl.clientHeight;
    const px = (screen.x * 0.5 + 0.5) * W, py = (-screen.y * 0.5 + 0.5) * H;
    const w = label.offsetWidth, h = label.offsetHeight, right = px >= W / 2;
    const x = right ? W - w - 10 : 10;
    // narrow canvases: the top corner, which the arm never reaches; wide ones:
    // just above the part (below it when the part is at the top), never over it
    const y = W < 600 ? 8 : py > h + 48 ? Math.min(H - h - 8, py - h - 28) : Math.min(H - h - 8, py + 36);
    label.style.transform = `translate(${x.toFixed(1)}px,${y.toFixed(1)}px)`;
    leaderLine.setAttribute("x1", right ? x - 4 : x + w + 4); leaderLine.setAttribute("y1", y + h / 2);
    leaderLine.setAttribute("x2", px); leaderLine.setAttribute("y2", py);
    leaderDot.setAttribute("cx", px); leaderDot.setAttribute("cy", py);
  }

  // camera distance: the base framing, pulled back just enough that the part
  // stays in view, until the visitor orbits or zooms themselves
  const baseDist = CAM_FINAL.distanceTo(controls.target);
  let desiredDist = baseDist, userTouched = false;
  controls.addEventListener("start", () => { userTouched = true; });
  const dirTmp = new THREE.Vector3(), posTmp = new THREE.Vector3(), ndc = new THREE.Vector3();
  function fitTarget() {
    if (userTouched) return;
    const saved = camera.position.clone();
    dirTmp.copy(CAM_FINAL).sub(controls.target).normalize();
    desiredDist = baseDist * 2.4;
    for (let d = baseDist; d <= baseDist * 2.4; d *= 1.06) {
      camera.position.copy(controls.target).addScaledVector(dirTmp, d);
      camera.lookAt(controls.target);
      camera.updateMatrixWorld();
      ndc.copy(partWorld).project(camera);
      if (Math.abs(ndc.x) < 0.8 && Math.abs(ndc.y) < 0.78) { desiredDist = d; break; }
    }
    camera.position.copy(saved);
    camera.lookAt(controls.target);
  }

  function resize() {
    const w = hostEl.clientWidth, h = hostEl.clientHeight;
    if (!w || !h) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
    const narrow = w < 600;
    ghost.material.opacity = narrow ? 0.06 : 0.12;
    arm.glow.material.opacity = narrow ? 0.3 : 0.55;
    invalidate();
  }
  new ResizeObserver(resize).observe(hostEl);
  resize();

  // ---- loop: renders only when something changed (a pose, the target, a
  // slider, an orbit) or is animating (fly-in, camera easing, the rim flash,
  // orbit damping), and only while the viewer is on screen in a visible tab.
  // Between changes no frame is drawn at all. The first render (shader
  // compiles, shadow maps) and the reflection environment wait for the viewer
  // to scroll into view, so they never block the page load. The hero listens
  // for "iqcr:viewer-active" and holds its idle sweep while this loop runs.
  function setActive(v) {
    if (active === v) return;
    active = v;
    document.dispatchEvent(new CustomEvent("iqcr:viewer-active", { detail: v }));
  }
  function frame(now) {
    running = false;
    if (!visible || hidden) { setActive(false); return; }
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    let animating = false;
    if (flyStart !== null) {
      const t = Math.min(1, (now - flyStart) / FLY_MS);
      const eased = 1 - Math.pow(1 - t, 3);
      camera.position.lerpVectors(CAM_START, CAM_FINAL, eased);
      if (t >= 1) flyStart = null; else animating = true;
    }
    if (flyStart === null && !userTouched) {
      posTmp.copy(camera.position).sub(controls.target);
      const dist = posTmp.length();
      if (Math.abs(dist - desiredDist) > 0.5) {
        camera.position.copy(controls.target).addScaledVector(posTmp.normalize(), reduceMotion.matches ? desiredDist : dist + (desiredDist - dist) * 0.08);
        animating = true;
      }
    }
    if (flash > 0) { flash = Math.max(0, flash - dt / 1.2); animating = true; }
    rimMat.opacity = 0.3 + 0.7 * flash;
    // OrbitControls.update() reports a camera move, which with damping keeps
    // going for a few frames after the pointer is released
    if (controls.update()) animating = true;
    if (dirty || animating) {
      dirty = false;
      placeLabel();
      renderer.render(scene, camera);
    }
    if (!envQueued) {
      envQueued = true;
      const addEnv = () => { makeEnvironment(renderer, scene); invalidate(); };
      if ("requestIdleCallback" in window) requestIdleCallback(addEnv, { timeout: 2000 }); else setTimeout(addEnv, 250);
    }
    if (animating || dirty) { setActive(true); schedule(); } else setActive(false);
  }
  function schedule() {
    if (running || !visible || hidden) return;
    running = true;
    last = performance.now();
    requestAnimationFrame(frame);
  }
  new IntersectionObserver(([e]) => { visible = e.isIntersecting; schedule(); }, { threshold: 0.02 }).observe(hostEl);
  document.addEventListener("visibilitychange", () => { hidden = document.hidden; schedule(); });
  controls.addEventListener("change", invalidate);
  controls.addEventListener("start", invalidate);

  return { updatePose, setTarget, setGhostVisible, flashTarget, setEnvelopePoints, setEnvelopeVisible };
}
