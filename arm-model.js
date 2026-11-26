// Shared ABB IRB 120 model: the painted links, graphite joint housings, lime
// sensor head, studio lights, floor, contact shadow and calibration grid that
// both the hero (hero.js) and the lab viewer (scene.js) render. Geometry is
// pose independent; buildArm().update(frames) moves the six link groups with
// the frames kinematics.js's fkChain(q) returns. One source of truth.
import * as THREE from "three";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

export const LIME = 0xC6F432;
const PAINT = 0xEDECE8;
const GRAPHITE = 0x242427;
const X_AXIS = new THREE.Vector3(1, 0, 0);
const Y_AXIS = new THREE.Vector3(0, 1, 0);
const Z_AXIS = new THREE.Vector3(0, 0, 1);

export function radialTexture(inner, outer, size = 256, hold = 0) {
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
  // six hex bolts hold the plate down
  const hexBolt = new THREE.CylinderGeometry(7, 7, 8, 6);
  for (let k = 0; k < 6; k++) {
    const a = (k / 6) * Math.PI * 2 + Math.PI / 6;
    add(group, hexBolt, graphite, { pos: new THREE.Vector3(134 * Math.cos(a), 134 * Math.sin(a), 40), rotX: Math.PI / 2 });
  }

  const links = [];
  for (let i = 0; i < 6; i++) {
    const g = new THREE.Group();
    g.matrixAutoUpdate = false;
    g.userData.link = i; // lets a clone of the group find its link frames
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
      // cable conduit from the shoulder drum to the elbow drum, routed along
      // the outside of the upper arm (both ends disappear into the housings)
      const side = new THREE.Vector3(0, 0, 1);
      const conduit = new THREE.CatmullRomCurve3([
        P.clone().addScaledVector(side, 70),
        P.clone().lerp(O, 0.3).addScaledVector(side, 64),
        P.clone().lerp(O, 0.7).addScaledVector(side, 60),
        O.clone().addScaledVector(side, 48),
      ]);
      add(g, new THREE.TubeGeometry(conduit, 24, 7, 12, false), rubber);
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
  glow.userData.fx = true;
  tool.add(glow);
  const cone = add(tool, new THREE.ConeGeometry(74, 190, 40, 1, true),
    new THREE.MeshBasicMaterial({ color: LIME, transparent: true, opacity: 0.07, side: THREE.DoubleSide, depthWrite: false, blending: THREE.AdditiveBlending }),
    { pos: toolDir.clone().multiplyScalar(22 + 95), quat: toolQ.clone().multiply(new THREE.Quaternion().setFromAxisAngle(X_AXIS, Math.PI)), castShadow: false });
  cone.receiveShadow = false;
  cone.userData.fx = true;

  const update = (frames) => poseLinks(links, frames);
  return { group, links, update, glow, cone, toolDir };
}

// Set the six link groups' matrices from fkChain(q).frames (row-major 4x4).
const tmpM = new THREE.Matrix4();
export function poseLinks(links, frames) {
  for (let i = 0; i < 6; i++) {
    const T = frames[i + 1];
    tmpM.set(
      T[0][0], T[0][1], T[0][2], T[0][3],
      T[1][0], T[1][1], T[1][2], T[1][3],
      T[2][0], T[2][1], T[2][2], T[2][3],
      0, 0, 0, 1
    );
    links[i].matrix.copy(tmpM);
    links[i].matrixWorldNeedsUpdate = true;
  }
}

// A translucent copy of an arm for showing a target pose: shares every
// geometry with the original, swaps all materials for one tinted material and
// drops the sensor glow and scan cone.
export function ghostArm(arm, color = LIME, opacity = 0.12) {
  const group = arm.group.clone();
  const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false });
  const drop = [];
  group.traverse((o) => { if (o.userData.fx) drop.push(o); else if (o.isMesh) { o.material = mat; o.castShadow = false; o.receiveShadow = false; } });
  drop.forEach((o) => o.parent.remove(o));
  const links = [];
  group.traverse((o) => { if (o.userData.link !== undefined) links[o.userData.link] = o; });
  return { group, links, material: mat, update: (frames) => poseLinks(links, frames) };
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
