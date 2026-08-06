// Three.js robot-arm scene: renders the IRB120 chain from kinematics.js's fkChain(q).
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

const AMBER = 0xe07a3f;
const BODY = 0xe9e4d8;
const BLUE = 0x4c93e8;
const DARK_METAL = 0x232326;

export function createRobotScene(hostEl) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x17181a);
  scene.fog = new THREE.Fog(0x17181a, 1800, 4200);

  const camera = new THREE.PerspectiveCamera(42, 1, 1, 6000);
  camera.up.set(0, 0, 1);
  camera.position.set(950, -1150, 780);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  hostEl.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 260);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 500;
  controls.maxDistance = 3200;
  controls.update();

  // lighting
  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  const key = new THREE.DirectionalLight(0xfff2e2, 1.1);
  key.position.set(900, -600, 1400);
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x6f9fdd, 0.45);
  rim.position.set(-800, 700, 500);
  scene.add(rim);

  // floor grid in the X-Y plane (Z up)
  const grid = new THREE.GridHelper(2400, 24, 0x3a3b3d, 0x24262a);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);

  // base platform
  const baseMesh = new THREE.Mesh(
    new THREE.CylinderGeometry(120, 140, 40, 32),
    new THREE.MeshStandardMaterial({ color: DARK_METAL, metalness: 0.55, roughness: 0.4 })
  );
  baseMesh.rotation.x = Math.PI / 2;
  baseMesh.position.z = 20;
  scene.add(baseMesh);

  // ---- robot chain visuals: 7 joint spheres + 6 link cylinders ----
  const jointGeo = new THREE.SphereGeometry(26, 20, 16);
  const jointMat = new THREE.MeshStandardMaterial({ color: AMBER, metalness: 0.5, roughness: 0.35 });
  const joints = Array.from({ length: 7 }, () => {
    const m = new THREE.Mesh(jointGeo, jointMat);
    scene.add(m);
    return m;
  });

  const linkGeo = new THREE.CylinderGeometry(15, 15, 1, 16, 1, true);
  const linkMat = new THREE.MeshStandardMaterial({ color: BODY, metalness: 0.15, roughness: 0.55 });
  const links = Array.from({ length: 6 }, () => {
    const m = new THREE.Mesh(linkGeo, linkMat);
    scene.add(m);
    return m;
  });

  // end-effector gizmo (small axes)
  const eeGroup = new THREE.Group();
  const axesHelper = new THREE.AxesHelper(70);
  eeGroup.add(axesHelper);
  const eeTip = new THREE.Mesh(
    new THREE.ConeGeometry(16, 40, 16),
    new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.3, roughness: 0.4 })
  );
  eeTip.rotation.x = Math.PI / 2;
  eeTip.position.z = 20;
  eeGroup.add(eeTip);
  scene.add(eeGroup);

  // target marker (IK mode)
  const targetGroup = new THREE.Group();
  const targetSphere = new THREE.Mesh(
    new THREE.SphereGeometry(30, 16, 12),
    new THREE.MeshBasicMaterial({ color: BLUE, transparent: true, opacity: 0.28, wireframe: true })
  );
  targetGroup.add(targetSphere);
  targetGroup.add(new THREE.AxesHelper(90));
  targetGroup.visible = false;
  scene.add(targetGroup);

  // workspace envelope point cloud
  const envelopeGeo = new THREE.BufferGeometry();
  const envelopeMat = new THREE.PointsMaterial({ color: AMBER, size: 5, transparent: true, opacity: 0.35 });
  const envelopePoints = new THREE.Points(envelopeGeo, envelopeMat);
  envelopePoints.visible = false;
  scene.add(envelopePoints);

  function setEnvelopePoints(pts) {
    const arr = new Float32Array(pts.length * 3);
    pts.forEach((p, i) => { arr[i * 3] = p[0]; arr[i * 3 + 1] = p[1] || 0; arr[i * 3 + 2] = p[2]; });
    envelopeGeo.setAttribute("position", new THREE.BufferAttribute(arr, 3));
  }
  function setEnvelopeVisible(v) { envelopePoints.visible = v; }

  function mat4ToThree(T) {
    // T is row-major [row][col]; THREE.Matrix4.set takes row-major args directly.
    const m = new THREE.Matrix4();
    m.set(
      T[0][0], T[0][1], T[0][2], T[0][3],
      T[1][0], T[1][1], T[1][2], T[1][3],
      T[2][0], T[2][1], T[2][2], T[2][3],
      0, 0, 0, 1
    );
    return m;
  }

  function placeLinkBetween(mesh, p0, p1) {
    const start = new THREE.Vector3(...p0);
    const end = new THREE.Vector3(...p1);
    const dir = new THREE.Vector3().subVectors(end, start);
    const len = dir.length();
    mesh.position.copy(start).addScaledVector(dir, 0.5);
    mesh.scale.set(1, Math.max(len, 0.001), 1);
    if (len > 1e-6) {
      const up = new THREE.Vector3(0, 1, 0);
      mesh.quaternion.setFromUnitVectors(up, dir.clone().normalize());
    }
  }

  function updatePose(frames) {
    // frames: array of 7 row-major 4x4 matrices from fkChain(q)
    const pts = frames.map((T) => [T[0][3], T[1][3], T[2][3]]);
    joints.forEach((j, i) => j.position.set(...pts[i]));
    for (let i = 0; i < 6; i++) placeLinkBetween(links[i], pts[i], pts[i + 1]);
    const eeM = mat4ToThree(frames[6]);
    eeGroup.position.setFromMatrixPosition(eeM);
    eeGroup.quaternion.setFromRotationMatrix(eeM);
    return pts[6];
  }

  function setTarget(T, visible) {
    targetGroup.visible = !!visible;
    if (T) {
      const m = mat4ToThree(T);
      targetGroup.position.setFromMatrixPosition(m);
      targetGroup.quaternion.setFromRotationMatrix(m);
    }
  }

  function resize() {
    const w = hostEl.clientWidth, h = hostEl.clientHeight;
    if (!w || !h) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }
  new ResizeObserver(resize).observe(hostEl);
  resize();

  function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  return { updatePose, setTarget, setEnvelopePoints, setEnvelopeVisible };
}
