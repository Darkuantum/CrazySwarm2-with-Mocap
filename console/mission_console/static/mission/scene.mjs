/* The mission window's 3D view: the room, the fleet, and whatever the mission
 * draws (visualization_msgs/MarkerArray, the same messages RViz renders).
 *
 * World frame = the mocap/ROS world frame, z up, metres. Two cameras:
 *   3d    perspective, orbit with the mouse (left drag rotate, right drag pan,
 *         wheel zoom)
 *   plan  orthographic, looking straight down with x to the right and y up --
 *         the orientation plan_escort --plot and escort_teleop's keys use, so
 *         "w" moves a target UP the screen here as well.
 *
 * Drawn from the arena.yaml the planners read (radius_tested amber,
 * radius_lost red, ceiling_tested as the cage top) and from crazyflies.yaml's
 * parking marks. It enforces nothing: it is a picture of the same numbers.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineSegments2 } from 'three/addons/lines/LineSegments2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineSegmentsGeometry } from 'three/addons/lines/LineSegmentsGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';

const C = {
  floor: 0x0b1017, grid: 0x1c2735, grid2: 0x141c27, tested: 0xd8a11d, lost: 0xf0564f,
  cage: 0x2f3b4d, drone: 0x7ee0ff, droneOff: 0x7e8c9f, body: 0xe9eff7, est: 0x58a6ff,
  mark: 0x58a6ff, axisX: 0xf0564f, axisY: 0x3fb950, axisZ: 0x58a6ff,
};
const DRONE_SCALE = 2.2;       // a Crazyflie is 92 mm across: invisible at room scale
const TRAIL_S = 12;            // seconds of trail per body

function label(text, cls) {
  const el = document.createElement('div');
  el.className = 'lbl ' + (cls || '');
  el.textContent = text;
  return new CSS2DObject(el);
}

function circlePoints(r, z = 0, n = 96) {
  const pts = [];
  for (let i = 0; i <= n; i += 1) {
    const a = (i / n) * Math.PI * 2;
    pts.push(r * Math.cos(a), r * Math.sin(a), z);
  }
  return pts;
}

export function createScene(host, opts = {}) {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x070a0f);
  host.appendChild(renderer.domElement);
  const labels = new CSS2DRenderer();
  labels.domElement.className = 'labels';
  host.appendChild(labels.domElement);

  const scene = new THREE.Scene();
  scene.add(new THREE.HemisphereLight(0xbcd4ff, 0x0b1017, 1.4));
  const sun = new THREE.DirectionalLight(0xffffff, 1.6);
  sun.position.set(-3, -4, 8);
  scene.add(sun);

  const lineMats = new Set();
  const lineMat = (color, width, opts2 = {}) => {
    const m = new LineMaterial({ color, linewidth: width, transparent: true,
      opacity: opts2.opacity ?? 1, dashed: !!opts2.dashed, dashSize: 0.12, gapSize: 0.08,
      vertexColors: !!opts2.vertexColors, worldUnits: false });
    m.resolution.set(...viewport());
    lineMats.add(m);
    return m;
  };
  function viewport() { return [host.clientWidth || 800, host.clientHeight || 500]; }

  const persp = new THREE.PerspectiveCamera(42, 1, 0.05, 200);
  persp.up.set(0, 0, 1);
  const ortho = new THREE.OrthographicCamera(-3, 3, 3, -3, -50, 50);
  ortho.up.set(0, 1, 0);
  ortho.position.set(0, 0, 20);
  let camera = persp;
  let mode = 'iso';

  const controls = new OrbitControls(persp, labels.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  const plan = new OrbitControls(ortho, labels.domElement);
  plan.enableRotate = false;
  plan.screenSpacePanning = true;
  plan.enabled = false;

  const world = new THREE.Group();
  scene.add(world);
  const arenaG = new THREE.Group();
  const marksG = new THREE.Group();
  const bodiesG = new THREE.Group();
  const markersG = new THREE.Group();
  world.add(arenaG, marksG, bodiesG, markersG);

  let centre = new THREE.Vector3(0, 0, 0);
  let span = 2.4;

  // ---------------------------------------------------------- the room
  function setArena(a) {
    arenaG.clear();
    const cx = a && a.centre ? a.centre[0] : 0;
    const cy = a && a.centre ? a.centre[1] : 0;
    const rt = (a && a.radius_tested) || 2.0;
    const rl = (a && a.radius_lost) || rt + 0.25;
    const ceil = (a && a.ceiling_tested) || 2.0;
    centre = new THREE.Vector3(cx, cy, 0);
    span = rl + 0.4;

    const floor = new THREE.Mesh(new THREE.CircleGeometry(rl + 0.35, 96),
      new THREE.MeshBasicMaterial({ color: C.floor }));
    floor.position.set(cx, cy, -0.002);
    arenaG.add(floor);

    const grid = new THREE.GridHelper(Math.ceil(rl * 2 + 2), Math.ceil(rl * 2 + 2) * 2, C.grid, C.grid2);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(Math.round(cx), Math.round(cy), 0);
    arenaG.add(grid);

    const ring = (r, color, w, z = 0, dashed = false, opacity = 1) => {
      const g = new LineGeometry();
      g.setPositions(circlePoints(r, z));
      const l = new Line2(g, lineMat(color, w, { dashed, opacity }));
      l.computeLineDistances();
      l.position.set(cx, cy, 0);
      arenaG.add(l);
      return l;
    };
    ring(rt, C.tested, 2.2);
    ring(rl, C.lost, 1.6, 0, true, 0.85);
    ring(rt, C.cage, 1.2, ceil, true, 0.55);
    const cage = [];
    for (let i = 0; i < 12; i += 1) {
      const ang = (i / 12) * Math.PI * 2;
      cage.push(rt * Math.cos(ang), rt * Math.sin(ang), 0, rt * Math.cos(ang), rt * Math.sin(ang), ceil);
    }
    const cg = new LineSegmentsGeometry();
    cg.setPositions(cage);
    const cl = new LineSegments2(cg, lineMat(C.cage, 1, { opacity: 0.45 }));
    cl.position.set(cx, cy, 0);
    arenaG.add(cl);

    // centre cross and the world axes
    const cross = new LineSegmentsGeometry();
    cross.setPositions([cx - 0.12, cy, 0.001, cx + 0.12, cy, 0.001, cx, cy - 0.12, 0.001, cx, cy + 0.12, 0.001]);
    arenaG.add(new LineSegments2(cross, lineMat(C.tested, 1.5, { opacity: 0.8 })));
    const axes = [[0.5, 0, 0, C.axisX, 'x'], [0, 0.5, 0, C.axisY, 'y'], [0, 0, 0.5, C.axisZ, 'z']];
    for (const [x, y, z, col, name] of axes) {
      const g = new LineGeometry();
      g.setPositions([0, 0, 0.002, x, y, z + 0.002]);
      arenaG.add(new Line2(g, lineMat(col, 2.5)));
      const t = label(name, 'axis');
      t.position.set(x * 1.12, y * 1.12, z * 1.12);
      arenaG.add(t);
    }
    const tl = label(`radius_tested ${rt.toFixed(2)} m`, 'room');
    tl.position.set(cx + rt * Math.cos(-0.6), cy + rt * Math.sin(-0.6), 0.02);
    arenaG.add(tl);
    const ll = label(`lost ${rl.toFixed(2)} m`, 'room lost');
    ll.position.set(cx + rl * Math.cos(-0.75), cy + rl * Math.sin(-0.75), 0.02);
    arenaG.add(ll);
    const cel = label(`ceiling ${ceil.toFixed(2)} m`, 'room');
    cel.position.set(cx + rt * Math.cos(2.4), cy + rt * Math.sin(2.4), ceil);
    arenaG.add(cel);
    resetView();
  }

  function setMarks(fleet) {
    marksG.clear();
    for (const d of fleet || []) {
      if (!d.initial_position) continue;
      const [x, y] = d.initial_position;
      const g = new LineGeometry();
      g.setPositions(circlePoints(0.11, 0.003, 32));
      const l = new Line2(g, lineMat(d.enabled ? C.mark : C.droneOff, 1.6, { opacity: d.enabled ? 0.9 : 0.4 }));
      l.position.set(x, y, 0);
      marksG.add(l);
      const t = label(d.name, 'mark' + (d.enabled ? '' : ' off'));
      t.position.set(x, y - 0.2, 0.01);
      marksG.add(t);
    }
  }

  // --------------------------------------------------------- the fleet
  const bodies = new Map();       // name -> {...}
  const armGeo = new THREE.BoxGeometry(0.092, 0.008, 0.006);
  const hubGeo = new THREE.BoxGeometry(0.03, 0.03, 0.012);
  const propGeo = new THREE.TorusGeometry(0.022, 0.0035, 6, 20);

  function makeDrone(color) {
    const g = new THREE.Group();
    const mat = new THREE.MeshStandardMaterial({ color: C.body, roughness: 0.5, metalness: 0.2 });
    const glow = new THREE.MeshBasicMaterial({ color });
    for (const a of [Math.PI / 4, -Math.PI / 4]) {
      const arm = new THREE.Mesh(armGeo, mat);
      arm.rotation.z = a;
      g.add(arm);
    }
    g.add(new THREE.Mesh(hubGeo, mat));
    for (const [sx, sy] of [[1, 1], [1, -1], [-1, 1], [-1, -1]]) {
      const p = new THREE.Mesh(propGeo, glow);
      p.position.set(sx * 0.0325, sy * 0.0325, 0.006);
      g.add(p);
    }
    // nose: +x is the drone's forward axis (the Motive body convention)
    const nose = new THREE.Mesh(new THREE.ConeGeometry(0.008, 0.026, 8), glow);
    nose.rotation.z = -Math.PI / 2;
    nose.position.set(0.05, 0, 0);
    g.add(nose);
    g.scale.setScalar(DRONE_SCALE);
    return { g, glow };
  }

  function makeTarget(color) {
    const g = new THREE.Group();
    const m = new THREE.Mesh(new THREE.OctahedronGeometry(0.09),
      new THREE.MeshStandardMaterial({ color, roughness: 0.4, emissive: color, emissiveIntensity: 0.25 }));
    g.add(m);
    return { g, glow: m.material };
  }

  function getBody(name, kind) {
    let b = bodies.get(name);
    if (b) return b;
    const isDrone = kind === 'drone';
    const { g, glow } = isDrone ? makeDrone(C.drone) : makeTarget(0xe9eff7);
    const drop = new LineGeometry();
    drop.setPositions([0, 0, 0, 0, 0, 1]);
    const dropL = new Line2(drop, lineMat(isDrone ? C.drone : 0xbcc8d8, 1, { opacity: 0.35, dashed: true }));
    const shadow = new THREE.Mesh(new THREE.CircleGeometry(isDrone ? 0.07 : 0.09, 24),
      new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.45 }));
    const tag = label(name, isDrone ? 'drone' : 'target');
    const trailGeo = new THREE.BufferGeometry();
    const N = 600;
    trailGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(N * 3), 3));
    trailGeo.setDrawRange(0, 0);
    const trail = new THREE.Line(trailGeo, new THREE.LineBasicMaterial({
      color: isDrone ? C.drone : 0xbcc8d8, transparent: true, opacity: 0.45 }));
    trail.frustumCulled = false;
    const est = isDrone ? new THREE.Mesh(new THREE.SphereGeometry(0.035, 12, 8),
      new THREE.MeshBasicMaterial({ color: C.est, wireframe: true, transparent: true, opacity: 0.8 })) : null;
    if (est) { est.visible = false; bodiesG.add(est); }
    bodiesG.add(g, dropL, shadow, trail);
    g.add(tag);
    tag.position.set(0, 0, 0.09);
    b = { name, kind, g, glow, dropL, shadow, tag, trail, trailN: 0, N, lastT: 0, est,
          seen: 0, pos: new THREE.Vector3(), estPos: null, estSeen: 0 };
    bodies.set(name, b);
    return b;
  }

  /** a pose from /poses (truth) -- kind 'drone' for fleet names, else 'target' */
  function setPose(name, p, q, kind, extra = '') {
    const b = getBody(name, kind);
    b.pos.set(p.x, p.y, p.z);
    b.g.position.copy(b.pos);
    if (q) b.g.quaternion.set(q.x, q.y, q.z, q.w);
    b.seen = performance.now();
    b.g.visible = true;
    b.dropL.geometry.setPositions([p.x, p.y, 0.002, p.x, p.y, p.z]);
    b.dropL.computeLineDistances();
    b.shadow.position.set(p.x, p.y, 0.002);
    b.tag.element.textContent = `${name}  ${p.z.toFixed(2)} m${extra}`;
    // trail, decimated to ~25 Hz
    const now = performance.now();
    if (now - b.lastT > 40) {
      b.lastT = now;
      const arr = b.trail.geometry.attributes.position.array;
      const max = Math.min(b.N, Math.round(TRAIL_S * 25));
      if (b.trailN >= max) { arr.copyWithin(0, 3, max * 3); b.trailN = max - 1; }
      arr[b.trailN * 3] = p.x; arr[b.trailN * 3 + 1] = p.y; arr[b.trailN * 3 + 2] = p.z;
      b.trailN += 1;
      b.trail.geometry.setDrawRange(0, b.trailN);
      b.trail.geometry.attributes.position.needsUpdate = true;
      b.trail.geometry.computeBoundingSphere();
    }
  }

  /** the drone's OWN estimate (/cfX/pose): a wireframe ghost. Where mocap is
   * missing (the simulator) the estimate drives the drone model instead. */
  function setEstimate(name, p, q) {
    const b = getBody(name, 'drone');
    b.estPos = new THREE.Vector3(p.x, p.y, p.z);
    b.estSeen = performance.now();
    const truthFresh = performance.now() - b.seen < 500;
    if (!truthFresh) { setPose(name, p, q, 'drone', ' (est)'); b.seen = 0; b.g.visible = true; }
    b.est.position.copy(b.estPos);
    b.est.visible = truthFresh;
  }

  function setTone(name, tone) {
    const b = bodies.get(name);
    if (!b) return;
    const col = { ok: C.drone, warn: 0xd8a11d, fail: 0xf0564f, idle: C.droneOff }[tone] || C.drone;
    if (b.glow.color) b.glow.color.setHex(col);
    b.tag.element.dataset.tone = tone || '';
  }

  // ------------------------------------------------------- markers
  const markers = new Map();      // key -> {obj, expires}

  const colorOf = (c) => new THREE.Color(c.r, c.g, c.b);
  function disposeObj(o) {
    o.traverse((x) => {
      if (x.geometry) x.geometry.dispose();
      if (x.material && !lineMats.has(x.material)) x.material.dispose?.();
      if (x.material && lineMats.has(x.material)) { lineMats.delete(x.material); x.material.dispose(); }
      if (x.isCSS2DObject && x.element) x.element.remove();
    });
  }
  function removeMarker(key) {
    const m = markers.get(key);
    if (!m) return;
    markersG.remove(m.obj);
    disposeObj(m.obj);
    markers.delete(key);
  }

  function buildMarker(mk) {
    const col = mk.color || { r: 1, g: 1, b: 1, a: 1 };
    const opacity = col.a == null ? 1 : col.a;
    const sc = mk.scale || { x: 0.1, y: 0.1, z: 0.1 };
    const pts = mk.points || [];
    const perPoint = mk.colors && mk.colors.length === pts.length && pts.length > 0;
    const solid = () => new THREE.MeshStandardMaterial({ color: colorOf(col), transparent: opacity < 1,
      opacity, roughness: 0.5, emissive: colorOf(col), emissiveIntensity: 0.2 });
    let obj;
    switch (mk.type) {
      case 0: {                                           // ARROW
        if (pts.length >= 2) {
          const a = new THREE.Vector3(pts[0].x, pts[0].y, pts[0].z);
          const b = new THREE.Vector3(pts[1].x, pts[1].y, pts[1].z);
          const d = b.clone().sub(a); const len = d.length() || 1e-3;
          obj = new THREE.ArrowHelper(d.normalize(), a, len, colorOf(col), Math.min(len * 0.3, sc.z || 0.1), sc.y || 0.05);
        } else {
          obj = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), sc.x || 0.3,
            colorOf(col), (sc.x || 0.3) * 0.3, (sc.y || 0.05) * 1.5);
        }
        break;
      }
      case 1: obj = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1), solid()); obj.scale.set(sc.x, sc.y, sc.z); break;
      case 2: obj = new THREE.Mesh(new THREE.SphereGeometry(0.5, 20, 14), solid()); obj.scale.set(sc.x, sc.y, sc.z); break;
      case 3: {
        const g = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
        g.rotateX(Math.PI / 2);                           // ROS cylinders run along z
        obj = new THREE.Mesh(g, solid()); obj.scale.set(sc.x, sc.y, sc.z);
        break;
      }
      case 4: case 5: {                                   // LINE_STRIP / LINE_LIST
        const flat = []; const cols = [];
        pts.forEach((p, i) => {
          flat.push(p.x, p.y, p.z);
          const c = perPoint ? mk.colors[i] : col;
          cols.push(c.r, c.g, c.b);
        });
        if (flat.length < 6) return new THREE.Group();
        const w = Math.max(1, Math.min(6, (sc.x || 0.01) * 120));
        if (mk.type === 4) {
          const g = new LineGeometry(); g.setPositions(flat); g.setColors(cols);
          obj = new Line2(g, lineMat(0xffffff, w, { vertexColors: true, opacity }));
        } else {
          const g = new LineSegmentsGeometry(); g.setPositions(flat.slice(0, Math.floor(flat.length / 6) * 6));
          g.setColors(cols.slice(0, Math.floor(cols.length / 6) * 6));
          obj = new LineSegments2(g, lineMat(0xffffff, w, { vertexColors: true, opacity }));
        }
        break;
      }
      case 6: case 7: {                                   // CUBE_LIST / SPHERE_LIST
        const geo = mk.type === 6 ? new THREE.BoxGeometry(1, 1, 1) : new THREE.SphereGeometry(0.5, 14, 10);
        const inst = new THREE.InstancedMesh(geo, new THREE.MeshStandardMaterial({ transparent: opacity < 1, opacity }), Math.max(1, pts.length));
        const m4 = new THREE.Matrix4();
        pts.forEach((p, i) => {
          m4.compose(new THREE.Vector3(p.x, p.y, p.z), new THREE.Quaternion(), new THREE.Vector3(sc.x, sc.y, sc.z));
          inst.setMatrixAt(i, m4);
          const c = perPoint ? mk.colors[i] : col;
          inst.setColorAt(i, new THREE.Color(c.r, c.g, c.b));
        });
        inst.count = pts.length;
        obj = inst;
        break;
      }
      case 8: {                                           // POINTS
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.Float32BufferAttribute(pts.flatMap((p) => [p.x, p.y, p.z]), 3));
        g.setAttribute('color', new THREE.Float32BufferAttribute(pts.flatMap((p, i) => {
          const c = perPoint ? mk.colors[i] : col; return [c.r, c.g, c.b];
        }), 3));
        obj = new THREE.Points(g, new THREE.PointsMaterial({ size: Math.max(2, (sc.x || 0.03) * 200),
          sizeAttenuation: false, vertexColors: true, transparent: opacity < 1, opacity }));
        break;
      }
      case 9: {                                           // TEXT_VIEW_FACING
        obj = new THREE.Group();
        const t = label(mk.text || '', 'marker');
        t.element.style.color = `rgb(${Math.round(col.r * 255)},${Math.round(col.g * 255)},${Math.round(col.b * 255)})`;
        obj.add(t);
        break;
      }
      default:
        return null;                                      // MESH_RESOURCE, TRIANGLE_LIST: not drawn
    }
    const p = mk.pose || {};
    if (p.position) obj.position.set(p.position.x, p.position.y, p.position.z);
    if (p.orientation && (p.orientation.w || p.orientation.x || p.orientation.y || p.orientation.z)) {
      obj.quaternion.set(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    }
    return obj;
  }

  function setMarkers(topic, arr) {
    const now = performance.now();
    for (const mk of arr.markers || []) {
      const key = `${topic}|${mk.ns}|${mk.id}`;
      if (mk.action === 3) {                              // DELETEALL
        for (const k of [...markers.keys()]) if (k.startsWith(topic + '|')) removeMarker(k);
        continue;
      }
      if (mk.action === 2) { removeMarker(key); continue; }
      removeMarker(key);
      let obj;
      try { obj = buildMarker(mk); } catch (e) { obj = null; }
      if (!obj) continue;
      const life = mk.lifetime ? mk.lifetime.sec + mk.lifetime.nanosec * 1e-9 : 0;
      markers.set(key, { obj, expires: life > 0 ? now + life * 1000 : 0, ns: mk.ns });
      markersG.add(obj);
    }
  }
  function clearMarkers() { for (const k of [...markers.keys()]) removeMarker(k); }

  // --------------------------------------------------------- views
  function resetView() {
    const c = centre;
    persp.position.set(c.x - span * 1.15, c.y - span * 1.55, span * 1.25);
    controls.target.set(c.x, c.y, 0.7);
    controls.update();
    ortho.position.set(c.x, c.y, 20);
    plan.target.set(c.x, c.y, 0);
    ortho.zoom = 1;
    fitOrtho();
    plan.update();
  }
  function fitOrtho() {
    const [w, h] = viewport();
    const a = w / h;
    const s = span * 1.08;
    if (a >= 1) { ortho.left = -s * a; ortho.right = s * a; ortho.top = s; ortho.bottom = -s; }
    else { ortho.left = -s; ortho.right = s; ortho.top = s / a; ortho.bottom = -s / a; }
    ortho.updateProjectionMatrix();
  }
  function setView(v) {
    mode = v;
    camera = v === 'plan' ? ortho : persp;
    controls.enabled = v !== 'plan';
    plan.enabled = v === 'plan';
  }

  function resize() {
    const [w, h] = viewport();
    renderer.setSize(w, h, false);
    renderer.domElement.style.width = w + 'px';
    renderer.domElement.style.height = h + 'px';
    labels.setSize(w, h);
    persp.aspect = w / h;
    persp.updateProjectionMatrix();
    fitOrtho();
    lineMats.forEach((m) => m.resolution.set(w, h));
  }
  const ro = new ResizeObserver(resize);
  ro.observe(host);
  resize();

  let running = true;
  let staleMs = opts.staleMs || 1500;
  function frame() {
    if (!running) return;
    requestAnimationFrame(frame);
    const now = performance.now();
    for (const [k, m] of markers) if (m.expires && now > m.expires) removeMarker(k);
    for (const b of bodies.values()) {
      const stale = now - Math.max(b.seen, b.estSeen) > staleMs;
      b.tag.element.classList.toggle('stale', stale);
      if (b.est && now - b.estSeen > staleMs) b.est.visible = false;
    }
    if (mode === 'plan') plan.update(); else controls.update();
    renderer.render(scene, camera);
    labels.render(scene, camera);
  }
  frame();
  setArena(null);

  return {
    setArena, setMarks, setPose, setEstimate, setTone, setMarkers, clearMarkers,
    setView, resetView,
    setTrails(on) { for (const b of bodies.values()) b.trail.visible = on; },
    clearTrails() { for (const b of bodies.values()) { b.trailN = 0; b.trail.geometry.setDrawRange(0, 0); } },
    // age: of mocap truth; estAge: of the onboard estimate (/cfX/pose or /tf).
    // In the simulator only the estimate exists and it drives the model.
    bodies: () => [...bodies.values()].map((b) => ({ name: b.name, kind: b.kind, pos: b.pos.toArray(),
      age: performance.now() - b.seen, estAge: performance.now() - b.estSeen,
      est: b.estPos ? b.estPos.toArray() : null })),
    dispose() { running = false; ro.disconnect(); renderer.dispose(); host.innerHTML = ''; },
  };
}
