/* The live view when the browser has no WebGL.
 *
 * Measured 2026-10-09 on the rig laptop: it is an NVIDIA Optimus hybrid, and
 * Chrome can fail to create a WebGL context there ("BindToCurrentSequence
 * failed ... Optimus = yes"). three.js then throws, and the first version of
 * the mission window lost its 3D view AND -- because the exception escaped a
 * Preact effect -- its Start button. This is the same picture in Canvas2D, in
 * plan view (x right, y up, the orientation of plan_escort --plot and the
 * teleop keys), with the same interface as scene.mjs, so the window works the
 * same either way. Altitude is written next to each drone.
 */

export function createPlan2D(host, opts = {}) {
  const canvas = document.createElement('canvas');
  canvas.style.display = 'block';
  host.appendChild(canvas);
  const ctx = canvas.getContext('2d');
  const banner = document.createElement('div');
  banner.className = 'fallback-note';
  banner.textContent = opts.reason
    ? '2D plan view -- this browser could not start WebGL (' + opts.reason + ')'
    : '2D plan view';
  host.appendChild(banner);

  let arena = { cx: 0, cy: 0, rt: 2.0, rl: 2.25, ceil: 1.95 };
  let marks = [];
  const bodies = new Map();
  const markers = new Map();
  let trails = true;
  let zoom = 1;
  let pan = [0, 0];
  const TRAIL_N = 300;

  function setArena(a) {
    if (!a) return;
    arena = { cx: a.centre ? a.centre[0] : 0, cy: a.centre ? a.centre[1] : 0,
      rt: a.radius_tested || 2.0, rl: a.radius_lost || (a.radius_tested || 2.0) + 0.25,
      ceil: a.ceiling_tested || 1.95 };
  }
  function setMarks(fleet) { marks = (fleet || []).filter((d) => d.initial_position); }

  function body(name, kind) {
    let b = bodies.get(name);
    if (!b) {
      b = { name, kind, pos: [0, 0, 0], yaw: 0, seen: 0, estSeen: 0, est: null, trail: [], tone: '' };
      bodies.set(name, b);
    }
    return b;
  }
  const yawOf = (q) => (q ? Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) : 0);
  function setPose(name, p, q, kind, extra = '') {
    const b = body(name, kind);
    b.pos = [p.x, p.y, p.z]; b.yaw = yawOf(q); b.seen = performance.now(); b.extra = extra;
    const t = b.trail;
    if (!t.length || Math.hypot(t[t.length - 1][0] - p.x, t[t.length - 1][1] - p.y) > 0.01) {
      t.push([p.x, p.y]);
      if (t.length > TRAIL_N) t.shift();
    }
  }
  function setEstimate(name, p, q) {
    const b = body(name, 'drone');
    b.est = [p.x, p.y, p.z];
    b.estSeen = performance.now();
    if (performance.now() - b.seen > 500) { setPose(name, p, q, 'drone', ' est'); b.seen = 0; }
  }
  function setTone(name, tone) { const b = bodies.get(name); if (b) b.tone = tone; }

  function setMarkers(topic, arr) {
    const now = performance.now();
    for (const mk of arr.markers || []) {
      const key = `${topic}|${mk.ns}|${mk.id}`;
      if (mk.action === 3) { for (const k of [...markers.keys()]) if (k.startsWith(topic + '|')) markers.delete(k); continue; }
      if (mk.action === 2) { markers.delete(key); continue; }
      const life = mk.lifetime ? mk.lifetime.sec + mk.lifetime.nanosec * 1e-9 : 0;
      markers.set(key, { mk, expires: life > 0 ? now + life * 1000 : 0 });
    }
  }
  function clearMarkers() { markers.clear(); }

  // ---- drawing
  let W = 0; let H = 0; let scale = 100; let ox = 0; let oy = 0;
  function resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    W = host.clientWidth || 800; H = host.clientHeight || 500;
    canvas.width = W * dpr; canvas.height = H * dpr;
    canvas.style.width = W + 'px'; canvas.style.height = H + 'px';
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
  const X = (x) => ox + (x - arena.cx) * scale;
  const Y = (y) => oy - (y - arena.cy) * scale;
  const col = (c, a) => `rgba(${Math.round(c.r * 255)},${Math.round(c.g * 255)},${Math.round(c.b * 255)},${a ?? (c.a ?? 1)})`;

  function circle(x, y, r, stroke, fill, dash, w = 1.5) {
    ctx.beginPath(); ctx.arc(X(x), Y(y), r * scale, 0, Math.PI * 2);
    ctx.setLineDash(dash || []);
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = w; ctx.stroke(); }
    ctx.setLineDash([]);
  }
  function text(t, x, y, color, font = '600 11px ui-monospace,monospace', dx = 0, dy = 0) {
    ctx.font = font; ctx.fillStyle = color; ctx.fillText(t, X(x) + dx, Y(y) + dy);
  }

  function drawMarker(mk) {
    const c = mk.color || { r: 1, g: 1, b: 1, a: 1 };
    const p = (mk.pose && mk.pose.position) || { x: 0, y: 0 };
    const pts = mk.points || [];
    const pc = (i) => (mk.colors && mk.colors.length === pts.length ? mk.colors[i] : c);
    ctx.lineWidth = Math.max(1, Math.min(4, (mk.scale ? mk.scale.x : 0.01) * 120));
    switch (mk.type) {
      case 4: case 5: {
        if (pts.length < 2) return;
        const step = mk.type === 4 ? 1 : 2;
        for (let i = 0; i + 1 < pts.length; i += step) {
          ctx.strokeStyle = col(pc(i)); ctx.beginPath();
          ctx.moveTo(X(p.x + pts[i].x), Y(p.y + pts[i].y));
          ctx.lineTo(X(p.x + pts[i + 1].x), Y(p.y + pts[i + 1].y)); ctx.stroke();
        }
        return;
      }
      case 2: case 3: case 1:
        circle(p.x, p.y, Math.max(0.02, (mk.scale ? mk.scale.x : 0.1) / 2), col(c), col(c, (c.a ?? 1) * 0.35));
        return;
      case 6: case 7: case 8:
        pts.forEach((q, i) => circle(p.x + q.x, p.y + q.y, Math.max(0.015, (mk.scale ? mk.scale.x : 0.05) / 2), null, col(pc(i))));
        return;
      case 0: {
        const a = pts.length >= 2 ? pts[0] : p; const b = pts.length >= 2 ? pts[1] : { x: p.x + (mk.scale ? mk.scale.x : 0.3), y: p.y };
        ctx.strokeStyle = col(c); ctx.beginPath(); ctx.moveTo(X(a.x), Y(a.y)); ctx.lineTo(X(b.x), Y(b.y)); ctx.stroke();
        return;
      }
      case 9:
        text(mk.text || '', p.x, p.y, col(c), '600 11px system-ui', 4, -4);
        return;
      default:
    }
  }

  let running = true;
  function frame() {
    if (!running) return;
    requestAnimationFrame(frame);
    if (W !== host.clientWidth || H !== host.clientHeight) resize();
    scale = (Math.min(W, H) / 2 - 18) / (arena.rl + 0.35) * zoom;
    ox = W / 2 + pan[0]; oy = H / 2 + pan[1];
    ctx.fillStyle = '#070a0f'; ctx.fillRect(0, 0, W, H);
    // grid, 0.5 m
    ctx.strokeStyle = 'rgba(28,39,53,.9)'; ctx.lineWidth = 1;
    const g0x = Math.floor(arena.cx - arena.rl - 1); const g1x = Math.ceil(arena.cx + arena.rl + 1);
    for (let x = g0x; x <= g1x; x += 0.5) { ctx.beginPath(); ctx.moveTo(X(x), 0); ctx.lineTo(X(x), H); ctx.stroke(); }
    const g0y = Math.floor(arena.cy - arena.rl - 1); const g1y = Math.ceil(arena.cy + arena.rl + 1);
    for (let y = g0y; y <= g1y; y += 0.5) { ctx.beginPath(); ctx.moveTo(0, Y(y)); ctx.lineTo(W, Y(y)); ctx.stroke(); }
    circle(arena.cx, arena.cy, arena.rl, '#f0564f', null, [6, 4]);
    circle(arena.cx, arena.cy, arena.rt, '#d8a11d', null, null, 2);
    text(`radius_tested ${arena.rt.toFixed(2)} m`, arena.cx + arena.rt * 0.72, arena.cy - arena.rt * 0.72, '#c8a24a', '500 10px ui-monospace,monospace');
    // world axes
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#f0564f'; ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(0.5), Y(0)); ctx.stroke();
    ctx.strokeStyle = '#3fb950'; ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(0), Y(0.5)); ctx.stroke();
    text('x', 0.55, 0, '#7e8c9f', '11px system-ui', 0, 4); text('y', 0, 0.55, '#7e8c9f', '11px system-ui', -3, 0);
    for (const d of marks) {
      circle(d.initial_position[0], d.initial_position[1], 0.11, d.enabled ? '#58a6ff' : '#4a5566');
      text(d.name, d.initial_position[0], d.initial_position[1], d.enabled ? '#79b8ff' : '#7e8c9f', '500 10px ui-monospace,monospace', -10, 22);
    }
    const now = performance.now();
    for (const [k, m] of markers) {
      if (m.expires && now > m.expires) { markers.delete(k); continue; }
      drawMarker(m.mk);
    }
    for (const b of bodies.values()) {
      const stale = now - Math.max(b.seen, b.estSeen) > (opts.staleMs || 1500);
      const tone = { warn: '#d8a11d', fail: '#f0564f', idle: '#7e8c9f' }[b.tone] || (b.kind === 'drone' ? '#7ee0ff' : '#e9eff7');
      if (trails && b.trail.length > 1) {
        ctx.strokeStyle = b.kind === 'drone' ? 'rgba(126,224,255,.4)' : 'rgba(233,239,247,.35)'; ctx.lineWidth = 1.2;
        ctx.beginPath(); b.trail.forEach(([x, y], i) => (i ? ctx.lineTo(X(x), Y(y)) : ctx.moveTo(X(x), Y(y)))); ctx.stroke();
      }
      const [x, y, z] = b.pos;
      ctx.globalAlpha = stale ? 0.4 : 1;
      if (b.kind === 'drone') {
        circle(x, y, 0.09, tone, 'rgba(126,224,255,.15)', null, 2);
        ctx.strokeStyle = tone; ctx.lineWidth = 2; ctx.beginPath();
        ctx.moveTo(X(x), Y(y)); ctx.lineTo(X(x + 0.14 * Math.cos(b.yaw)), Y(y + 0.14 * Math.sin(b.yaw))); ctx.stroke();
        if (b.est && now - b.estSeen < 1500 && b.seen) circle(b.est[0], b.est[1], 0.04, '#58a6ff', null, [2, 2]);
      } else {
        ctx.fillStyle = tone; ctx.beginPath();
        ctx.moveTo(X(x), Y(y) - 8); ctx.lineTo(X(x) + 8, Y(y)); ctx.lineTo(X(x), Y(y) + 8); ctx.lineTo(X(x) - 8, Y(y)); ctx.fill();
      }
      text(`${b.name} ${z.toFixed(2)} m${b.extra || ''}`, x, y, stale ? '#7e8c9f' : '#e9eff7', '600 11px ui-monospace,monospace', 12, -10);
      ctx.globalAlpha = 1;
    }
  }

  // wheel = zoom, drag = pan
  canvas.addEventListener('wheel', (e) => { e.preventDefault(); zoom = Math.max(0.4, Math.min(6, zoom * (e.deltaY < 0 ? 1.12 : 0.89))); }, { passive: false });
  let drag = null;
  canvas.addEventListener('pointerdown', (e) => { drag = [e.clientX - pan[0], e.clientY - pan[1]]; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener('pointermove', (e) => { if (drag) pan = [e.clientX - drag[0], e.clientY - drag[1]]; });
  canvas.addEventListener('pointerup', () => { drag = null; });

  resize();
  frame();
  return {
    kind: '2d', setArena, setMarks, setPose, setEstimate, setTone, setMarkers, clearMarkers,
    setView() {}, resetView() { zoom = 1; pan = [0, 0]; },
    setTrails(on) { trails = on; }, clearTrails() { for (const b of bodies.values()) b.trail = []; },
    bodies: () => [...bodies.values()].map((b) => ({ name: b.name, kind: b.kind, pos: b.pos,
      age: performance.now() - b.seen, estAge: performance.now() - b.estSeen, est: b.est })),
    dispose() { running = false; host.innerHTML = ''; },
  };
}
