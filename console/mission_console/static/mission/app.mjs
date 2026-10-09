/* Mission window: one mission, its own layout, everything it needs on screen.
 *
 * The console is the generic tool -- every action, every check. This window
 * is the opposite: one mission spec (missions/<name>.yaml in the show's
 * package, see docs/MISSIONS.md) decides what is on screen, so a new demo
 * gets its own controls without anyone editing the console.
 *
 * Where each thing comes from -- nothing here is a second implementation:
 *   live 3D        foxglove_bridge (ros.mjs), read-only
 *   processes      the console backend: the same /api/run, /api/input,
 *                  /api/stop the console uses, so every command lands in
 *                  the Command log and the usage log
 *   gate / status  the script's OWN output, matched with the regexes in the
 *                  spec -- the show is not modified to feed this window
 *   fleet health   the console's health sweep (SSE), the same tiles' data
 *
 * Contract kept from the console: anything that moves a drone shows its exact
 * command line in a confirm step first; E-STOP is one click with an answer;
 * landing a show goes through the show's own abort (one SIGINT, no SIGTERM
 * chasing it -- see procs.interrupt).
 */
import { h, html, render, Component, useState, useEffect, useRef, useMemo, useCallback }
  from '/static/vendor/preact/htm-preact-standalone.mjs';
import { Bridge } from './ros.mjs';
import { createScene } from './scene.mjs';
import { createPlan2D } from './plan2d.mjs';

/* ================================================================ store */
const Q = new URLSearchParams(location.search);
const MID = Q.get('m') || '';
const BRIDGE_URL = Q.get('bridge') || `ws://${location.hostname || 'localhost'}:8765`;

const S = {
  mission: null, error: '', missions: [], unviewed: [], build: null, arena: null,
  settings: {}, fleet: [], adoptedId: null,
  health: null, procs: new Map(), lines: new Map(), sse: 'connecting',
  derived: new Map(),            // procId -> derived state of a MAIN run
  heldBy: null,                  // which teleop owns the keyboard
  estop: null, posesHz: null, bridgeRate: 0,
};
const subs = new Set();
let pending = false;
function changed() {
  if (pending) return;
  pending = true;
  requestAnimationFrame(() => { pending = false; subs.forEach((f) => f()); });
}
function useStore() {
  const [, set] = useState(0);
  useEffect(() => { const f = () => set((x) => x + 1); subs.add(f); return () => subs.delete(f); }, []);
  return S;
}

async function api(path, opts) {
  const r = await fetch(path, opts);
  const t = await r.text();
  let j; try { j = JSON.parse(t); } catch { j = { error: t }; }
  if (!r.ok || (j && j.error)) throw new Error((j && j.error) || r.statusText);
  return j;
}
const post = (path, body) => api(path, { method: 'POST', headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body || {}) });
const usage = (ev, extra) => post('/api/usage', { ev, ...extra }).catch(() => {});

const facts = () => (S.health && S.health.facts) || {};
const prefix = () => `mission:${MID}:`;
const roleOf = (p) => (p.action_id.startsWith(prefix()) ? p.action_id.slice(prefix().length) : null);
const alive = (p) => p && (p.state === 'running' || p.state === 'stopping');

/** the executable this window's main role runs */
function runTarget() {
  const r = S.mission && S.mission.run;
  if (!r) return null;
  if (r.exe) return { pkg: r.pkg, exe: r.exe };
  const v = (S.launchValues && S.launchValues.script) || '';
  const [pkg, exe] = v.split(' ');
  return exe ? { pkg, exe } : null;
}

/** The same show running but NOT started by this window -- from the
 * dashboard's Fly (a console process: adopted, with its output and its stdin),
 * or outside the console altogether (census: seen, stoppable, no output).
 * One source of truth: the window shows what is running, whoever started it. */
function foreignRun() {
  const t = runTarget();
  if (!t) return {};
  const flights = ((facts().system || {}).flights || []).filter((x) => x.exe === t.exe && x.pkg === t.pkg);
  const viaConsole = flights.find((x) => x.console_proc && S.procs.get(x.console_proc)
    && !roleOf(S.procs.get(x.console_proc)));
  return { consoleProc: viaConsole ? S.procs.get(viaConsole.console_proc) : null,
           outside: flights.find((x) => x.by === 'outside') || null };
}

/** newest process for a role of THIS mission (for 'main': or the adopted run) */
function procFor(role) {
  let best = null;
  for (const p of S.procs.values()) {
    if (roleOf(p) === role && (!best || p.started > best.started)) best = p;
  }
  if (role === 'main' && !alive(best)) {
    const f = foreignRun().consoleProc;
    if (f && alive(f)) {
      if (S.adoptedId !== f.id) { S.adoptedId = f.id; S.derived.delete(f.id); loadLines(f.id); }
      return f;
    }
  }
  return best;
}
const isMain = (p) => p && (roleOf(p) === 'main' || p.id === S.adoptedId);

/* ---------------------------------------------- derived state of a run */
function tpl(s, m) {
  return String(s).replace(/\$(\w+)/g, (_, k) => {
    const v = /^\d+$/.test(k) ? m[+k] : (m.groups || {})[k];
    return v == null ? '' : v.trim();
  });
}
function freshDerived(spec) {
  const ind = new Map();
  for (const r of spec.indicators) if (!ind.has(r.label)) ind.set(r.label, { value: r.initial, tone: '', ts: 0 });
  return { indicators: ind, gate: null, collecting: false, events: [], enabled: new Set(), answered: 0, lastPrompt: '' };
}
const RE = new Map();
// the backend ships patterns in JavaScript's dialect; a leading (?i) -- the
// one inline flag it allows -- becomes the RegExp flag
const re = (src) => {
  if (!RE.has(src)) RE.set(src, src.startsWith('(?i)') ? new RegExp(src.slice(4), 'i') : new RegExp(src));
  return RE.get(src);
};

function ingest(procId, text, ts) {
  const spec = S.mission;
  const p = S.procs.get(procId);
  if (!spec || !isMain(p)) return;
  if (!S.derived.has(procId)) S.derived.set(procId, freshDerived(spec));
  const d = S.derived.get(procId);
  for (const r of spec.indicators) {
    const m = re(r.match).exec(text);
    if (m) {
      const value = tpl(r.value, m);
      d.indicators.set(r.label, { value, tone: r.tone[value] || '', ts });
    }
  }
  const g = spec.gate;
  if (g) {
    const m = re(g.prompt).exec(text);
    if (m) {
      d.gate = { text: ((m.groups && m.groups.text) || m[1] || text).trim(), details: [], ts };
      d.collecting = true;
      d.lastPrompt = d.gate.text;
    } else if (d.collecting) {
      if (g.until && re(g.until).test(text)) d.collecting = false;
      else if (text.trim() && d.gate && d.gate.details.length < 6) d.gate.details.push(text.trim());
    }
  }
  if (spec.events.some((e) => re(e).test(text))) {
    d.events.push({ ts, text: text.trim() });
    if (d.events.length > 400) d.events.splice(0, 100);
  }
  for (const hlp of Object.values(spec.helpers)) {
    if (hlp.enable_after && re(hlp.enable_after).test(text)) d.enabled.add(hlp.key);
  }
}

function pushLine(procId, seq, text, ts) {
  if (!S.lines.has(procId)) S.lines.set(procId, []);
  const arr = S.lines.get(procId);
  if (arr.length && arr[arr.length - 1].seq >= seq) return;
  arr.push({ seq, text, ts });
  if (arr.length > 3000) arr.splice(0, 600);
  ingest(procId, text, ts);
}

async function loadLines(procId) {
  try {
    const r = await api('/api/proc/' + procId);
    S.lines.set(procId, []);
    S.derived.delete(procId);
    S.procs.set(procId, r.proc);
    r.lines.forEach((l) => pushLine(procId, l.seq, l.text, l.ts * 1000));
    changed();
  } catch (e) { /* pruned */ }
}

async function refreshBuild() {
  try {
    if (MID) S.build = (await api('/api/mission?id=' + encodeURIComponent(MID))).build;
    else { const ml = await api('/api/missions'); S.missions = ml.missions; S.unviewed = ml.unviewed || []; }
    changed();
  } catch (e) { /* next event */ }
}

/* Build a package with the console's own catalog action (./scripts/build.sh
 * <pkg>), so it lands in the Command log like any other command. */
async function buildPkg(pkg) {
  const ok = await confirmBox(`Build ${pkg}`, html`<p>This runs the workspace build for one package:</p>
    <div class="cmdline">./scripts/build.sh ${pkg}</div>
    <p class="hint">A brand-new package also needs the console restarted afterwards (./console/run.sh) --
    its install is not on the running console's environment yet. This window says so when that is the case.</p>`,
  'Build', 'primary');
  if (!ok) return;
  try { await post('/api/run', { action_id: 'build.workspace', values: { package: pkg }, source: 'mission' }); }
  catch (e) { S.estop = { kind: 'fail', title: 'Build could not start', detail: e.message }; changed(); }
}

function upsertProc(p) {
  const prev = S.procs.get(p.id);
  S.procs.set(p.id, p);
  if (p.action_id === 'build.workspace' && prev && alive(prev) && !alive(p)) refreshBuild();
  const role = roleOf(p);
  if (role && !prev) loadLines(p.id);
  // the main script ended: the helpers it needed have nothing left to do
  if (role === 'main' && prev && alive(prev) && !alive(p)) {
    for (const q of S.procs.values()) {
      if (roleOf(q) && roleOf(q) !== 'main' && alive(q)) post('/api/stop', { proc: q.id, source: 'mission' }).catch(() => {});
    }
    releaseKeys();
  }
}

/* ---------------------------------------------------------------- boot */
async function boot() {
  try {
    const b = await api('/api/bootstrap');
    S.health = b.health;
    S.missions = b.missions || [];
    S.settings = b.settings || {};
    S.fleet = (b.journal && b.journal.events) || [];
    if (!MID) {
      const ml = await api('/api/missions');
      S.missions = ml.missions; S.unviewed = ml.unviewed || [];
    }
    b.procs.forEach((p) => S.procs.set(p.id, p));
  } catch (e) { S.error = 'console backend unreachable: ' + e.message; }
  try { S.arena = await api('/api/arena'); } catch (e) { S.arena = { error: e.message }; }
  if (MID) {
    try {
      const r = await api('/api/mission?id=' + encodeURIComponent(MID));
      S.mission = r.mission;
      S.build = r.build;
      document.title = `${S.mission.title} - mission`;
      for (const p of r.procs) { S.procs.set(p.id, p); loadLines(p.id); }
      usage('mission_open', { mission: MID });
    } catch (e) { S.error = e.message; }
  }
  changed();
  const src = new EventSource('/api/events');
  src.onopen = () => { S.sse = 'open'; changed(); };
  src.onerror = () => { S.sse = 'down'; changed(); };
  src.onmessage = (e) => {
    const ev = JSON.parse(e.data);
    if (ev.type === 'health') { S.health = ev.health; changed(); }
    else if (ev.type === 'proc') { upsertProc(ev.proc); changed(); }
    else if (ev.type === 'line') {
      const p = S.procs.get(ev.proc);
      if (p && (roleOf(p) || p.id === S.adoptedId)) { pushLine(ev.proc, ev.seq, ev.text, Date.now()); changed(); }
      else if (!p) {                    // the "$ cmd" line arrives before the proc event
        pushLine(ev.proc, ev.seq, ev.text, Date.now());
      }
    } else if (ev.type === 'partial') {
      const p = S.procs.get(ev.proc);
      if (p) { p.partial = ev.text; changed(); }
    } else if (ev.type === 'settings') {
      S.settings = ev.settings || {}; changed();
    } else if (ev.type === 'fleet') {
      onFleet(ev.event); changed();
    }
  };
}

/* A command the server acted on, from ANYONE (journal.py): it goes on the
 * timeline, and an e-stop from anywhere raises the banner here too. */
function onFleet(ev) {
  S.fleet.push(ev);
  if (S.fleet.length > 300) S.fleet.splice(0, 100);
  if (ev.verb !== 'emergency' || ev.backlog) return;
  const t = new Date(ev.ts * 1000).toLocaleTimeString([], { hour12: false });
  if (ev.sim_noop) {
    S.estop = { kind: 'fail', title: `E-STOP at ${t} did NOTHING -- the simulator does not implement it`,
      detail: 'The simulated drones keep flying. Abort & land the script, or Land all.' };
  } else if (!(S.ownEstopAt && Date.now() - S.ownEstopAt < 6000)) {
    S.estop = { kind: 'ok', title: `E-STOP fired outside this window at ${t}`,
      detail: `crazyflie_server: "${ev.text}" -- from the console, the preflight GUI, a terminal or a script.` };
  }
}

/* ------------------------------------------------------------- actions */
async function startRole(role, values, sim) {
  return post('/api/mission/start', { id: MID, role, values, sim });
}
async function sendLine(procId, text, source = 'mission') {
  await post('/api/input', { proc: procId, text: text + '\n', source });
  const p = S.procs.get(procId);
  const d = S.derived.get(procId);
  if (d && d.gate) { d.gate = null; d.collecting = false; d.answered += 1; }
  if (p) p.partial = '';
  changed();
}
async function fireEstop() {
  S.ownEstopAt = Date.now();
  S.estop = { kind: 'pending', title: 'E-STOP sent', detail: 'waiting for the crazyflie_server...' };
  changed();
  try {
    const r = await post('/api/estop', { source: 'mission' });
    S.estop = r.confirmed
      ? { kind: 'ok', title: `E-STOP confirmed in ${r.elapsed.toFixed(2)} s`, detail: 'Motors cut. Power-cycle before the next arm.' }
      : { kind: 'fail', title: r.pending ? 'E-STOP NOT CONFIRMED' : 'E-STOP FAILED', detail: r.reason };
  } catch (e) {
    S.estop = { kind: 'fail', title: 'E-STOP could not reach the console', detail: e.message + ' -- use the preflight GUI (e) or cut power.' };
  }
  changed();
}

/* ---------------------------------------------------------- confirm UI */
let confirmState = null;
function confirmBox(title, body, ok = 'Run it', tone = 'danger') {
  return new Promise((resolve) => { confirmState = { title, body, ok, tone, resolve }; changed(); });
}
function Confirm() {
  useStore();
  if (!confirmState) return null;
  const c = confirmState;
  const done = (v) => { confirmState = null; changed(); c.resolve(v); };
  return html`<div class="modal" onClick=${(e) => e.target.classList.contains('modal') && done(false)}>
    <div class="modalbox">
      <div class="modalhead">${c.title}</div>
      <div class="modalbody">${c.body}</div>
      <div class="modalfoot">
        <button class="btn ghost" onClick=${() => done(false)}>Cancel</button>
        <button class="btn ${c.tone}" autofocus onClick=${() => done(true)}>${c.ok}</button>
      </div></div></div>`;
}

/* ================================================================ widgets */
function Panel({ title, right, children, cls = '' }) {
  return html`<section class="panel ${cls}">
    ${title && html`<header class="ph"><span class="pt">${title}</span><span class="pr">${right}</span></header>`}
    <div class="pb">${children}</div></section>`;
}

/* ------------------------------------------------------------- scene */
let BRIDGE = null;
function bridge() {
  if (!BRIDGE) { BRIDGE = new Bridge(BRIDGE_URL); BRIDGE.onChange(changed); BRIDGE.connect(); }
  return BRIDGE;
}

function Scene() {
  const s = useStore();
  const host = useRef(null);
  const sc = useRef(null);
  const [view, setView] = useState(() => { try { return localStorage.getItem('mw_view') || 'iso'; } catch (e) { return 'iso'; } });
  const [trails, setTrails] = useState(true);
  const b = bridge();

  useEffect(() => {
    try {
      sc.current = createScene(host.current);
    } catch (e) {
      // no WebGL (seen on this Optimus laptop): the same view in 2D, same API
      host.current.innerHTML = '';
      sc.current = createPlan2D(host.current, { reason: String(e.message || e).slice(0, 80) });
    }
    window.__scene = sc.current;          // for a custom view, and for debugging
    return () => sc.current.dispose();
  }, []);
  useEffect(() => {
    if (!sc.current || !s.arena) return;
    sc.current.setArena(s.arena.arena);
    sc.current.setMarks(s.arena.fleet);
  }, [s.arena]);
  useEffect(() => { sc.current && sc.current.setView(view); try { localStorage.setItem('mw_view', view); } catch (e) { /* */ } }, [view]);
  useEffect(() => { sc.current && sc.current.setTrails(trails); }, [trails]);

  // subscriptions: mocap truth, each enabled drone's own estimate, the
  // mission's marker topics. Re-made when the fleet or the spec changes.
  const fleetNames = useMemo(() => ((s.arena && s.arena.fleet) || []).map((d) => d.name), [s.arena]);
  const enabled = useMemo(() => ((s.arena && s.arena.fleet) || []).filter((d) => d.enabled).map((d) => d.name), [s.arena]);
  const markerTopics = (s.mission && s.mission.topics.markers) || [];
  useEffect(() => {
    const offs = [];
    let n = 0; let t0 = performance.now();
    offs.push(b.subscribe('/poses', (msg) => {
      n += 1;
      const now = performance.now();
      if (now - t0 > 1000) { S.posesHz = n * 1000 / (now - t0); n = 0; t0 = now; }
      for (const np of msg.poses || []) {
        const kind = fleetNames.includes(np.name) ? 'drone' : 'target';
        sc.current && sc.current.setPose(np.name, np.pose.position, np.pose.orientation, kind);
      }
    }));
    for (const name of enabled) {
      offs.push(b.subscribe(`/${name}/pose`, (msg) => {
        sc.current && sc.current.setEstimate(name, msg.pose.position, msg.pose.orientation);
      }));
    }
    // /tf as well: the simulator publishes ONLY /tf (no /cfX/pose), and it is
    // what RViz draws from. world -> cfX is the same onboard estimate.
    offs.push(b.subscribe('/tf', (msg) => {
      for (const t of msg.transforms || []) {
        if (!enabled.includes(t.child_frame_id) || !sc.current) continue;
        const tr = t.transform;
        sc.current.setEstimate(t.child_frame_id, tr.translation, tr.rotation);
      }
    }));
    for (const t of markerTopics) {
      offs.push(b.subscribe(t, (msg) => sc.current && sc.current.setMarkers(t, msg)));
    }
    return () => { offs.forEach((f) => f()); if (sc.current) sc.current.clearMarkers(); };
  }, [fleetNames.join(','), enabled.join(','), markerTopics.join(',')]);

  // drone colour follows its health tile
  useEffect(() => {
    if (!sc.current || !s.health) return;
    for (const n of s.health.nodes || []) {
      if (n.id.startsWith('drone.')) sc.current.setTone(n.id.slice(6), { ok: 'ok', warn: 'warn', fail: 'fail' }[n.status] || 'idle');
    }
  }, [s.health]);

  const f = facts();
  let note = '';
  if (b.state !== 'open') {
    note = html`<b>No live view.</b> Nothing answers at <code>${BRIDGE_URL}</code>. foxglove_bridge
      starts with the stack (<code>foxglove:=True</code>, the default)${f.server_running ? ' -- the server is up, so check the launch log for foxglove_bridge' : ' -- start the stack'}.`;
  } else if (!b.byTopic.has('/poses') && !enabled.some((n) => b.byTopic.has(`/${n}/pose`))) {
    note = html`Bridge connected, but no <code>/poses</code> and no <code>/cfX/pose</code> yet.`;
  }
  return html`<div class="scene">
    <div class="scenehost" ref=${host}></div>
    <div class="scenebar">
      <div class="seg" hidden=${sc.current && sc.current.kind === '2d'}>
        <button class=${view === 'iso' ? 'on' : ''} onClick=${() => setView('iso')} title="Perspective: drag to orbit, right-drag to pan, wheel to zoom">3D</button>
        <button class=${view === 'plan' ? 'on' : ''} onClick=${() => setView('plan')} title="Plan view: x right, y up -- the orientation of plan_escort --plot and the teleop keys">Plan</button>
      </div>
      <button class="btn ghost sm" onClick=${() => sc.current && sc.current.resetView()}>Reset view</button>
      <label class="chk"><input type="checkbox" checked=${trails} onChange=${(e) => setTrails(e.target.checked)} /> trails</label>
      <button class="btn ghost sm" onClick=${() => sc.current && sc.current.clearTrails()}>clear</button>
    </div>
    <div class="legend">
      <span><i class="sw drone"></i>drone (mocap)</span><span><i class="sw est"></i>onboard estimate</span>
      <span><i class="sw tgt"></i>other rigid body</span><span><i class="sw tested"></i>radius_tested</span>
      <span><i class="sw lost"></i>radius_lost</span><span><i class="sw mark"></i>parking mark</span>
    </div>
    ${note && html`<div class="scenenote">${note}</div>`}
  </div>`;
}

/* -------------------------------------------------------------- gate */
function Gate() {
  const s = useStore();
  const spec = s.mission;
  const main = procFor('main');
  const d = main && s.derived.get(main.id);
  const [text, setText] = useState('');
  const running = alive(main);
  const gate = running && d && d.gate;
  const g = spec.gate || { buttons: [] };
  const partial = running && main.partial && (!g.partial || re(g.partial).test(main.partial)) ? main.partial : '';

  useEffect(() => {
    if (!gate) return undefined;
    const enter = (g.buttons || []).find((b) => b.key === 'Enter');
    if (!enter) return undefined;
    const onKey = (e) => {
      if (e.key !== 'Enter' || e.repeat || confirmState) return;
      if (/^(input|textarea|select|button)$/i.test(document.activeElement && document.activeElement.tagName)) return;
      e.preventDefault();
      sendLine(main.id, enter.send);
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [gate && gate.ts, main && main.id]);

  if (!spec.gate) return null;
  let body;
  if (gate) {
    body = html`<div class="gate open">
      <div class="gprompt">${gate.text}</div>
      ${gate.details.length > 0 && html`<div class="gdetails">${gate.details.map((l) => html`<div>${l}</div>`)}</div>`}
      <div class="gbtns">${(g.buttons || []).map((bt) => html`
        <button class="btn ${bt.tone || ''} big" onClick=${() => sendLine(main.id, bt.send)}>
          ${bt.label}${bt.key ? html` <kbd>${bt.key}</kbd>` : ''}</button>`)}</div>
    </div>`;
  } else if (partial) {
    body = html`<div class="gate open">
      <div class="gprompt">${partial}</div>
      <form class="ginput" onSubmit=${(e) => { e.preventDefault(); sendLine(main.id, text); setText(''); }}>
        <input value=${text} onInput=${(e) => setText(e.target.value)} placeholder="answer (Enter sends; empty = bare Enter)" />
        <button class="btn primary">Send</button></form>
    </div>`;
  } else if (running) {
    body = html`<div class="gate wait"><span class="spin"></span> running -- no prompt waiting
      ${d && d.lastPrompt && html`<div class="glast">last: ${d.lastPrompt}</div>`}</div>`;
  } else {
    body = html`<div class="gate idle">Prompts from the script appear here as buttons when it waits for you.</div>`;
  }
  return html`<${Panel} title="Operator" cls=${gate || partial ? 'hot' : ''}>${body}<//>`;
}

/* -------------------------------------------------------- indicators */
function Indicators() {
  const s = useStore();
  const spec = s.mission;
  if (!spec.indicators.length) return null;
  const main = procFor('main');
  const d = main && s.derived.get(main.id);
  const labels = [...new Set(spec.indicators.map((r) => r.label))];
  return html`<${Panel} title="Status" right=${main ? html`<span class="st ${main.state}">${main.state}</span>` : 'not started'}>
    <div class="inds">${labels.map((l) => {
      const v = (d && d.indicators.get(l)) || { value: spec.indicators.find((r) => r.label === l).initial, tone: '' };
      return html`<div class="ind ${v.tone}"><span class="k">${l}</span><span class="v">${v.value}</span></div>`;
    })}</div><//>`;
}

/* ------------------------------------------------------------ teleop */
const held = new Map();      // code -> interval
function releaseKeys() {
  for (const [, it] of held) clearInterval(it.timer);
  held.clear();
  S.heldBy = null;
}
window.addEventListener('blur', () => { releaseKeys(); changed(); });
document.addEventListener('visibilitychange', () => { if (document.hidden) { releaseKeys(); changed(); } });

function Teleop({ only }) {
  const s = useStore();
  const spec = s.mission;
  const main = procFor('main');
  const d = main && s.derived.get(main.id);
  const values = s.launchValues || {};
  const helpers = Object.values(spec.helpers).filter((hp) => hp.teleop && (!only || hp.key === only));
  const shown = helpers.filter((hp) => Object.entries(hp.when).every(([k, v]) => String(values[k] ?? optDefault(k)) === v));
  if (!shown.length) return null;
  return shown.map((hp) => html`<${TeleopPad} key=${hp.key} hp=${hp} main=${main} d=${d} />`);
}

function optDefault(name) {
  const o = S.mission.options.find((x) => x.name === name);
  if (!o) return '';
  return o.type === 'bool' ? String(!!o.default) : (o.default ?? '');
}

function TeleopPad({ hp, main, d }) {
  const p = procFor(hp.key);
  const running = alive(p);
  const [unlock, setUnlock] = useState(false);
  const live = !hp.enable_after || (d && d.enabled.has(hp.key)) || unlock;
  const usable = running && live;
  const [kbd, setKbd] = useState(false);
  const [pressed, setPressed] = useState('');

  const send = useCallback((key) => {
    if (!p) return;
    post('/api/input', { proc: p.id, text: key, source: 'teleop' }).catch(() => {});
  }, [p && p.id]);
  const down = (k) => {
    if (!usable || held.has(k.key)) return;
    send(k.key);
    setPressed(k.key);
    if (k.hold) held.set(k.key, { timer: setInterval(() => send(k.key), 90) });
  };
  const up = (k) => {
    const it = held.get(k.key);
    if (it) { clearInterval(it.timer); held.delete(k.key); send(' '); }
    setPressed('');
  };

  useEffect(() => {
    if (!kbd || !usable) return undefined;
    S.heldBy = hp.key; changed();
    usage('teleop', { mission: MID, helper: hp.key });
    const map = new Map(hp.teleop.filter((k) => k.kbd).map((k) => [k.kbd, k]));
    const onDown = (e) => {
      const k = map.get(e.code);
      if (!k || /^(input|textarea|select)$/i.test(document.activeElement && document.activeElement.tagName)) return;
      e.preventDefault();
      if (!e.repeat) down(k);
    };
    const onUp = (e) => { const k = map.get(e.code); if (k) up(k); };
    window.addEventListener('keydown', onDown);
    window.addEventListener('keyup', onUp);
    return () => {
      window.removeEventListener('keydown', onDown);
      window.removeEventListener('keyup', onUp);
      releaseKeys();
      if (S.heldBy === hp.key) S.heldBy = null;
    };
  }, [kbd, usable]);
  useEffect(() => { if (S.heldBy && S.heldBy !== hp.key && kbd) setKbd(false); });

  const cols = Math.max(...hp.teleop.map((k) => (k.cell ? k.cell[0] : 0))) + 1;
  let state;
  if (!running) state = html`<span class="st">not running</span>`;
  else if (!live) state = html`<span class="st stopping">locked</span>`;
  else state = html`<span class="st running">live</span>`;

  return html`<${Panel} title=${hp.label} right=${state} cls="teleop">
    ${!running && html`<div class="hint">${hp.autostart ? 'Starts with the mission.' : 'Not started.'}
      <button class="btn ghost sm" onClick=${() => startRole(hp.key, S.launchValues || {}, S.launchSim).catch((e) => { S.estop = { kind: 'fail', title: hp.label + ' did not start', detail: e.message }; changed(); })}>start now</button></div>`}
    ${running && !live && html`<div class="hint warn">Locked until the script prints <code>${hp.enable_after}</code>.
      The runbook: start the teleop, then touch nothing until the show is live -- a target that is already moving makes the gather refuse.
      <a href="#" onClick=${(e) => { e.preventDefault(); setUnlock(true); }}>unlock anyway</a></div>`}
    <div class="pad ${usable ? '' : 'off'}" style=${`grid-template-columns:repeat(${cols},1fr)`}>
      ${hp.teleop.map((k) => html`<button class="key ${pressed === k.key ? 'down' : ''}" disabled=${!usable}
          style=${k.cell ? `grid-column:${k.cell[0] + 1};grid-row:${k.cell[1] + 1}` : ''}
          onPointerDown=${(e) => { e.preventDefault(); e.currentTarget.setPointerCapture(e.pointerId); down(k); }}
          onPointerUp=${() => up(k)} onPointerCancel=${() => up(k)} onLostPointerCapture=${() => up(k)}
          title=${k.hold ? 'hold to move' : 'tap'}>
          <span>${k.label}</span><small>${k.key === ' ' ? 'space' : k.key}</small></button>`)}
    </div>
    <div class="padfoot">
      <label class="chk"><input type="checkbox" checked=${kbd} disabled=${!usable} onChange=${(e) => setKbd(e.target.checked)} />
        keyboard (${hp.teleop.filter((k) => k.kbd).map((k) => (k.key === ' ' ? 'space' : k.key)).join(' ')})</label>
      ${running && html`<button class="btn ghost sm" onClick=${() => post('/api/stop', { proc: p.id, source: 'mission' })}>stop helper</button>`}
    </div>
    <div class="hint">Hold to move: the key is re-sent every 90 ms and the target coasts to a stop 0.35 s after release
      (escort_teleop's own key_timeout). Letting go, leaving the window or losing focus stops it.</div>
  <//>`;
}

/* ------------------------------------------------------------- fleet */
function Fleet() {
  const s = useStore();
  const [, tick] = useState(0);
  useEffect(() => { const t = setInterval(() => tick((x) => x + 1), 300); return () => clearInterval(t); }, []);
  const nodes = ((s.health && s.health.nodes) || []).filter((n) => n.id.startsWith('drone.'));
  const live = new Map(((window.__scene && window.__scene.bodies()) || []).map((b) => [b.name, b]));
  if (!nodes.length) return html`<${Panel} title="Fleet"><div class="hint">No enabled drones reported yet.</div><//>`;
  return html`<${Panel} title="Fleet" right=${html`<a href="/?node=drone.${nodes[0].label}#health" target="console">health</a>`}>
    <div class="fleet">${nodes.map((n) => {
      const m = n.metrics || {};
      const b = live.get(n.label);
      const truth = b && b.age < 1000;                       // mocap
      const fresh = truth || (b && b.estAge < 1000);         // or at least its own estimate
      const err = b && b.est && truth && b.estAge < 1000
        ? Math.hypot(b.pos[0] - b.est[0], b.pos[1] - b.est[1], b.pos[2] - b.est[2]) : null;
      const pct = m.volts != null ? Math.max(3, Math.min(100, Math.round((m.volts - 3.3) / 0.9 * 100))) : 0;
      const bc = m.volts == null ? '' : m.volts < 3.7 ? 'bad' : m.volts < 3.8 ? 'warn' : '';
      return html`<div class="ft ${{ ok: 'ok', warn: 'warn', fail: 'bad' }[n.status] || 'idle'}" title=${n.summary}>
        <div class="fh"><b>${n.label}</b><span>${m.state || n.status}</span></div>
        <div class="fv"><span>${m.volts != null ? m.volts.toFixed(2) + ' V' : '--'}</span>
          <span title=${truth ? 'mocap' : 'onboard estimate -- no mocap for this drone'}>${fresh ? 'z ' + b.pos[2].toFixed(2) + (truth ? '' : ' est') : 'no pose'}</span></div>
        <div class="bar"><i class=${bc} style=${`width:${pct}%`}></i></div>
        ${err != null && html`<div class=${'fe ' + (err > 0.1 ? 'warn' : '')}>est-mocap ${(err * 100).toFixed(0)} mm</div>`}
      </div>`;
    })}</div><//>`;
}

/* ------------------------------------------------------------ launch */
function Launch() {
  const s = useStore();
  const spec = s.mission;
  const f = facts();
  // the operator's choices live in the console's ONE settings store, under
  // mission.<id>.<option> -- not in this browser -- so a choice made here is
  // the one the backend runs, and any other open window sees it change
  const fromStore = () => {
    const out = {};
    for (const o of spec.options) {
      const k = `mission.${MID}.${o.name}`;
      if (s.settings[k] != null) out[o.name] = o.type === 'bool' ? s.settings[k] === 'true' : s.settings[k];
    }
    if (s.settings[`mission.${MID}.extra`] != null) out.extra = s.settings[`mission.${MID}.extra`];
    return out;
  };
  const [values, setValues] = useState(() => {
    let saved = fromStore();
    const v = {};
    for (const o of spec.options) {
      const sv = saved[o.name];
      v[o.name] = sv != null && (o.type !== 'select' || o.choices.includes(sv)) ? sv : (o.type === 'bool' ? !!o.default : (o.default ?? ''));
    }
    v.extra = saved.extra || '';
    // ?script=<pkg exe> (from the picker's "shows without a mission view")
    const want = Q.get('script');
    const so = spec.options.find((o) => o.name === 'script');
    if (want && so && so.choices.includes(want)) v.script = want;
    return v;
  });
  const [sim, setSim] = useState(null);
  const simOn = sim == null ? !!f.sim : sim;
  const [preview, setPreview] = useState({ cmd: '', helpers: [], err: '' });
  const [busy, setBusy] = useState(false);
  S.launchValues = values; S.launchSim = simOn;

  // another window changed a choice: follow it
  const storeSig = JSON.stringify(fromStore());
  useEffect(() => { setValues((cur) => ({ ...cur, ...fromStore() })); }, [storeSig]);
  const [err, setErr] = useState('');

  useEffect(() => {
    let dead = false;
    post('/api/mission/preview', { id: MID, role: 'main', values, sim: simOn })
      .then((r) => !dead && setPreview({ cmd: r.cmdline, helpers: r.helpers, err: '' }))
      .catch((e) => !dead && setPreview({ cmd: '', helpers: [], err: e.message }));
    changed();
    return () => { dead = true; };
  }, [JSON.stringify(values), simOn]);

  const main = procFor('main');
  const running = alive(main);
  const visible = (o) => Object.entries(o.when || {}).every(([k, want]) => {
    const ov = spec.options.find((x) => x.name === k);
    const v = values[k];
    return (ov && ov.type === 'bool' ? String(!!v) : String(v)) === want;
  });

  async function start() {
    if (preview.err) return;
    const helpers = preview.helpers.map((k) => spec.helpers[k]).filter((hp) => hp.autostart && !alive(procFor(hp.key)));
    const cmds = [];
    for (const hp of helpers) {
      const r = await post('/api/mission/preview', { id: MID, role: hp.key, values, sim: simOn });
      cmds.push([hp.label, r.cmdline]);
    }
    cmds.push([spec.title, preview.cmd]);
    const warn = [];
    if (!f.server_running) {
      warn.push(f.server_any ? `A crazyflie_server runs, but on ROS_DOMAIN_ID=${((f.system || {}).stack || {}).domain} -- this console is on ${(f.system || {}).my_domain}, so the script cannot reach it.`
        : 'The crazyflie_server is not running -- the script will wait or fail.');
    }
    if (f.server_running && !!f.sim !== simOn) warn.push(simOn ? 'Simulation clock ON, but the running stack is not the simulator.' : 'The running stack is the SIMULATOR but the simulation clock is off: the script will race ahead of the physics.');
    if (!f.foxglove_running) warn.push('foxglove_bridge is not running: the 3D view will stay empty (the flight is unaffected).');
    const dry = values.dry_run === true;
    const ok = await confirmBox(spec.title, html`
      ${spec.flight && !dry && html`<p><b>The drones will move. Area clear, everyone back, hand near the E-STOP?</b></p>`}
      ${warn.map((w) => html`<p class="warnline">${w}</p>`)}
      <p>This runs, in order:</p>
      ${cmds.map(([l, c]) => html`<div class="cmdlabel">${l}</div><div class="cmdline">${c}</div>`)}`,
    spec.flight && !dry ? 'Fly it' : 'Run it', spec.flight && !dry ? 'danger' : 'primary');
    if (!ok) return;
    setBusy(true);
    try {
      for (const hp of helpers) await startRole(hp.key, values, simOn);
      if (helpers.length) await new Promise((r) => setTimeout(r, 1500));   // let the helper publish first
      await startRole('main', values, simOn);
      setErr('');
    } catch (e) { setErr(e.message); }
    finally { setBusy(false); }
  }

  const fr = foreignRun();
  const outside = !running && fr.outside;
  async function stopOutside(mode) {
    const ok = await confirmBox(mode === 'kill' ? 'Kill the show (started outside the console)' : 'Stop the show (started outside the console)',
      html`<p>${mode === 'kill' ? html`<b>SIGKILL: no landing.</b>` : 'One SIGINT, as Ctrl-C in its terminal: the show lands itself.'}</p>
        <div class="cmdline">pid ${outside.pid}, ROS_DOMAIN_ID=${outside.domain}</div>`, mode === 'kill' ? 'Kill it' : 'Stop it', 'danger');
    if (!ok) return;
    try { await post('/api/signal', { pid: outside.pid, mode }); } catch (e) { setErr(e.message); }
  }
  async function abort() {
    if (!main) return;
    if (spec.abort === 'q') await sendLine(main.id, 'q');
    else await post('/api/stop', { proc: main.id, mode: 'interrupt', source: 'mission' });
  }
  async function kill() {
    const ok = await confirmBox('Kill the script',
      html`<p><b>SIGKILL ends the script without its landing.</b> Anything in the air keeps its last command
      until it times out -- be ready to E-STOP.</p><div class="cmdline">kill -KILL (process group of ${main.cmdline})</div>`,
      'Kill it', 'danger');
    if (ok) await post('/api/stop', { proc: main.id, mode: 'kill', source: 'mission' });
  }

  const set = (k, v) => {
    setValues((x) => ({ ...x, [k]: v }));
    post('/api/settings', { values: { [`mission.${MID}.${k}`]: String(v) } }).catch(() => {});
  };
  const bs = s.build;
  const building = [...s.procs.values()].some((q) => q.action_id === 'build.workspace' && alive(q));
  if (bs && !bs.ok && !running) {
    return html`<${Panel} title="Run" cls="hot" right=${html`<span class="st failed">not runnable</span>`}>
      ${bs.problems.length
        ? html`<div class="warnline"><b>Not built.</b> ${bs.problems.map((x) => html`<div>${x}</div>`)}</div>
          <div class="hint">colcon's symlink install makes EDITS to an existing module live, but a new package, a new
            script in setup.cfg or a new module file only exists after a build.</div>
          <div class="cmdline">${bs.build_cmd}</div>
          <div class="runbtns">${[...new Set(bs.build_cmd.split(' && ').map((c) => c.split(' ').pop()))].map((pkg) => html`
            <button class="btn primary big" disabled=${building} onClick=${() => buildPkg(pkg)}>${building ? 'building...' : 'Build ' + pkg}</button>`)}</div>`
        : html`<div class="warnline"><b>Built after this console started.</b> Its install is not on the console's
            environment, so <code>ros2 run</code> from here would say "Package not found".
            Restart <code>./console/run.sh</code>, then reload this window.</div>`}
    <//>`;
  }
  return html`<${Panel} title="Run" right=${main ? html`<span class="st ${main.state}">${main.state}${main.returncode != null && !running ? ' ' + main.returncode : ''}</span>` : ''}>
    <div class="opts">
      ${spec.options.filter(visible).map((o) => html`<label class="opt ${o.type}">
        <span class="ol">${o.label}</span>
        ${o.type === 'bool'
          ? html`<input type="checkbox" checked=${!!values[o.name]} disabled=${running} onChange=${(e) => set(o.name, e.target.checked)} />`
          : o.type === 'select'
            ? html`<select disabled=${running} value=${values[o.name]} onChange=${(e) => set(o.name, e.target.value)}>
                ${o.choices.map((c) => html`<option value=${c}>${o.pkgGrouped ? c : c}</option>`)}</select>`
            : html`<input type=${o.type === 'number' ? 'number' : 'text'} step="any" disabled=${running} value=${values[o.name]} onInput=${(e) => set(o.name, e.target.value)} />`}
        ${o.descriptions && o.descriptions[values[o.name]] && html`<span class="od">${o.descriptions[values[o.name]]}</span>`}
      </label>`)}
      <label class="opt bool"><span class="ol">Simulation clock (use_sim_time)</span>
        <input type="checkbox" checked=${simOn} disabled=${running} onChange=${(e) => setSim(e.target.checked)} />
        <span class="od">${f.sim ? 'the running stack is the simulator' : 'ON only with backend:=sim -- never on hardware'}</span></label>
      <details class="extra"><summary>more ROS parameters</summary>
        <input type="text" disabled=${running} value=${values.extra} placeholder="name:=value name:=value"
          onInput=${(e) => set('extra', e.target.value)} /></details>
    </div>
    <div class=${'cmdline ' + (preview.err ? 'bad' : '')} title="exactly what will run">${preview.err || preview.cmd || '...'}</div>
    ${running && main && main.id === S.adoptedId && html`<div class="hint warn">This show was started from the
      console dashboard (Fly), not this window -- it is the same run: its prompts, output and abort are here.</div>`}
    ${outside && html`<div class="warnline"><b>This show is already running, started outside the console</b>
      (pid ${outside.pid}, ROS_DOMAIN_ID=${outside.domain}). Its output went to the terminal it was started from,
      so no prompts or status here -- but the 3D view is live, and you can stop it.
      <div class="runbtns" style="margin-top:6px">
        <button class="btn warn" onClick=${() => stopOutside('int')}>Stop (lands itself)</button>
        <button class="btn ghost" onClick=${() => stopOutside('kill')}>Kill</button></div></div>`}
    ${err && html`<div class="warnline">${err} <a href="#" onClick=${(e) => { e.preventDefault(); setErr(''); }}>dismiss</a></div>`}
    <div class="runbtns">
      ${!running && html`<button class="btn ${spec.flight && values.dry_run !== true ? 'danger' : 'primary'} big" disabled=${busy || !!preview.err || !!outside} onClick=${start}>
        ${busy ? 'starting...' : values.dry_run === true ? 'Dry run' : 'Start mission'}</button>`}
      ${running && html`<button class="btn warn big" onClick=${abort}
          title=${spec.abort === 'q' ? 'sends q to the script' : 'one SIGINT: the show lands itself (abort.py). Nothing escalates.'}>
          ${main.state === 'stopping' ? 'Landing...' : 'Abort & land'}</button>
        <button class="btn ghost" onClick=${kill} title="SIGKILL: no landing">Kill</button>`}
    </div>
    ${main && main.state === 'stopping' && html`<div class="hint">Abort sent as one SIGINT. The script is landing itself; nothing
      follows it up. If it is stuck, Kill it -- or E-STOP if anything is still flying.</div>`}
    ${preview.helpers.length > 0 && html`<div class="helpers">${preview.helpers.map((k) => {
      const hp = spec.helpers[k]; const hpp = procFor(k);
      return html`<div class="helper"><span class="dot ${alive(hpp) ? 'on' : ''}"></span>${hp.label}
        <small>${hp.autostart ? 'starts with the mission' : 'manual'}</small>
        ${alive(hpp) ? html`<button class="btn ghost sm" onClick=${() => post('/api/stop', { proc: hpp.id, source: 'mission' })}>stop</button>`
          : html`<button class="btn ghost sm" onClick=${() => startRole(k, values, simOn).catch((e) => setErr(e.message))}>start</button>`}</div>`;
    })}</div>`}
    ${spec.docs && html`<div class="hint">Runbook: <code>${spec.docs}</code></div>`}
  <//>`;
}

/* ------------------------------------------------------------ events */
function fmt(ts) { return new Date(ts).toLocaleTimeString([], { hour12: false }); }
function Events() {
  const s = useStore();
  const main = procFor('main');
  const d = main && s.derived.get(main.id);
  const ref = useRef(null);
  useEffect(() => { const el = ref.current; if (el) el.scrollTop = el.scrollHeight; });
  // the script's own key lines, interleaved with every command the SERVER
  // acted on while this window was open -- from anyone (journal.py)
  const rows = [...((d && d.events) || []).map((e) => ({ ...e, kind: 'run' })),
    ...s.fleet.filter((e) => !e.backlog).map((e) => ({ ts: e.ts * 1000, kind: 'fleet', verb: e.verb,
      text: `${e.text}${e.sim_noop ? '  (sim: not implemented)' : ''}` }))]
    .sort((a, b) => a.ts - b.ts).slice(-300);
  return html`<${Panel} title="Timeline" cls="events" right=${html`<span title="lines tagged fleet are commands crazyflie_server acted on, whoever sent them">script + fleet commands</span>`}>
    <div class="evs" ref=${ref}>${rows.length ? rows.map((e) => html`
      <div class=${'ev ' + (e.kind === 'fleet' ? 'fleet ' + (e.verb === 'emergency' ? 'l-err' : '') : lineClass(e.text))}>
        <time>${fmt(e.ts)}</time>${e.kind === 'fleet' ? html`<b class="tag">fleet</b>` : ''}<span>${e.text}</span></div>`)
      : html`<div class="hint">Key moments of the run land here (the spec's <code>events</code> patterns), with every
          command the server acts on -- from this window, the console, the preflight GUI or a terminal.</div>`}</div><//>`;
}

/* ------------------------------------------------------------ output */
function lineClass(t) {
  if (/^\$ /.test(t)) return 'l-cmd';
  if (/^\[(console|exit)/.test(t)) return 'l-meta';
  if (/\*\*\*|\b(error|ERROR|Traceback|FATAL|failed|refused|cannot|ABORT)\b/.test(t)) return 'l-err';
  if (/\b(warn|WARN|WARNING|HOLD)\b/.test(t)) return 'l-warn';
  if (/>>>/.test(t)) return 'l-gate';
  return '';
}
function Output() {
  const s = useStore();
  const roles = ['main', ...Object.keys(s.mission.helpers)].filter((r) => procFor(r));
  const [role, setRole] = useState('main');
  const [text, setText] = useState('');
  const cur = roles.includes(role) ? role : roles[0];
  const p = cur && procFor(cur);
  const ref = useRef(null);
  const lines = (p && s.lines.get(p.id)) || [];
  useEffect(() => {
    const el = ref.current;
    if (el && el.scrollHeight - el.scrollTop - el.clientHeight < 80) el.scrollTop = el.scrollHeight;
  });
  return html`<${Panel} title="Output" cls="output" right=${html`<span class="tabs">${roles.map((r) => html`
      <button class=${r === cur ? 'on' : ''} onClick=${() => setRole(r)}>${r === 'main' ? s.mission.title : s.mission.helpers[r].label}</button>`)}
      ${p && html`<a href=${'/#dash'} target="console" title="the same process in the console dashboard's output pane">console</a>`}</span>`}>
    <div class="term" ref=${ref}>${lines.slice(-500).map((l) => html`<div class=${lineClass(l.text)}>${l.text || '\u00a0'}</div>`)}${p && p.partial ? html`<div class="l-gate">${p.partial}</div>` : ''}</div>
    ${p && alive(p) && cur === 'main' && html`<form class="stdin" onSubmit=${(e) => { e.preventDefault(); sendLine(p.id, text); setText(''); }}>
      <input value=${text} onInput=${(e) => setText(e.target.value)} placeholder="send a line to the script (empty = Enter)" />
      <button class="btn ghost sm">Send</button></form>`}
  <//>`;
}

/* ================================================================ shell */
const STOCK = { scene: Scene, gate: Gate, indicators: Indicators, teleop: Teleop, fleet: Fleet,
  launch: Launch, events: Events, output: Output };

function Header() {
  const s = useStore();
  const f = facts();
  const b = bridge();
  const main = s.mission && procFor('main');
  const d = main && s.derived.get(main.id);
  let state = 'READY'; let tone = '';
  if (!s.mission) { state = 'MISSIONS'; }
  else if (main && main.state === 'stopping') { state = 'LANDING'; tone = 'warn'; }
  else if (alive(main) && d && d.gate) { state = 'WAITING FOR YOU'; tone = 'hot'; }
  else if (alive(main)) { state = 'RUNNING'; tone = 'ok'; }
  else if (s.mission && foreignRun().outside) { state = 'RUNNING OUTSIDE THE CONSOLE'; tone = 'warn'; }
  else if (main && main.state === 'failed') { state = 'ENDED (FAILED)'; tone = 'fail'; }
  else if (main) { state = 'ENDED'; }
  const pill = (k, on, v, tip) => html`<span class=${'pill ' + (on ? 'on' : 'off')} title=${tip}><i></i><span class="k">${k}</span><b>${v}</b></span>`;
  return html`<header class="top">
    <a class="back" href="/" target="console" title="the mission console">console</a>
    <div class="ttl"><div class="t">${s.mission ? s.mission.title : 'Mission views'}</div>
      <div class="sub">${s.mission ? s.mission.id : 'pick a mission'}</div></div>
    ${s.mission && html`<div class=${'state ' + tone}>${state}</div>`}
    <div class="pills">
      ${pill('server', f.server_running, f.server_running ? (f.sim ? 'sim' : 'up') : 'down', 'crazyflie_server, from the console health sweep')}
      ${pill('bridge', b.state === 'open', b.state, `foxglove_bridge at ${BRIDGE_URL} -- the live 3D view`)}
      ${f.sim
        ? pill('/tf', b.byTopic.has('/tf'), 'sim', 'the simulator has no mocap: drones are drawn from /tf (their own state)')
        : pill('/poses', S.posesHz > 20, S.posesHz != null && b.state === 'open' ? S.posesHz.toFixed(0) + ' Hz' : (f.poses_hz != null ? f.poses_hz.toFixed(0) + ' Hz' : '--'), 'mocap rate as this window receives it')}
      ${pill('console', s.sse === 'open', s.sse === 'open' ? 'live' : 'lost', 'event stream from the console backend')}
    </div>
    <button class="btn estop" onClick=${fireEstop} title="One click, no confirmation: ros2 service call /all/emergency std_srvs/srv/Empty {}">E-STOP</button>
  </header>
  ${s.estop && html`<div class=${'estopban ' + s.estop.kind}><b>${s.estop.title}</b> <span>${s.estop.detail}</span>
    <button class="btn ghost sm" onClick=${() => { S.estop = null; changed(); }}>dismiss</button></div>`}`;
}

function Picker() {
  const s = useStore();
  return html`<div class="picker">
    <p class="lead">A mission view is a window built for one demo: its own controls, a live 3D view, the
      prompts it waits on as buttons. Each one is a <code>${'missions/<name>.yaml'}</code> in the show's package
      (docs/MISSIONS.md) -- add a file and it appears here.</p>
    <div class="cards">${s.missions.map((m) => html`<a class=${'mcard ' + (m.error ? 'bad' : '') + (m.build && !m.build.ok ? ' unbuilt' : '')} href=${m.error ? undefined : '/mission?m=' + encodeURIComponent(m.id)}>
      <div class="mt">${m.title}${m.running.length ? html` <span class="st running">running</span>` : ''}
        ${m.build && !m.build.ok ? html` <span class="st failed">${m.build.problems.length ? 'not built' : 'restart console'}</span>` : ''}</div>
      <div class="mid">${m.id}</div>
      <div class="ms">${m.error ? 'cannot load: ' + m.error : m.summary}</div>
      ${m.build && m.build.problems.length > 0 && html`<div class="ms warn">${m.build.problems.join('; ')}</div>`}</a>`)}</div>
    ${s.unviewed.length > 0 && html`<h3 class="sect">Shows without a mission view</h3>
      <p class="lead">Every script in a show package (anything that depends on crazyflie_py) that no${' '}
        <code>missions/*.yaml</code> runs. They open in the generic window; a script declared in${' '}
        <code>setup.cfg</code> but not built yet is listed too, with its build.</p>
      <div class="cards small">${Object.entries(s.unviewed.reduce((g, u) => ((g[u.pkg] = g[u.pkg] || []).push(u), g), {}))
        .sort(([a], [b]) => (a === 'crazyflie_examples') - (b === 'crazyflie_examples'))
        .map(([pkg, list]) => html`<div class="mcard pkg"><div class="mt">${pkg}</div>
          ${list.map((u) => html`<div class="urow">
            ${u.runnable
              ? html`<a href=${'/mission?m=console/script&script=' + encodeURIComponent(u.pkg + ' ' + u.exe)}>${u.exe}</a>`
              : html`<span>${u.exe}</span> <span class="st failed">${u.built ? 'restart console' : 'not built'}</span>`}</div>`)}
          ${list.some((u) => !u.built) && html`<button class="btn primary sm" onClick=${() => buildPkg(pkg)}>Build ${pkg}</button>`}
        </div>`)}</div>`}
    </div>`;
}

let CUSTOM = null;
async function loadCustom(spec) {
  if (!spec.view) return null;
  const [pkg, name] = spec.id.split('/');
  try { return await import(`/mission-assets/${pkg}/${name}/${spec.view}`); }
  catch (e) { return { error: String(e) }; }
}

/* One widget failing must never take the window down with it. The first
 * version had no boundary: three.js threw on a machine without WebGL, the
 * exception escaped a Preact effect, the effect queue stopped, and the Start
 * button's confirm dialog never appeared. Each widget is now fenced. */
class Guard extends Component {
  constructor() { super(); this.state = { err: null }; }
  componentDidCatch(err) { this.setState({ err }); console.error(err); }
  render() {
    if (this.state.err) {
      return html`<section class="panel"><div class="pb"><div class="warnline"><b>${this.props.name} failed:</b>
        ${String(this.state.err.message || this.state.err)}</div>
        <button class="btn ghost sm" onClick=${() => this.setState({ err: null })}>retry</button></div></section>`;
    }
    return this.props.children;
  }
}

function Body() {
  const s = useStore();
  const [custom, setCustom] = useState(null);
  useEffect(() => { if (s.mission) loadCustom(s.mission).then((m) => { CUSTOM = m; setCustom(m || {}); }); }, [s.mission && s.mission.id]);
  if (s.error) return html`<div class="fatal">${s.error}</div>`;
  if (!MID) return html`<${Picker} />`;
  if (!s.mission) return html`<div class="fatal">loading...</div>`;
  if (s.mission.view && !custom) return html`<div class="fatal">loading the mission's view...</div>`;
  const reg = { ...STOCK, ...((custom && custom.widgets) || {}) };
  const ctx = { html, h, useState, useEffect, useRef, useMemo, store: S, useStore, procFor, sendLine,
    startRole, post, bridge, facts, W: reg };
  if (custom && custom.error) {
    return html`<div class="fatal">the mission's view module failed to load: ${custom.error}</div>`;
  }
  if (custom && custom.default) return h(custom.default, { ctx });
  const slot = (name) => s.mission.layout[name].map((w) => {
    const [kind, arg] = w.split(':');
    const C = reg[kind];
    return C ? html`<${Guard} key=${w} name=${w}><${C} only=${arg} ctx=${ctx} /><//>` : html`<div class="hint">unknown widget ${w}</div>`;
  });
  return html`<main class="grid">
    <div class="main">${slot('main')}</div>
    <aside class="side">${slot('side')}</aside>
    <div class="bottom">${slot('bottom')}</div>
  </main>`;
}

function App() {
  return html`<${Header} /><${Body} /><${Confirm} />`;
}

boot();
render(html`<${App} />`, document.getElementById('app'));
