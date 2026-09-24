/* Mission console front end.
 *
 * Rules this file follows:
 *  - the command shown on a card is fetched from the backend (/api/preview), so
 *    what you read is what will run -- never a string built twice;
 *  - nothing destructive happens without a confirm step that shows the command;
 *  - every health box carries the command that decided its colour.
 */
'use strict';

const S = {
  catalog: [], groups: [], files: [], health: null, procs: [], history: [],
  lines: {}, selNode: null, selGroup: null, selFile: 'crazyflies', selProc: null, factsSig: '',
  cfg: {}, repo: '', env: {}, dashProc: null, checkedAt: 0,
};
const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => (
  { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

/* ------------------------------------------------------------------ api */
async function api(path, opts) {
  const r = await fetch(path, opts);
  const t = await r.text();
  let j; try { j = JSON.parse(t); } catch { j = { error: t }; }
  // A route the running backend does not have answers exactly {"error": "not
  // found"} (an unknown PROCESS says "no such process" instead). That means
  // this page is newer than the console process serving it: say so, instead
  // of surfacing a bare "not found".
  if (r.status === 404 && j && j.error === 'not found' && path.startsWith('/api/')) {
    markStale(`${path.split('?')[0]} is unknown to the running console`);
    const err = new Error(`${path.split('?')[0]} not found -- this console is older than the page`);
    err.stale = true;
    throw err;
  }
  if (j && j.error) throw new Error(j.error);
  return j;
}

function markStale(why) {
  const b = $('#stale-banner');
  if (!b) return;
  b.innerHTML = `<b>This console is out of date.</b> The page was updated but the console
    process was not restarted${why ? ` (${esc(why)})` : ''}. New buttons can answer
    "not found". Restart it: stop <code>./console/run.sh</code> and start it again.`;
  b.hidden = false;
}
const post = (path, body) => api(path, {
  method: 'POST', headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body || {}),
});

function toast(msg, kind = '') {
  const el = document.createElement('div');
  el.className = 'toast ' + kind;
  el.innerHTML = esc(msg).replace(/\n/g, '<br>');
  $('#toasts').appendChild(el);
  setTimeout(() => el.remove(), kind === 'err' ? 12000 : 5000);
}

function confirmRun(title, html, okLabel = 'Run it') {
  return new Promise((resolve) => {
    $('#modal-title').textContent = title;
    $('#modal-body').innerHTML = html;
    $('#modal-ok').textContent = okLabel;
    $('#modal').hidden = false;
    const done = (v) => {
      $('#modal').hidden = true;
      $('#modal-ok').onclick = $('#modal-cancel').onclick = null;
      resolve(v);
    };
    $('#modal-ok').onclick = () => done(true);
    $('#modal-cancel').onclick = () => done(false);
  });
}

/* ----------------------------------------------------------------- boot */
async function boot() {
  const b = await api('/api/bootstrap');
  S.catalog = b.catalog; S.groups = b.groups; S.files = b.files;
  S.health = b.health; S.procs = b.procs; S.history = b.history;
  if (S.procs.length) S.selProc = S.procs[S.procs.length - 1].id;   // show the newest on load
  S.repo = b.repo; S.env = b.env;
  if (b.code_changed) markStale('console code changed since it started');
  S.selGroup = S.groups[0];
  // ?node=<id> selects a health box on load, so a failing check can be linked to.
  S.selNode = new URLSearchParams(location.search).get('node') || null;
  renderTabs(); renderHealth(); renderControl(); renderDash();
  renderProcs(); renderLog(); renderTop();
  if (!location.search.includes("live=0")) events();
}

function events() {
  const src = new EventSource('/api/events');
  src.onmessage = (e) => {
    const ev = JSON.parse(e.data);
    if (ev.type === 'health') {
      S.health = ev.health; renderHealth(); renderTop();
      renderDashLive();
      const sig = JSON.stringify([ev.health.facts.server_running, ev.health.facts.enabled_drones]);
      if (sig !== S.factsSig) { S.factsSig = sig; renderControl(); }
    }
    else if (ev.type === 'proc') { upsertProc(ev.proc); renderProcs(); }
    else if (ev.type === 'line') { pushLine(ev); }
    else if (ev.type === 'history') { S.history.push(ev.entry); renderLog(); }
    else if (ev.type === 'config') { if ($('#tab-config').classList.contains('active')) loadConfig(S.selFile); }
  };
  let opened = false;
  src.onopen = async () => {
    // EventSource reconnects by itself, but events sent while it was down are
    // gone -- so resynchronise the state that matters after a reconnect.
    if (!opened) { opened = true; return; }
    try {
      const b = await api('/api/bootstrap');
      S.health = b.health; S.procs = b.procs; S.history = b.history;
      renderHealth(); renderTop(); renderProcs(); renderLog();
    } catch (e) { /* the next event or poll will catch up */ }
  };
  src.onerror = () => { /* EventSource reconnects on its own */ };
}

function renderTabs() {
  $$('#tabs button').forEach((b) => {
    b.onclick = () => {
      $$('#tabs button').forEach((x) => {
        x.classList.toggle('active', x === b);
        if (x.role || x.getAttribute('role') === 'tab') x.setAttribute('aria-selected', String(x === b));
      });
      $$('.tab').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
      history.replaceState(null, '', '#' + b.dataset.tab);   // deep-linkable tabs
      if (b.dataset.tab === 'config') loadConfig(S.selFile);
      if (b.dataset.tab === 'dash') renderDash();
    };
  });
  const want = location.hash.replace('#', '');
  if (want && $(`#tabs button[data-tab="${want}"]`)) $(`#tabs button[data-tab="${want}"]`).click();
  $('#btn-refresh').onclick = async () => {
    $('#btn-refresh').disabled = true;
    $('#headline').textContent = 'running every probe...';
    try {
      S.health = await api('/api/health?refresh=1');
      renderHealth(); renderTop(); renderDashLive();
    }
    catch (e) { toast(e.message, 'err'); }
    finally { $('#btn-refresh').disabled = false; }
  };
  wireZoom();
  wirePalette();
  $('#btn-keys').onclick = openKeyHelp;
  $('#btn-estop').onclick = fireEstop;
  $('#btn-scan').onclick = scanFleet;
  $('#btn-send').onclick = sendStdin;
  $('#stdin').onkeydown = (e) => { if (e.key === 'Enter') sendStdin(); };
}

/* --------------------------------------------------------------- ui scale */
/* Remembered here rather than relying on browser zoom, which is per-site and
 * silently lost when you open the console from a different host or profile. */
const ZOOM_STEPS = [0.7, 0.8, 0.9, 1.0, 1.1, 1.25, 1.5];

function applyZoom(z) {
  document.documentElement.style.setProperty('--ui-zoom', String(z));
  const pct = Math.round(z * 100) + '%';
  const b = $('#btn-zoom-reset');
  if (b) b.textContent = pct;
  try { localStorage.setItem('mc_zoom', String(z)); } catch (e) { /* private mode */ }
}

function currentZoom() {
  let z = 1;
  try { z = parseFloat(localStorage.getItem('mc_zoom')) || 1; } catch (e) { z = 1; }
  return ZOOM_STEPS.includes(z) ? z : 1;
}

function wireZoom() {
  const step = (dir) => {
    const i = ZOOM_STEPS.indexOf(currentZoom());
    const next = ZOOM_STEPS[Math.min(ZOOM_STEPS.length - 1, Math.max(0, i + dir))];
    applyZoom(next);
  };
  $('#btn-zoom-out').onclick = () => step(-1);
  $('#btn-zoom-in').onclick = () => step(1);
  $('#btn-zoom-reset').onclick = () => applyZoom(1.0);
  applyZoom(currentZoom());
}

/* ------------------------------------------------------------- top bar */
/* One verdict a visitor can read from two metres, then the numbers behind it.
 * The verdict word is deliberately not the raw status name: "warn" means
 * nothing to someone standing behind you; "ATTENTION" does. */
const VERDICT = {
  ok: ['ALL SYSTEMS GO', 'ok'], skip: ['ALL SYSTEMS GO', 'ok'],
  warn: ['ATTENTION', 'warn'], fail: ['FAULT', 'fail'],
  blocked: ['FAULT', 'fail'], unknown: ['CHECKING', ''],
};
const HZ_HISTORY = [];            // rolling /poses rate; the topbar draws it

/* A 46x14 sparkline over the data's own range, with a minimum span so a rock
 * steady 50 Hz draws a flat line through the middle rather than a jittery one.
 * (A fixed 0..max scale pinned the line to the top edge and the fill below it
 * became a solid block -- no information at all.) */
function sparkline(values, w = 46, h = 14) {
  if (values.length < 3) return '';
  const lo = Math.min(...values); const hi = Math.max(...values);
  const span = Math.max(hi - lo, 6);
  const mid = (hi + lo) / 2;
  const base = mid - span / 2;
  const pts = values.map((v, i) => [
    (i / (values.length - 1)) * w, h - ((v - base) / span) * (h - 2) - 1]);
  const line = pts.map(([x, y], i) => `${i ? 'L' : 'M'}${x.toFixed(1)},${y.toFixed(1)}`).join('');
  return `<svg class="spark" viewBox="0 0 ${w} ${h}" aria-hidden="true">` +
    `<path class="area" d="${line}L${w},${h}L0,${h}Z"/><path d="${line}"/></svg>`;
}

function renderTop() {
  const h = S.health || {}; const f = h.facts || {};
  const [word, cls] = VERDICT[h.worst] || VERDICT.unknown;
  $('#verdict').className = 'verdict ' + cls;
  $('#verdict-text').textContent = word;
  $('#headline').textContent = h.headline || '';
  $('#verdict').title = (h.headline || 'running the first probes') + ' -- open the health diagram';
  S.checkedAt = h.ts || S.checkedAt;

  const hz = f.poses_hz;
  if (hz != null) { HZ_HISTORY.push(hz); if (HZ_HISTORY.length > 40) HZ_HISTORY.shift(); }
  const live = (k, on, target, tip) =>
    `<a class="pill" data-go="${target}" title="${esc(tip)}"><span class="livedot ${on ? '' : 'off'}"></span>` +
    `<span class="k">${esc(k)}</span><b>${on ? 'running' : 'stopped'}</b></a>`;
  const drones = (f.enabled_drones || []);
  $('#pills').innerHTML = [
    live('server', f.server_running, 'node:server.node',
      'crazyflie_server: owns the Crazyradio and every /all/* service'),
    live('mocap', f.mocap_running, 'node:mocap.node',
      'motion_capture_tracking: the OptiTrack -> /poses driver'),
    `<a class="pill ${hz == null ? '' : (hz > 30 ? 'ok' : 'warn')}" data-go="node:mocap.poses"
       title="/poses rate -- Motive streams at 50 Hz; the last ${HZ_HISTORY.length} samples are drawn">
       <span class="k">/poses</span><b>${hz == null ? '--' : hz.toFixed(1) + ' Hz'}</b>${sparkline(HZ_HISTORY)}</a>`,
    `<a class="pill" data-go="config:crazyflies" title="enabled: ${esc(drones.join(' ') || 'none')} -- edit crazyflies.yaml">
       <span class="k">fleet</span><b>${drones.length}</b></a>`,
    `<a class="pill wide-only" data-go="node:env.ros"
       title="ROS ${esc(S.env.ros_distro)}, ROS_DOMAIN_ID=${esc(S.env.domain_id)} -- must match the terminal that started the stack">
       <span class="k">domain</span><b>${esc(S.env.domain_id)}</b></a>`,
  ].join('');
  renderCheckedAgo();
}

/* "checked 4 s ago" is how you tell a frozen page from a quiet rig. */
function renderCheckedAgo() {
  const el = $('#checked-ago');
  if (!el) return;
  if (!S.checkedAt) { el.textContent = ''; return; }
  const dt = Math.max(0, Date.now() / 1000 - S.checkedAt);
  el.textContent = '\u21bb ' + (dt < 60 ? `${Math.round(dt)} s`
    : dt < 3600 ? `${Math.round(dt / 60)} min` : 'a while');
  el.title = 'every health probe last finished ' +
    (dt < 60 ? `${Math.round(dt)} seconds` : `${Math.round(dt / 60)} minutes`) + ' ago';
}
setInterval(renderCheckedAgo, 1000);

/* -------------------------------------------------------- health graph */
// Boxes are laid out in viewBox units and the SVG scales to fit the pane, so
// the whole chain -- workspace to drones -- is visible at any window size with
// no scrolling. The box is tall enough for three lines of evidence: the summary
// IS the diagnosis, and truncating it to one line hides the useful half.
const NW = 172, NH = 86, CGAP = 26, RGAP = 20, PAD = 14, TOP = 28;

function renderHealth() {
  const h = S.health;
  if (!h || !h.nodes || !h.nodes.length) {
    $('#graph').innerHTML = '<div class="dc-empty"><span class="big">Running the first checks...</span>' +
      'Each box is one probe; they run in parallel.</div>';
    return;
  }
  const byId = Object.fromEntries(h.nodes.map((n) => [n.id, n]));
  const pos = (n) => ({ x: PAD + n.col * (NW + CGAP), y: TOP + n.row * (NH + RGAP) });
  const maxCol = Math.max(...h.nodes.map((n) => n.col));
  const maxRow = Math.max(...h.nodes.map((n) => n.row));
  const W = PAD * 2 + (maxCol + 1) * NW + maxCol * CGAP;
  const H = TOP + (maxRow + 1) * (NH + RGAP) + 14;

  // group captions sit above the first box of each group
  const caps = {};
  h.nodes.forEach((n) => {
    const p = pos(n);
    const cur = caps[n.group];
    if (!cur || p.y < cur.y || (p.y === cur.y && p.x < cur.x)) caps[n.group] = { ...p, g: n.group };
  });

  const edges = (h.edges || []).map((e) => {
    const a = byId[e.from]; const b = byId[e.to];
    if (!a || !b) return '';
    const p1 = pos(a); const p2 = pos(b);
    const x1 = p1.x + NW; const y1 = p1.y + NH / 2;
    const x2 = p2.x; const y2 = p2.y + NH / 2;
    const dead = ['fail', 'blocked', 'unknown', 'skip'].includes(b.status) ||
                 ['fail', 'blocked'].includes(a.status);
    // Marching dashes mean "messages are moving through this link right now",
    // so only the real data path gets them -- /poses out of the mocap node, and
    // the server's link to each drone. A dependency edge between two green
    // boxes (config -> radio, say) carries no traffic and stays static.
    const carries = e.to === 'mocap.poses' || e.to === 'server.services' ||
                    e.to.startsWith('drone.');
    const flow = carries && !dead && a.status === 'ok' && b.status === 'ok';
    const c = Math.max(26, (x2 - x1) / 2);
    return `<path class="gedge ${dead ? 'dead' : ''} ${flow ? 'flow' : ''}"
      d="M${x1},${y1} C${x1 + c},${y1} ${x2 - c},${y2} ${x2},${y2}"/>`;
  }).join('');

  const fill = { ok: 'rgba(63,185,80,.07)', warn: 'rgba(216,161,29,.09)', fail: 'rgba(240,86,79,.11)',
                 blocked: 'rgba(125,135,148,.05)', unknown: 'rgba(125,135,148,.05)',
                 skip: 'rgba(125,135,148,.05)' };
  const stroke = { ok: 'var(--ok)', warn: 'var(--warn)', fail: 'var(--fail)',
                   blocked: 'var(--blocked)', unknown: 'var(--unknown)', skip: 'var(--blocked)' };
  const boxes = h.nodes.map((n) => {
    const p = pos(n);
    const c = stroke[n.status] || 'var(--unknown)';
    const sub = wrap(n.summary || n.status, 26, 3);
    return `<g class="gnode st-${esc(n.status)} ${S.selNode === n.id ? 'sel' : ''}" data-id="${esc(n.id)}"
        transform="translate(${p.x},${p.y})">
      <title>${esc(n.label)} -- ${esc(n.summary || n.status)}</title>
      <rect class="box" width="${NW}" height="${NH}" rx="9" fill="${fill[n.status] || 'rgba(125,135,148,.05)'}"
        stroke="${c}"/>
      <rect class="rail" x="1.4" y="11" width="3" height="${NH - 22}" rx="1.5" fill="${c}"/>
      <circle class="halo" cx="17" cy="20" r="4.5" fill="${c}" opacity="0"/>
      <circle cx="17" cy="20" r="4.5" fill="${c}"/>
      <text class="lbl" x="28" y="24">${esc(clip(n.label, 22))}</text>
      ${sub.map((l, i) => `<text class="sub" x="14" y="${46 + i * 14}">${esc(l)}</text>`).join('')}
    </g>`;
  }).join('');

  const captions = Object.values(caps).map((c) =>
    `<text class="glabel" x="${c.x}" y="${c.y - 11}">${esc(c.g)}</text>`).join('');

  $('#graph').innerHTML =
    `<svg class="gsvg" viewBox="0 0 ${W} ${H}" preserveAspectRatio="xMidYMid meet">${edges}${captions}${boxes}</svg>`;
  $$('#graph .gnode').forEach((g) => {
    g.onclick = () => {
      S.selNode = g.dataset.id;
      const q = new URLSearchParams(location.search);
      q.set('node', S.selNode);
      history.replaceState(null, '', '?' + q + location.hash);
      renderHealth(); renderDetail();
    };
  });
  if (S.selNode) renderDetail();
}

function clip(s, n) { s = String(s || ''); return s.length > n ? s.slice(0, n - 1) + '…' : s; }
function wrap(text, width, maxLines) {
  const words = String(text || '').split(/\s+/); const out = []; let line = '';
  for (const w of words) {
    if ((line + ' ' + w).trim().length > width) { out.push(line.trim()); line = w; }
    else line = (line + ' ' + w).trim();
    if (out.length === maxLines) break;
  }
  if (out.length < maxLines && line) out.push(line.trim());
  if (out.length === maxLines) {
    const used = out.join(' ').length;
    if (used < String(text).length) out[maxLines - 1] = clip(out[maxLines - 1], width);
  }
  return out.slice(0, maxLines);
}

function renderDetail() {
  const n = (S.health.nodes || []).find((x) => x.id === S.selNode);
  const el = $('#node-detail');
  if (!n) { el.innerHTML = '<div class="empty">Pick a box in the diagram.</div>'; return; }
  const parts = [];
  parts.push(`<h2>${esc(n.label)}</h2><div class="why">${esc(n.why)}</div>`);
  parts.push(`<div class="statusline ${n.status}">${esc(n.status.toUpperCase())} &mdash; ${esc(n.summary || '')}</div>`);
  if (n.detail) parts.push(`<div class="block"><h3>What was measured</h3><pre class="out">${esc(n.detail)}</pre></div>`);
  if (n.fix) parts.push(`<div class="block"><h3>How to fix it</h3><div class="fix">${esc(n.fix)}</div></div>`);
  if (n.findings && n.findings.length) {
    parts.push('<div class="block"><h3>From the logs</h3>' + n.findings.map((f) =>
      `<div class="finding ${f.level}"><b>${esc(f.title)}</b><small>${esc(f.detail)}</small>
       ${f.fix ? `<small><br>Fix: ${esc(f.fix)}</small>` : ''}</div>`).join('') + '</div>');
  }
  if (n.commands && n.commands.length) {
    parts.push('<div class="block"><h3>Commands behind this check</h3>' + n.commands.map((c) =>
      `<div class="cmdline">${esc(c.cmd)}</div>
       ${c.out ? `<pre class="out">${esc(c.out)}</pre>` : ''}`).join('') + '</div>');
  }
  if (n.id === 'radio.drones') {
    parts.push('<div class="block"><button class="btn" onclick="window.__scan()">Scan the fleet now</button></div>');
  }
  el.innerHTML = parts.join('');
}

async function scanFleet() {
  const f = (S.health && S.health.facts) || {};
  if (f.server_running) {
    const go = await confirmRun('The server owns the radio',
      '<p>Only one process can own a Crazyradio, so a scan cannot run while ' +
      '<code>crazyflie_server</code> is up. Stop the stack first.</p>', 'Close');
    if (go !== null) return;
  }
  $('#btn-scan').disabled = true;
  $('#btn-scan').textContent = 'scanning...';
  try {
    const r = await post('/api/scan_fleet', {});
    S.health = r.health; S.selNode = 'radio.drones';
    renderHealth(); renderTop();
    const bad = Object.entries(r.scan).filter(([, v]) => !v.ok);
    toast(bad.length ? `${bad.length} address(es) did not answer -- do not launch yet`
                     : 'every enabled drone answered', bad.length ? 'err' : 'ok');
  } catch (e) { toast(e.message, 'err'); }
  finally { $('#btn-scan').disabled = false; $('#btn-scan').textContent = 'Scan the fleet'; }
}
window.__scan = scanFleet;

/* ------------------------------------------------------------- control */
/* ---------------------------------------------------------- dashboard */
/* The Control tab is the reference: one verbose card per action, with the
 * command, why it exists and how to read it. The dashboard is the cockpit: the
 * same actions as one-line buttons, in the order a session uses them, with
 * everything on one screen at the rig laptop's ~1280x660 viewport. Nothing is
 * defined twice -- a dashboard button runs the catalog action, its tooltip is
 * the real argv from /api/preview, and its "..." opens the full card. */
const DASH = [
  { title: 'Before the server', sub: 'needs the radio free', items: [
    { id: 'check.scan_all', label: 'Scan the fleet' },
    { id: 'check.battery', label: 'Battery', params: ['drone'] },
    { id: 'pos.sync_dry', label: 'Preview position sync' },
    { id: 'pos.sync_apply', label: 'Apply position sync' },
  ] },
  { title: 'Bring it up', items: [
    { id: 'launch.stack', label: 'Start the stack', params: ['backend'] },
    { id: 'launch.stop', label: 'Stop the stack' },
  ] },
  { title: 'Check', items: [
    { id: 'check.poses_hz', label: 'Measure /poses' },
    { id: 'check.status_once', label: 'Drone status', params: ['drone'] },
    { id: 'check.services', label: '/all/* services up?' },
    { id: 'tool.script', label: 'Ground check', params: ['script'] },
  ] },
  { title: 'Fly', sub: 'E-STOP is top right: one click, no confirm', items: [
    { id: 'fly.script', label: 'Fly', params: ['script', 'sim'] },
    { id: 'srv.land_all', label: 'Land all' },
  ] },
];

/* ---- remembered choices: the script you flew last is the one offered next */
function optionsHtml(p, cur) {
  const one = (o, text) =>
    `<option value="${esc(o)}" ${String(o) === String(cur) ? 'selected' : ''}>${esc(text)}</option>`;
  if (!p.grouped) return p.options.map((o) => one(o, o)).join('');
  const groups = {};
  p.options.forEach((o) => {
    const i = String(o).indexOf(' ');
    (groups[String(o).slice(0, i)] = groups[String(o).slice(0, i)] || []).push(o);
  });
  return Object.entries(groups).map(([g, opts]) =>
    `<optgroup label="${esc(g)}">${opts.map((o) => one(o, String(o).slice(g.length + 1))).join('')}</optgroup>`).join('');
}

function savedParam(actionId, p) {
  try {
    const v = localStorage.getItem(`mc_p_${actionId}_${p.name}`);
    if (v !== null && (p.type !== 'select' || p.options.map(String).includes(v))) return v;
  } catch (e) { /* storage unavailable */ }
  return p.default;
}
function saveParam(actionId, name, value) {
  try { localStorage.setItem(`mc_p_${actionId}_${name}`, value); } catch (e) { /* ignore */ }
}

/* ---- one navigation vocabulary for every link on the page: data-go="kind:arg" */
function openHealthNode(id) {
  S.selNode = id;
  gotoTab('health');
  renderHealth(); renderDetail();
}
function openCard(actionId) {
  const a = S.catalog.find((x) => x.id === actionId);
  if (!a) return;
  S.selGroup = a.group;
  gotoTab('control');
  renderControl();
  const card = $(`.card[data-act="${CSS.escape(actionId)}"]`);
  if (card) {
    card.scrollIntoView({ block: 'center' });
    card.classList.add('flash');
    setTimeout(() => card.classList.remove('flash'), 1600);
  }
}
function openProc(id) {
  S.selProc = id;
  gotoTab('procs');
  renderProcs();
}
function go(target) {
  const i = target.indexOf(':');
  const kind = target.slice(0, i); const arg = target.slice(i + 1);
  if (kind === 'node') openHealthNode(arg);
  else if (kind === 'card') openCard(arg);
  else if (kind === 'proc') openProc(arg);
  else if (kind === 'config') { S.selFile = arg; gotoTab('config'); }
  else if (kind === 'tab') gotoTab(arg);
}
document.addEventListener('click', (e) => {
  const el = e.target.closest('[data-go]');
  if (!el) return;
  e.preventDefault();
  go(el.dataset.go);
});

/* ---- live parts: fleet tiles, attention chips, activity (re-rendered often) */

/* Battery bar: a 1S LiPo runs 3.3 V (empty) to 4.2 V (full), which is all the
 * bar LENGTH means. The COLOUR comes from the same thresholds the health probe
 * uses -- 3.8 V warning, 3.7 V critical -- so a tile can never look reassuring
 * about a voltage the check already calls no-fly. */
const batteryPct = (v) => Math.max(3, Math.min(100, Math.round((v - 3.3) / 0.9 * 100)));

/* RSSI arrives as a positive number of dB below zero (53 means -53 dBm), so
 * smaller is better. Four ticks, purely a reading aid: the tile's colour still
 * comes from the probe's verdict, never from this mapping. */
const sigBars = (rssi) => (rssi == null ? 0 : rssi <= 45 ? 4 : rssi <= 60 ? 3 : rssi <= 75 ? 2 : 1);

function tileHtml(o) {
  return `<a class="dtile ${o.cls}" data-go="${esc(o.target)}" title="${esc(o.tip)}">
    <span class="dt-head"><span class="dt-name">${esc(o.name)}</span>
      <span class="dt-state">${esc(o.state)}</span></span>
    <span class="dt-main">${o.main}</span>
    ${o.bar || ''}
    ${o.foot ? `<span class="dt-foot">${o.foot}</span>` : ''}</a>`;
}

function fleetTile(n) {
  const cls = { ok: 'ok', warn: 'warn', fail: 'bad' }[n.status] || 'idle';
  const m = n.metrics || {};
  // The supervisor's own words when we have them -- E-STOPPED, FLIPPED, CRASHED,
  // FLYING -- rather than the probe's verdict ("fail" tells you nothing about
  // which of those it is, which is the thing you need walking up to the rig).
  const state = m.state ||
    ({ blocked: 'standby', skip: 'n/a', unknown: 'unchecked' }[n.status] || n.status).toUpperCase();
  let main, bar = '', foot = '';
  if (m.volts != null) {
    const pct = batteryPct(m.volts);
    const bc = m.volts < 3.7 ? 'bad' : m.volts < 3.8 ? 'warn' : '';
    main = `<span class="dt-val">${m.volts.toFixed(2)}</span><span class="dt-unit">V &middot; ${pct}%</span>`;
    bar = `<span class="bar"><i class="${bc}" style="width:${pct}%"></i></span>`;
    const bars = sigBars(m.rssi);
    foot = (m.rssi != null
      ? `<span class="sig ${bars <= 2 ? 'weak' : ''}">${[1, 2, 3, 4].map((i) =>
          `<i class="${i <= bars ? 'on' : ''}"></i>`).join('')}</span><span>${Math.round(m.rssi)} dB</span>` : '') +
      (m.latency != null ? `<span class="sep">|</span><span>${m.latency.toFixed(1)} ms</span>` : '');
  } else {
    // no telemetry (standby, sim, or a drone that never connected): keep the
    // same shape as a live tile so the row reads as one instrument, not five
    main = `<span class="dt-val dim">${esc(clip(n.summary || n.status, 48))}</span>`;
    bar = '<span class="bar"></span>';
  }
  const tip = `${n.label}: ${n.summary || n.status}` +
    ((m.flags && m.flags.length) ? `\nsupervisor: ${m.flags.join(', ')}` : '');
  return tileHtml({ cls, name: n.label, state, main, bar, foot, target: `node:${n.id}`, tip });
}

function dashStrip() {
  /* The topbar pills already say server / mocap / /poses / fleet / domain on
   * every tab, so this row carries what they cannot: each drone on its own,
   * with the two numbers that decide whether it flies (battery, link), plus the
   * radio it all goes through. Every tile opens the health check behind it. */
  const nodes = (S.health && S.health.nodes) || [];
  const radio = nodes.find((n) => n.id === 'radio.usb');
  const drones = nodes.filter((n) => n.id.startsWith('drone.'));
  if (!nodes.length) {
    return [0, 1, 2, 3, 4].map(() =>
      '<span class="dtile idle"><span class="skel" style="width:42%"></span>' +
      '<span class="skel" style="width:70%;height:17px"></span></span>').join('');
  }
  const rcls = { ok: 'ok', warn: 'warn', fail: 'bad' }[radio && radio.status] || 'idle';
  return (radio ? tileHtml({
    cls: rcls, name: 'radio',
    state: (radio.status === 'skip' ? 'n/a' : radio.status).toUpperCase(),
    main: `<span class="dt-val dim">${esc(clip(radio.summary || '--', 46))}</span>`,
    foot: '<span>Crazyradio / USB</span>', target: 'node:radio.usb',
    tip: radio.summary || 'the USB dongle every drone talks through',
  }) : '') + drones.map(fleetTile).join('');
}

function dashIssues() {
  const nodes = (S.health && S.health.nodes) || [];
  if (!nodes.length) return '<span class="hint">running the first checks...</span>';
  const rank = { fail: 0, warn: 1 };
  const bad = nodes.filter((n) => n.status in rank).sort((a, b) => rank[a.status] - rank[b.status]);
  const blocked = nodes.filter((n) => n.status === 'blocked').length;
  if (!bad.length && !blocked) return '<span class="iss ok">every check is green</span>';
  const MAX = 5;
  const chips = bad.slice(0, MAX).map((n) =>
    `<a class="iss ${n.status}" data-go="node:${esc(n.id)}" title="${esc(n.label + ': ' + (n.summary || ''))}">` +
    `<b>${esc(clip(n.label, 22))}</b> ${esc(clip(n.summary || n.status, 38))}</a>`);
  if (bad.length > MAX) chips.push(`<a class="iss more" data-go="tab:health">+${bad.length - MAX} more</a>`);
  if (blocked) {
    chips.push(`<a class="iss blocked" data-go="tab:health" title="Checks that cannot run because something upstream failed">` +
      `${blocked} blocked downstream</a>`);
  }
  return `<span class="isslabel">needs attention</span>${chips.join('')}`;
}

/* ---- the live console: which process, and what it is saying right now */
function dashProcId() {
  if (S.dashProc && S.procs.some((p) => p.id === S.dashProc)) return S.dashProc;
  const running = S.procs.filter((p) => p.state === 'running');
  const p = running.length ? running[running.length - 1] : S.procs[S.procs.length - 1];
  return p ? p.id : null;
}

function dashActivity() {
  if (!S.procs.length) {
    return '<div class="hint" style="padding:8px">Nothing run yet.</div>';
  }
  const sel = dashProcId();
  return S.procs.slice(-40).reverse().map((p) => `<a class="actrow ${esc(p.state)} ${p.id === sel ? 'sel' : ''}"
      data-dashproc="${esc(p.id)}" title="${esc(p.cmdline)}">
      <span class="adot"></span><span class="alabel">${esc(clip(p.label, 34))}</span>
      <span class="astate">${esc(p.state === 'running' ? 'live' : p.state)}${
        p.returncode != null && p.state !== 'running' ? ' ' + p.returncode : ''}</span></a>`).join('');
}

function dashOutput() {
  const head = $('#dashouthead'); const tail = $('#dashtail');
  if (!head || !tail) return;
  const p = S.procs.find((x) => x.id === dashProcId());
  if (!p) {
    head.innerHTML = '';
    tail.innerHTML = '<div class="dc-empty"><span class="big">No output yet.</span>' +
      'Run anything above and it streams here &mdash; without leaving the dashboard.</div>';
    return;
  }
  head.innerHTML = `<span class="nm">${esc(clip(p.label, 26))}</span>
    <span class="st ${esc(p.state)}">${esc(p.state)}</span>
    <span class="cmd" title="${esc(p.cmdline)}">${esc(p.cmdline)}</span>
    ${p.state === 'running' || p.state === 'stopping'
      ? `<button class="btn ghost sm" onclick="window.__stop('${esc(p.id)}',false)">Stop</button>` : ''}
    <a class="hlink" data-go="proc:${esc(p.id)}">full output &rarr;</a>`;
  const rows = S.lines[p.id];
  if (!rows) { loadProcLines(p.id); tail.innerHTML = '<span class="l-meta">loading output...</span>'; return; }
  const near = tail.scrollHeight - tail.scrollTop - tail.clientHeight < 40;
  tail.innerHTML = rows.slice(-80).map((l) =>
    `<span class="${lineClass(l.text)}">${esc(l.text)}</span>`).join('\n') ||
    '<span class="l-meta">waiting for output...</span>';
  if (near) tail.scrollTop = tail.scrollHeight;
}

function dashBlocked(a, f) {
  if (a.requires === 'server_running' && !f.server_running) return 'needs the server running';
  if (a.requires === 'server_stopped' && f.server_running) return 'the server owns the radio -- stop it first';
  return '';
}

function renderDashLive() {
  if (!$('#tab-dash').classList.contains('active')) return;
  const f = (S.health && S.health.facts) || {};
  $('#dashstrip').innerHTML = dashStrip();
  $('#dashissues').innerHTML = dashIssues();
  $('#dashactivity').innerHTML = dashActivity();
  dashOutput();
  // when every button in a column is blocked for the same reason, say it once
  // under the heading instead of repeating the same sentence four times
  $$('#dashcols .dcol').forEach((col) => {
    const items = $$('.qa', col);
    const whys = items.map((el) => {
      const a = S.catalog.find((x) => x.id === el.dataset.qa);
      return a ? dashBlocked(a, f) : '';
    });
    const shared = whys.length > 1 && whys.every((w) => w && w === whys[0]);
    const note = $('.dcol-why', col);
    if (note) note.textContent = shared ? whys[0] : '';
    let last = '';
    items.forEach((el, i) => {
      el.classList.toggle('blocked', !!whys[i]);
      // say it once per run of buttons blocked by the same thing
      $('.qa-why', el).textContent = (shared || whys[i] === last) ? '' : whys[i];
      last = whys[i];
    });
  });
}
let dashTimer = null;
function renderDashSoon() {
  if (dashTimer) return;
  dashTimer = setTimeout(() => { dashTimer = null; renderDashLive(); }, 250);
}

/* Clicking a row in Activity keeps you on the dashboard and shows its output
 * beside it; "full output ->" is there for when you want the whole pane. */
document.addEventListener('click', (e) => {
  const el = e.target.closest('[data-dashproc]');
  if (!el) return;
  S.dashProc = el.dataset.dashproc;
  renderDashLive();
});

/* ---- structure: the step columns (rebuilt only when the catalog changes) */
function qaHtml(it, a) {
  const shown = (it.params || []).map((n) => a.params.find((q) => q.name === n)).filter(Boolean);
  const sel = shown.map((q) => {
    const opts = optionsHtml(q, savedParam(a.id, q));
    return `<select data-param="${esc(q.name)}" title="${esc(q.label + (q.help ? ' -- ' + q.help : ''))}">${opts}</select>`;
  }).join('');
  const desc = shown.some((q) => Object.keys(q.descriptions || {}).length)
    ? '<div class="qa-desc"></div>' : '';
  const kind = a.danger === 'flight' ? 'flight' : '';
  return `<div class="qa" data-qa="${esc(a.id)}">
    <div class="qa-row">
      <button class="btn sm qa-run ${kind}" data-run>${esc(it.label)}</button>
      <a class="qa-more" data-go="card:${esc(a.id)}" title="Full card: every option, the exact command and how to read it">&#8943;</a>
    </div>
    ${sel ? `<div class="qa-params">${sel}</div>` : ''}
    ${desc}<div class="qa-why"></div></div>`;
}

function wireQa(el, a) {
  const values = () => {
    const v = {};
    $$('[data-param]', el).forEach((s) => { v[s.dataset.param] = s.value; });
    return v;
  };
  const run = $('[data-run]', el);
  const refresh = async () => {
    const v = values();
    const d = $('.qa-desc', el);
    if (d) {
      const q = a.params.find((x) => Object.keys(x.descriptions || {}).length);
      d.textContent = (q && q.descriptions[v[q.name]]) || '';
    }
    try {
      const r = await post('/api/preview', { action_id: a.id, values: v });
      run.title = `${r.cmdline}\n\n${a.why}`;
    } catch (e) { run.title = a.why; }
  };
  $$('[data-param]', el).forEach((s) => {
    s.onchange = () => { saveParam(a.id, s.dataset.param, s.value); refresh(); };
  });
  run.onclick = () => runAction(a.id, values(), { stay: true });
  refresh();
}

function renderDash() {
  const byId = Object.fromEntries(S.catalog.map((a) => [a.id, a]));
  $('#dashcols').innerHTML = DASH.map((c, i) => {
    const items = c.items.filter((it) => byId[it.id]);
    return `<section class="dcol"><h2><span class="step">${i + 1}</span>${esc(c.title)}</h2>
      ${c.sub ? `<div class="dsub">${esc(c.sub)}</div>` : ''}
      <div class="qa-why dcol-why"></div>
      ${items.map((it) => qaHtml(it, byId[it.id])).join('')}</section>`;
  }).join('');
  DASH.forEach((c) => c.items.forEach((it) => {
    const el = $(`#dashcols .qa[data-qa="${CSS.escape(it.id)}"]`);
    if (el) wireQa(el, byId[it.id]);
  }));
  renderDashLive();
}

function renderControl() {
  $('#grouplist').innerHTML = S.groups.map((g) =>
    `<button class="${g === S.selGroup ? 'active' : ''}" data-g="${esc(g)}">${esc(g)}</button>`).join('');
  $$('#grouplist button').forEach((b) => {
    b.onclick = () => { S.selGroup = b.dataset.g; renderControl(); };
  });
  const f = (S.health && S.health.facts) || {};
  const acts = S.catalog.filter((a) => a.group === S.selGroup);
  $('#actions').innerHTML = acts.map((a) => cardHtml(a, f)).join('');
  acts.forEach((a) => wireCard(a));
}

function cardHtml(a, f) {
  const tags = [];
  if (a.kind === 'service') tags.push('<span class="tag">stays running</span>');
  if (a.danger === 'flight') tags.push('<span class="tag flight">drones move</span>');
  if (a.danger === 'estop') tags.push('<span class="tag estop">cuts motors</span>');
  let warn = '';
  if (a.requires === 'server_running' && !f.server_running) {
    warn = 'The crazyflie_server is not running, so this has nothing to talk to.';
  } else if (a.requires === 'server_stopped' && f.server_running) {
    warn = 'The server is running and owns the Crazyradio. Stop the stack first, or this will fail.';
  }
  return `<div class="card" data-act="${esc(a.id)}">
    <h3>${esc(a.label)} ${tags.join(' ')}</h3>
    <div class="why">${esc(a.why)}</div>
    ${a.params.length ? `<div class="params">${a.params.map((q) => paramHtml(q, a.id)).join('')}</div>` : ''}
    <div class="cmdline" data-preview>building the command...</div>
    <div class="cardfoot">
      <button class="btn primary" data-run>Run</button>
      <button class="btn ghost sm" data-copy>Copy command</button>
      ${a.docs ? `<span class="hint">docs: ${esc(a.docs)}</span>` : ''}
    </div>
    ${warn ? `<div class="warnline">${esc(warn)}</div>` : ''}
    ${a.teaches ? `<details class="teaches"><summary>How to read this command</summary>${esc(a.teaches)}</details>` : ''}
  </div>`;
}

function paramHtml(p, actionId) {
  const id = `p_${Math.random().toString(36).slice(2, 8)}`;
  const cur = actionId ? savedParam(actionId, p) : p.default;
  if (p.type === 'select') {
    const hasDesc = Object.keys(p.descriptions || {}).length;
    return `<div class="${hasDesc ? 'wide' : ''}"><label for="${id}" title="${esc(p.help)}">${esc(p.label)}</label>
      <select id="${id}" data-param="${esc(p.name)}">${optionsHtml(p, cur)}</select>
      ${hasDesc ? `<div class="optdesc" data-desc="${esc(p.name)}">${esc(p.descriptions[cur] || '')}</div>` : ''}</div>`;
  }
  const type = p.type === 'number' ? 'number' : 'text';
  return `<div><label for="${id}" title="${esc(p.help)}">${esc(p.label)}</label>
    <input id="${id}" type="${type}" step="any" data-param="${esc(p.name)}" value="${esc(cur)}"></div>`;
}

function cardValues(card) {
  const v = {};
  $$('[data-param]', card).forEach((el) => { v[el.dataset.param] = el.value; });
  return v;
}

function wireCard(a) {
  const card = $(`.card[data-act="${CSS.escape(a.id)}"]`);
  if (!card) return;
  const preview = $('[data-preview]', card);
  let timer = null;
  const refresh = async () => {
    try {
      const r = await post('/api/preview', { action_id: a.id, values: cardValues(card) });
      preview.textContent = r.cmdline;
      card.dataset.cmd = r.cmdline;
    } catch (e) { preview.textContent = 'preview failed: ' + e.message; }
  };
  $$('[data-param]', card).forEach((el) => {
    el.oninput = () => { clearTimeout(timer); timer = setTimeout(refresh, 200); };
    el.onchange = () => {
      saveParam(a.id, el.dataset.param, el.value);
      const q = a.params.find((x) => x.name === el.dataset.param);
      const d = $(`[data-desc="${CSS.escape(el.dataset.param)}"]`, card);
      if (d && q) d.textContent = (q.descriptions || {})[el.value] || '';
      refresh();
    };
  });
  refresh();
  $('[data-copy]', card).onclick = () => {
    navigator.clipboard.writeText(card.dataset.cmd || '').then(
      () => toast('command copied'), () => toast('could not copy', 'err'));
  };
  $('[data-run]', card).onclick = () => runAction(a.id, cardValues(card));
}

/* --------------------------------------------------------------- e-stop */
/* One click, no confirmation, no tab switch -- the same contract as the
 * preflight GUI's button and its `e` key. It used to go through the generic
 * run path: fetch a preview, open a confirm modal, then jump to the Processes
 * tab, where an unreachable server (dead, hung, or on another ROS_DOMAIN_ID)
 * showed the bare command and then nothing, forever. Now the backend answers
 * within ESTOP_CONFIRM_S and this banner says what actually happened. */
function estopBanner(kind, title, detail, procId) {
  const b = $('#estop-banner');
  b.className = 'estopban ' + kind;
  b.innerHTML = `<div class="eb-main"><b>${esc(title)}</b>
      ${detail ? `<span class="eb-detail">${esc(detail)}</span>` : ''}</div>
    <div class="eb-actions">
      ${procId ? `<button class="btn ghost sm" data-eb-proc="${esc(procId)}">output</button>` : ''}
      <button class="btn ghost sm" data-eb-close>dismiss</button></div>`;
  b.hidden = false;
  const po = $('[data-eb-proc]', b);
  if (po) po.onclick = () => { S.selProc = po.dataset.ebProc; renderProcs(); gotoTab('procs'); };
  $('[data-eb-close]', b).onclick = () => { b.hidden = true; };
}

let estopBusy = false;
async function fireEstop() {
  if (estopBusy) return;          // a second click must not queue a second call
  estopBusy = true;
  const btn = $('#btn-estop');
  btn.classList.add('firing');
  estopBanner('pending', 'E-STOP sent', 'waiting for the crazyflie_server to confirm...');
  try {
    let r;
    try {
      r = await post('/api/estop', {});
    } catch (e) {
      if (e.stale) return await estopLegacy();   // outdated backend: still fire
      throw e;
    }
    S.estopPending = r.pending ? r.proc : null;
    if (r.confirmed) {
      estopBanner('ok', `E-STOP confirmed in ${r.elapsed.toFixed(2)} s`,
        'Motors cut on every drone. They need a reboot (power-cycle) before they will arm again.', r.proc);
    } else {
      estopBanner('fail', r.pending
        ? `E-STOP NOT CONFIRMED after ${r.elapsed.toFixed(1)} s`
        : 'E-STOP FAILED', r.reason, r.proc);
    }
  } catch (e) {
    estopBanner('fail', 'E-STOP could not reach the console backend',
      `${e.message} -- use the preflight GUI (e) or cut power.`);
  } finally {
    estopBusy = false;
    btn.classList.remove('firing');
  }
}

/* The running console predates /api/estop. An e-stop must still fire, so use
 * the route every version has (/api/run -- without the confirm modal) and do
 * the confirming here, by watching the process it starts. */
async function estopLegacy() {
  const t0 = performance.now();
  let p;
  try {
    p = await post('/api/run', { action_id: 'srv.estop', values: {} });
  } catch (e) {
    estopBanner('fail', 'E-STOP could not be sent',
      `${e.message} -- use the preflight GUI (e) or cut power.`);
    return;
  }
  const sleep = (ms) => new Promise((res) => setTimeout(res, ms));
  while (performance.now() - t0 < 3000) {
    await sleep(100);
    let q = null;
    try { q = (await api('/api/procs')).procs.find((x) => x.id === p.id); } catch (e) { /* keep polling */ }
    if (q && q.state === 'done') {
      estopBanner('ok', `E-STOP confirmed in ${((performance.now() - t0) / 1000).toFixed(2)} s`,
        'Sent through an OUT-OF-DATE console -- restart ./console/run.sh. Drones need a power-cycle before they arm again.', p.id);
      return;
    }
    if (q && (q.state === 'failed' || q.state === 'stopped')) {
      estopBanner('fail', 'E-STOP FAILED', `exited ${q.returncode} -- use the preflight GUI (e) or cut power.`, p.id);
      return;
    }
  }
  estopBanner('fail', 'E-STOP NOT CONFIRMED after 3.0 s',
    'Sent through an OUT-OF-DATE console, which cannot time the call out: it may still be pending. ' +
    'Use the preflight GUI (e) or cut power, then restart ./console/run.sh.', p.id);
}

/* A NOT-CONFIRMED call keeps retrying in the backend; if it lands late, say so. */
function estopLate(p) {
  if (!S.estopPending || p.id !== S.estopPending) return;
  if (p.state === 'done') {
    S.estopPending = null;
    const took = p.ended && p.started ? (p.ended - p.started).toFixed(1) + ' s' : 'late';
    estopBanner('ok', `E-STOP confirmed late (${took})`,
      'The server did accept it, but slowly -- find out why before the next flight.', p.id);
  } else if (p.state === 'stopped' || p.state === 'failed') {
    S.estopPending = null;
    estopBanner('fail', 'E-STOP was never confirmed',
      'The backend gave up waiting. Assume the motors were NOT cut.', p.id);
  }
}

async function runAction(actionId, values, opts = {}) {
  const a = S.catalog.find((x) => x.id === actionId);
  if (!a) return;
  if (a.danger === 'estop') return fireEstop();
  let cmd = '';
  try {
    cmd = (await post('/api/preview', { action_id: actionId, values })).cmdline;
  } catch (e) { toast(e.message, 'err'); return; }
  const f = (S.health && S.health.facts) || {};
  const notes = [];
  if (a.confirm) notes.push(`<p><b>${esc(a.confirm)}</b></p>`);
  if (a.requires === 'server_running' && !f.server_running) {
    notes.push('<p class="warnline">The crazyflie_server is not running.</p>');
  }
  if (a.requires === 'server_stopped' && f.server_running) {
    notes.push('<p class="warnline">The server is running and owns the radio; this will probably fail.</p>');
  }
  if (a.danger !== 'none' || notes.length) {
    const ok = await confirmRun(a.label,
      `${notes.join('')}<p>This runs:</p><div class="cmdline">${esc(cmd)}</div>
       <p class="hint">Working directory: ${esc(S.repo)}</p>`,
      a.danger === 'estop' ? 'CUT THE MOTORS' : 'Run it');
    if (!ok) return;
  }
  try {
    const p = await post('/api/run', { action_id: actionId, values });
    upsertProc(p); S.selProc = p.id; S.lines[p.id] = S.lines[p.id] || [];
    renderProcs();
    toast('started: ' + p.cmdline.slice(0, 70));
    if (opts.stay) renderDashLive(); else gotoTab('procs');
  } catch (e) { toast(e.message, 'err'); }
}

function gotoTab(name) {
  const b = $(`#tabs button[data-tab="${name}"]`);
  if (b) b.click();
}

/* -------------------------------------------------------------- config */
async function loadConfig(key) {
  S.selFile = key;
  $('#filelist').innerHTML = S.files.map((f) =>
    `<button class="${f.key === key ? 'active' : ''}" data-f="${esc(f.key)}">${esc(f.label)}</button>`).join('');
  $$('#filelist button').forEach((b) => { b.onclick = () => loadConfig(b.dataset.f); });
  const pane = $('#configpane');
  pane.innerHTML = '<p class="hint">loading...</p>';
  let data;
  try { data = await api('/api/config/' + key); } catch (e) { pane.innerHTML = `<p class="warnline">${esc(e.message)}</p>`; return; }
  S.cfg[key] = data;
  const f = (S.health && S.health.facts) || {};
  const live = f.server_running
    ? `<div class="warnline">The server is running. crazyflies.yaml is read ONLY at launch, so
        changes here take effect after you restart the stack.</div>` : '';
  const head = `<h2>${esc(data.label)}</h2>
    <div class="blurb">${esc(data.blurb)}<br><span class="hint">${esc(data.path)}</span></div>${live}`;
  pane.innerHTML = head + (key === 'crazyflies' ? fleetEditor(data) : kvEditor(key, data)) + rawEditor(key, data);
  wireConfig(key, data);
}

function findingsHtml(findings) {
  if (!findings || !findings.length) {
    return '<div class="block"><div class="finding info"><b>No problems found</b>' +
      '<small>Checked addresses, drone separation, robot types, dongle channel spacing ' +
      'and the 26-byte firmware log-block budget.</small></div></div>';
  }
  return '<div class="block">' + findings.map((f) =>
    `<div class="finding ${f.level === 'fail' ? '' : f.level}"><b>${esc(f.title)}</b>
      <small>${esc(f.detail)}</small>${f.fix ? `<small><br>Fix: ${esc(f.fix)}</small>` : ''}</div>`).join('') +
    '</div>';
}

function fleetEditor(data) {
  const types = data.types || ['cf21'];
  const rows = (data.fleet || []).map((d) => `<tr data-drone="${esc(d.name)}" class="${d.enabled ? '' : 'off'}">
    <td><input type="checkbox" data-field="enabled" ${d.enabled ? 'checked' : ''}></td>
    <td><b>${esc(d.name)}</b></td>
    <td><input type="text" data-field="uri" value="${esc(d.uri)}"></td>
    <td class="num"><input type="number" step="0.001" data-field="x" value="${esc((d.initial_position || [0, 0, 0])[0])}"></td>
    <td class="num"><input type="number" step="0.001" data-field="y" value="${esc((d.initial_position || [0, 0, 0])[1])}"></td>
    <td class="num"><input type="number" step="0.001" data-field="z" value="${esc((d.initial_position || [0, 0, 0])[2])}"></td>
    <td><select data-field="type">${types.map((t) =>
      `<option ${t === d.type ? 'selected' : ''}>${esc(t)}</option>`).join('')}</select></td>
    <td><button class="btn ghost sm" data-remove="${esc(d.name)}">remove</button></td>
  </tr>`).join('');
  return `<div class="block"><h3>Fleet</h3>
    <table class="fleet"><thead><tr>
      <th>on</th><th>name</th><th>uri</th><th>x [m]</th><th>y [m]</th><th>z [m]</th><th>type</th><th></th>
    </tr></thead><tbody id="fleetbody">${rows}</tbody></table>
    <div class="cardfoot">
      <button class="btn ghost sm" id="btn-adddrone">Add a drone</button>
      <span class="hint">Enabled drones must sit at least 1 m apart, and every address must be unique.
        The server blocks forever on the first enabled drone that does not answer radio, so disable
        a dead one here rather than launching and waiting.</span>
    </div>
    <div class="cardfoot">
      <button class="btn" id="btn-fleet-preview">Preview changes</button>
      <button class="btn primary" id="btn-fleet-save">Save to crazyflies.yaml</button>
      <span class="hint">A timestamped backup goes to console/backups/ first. Comments are preserved.</span>
    </div>
    <pre class="diff" id="fleetdiff" hidden></pre>
    </div>
    <div class="block"><h3>Validation</h3>${findingsHtml(data.findings)}</div>`;
}

function kvEditor(key, data) {
  if (key !== 'motion_capture') return findingsHtml(data.findings);
  const p = ((data.doc_params || {}));
  return `<div class="block"><h3>Quick settings</h3>
    <div class="params">
      <div><label>Motive host</label>
        <input type="text" id="mc-host" value="${esc(mcValue(data, 'hostname'))}"
          placeholder="auto"></div>
      <div><label>parser type</label>
        <select id="mc-type">${['optitrack', 'optitrack_closed_source', 'vicon', 'qualisys', 'vrpn'].map((t) =>
          `<option ${t === mcValue(data, 'type') ? 'selected' : ''}>${esc(t)}</option>`).join('')}</select></div>
    </div>
    <div class="cardfoot">
      <button class="btn primary" id="btn-mc-save">Save</button>
      <span class="hint">"auto" makes launch.py discover the Motive PC with a NatNet ping --
        the right choice in this lab, where its DHCP address keeps moving.</span>
    </div></div>` + findingsHtml(data.findings);
}

function mcValue(data, field) {
  const m = new RegExp('^\\s*' + field + ':\\s*"?([^"#\\n]+)"?', 'm').exec(data.text || '');
  return m ? m[1].trim() : '';
}

function rawEditor(key, data) {
  return `<div class="block"><h3>Raw file</h3>
    <textarea class="raw" id="rawtext" spellcheck="false">${esc(data.text)}</textarea>
    <div class="cardfoot">
      <button class="btn" id="btn-raw-preview">Preview diff</button>
      <button class="btn primary" id="btn-raw-save">Save file</button>
      <button class="btn ghost" id="btn-raw-reload">Reload from disk</button>
      <span class="hint">The file is parsed before anything is written -- invalid YAML is refused.</span>
    </div>
    <pre class="diff" id="rawdiff" hidden></pre></div>`;
}

function wireConfig(key, data) {
  const raw = $('#rawtext');
  $('#btn-raw-reload').onclick = () => loadConfig(key);
  $('#btn-raw-preview').onclick = async () => {
    try {
      const r = await post('/api/config/raw', { file: key, text: raw.value, apply: false });
      showDiff('#rawdiff', r.diff);
    } catch (e) { toast(e.message, 'err'); }
  };
  $('#btn-raw-save').onclick = async () => {
    try {
      const r = await post('/api/config/raw', { file: key, text: raw.value, apply: true });
      toast(r.written ? 'saved (backup: ' + r.backup + ')' : 'nothing changed', 'ok');
      showDiff('#rawdiff', r.diff);
      loadConfig(key);
    } catch (e) { toast(e.message, 'err'); }
  };

  if (key === 'motion_capture') {
    $('#btn-mc-save').onclick = async () => {
      const edits = [
        { keys: ['/motion_capture_tracking', 'ros__parameters', 'hostname'], value: '"' + $('#mc-host').value.trim() + '"' },
        { keys: ['/motion_capture_tracking', 'ros__parameters', 'type'], value: '"' + $('#mc-type').value + '"' },
      ];
      try {
        const r = await post('/api/config/kv', { file: key, edits, apply: true });
        toast(r.written ? 'saved (backup: ' + r.backup + ')' : 'nothing changed', 'ok');
        loadConfig(key);
      } catch (e) { toast(e.message, 'err'); }
    };
  }

  if (key !== 'crazyflies') return;
  const removals = new Set();
  const adds = [];
  $$('#fleetbody [data-remove]').forEach((b) => {
    b.onclick = () => {
      const row = b.closest('tr');
      if (removals.has(b.dataset.remove)) {
        removals.delete(b.dataset.remove); row.style.textDecoration = ''; b.textContent = 'remove';
      } else {
        removals.add(b.dataset.remove); row.style.textDecoration = 'line-through'; b.textContent = 'undo';
      }
    };
  });
  $('#btn-adddrone').onclick = async () => {
    const name = prompt('New drone name (must match the Motive rigid body), e.g. cf7');
    if (!name) return;
    const n = parseInt(String(name).replace(/\D/g, ''), 10);
    const addr = Number.isFinite(n) ? 'E7E7E7E7' + n.toString(16).toUpperCase().padStart(2, '0') : 'E7E7E7E701';
    adds.push({ name: name.trim(), uri: `radio://0/80/2M/${addr}`, x: 0, y: 0, z: 0, type: 'cf21', enabled: true });
    toast(`${name} staged with uri radio://0/80/2M/${addr} -- press Save, then scan it before launching`);
  };

  const collect = () => {
    const edits = [];
    $$('#fleetbody tr').forEach((tr) => {
      const name = tr.dataset.drone;
      if (removals.has(name)) return;
      const orig = (data.fleet || []).find((d) => d.name === name) || {};
      const en = $('[data-field=enabled]', tr).checked;
      const uri = $('[data-field=uri]', tr).value.trim();
      const type = $('[data-field=type]', tr).value;
      const pos = ['x', 'y', 'z'].map((f) => parseFloat($(`[data-field=${f}]`, tr).value) || 0);
      if (en !== orig.enabled) edits.push({ name, field: 'enabled', value: en });
      if (uri !== orig.uri) edits.push({ name, field: 'uri', value: uri });
      if (type !== orig.type) edits.push({ name, field: 'type', value: type });
      const op = (orig.initial_position || [0, 0, 0]).map(Number);
      if (pos.some((v, i) => Math.abs(v - op[i]) > 1e-9)) {
        edits.push({ name, field: 'initial_position', value: pos });
      }
    });
    return { edits, adds, removes: [...removals] };
  };

  $('#btn-fleet-preview').onclick = async () => {
    const body = { ...collect(), apply: false };
    if (!body.edits.length && !body.adds.length && !body.removes.length) {
      toast('nothing changed'); return;
    }
    try { showDiff('#fleetdiff', (await post('/api/config/robots', body)).diff); }
    catch (e) { toast(e.message, 'err'); }
  };
  $('#btn-fleet-save').onclick = async () => {
    const body = { ...collect(), apply: false };
    if (!body.edits.length && !body.adds.length && !body.removes.length) { toast('nothing changed'); return; }
    let diff = '';
    try { diff = (await post('/api/config/robots', body)).diff; }
    catch (e) { toast(e.message, 'err'); return; }
    const ok = await confirmRun('Write crazyflies.yaml',
      `<p>This is exactly what changes (a backup is taken first):</p>
       <pre class="diff">${diffHtml(diff)}</pre>`, 'Write the file');
    if (!ok) return;
    try {
      const r = await post('/api/config/robots', { ...collect(), apply: true });
      toast(r.written ? 'saved (backup: ' + r.backup + ')' : 'nothing changed', 'ok');
      loadConfig('crazyflies');
    } catch (e) { toast(e.message, 'err'); }
  };
}

function diffHtml(diff) {
  return String(diff || '(no change)').split('\n').map((l) => {
    const c = l.startsWith('+') && !l.startsWith('+++') ? 'add'
      : l.startsWith('-') && !l.startsWith('---') ? 'del'
        : l.startsWith('@@') ? 'hd' : '';
    return c ? `<span class="${c}">${esc(l)}</span>` : esc(l);
  }).join('\n');
}
function showDiff(sel, diff) {
  const el = $(sel);
  el.hidden = false;
  el.innerHTML = diffHtml(diff || '(no change)');
}

/* ----------------------------------------------------------- processes */
function upsertProc(p) {
  const i = S.procs.findIndex((x) => x.id === p.id);
  if (i >= 0) S.procs[i] = p; else S.procs.push(p);
  estopLate(p);
  renderDashSoon();
  if (!S.selProc) S.selProc = p.id;
}

function pushLine(ev) {
  (S.lines[ev.proc] = S.lines[ev.proc] || []).push({ seq: ev.seq, text: ev.text });
  if (S.lines[ev.proc].length > 3000) S.lines[ev.proc].splice(0, 500);
  if (ev.proc === S.selProc) appendLine(ev.text);
  renderDashSoon();
}

function lineClass(t) {
  if (/^\$ /.test(t)) return 'l-cmd';
  if (/^\[(console|exit)/.test(t)) return 'l-meta';
  if (/\b(error|ERROR|Traceback|FATAL|failed|refused|cannot)\b/.test(t)) return 'l-err';
  if (/\b(warn|WARN|WARNING)\b/.test(t)) return 'l-warn';
  return '';
}

function appendLine(text) {
  const body = $('#outbody');
  const span = document.createElement('span');
  span.className = lineClass(text);
  span.textContent = text + '\n';
  body.appendChild(span);
  if ($('#autoscroll').checked) body.scrollTop = body.scrollHeight;
}

function renderProcs() {
  const alive = (p) => p.state === 'running' || p.state === 'stopping';
  const live = S.procs.filter(alive).length;
  $('#proc-badge').textContent = live || '';
  document.body.classList.toggle('busy', live > 0);
  $('#proclist').innerHTML = S.procs.slice().reverse().map((p) =>
    `<button class="${p.id === S.selProc ? 'active' : ''}" data-p="${esc(p.id)}">
      <div>${esc(clip(p.label, 26))}</div>
      <div class="st ${p.state}">${esc(p.state)}${p.returncode != null && !alive(p) ? ' (' + p.returncode + ')' : ''}</div>
    </button>`).join('') || '<div class="hint" style="padding:12px">Nothing has been run yet.</div>';
  $$('#proclist button').forEach((b) => {
    b.onclick = () => { S.selProc = b.dataset.p; renderProcs(); loadProcLines(b.dataset.p); };
  });
  const p = S.procs.find((x) => x.id === S.selProc);
  $('#outhead').innerHTML = p
    ? `<b>${esc(p.label)}</b><span class="st ${p.state}">${esc(p.state)}</span>
       <div class="cmdline" style="flex:1 0 100%">${esc(p.cmdline)}</div>
       ${alive(p)
        ? `<button class="btn ghost sm" onclick="window.__stop('${p.id}',false)">Stop (SIGINT)</button>
           <button class="btn danger sm" onclick="window.__stop('${p.id}',true)">Kill (SIGKILL)</button>` : ''}
       <button class="btn ghost sm" onclick="window.__copy('${p.id}')">Copy command</button>`
    : '<span class="hint">No process selected.</span>';
  if (p && !S.lines[p.id]) loadProcLines(p.id);
  else if (p) redrawLines();
}

function redrawLines() {
  const body = $('#outbody');
  body.textContent = '';
  (S.lines[S.selProc] || []).forEach((l) => appendLine(l.text));
}

async function loadProcLines(id) {
  try {
    const r = await api('/api/proc/' + id);
    S.lines[id] = r.lines.map((l) => ({ seq: l.seq, text: l.text }));
    if (id === S.selProc) redrawLines();
    renderDashSoon();
  } catch (e) { /* the process may have been pruned */ }
}

window.__stop = async (id, hard) => {
  try { await post('/api/stop', { proc: id, hard }); toast(hard ? 'SIGKILL sent' : 'SIGINT sent'); }
  catch (e) { toast(e.message, 'err'); }
};
window.__copy = (id) => {
  const p = S.procs.find((x) => x.id === id);
  if (p) navigator.clipboard.writeText(p.cmdline).then(() => toast('command copied'));
};

function sendStdin() {
  const el = $('#stdin');
  if (!S.selProc || !el.value) return;
  post('/api/input', { proc: S.selProc, text: el.value + '\n' })
    .then(() => { el.value = ''; })
    .catch((e) => toast(e.message, 'err'));
}

/* ------------------------------------------------------------------ log */
function renderLog() {
  $('#history').innerHTML = S.history.slice().reverse().map((h) => `
    <div class="logrow">
      <time>${new Date(h.ts * 1000).toLocaleTimeString()}</time>
      <div class="c">
        <div class="lab">${esc(h.label)}</div>
        <div class="cmdline">${esc(h.cmdline)}</div>
      </div>
      <button class="btn ghost sm" onclick="navigator.clipboard.writeText(this.parentNode.querySelector('.cmdline').textContent)">copy</button>
    </div>`).join('') || '<p class="hint">Nothing run yet. Every command will be listed here.</p>';
}

/* ====================================================== command palette */
/* One search box over everything the console can do: every catalog action,
 * every health check, every config file, every process, every tab. It is the
 * fastest path during a demo (no hunting through tabs) and it still runs the
 * SAME catalog action as the buttons -- including the confirm step, so Enter
 * can never quietly fly a drone. */
const PAL_TABS = [['dash', 'Dashboard'], ['health', 'System health'], ['control', 'Control'],
                  ['config', 'Config'], ['procs', 'Processes'], ['log', 'Command log']];

function palItems() {
  const out = [];
  S.catalog.forEach((a) => out.push({
    kind: a.group, label: a.label, desc: a.why, go: 'act:' + a.id, key: a.id,
    flag: a.danger === 'flight' ? 'drones move' : a.danger === 'estop' ? 'cuts motors' : '',
  }));
  ((S.health && S.health.nodes) || []).forEach((n) => out.push({
    kind: 'check', label: n.label, desc: n.summary || n.why, go: 'node:' + n.id, key: n.id,
  }));
  S.files.forEach((f) => out.push({
    kind: 'config', label: f.label, desc: f.path || '', go: 'config:' + f.key, key: f.key }));
  PAL_TABS.forEach(([k, l]) => out.push({ kind: 'tab', label: l, desc: 'go to the ' + l + ' tab', go: 'tab:' + k }));
  S.procs.slice(-12).reverse().forEach((p) => out.push({
    kind: 'process', label: p.label, desc: p.cmdline, go: 'proc:' + p.id }));
  return out;
}

/* A ranking ladder rather than a fuzzy-search dependency. The action id counts
 * as a search key: "fly" finds fly.script, which no word of its label contains.
 * The loose subsequence pass is last and scores low, so typing "fly" can never
 * put an unrelated action above the one actually named that. */
function palScore(item, q) {
  if (!q) return 1;
  const label = item.label.toLowerCase();
  const key = (item.key || '').toLowerCase();
  const desc = (item.desc || '').toLowerCase();
  if (label.startsWith(q)) return 1000;
  if (key.includes(q)) return 900;
  if (new RegExp('\\b' + q.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')).test(label)) return 850;
  if (label.includes(q)) return 800 - label.indexOf(q);
  if (item.kind.toLowerCase().startsWith(q)) return 700;
  if (desc.includes(q)) return 500;
  let i = 0;
  for (const ch of label) if (ch === q[i]) i += 1;
  return i === q.length ? 150 : 0;
}

function palRender() {
  const q = $('#pal-input').value.trim().toLowerCase();
  const rows = palItems().map((it) => ({ it, sc: palScore(it, q) }))
    .filter((r) => r.sc > 0)
    .sort((a, b) => b.sc - a.sc)
    .slice(0, 40).map((r) => r.it);
  S.palRows = rows;
  if (S.palSel >= rows.length) S.palSel = Math.max(0, rows.length - 1);
  $('#pal-rows').innerHTML = rows.length ? rows.map((it, i) => `
    <div class="palrow ${i === S.palSel ? 'sel' : ''}" data-i="${i}">
      <span class="pk">${esc(it.kind)}</span>
      <span class="pl">${esc(it.label)}</span>
      <span class="pd">${esc(clip(it.desc || '', 90))}</span>
      ${it.flag ? `<span class="pf">${esc(it.flag)}</span>` : ''}
    </div>`).join('') : '<div class="palempty">Nothing matches that.</div>';
  const sel = $('#pal-rows .palrow.sel');
  if (sel) sel.scrollIntoView({ block: 'nearest' });
  $$('#pal-rows .palrow').forEach((el) => {
    el.onmousemove = () => { if (S.palSel !== +el.dataset.i) { S.palSel = +el.dataset.i; palRender(); } };
    el.onclick = (e) => palChoose(e.shiftKey);
  });
}

function palChoose(shift) {
  const it = (S.palRows || [])[S.palSel];
  if (!it) return;
  closePalette();
  if (!it.go.startsWith('act:')) { go(it.go); return; }
  const id = it.go.slice(4);
  if (shift) { openCard(id); return; }
  const a = S.catalog.find((x) => x.id === id);
  if (!a) return;
  const values = {};
  a.params.forEach((q) => { values[q.name] = savedParam(a.id, q); });
  runAction(id, values, { stay: $('#tab-dash').classList.contains('active') });
}

function openPalette() {
  closeKeyHelp();
  S.palSel = 0;
  const el = $('#palette');
  el.hidden = false;
  const inp = $('#pal-input');
  inp.value = '';
  palRender();
  inp.focus();
}
function closePalette() { $('#palette').hidden = true; }

function wirePalette() {
  $('#pal-input').oninput = () => { S.palSel = 0; palRender(); };
  $('#pal-input').onkeydown = (e) => {
    if (e.key === 'ArrowDown' || (e.key === 'n' && e.ctrlKey)) {
      e.preventDefault(); S.palSel = Math.min((S.palRows || []).length - 1, S.palSel + 1); palRender();
    } else if (e.key === 'ArrowUp' || (e.key === 'p' && e.ctrlKey)) {
      e.preventDefault(); S.palSel = Math.max(0, S.palSel - 1); palRender();
    } else if (e.key === 'Enter') {
      e.preventDefault(); palChoose(e.shiftKey);
    } else if (e.key === 'Escape') {
      e.preventDefault(); closePalette();
    }
  };
  $('#palette').onclick = (e) => { if (e.target.id === 'palette') closePalette(); };
  $('#keyhelp').onclick = () => closeKeyHelp();
  $('#btn-palette').onclick = openPalette;
}

function openKeyHelp() { closePalette(); $('#keyhelp').hidden = false; }
function closeKeyHelp() { $('#keyhelp').hidden = true; }

/* ---- keyboard. Deliberately no key for E-STOP: a stray keypress must never
 * cut the motors, and mouse-only keeps it a decision. */
const typing = (el) => !!el && (/^(input|textarea|select)$/i.test(el.tagName) || el.isContentEditable);

document.addEventListener('keydown', (e) => {
  const palOpen = !$('#palette').hidden;
  if ((e.key === 'k' || e.key === 'K') && (e.metaKey || e.ctrlKey)) {
    e.preventDefault();
    if (palOpen) closePalette(); else openPalette();
    return;
  }
  if (palOpen) return;                       // the palette input owns the rest
  if (e.key === 'Escape') {
    if (!$('#keyhelp').hidden) closeKeyHelp();
    else if (!$('#modal').hidden) $('#modal-cancel').click();
    else if (typing(e.target)) e.target.blur();
    return;
  }
  if (typing(e.target) || e.metaKey || e.ctrlKey || e.altKey) return;
  if (!$('#modal').hidden) return;           // a confirm dialog is a decision, not a shortcut
  if (e.key === '/') { e.preventDefault(); openPalette(); }
  else if (e.key === '?') { e.preventDefault(); if ($('#keyhelp').hidden) openKeyHelp(); else closeKeyHelp(); }
  else if (e.key === 'r') { $('#btn-refresh').click(); }
  else if (/^[1-6]$/.test(e.key)) {
    const b = $$('#tabs button')[Number(e.key) - 1];
    if (b) b.click();
  }
});

boot().catch((e) => {
  document.body.innerHTML = `<pre class="out" style="margin:40px">console failed to start:\n\n${esc(e.message)}</pre>`;
});
