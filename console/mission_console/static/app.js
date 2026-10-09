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
  lines: {}, selNode: null, selGroup: null, selFile: 'crazyflies', factsSig: '', follow: true,
  cfg: {}, repo: '', env: {}, dashProc: null, checkedAt: 0, checkedFullAt: 0,
  missions: [], prefs: { pins: [] }, usage: null, step: null, stepAuto: null,
  settings: {}, journal: [], journalState: '', actView: 'procs', selOutside: null,
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

/* The usage log (mission_console/usage.py). Fire-and-forget: a lost beacon
 * costs one row of statistics, never an action. */
function usage(ev, extra) { post('/api/usage', { ev, ...(extra || {}) }).catch(() => {}); }

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
  S.repo = b.repo; S.env = b.env;
  S.missions = b.missions || []; S.prefs = b.prefs || { pins: [] }; S.usage = b.usage || null;
  S.settings = b.settings || {};
  S.journal = (b.journal && b.journal.events) || []; S.journalState = b.journal ? b.journal.state : '';
  if (!Array.isArray(S.prefs.pins)) S.prefs.pins = [];
  if (b.code_changed) markStale('console code changed since it started');
  S.selGroup = S.groups[0];
  // ?node=<id> selects a health box on load, so a failing check can be linked to.
  S.selNode = new URLSearchParams(location.search).get('node') || null;
  renderTabs(); renderHealth(); renderControl(); renderDash();
  renderProcs(); renderLog(); renderTop();
  S.booted = true;
  migrateLocalSettings();
  if (!location.search.includes("live=0")) events();
}

function events() {
  const src = new EventSource('/api/events');
  src.onmessage = (e) => {
    const ev = JSON.parse(e.data);
    if (ev.type === 'health') {
      S.health = ev.health; renderHealth(); renderTop();
      renderDashLive(); renderSession();
      const sig = JSON.stringify([ev.health.facts.server_running, ev.health.facts.enabled_drones]);
      if (sig !== S.factsSig) { S.factsSig = sig; renderControl(); }
    }
    else if (ev.type === 'proc') { upsertProc(ev.proc); renderProcs(); }
    else if (ev.type === 'line') { pushLine(ev); }
    else if (ev.type === 'history') { S.history.push(ev.entry); renderLog(); }
    else if (ev.type === 'settings') { S.settings = ev.settings || {}; renderSettingsChanged(); }
    else if (ev.type === 'fleet') { onFleetEvent(ev.event); }
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
      if (S.booted) usage('tab', { tab: b.dataset.tab });
      if (b.dataset.tab === 'log') loadUsage();
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
  wirePalette();
  $('#btn-keys').onclick = openKeyHelp;
  $('#btn-more').onclick = (e) => { e.stopPropagation(); if ($('#morepop').hidden) openMore(); else closeMore(); };
  $('#more-filter').oninput = renderMore;
  $('#more-filter').onkeydown = (e) => { if (e.key === 'Escape') closeMore(); };
  document.addEventListener('click', (e) => {
    if (!$('#morepop').hidden && !e.target.closest('#morepop') && !e.target.closest('#btn-more')) closeMore();
  });
  $('#btn-estop').onclick = fireEstop;
  $('#btn-scan').onclick = scanFleet;
  $('#dc-stdin').onsubmit = (e) => { e.preventDefault(); sendStdin(); };
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
  // Two cadences: a live sweep every few seconds (ROS graph, /poses, per-drone
  // telemetry) and a full one every 30 s. The stamp reports the live sweep --
  // which is the number you want when watching a drone -- and the tooltip says
  // when everything else was last verified, so it cannot quietly overstate.
  if (h.full && h.ts) S.checkedFullAt = h.ts;

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
  const ago = (t) => {
    const d = Math.max(0, Date.now() / 1000 - t);
    return d < 60 ? `${Math.round(d)} s ago` : `${Math.round(d / 60)} min ago`;
  };
  el.title = `live checks (ROS graph, /poses, each drone's battery, link and ` +
    `supervisor state) ran ${ago(S.checkedAt)}` +
    (S.checkedFullAt
      ? `\neverything else (workspace, config, radio, Motive) ran ${ago(S.checkedFullAt)}` +
        ' -- press Re-check to force a full sweep now'
      : '');
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
    { id: 'launch.mocap', label: 'Start mocap only' },
  ] },
  /* The sync reads /poses, so mocap must be UP -- and the server reads
   * crazyflies.yaml only when it starts, so it must start AFTER the sync.
   * Hence its own step between the two: mocap only, sync, then restart. */
  { title: 'Fleet positions', sub: 'mocap up, no server: sync, then restart', items: [
    { id: 'pos.sync_dry', label: 'Preview sync' },
    { id: 'pos.sync_apply', label: 'Apply sync' },
    { id: 'launch.restart', label: 'Restart with the server', params: ['backend'] },
  ] },
  { title: 'Bring it up', items: [
    { id: 'launch.stack', label: 'Start the stack', params: ['backend'] },
    { id: 'launch.restart', label: 'Restart', params: ['backend'] },
    { id: 'launch.stop', label: 'Stop the stack' },
  ] },
  { title: 'Check', items: [
    { id: 'check.poses_hz', label: 'Measure /poses' },
    { id: 'check.status_once', label: 'Drone status', params: ['drone'] },
    { id: 'check.services', label: '/all/* services up?' },
    { id: 'tool.script', label: 'Ground check', params: ['script'] },
  ] },
  { title: 'Fly', sub: 'missions open in their own window below', items: [
    { id: 'fly.script', label: 'Fly', params: ['script', 'sim'] },
    { id: 'srv.land_all', label: 'Land all' },
    { id: 'launch.stop', label: 'Stop the stack' },
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

/* ---- the single source of truth for every choice: settings.py, server-side.
 * A param's `key` says what the choice IS (`rviz`, `drone`, or
 * `<action>.<param>`), not which button showed it, so the Control card, the
 * dashboard row, a pin and the palette all read and write the same value --
 * and the backend applies it even to a request that did not carry it. Every
 * open page hears a change as an SSE `settings` event. (It was per-button
 * browser storage until 2026-10-09: rviz:=False on a Control card did not
 * reach the dashboard's Start the stack.) */
function settingKey(actionId, p) { return p.key || `${actionId}.${p.name}`; }

function savedParam(actionId, p) {
  const v = (S.settings || {})[settingKey(actionId, p)];
  if (v != null && (p.type !== 'select' || p.options.map(String).includes(String(v)))) return v;
  return p.default;
}
function saveParam(actionId, name, value) {
  const a = S.catalog.find((x) => x.id === actionId);
  const p = a && a.params.find((q) => q.name === name);
  const key = p ? settingKey(actionId, p) : `${actionId}.${name}`;
  S.settings[key] = value;
  post('/api/settings', { values: { [key]: value } }).catch((e) => toast(e.message, 'err'));
}

/* One-time move of the old per-button browser values into the server store.
 * Only keys the server does not have yet, so a choice made on another page or
 * browser since then is never overwritten. */
function migrateLocalSettings() {
  const moved = {};
  const drop = [];
  try {
    S.catalog.forEach((a) => a.params.forEach((p) => {
      const lk = `mc_p_${a.id}_${p.name}`;
      const v = localStorage.getItem(lk);
      if (v === null) return;
      drop.push(lk);
      const k = settingKey(a.id, p);
      if (S.settings[k] == null && moved[k] == null) moved[k] = v;
    }));
  } catch (e) { return; }
  if (!Object.keys(moved).length && !drop.length) return;
  post('/api/settings', { values: moved }).then((cur) => {
    S.settings = cur;
    try { drop.forEach((k) => localStorage.removeItem(k)); } catch (e) { /* ignore */ }
    renderSettingsChanged();
  }).catch(() => {});
}

/* A setting changed somewhere (this page, another tab, a mission window):
 * redraw what shows settings -- but never under the operator's cursor. */
function renderSettingsChanged() {
  const act = document.activeElement;
  const typing2 = act && /^(input|textarea|select)$/i.test(act.tagName);
  if (!(typing2 && act.closest('#dacts'))) { if ($('#dsteps')) $('#dsteps').dataset.sig = ''; renderSession(); }
  if (!(typing2 && act.closest('#actions'))) renderControl();
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
  S.dashProc = id;
  gotoTab('dash');
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
  else if (kind === 'mission') openMission(arg);
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

/* Rig processes the console did NOT start -- a terminal launch, a show run by
 * hand, a test stack on another ROS_DOMAIN_ID -- from the census
 * (facts.system.outside). One row per process group: a launch and its nodes
 * are one thing to stop. */
const ROLE_NAME = { launch: 'stack', server: 'server', mocap: 'mocap node', bridge: 'foxglove bridge',
  rviz: 'RViz', preflight: 'preflight GUI' };
function outsideGroups() {
  const out = ((S.health && S.health.facts && S.health.facts.system) || {}).outside || [];
  const groups = new Map();
  out.forEach((p) => {
    const g = groups.get(p.pgid) || { pgid: p.pgid, procs: [] };
    g.procs.push(p);
    groups.set(p.pgid, g);
  });
  const rank = { launch: 0, server: 1, mocap: 2, script: 3 };
  return [...groups.values()].map((g) => {
    g.procs.sort((a, b) => (rank[a.role] ?? 9) - (rank[b.role] ?? 9));
    const lead = g.procs[0];
    const what = lead.role === 'script' ? lead.exe
      : lead.role === 'launch' ? `${lead.backend || 'cpp'} stack${lead.server ? '' : ' (mocap only)'}`
        : ROLE_NAME[lead.role] || lead.role;
    return { ...g, lead, label: what, domain: lead.domain };
  });
}

function dashActivity() {
  if (S.actView === 'fleet') return fleetRows();
  const groups = outsideGroups();
  const myDom = ((S.health && S.health.facts && S.health.facts.system) || {}).my_domain;
  const outside = groups.map((g) => `<a class="actrow running outside ${S.selOutside === g.lead.pid ? 'sel' : ''}"
      data-outside="${g.lead.pid}" title="${esc(g.lead.cmdline)}">
      <span class="adot"></span><span class="alabel">${esc(clip(g.label, 26))}</span>
      <span class="astate">outside${g.domain && g.domain !== myDom ? ' &middot; dom ' + esc(g.domain) : ''}</span></a>`).join('');
  if (!S.procs.length && !outside) {
    return '<div class="hint" style="padding:8px">Nothing running, and nothing run yet.</div>';
  }
  const sel = S.selOutside ? null : dashProcId();
  return outside + S.procs.slice(-40).reverse().map((p) => `<a class="actrow ${esc(p.state)} ${p.id === sel ? 'sel' : ''}"
      data-dashproc="${esc(p.id)}" title="${esc(p.cmdline)}">
      <span class="adot"></span><span class="alabel">${esc(clip(p.label, 34))}</span>
      <span class="astate">${esc(p.state === 'running' ? 'live' : p.state)}${
        p.returncode != null && p.state !== 'running' ? ' ' + p.returncode : ''}</span></a>`).join('');
}

/* ---- the fleet-command journal (journal.py): what the SERVER acted on,
 * from anyone -- this console, the preflight GUI, a terminal, a show. */
function fleetRows() {
  const evs = S.journal.slice(-80).reverse();
  if (!evs.length) {
    return `<div class="hint" style="padding:8px">No fleet commands heard yet on ROS_DOMAIN_ID=${esc(S.env.domain_id)}.
      Listening with <code>ros2 topic echo /rosout --csv</code> (${esc(S.journalState || '?')}).</div>`;
  }
  return evs.map((e) => `<div class="fleetrow ${e.verb === 'emergency' ? 'estop' : ''} ${e.backlog ? 'old' : ''}"
      title="${esc(e.text)}${e.backlog ? '\n(from before this console started)' : ''}">
      <time>${new Date(e.ts * 1000).toLocaleTimeString([], { hour12: false })}</time>
      <b>${esc(e.target)}</b> <span>${esc(e.verb.replace('_', ' '))}</span>
      ${e.sim_noop ? '<em>sim: not implemented</em>' : ''}</div>`).join('');
}

function onFleetEvent(ev) {
  S.journal.push(ev);
  if (S.journal.length > 300) S.journal.splice(0, 100);
  if (ev.verb === 'emergency' && !ev.backlog) {
    const ours = S.ownEstopAt && Date.now() - S.ownEstopAt < 6000;
    const t = new Date(ev.ts * 1000).toLocaleTimeString([], { hour12: false });
    if (ev.sim_noop) {
      estopBanner('fail', `E-STOP at ${t} did NOTHING -- the simulator does not implement it`,
        `crazyflie_server logged "${ev.text}". The simulated drones keep flying: land them, or stop the script.`);
    } else if (!ours) {
      estopBanner('ok', `E-STOP fired outside this console at ${t}`,
        `crazyflie_server: "${ev.text}" -- from the preflight GUI, a terminal or a script. Motors are cut; ` +
        'the drones need a power-cycle before they arm again.');
    }
  }
  renderDashSoon();
}

/* the selected OUTSIDE process: what it is, and how to stop it -- no output,
 * the console did not start it and has no pty to read */
function outsideDetail(head, tail) {
  const g = outsideGroups().find((x) => x.lead.pid === S.selOutside);
  if (!g) { S.selOutside = null; return false; }
  const sig = `${g.lead.pid}|${g.procs.length}`;
  if (head.dataset.sig !== 'o' + sig) {
    head.dataset.sig = 'o' + sig;
    head.innerHTML = `<span class="nm">${esc(g.label)}</span><span class="st running">outside</span>
      <span class="cmd" title="${esc(g.lead.cmdline)}">${esc(g.lead.cmdline)}</span>
      <button class="btn ghost sm" data-osig="int" title="SIGINT to its process group: the same as Ctrl-C in its terminal">Stop</button>
      <button class="btn danger sm" data-osig="kill" title="SIGKILL to its process group">Kill</button>`;
    $$('[data-osig]', head).forEach((b) => { b.onclick = () => signalOutside(g, b.dataset.osig); });
  }
  tail.dataset.proc = '';
  const myDom = (S.health.facts.system || {}).my_domain;
  tail.innerHTML = `<div class="l-meta">Started OUTSIDE this console${g.domain !== myDom
      ? ` on ROS_DOMAIN_ID=${esc(g.domain)} -- this console is on ${esc(myDom)}, so its ROS graph, topics and
         services are invisible here; it still holds the machine's Crazyradio / UDP 1511` : ''}.
    Its output went to wherever it was started; the console can only see that it runs, and stop it.</div>
    ${g.procs.map((p) => `<div><span class="l-cmd">${esc(p.role)}</span> pid ${p.pid} (group ${p.pgid}),
      started ${new Date(p.started * 1000).toLocaleTimeString([], { hour12: false })}, domain ${esc(p.domain ?? '?')}
      <div class="l-meta">${esc(p.cmdline)}</div></div>`).join('')}`;
  return true;
}

async function signalOutside(g, mode) {
  const kill = mode === 'kill';
  const ok = await confirmRun(`${kill ? 'Kill' : 'Stop'} ${g.label} (started outside the console)`,
    `<p>${kill ? '<b>SIGKILL ends it with no clean-up.</b> A show will not land.'
      : g.lead.role === 'script'
        ? 'One SIGINT -- the same as Ctrl-C in its terminal. A show lands itself (abort.py); nothing follows it up.'
        : 'SIGINT, then SIGTERM after 6 s and SIGKILL after 10 s if it is still there -- a process started in the background ignores Ctrl-C.'}</p>
     <div class="cmdline">kill -${kill ? 'KILL' : 'INT'} -${g.pgid}</div>
     ${g.lead.role === 'launch' || g.lead.role === 'server' ? '<p class="warnline">If drones are flying, this removes their commander -- land first.</p>' : ''}`,
    kill ? 'Kill it' : 'Stop it');
  if (!ok) return;
  try {
    const r = await post('/api/signal', { pid: g.lead.pid, mode: kill ? 'kill' : 'int' });
    toast(`${r.signal} sent to process group ${r.pgid}`);
  } catch (e) { toast(e.message, 'err'); }
}

/* The output pane is the console's only process view (the Processes tab was
 * folded in here on 2026-10-09): stop, kill, copy, follow, and the stdin line.
 * New lines are APPENDED, so scrolling back through a long log is not undone
 * by the next line arriving. */
const OUT_MAX = 2000;
function dashOutput() {
  const head = $('#dashouthead'); const tail = $('#dashtail');
  if (!head || !tail) return;
  const form = $('#dc-stdin');
  if (S.selOutside && outsideDetail(head, tail)) { if (form) form.hidden = true; return; }
  const p = S.procs.find((x) => x.id === dashProcId());
  if (!p) {
    head.innerHTML = '';
    tail.dataset.proc = '';
    tail.innerHTML = '<div class="dc-empty"><span class="big">No output yet.</span>' +
      'Run anything above and it streams here &mdash; without leaving the dashboard.</div>';
    if (form) form.hidden = true;
    return;
  }
  const live = p.state === 'running' || p.state === 'stopping';
  const headSig = `${p.id}|${p.state}|${p.returncode}`;
  if (head.dataset.sig !== headSig) {
    head.dataset.sig = headSig;
    head.innerHTML = `<span class="nm">${esc(clip(p.label, 26))}</span>
      <span class="st ${esc(p.state)}">${esc(p.state)}${p.returncode != null && !live ? ' ' + p.returncode : ''}</span>
      <span class="cmd" title="${esc(p.cmdline)}">${esc(p.cmdline)}</span>
      ${live ? `<button class="btn ghost sm" onclick="window.__stop('${esc(p.id)}',false)"
          title="SIGINT to the process group, escalating to SIGTERM and SIGKILL (a flight script gets 15 s first)">Stop</button>
        <button class="btn danger sm" onclick="window.__stop('${esc(p.id)}',true)" title="SIGKILL now">Kill</button>` : ''}
      <button class="btn ghost sm" onclick="window.__copy('${esc(p.id)}')">Copy</button>
      <label class="chk" title="keep the newest line in view"><input type="checkbox" id="dc-follow" ${S.follow === false ? '' : 'checked'}> follow</label>`;
    const fl = $('#dc-follow');
    if (fl) fl.onchange = () => { S.follow = fl.checked; if (fl.checked) tail.scrollTop = tail.scrollHeight; };
  }
  if (form) form.hidden = !live;
  const rows = S.lines[p.id];
  if (!rows) { tail.dataset.proc = ''; loadProcLines(p.id); tail.innerHTML = '<span class="l-meta">loading output...</span>'; return; }
  const follow = S.follow !== false;
  const line = (l) => `<div class="${lineClass(l.text)}">${esc(l.text) || '&nbsp;'}</div>`;
  const lastSeq = Number(tail.dataset.seq || 0);
  if (tail.dataset.proc !== p.id || !rows.length || rows[0].seq > lastSeq + 1 && lastSeq) {
    tail.dataset.proc = p.id;
    tail.innerHTML = rows.slice(-OUT_MAX).map(line).join('') ||
      '<span class="l-meta">waiting for output...</span>';
  } else {
    const fresh = rows.filter((l) => l.seq > lastSeq);
    if (fresh.length) {
      if (!tail.querySelector('div')) tail.innerHTML = '';
      tail.insertAdjacentHTML('beforeend', fresh.map(line).join(''));
      while (tail.childElementCount > OUT_MAX) tail.firstElementChild.remove();
    }
  }
  tail.dataset.seq = rows.length ? rows[rows.length - 1].seq : 0;
  if (p.partial) {
    let pr = $('.dc-partial', tail);
    if (!pr) { pr = document.createElement('div'); pr.className = 'dc-partial l-gate'; tail.appendChild(pr); }
    pr.textContent = p.partial;
  } else { const pr = $('.dc-partial', tail); if (pr) pr.remove(); }
  if (follow) tail.scrollTop = tail.scrollHeight;
}

function dashBlocked(a, f) {
  return blockedWhy(a.requires, f);
}
/* One sentence per precondition, used by the dashboard, the cards and the
 * confirm step alike. */
function blockedWhy(req, f) {
  // The radio and UDP 1511 belong to the MACHINE, not to a ROS domain: a
  // server started in a terminal, or on another ROS_DOMAIN_ID, owns them just
  // the same. `*_any` comes from the process census (system.py).
  const sys = f.system || {};
  const st = sys.stack;
  const where = st ? (st.by === 'outside' ? ' (started outside this console' +
    (st.domain && st.domain !== sys.my_domain ? `, on ROS_DOMAIN_ID=${st.domain}` : '') + ')' : '') : '';
  if (req === 'server_running' && !f.server_running) {
    return f.server_any ? `the running server is on ROS_DOMAIN_ID=${st && st.domain} -- this console cannot reach it`
      : 'needs the server running';
  }
  if (req === 'server_stopped' && (f.server_any || f.server_running)) return 'a server owns the radio' + where + ' -- stop it first';
  if (req === 'mocap_running' && !f.mocap_running) return 'needs mocap up -- "Start mocap only" first';
  if (req === 'stack_stopped' && (f.stack_any || f.server_running || f.mocap_running)) {
    return (f.server_any || f.server_running)
      ? 'a stack is already running' + where + ' -- stop it, or use "Restart with the server"'
      : 'mocap-only is running' + where + ' -- use "Restart with the server" (a second launch would start a second mocap node on UDP 1511)';
  }
  return '';
}

function renderDashLive() {
  if (!$('#tab-dash').classList.contains('active')) return;
  $('#dashstrip').innerHTML = dashStrip();
  $('#dashissues').innerHTML = dashIssues();
  $('#dashactivity').innerHTML = dashActivity();
  dashOutput();
  renderBlocked();
  renderMissions();
}

let dashTimer = null;
function renderDashSoon() {
  if (dashTimer) return;
  dashTimer = setTimeout(() => { dashTimer = null; renderDashLive(); }, 250);
}

/* Clicking a row in Activity keeps you on the dashboard and shows its output
 * beside it; "full output ->" is there for when you want the whole pane. */
document.addEventListener('click', (e) => {
  const view = e.target.closest('[data-actview]');
  if (view) {
    S.actView = view.dataset.actview;
    $$('[data-actview]').forEach((b) => b.classList.toggle('on', b === view));
    renderDashLive();
    return;
  }
  const out = e.target.closest('[data-outside]');
  if (out) { S.selOutside = Number(out.dataset.outside); $('#dashtail').dataset.proc = ''; renderDashLive(); return; }
  const el = e.target.closest('[data-dashproc]');
  if (!el) return;
  S.selOutside = null;
  $('#dashouthead').dataset.sig = '';
  S.dashProc = el.dataset.dashproc;
  renderDashLive();
});

/* ---- structure: ONE step at a time.
 *
 * The four steps used to be four columns of buttons on screen at once --
 * twelve buttons and six dropdowns, most of them for a step the session was
 * not in, squeezing the live output to a few lines. Now a stepper shows where
 * the session is and the row under it holds that step's buttons only. The
 * stepper follows the rig (server down -> scan / bring it up; server up ->
 * check / fly) until you pick a step yourself; your pick holds until the rig
 * changes state. Pinned shortcuts sit beside the step; everything else is in
 * More, ranked by the usage log. */
function autoStep() {
  const f = (S.health && S.health.facts) || {};
  const node = (id) => ((S.health && S.health.nodes) || []).find((n) => n.id === id);
  if (!f.server_running) {
    if (f.mocap_running) return 1;               // mocap only: the sync step
    // nothing up: stay here -- "Start mocap only" (the route with the sync)
    // is in this step; "Bring it up" is one click for a launch without a sync
    return 0;
  }
  const srv = node('server.services');
  return srv && srv.status === 'ok' ? 4 : 3;
}
function currentStep() {
  const auto = autoStep();
  if (S.step != null && S.stepAuto === auto) return S.step;
  S.step = null;                       // the rig moved on: follow it again
  return auto;
}

function qaHtml(it, a) {
  const shown = (it.params || []).map((n) => a.params.find((q) => q.name === n)).filter(Boolean);
  const sel = shown.map((q) => {
    const opts = optionsHtml(q, savedParam(a.id, q));
    return `<select data-param="${esc(q.name)}" title="${esc(q.label + (q.help ? ' -- ' + q.help : ''))}">${opts}</select>`;
  }).join('');
  const kind = a.danger === 'flight' ? 'flight' : '';
  return `<div class="qa" data-qa="${esc(a.id)}">
    <button class="btn sm qa-run ${kind}" data-run>${esc(it.label)}</button>${sel}
    <a class="qa-more" data-go="card:${esc(a.id)}" title="Full card: every option, the exact command and how to read it">&#8943;</a>
    </div>`;
}

function wireQa(el, a, source) {
  const values = () => {
    const v = {};
    $$('[data-param]', el).forEach((s) => { v[s.dataset.param] = s.value; });
    return v;
  };
  const run = $('[data-run]', el);
  const refresh = async () => {
    const v = values();
    const q = a.params.find((x) => Object.keys(x.descriptions || {}).length);
    const desc = (q && q.descriptions[v[q.name]]) || '';
    try {
      const r = await post('/api/preview', { action_id: a.id, values: v });
      run.title = `${r.cmdline}\n\n${desc ? desc + '\n\n' : ''}${a.why}`;
    } catch (e) { run.title = a.why; }
  };
  $$('[data-param]', el).forEach((s) => {
    s.onchange = () => {
      saveParam(a.id, s.dataset.param, s.value);
      // the same setting may be on another button in this row (Start / Restart)
      $$('#dacts .qa').forEach((other) => {
        if (other === el) return;
        const oa = S.catalog.find((x) => x.id === other.dataset.qa);
        const q = oa && oa.params.find((x) => x.name === s.dataset.param);
        const me = a.params.find((x) => x.name === s.dataset.param);
        const os = $(`[data-param="${CSS.escape(s.dataset.param)}"]`, other);
        if (q && me && os && settingKey(oa.id, q) === settingKey(a.id, me)) os.value = s.value;
      });
      refresh();
    };
  });
  run.onclick = () => runAction(a.id, values(), { stay: true, source });
  refresh();
}

function renderSession() {
  if (!$('#dsteps')) return;
  const byId = Object.fromEntries(S.catalog.map((a) => [a.id, a]));
  const cur = currentStep();
  const auto = autoStep();
  if ($('#dsteps').dataset.sig !== `${cur}|${auto}`) {
    $('#dsteps').dataset.sig = `${cur}|${auto}`;
    $('#dsteps').innerHTML = DASH.map((c, i) => `<button role="tab" aria-selected="${i === cur}"
        class="dstep ${i === cur ? 'on' : ''} ${i < auto ? 'past' : ''} ${i === auto ? 'now' : ''}" data-step="${i}"
        title="${esc(c.title + (c.sub ? ' -- ' + c.sub : '') + (i === auto ? '\n(where the rig is now)' : ''))}">
        <span class="n">${i + 1}</span><span class="t">${esc(c.title)}</span></button>`).join('');
    $$('#dsteps [data-step]').forEach((b) => {
      b.onclick = () => {
        const i = Number(b.dataset.step);
        S.step = i === autoStep() ? null : i;
        S.stepAuto = autoStep();
        usage('step', { step: DASH[i].title });
        renderSession();
      };
    });
    const items = DASH[cur].items.filter((it) => byId[it.id]);
    $('#dacts').innerHTML = items.map((it) => qaHtml(it, byId[it.id])).join('') +
      '<span class="qa-why dacts-why"></span>';
    $$('#dacts .qa').forEach((el, i) => wireQa(el, byId[items[i].id], 'dash'));
  }
  renderPins();
  renderBlocked();
}

/* every button in the row that cannot work right now says why -- once */
function renderBlocked() {
  const f = (S.health && S.health.facts) || {};
  const els = $$('#dacts .qa, #dpins .pin');
  const whys = els.map((el) => {
    const a = S.catalog.find((x) => x.id === (el.dataset.qa || el.dataset.pin));
    return a ? dashBlocked(a, f) : '';
  });
  els.forEach((el, i) => {
    el.classList.toggle('blocked', !!whys[i]);
    const b = el.matches('.qa') ? $('[data-run]', el) : el;
    if (b && whys[i]) b.dataset.why = whys[i]; else if (b) delete b.dataset.why;
  });
  const qaWhys = whys.slice(0, $$('#dacts .qa').length).filter(Boolean);
  const why = $('.dacts-why');
  if (why) why.textContent = qaWhys.length ? [...new Set(qaWhys)].join('; ') : '';
}

/* ---- pins: the operator's own shortcuts, kept in console/usage/dashboard.json */
function savePins() {
  post('/api/prefs', { pins: S.prefs.pins }).then((p) => { S.prefs = p; }).catch((e) => toast(e.message, 'err'));
}
function togglePin(id) {
  const i = S.prefs.pins.indexOf(id);
  if (i >= 0) S.prefs.pins.splice(i, 1); else S.prefs.pins.push(id);
  savePins(); renderPins(); renderMore();
}
function renderPins() {
  const el = $('#dpins');
  if (!el) return;
  const onRow = new Set(DASH[currentStep()].items.map((it) => it.id));
  const pins = S.prefs.pins.map((id) => S.catalog.find((a) => a.id === id))
    .filter((a) => a && !onRow.has(a.id));
  el.innerHTML = pins.length
    ? '<span class="dpins-label">pinned</span>' + pins.map((a) => `<button class="btn sm pin ${a.danger === 'flight' ? 'flight' : ''}"
        data-pin="${esc(a.id)}" title="${esc(a.label + ' -- ' + a.why)}">${esc(a.label)}</button>`).join('')
    : '';
  $$('#dpins [data-pin]').forEach((b) => {
    b.onclick = () => {
      const a = S.catalog.find((x) => x.id === b.dataset.pin);
      const values = {};
      a.params.forEach((q) => { values[q.name] = savedParam(a.id, q); });
      runAction(a.id, values, { stay: true, source: 'pin' });
    };
  });
  renderBlocked();
}

/* ---- More: every action, the ones you actually use first */
function usageCount(id) {
  const u = S.usage && S.usage.actions && S.usage.actions.find((x) => x.id === id);
  return u ? u.count : 0;
}
function renderMore() {
  const rows = $('#morerows');
  if (!rows || $('#morepop').hidden) return;
  const q = ($('#more-filter').value || '').trim().toLowerCase();
  const onRow = new Set(DASH[currentStep()].items.map((it) => it.id));
  const acts = S.catalog.filter((a) => a.danger !== 'estop')
    .filter((a) => !q || (a.label + ' ' + a.id + ' ' + a.group).toLowerCase().includes(q))
    .map((a) => ({ a, n: usageCount(a.id) }))
    .sort((x, y) => (y.n - x.n) || S.groups.indexOf(x.a.group) - S.groups.indexOf(y.a.group));
  const days = S.usage && S.usage.since ? Math.max(1, Math.round((Date.now() / 1000 - S.usage.since) / 86400)) : 0;
  $('#more-note').textContent = S.usage && S.usage.actions && S.usage.actions.length
    ? `ranked by your use over ${days ? days + ' day' + (days > 1 ? 's' : '') : 'today'}`
    : 'no usage recorded yet -- the order learns as you work';
  rows.innerHTML = acts.map(({ a, n }) => {
    const pinned = S.prefs.pins.includes(a.id);
    return `<div class="morerow ${onRow.has(a.id) ? 'onrow' : ''}" data-act="${esc(a.id)}">
      <button class="star ${pinned ? 'on' : ''}" data-star="${esc(a.id)}" title="${pinned ? 'Unpin' : 'Pin to the dashboard'}">${pinned ? '&#9733;' : '&#9734;'}</button>
      <span class="mg">${esc(a.group)}</span>
      <span class="ml">${esc(a.label)}${a.danger === 'flight' ? ' <span class="tag flight">drones move</span>' : ''}</span>
      <span class="mn" title="times run">${n ? n + '&times;' : ''}</span>
      <a class="qa-more" data-go="card:${esc(a.id)}" title="open the full card">&#8943;</a></div>`;
  }).join('') || '<div class="hint" style="padding:10px">Nothing matches.</div>';
  $$('#morerows [data-star]').forEach((b) => { b.onclick = (e) => { e.stopPropagation(); togglePin(b.dataset.star); }; });
  $$('#morerows .morerow').forEach((r) => {
    r.onclick = (e) => {
      if (e.target.closest('[data-go],[data-star]')) { closeMore(); return; }
      const a = S.catalog.find((x) => x.id === r.dataset.act);
      const values = {};
      a.params.forEach((q2) => { values[q2.name] = savedParam(a.id, q2); });
      closeMore();
      runAction(a.id, values, { stay: true, source: 'more' });
    };
  });
}
function openMore() {
  const pop = $('#morepop');
  const r = $('#btn-more').getBoundingClientRect();
  pop.hidden = false;
  pop.style.top = `${Math.round(r.bottom + 6)}px`;
  pop.style.right = `${Math.max(8, Math.round(window.innerWidth - r.right))}px`;
  $('#more-filter').value = '';
  usage('more');
  renderMore();
  $('#more-filter').focus();
}
function closeMore() { $('#morepop').hidden = true; }

/* ---- missions: a window per mission, defined by the show package */
function openMission(id) {
  window.open('/mission?m=' + encodeURIComponent(id), 'mission-' + id.replace(/\W/g, '_'));
}
function renderMissions() {
  const el = $('#dmissions');
  if (!el) return;
  const running = new Set(S.procs.filter((p) => p.state === 'running' || p.state === 'stopping')
    .map((p) => (p.action_id.match(/^mission:(.+):[^:]+$/) || [])[1]).filter(Boolean));
  const sig = JSON.stringify([S.missions.map((m) => [m.id, m.error, m.build && m.build.ok]), [...running]]);
  if (el.dataset.sig === sig) return;
  el.dataset.sig = sig;
  el.innerHTML = `<span class="dm-label" title="Each one is missions/<name>.yaml in a show package (docs/MISSIONS.md)">Missions</span>` +
    S.missions.map((m) => {
      const nb = m.build && !m.build.ok;
      const why = nb ? (m.build.problems.length ? 'NOT BUILT: ' + m.build.problems.join('; ') + '\nfix: ' + m.build.build_cmd
        : 'built after this console started -- restart ./console/run.sh') + '\n\n' : '';
      return `<button class="mchip ${m.error ? 'bad' : ''} ${nb ? 'unbuilt' : ''} ${running.has(m.id) ? 'live' : ''} ${m.builtin ? 'generic' : ''}"
      data-mission="${esc(m.id)}" ${m.error ? 'disabled' : ''}
      title="${esc(m.error ? 'cannot load: ' + m.error : why + (m.summary || m.title) + '\n\nopens in its own window')}">
      ${running.has(m.id) ? '<span class="livedot"></span>' : ''}${esc(m.title)}${nb ? '<span class="nb">not built</span>' : ''}<span class="ext">&#8599;</span></button>`;
    }).join('') +
    `<a class="hlink dm-all" href="/mission" target="mission-picker">all</a>`;
  $$('#dmissions [data-mission]').forEach((b) => { b.onclick = () => openMission(b.dataset.mission); });
}

function renderDash() {
  $('#dsteps').dataset.sig = '';
  renderSession();
  renderMissions();
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
  } else if (blockedWhy(a.requires, f)) {
    warn = blockedWhy(a.requires, f) + '.';
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
  $('[data-run]', card).onclick = () => runAction(a.id, cardValues(card), { source: 'card' });
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
  if (po) po.onclick = () => openProc(po.dataset.ebProc);
  $('[data-eb-close]', b).onclick = () => { b.hidden = true; };
}

let estopBusy = false;
async function fireEstop() {
  if (estopBusy) return;          // a second click must not queue a second call
  estopBusy = true;
  S.ownEstopAt = Date.now();      // so the journal's echo of it is not called "outside"
  const btn = $('#btn-estop');
  btn.classList.add('firing');
  estopBanner('pending', 'E-STOP sent', 'waiting for the crazyflie_server to confirm...');
  try {
    let r;
    try {
      r = await post('/api/estop', { source: 'console' });
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
  if (['mocap_running', 'stack_stopped'].includes(a.requires) && blockedWhy(a.requires, f)) {
    notes.push(`<p class="warnline">${esc(blockedWhy(a.requires, f))}.</p>`);
  }
  if (a.danger !== 'none' || notes.length) {
    const ok = await confirmRun(a.label,
      `${notes.join('')}<p>This runs:</p><div class="cmdline">${esc(cmd)}</div>
       <p class="hint">Working directory: ${esc(S.repo)}</p>`,
      a.danger === 'estop' ? 'CUT THE MOTORS' : 'Run it');
    if (!ok) return;
  }
  try {
    const p = await post('/api/run', { action_id: actionId, values,
      source: opts.source || ($('#tab-control').classList.contains('active') ? 'card' : 'other') });
    upsertProc(p); S.dashProc = p.id; S.lines[p.id] = S.lines[p.id] || [];
    bumpUsage(actionId);
    renderProcs();
    toast('started: ' + p.cmdline.slice(0, 70));
    if (opts.stay) renderDashLive(); else gotoTab('dash');      // its output is on the dashboard
  } catch (e) { toast(e.message, 'err'); }
}

/* keep the More ranking current without refetching the whole log */
function bumpUsage(id) {
  if (!S.usage) S.usage = { actions: [], since: Date.now() / 1000 };
  if (!S.usage.since) S.usage.since = Date.now() / 1000;
  const u = S.usage.actions.find((x) => x.id === id);
  if (u) u.count += 1; else S.usage.actions.push({ id, count: 1, last: Date.now() / 1000 });
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
async function refreshMissions() {
  try { S.missions = (await api('/api/missions')).missions; renderMissions(); } catch (e) { /* next sweep */ }
}

function upsertProc(p) {
  const prev = S.procs.find((x) => x.id === p.id);
  if (p.action_id === 'build.workspace' && prev && prev.state === 'running' && p.state !== 'running') refreshMissions();
  const i = S.procs.findIndex((x) => x.id === p.id);
  if (i >= 0) S.procs[i] = p; else S.procs.push(p);
  estopLate(p);
  renderProcBadge();
  renderDashSoon();
}

function pushLine(ev) {
  (S.lines[ev.proc] = S.lines[ev.proc] || []).push({ seq: ev.seq, text: ev.text });
  if (S.lines[ev.proc].length > 3000) S.lines[ev.proc].splice(0, 500);
  const p = S.procs.find((x) => x.id === ev.proc);
  if (p && p.partial) p.partial = '';
  renderDashSoon();
}

function lineClass(t) {
  if (/^\$ /.test(t)) return 'l-cmd';
  if (/^\[(console|exit)/.test(t)) return 'l-meta';
  if (/\b(error|ERROR|Traceback|FATAL|failed|refused|cannot)\b/.test(t)) return 'l-err';
  if (/\b(warn|WARN|WARNING)\b/.test(t)) return 'l-warn';
  if (/^\s*>>>/.test(t)) return 'l-gate';
  return '';
}

/* the count of live processes rides on the Dashboard tab */
function renderProcBadge() {
  const live = S.procs.filter((p) => p.state === 'running' || p.state === 'stopping').length;
  $('#proc-badge').textContent = live || '';
  document.body.classList.toggle('busy', live > 0);
}
function renderProcs() { renderProcBadge(); renderDashSoon(); }

async function loadProcLines(id) {
  try {
    const r = await api('/api/proc/' + id);
    S.lines[id] = r.lines.map((l) => ({ seq: l.seq, text: l.text }));
    const i = S.procs.findIndex((x) => x.id === id);
    if (i >= 0) S.procs[i] = r.proc;
    renderDashSoon();
  } catch (e) { /* the process may have been pruned */ }
}

window.__stop = async (id, hard) => {
  try { await post('/api/stop', { proc: id, hard, source: 'dash' }); toast(hard ? 'SIGKILL sent' : 'SIGINT sent'); }
  catch (e) { toast(e.message, 'err'); }
};
window.__copy = (id) => {
  const p = S.procs.find((x) => x.id === id);
  if (p) navigator.clipboard.writeText(p.cmdline).then(() => toast('command copied'));
};

function sendStdin() {
  const el = $('#stdin');
  /* An EMPTY line is a legitimate message: a paced show gates on a bare
     Enter, so refusing to send one made every gate unreachable from here and
     operators typed a stray letter to get past it. Only a missing process is
     a reason not to send. */
  const id = dashProcId();
  if (!id) return;
  post('/api/input', { proc: id, text: el.value + '\n', source: 'dash' })
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

/* ----------------------------------------------------------- usage view */
/* What the operator actually uses, across sessions: the data the dashboard's
 * layout is supposed to follow. Read from console/usage/usage.jsonl. */
async function loadUsage() {
  try { S.usage = await api('/api/usage'); } catch (e) { return; }
  renderUsage();
}
function renderUsage() {
  const u = S.usage;
  if (!u || !$('#usagebody')) return;
  const ago = (t) => {
    const d = Date.now() / 1000 - t;
    return d < 3600 ? `${Math.max(1, Math.round(d / 60))} min ago` : d < 86400 ? `${Math.round(d / 3600)} h ago` : `${Math.round(d / 86400)} d ago`;
  };
  const total = (u.actions || []).reduce((n, a) => n + a.count, 0);
  $('#usage-sub').textContent = u.since
    ? `${total} runs in ${u.sessions} session${u.sessions === 1 ? '' : 's'} since ${new Date(u.since * 1000).toLocaleDateString()}`
    : 'nothing recorded yet';
  const max = Math.max(1, ...(u.actions || []).map((a) => a.count));
  const srcs = (o) => Object.entries(o || {}).map(([k, v]) => `${esc(k)} ${v}`).join(', ');
  const tabs = Object.entries(u.tabs || {}).map(([k, v]) => `<span class="utag">${esc(k)} <b>${v}</b></span>`).join('');
  const missions = Object.entries(u.missions || {}).map(([k, v]) =>
    `<span class="utag">${esc(k)} <b>${v.opens}</b> opened, <b>${v.starts}</b> started</span>`).join('');
  $('#usagebody').innerHTML = `
    <p class="hint">Every run, stop, e-stop, tab and mission, appended to <code>${esc(u.log || 'console/usage/usage.jsonl')}</code>
      (git-ignored; stdin is logged only as enter / q / text, never its content). The dashboard's More menu is
      ranked from this. Read it from a shell, or ask Claude to.</p>
    <table class="utable"><thead><tr><th>action</th><th>runs</th><th></th><th>started from</th><th>last</th></tr></thead><tbody>
    ${(u.actions || []).slice(0, 30).map((a) => `<tr><td>${esc(a.label)}</td><td class="num">${a.count}</td>
      <td class="ubar"><i style="width:${Math.round(a.count / max * 100)}%"></i></td>
      <td class="hint">${srcs(a.sources)}</td><td class="hint">${ago(a.last)}</td></tr>`).join('') ||
      '<tr><td colspan="5" class="hint">No runs recorded yet.</td></tr>'}
    </tbody></table>
    ${tabs ? `<div class="urow"><b>tabs opened</b> ${tabs}</div>` : ''}
    ${missions ? `<div class="urow"><b>missions</b> ${missions}</div>` : ''}`;
}

/* ====================================================== command palette */
/* One search box over everything the console can do: every catalog action,
 * every health check, every config file, every process, every tab. It is the
 * fastest path during a demo (no hunting through tabs) and it still runs the
 * SAME catalog action as the buttons -- including the confirm step, so Enter
 * can never quietly fly a drone. */
const PAL_TABS = [['dash', 'Dashboard'], ['health', 'System health'], ['control', 'Control'],
                  ['config', 'Config'], ['log', 'Command log']];

function palItems() {
  const out = [];
  S.catalog.forEach((a) => out.push({
    kind: a.group, label: a.label, desc: a.why, go: 'act:' + a.id, key: a.id,
    flag: a.danger === 'flight' ? 'drones move' : a.danger === 'estop' ? 'cuts motors' : '',
  }));
  ((S.health && S.health.nodes) || []).forEach((n) => out.push({
    kind: 'check', label: n.label, desc: n.summary || n.why, go: 'node:' + n.id, key: n.id,
  }));
  S.missions.filter((m) => !m.error).forEach((m) => out.push({
    kind: 'mission', label: m.title, desc: 'open the mission window -- ' + (m.summary || ''), go: 'mission:' + m.id, key: m.id }));
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
  usage('palette', { chose: it.go });
  if (!it.go.startsWith('act:')) { go(it.go); return; }
  const id = it.go.slice(4);
  if (shift) { openCard(id); return; }
  const a = S.catalog.find((x) => x.id === id);
  if (!a) return;
  const values = {};
  a.params.forEach((q) => { values[q.name] = savedParam(a.id, q); });
  runAction(id, values, { stay: $('#tab-dash').classList.contains('active'), source: 'palette' });
}

function openPalette() {
  closeKeyHelp();
  usage('palette', { opened: true });
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
    if (!$('#morepop').hidden) closeMore();
    else if (!$('#keyhelp').hidden) closeKeyHelp();
    else if (!$('#modal').hidden) $('#modal-cancel').click();
    else if (typing(e.target)) e.target.blur();
    return;
  }
  if (typing(e.target) || e.metaKey || e.ctrlKey || e.altKey) return;
  if (!$('#modal').hidden) return;           // a confirm dialog is a decision, not a shortcut
  if (e.key === '/') { e.preventDefault(); openPalette(); }
  else if (e.key === '?') { e.preventDefault(); if ($('#keyhelp').hidden) openKeyHelp(); else closeKeyHelp(); }
  else if (e.key === 'r') { $('#btn-refresh').click(); }
  else if (/^[1-5]$/.test(e.key)) {
    const b = $$('#tabs button')[Number(e.key) - 1];
    if (b) b.click();
  }
});

boot().catch((e) => {
  document.body.innerHTML = `<pre class="out" style="margin:40px">console failed to start:\n\n${esc(e.message)}</pre>`;
});
