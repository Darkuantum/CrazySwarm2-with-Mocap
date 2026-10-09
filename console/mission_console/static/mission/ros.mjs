/* Live ROS data for the mission window, through foxglove_bridge.
 *
 * Why this and not the ros2 CLI the rest of the console uses: a 3D view needs
 * /poses at 50 Hz and marker arrays at 10 Hz, which is a stream, not a
 * command. launch.py already starts foxglove_bridge (foxglove:=True is the
 * default), so the browser subscribes to it directly -- no rclpy in the
 * console (the rule in CLAUDE.md stands) and no new node on the rig.
 *
 * The bridge sends each message as CDR bytes plus, once per topic, the
 * message definition as ros2msg text. This file parses that text and decodes
 * the bytes; it has no dependencies, so it works on the offline rig network.
 *
 * Protocol: foxglove_bridge 3.x (the Rust SDK) accepts only the
 * `foxglove.sdk.v1` subprotocol and refuses the classic `foxglove.websocket.v1`
 * handshake -- measured on this laptop, 2026-10-09, bridge 3.5.0. Older
 * bridges are the other way round. We offer both and let the server pick.
 *
 * Read-only by design: nothing here publishes or calls a service. Commands go
 * through the console backend, where they are logged and confirmed.
 */

const PRIM = {
  bool: [1, 'u8'], byte: [1, 'u8'], char: [1, 'u8'], octet: [1, 'u8'],
  int8: [1, 'i8'], uint8: [1, 'u8'], int16: [2, 'i16'], uint16: [2, 'u16'],
  int32: [4, 'i32'], uint32: [4, 'u32'], int64: [8, 'i64'], uint64: [8, 'u64'],
  float32: [4, 'f32'], float64: [8, 'f64'],
};
const ALIAS = { time: 'builtin_interfaces/Time', duration: 'builtin_interfaces/Duration',
  Header: 'std_msgs/Header' };
const BUILTIN = {
  'builtin_interfaces/Time': [{ name: 'sec', type: 'int32' }, { name: 'nanosec', type: 'uint32' }],
  'builtin_interfaces/Duration': [{ name: 'sec', type: 'int32' }, { name: 'nanosec', type: 'uint32' }],
};

const norm = (t) => t.replace('/msg/', '/');

/** ros2msg text (with ===/MSG: sections) -> { root, types: {name: fields[]} } */
export function parseSchema(rootName, text) {
  const types = { ...BUILTIN };
  const sections = text.split(/^=+\s*$/m);
  sections.forEach((sec) => {
    let name = norm(rootName);
    const lines = sec.split('\n');
    const fields = [];
    for (let raw of lines) {
      const m = /^\s*MSG:\s*(\S+)/.exec(raw);
      if (m) { name = norm(m[1]); continue; }
      raw = raw.replace(/#.*/, '').trim();
      if (!raw) continue;
      // constants: TYPE NAME=VALUE  (no field in the wire format)
      if (/^\S+\s+[A-Za-z_]\w*\s*=/.test(raw)) continue;
      const f = /^(\S+)\s+([A-Za-z_]\w*)/.exec(raw);
      if (!f) continue;
      fields.push({ name: f[2], ...parseType(f[1]) });
    }
    types[name] = fields;            // an empty message (std_msgs/Empty) is still a type
  });
  return { root: norm(rootName), types };
}

function parseType(tok) {
  let arr = null;
  const a = /^(.*)\[(<=)?(\d*)\]$/.exec(tok);
  if (a) {
    tok = a[1];
    arr = a[3] && !a[2] ? { fixed: +a[3] } : { seq: true };
  }
  tok = tok.replace(/<=\d+$/, '');            // bounded string: string<=10
  return { type: ALIAS[tok] || tok, arr };
}

/* ---------------------------------------------------------------- CDR */
class Reader {
  constructor(buf) {
    this.dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
    this.u8 = buf;
    // encapsulation: [0, kind, opt, opt]; kind odd = little endian.
    // kinds >= 6 are XCDR2, where 8-byte values align to 4.
    const kind = buf[1];
    this.le = (kind & 1) === 1;
    this.max = kind >= 6 ? 4 : 8;
    this.o = 4;
  }
  align(n) {
    n = Math.min(n, this.max);
    const r = (this.o - 4) % n;
    if (r) this.o += n - r;
  }
  prim(kind, size) {
    this.align(size);
    const dv = this.dv; const o = this.o; const le = this.le;
    this.o += size;
    switch (kind) {
      case 'u8': return dv.getUint8(o);
      case 'i8': return dv.getInt8(o);
      case 'i16': return dv.getInt16(o, le);
      case 'u16': return dv.getUint16(o, le);
      case 'i32': return dv.getInt32(o, le);
      case 'u32': return dv.getUint32(o, le);
      case 'i64': return Number(dv.getBigInt64(o, le));
      case 'u64': return Number(dv.getBigUint64(o, le));
      case 'f32': return dv.getFloat32(o, le);
      case 'f64': return dv.getFloat64(o, le);
      default: throw new Error('bad primitive ' + kind);
    }
  }
  string() {
    const n = this.prim('u32', 4);
    const s = n > 0 ? new TextDecoder().decode(this.u8.subarray(this.o, this.o + n - 1)) : '';
    this.o += n;
    return s;
  }
  wstring() {
    const n = this.prim('u32', 4);
    let s = '';
    for (let i = 0; i < n; i += 1) s += String.fromCodePoint(this.prim('u32', 4));
    return s;
  }
}

function readValue(r, type, types) {
  if (type === 'string') return r.string();
  if (type === 'wstring') return r.wstring();
  const p = PRIM[type];
  if (p) {
    const v = r.prim(p[1], p[0]);
    return type === 'bool' ? v !== 0 : v;
  }
  return readMsg(r, type, types);
}

function resolve(type, types) {
  if (types[type]) return types[type];
  // a bare name inside a package's own .msg: match on the last segment
  const k = Object.keys(types).find((t) => t.endsWith('/' + type));
  if (k) return types[k];
  throw new Error(`no definition for ${type}`);
}

function readMsg(r, type, types) {
  const out = {};
  for (const f of resolve(type, types)) {
    if (f.arr) {
      const n = f.arr.fixed != null ? f.arr.fixed : r.prim('u32', 4);
      const a = new Array(n);
      for (let i = 0; i < n; i += 1) a[i] = readValue(r, f.type, types);
      out[f.name] = a;
    } else {
      out[f.name] = readValue(r, f.type, types);
    }
  }
  return out;
}

export function makeDecoder(schemaName, schemaText) {
  const { root, types } = parseSchema(schemaName, schemaText);
  return (bytes) => readMsg(new Reader(bytes), root, types);
}

/* ------------------------------------------------------------- bridge */
const SUBPROTOCOLS = ['foxglove.sdk.v1', 'foxglove.websocket.v1'];

export class Bridge {
  constructor(url) {
    this.url = url;
    this.ws = null;
    this.state = 'idle';
    this.error = '';
    this.channels = new Map();      // channelId -> {topic, schemaName, decode, ...}
    this.byTopic = new Map();       // topic -> channelId
    this.wants = new Map();         // topic -> Set(callback)
    this.subs = new Map();          // subId -> {topic, channelId}
    this.subOfTopic = new Map();    // topic -> subId
    this.nextSub = 1;
    this.listeners = new Set();
    this.rx = 0;
    this.retry = 1000;
  }

  onChange(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); }
  emit() { this.listeners.forEach((fn) => { try { fn(this); } catch (e) { /* a view bug must not kill the link */ } }); }

  connect() {
    if (this.ws) return;
    this.state = 'connecting'; this.emit();
    let ws;
    try {
      ws = new WebSocket(this.url, SUBPROTOCOLS);
    } catch (e) {
      this.state = 'closed'; this.error = String(e); this.emit();
      setTimeout(() => this.connect(), 4000);
      return;
    }
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    ws.onopen = () => { this.state = 'open'; this.error = ''; this.retry = 1000; this.emit(); };
    ws.onclose = () => {
      this.ws = null;
      this.state = 'closed';
      this.channels.clear(); this.byTopic.clear(); this.subs.clear(); this.subOfTopic.clear();
      this.emit();
      // keep trying: the bridge comes and goes with the stack
      setTimeout(() => this.connect(), this.retry);
      this.retry = Math.min(this.retry * 1.6, 8000);
    };
    ws.onerror = () => { this.error = `no foxglove_bridge at ${this.url}`; };
    ws.onmessage = (ev) => {
      if (typeof ev.data === 'string') this.onText(JSON.parse(ev.data));
      else this.onBinary(new Uint8Array(ev.data));
    };
  }

  onText(msg) {
    if (msg.op === 'advertise') {
      for (const c of msg.channels) {
        this.channels.set(c.id, { ...c, decode: null });
        if (!this.byTopic.has(c.topic)) this.byTopic.set(c.topic, c.id);
        if (this.wants.has(c.topic)) this.subscribeChannel(c.topic);
      }
      this.emit();
    } else if (msg.op === 'unadvertise') {
      for (const id of msg.channelIds) {
        const c = this.channels.get(id);
        if (!c) continue;
        this.channels.delete(id);
        if (this.byTopic.get(c.topic) === id) this.byTopic.delete(c.topic);
        const sid = this.subOfTopic.get(c.topic);
        if (sid != null && this.subs.get(sid).channelId === id) {
          this.subs.delete(sid); this.subOfTopic.delete(c.topic);
        }
      }
      this.emit();
    } else if (msg.op === 'serverInfo') {
      this.server = msg; this.emit();
    }
  }

  onBinary(b) {
    if (b[0] !== 1) return;              // 1 = MessageData
    const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
    const sid = dv.getUint32(1, true);
    const sub = this.subs.get(sid);
    if (!sub) return;
    const c = this.channels.get(sub.channelId);
    if (!c) return;
    const payload = b.subarray(13);
    let msg;
    try {
      if (c.encoding === 'json') msg = JSON.parse(new TextDecoder().decode(payload));
      else {
        if (!c.decode) c.decode = makeDecoder(c.schemaName, c.schema);
        msg = c.decode(payload);
      }
    } catch (e) {
      c.decodeError = String(e);
      return;
    }
    this.rx += 1;
    const cbs = this.wants.get(c.topic);
    if (cbs) cbs.forEach((fn) => { try { fn(msg, c); } catch (e) { console.error(e); } });
  }

  subscribeChannel(topic) {
    if (this.subOfTopic.has(topic) || !this.ws || this.state !== 'open') return;
    const channelId = this.byTopic.get(topic);
    if (channelId == null) return;
    const id = this.nextSub++;
    this.subs.set(id, { topic, channelId });
    this.subOfTopic.set(topic, id);
    this.ws.send(JSON.stringify({ op: 'subscribe', subscriptions: [{ id, channelId }] }));
  }

  /** subscribe(topic, cb) -> unsubscribe(). Safe before the topic exists. */
  subscribe(topic, cb) {
    if (!this.wants.has(topic)) this.wants.set(topic, new Set());
    this.wants.get(topic).add(cb);
    this.subscribeChannel(topic);
    return () => {
      const set = this.wants.get(topic);
      if (!set) return;
      set.delete(cb);
      if (set.size) return;
      this.wants.delete(topic);
      const sid = this.subOfTopic.get(topic);
      if (sid != null && this.ws && this.state === 'open') {
        this.ws.send(JSON.stringify({ op: 'unsubscribe', subscriptionIds: [sid] }));
      }
      this.subs.delete(sid); this.subOfTopic.delete(topic);
    };
  }

  topics() {
    return [...this.channels.values()].map((c) => ({ topic: c.topic, type: c.schemaName }));
  }
}
