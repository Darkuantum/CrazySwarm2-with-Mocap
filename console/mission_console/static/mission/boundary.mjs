/* What a mission window draws as its boundary -- shared by scene.mjs (3D) and
 * plan2d.mjs (the no-WebGL fallback) so the two can never disagree.
 *
 * The spec's `arena:` (normalised by missions._arena) is a list of entries:
 *   {kind: 'room'}                         arena.yaml, the measured room
 *   {kind: 'rect', rect: [x0, y0, x1, y1]} a rectangle the mission defines
 *   {kind: 'circle', circle: [cx, cy, r]}  a circle the mission defines
 * each with optional label / ceiling / color / dashed. An empty list draws no
 * boundary (grid and axes only). A spec without `arena:` gets [{kind:'room'}],
 * which is what every window drew before the key existed.
 *
 * resolveBoundaries() turns that into flat shapes the views draw:
 *   {kind:'room', cx, cy, rt, rl, ceil}
 *   {kind:'rect', x0, y0, x1, y1, ceil, label, color, dashed}
 *   {kind:'circle', cx, cy, r, ceil, label, color, dashed}
 * It enforces nothing -- like the rest of the view, it is a picture.
 */

export const DEFAULT_COLOR = '#d8a11d';       // = the room's radius_tested amber

export function roomShape(a) {
  const rt = (a && a.radius_tested) || 2.0;
  return {
    kind: 'room',
    cx: a && a.centre ? a.centre[0] : 0,
    cy: a && a.centre ? a.centre[1] : 0,
    rt,
    rl: (a && a.radius_lost) || rt + 0.25,
    ceil: (a && a.ceiling_tested) || 2.0,
  };
}

/* specArena: the mission's normalised `arena` (undefined = room).
 * room: arena.yaml's `arena` mapping from /api/arena (may be null). */
export function resolveBoundaries(specArena, room) {
  const entries = Array.isArray(specArena) ? specArena : [{ kind: 'room' }];
  const out = [];
  for (const e of entries) {
    const base = { label: e.label || '', color: e.color || DEFAULT_COLOR,
      dashed: !!e.dashed, ceil: e.ceiling || null };
    if (e.kind === 'room') out.push(roomShape(room));
    else if (e.kind === 'rect') {
      const [x0, y0, x1, y1] = e.rect;
      out.push({ ...base, kind: 'rect', x0, y0, x1, y1 });
    } else if (e.kind === 'circle') {
      const [cx, cy, r] = e.circle;
      out.push({ ...base, kind: 'circle', cx, cy, r });
    }
  }
  return out;
}

/* Accept either a shape list or (from an older custom view) a bare
 * arena.yaml mapping / null, which always meant "the room". */
export function asShapes(x) {
  return Array.isArray(x) ? x : [roomShape(x)];
}

/* Axis-aligned extent of everything drawn: {cx, cy, half} (half = the larger
 * half-width), used for the camera, the grid and the 2D scale. Nothing drawn
 * -> the origin and the 2.4 m span the view always opened at. */
export function boundsOf(shapes) {
  let x0 = Infinity; let y0 = Infinity; let x1 = -Infinity; let y1 = -Infinity;
  const add = (a, b, c, d) => { x0 = Math.min(x0, a); y0 = Math.min(y0, b); x1 = Math.max(x1, c); y1 = Math.max(y1, d); };
  for (const s of shapes) {
    if (s.kind === 'room') add(s.cx - s.rl, s.cy - s.rl, s.cx + s.rl, s.cy + s.rl);
    else if (s.kind === 'rect') add(s.x0, s.y0, s.x1, s.y1);
    else if (s.kind === 'circle') add(s.cx - s.r, s.cy - s.r, s.cx + s.r, s.cy + s.r);
  }
  if (!isFinite(x0)) return { cx: 0, cy: 0, half: 2.0 };
  return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, half: Math.max(x1 - x0, y1 - y0) / 2 };
}
