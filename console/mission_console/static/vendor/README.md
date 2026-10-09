# Vendored front-end libraries (mission window only)

Copied verbatim from the npm registry via jsDelivr on 2026-10-09 and committed,
because the rig laptop flies on the Motive network, which has no internet: a
CDN import would leave the mission window blank exactly when it is needed.
Nothing here is modified. To update, re-download the same paths at a new
version and keep the directory layout (the addons import each other by
relative path).

| path | package | version | licence |
|------|---------|---------|---------|
| `preact/htm-preact-standalone.mjs` | `htm/preact/standalone.module.js` (htm + Preact + hooks, one file) | htm 3.1.1 | Apache-2.0 (htm), MIT (Preact) |
| `three/three.module.min.js` | `three/build/three.module.min.js` | three 0.169.0 | MIT |
| `three/addons/controls/OrbitControls.js` | `three/examples/jsm/controls/` | three 0.169.0 | MIT |
| `three/addons/lines/*.js` | `three/examples/jsm/lines/` (Line2, LineSegments2, LineMaterial, ...) | three 0.169.0 | MIT |
| `three/addons/renderers/CSS2DRenderer.js` | `three/examples/jsm/renderers/` | three 0.169.0 | MIT |

The console's own pages (`index.html`, `app.js`) do not use any of this.
