// Chart data assembly for the GNC console. Pure module (no DOM, no uPlot).
// A chart spec (see regimes.js) lists series by dictionary parameter path; x is onboard time [s].
import { PARAM_APID } from './telemetryStore.js';
import { createAligner } from './decimate.js';

export const ENGINE_COLORS = ['#35d7ff', '#f0b030', '#4ccf6a'];

// Expand series marked perEngine ("{e}" in the param) for the selected engine (0..2 or 'all').
// With 'all' (or allEngines on the series) they take the engine color; with one engine they keep
// their own color.
export function expandSeries(series, engineSel) {
  const out = [];
  for (const s of series) {
    if (!s.perEngine) { out.push(s); continue; }
    const all = s.allEngines || engineSel === 'all';
    for (const e of all ? [0, 1, 2] : [engineSel]) {
      out.push({ ...s, param: s.param.replace('{e}', e), label: 'E' + (e + 1) + ' ' + s.label, color: all ? ENGINE_COLORS[e] : s.color });
    }
  }
  return out;
}

// Build [x, y...] for a series list from the store, aligning across APIDs on onboard time.
// Typed-array views into store scratch: consume before the next store.series call on the same APID.
const sharedAlign = createAligner();
export function buildData(store, series, tMin, align = sharedAlign) {
  const byApid = new Map();
  series.forEach((s, i) => {
    const a = PARAM_APID[s.param];
    if (!byApid.has(a)) byApid.set(a, []);
    byApid.get(a).push(i);
  });
  const tables = [], order = [];
  for (const [, idxs] of byApid) {
    tables.push(store.series(idxs.map((i) => series[i].param), tMin));
    order.push(idxs);
  }
  if (tables.length === 1) return tables[0];
  const joined = align(tables);
  // joined is [x, ...columns in table order]; restore series order.
  const out = [joined[0]];
  let col = 1;
  const placed = new Array(series.length);
  for (const idxs of order) for (const i of idxs) placed[i] = joined[col++];
  for (let i = 0; i < placed.length; i++) out.push(placed[i]);
  return out;
}
