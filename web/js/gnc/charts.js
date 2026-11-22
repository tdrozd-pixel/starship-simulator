// uPlot charts for the GNC console. x is onboard time [s]; every chart shows the same window
// ending at the newest onboard time. DOM-bound.
import uPlot from 'uplot';
import { PARAM_APID } from './telemetryStore.js';

const C = { cyan: '#35d7ff', magenta: '#ff5bd6', amber: '#f0b030', green: '#4ccf6a', red: '#ff6a5a', blue: '#6aa0ff', white: '#e8eef8' };
const ENGINE_COLORS = [C.cyan, C.amber, C.green];

// Series spec: {name: store parameter, label, color, dash?, scale?}
function xyz(prefix, unit) {
  return ['X', 'Y', 'Z'].map((a, i) => ({ name: prefix + a, label: a + ' ' + unit, color: [C.red, C.green, C.blue][i] }));
}

export function chartDefs(engineSel) {
  const eng = engineSel === 'all' ? [0, 1, 2] : [engineSel];
  const gimbal = [];
  for (const i of eng) {
    const col = engineSel === 'all' ? ENGINE_COLORS[i] : null;
    gimbal.push(
      { name: 'cgp' + i, label: 'E' + (i + 1) + ' pitch cmd', color: col || C.cyan, dash: [5, 4] },
      { name: 'mgp' + i, label: 'E' + (i + 1) + ' pitch meas', color: col || C.cyan },
      { name: 'cgy' + i, label: 'E' + (i + 1) + ' yaw cmd', color: col || C.magenta, dash: [5, 4] },
      { name: 'mgy' + i, label: 'E' + (i + 1) + ' yaw meas', color: col || C.magenta });
  }
  return [
    { id: 'att', title: 'Attitude estimate (rotation vector)', unit: 'deg', series: xyz('att', 'deg') },
    { id: 'rate', title: 'Body rates (IMU)', unit: 'deg/s', series: xyz('rate', 'deg/s') },
    { id: 'gimbal', title: 'Gimbal command (dashed) vs measured', unit: 'deg', series: gimbal },
    { id: 'thr', title: 'Throttle command', unit: '%', series: [0, 1, 2].map((i) => ({ name: 'thr' + i, label: 'E' + (i + 1), color: ENGINE_COLORS[i] })) },
    { id: 'alt', title: 'Altitude estimate vs setpoint', unit: 'm', series: [
      { name: 'alt', label: 'estimate', color: C.cyan }, { name: 'altSp', label: 'setpoint', color: C.magenta, dash: [5, 4] }] },
    { id: 'vz', title: 'Vertical velocity vs setpoint', unit: 'm/s', series: [
      { name: 'vz', label: 'estimate', color: C.cyan }, { name: 'vSp', label: 'setpoint', color: C.magenta, dash: [5, 4] }] },
    { id: 'pos', title: 'East/north estimate and accel commands', unit: 'm', series: [
      { name: 'east', label: 'east m', color: C.cyan }, { name: 'north', label: 'north m', color: C.green },
      { name: 'aE', label: 'accel E m/s2', color: C.cyan, dash: [5, 4], scale: 'a' },
      { name: 'aN', label: 'accel N m/s2', color: C.green, dash: [5, 4], scale: 'a' }] },
  ];
}

// Build [x, y...] for a chart def from the store, joining across APIDs on onboard time.
export function buildData(store, def, tMin) {
  const byApid = new Map();
  def.series.forEach((s, i) => {
    const a = PARAM_APID[s.name];
    if (!byApid.has(a)) byApid.set(a, []);
    byApid.get(a).push(i);
  });
  const tables = [], order = [];
  for (const [, idxs] of byApid) {
    tables.push(store.series(idxs.map((i) => def.series[i].name), tMin));
    order.push(idxs);
  }
  const joined = tables.length === 1 ? tables[0] : uPlot.join(tables);
  if (tables.length === 1) return joined;
  // join returns [x, ...columns in table order]; restore series order.
  const out = [joined[0]];
  let col = 1;
  const placed = new Array(def.series.length);
  for (const idxs of order) for (const i of idxs) placed[i] = joined[col++];
  for (let i = 0; i < placed.length; i++) out.push(placed[i]);
  return out;
}

export function createCharts(container, store, opts = {}) {
  let windowSec = opts.windowSec || 60;
  let engineSel = 'all';
  const xRange = { min: -60, max: 0 };
  const holders = new Map(); // id -> {el, plot, def}

  function plotOpts(def, w, h) {
    const scales = { x: { time: false, range: () => [xRange.min, xRange.max] } };
    const axes = [
      { stroke: '#8da0b8', grid: { stroke: '#1b2533', width: 1 }, ticks: { stroke: '#2a3748' }, font: '10px ui-monospace, Consolas, monospace', size: 24, gap: 2 },
      { stroke: '#8da0b8', grid: { stroke: '#1b2533', width: 1 }, ticks: { stroke: '#2a3748' }, font: '10px ui-monospace, Consolas, monospace', size: 46, gap: 2 },
    ];
    if (def.series.some((s) => s.scale === 'a')) {
      scales.a = { auto: true };
      axes.push({ scale: 'a', side: 1, stroke: '#8da0b8', grid: { show: false }, ticks: { stroke: '#2a3748' }, font: '10px ui-monospace, Consolas, monospace', size: 40, gap: 2 });
    }
    return {
      width: w, height: h, scales, axes,
      legend: { show: true },
      cursor: { show: true, drag: { x: false, y: false } },
      series: [{ label: 't (s)' }, ...def.series.map((s) => ({
        label: s.label, stroke: s.color, width: 1.5, dash: s.dash, scale: s.scale || 'y', spanGaps: true, points: { show: false },
      }))],
    };
  }

  function build() {
    for (const h of holders.values()) { h.plot.destroy(); h.el.remove(); }
    holders.clear();
    for (const def of chartDefs(engineSel)) {
      const el = document.createElement('div');
      el.className = 'chart';
      el.dataset.chart = def.id;
      const head = document.createElement('div');
      head.className = 'chart-title';
      head.textContent = def.title + ' [' + def.unit + ']';
      const body = document.createElement('div');
      body.className = 'chart-body';
      el.append(head, body);
      container.append(el);
      const w = Math.max(200, body.clientWidth), h = Math.max(120, body.clientHeight);
      const plot = new uPlot(plotOpts(def, w, h), [[], ...def.series.map(() => [])], body);
      holders.set(def.id, { el, body, plot, def });
    }
    resize(); // the first charts were created before the grid had its final column widths
  }

  function resize() {
    for (const h of holders.values()) {
      const w = Math.floor(h.body.clientWidth), ht = Math.floor(h.body.clientHeight);
      if (w > 50 && ht > 50 && (w !== h.plot.width || ht !== h.plot.height)) h.plot.setSize({ width: w, height: ht });
    }
  }

  function redraw() {
    const tMax = store.met, tMin = tMax - windowSec;
    xRange.min = tMin; xRange.max = Math.max(tMax, tMin + 1);
    for (const h of holders.values()) {
      const data = buildData(store, h.def, tMin);
      h.plot.setData(data, true);
    }
  }

  build();
  if (typeof ResizeObserver === 'function') new ResizeObserver(() => resize()).observe(container);
  return {
    redraw, resize,
    setWindow(s) { windowSec = s; redraw(); },
    setEngine(sel) { engineSel = sel; build(); redraw(); },
    getWindow() { return windowSec; },
    plots() { return holders; },
    reset() { redraw(); },
  };
}
