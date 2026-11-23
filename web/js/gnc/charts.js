// uPlot charts for the screen being shown. DOM-bound. x is onboard time [s]; every chart shows the
// same window ending at the newest onboard time. A chart set exists only while its screen is shown:
// switching screens destroys the set (plots, observers, DOM) and builds the next one from the store
// history, so hidden screens cost nothing.
import uPlot from 'uplot';
import { expandSeries, buildData } from './chartDefs.js';
import { createDecimator, createAligner } from './decimate.js';

const REDRAW_PERIOD_MS = 250; // 4 Hz (limit: 5 Hz)
const TICK_BUDGET_MS = 6; // stop starting new chart redraws in a frame after this long
const PLOT_MARGIN_PX = 50; // y axis and padding inside the plot width
const AXIS_FONT = '10px ui-monospace, Consolas, monospace';

// Draw counters for tests and the perf readout: total and per chart id.
export const chartStats = { draws: 0, byId: {} };

// opts: {windowSec, engine (0..2 or 'all')}. Charts are added with add(container, spec).
export function createChartSet(store, opts) {
  const windowSec = opts.windowSec || 60;
  const engine = opts.engine === undefined ? 0 : opts.engine;
  const xRange = { min: -windowSec, max: 0 };
  const holders = [];
  const pending = [];
  let cycleStart = -Infinity, lastVersion = -1, destroyed = false;

  function plotOpts(def, w, h) {
    const axis = (size, extra) => ({ stroke: '#8da0b8', grid: { stroke: '#1b2533', width: 1 }, ticks: { stroke: '#2a3748' }, font: AXIS_FONT, size, gap: 2, ...extra });
    return {
      width: w, height: h,
      scales: { x: { time: false, range: () => [xRange.min, xRange.max] } },
      axes: [axis(24), axis(46)],
      legend: { show: false }, // the chart header carries a static legend
      cursor: { show: false },
      series: [{ label: 't (s)' }, ...def.series.map((s) => ({
        label: s.label, stroke: s.color, width: 1.5, dash: s.dash, spanGaps: true, points: { show: false },
      }))],
    };
  }

  function legendEl(def) {
    const box = document.createElement('span');
    box.className = 'chart-legend';
    for (const s of def.series) {
      const k = document.createElement('span');
      k.textContent = s.label;
      k.style.color = s.color;
      k.className = s.dash ? 'dashed' : '';
      box.append(k);
    }
    return box;
  }

  const ro = typeof ResizeObserver === 'function' ? new ResizeObserver((entries) => {
    // Sizes come from the observer entry, so no layout is read while updating.
    for (const e of entries) {
      const h = e.target.__holder;
      if (!h || destroyed) continue;
      const r = e.contentRect;
      if (r.width > 50 && r.height > 50 && (Math.floor(r.width) !== h.plot.width || Math.floor(r.height) !== h.plot.height)) {
        h.plot.setSize({ width: Math.floor(r.width), height: Math.floor(r.height) });
        h.drawnVersion = -1;
      }
    }
  }) : null;

  // spec: {id, title, unit, series}. Builds one chart (header with legend, plot) inside container.
  function add(container, spec) {
    const def = { id: spec.id, title: spec.title, unit: spec.unit, series: expandSeries(spec.series, engine) };
    const el = document.createElement('div');
    el.className = 'chart';
    el.dataset.chart = def.id;
    const head = document.createElement('div');
    head.className = 'chart-title';
    const title = document.createElement('span');
    title.textContent = def.title + ' [' + def.unit + ']';
    head.append(title, legendEl(def));
    const body = document.createElement('div');
    body.className = 'chart-body';
    el.append(head, body);
    container.append(el);
    const w = Math.max(200, body.clientWidth), h = Math.max(100, body.clientHeight); // one layout read per build
    const plot = new uPlot(plotOpts(def, w, h), [[], ...def.series.map(() => [])], body);
    const holder = { el, body, plot, def, dec: createDecimator(), align: createAligner(), drawnVersion: -1 };
    body.__holder = holder;
    holders.push(holder);
    if (ro) ro.observe(body);
  }

  function drawChart(h) {
    const maxPts = 2 * Math.max(50, h.plot.width - PLOT_MARGIN_PX);
    h.plot.setData(h.dec(buildData(store, h.def.series, xRange.min, h.align), maxPts), true);
    h.drawnVersion = store.version;
    chartStats.draws++;
    chartStats.byId[h.def.id] = (chartStats.byId[h.def.id] || 0) + 1;
  }

  function setRange() {
    const tMax = store.met, tMin = tMax - windowSec;
    xRange.min = tMin; xRange.max = Math.max(tMax, tMin + 1);
  }

  // Draw every chart now (view switch, window change, tests).
  function drawAll() {
    setRange();
    for (const h of holders) drawChart(h);
    pending.length = 0;
    lastVersion = store.version;
    cycleStart = performance.now();
  }

  // Frame-loop entry. Starts a cycle at most every 200 ms when the store changed, and draws a few
  // charts per call within a time budget so no frame stalls. Returns true if anything was drawn.
  function update(nowMs) {
    if (pending.length === 0 && nowMs - cycleStart >= REDRAW_PERIOD_MS - 4 && store.version !== lastVersion) {
      cycleStart = nowMs;
      lastVersion = store.version;
      for (const h of holders) pending.push(h);
      setRange();
    }
    if (pending.length === 0) return false;
    const t0 = performance.now();
    do { drawChart(pending.shift()); } while (pending.length && performance.now() - t0 < TICK_BUDGET_MS);
    return true;
  }

  function destroy() {
    destroyed = true;
    if (ro) ro.disconnect();
    for (const h of holders) { h.plot.destroy(); h.el.remove(); }
    holders.length = 0;
    pending.length = 0;
  }

  return { add, drawAll, update, destroy, ids: () => holders.map((h) => h.def.id) };
}
