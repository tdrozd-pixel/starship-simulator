// Optional perf readout for the GNC console (?debug=1 or the menu toggle). DOM only when enabled;
// the page keeps a null monitor otherwise and every call site is guarded, so the disabled cost is zero.
const N = 240; // rolling window length

class Ring {
  constructor() { this.v = new Float64Array(N); this.n = 0; this.i = 0; this.scratch = new Float64Array(N); }
  push(x) { this.v[this.i] = x; this.i = (this.i + 1) % N; if (this.n < N) this.n++; }
  mean() { let s = 0; for (let k = 0; k < this.n; k++) s += this.v[k]; return this.n ? s / this.n : 0; }
  max() { let m = 0; for (let k = 0; k < this.n; k++) if (this.v[k] > m) m = this.v[k]; return m; }
  pct(p) {
    if (!this.n) return 0;
    const a = this.scratch.subarray(0, this.n);
    a.set(this.v.subarray(0, this.n));
    a.sort();
    return a[Math.min(this.n - 1, Math.floor(p * this.n))];
  }
}

// getPackets: () => total packets received so far.
export function createPerfMonitor(getPackets, { overlay = true } = {}) {
  const rings = { frame: new Ring(), charts: new Ring(), panels: new Ring() };
  const st = { longTasks: 0, longTaskMs: 0, packetsPerSec: 0 };
  let lastFrame = 0, lastPk = getPackets(), lastPkT = performance.now();
  let el = null, lastShow = 0;
  let observer = null;
  try {
    observer = new PerformanceObserver((list) => {
      for (const e of list.getEntries()) { st.longTasks++; st.longTaskMs += e.duration; }
    });
    observer.observe({ entryTypes: ['longtask'] });
  } catch (e) { observer = null; /* longtask not supported */ }
  if (overlay) {
    el = document.createElement('pre');
    el.id = 'perf';
    el.style.cssText = 'position:fixed;right:6px;bottom:6px;margin:0;padding:4px 6px;background:#000c;color:#9f9;font:11px ui-monospace,Consolas,monospace;z-index:9;pointer-events:none';
    document.body.append(el);
  }

  function snapshot() {
    return {
      frameP50: rings.frame.pct(0.5), frameP95: rings.frame.pct(0.95), frameMax: rings.frame.max(),
      chartsMs: rings.charts.mean(), panelsMs: rings.panels.mean(),
      chartsMaxMs: rings.charts.max(), panelsMaxMs: rings.panels.max(),
      packetsPerSec: st.packetsPerSec, longTasks: st.longTasks, longTaskMs: st.longTaskMs,
    };
  }

  return {
    // Call once per update tick with performance.now() [ms].
    frame(ms) {
      if (lastFrame) rings.frame.push(ms - lastFrame);
      lastFrame = ms;
      const now = performance.now();
      if (now - lastPkT >= 1000) {
        const pk = getPackets();
        st.packetsPerSec = (pk - lastPk) * 1000 / (now - lastPkT);
        lastPk = pk; lastPkT = now;
      }
      if (el && now - lastShow >= 500) {
        lastShow = now;
        const s = snapshot();
        el.textContent = 'tick p50 ' + s.frameP50.toFixed(1) + ' p95 ' + s.frameP95.toFixed(1) + ' ms\n' +
          'charts ' + s.chartsMs.toFixed(2) + '  panels ' + s.panelsMs.toFixed(2) + ' ms/call\n' +
          'pkts/s ' + s.packetsPerSec.toFixed(0) + '  long tasks ' + st.longTasks;
      }
    },
    record(name, ms) { rings[name].push(ms); },
    destroy() { if (observer) observer.disconnect(); if (el) el.remove(); },
    snapshot,
  };
}
