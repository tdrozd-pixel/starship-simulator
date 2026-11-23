// Display decimation and alignment for chart data. Pure module (no DOM, no uPlot).
// A chart plot is a few hundred pixels wide, so drawing 3000 points per series is wasted work.

// Create a decimator with its own scratch buffers. The returned function takes
// data = [x, y1, y2, ...] (arrays or typed arrays, x monotonic) and maxPoints (about 2 per pixel)
// and returns [x', y1', ...] as Float64Array views into the decimator's scratch, valid until the
// next call. Always copies, so the result never aliases the input.
// Reduction: the index range is cut into maxPoints/2 equal buckets; in each bucket the series
// with the largest range (normalised by that series' overall span) contributes its min and max
// samples. All series are sampled at the chosen indices, so spikes of the dominant series are
// kept and x stays monotonic. The newest sample is always kept (the live edge).
export function createDecimator() {
  let cap = 0, nSeries = 0;
  let xs = null, cols = [], idx = null, span = null;

  function ensure(points, series) {
    if (points <= cap && series <= nSeries) return;
    cap = Math.max(points + 256, cap); nSeries = Math.max(series, nSeries);
    xs = new Float64Array(cap + 2);
    cols = [];
    for (let i = 0; i < nSeries; i++) cols.push(new Float64Array(cap + 2));
    idx = new Int32Array(cap + 2);
    span = new Float64Array(nSeries);
  }

  return function decimate(data, maxPoints) {
    const x = data[0], s = data.length - 1, n = x.length;
    maxPoints = Math.max(4, Math.floor(maxPoints));
    ensure(Math.max(maxPoints, Math.min(n, maxPoints)) + 2, s);
    let m = 0;
    if (n <= maxPoints) {
      for (let i = 0; i < n; i++) idx[i] = i;
      m = n;
    } else {
      for (let c = 0; c < s; c++) {
        const y = data[c + 1];
        let lo = Infinity, hi = -Infinity;
        for (let i = 0; i < n; i++) { const v = y[i]; if (v < lo) lo = v; if (v > hi) hi = v; }
        span[c] = hi > lo ? hi - lo : 1;
      }
      const buckets = maxPoints >> 1;
      for (let k = 0; k < buckets; k++) {
        const a = Math.floor(k * n / buckets), b = Math.floor((k + 1) * n / buckets);
        if (b - a <= 2) { for (let i = a; i < b; i++) idx[m++] = i; continue; }
        let best = -1, bestRange = -1, bestMin = a, bestMax = a;
        for (let c = 0; c < s; c++) {
          const y = data[c + 1];
          let mn = a, mx = a, vmn = y[a], vmx = y[a];
          for (let i = a + 1; i < b; i++) {
            const v = y[i];
            if (v < vmn) { vmn = v; mn = i; }
            if (v > vmx) { vmx = v; mx = i; }
          }
          const r = (vmx - vmn) / span[c];
          if (r > bestRange) { bestRange = r; best = c; bestMin = mn; bestMax = mx; }
        }
        if (best < 0) { idx[m++] = a; continue; } // all NaN
        if (bestMin === bestMax) idx[m++] = bestMin;
        else if (bestMin < bestMax) { idx[m++] = bestMin; idx[m++] = bestMax; }
        else { idx[m++] = bestMax; idx[m++] = bestMin; }
      }
      if (m === 0 || idx[m - 1] !== n - 1) idx[m++] = n - 1;
    }
    for (let i = 0; i < m; i++) xs[i] = x[idx[i]];
    const out = [xs.subarray(0, m)];
    for (let c = 0; c < s; c++) {
      const y = data[c + 1], dst = cols[c];
      for (let i = 0; i < m; i++) dst[i] = y[idx[i]];
      out.push(dst.subarray(0, m));
    }
    return out;
  };
}

const cut = (a, n) => (a.length === n ? a : a.subarray ? a.subarray(0, n) : a.slice(0, n));

// Aligner for per-APID tables ([x, ...cols] each) into one [x, ...cols of table 0, cols of table 1, ...].
// Packets of different APIDs from one flight cycle carry identical onboard timestamps, so when
// the x arrays agree on their common prefix the columns are simply concatenated as views (the
// newest samples of a longer table are dropped until the others catch up). When they do not (the
// comms model loses packets), the x arrays are merged in one linear pass over the union of
// timestamps and a table that lacks a timestamp holds its previous value for that row (no nulls,
// so typed arrays work, and no uPlot.join). The result is views into scratch, valid until the
// next call; consume or copy it before aligning again.
export function createAligner() {
  let cap = 0, cols = 0, xs = null, out = [];
  const ptr = [], last = [];
  function ensure(rows, ncols) {
    if (rows <= cap && ncols <= cols) return;
    cap = Math.max(rows + 1024, cap); cols = Math.max(ncols, cols);
    xs = new Float64Array(cap);
    out = [];
    for (let i = 0; i < cols; i++) out.push(new Float64Array(cap));
  }
  return function align(tables) {
    if (tables.length === 1) return tables[0];
    let n = Infinity;
    for (const t of tables) n = Math.min(n, t[0].length);
    const x0 = tables[0][0];
    let same = true;
    for (let k = 1; k < tables.length && same; k++) {
      const xk = tables[k][0];
      for (let i = 0; i < n; i++) if (xk[i] !== x0[i]) { same = false; break; }
    }
    if (same) {
      const res = [cut(x0, n)];
      for (const t of tables) for (let c = 1; c < t.length; c++) res.push(cut(t[c], n));
      return res;
    }
    let rows = 0, ncols = 0;
    for (const t of tables) { rows += t[0].length; ncols += t.length - 1; }
    ensure(rows, ncols);
    for (let k = 0; k < tables.length; k++) { ptr[k] = 0; last[k] = -1; }
    let m = 0;
    for (;;) {
      let tNext = Infinity;
      for (let k = 0; k < tables.length; k++) {
        const x = tables[k][0];
        if (ptr[k] < x.length && x[ptr[k]] < tNext) tNext = x[ptr[k]];
      }
      if (tNext === Infinity) break;
      xs[m] = tNext;
      let o = 0;
      for (let k = 0; k < tables.length; k++) {
        const t = tables[k], x = t[0];
        if (ptr[k] < x.length && x[ptr[k]] === tNext) last[k] = ptr[k]++;
        const src = last[k] >= 0 ? last[k] : (x.length ? 0 : -1); // before its first sample: hold the first
        for (let c = 1; c < t.length; c++) out[o++][m] = src >= 0 ? t[c][src] : NaN;
      }
      m++;
    }
    const res = [xs.subarray(0, m)];
    for (let i = 0; i < ncols; i++) res.push(out[i].subarray(0, m));
    return res;
  };
}
