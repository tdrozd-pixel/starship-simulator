// Catch corridor view for the GNC console. The Ship comes straight down over the seat, so the shared
// horizontal axis is the pin height above the seat (the vertical descent): the upper panel plots the
// lateral error against it (with the funnel, which widens with height, and the catch window at the
// seat), the lower panel the along-axis error. Drawn from catch_status and catch_config only
// (downlinked telemetry, never truth). The geometry helpers at the top are pure and unit tested; the
// canvas drawing at the bottom is DOM-bound and runs only while the screen is shown.
import { fitCanvas, COLORS } from './canvasUtil.js';

// ---- Pure geometry (target-frame ENU, metres) ----

export const TRAIL_SECONDS = 20; // [s] of catch_status history drawn as the trail
const ALONG_STEPS = [5, 10, 15, 20, 30, 50, 100, 200]; // [m] nice extents of the shared axis (height above the seat)
const LATERAL_STEPS = [1, 2, 3, 5, 10, 20]; // [m] nice lateral half extents
const HEIGHT_STEPS = [2, 4, 6, 10, 20, 50]; // [m] nice height extents
const MIN_FIT_SPAN = 0.5; // [m] along span needed to fit the funnel line

// Smallest step that is >= v (the largest step when v exceeds all).
export function niceCeil(v, steps) {
  for (const s of steps) if (s >= v) return s;
  return steps[steps.length - 1];
}

// Unit vectors of the tower approach frame from the downlinked approachDir (east, north: the ship's
// motion toward the tower). a: from the tower toward the approaching ship (-d); b = a rotated +90 deg
// (the +b side is positive lateralError). Normalizes; falls back to (1, 0) for a degenerate vector.
export function approachFrame(approachDir) {
  let dE = approachDir ? approachDir[0] : 1, dN = approachDir ? approachDir[1] : 0;
  const n = Math.hypot(dE, dN);
  if (!(n > 1e-6)) { dE = 1; dN = 0; } else { dE /= n; dN /= n; }
  return { dE, dN, aE: -dE, aN: -dN, bE: dN, bN: -dE };
}

// Funnel limit(along) = base + slope * along, fitted through the two samples with the smallest and
// the largest along (the flight software funnel is linear). Returns null for fewer than two samples
// or a span under MIN_FIT_SPAN. along and limit are indexable arrays, n the sample count.
export function funnelFit(along, limit, n) {
  let lo = -1, hi = -1;
  for (let i = 0; i < n; i++) {
    if (!Number.isFinite(along[i]) || !Number.isFinite(limit[i])) continue;
    if (lo < 0 || along[i] < along[lo]) lo = i;
    if (hi < 0 || along[i] > along[hi]) hi = i;
  }
  if (lo < 0 || along[hi] - along[lo] < MIN_FIT_SPAN) return null;
  const slope = (limit[hi] - limit[lo]) / (along[hi] - along[lo]);
  return { base: limit[lo] - slope * along[lo], slope };
}
// Funnel half width at a given along distance (the funnel does not widen behind the seat).
export function funnelLimit(fit, along) { return fit.base + fit.slope * Math.max(along, 0); }

// Axis extents of the corridor view. points: arrays of trail samples (along, lateral, height, count n);
// cfg: catch_config or null; fit: funnelFit result or null. Returns {alongMax, alongMin, lateralMax,
// heightMax, heightMin}, all snapped to nice steps so the scales do not jitter.
export function corridorExtents(along, lateral, height, n, cfg, fit) {
  let a = 0, l = 0, h = 0, hmin = 0;
  for (let i = 0; i < n; i++) {
    if (Number.isFinite(along[i])) a = Math.max(a, along[i]);
    if (Number.isFinite(lateral[i])) l = Math.max(l, Math.abs(lateral[i]));
    if (Number.isFinite(height[i])) { h = Math.max(h, height[i]); hmin = Math.min(hmin, height[i]); }
  }
  if (cfg) l = Math.max(l, cfg.windowLateral * 1.5);
  const alongMax = niceCeil(a * 1.1, ALONG_STEPS);
  if (fit) l = Math.max(l, funnelLimit(fit, alongMax) * 1.1);
  return {
    alongMax,
    alongMin: -(cfg ? Math.max(2 * cfg.windowAlong, 1) : 2), // a little room past the seat
    lateralMax: niceCeil(l * 1.1, LATERAL_STEPS),
    heightMax: niceCeil(h * 1.1, HEIGHT_STEPS),
    heightMin: hmin < -0.25 ? -niceCeil(-hmin, [0.5, 1, 2, 5]) : -0.5,
  };
}

// The catch window as a box {along, lateral}: half extents about the seat point [m].
export function windowBox(cfg) { return cfg ? { along: cfg.windowAlong, lateral: cfg.windowLateral } : null; }

// Tower and arm geometry for the map, from catch_config. All points [e, n] in the target frame.
export function mastCorners(cfg) {
  const f = approachFrame(cfg.approachDir), [te, tn] = cfg.towerPosition, w = cfg.mastHalfWidth;
  const out = [];
  for (const [sa, sb] of [[1, 1], [1, -1], [-1, -1], [-1, 1]]) {
    out.push([te + sa * w * f.dE + sb * w * f.bE, tn + sa * w * f.dN + sb * w * f.bN]);
  }
  return out;
}
// One arm. side: +1 for the +b arm, -1 for the -b arm. closed false: open pose (armOpenAngle, outward
// from the approach axis); true: closed pose (armClosedAngle, inward). Returns {hinge, tip}.
export function armSegment(cfg, side, closed) {
  const f = approachFrame(cfg.approachDir), [te, tn] = cfg.towerPosition;
  const hE = te + cfg.hingeSetback * f.aE + side * cfg.hingeHalfSpacing * f.bE;
  const hN = tn + cfg.hingeSetback * f.aN + side * cfg.hingeHalfSpacing * f.bN;
  // Direction from a, rotated toward +b by the outward angle (open) or toward the axis (closed).
  const phi = closed ? -cfg.armClosedAngle : cfg.armOpenAngle;
  const c = Math.cos(phi), s = side * Math.sin(phi);
  const dE = c * f.aE + s * f.bE, dN = c * f.aN + s * f.bN;
  return { hinge: [hE, hN], tip: [hE + cfg.armLength * dE, hN + cfg.armLength * dN] };
}
// True when the flight software translates to a staging point beside the seat (a horizontal approach).
// The vertical catch has stagingDistance 0: there is no approach line to draw, only the seat point.
export function hasApproachLine(cfg) { return !!cfg && cfg.stagingDistance > 0.5; }
// Staging point and seat point of the approach line [e, n] (they coincide when there is no staging distance).
export function approachLine(cfg) {
  const f = approachFrame(cfg.approachDir);
  const seat = [cfg.seatPoint[0], cfg.seatPoint[1]];
  return { staging: [seat[0] - cfg.stagingDistance * f.dE, seat[1] - cfg.stagingDistance * f.dN], seat };
}

// ---- Canvas drawing ----

const FONT = '11px ui-monospace, Consolas, monospace';

// status: decoded catch_status or null; cfg: decoded catch_config or null; trail: {n, along, lateral,
// height} (typed-array views of the stored columns, oldest first) or null. Draws the whole canvas.
export function drawCorridor(canvas, status, cfg, trail) {
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, w, h);
  ctx.font = FONT;
  if (!status) {
    ctx.fillStyle = COLORS.dim; ctx.textAlign = 'center';
    ctx.fillText('NO CATCH DATA', w / 2, h / 2);
    return;
  }
  const n = trail ? trail.n : 0;
  // Shared axis: height above the seat. Upper panel: lateral error. Lower panel: along-axis error (signed).
  const along = trail ? trail.height : [], lateral = trail ? trail.lateral : [], height = trail ? trail.along : [];
  const limits = trail && trail.limit ? trail.limit : [];
  const fit = trail ? funnelFit(along, limits, n) : null;
  const ex = corridorExtents(along, lateral, height, n, cfg, fit);
  ex.alongMin = -2; // [m] a little room below the seat
  if (cfg) ex.heightMax = Math.max(ex.heightMax, niceCeil(cfg.windowAlong * 1.5, HEIGHT_STEPS));
  if (cfg) ex.heightMin = Math.min(ex.heightMin, -niceCeil(cfg.windowAlong * 1.5, [0.5, 1, 2, 5]));

  const left = 44, right = w - 10, gap = 26;
  const topY0 = 16, topY1 = Math.round(h * 0.58), sideY0 = topY1 + gap, sideY1 = h - 22;
  const X = (a) => left + (ex.alongMax - a) / (ex.alongMax - ex.alongMin) * (right - left);
  const topY = (l) => (topY0 + topY1) / 2 + l / ex.lateralMax * (topY1 - topY0) / 2; // +b down
  const sideY = (hh) => sideY1 - (hh - ex.heightMin) / (ex.heightMax - ex.heightMin) * (sideY1 - sideY0);

  // Frames and zero lines.
  ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1;
  ctx.strokeRect(left, topY0, right - left, topY1 - topY0);
  ctx.strokeRect(left, sideY0, right - left, sideY1 - sideY0);
  ctx.strokeStyle = COLORS.axis;
  ctx.beginPath();
  ctx.moveTo(left, topY(0)); ctx.lineTo(right, topY(0)); // approach axis
  ctx.moveTo(X(0), topY0); ctx.lineTo(X(0), topY1); ctx.moveTo(X(0), sideY0); ctx.lineTo(X(0), sideY1); // seat height
  ctx.moveTo(left, sideY(0)); ctx.lineTo(right, sideY(0)); // zero along error
  ctx.stroke();

  // Catch window (physical, at the seat): lateral and along limits as dashed lines.
  const win = windowBox(cfg);
  if (win) {
    ctx.strokeStyle = COLORS.green; ctx.setLineDash([4, 3]); ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (const sgn of [1, -1]) {
      ctx.moveTo(left, topY(sgn * win.lateral)); ctx.lineTo(right, topY(sgn * win.lateral));
      ctx.moveTo(left, sideY(sgn * win.along)); ctx.lineTo(right, sideY(sgn * win.along));
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Funnel (fitted from the downlinked limit samples; the current limit marks it at the pin).
  ctx.strokeStyle = COLORS.amber; ctx.lineWidth = 1.5;
  if (fit) {
    for (const sgn of [1, -1]) {
      ctx.beginPath();
      ctx.moveTo(X(ex.alongMax), topY(sgn * funnelLimit(fit, ex.alongMax)));
      ctx.lineTo(X(0), topY(sgn * funnelLimit(fit, 0)));  // widens with height above the seat
      ctx.stroke();
    }
  } else if (Number.isFinite(status.lateralLimit)) {
    ctx.setLineDash([2, 3]);
    for (const sgn of [1, -1]) { ctx.beginPath(); ctx.moveTo(left, topY(sgn * status.lateralLimit)); ctx.lineTo(right, topY(sgn * status.lateralLimit)); ctx.stroke(); }
    ctx.setLineDash([]);
  }

  // Trail and current position.
  const trailPath = (xs, ys, X2, Y2) => {
    if (n < 2) return;
    ctx.strokeStyle = COLORS.cyan; ctx.lineWidth = 1.5;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < n; i++) {
      if (!Number.isFinite(xs[i]) || !Number.isFinite(ys[i])) continue;
      if (started) ctx.lineTo(X2(xs[i]), Y2(ys[i])); else { ctx.moveTo(X2(xs[i]), Y2(ys[i])); started = true; }
    }
    ctx.stroke();
  };
  trailPath(along, lateral, X, topY);
  trailPath(along, height, X, sideY);  // (height here is the along-axis error, see above)
  const ring = status.go ? COLORS.green : status.corridorOk ? COLORS.amber : COLORS.red;
  const marker = (x, y) => {
    ctx.fillStyle = COLORS.white;
    ctx.beginPath(); ctx.arc(x, y, 4, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = ring; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 7, 0, 2 * Math.PI); ctx.stroke();
  };
  const cl = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  if (Number.isFinite(status.pinHeightAboveSeat) && Number.isFinite(status.lateralError)) {
    marker(cl(X(status.pinHeightAboveSeat), left, right), cl(topY(status.lateralError), topY0, topY1));
  }
  if (Number.isFinite(status.pinHeightAboveSeat) && Number.isFinite(status.alongRemaining)) {
    marker(cl(X(status.pinHeightAboveSeat), left, right), cl(sideY(status.alongRemaining), sideY0, sideY1));
  }

  // Labels.
  ctx.fillStyle = COLORS.text; ctx.textAlign = 'left';
  ctx.fillText('LATERAL ERROR [m] (+b down) against height above the seat', left + 4, topY0 + 12);
  ctx.fillText('ALONG-AXIS ERROR [m] (+ = short of the seat)', left + 4, sideY0 + 12);
  ctx.fillStyle = COLORS.dim; ctx.textAlign = 'right';
  ctx.fillText('+' + ex.lateralMax + ' (+b)', left - 4, topY(ex.lateralMax) + 10);
  ctx.fillText('-' + ex.lateralMax, left - 4, topY(-ex.lateralMax));
  ctx.fillText('0', left - 4, topY(0) + 4);
  ctx.fillText('+' + ex.heightMax, left - 4, sideY(ex.heightMax) + 10);
  ctx.fillText('0', left - 4, sideY(0) + 4);
  ctx.textAlign = 'center';
  ctx.fillText('SEAT HEIGHT', X(0), sideY1 + 12);
  ctx.fillText(ex.alongMax + ' m above', X(ex.alongMax) - 22, sideY1 + 12);
  ctx.textAlign = 'right';
  ctx.fillText('ship descends from the left; amber funnel (flight software gate), green dashed catch window', right, sideY1 + 12);
}
