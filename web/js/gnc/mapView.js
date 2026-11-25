// Top-down position map, pad centered, north up, auto scaling. DOM/canvas only. The trail and fixes
// live here (collected on every screen); draw(canvas, ...) paints them on whichever canvas is shown.
import { fitCanvas, COLORS } from './canvasUtil.js';
import { geodeticToEnu } from './geodesy.js';
import { mastCorners, armSegment, approachLine, hasApproachLine } from './corridorView.js';

// Pad radius is a placeholder: the downlinked config carries only the alignment tolerance.
export const PAD_RADIUS = 9; // [m] PLACEHOLDER, not a sourced pad dimension
const TRAIL_MAX = 3000;
const TRAIL_DT = 0.2; // [s] onboard time between trail points
const FIX_MAX = 600;
const NICE = [10, 15, 25, 50, 100, 200, 500, 1000, 2000, 5000, 10000];

export function createMapView() {
  const trail = [];
  const fixes = [];
  let lastTrailT = -Infinity, half = NICE[0];

  function reset() { trail.length = 0; fixes.length = 0; lastTrailT = -Infinity; half = NICE[0]; }

  // Feed the newest nav estimate (onboard time t) for the trail.
  function addNav(t, east, north) {
    if (t < lastTrailT) { trail.length = 0; }
    if (t - lastTrailT < TRAIL_DT) return;
    lastTrailT = t;
    trail.push(east, north);
    if (trail.length > TRAIL_MAX * 2) trail.splice(0, 2);
  }
  function addFix(lat, lon, h) {
    fixes.push(lat, lon, h);
    if (fixes.length > FIX_MAX * 3) fixes.splice(0, 3);
  }

  const tmp = [0, 0, 0];
  // catchCfg: catch_config or null (tower, arms, approach line). frame: target ENU frame or null; cfg: mission config or null; sp: {east,north} or null; cur: nav {east,north} or null.
  function draw(canvas, frame, cfg, sp, cur, catchCfg = null) {
    const { ctx, w, h } = fitCanvas(canvas);
    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(0, 0, w, h);

    // Extent: everything plotted, with a floor, snapped up to a nice number (shrinks slowly).
    let ext = Math.max(PAD_RADIUS, cfg ? cfg.alignTolerance : 0) * 1.5;
    for (let i = 0; i < trail.length; i += 2) ext = Math.max(ext, Math.abs(trail[i]), Math.abs(trail[i + 1]));
    const enuFixes = [];
    if (frame) {
      for (let i = 0; i < fixes.length; i += 3) {
        geodeticToEnu(frame, fixes[i], fixes[i + 1], fixes[i + 2], tmp);
        enuFixes.push(tmp[0], tmp[1]);
        ext = Math.max(ext, Math.abs(tmp[0]), Math.abs(tmp[1]));
      }
    }
    if (sp) ext = Math.max(ext, Math.abs(sp.east), Math.abs(sp.north));
    let tower = null;
    if (catchCfg) {
      tower = { mast: mastCorners(catchCfg), arms: [armSegment(catchCfg, 1, false), armSegment(catchCfg, -1, false)], line: approachLine(catchCfg) };
      const pts = [...tower.mast, tower.line.staging, tower.line.seat];
      for (const a of tower.arms) pts.push(a.hinge, a.tip);
      for (const q of pts) ext = Math.max(ext, Math.abs(q[0]), Math.abs(q[1]));
    }
    ext *= 1.15;
    let want = NICE[NICE.length - 1];
    for (const n of NICE) if (n >= ext) { want = n; break; }
    half = want >= half ? want : Math.max(want, half * 0.97);
    const snap = NICE.find((n) => n >= half) || half;
    const scale = (Math.min(w, h) / 2 - 8) / snap; // [px/m]
    const cx = w / 2, cy = h / 2;
    const X = (e) => cx + e * scale, Y = (n) => cy - n * scale;

    // Grid rings / axes.
    ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, cy); ctx.lineTo(w, cy); ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
    ctx.stroke();

    // Pad and alignment tolerance.
    ctx.strokeStyle = COLORS.axis; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(cx, cy, PAD_RADIUS * scale, 0, 2 * Math.PI); ctx.stroke();
    if (cfg) {
      ctx.strokeStyle = COLORS.green; ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.arc(cx, cy, Math.max(1, cfg.alignTolerance * scale), 0, 2 * Math.PI); ctx.stroke();
      ctx.setLineDash([]);
    }

    // Catch tower: mast square, arms (open pose from the config, not live) and the approach line.
    if (tower) {
      ctx.fillStyle = '#5a6678'; ctx.strokeStyle = '#8a97aa'; ctx.lineWidth = 1;
      ctx.beginPath();
      tower.mast.forEach((q, i) => (i ? ctx.lineTo(X(q[0]), Y(q[1])) : ctx.moveTo(X(q[0]), Y(q[1]))));
      ctx.closePath(); ctx.fill(); ctx.stroke();
      ctx.strokeStyle = '#6aa0ff'; ctx.lineWidth = 3;
      for (const a of tower.arms) { ctx.beginPath(); ctx.moveTo(X(a.hinge[0]), Y(a.hinge[1])); ctx.lineTo(X(a.tip[0]), Y(a.tip[1])); ctx.stroke(); }
      if (catchCfg.landingMode === 1) {
        // The vertical catch has no staging point: the Ship translates over the seat early, so mark the seat only.
        // (With a staging distance, as in an older sequence, draw the approach line too.)
        const { staging, seat } = tower.line;
        ctx.fillStyle = COLORS.cyan;
        if (hasApproachLine(catchCfg)) {
          ctx.strokeStyle = COLORS.cyan; ctx.lineWidth = 1; ctx.setLineDash([5, 4]);
          ctx.beginPath(); ctx.moveTo(X(staging[0]), Y(staging[1])); ctx.lineTo(X(seat[0]), Y(seat[1])); ctx.stroke();
          ctx.setLineDash([]);
          ctx.fillRect(X(staging[0]) - 2.5, Y(staging[1]) - 2.5, 5, 5);
        }
        ctx.beginPath(); ctx.arc(X(seat[0]), Y(seat[1]), 3, 0, 2 * Math.PI); ctx.fill();
      }
    }

    // Nav trail.
    if (trail.length >= 4) {
      ctx.strokeStyle = COLORS.cyan; ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(X(trail[0]), Y(trail[1]));
      for (let i = 2; i < trail.length; i += 2) ctx.lineTo(X(trail[i]), Y(trail[i + 1]));
      ctx.stroke();
    }
    // Raw GNSS fixes.
    ctx.fillStyle = COLORS.amber;
    for (let i = 0; i < enuFixes.length; i += 2) ctx.fillRect(X(enuFixes[i]) - 1.5, Y(enuFixes[i + 1]) - 1.5, 3, 3);
    // Setpoint marker (x) and current nav position.
    if (sp) {
      const sx = X(sp.east), sy = Y(sp.north);
      ctx.strokeStyle = COLORS.magenta; ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(sx - 5, sy - 5); ctx.lineTo(sx + 5, sy + 5); ctx.moveTo(sx - 5, sy + 5); ctx.lineTo(sx + 5, sy - 5);
      ctx.stroke();
    }
    if (cur) {
      ctx.fillStyle = COLORS.white;
      ctx.beginPath(); ctx.arc(X(cur.east), Y(cur.north), 3.5, 0, 2 * Math.PI); ctx.fill();
    }

    // North arrow and scale bar.
    ctx.font = '11px ui-monospace, Consolas, monospace';
    ctx.fillStyle = COLORS.text; ctx.strokeStyle = COLORS.text; ctx.lineWidth = 1.5;
    ctx.textAlign = 'center';
    ctx.beginPath(); ctx.moveTo(w - 22, 38); ctx.lineTo(w - 22, 14); ctx.lineTo(w - 27, 22); ctx.moveTo(w - 22, 14); ctx.lineTo(w - 17, 22); ctx.stroke();
    ctx.fillText('N', w - 22, 52);
    const bar = snap / 2; // [m] half the visible half-width
    ctx.textAlign = 'left';
    ctx.beginPath(); ctx.moveTo(10, h - 14); ctx.lineTo(10 + bar * scale, h - 14);
    ctx.moveTo(10, h - 18); ctx.lineTo(10, h - 10); ctx.moveTo(10 + bar * scale, h - 18); ctx.lineTo(10 + bar * scale, h - 10);
    ctx.stroke();
    ctx.fillText(bar + ' m', 10, h - 22);
  }

  return { reset, addNav, addFix, draw };
}
