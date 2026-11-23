// One gimbal box per engine: +-15 deg crosshair, measured dot vs commanded ring. DOM/canvas only.
import { fitCanvas, cssSize, COLORS } from './canvasUtil.js';
import { GIMBAL_LIMIT_DEG, RAD2DEG } from './gncPackets.js';

// cmd: {pitch,yaw,on} rad or null; meas: {pitch,yaw,valid} rad or null. Yaw is horizontal, pitch vertical (up positive).
// Skips the redraw (returns false) when nothing visible changed since the last draw of this canvas.
const lastKey = new WeakMap();
const EPS = 0.0003; // [rad] about 0.017 deg: well under one pixel of the box
export function drawGimbalBox(canvas, cmd, meas) {
  const sz = cssSize(canvas);
  const key = [cmd ? cmd.pitch : NaN, cmd ? cmd.yaw : NaN, meas ? meas.pitch : NaN, meas ? meas.yaw : NaN, meas && meas.valid ? 1 : 0, sz.w, sz.h];
  const prev = lastKey.get(canvas);
  if (prev) {
    let same = true;
    for (let i = 0; i < key.length && same; i++) {
      const a = key[i], b = prev[i];
      if (!(a === b || (a !== a && b !== b) || (i < 4 && Math.abs(a - b) < EPS))) same = false;
    }
    if (same) return false;
  }
  lastKey.set(canvas, key);
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, w, h);
  const m = 14, size = Math.min(w, h) - 2 * m;
  const cx = w / 2, cy = h / 2, half = size / 2;
  const px = (deg) => (deg / GIMBAL_LIMIT_DEG) * half;
  ctx.lineWidth = 1;
  ctx.strokeStyle = COLORS.grid;
  for (const f of [0.5]) {
    ctx.strokeRect(cx - half * f, cy - half * f, size * f, size * f);
  }
  ctx.strokeStyle = COLORS.axis;
  ctx.strokeRect(cx - half, cy - half, size, size);
  ctx.beginPath();
  ctx.moveTo(cx - half, cy); ctx.lineTo(cx + half, cy);
  ctx.moveTo(cx, cy - half); ctx.lineTo(cx, cy + half);
  ctx.stroke();
  ctx.fillStyle = COLORS.dim;
  ctx.font = '10px ui-monospace, Consolas, monospace';
  ctx.textAlign = 'center';
  ctx.fillText('YAW', cx, h - 2);
  ctx.fillText('+15', cx + half, h - 2);
  ctx.fillText('-15', cx - half, h - 2);
  ctx.textAlign = 'left';
  ctx.fillText('PITCH', 2, 10);
  const clamp = (v) => Math.max(-1.2 * GIMBAL_LIMIT_DEG, Math.min(1.2 * GIMBAL_LIMIT_DEG, v));
  if (cmd) {
    ctx.strokeStyle = COLORS.magenta;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx + px(clamp(cmd.yaw * RAD2DEG)), cy - px(clamp(cmd.pitch * RAD2DEG)), 7, 0, 2 * Math.PI);
    ctx.stroke();
  }
  if (meas) {
    ctx.fillStyle = meas.valid ? COLORS.green : COLORS.amber;
    ctx.beginPath();
    ctx.arc(cx + px(clamp(meas.yaw * RAD2DEG)), cy - px(clamp(meas.pitch * RAD2DEG)), 4, 0, 2 * Math.PI);
    ctx.fill();
  }
  return true;
}
