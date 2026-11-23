// 2D attitude indicator: two side silhouettes of the vehicle (seen from the south and from the
// west) tilted by the estimated attitude. Tilt is shown exaggerated so a few degrees is visible.
// DOM/canvas only; drawn only while its view is shown.
import { fitCanvas, cssSize, COLORS } from './canvasUtil.js';

export const TILT_GAIN = 5; // drawn angle = gain * tilt, labeled on the canvas
const EPS = 0.02; // [deg] below the pixel resolution of the drawing

let last = null;
function silhouette(ctx, cx, cy, hgt, angleDeg, label, ly) {
  const w = hgt * 0.16; // body width
  ctx.save();
  ctx.translate(cx, cy);
  ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1; // vertical reference
  ctx.setLineDash([3, 4]);
  ctx.beginPath(); ctx.moveTo(0, -hgt * 0.62); ctx.lineTo(0, hgt * 0.62); ctx.stroke();
  ctx.setLineDash([]);
  ctx.rotate(angleDeg * Math.PI / 180);
  ctx.fillStyle = '#2a3b52'; ctx.strokeStyle = COLORS.cyan; ctx.lineWidth = 1.5;
  ctx.beginPath(); // body, nose cone up, skirt and legs at the base
  ctx.moveTo(-w / 2, hgt * 0.5); ctx.lineTo(-w / 2, -hgt * 0.22);
  ctx.quadraticCurveTo(-w / 2, -hgt * 0.42, 0, -hgt * 0.5);
  ctx.quadraticCurveTo(w / 2, -hgt * 0.42, w / 2, -hgt * 0.22);
  ctx.lineTo(w / 2, hgt * 0.5);
  ctx.lineTo(w * 0.85, hgt * 0.58); ctx.lineTo(-w * 0.85, hgt * 0.58);
  ctx.closePath(); ctx.fill(); ctx.stroke();
  ctx.fillStyle = COLORS.amber; // exhaust marker below the base
  ctx.fillRect(-w * 0.25, hgt * 0.6, w * 0.5, 3);
  ctx.restore();
  ctx.fillStyle = COLORS.dim; ctx.textAlign = 'center';
  ctx.fillText(label, cx, ly);
}

// att: {tiltE, tiltN, roll} [deg] or null.
export function drawAttitude(canvas, att) {
  const sz = cssSize(canvas);
  const key = att ? [att.tiltE, att.tiltN, sz.w, sz.h] : [NaN, NaN, sz.w, sz.h];
  if (last && last.canvas === canvas && last.key.every((v, i) => v === key[i] || (v !== v && key[i] !== key[i]) || (i < 2 && Math.abs(v - key[i]) < EPS))) return false;
  last = { canvas, key };
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.fillStyle = COLORS.bg; ctx.fillRect(0, 0, w, h);
  ctx.font = '11px ui-monospace, Consolas, monospace';
  const hgt = Math.min((h - 44) / 1.2, w / 2.4); // the drawing spans -0.5 to +0.62 hgt around the center
  const cy = 12 + hgt * 0.55;
  const ly = h - 8;
  silhouette(ctx, w * 0.27, cy, hgt, att ? TILT_GAIN * att.tiltE : 0, 'FROM SOUTH: EAST TILT ' + (att ? att.tiltE.toFixed(2) : '--') + ' deg', ly);
  silhouette(ctx, w * 0.73, cy, hgt, att ? -TILT_GAIN * att.tiltN : 0, 'FROM WEST: NORTH TILT ' + (att ? att.tiltN.toFixed(2) : '--') + ' deg', ly);
  ctx.textAlign = 'left'; ctx.fillStyle = COLORS.dim;
  ctx.fillText('tilt drawn x' + TILT_GAIN, 6, 14);
  return true;
}

let miniLast = null;
export function drawAttitudeMini(canvas, att) {
  const sz = cssSize(canvas);
  const key = att ? [att.tiltE, att.tiltN, att.roll, sz.w, sz.h] : [NaN, NaN, NaN, sz.w, sz.h];
  if (miniLast && miniLast.canvas === canvas &&
      miniLast.key.every((v, i) => v === key[i] || (v !== v && key[i] !== key[i]) || (i < 3 && Math.abs(v - key[i]) < EPS))) {
    return false;
  }
  miniLast = { canvas, key };
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.fillStyle = COLORS.bg;
  ctx.fillRect(0, 0, w, h);
  ctx.font = '10px ui-monospace, Consolas, monospace';
  ctx.textBaseline = 'middle';

  const cx = 13, cy = h / 2, r = Math.min(11, h * 0.35);
  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();
  if (att) {
    const roll = (att.roll - 90) * Math.PI / 180;
    ctx.strokeStyle = COLORS.magenta;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(roll) * r, cy + Math.sin(roll) * r);
    ctx.stroke();
  }
  ctx.fillStyle = COLORS.dim;
  ctx.fillText('R', 28, cy);
  ctx.fillStyle = COLORS.magenta;
  ctx.fillText(att ? att.roll.toFixed(1) + '\u00b0' : '--', 50, cy);

  ctx.strokeStyle = COLORS.grid;
  ctx.beginPath(); ctx.moveTo(78, cy); ctx.lineTo(w - 3, cy); ctx.stroke();
  if (att) {
    const tilt = Math.max(-35, Math.min(35, att.tiltE));
    ctx.save();
    ctx.translate(99, cy);
    ctx.rotate(-tilt * Math.PI / 180);
    ctx.strokeStyle = COLORS.cyan;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(0, 8); ctx.lineTo(0, -8); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(-5, -4); ctx.lineTo(0, -9); ctx.lineTo(5, -4); ctx.stroke();
    ctx.restore();
  }
  ctx.fillStyle = COLORS.dim;
  ctx.fillText('P', 113, cy);
  ctx.fillStyle = COLORS.cyan;
  ctx.textAlign = 'right';
  ctx.fillText(att ? att.tiltE.toFixed(1) + '\u00b0' : '--', w - 4, cy);
  ctx.textAlign = 'left';
  return true;
}
