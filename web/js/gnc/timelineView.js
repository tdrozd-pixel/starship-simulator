// Flight computer mode timeline: one colored segment per mode in onboard time. DOM/canvas only.
import { fitCanvas, COLORS } from './canvasUtil.js';
import { modeName, MODE_COLORS } from './gncModel.js';

// segs: [{mode, t0}] from createModeTimeline; met: newest onboard time [s].
export function drawTimeline(canvas, segs, met) {
  const { ctx, w, h } = fitCanvas(canvas);
  ctx.fillStyle = COLORS.bg; ctx.fillRect(0, 0, w, h);
  if (!segs.length) return;
  const t0 = segs[0].t0, t1 = Math.max(met, t0 + 1);
  ctx.font = '11px ui-monospace, Consolas, monospace'; ctx.textBaseline = 'middle';
  for (let i = 0; i < segs.length; i++) {
    const a = ((segs[i].t0 - t0) / (t1 - t0)) * w;
    const b = ((i + 1 < segs.length ? segs[i + 1].t0 : t1) - t0) / (t1 - t0) * w;
    ctx.fillStyle = MODE_COLORS[segs[i].mode] || '#667788';
    ctx.fillRect(a, 0, Math.max(1, b - a - 1), h);
    const label = modeName(segs[i].mode);
    if (b - a > ctx.measureText(label).width + 10) { ctx.fillStyle = '#000'; ctx.fillText(label, a + 4, h / 2); }
  }
}
