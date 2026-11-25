// Draws the HUD into a 2D canvas for recorded frames. Mirrors the layout and colors
// of css/hud.css; the text comes from hudModel.js, the same source as the DOM HUD.
import { TELEMETRY_ROWS } from './hudModel.js';

const FONT_STACK = 'ui-monospace, Consolas, Menlo, monospace';
const PANEL = 'rgba(10, 14, 20, 0.62)';
const BADGE = { green: '#4ccf6a', amber: '#f0b030', red: '#e04848' };
const LAMP_ON = '#4ccf6a';
const LAMP_OFF = '#333';
// Layout units are CSS pixels of a 900 px tall window; scaled to the frame height
// so 720p, 1080p and 1440p recordings look the same. (Style.)
const REFERENCE_HEIGHT = 900;

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

function panel(ctx, x, y, w, h, s) {
  ctx.fillStyle = PANEL;
  roundRect(ctx, x, y, w, h, 4 * s);
  ctx.fill();
}

// telemetry: from buildTelemetry(); status: { connText, connClass, run, scenario, timeScale, banner }
export function drawHud(ctx, w, h, telemetry, status) {
  const s = h / REFERENCE_HEIGHT;
  const font = (px, bold) => (bold ? 'bold ' : '') + Math.round(px * s) + 'px ' + FONT_STACK;
  const line = 13 * 1.4 * s;
  const pad = 12 * s;
  const margin = 12 * s;
  ctx.save();
  ctx.textBaseline = 'middle';

  // Status, top left.
  ctx.font = font(13);
  const statusLines = ['Run ' + status.run, 'Scenario ' + status.scenario, 'Time scale ' + status.timeScale];
  ctx.font = font(13, true);
  const badgeW = ctx.measureText(status.connText).width + 12 * s;
  ctx.font = font(13);
  const statusW = Math.max(badgeW, ...statusLines.map((t) => ctx.measureText(t).width)) + 2 * pad;
  const statusH = line * 4 + 16 * s;
  panel(ctx, margin, margin, statusW, statusH, s);
  let y = margin + 8 * s + line / 2;
  ctx.fillStyle = BADGE[status.connClass] || BADGE.amber;
  roundRect(ctx, margin + pad, y - line / 2 + 2 * s, badgeW, line - 4 * s, 3 * s);
  ctx.fill();
  ctx.font = font(13, true);
  ctx.fillStyle = status.connClass === 'red' ? '#fff' : '#000';
  ctx.textAlign = 'left';
  ctx.fillText(status.connText, margin + pad + 6 * s, y);
  ctx.font = font(13);
  ctx.fillStyle = '#fff';
  for (const t of statusLines) {
    y += line;
    ctx.fillText(t, margin + pad, y);
  }

  // Mission elapsed time, top center.
  ctx.font = font(22);
  const metW = ctx.measureText(telemetry.met).width + 2 * pad;
  const metH = 22 * 1.4 * s + 16 * s;
  panel(ctx, (w - metW) / 2, margin, metW, metH, s);
  ctx.fillStyle = '#fff';
  ctx.textAlign = 'center';
  ctx.fillText(telemetry.met, w / 2, margin + metH / 2);

  // Catch outcome, below the elapsed time (same text and colors as the DOM banner).
  if (telemetry.result) {
    ctx.font = font(28, true);
    const rw = ctx.measureText(telemetry.result).width + 48 * s;
    const rh = 28 * 1.4 * s + 12 * s;
    ctx.fillStyle = telemetry.result === 'CAUGHT' ? BADGE.green : BADGE.red;
    roundRect(ctx, (w - rw) / 2, margin + metH + 8 * s, rw, rh, 4 * s);
    ctx.fill();
    ctx.fillStyle = telemetry.result === 'CAUGHT' ? '#000' : '#fff';
    ctx.textAlign = 'center';
    ctx.fillText(telemetry.result, w / 2, margin + metH + 8 * s + rh / 2);
  }

  // Telemetry, top right.
  const telW = 230 * s + 2 * pad;
  const telX = w - margin - telW;
  const lampH = 16 * s;
  const telH = 8 * s + line * (1 + TELEMETRY_ROWS.length) + 8 * s + lampH + (telemetry.waiting ? line : 0) + 12 * s;
  panel(ctx, telX, margin, telW, telH, s);
  y = margin + 8 * s + line / 2;
  ctx.textAlign = 'left';
  ctx.font = font(13, true);
  ctx.fillStyle = '#fff';
  ctx.fillText('TRUTH VIEW', telX + pad, y);
  ctx.font = font(13);
  for (const row of TELEMETRY_ROWS) {
    y += line;
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'left';
    ctx.fillText(row.label, telX + pad, y);
    ctx.fillStyle = row.color || '#fff';
    ctx.textAlign = 'right';
    ctx.fillText(telemetry.values[row.key], telX + telW - pad, y);
  }
  y += line / 2 + 8 * s;
  ctx.font = font(11);
  ctx.textAlign = 'left';
  let x = telX + pad;
  for (const [text, on] of [['ENGINE', telemetry.engine], ['TOUCHDOWN', telemetry.touchdown],
    ['RUN FINISHED', telemetry.finished]]) {
    const lw = ctx.measureText(text).width + 12 * s;
    ctx.fillStyle = on ? LAMP_ON : LAMP_OFF;
    roundRect(ctx, x, y, lw, lampH, 3 * s);
    ctx.fill();
    ctx.fillStyle = on ? '#000' : '#888';
    ctx.fillText(text, x + 6 * s, y + lampH / 2);
    x += lw + 6 * s;
  }
  if (telemetry.waiting) {
    ctx.font = font(13);
    ctx.fillStyle = '#f0b030';
    ctx.fillText('waiting for data', telX + pad, y + lampH + line / 2 + 6 * s);
  }

  // Banner, bottom.
  if (status.banner) {
    const bannerH = 12 * 1.4 * s + 12 * s;
    ctx.fillStyle = 'rgba(10, 14, 20, 0.7)';
    ctx.fillRect(0, h - bannerH, w, bannerH);
    ctx.font = font(12);
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'center';
    ctx.fillText(status.banner, w / 2, h - bannerH / 2, w - 2 * pad);
  }
  ctx.restore();
}
