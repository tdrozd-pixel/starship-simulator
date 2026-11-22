// Small helpers for the 2D canvas views. DOM-bound, no three.

// Size the canvas backing store to its CSS box (device pixel ratio capped at 1.5) and return a
// context scaled so drawing uses CSS pixels. Returns {ctx, w, h} in CSS pixels.
export function fitCanvas(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
  const w = Math.max(1, Math.round(canvas.clientWidth));
  const h = Math.max(1, Math.round(canvas.clientHeight));
  const bw = Math.round(w * ratio), bh = Math.round(h * ratio);
  if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, w, h };
}

export const COLORS = {
  bg: '#0b0f16', grid: '#1e2836', axis: '#3a4a60', text: '#9db0c8', dim: '#5d6e86',
  cyan: '#35d7ff', magenta: '#ff5bd6', amber: '#f0b030', green: '#4ccf6a', red: '#e04848', white: '#e8eef8',
};
