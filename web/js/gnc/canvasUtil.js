// Small helpers for the 2D canvas views. DOM-bound, no three.

// CSS box sizes are cached and kept current by one ResizeObserver, so drawing never reads layout
// (clientWidth/clientHeight force a synchronous layout right after DOM writes). The first call for
// an element reads it once to seed the cache.
const sizes = new WeakMap(); // element -> {w, h} in CSS pixels
let observer = null;
function onResize(entries) {
  for (const e of entries) {
    const s = sizes.get(e.target);
    const box = e.contentBoxSize && e.contentBoxSize[0];
    if (s && box) { s.w = box.inlineSize; s.h = box.blockSize; }
    else if (s) { s.w = e.contentRect.width; s.h = e.contentRect.height; }
  }
}

// Cached CSS size {w, h} of an element (live object, updated by the observer).
export function cssSize(el) {
  let s = sizes.get(el);
  if (!s) {
    s = { w: el.clientWidth, h: el.clientHeight };
    sizes.set(el, s);
    if (typeof ResizeObserver === 'function') {
      if (!observer) observer = new ResizeObserver(onResize);
      observer.observe(el);
    }
  }
  return s;
}

// Re-read the size of an element that was just inserted or shown (one layout read, never in the
// frame loop). The observer keeps the cache current afterwards.
export function measure(el) {
  const s = cssSize(el);
  s.w = el.clientWidth; s.h = el.clientHeight;
}

// Size the canvas backing store to its CSS box (device pixel ratio capped at 1.5) and return a
// context scaled so drawing uses CSS pixels. Returns {ctx, w, h} in CSS pixels. The backing store
// is only resized when the size really changes.
export function fitCanvas(canvas) {
  const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
  const s = cssSize(canvas);
  const w = Math.max(1, Math.round(s.w));
  const h = Math.max(1, Math.round(s.h));
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
