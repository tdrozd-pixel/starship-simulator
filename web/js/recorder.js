// Built-in recorder: composes the 3D canvas and the HUD into a fixed-size frame and
// encodes it with MediaRecorder, so no OS screen recorder is needed.
//
// The pure helpers (pickMimeType, makeFilename, createFramePacer, bitrateFor) have no
// DOM access and are unit tested; createRecorder needs a browser.

// Preferred first: MP4/H.264 plays everywhere and is the only format Safari records.
const MIME_CANDIDATES = Object.freeze([
  { mimeType: 'video/mp4;codecs=avc1.640028', ext: 'mp4' },
  { mimeType: 'video/mp4', ext: 'mp4' },
  { mimeType: 'video/webm;codecs=vp9', ext: 'webm' },
  { mimeType: 'video/webm;codecs=vp8', ext: 'webm' },
  { mimeType: 'video/webm', ext: 'webm' },
]);

export function pickMimeType(isTypeSupported) {
  if (typeof isTypeSupported !== 'function') return null;
  for (const c of MIME_CANDIDATES) {
    try {
      if (isTypeSupported(c.mimeType)) return { mimeType: c.mimeType, ext: c.ext };
    } catch (e) { /* treat as unsupported */ }
  }
  return null;
}

// starship-run12-2026-11-22T14-05-09.mp4 (local time, filesystem-safe).
export function makeFilename(runId, date, ext) {
  const p = (n) => String(n).padStart(2, '0');
  const stamp = date.getFullYear() + '-' + p(date.getMonth() + 1) + '-' + p(date.getDate()) +
    'T' + p(date.getHours()) + '-' + p(date.getMinutes()) + '-' + p(date.getSeconds());
  const run = Number.isFinite(runId) ? 'run' + runId + '-' : '';
  return 'starship-' + run + stamp + '.' + ext;
}

// Encoder bitrate [bit/s] from pixels per second.
export function bitrateFor(width, height, fps, bitsPerPixel) {
  return Math.round(width * height * fps * bitsPerPixel);
}

// Decides which animation frames to render while recording. Drift-free: the
// schedule advances by exactly one period per frame, and if the page stalled for
// more than a period it resyncs to now instead of bursting to catch up.
export function createFramePacer(fps) {
  const period = 1000 / fps;
  let next = null;
  return {
    period,
    due(nowMs) {
      if (next === null) { next = nowMs + period; return true; }
      // Half a millisecond of slack absorbs rAF timestamp jitter.
      if (nowMs + 0.5 < next) return false;
      next += period;
      if (nowMs - next > period) next = nowMs + period;
      return true;
    },
    reset() { next = null; },
  };
}

// Browser part. `drawOverlay(ctx, w, h)` draws the HUD (or nothing) on top.
export function createRecorder({ sourceCanvas, width, height, fps, bitsPerPixel, drawOverlay, onStats }) {
  const format = pickMimeType(globalThis.MediaRecorder && ((t) => MediaRecorder.isTypeSupported(t)));
  if (!format) throw new Error('this browser cannot record video (no supported MediaRecorder format)');

  const canvas = document.createElement('canvas');
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d', { alpha: false });
  const stream = canvas.captureStream(fps);
  const recorder = new MediaRecorder(stream, {
    mimeType: format.mimeType,
    videoBitsPerSecond: bitrateFor(width, height, fps, bitsPerPixel),
  });

  const chunks = [];
  let bytes = 0;
  let startedAt = 0;
  recorder.ondataavailable = (e) => {
    if (e.data && e.data.size > 0) {
      chunks.push(e.data);
      bytes += e.data.size;
      if (onStats) onStats({ bytes });
    }
  };

  // Paint one black frame so the stream never starts with an empty canvas.
  ctx.fillStyle = '#000';
  ctx.fillRect(0, 0, width, height);
  recorder.start(1000);
  startedAt = performance.now();

  return {
    format,
    width, height, fps,
    canvas,  // the composed frame being encoded (read by tests)
    // Call right after renderer.render(): the WebGL drawing buffer is still valid
    // within the same task, so preserveDrawingBuffer is not needed.
    captureFrame() {
      ctx.drawImage(sourceCanvas, 0, 0, width, height);
      if (drawOverlay) drawOverlay(ctx, width, height);
    },
    elapsed() { return (performance.now() - startedAt) / 1000; },
    bytes() { return bytes; },
    stop() {
      return new Promise((resolve, reject) => {
        recorder.onstop = () => {
          for (const t of stream.getTracks()) t.stop();
          resolve({
            blob: new Blob(chunks, { type: format.mimeType.split(';')[0] }),
            mimeType: format.mimeType,
            ext: format.ext,
            durationS: (performance.now() - startedAt) / 1000,
          });
        };
        recorder.onerror = (e) => reject(e.error || new Error('MediaRecorder error'));
        if (recorder.state === 'inactive') recorder.onstop();
        else recorder.stop();
      });
    },
  };
}
