import * as THREE from 'three';
import { VISUAL } from './visualParams.js';
import { createInterpolator } from './interpolator.js';
import { createTruthClient } from './truthClient.js';
import { buildScene } from './sceneBuilder.js';
import { createVehicleView } from './vehicleView.js';
import { createCameraRig } from './cameraRig.js';
import { createHud } from './hud.js';
import { buildTelemetry } from './hudModel.js';
import { drawHud } from './hudCanvas.js';
import { createRecorder, createFramePacer, pickMimeType, makeFilename } from './recorder.js';

const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
const scene = buildScene();
const vehicle = createVehicleView();
scene.add(vehicle.root);
const rig = createCameraRig(renderer, canvas);

const interp = createInterpolator({
  delaySnapshots: VISUAL.interpDelaySnapshots,
  bufferSeconds: VISUAL.bufferSeconds,
  offsetWindowSeconds: VISUAL.offsetWindowSeconds,
});

let client = null;
let lastRunId = null;
const hud = createHud({
  onRestart: () => client && client.sendRestart(),
  onResetView: () => rig.reset(),
  onRecordToggle: () => toggleRecording(),
});

const url = (location.protocol === 'https:' ? 'wss' : 'ws') + '://' + location.host + '/ws/truth';
client = createTruthClient({
  url,
  onHello: (h) => {
    hud.setHello(h);
    interp.setTimeScale(h.timeScale);
    interp.setSnapshotRate(h.snapshotRateHz);
    interp.reset(h.runId);
    lastRunId = h.runId;
  },
  onRunStarted: (m) => { interp.reset(m.runId); hud.setRun(m.runId); lastRunId = m.runId; },
  onSnapshot: (s) => interp.push(s, performance.now() / 1000),
  onState: (s, extra) => hud.setConnection(s, extra),
  onProtocolError: (r) => hud.setError(r),
});
hud.setStaleSource(() => client.secondsSinceMessage());
client.connect();

// ---- Built-in recorder ---------------------------------------------------------
// Records the canvas with the HUD drawn into each frame, at a fixed size and rate,
// so no OS screen recorder is needed (OS capture plus a full-rate, full-DPR WebGL
// canvas is what made recording lag).

const canRecord = typeof HTMLCanvasElement.prototype.captureStream === 'function' &&
  pickMimeType(globalThis.MediaRecorder && ((t) => MediaRecorder.isTypeSupported(t))) !== null;
hud.setRecordSupported(canRecord, 'This browser cannot record the canvas (needs MediaRecorder and canvas.captureStream).');

let recording = null;  // { recorder, pacer, includeHud }
let currentSample = null;
let stopping = false;
let lastStatusMs = 0;

function saveBlob(blob, name) {
  const href = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = href;
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(href), 10000);
}

async function toggleRecording() {
  if (stopping) return;
  if (!recording) {
    const opts = hud.recordOptions();
    rig.setFixedSize(opts.width, opts.height);
    try {
      const recorder = createRecorder({
        sourceCanvas: canvas,
        width: opts.width,
        height: opts.height,
        fps: opts.fps,
        bitsPerPixel: VISUAL.recordBitsPerPixel,
        drawOverlay: opts.includeHud
          ? (ctx, w, h) => drawHud(ctx, w, h, buildTelemetry(currentSample), hud.status())
          : null,
      });
      recording = { recorder, pacer: createFramePacer(opts.fps) };
      window.__viewStats.recordCanvas = recorder.canvas;
      hud.setRecording(true);
    } catch (e) {
      rig.clearFixedSize();
      hud.setRecordStatus(String(e.message || e), true);
    }
    return;
  }
  stopping = true;
  const { recorder } = recording;
  recording = null;
  try {
    const result = await recorder.stop();
    saveBlob(result.blob, makeFilename(lastRunId, new Date(), result.ext));
  } catch (e) {
    hud.setRecordStatus('recording failed: ' + String(e.message || e), true);
  } finally {
    rig.clearFixedSize();
    hud.setRecording(false);
    stopping = false;
  }
}

function updateRecordStatus(nowMs) {
  if (!recording || nowMs - lastStatusMs < 250) return;
  lastStatusMs = nowMs;
  const r = recording.recorder;
  const t = Math.floor(r.elapsed());
  const mb = r.bytes() / 1e6;
  const text = 'REC ' + String(Math.floor(t / 60)).padStart(2, '0') + ':' + String(t % 60).padStart(2, '0') +
    '  ' + mb.toFixed(0) + ' MB  ' + r.width + 'x' + r.height + ' ' + r.fps + ' fps';
  const warn = r.bytes() > VISUAL.recordMemoryWarnBytes;
  hud.setRecordStatus(warn ? text + '  (large: stop soon, held in memory)' : text, warn);
}

// Background tabs stop requestAnimationFrame, so the recording would freeze.
document.addEventListener('visibilitychange', () => {
  if (recording && document.hidden) hud.setRecordStatus('Recording paused: keep this tab visible', true);
});

// ?debug=1: link health of the truth stream (jitter buffer), for diagnosing remote viewers.
const debugEl = new URLSearchParams(location.search).get('debug') === '1' ? document.createElement('div') : null;
let lastDebugMs = 0;
if (debugEl) {
  debugEl.id = 'netdebug';
  document.getElementById('hud').appendChild(debugEl);
}
function updateDebug(nowMs) {
  if (!debugEl || nowMs - lastDebugMs < 250) return;
  lastDebugMs = nowMs;
  const st = interp.stats();
  debugEl.textContent = 'buffer ' + st.delayMs.toFixed(0) + ' ms (target ' + st.targetDelayMs.toFixed(0) +
    ')  jitter ' + st.jitterMs.toFixed(0) + ' ms  starved ' + st.starvedFrames + ' (extrapolated ' +
    st.extrapolatedFrames + ', held ' + st.heldFrames + ')';
}

// Test hook (web/tests/run_web_tests.py): rendered frame count.
window.__viewStats = { renders: 0 };

// ---- Render loop ---------------------------------------------------------------

let last = performance.now();
function frame(now) {
  requestAnimationFrame(frame);
  // While recording, render only the frames the recording needs.
  if (recording && !recording.pacer.due(now)) {
    hud.update(currentSample, now);
    return;
  }
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  const s = interp.sample(now / 1000);
  if (s) {
    vehicle.apply(s);
    rig.update(vehicle.root.position, dt);
  } else {
    vehicle.hide();
    rig.update(null, dt);
  }
  hud.update(s, now);
  renderer.render(scene, rig.camera);
  window.__viewStats.renders++;
  updateDebug(now);
  currentSample = s;
  if (recording) {
    recording.recorder.captureFrame();
    updateRecordStatus(now);
  }
}
requestAnimationFrame(frame);
