// HUD: cached element refs, textContent only, DOM writes throttled to ~20 Hz.
// Text comes from hudModel.js, shared with the recorder's canvas HUD.
import { VISUAL } from './visualParams.js';
import { buildTelemetry, formatTimeScale, vehicleNote, TELEMETRY_ROWS } from './hudModel.js';

export { formatMet, formatTimeScale, formatBodyAttitude, signed } from './hudModel.js';

export function createHud(handlers) {
  const $ = (id) => document.getElementById(id);
  const el = {
    conn: $('conn'), run: $('run'), scenario: $('scenario'), timescale: $('timescale'),
    met: $('met'), lampEngine: $('lamp-engine'), lampTd: $('lamp-td'),
    lampFin: $('lamp-fin'), waiting: $('waiting'), restart: $('btn-restart'), resetView: $('btn-reset'),
    banner: $('banner'),
    rec: $('btn-rec'), recRes: $('rec-res'), recFps: $('rec-fps'), recHud: $('rec-hud'), recStatus: $('rec-status'),
  };
  const values = {};
  for (const r of TELEMETRY_ROWS) values[r.key] = $('t-' + r.key);

  let state = 'connecting', reconnectDelay = 0, errorReason = '';
  let lastWrite = 0;
  let secondsSinceMessage = () => 0;
  let recording = false;
  // Latest connection badge, for the recorder's canvas HUD.
  const status = { connText: 'CONNECTING', connClass: 'amber', run: '#-', scenario: '-', timeScale: 'x1.0', banner: '' };

  status.banner = el.banner.textContent = vehicleNote(null);
  el.restart.addEventListener('click', () => handlers.onRestart && handlers.onRestart());
  el.resetView.addEventListener('click', () => handlers.onResetView && handlers.onResetView());
  el.rec.addEventListener('click', () => handlers.onRecordToggle && handlers.onRecordToggle());

  function renderConnection() {
    let text, cls;
    if (state === 'open') {
      if (secondsSinceMessage() > VISUAL.staleSeconds) { text = 'STALE'; cls = 'amber'; }
      else { text = 'CONNECTED'; cls = 'green'; }
    } else if (state === 'reconnecting') { text = 'RECONNECTING in ' + Math.ceil(reconnectDelay) + 's'; cls = 'amber'; }
    else if (state.startsWith('error')) { text = 'ERROR ' + errorReason; cls = 'red'; }
    else if (state === 'closed') { text = 'DISCONNECTED'; cls = 'amber'; }
    else { text = 'CONNECTING'; cls = 'amber'; }
    status.connText = text;
    status.connClass = cls;
    el.conn.textContent = text;
    el.conn.className = 'badge ' + cls;
    el.restart.disabled = state !== 'open';
  }

  function setLamp(lamp, on) { lamp.classList.toggle('on', on); }

  return {
    setHello(h) {
      status.scenario = el.scenario.textContent = String(h.scenario);
      status.timeScale = el.timescale.textContent = formatTimeScale(h.timeScale);
      status.run = el.run.textContent = '#' + h.runId;
      status.banner = el.banner.textContent = vehicleNote(h.vehicle);
    },
    setRun(id) { status.run = el.run.textContent = '#' + id; },
    setConnection(s, extra) {
      state = s;
      if (s === 'reconnecting') reconnectDelay = extra || 0;
      if (s.startsWith('error:')) errorReason = s.slice(6);
      renderConnection();
    },
    setError(reason) { errorReason = reason; },
    setStaleSource(fn) { secondsSinceMessage = fn; },
    // For the recorder: the strings the HUD shows right now.
    status() { return status; },
    update(sample, nowMs) {
      if (nowMs - lastWrite < 50) return;
      lastWrite = nowMs;
      renderConnection();
      const t = buildTelemetry(sample);
      el.waiting.style.display = t.waiting ? 'block' : 'none';
      el.met.textContent = t.met;
      for (const r of TELEMETRY_ROWS) values[r.key].textContent = t.values[r.key];
      setLamp(el.lampEngine, t.engine);
      setLamp(el.lampTd, t.touchdown);
      setLamp(el.lampFin, t.finished);
    },

    // Recorder controls (DOM only: never drawn into the video).
    recordOptions() {
      const [width, height] = el.recRes.value.split('x').map(Number);
      return { width, height, fps: Number(el.recFps.value), includeHud: el.recHud.checked };
    },
    setRecordSupported(supported, reason) {
      el.rec.disabled = !supported;
      el.rec.title = supported ? '' : reason;
      if (!supported) el.recStatus.textContent = 'recording unavailable';
    },
    setRecording(on) {
      recording = on;
      el.rec.textContent = on ? 'Stop' : 'Record';
      el.rec.classList.toggle('recording', on);
      for (const c of [el.recRes, el.recFps, el.recHud]) c.disabled = on;
      if (!on) { el.recStatus.textContent = ''; el.recStatus.className = ''; }
    },
    setRecordStatus(text, warn) {
      if (!recording && !warn) return;
      el.recStatus.textContent = text;
      el.recStatus.className = warn ? 'warn' : 'live';
    },
  };
}
