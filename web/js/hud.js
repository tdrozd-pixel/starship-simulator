// HUD: cached element refs, textContent only, DOM writes throttled to ~20 Hz.
import { VISUAL, placeholderNote } from './visualParams.js';

const RAD2DEG = 180 / Math.PI;

// Pure formatters (exported for unit tests).
export function formatMet(t) {
  if (!Number.isFinite(t) || t < 0) return 'T+--:--.-';
  const tenths = Math.floor(t * 10 + 1e-6); // round to tenths first, then split
  const m = Math.floor(tenths / 600);
  const rem = tenths - m * 600;
  const s = Math.floor(rem / 10);
  return 'T+' + String(m).padStart(2, '0') + ':' + String(s).padStart(2, '0') + '.' + (rem % 10);
}

export function formatTimeScale(ts) {
  if (typeof ts !== 'number' || !Number.isFinite(ts) || ts <= 0) return 'x-';
  const r = +ts.toFixed(3);
  return 'x' + (Number.isInteger(r) ? r.toFixed(1) : String(r));
}

function signed(v, dp) {
  const s = v.toFixed(dp);
  return v >= 0 ? '+' + s : s;
}

export function formatBodyAttitude(quat) {
  let [w, x, y, z] = quat;
  const norm = Math.hypot(w, x, y, z);
  if (!Number.isFinite(norm) || norm <= 1e-12) return ['+0.00', '+0.00', '+0.00'];
  w /= norm; x /= norm; y /= norm; z /= norm;
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  const vectorNorm = Math.hypot(x, y, z);
  if (vectorNorm <= 1e-12) return ['+0.00', '+0.00', '+0.00'];
  const angleDeg = 2 * Math.atan2(vectorNorm, Math.max(-1, Math.min(1, w))) * RAD2DEG;
  const scale = angleDeg / vectorNorm;
  return [signed(x * scale, 2), signed(y * scale, 2), signed(z * scale, 2)];
}

export function createHud(handlers) {
  const $ = (id) => document.getElementById(id);
  const el = {
    conn: $('conn'), run: $('run'), scenario: $('scenario'), timescale: $('timescale'),
    met: $('met'), alt: $('t-alt'), vs: $('t-vs'), thr: $('t-thr'), gp: $('t-gp'), gy: $('t-gy'),
    ax: $('t-ax'), ay: $('t-ay'), az: $('t-az'),
    prop: $('t-prop'), mass: $('t-mass'), lampEngine: $('lamp-engine'), lampTd: $('lamp-td'),
    lampFin: $('lamp-fin'), waiting: $('waiting'), restart: $('btn-restart'), resetView: $('btn-reset'),
    banner: $('banner'),
  };
  let state = 'connecting', reconnectDelay = 0, errorReason = '';
  let lastWrite = 0;
  let secondsSinceMessage = () => 0;

  el.banner.textContent = placeholderNote(null);
  el.restart.addEventListener('click', () => handlers.onRestart && handlers.onRestart());
  el.resetView.addEventListener('click', () => handlers.onResetView && handlers.onResetView());

  function renderConnection() {
    let text, cls;
    if (state === 'open') {
      if (secondsSinceMessage() > VISUAL.staleSeconds) { text = 'STALE'; cls = 'amber'; }
      else { text = 'CONNECTED'; cls = 'green'; }
    } else if (state === 'reconnecting') { text = 'RECONNECTING in ' + Math.ceil(reconnectDelay) + 's'; cls = 'amber'; }
    else if (state.startsWith('error')) { text = 'ERROR ' + errorReason; cls = 'red'; }
    else if (state === 'closed') { text = 'DISCONNECTED'; cls = 'amber'; }
    else { text = 'CONNECTING'; cls = 'amber'; }
    el.conn.textContent = text;
    el.conn.className = 'badge ' + cls;
    el.restart.disabled = state !== 'open';
  }

  function setLamp(lamp, on) { lamp.classList.toggle('on', on); }

  return {
    setHello(h) {
      el.scenario.textContent = String(h.scenario);
      el.timescale.textContent = formatTimeScale(h.timeScale);
      el.run.textContent = '#' + h.runId;
      el.banner.textContent = placeholderNote(h.vehicle);
    },
    setRun(id) { el.run.textContent = '#' + id; },
    setConnection(s, extra) {
      state = s;
      if (s === 'reconnecting') reconnectDelay = extra || 0;
      if (s.startsWith('error:')) errorReason = s.slice(6);
      renderConnection();
    },
    setError(reason) { errorReason = reason; },
    setStaleSource(fn) { secondsSinceMessage = fn; },
    update(sample, nowMs) {
      if (nowMs - lastWrite < 50) return;
      lastWrite = nowMs;
      renderConnection();
      if (!sample) {
        el.waiting.style.display = 'block';
        el.met.textContent = formatMet(NaN);
        for (const k of ['alt', 'vs', 'thr', 'gp', 'gy', 'ax', 'ay', 'az', 'prop', 'mass']) el[k].textContent = '--';
        setLamp(el.lampEngine, false); setLamp(el.lampTd, false); setLamp(el.lampFin, false);
        return;
      }
      el.waiting.style.display = 'none';
      el.met.textContent = formatMet(sample.simTime);
      el.alt.textContent = sample.pos[2].toFixed(1);
      el.vs.textContent = signed(sample.vel[2], 1);
      el.thr.textContent = (sample.throttle * 100).toFixed(0);
      el.gp.textContent = signed(sample.gimbalPitch * RAD2DEG, 2);
      el.gy.textContent = signed(sample.gimbalYaw * RAD2DEG, 2);
      const attitude = formatBodyAttitude(sample.quat);
      el.ax.textContent = attitude[0];
      el.ay.textContent = attitude[1];
      el.az.textContent = attitude[2];
      el.prop.textContent = sample.propellantMass.toFixed(0);
      el.mass.textContent = sample.totalMass.toFixed(0);
      setLamp(el.lampEngine, sample.engineRunning);
      setLamp(el.lampTd, sample.touchdown);
      setLamp(el.lampFin, sample.halted);
    },
  };
}
