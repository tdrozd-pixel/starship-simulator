// GNC console page. Connects ONLY to /ws/telemetry; never opens the truth stream.
// Everything shown comes from downlinked packets; time shown is onboard time.
import { createWsClient } from '../wsClient.js';
import { decodeGroundFrame } from './ccsds.js';
import { APID, APID_NAMES, NUM_ENGINES, RAD2DEG } from './gncPackets.js';
import { createTelemetryStore } from './telemetryStore.js';
import { buildModel, createModeTimeline, modeName, MODE_COLORS, fmtSigned } from './gncModel.js';
import { createCharts } from './charts.js';
import { createMapView } from './mapView.js';
import { drawGimbalBox } from './gimbalView.js';
import { fitCanvas, COLORS } from './canvasUtil.js';

const $ = (sel) => document.querySelector(sel);
const wall = () => performance.now() / 1000;

const store = createTelemetryStore();
const timeline = createModeTimeline();
const map = createMapView($('#map'));
const charts = createCharts($('#charts'), store, { windowSec: 60 });
let shipView = null;
try {
  const { createShipView } = await import('./shipView.js');
  shipView = createShipView($('#ship'));
} catch (e) {
  $('#ship').replaceWith(Object.assign(document.createElement('div'), { textContent: 'WebGL unavailable', className: 'note' }));
}

// Bind elements once.
const textEls = new Map();
for (const el of document.querySelectorAll('[data-k]')) textEls.set(el, el.dataset.k);
const lampEls = [...document.querySelectorAll('[data-lamp]')];
const gimbalCanvases = [...document.querySelectorAll('canvas[data-g]')];
const gimbalTexts = [...document.querySelectorAll('[data-gv]')];
const modeBadge = $('#mode-badge');
const staleBox = $('#stale');
const staleEls = {};
for (const name of Object.values(APID_NAMES)) {
  const s = document.createElement('span');
  s.className = 'apid';
  s.textContent = name;
  staleBox.append(s);
  staleEls[name] = s;
}
const connEl = $('#conn');

function setConn(text, cls) { connEl.textContent = text; connEl.className = 'badge ' + cls; }

// Link rate: frames per wall second over the last second or more.
const link = { fps: NaN };
let linkT = wall(), linkN = 0;
function updateLink(now) {
  if (now - linkT >= 1) {
    link.fps = (store.totalReceived - linkN) / (now - linkT);
    linkN = store.totalReceived;
    linkT = now;
  }
}

function resetViews() {
  timeline.reset();
  map.reset();
  linkN = 0;
}

const client = createWsClient({
  url: (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws/telemetry',
  decodeBinary: decodeGroundFrame,
  acceptHello: (m) => (m.stream === 'telemetry' ? null : 'unexpected stream ' + m.stream),
  onData: (pkt) => ingestPacket(pkt),
  onState: (s) => {
    if (s === 'open') setConn('LINK UP', 'green');
    else if (s === 'connecting') setConn('CONNECTING', 'amber');
    else if (s === 'reconnecting') setConn('RECONNECTING', 'amber');
    else if (s.startsWith('error:')) setConn('PROTOCOL ERROR', 'red');
    else setConn('CLOSED', 'red');
  },
  onRunStarted: (m) => { store.reset(); store.runId = m.runId; resetViews(); },
});
function ingestPacket(pkt) {
  const before = store.runId;
  const accepted = store.ingest(pkt, wall());
  if (store.runId !== before) resetViews();
  if (!accepted) return;
  const d = store.latest[pkt.apid];
  if (pkt.apid === APID.FSW) timeline.update(d.mode, pkt.time);
  else if (pkt.apid === APID.NAV) map.addNav(pkt.time, d.east, d.north);
  else if (pkt.apid === APID.GNSS && d.fixType > 0) map.addFix(d.latitude, d.longitude, d.height);
}

function drawTimeline() {
  const { ctx, w, h } = fitCanvas($('#timeline'));
  ctx.fillStyle = COLORS.bg; ctx.fillRect(0, 0, w, h);
  const segs = timeline.segs;
  if (!segs.length) return;
  const t0 = segs[0].t0, t1 = Math.max(store.met, t0 + 1);
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

function gimbalText(cmd, meas) {
  const f = (r) => fmtSigned(r * RAD2DEG, 2);
  const c = cmd ? 'CMD  P ' + f(cmd.pitch) + ' Y ' + f(cmd.yaw) : 'CMD  --';
  const m = meas ? 'MEAS P ' + f(meas.pitch) + ' Y ' + f(meas.yaw) : 'MEAS --';
  return c + '<br>' + (meas && !meas.valid ? '<span class="bad">' + m + ' INVALID</span>' : m);
}

function updatePanels(now) {
  updateLink(now);
  const model = buildModel(store, now, link);
  for (const [el, k] of textEls) {
    const v = model.text[k];
    if (v !== undefined && el.textContent !== v) el.textContent = v;
  }
  modeBadge.style.background = model.modeColor;
  for (const el of lampEls) el.classList.toggle('on', !!model.lamps[el.dataset.lamp]);
  for (const [name, state] of Object.entries(model.stale)) {
    staleEls[name].className = 'apid ' + (state === 'ok' ? 'ok' : state === 'stale' ? 'stale' : '');
    staleEls[name].textContent = name + (state === 'stale' ? ' STALE' : '');
  }
  const gnc = store.latest[APID.GNC], fb = store.latest[APID.FEEDBACK];
  for (let i = 0; i < NUM_ENGINES; i++) {
    const cmd = gnc ? gnc.engines[i] : null, meas = fb ? fb.engines[i] : null;
    drawGimbalBox(gimbalCanvases[i], cmd, meas);
    const html = gimbalText(cmd, meas);
    if (gimbalTexts[i].dataset.last !== html) { gimbalTexts[i].innerHTML = html; gimbalTexts[i].dataset.last = html; }
  }
  const nav = store.latest[APID.NAV];
  map.draw(model.frame, store.latest[APID.CONFIG] || null,
    gnc ? { east: gnc.eastSetpoint, north: gnc.northSetpoint } : null,
    nav ? { east: nav.east, north: nav.north } : null);
  drawTimeline();
}

function render3d() {
  if (!shipView) return;
  const nav = store.latest[APID.NAV];
  shipView.render(nav ? nav.attitude : null, store.latest[APID.GNC] || null, store.latest[APID.FEEDBACK] || null);
}

let t3 = 0, tPanels = 0, tCharts = 0;
function frame(ms) {
  const now = ms / 1000;
  if (now - t3 >= 1 / 30 - 0.002) { t3 = now; render3d(); }
  if (now - tPanels >= 0.1 - 0.002) { tPanels = now; updatePanels(wall()); }
  if (now - tCharts >= 0.2 - 0.002) { tCharts = now; charts.redraw(); }
  requestAnimationFrame(frame);
}

$('#win').addEventListener('change', (e) => charts.setWindow(Number(e.target.value)));
$('#eng').addEventListener('change', (e) => charts.setEngine(e.target.value === 'all' ? 'all' : Number(e.target.value)));
window.addEventListener('resize', () => charts.resize());

// Test hooks: feed a ground frame as if received, and read key displayed values.
window.__gncInject = (arrayBuffer) => {
  const pkt = decodeGroundFrame(arrayBuffer);
  if (pkt.kind === 'packet') ingestPacket(pkt);
  return pkt.kind;
};
window.__gncCharts = charts;
window.__gncState = () => {
  const m = buildModel(store, wall(), link);
  const g = store.latest[APID.GNSS];
  return {
    runId: store.runId, met: m.text.met, mode: m.text.mode, modeValue: m.modeValue,
    fixType: m.text.gnssFix, sats: m.text.gnssSats,
    satsUsed: g ? g.satellitesUsed : null, satsVisible: g ? g.satellitesVisible : null,
    rawDelta: m.raw, navDelta: m.navDelta, text: m.text, lamps: m.lamps, stale: m.stale,
    received: Object.fromEntries(Object.entries(store.counts).map(([a, c]) => [a, { ...c }])),
    segments: timeline.segs.map((s) => ({ ...s })),
    sampleCounts: { nav: store.sampleCount(APID.NAV), gnc: store.sampleCount(APID.GNC) },
    chartWindow: charts.getWindow(),
  };
};

client.connect();
requestAnimationFrame(frame);
