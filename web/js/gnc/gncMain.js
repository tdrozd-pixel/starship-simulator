// GNC console page. Connects ONLY to /ws/telemetry; never opens the truth stream.
// Everything shown comes from downlinked packets; time shown is onboard time.
// The page shows one screen at a time (screens are data: regimes.js). The store collects every
// packet regardless of the screen; only the screen being shown builds DOM, draws canvases or charts.
import { createWsClient } from '../wsClient.js';
import { decodeGroundFrame } from './ccsds.js';
import { APID, APID_NAMES } from './gncPackets.js';
import { createTelemetryStore } from './telemetryStore.js';
import { attitudeAngles, buildModel, createModeTimeline } from './gncModel.js';
import { createChartSet, chartStats } from './charts.js';
import { createMapView } from './mapView.js';
import { createPerfMonitor } from './perfMonitor.js';
import { buildScreen } from './widgets.js';
import { drawTimeline } from './timelineView.js';
import { drawAttitudeMini } from './attitudeView.js';
import { createNavInterpolator } from './navInterpolator.js';
import {
  REGIMES, DEFAULT_REGIME, getRegime, isLive, screensOf, getScreen, screenFromKey, formatHash, parseHash,
  initialSelection, followStep, selectManual, resumeFollow,
} from './regimes.js';

const $ = (sel) => document.querySelector(sel);
const wall = () => performance.now() / 1000;
const NOTICE_MS = 6000;

// localStorage may be blocked or throw: every access is guarded.
const lsGet = (k) => { try { return localStorage.getItem(k); } catch (e) { return null; } };
const lsSet = (k, v) => { try { localStorage.setItem(k, v); } catch (e) { /* ignore */ } };

const store = createTelemetryStore();
const timeline = createModeTimeline();
const navInterpolator = createNavInterpolator();
const map = createMapView();
let perf = null; // null: zero cost when off
const link = { fps: NaN, rates: {} };
const apids = Object.entries(APID_NAMES);

// ---- Selection state: regime, screen, follow ----
let sel = { follow: lsGet('gnc.follow') !== '0', ...initialSelection(location.hash, lsGet('gnc.sel')) };
if (parseHash(location.hash)) sel.follow = false; // a deep link pins its screen
let lastMode = null;
let windowSec = 60, engine = 0;
let screenRt = null, chartSet = null;
const host = $('#screen-host');
const tabsEl = $('#tabs');

const textEls = new Map();
for (const el of $('#bar').querySelectorAll('[data-k]')) textEls.set(el, el.dataset.k);
const modeBadge = $('#mode-badge');
const timelineEl = $('#timeline');
const connEl = $('#conn');
function setConn(text, cls) { connEl.textContent = text; connEl.className = 'badge ' + cls; }

// ---- Popovers ----
const pops = [];
function addPopover(btn, pop) {
  pops.push({ btn, pop });
  btn.addEventListener('click', () => {
    const open = pop.hidden;
    closePopovers();
    pop.hidden = !open;
    btn.setAttribute('aria-expanded', String(open));
    if (open && pop.id === 'stale-pop') updateStalePop(buildModel(store, wall(), link));
  });
}
function closePopovers() { for (const p of pops) { p.pop.hidden = true; p.btn.setAttribute('aria-expanded', 'false'); } }
document.addEventListener('pointerdown', (e) => { if (!e.target.closest('.pop-anchor')) closePopovers(); });

// ---- Regime menu, tabs, follow ----
const regimePop = $('#regime-pop');
function buildRegimeMenu() {
  for (const r of REGIMES) {
    const b = document.createElement('button');
    b.className = 'regime-item' + (r.status === 'live' ? '' : ' planned');
    b.dataset.regime = r.id;
    const name = document.createElement('b');
    name.textContent = r.label;
    const desc = document.createElement('span');
    desc.textContent = r.status === 'live' ? r.description : r.description + ' ' + (r.note || 'Not yet simulated.');
    b.append(name, desc);
    if (r.status !== 'live') { b.disabled = true; b.title = r.note || 'Not yet simulated'; }
    b.addEventListener('click', () => { closePopovers(); applySelection(selectManual(sel, r.id, sel.screen), true); });
    regimePop.append(b);
  }
}

function renderTabs() {
  tabsEl.replaceChildren();
  screensOf(sel.regime).forEach((s, i) => {
    const b = document.createElement('button');
    b.setAttribute('role', 'tab');
    b.dataset.screen = s.id;
    b.setAttribute('aria-selected', String(s.id === sel.screen));
    b.textContent = (i + 1) + '  ' + s.title;
    b.addEventListener('click', () => applySelection(selectManual(sel, sel.regime, s.id), true));
    tabsEl.append(b);
  });
  for (const b of regimePop.children) b.classList.toggle('current', b.dataset.regime === sel.regime);
  $('#regime-label').textContent = getRegime(sel.regime).label.toUpperCase();
  const fb = $('#follow-btn');
  fb.textContent = sel.follow ? 'FOLLOW PHASE: ON' : 'FOLLOW PHASE: OFF (CLICK TO RESUME)';
  fb.setAttribute('aria-pressed', String(sel.follow));
  fb.classList.toggle('off', !sel.follow);
}

let noticeTimer = 0;
function showNotice(text) {
  const el = $('#notice');
  el.textContent = text;
  clearTimeout(noticeTimer);
  noticeTimer = setTimeout(() => { el.textContent = ''; }, NOTICE_MS);
}

// Remember the operator's choice (manual selections only) and keep the URL in step: the hash is
// present only while pinned, so a reload while following keeps following.
function persist(manual) {
  lsSet('gnc.follow', sel.follow ? '1' : '0');
  if (manual) lsSet('gnc.sel', formatHash(sel.regime, sel.screen));
  try {
    history.replaceState(null, '', sel.follow ? location.pathname + location.search : location.pathname + location.search + formatHash(sel.regime, sel.screen));
  } catch (e) { /* ignore */ }
}

function applySelection(next, manual) {
  const changed = next.regime !== sel.regime || next.screen !== sel.screen;
  sel = next;
  persist(manual);
  if (changed || !screenRt) showScreen(); else renderTabs();
}

function showScreen() {
  if (screenRt) screenRt.destroy();
  chartSet = createChartSet(store, { windowSec, engine });
  const spec = getScreen(sel.regime, sel.screen);
  screenRt = buildScreen(spec, { store, charts: chartSet, map, timeline, link, apids }, host);
  renderTabs();
  refresh();
}

function toggleFollow() {
  const next = sel.follow ? { ...sel, follow: false } : resumeFollow(sel, store.latest[APID.FSW] ? store.latest[APID.FSW].mode : -1);
  lastMode = null; // the jump back is silent
  applySelection(next, false);
}

function onMode(mode) {
  const r = followStep(sel, mode, lastMode !== null);
  lastMode = mode;
  if (r.state !== sel) { applySelection(r.state, false); if (r.notice) showNotice(r.notice); }
}

// ---- Link and diagnostics ----
let linkT = wall(), linkN = 0;
const prevCounts = {};
function updateLink(now) {
  const dt = now - linkT;
  if (dt < 1) return;
  link.fps = (store.totalReceived - linkN) / dt;
  linkN = store.totalReceived;
  for (const [apid] of apids) {
    const c = store.counts[apid];
    link.rates[apid] = c ? (c.received - (prevCounts[apid] || 0)) / dt : NaN;
    prevCounts[apid] = c ? c.received : 0;
  }
  linkT = now;
}

function resetViews() {
  timeline.reset();
  map.reset();
  navInterpolator.reset();
  linkN = 0;
  for (const k of Object.keys(prevCounts)) delete prevCounts[k];
  lastMode = null;
  if (chartSet) chartSet.drawAll();
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
  if (pkt.apid === APID.FSW) { timeline.update(d.mode, pkt.time); onMode(d.mode); }
  else if (pkt.apid === APID.NAV) map.addNav(pkt.time, d.east, d.north);
  if (pkt.apid === APID.NAV) navInterpolator.push(d, wall());
  else if (pkt.apid === APID.GNSS && d.fixType > 0) map.addFix(d.latitude, d.longitude, d.height);
}

// ---- Header ----
const staleBtn = $('#stale-btn'), stalePop = $('#stale-pop');
const staleRows = {};
for (const [, name] of apids) {
  const row = document.createElement('div');
  row.className = 'stale-row';
  row.append(document.createElement('span'), document.createElement('b'));
  row.firstChild.textContent = name;
  stalePop.append(row);
  staleRows[name] = row.lastChild;
}
function updateStalePop(model) {
  for (const [name, state] of Object.entries(model.stale)) {
    const e = staleRows[name];
    const txt = state === 'ok' ? 'OK' : state === 'stale' ? 'STALE' : 'NO DATA';
    if (e.textContent !== txt) { e.textContent = txt; e.className = state; }
  }
}
function updateHeader(model) {
  for (const [el, k] of textEls) {
    const v = model.text[k];
    if (v !== undefined && el.textContent !== v) el.textContent = v;
  }
  modeBadge.style.background = model.modeColor;
  const states = Object.values(model.stale);
  const nStale = states.filter((s) => s === 'stale').length;
  const txt = states.every((s) => s === 'none') ? 'TLM --' : nStale ? 'TLM STALE ' + nStale : 'TLM OK';
  if (staleBtn.textContent !== txt) { staleBtn.textContent = txt; staleBtn.className = 'chip ' + (nStale ? 'bad' : txt === 'TLM OK' ? 'ok' : ''); }
  if (!stalePop.hidden) updateStalePop(model);
}

// ---- Update loop ----
// Text panels refresh at 20 Hz, screen canvases at 10 Hz, and charts remain staggered at 4 Hz.
// The always-visible attitude indicator is interpolated and drawn separately at display rate.
const TICK_MS = 50;
let panelTicks = 0, lastSegs = -1, needFull = false;
// canvases: also redraw the screen's canvases and the timeline.
function updatePanels(now, canvases) {
  updateLink(now);
  const nav = navInterpolator.sample(now);
  const model = buildModel(store, now, link, nav);
  updateHeader(model);
  if (screenRt) screenRt.update(model, canvases);
  if (canvases && (panelTicks % 4 === 0 || timeline.segs.length !== lastSegs)) { lastSegs = timeline.segs.length; drawTimeline(timelineEl, timeline.segs, store.met); }
}
function refresh() {
  updatePanels(wall(), true);
  if (chartSet) chartSet.drawAll();
}

function tick() {
  if (document.hidden) { needFull = true; return; }
  const ms = performance.now();
  if (perf) perf.frame(ms);
  if (needFull) { needFull = false; refresh(); return; }
  panelTicks++;
  const t0 = perf ? performance.now() : 0;
  updatePanels(wall(), (panelTicks & 1) === 0);
  if (perf) perf.record('panels', performance.now() - t0);
  const chartT0 = perf ? performance.now() : 0;
  if (chartSet.update(ms) && perf) perf.record('charts', performance.now() - chartT0);
}

const miniAttitude = $('#attitude-mini-canvas');
const miniAngles = { tiltE: 0, tiltN: 0, roll: 0 };
function drawMiniFrame() {
  if (!document.hidden) {
    const now = wall();
    const nav = navInterpolator.sample(now);
    const attitude = nav ? attitudeAngles(nav.attitude, miniAngles) : null;
    drawAttitudeMini(miniAttitude, attitude);
  }
  requestAnimationFrame(drawMiniFrame);
}

// ---- Menu and keys ----
function setPerf(on) {
  if (on && !perf) perf = createPerfMonitor(() => store.totalReceived);
  else if (!on && perf) { perf.destroy(); perf = null; }
  $('#perf-toggle').checked = on;
}
$('#win').addEventListener('change', (e) => { windowSec = Number(e.target.value); showScreen(); });
$('#eng').addEventListener('change', (e) => { engine = e.target.value === 'all' ? 'all' : Number(e.target.value); showScreen(); });
$('#perf-toggle').addEventListener('change', (e) => setPerf(e.target.checked));
$('#clear-btn').addEventListener('click', () => { store.reset(); resetViews(); closePopovers(); refresh(); });
$('#follow-btn').addEventListener('click', toggleFollow);
addPopover($('#regime-btn'), regimePop);
addPopover(staleBtn, stalePop);
addPopover($('#menu-btn'), $('#menu'));
document.addEventListener('keydown', (e) => {
  if (e.key === 'Escape') { closePopovers(); return; }
  if (e.ctrlKey || e.metaKey || e.altKey || /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;
  const id = screenFromKey(sel.regime, e.key);
  if (id) applySelection(selectManual(sel, sel.regime, id), true);
});
window.addEventListener('hashchange', () => {
  const s = parseHash(location.hash);
  if (s && (s.regime !== sel.regime || s.screen !== sel.screen)) applySelection(selectManual({ ...sel, follow: false }, s.regime, s.screen), true);
});

// Test hooks: feed a ground frame as if received, and read key displayed values.
window.__gncInject = (arrayBuffer) => {
  const pkt = decodeGroundFrame(arrayBuffer);
  if (pkt.kind === 'packet') ingestPacket(pkt);
  return pkt.kind;
};
// Charts of the screen being shown and how often each chart id has been drawn.
window.__gncCharts = {
  ids: () => (chartSet ? chartSet.ids() : []),
  draws: () => ({ total: chartStats.draws, byId: { ...chartStats.byId } }),
};
window.__gncPerf = () => (perf ? perf.snapshot() : null);
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
    chartWindow: windowSec, regime: sel.regime, screen: sel.screen, follow: sel.follow,
  };
};

buildRegimeMenu();
if (!isLive(sel.regime)) sel = { follow: true, regime: DEFAULT_REGIME, screen: screensOf(DEFAULT_REGIME)[0].id };
setPerf(new URLSearchParams(location.search).get('debug') === '1');
showScreen();
persist(false);
client.connect();
setInterval(tick, TICK_MS);
requestAnimationFrame(drawMiniFrame);
document.addEventListener('visibilitychange', () => { if (!document.hidden) tick(); });
