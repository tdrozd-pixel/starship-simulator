// Unit tests for the pure modules. Results go to window.__results and the page.
import { enuToThree, threeToEnu, bodyQuatToThree, bodyDirToThreeLocal, exhaustDirBody } from '../js/frames.js';
import { decodeBinary, parseControl, makeRestartMessage, SNAPSHOT_SIZE } from '../js/protocol.js';
import { formatMet, formatTimeScale, formatBodyAttitude } from '../js/hud.js';
import { createTruthClient } from '../js/truthClient.js';
import { createInterpolator } from '../js/interpolator.js';
import { pickMimeType, makeFilename, createFramePacer, bitrateFor } from '../js/recorder.js';
import { buildTelemetry, vehicleNote, TELEMETRY_ROWS } from '../js/hudModel.js';

const failures = [];
let passed = 0;
function ok(cond, msg) { if (cond) passed++; else failures.push(msg); }
function approx(a, b, msg, tol = 1e-9) { ok(Math.abs(a - b) <= tol, msg + ': got ' + a + ' expected ' + b); }
function approxArr(a, b, msg, tol = 1e-9) { a.forEach((v, i) => approx(v, b[i], msg + '[' + i + ']', tol)); }
function test(name, fn) {
  try { fn(); } catch (e) { failures.push(name + ' threw: ' + e.message); }
}

// Rotate vector v by quaternion (x,y,z,w).
function rot([qx, qy, qz, qw], [vx, vy, vz]) {
  const tx = 2 * (qy * vz - qz * vy), ty = 2 * (qz * vx - qx * vz), tz = 2 * (qx * vy - qy * vx);
  return [vx + qw * tx + (qy * tz - qz * ty), vy + qw * ty + (qz * tx - qx * tz), vz + qw * tz + (qx * ty - qy * tx)];
}
// Rotate by sim quaternion (w,x,y,z).
function rotSim([w, x, y, z], v) { return rot([x, y, z, w], v); }

test('frames', () => {
  const q = bodyQuatToThree(1, 0, 0, 0, [0, 0, 0, 0]);
  approxArr(q, [0, 0, 0, 1], 'identity quat');
  approxArr(rot(q, [0, 1, 0]), [0, 1, 0], 'identity local +Y stays +Y');
  approxArr(enuToThree(1, 2, 3, [0, 0, 0]), [1, 3, -2], 'position');
  approxArr(threeToEnu(1, 3, -2, [0, 0, 0]), [1, 2, 3], 'inverse position');
  const h = Math.SQRT1_2;
  const w = rotSim([h, h, 0, 0], [0, 0, 1]);
  approxArr(w, [0, -1, 0], 'sim +90 about x sends body +z to south');
  const q3 = bodyQuatToThree(h, h, 0, 0, [0, 0, 0, 0]);
  approxArr(rot(q3, [0, 1, 0]), [0, 0, 1], 'converted quat: local +Y to three +Z');
  // consistency: threeToEnu of converted result equals the sim world vector
  const r = rot(q3, [0, 1, 0]);
  approxArr(threeToEnu(r[0], r[1], r[2], [0, 0, 0]), w, 'consistent with sim world');
  const e = exhaustDirBody(0, 0, [0, 0, 0]);
  approxArr(bodyDirToThreeLocal(e[0], e[1], e[2], [0, 0, 0]), [0, -1, 0], 'zero gimbal exhaust local -Y');
  const pitch = exhaustDirBody(Math.PI / 6, 0, [0, 0, 0]);
  approxArr(bodyDirToThreeLocal(pitch[0], pitch[1], pitch[2], [0, 0, 0]),
    [0, -Math.cos(Math.PI / 6), -Math.sin(Math.PI / 6)], 'pitch gimbal direction');
  const yaw = exhaustDirBody(0, Math.PI / 6, [0, 0, 0]);
  approxArr(bodyDirToThreeLocal(yaw[0], yaw[1], yaw[2], [0, 0, 0]),
    [-Math.sin(Math.PI / 6), -Math.cos(Math.PI / 6), 0], 'yaw gimbal direction');
});

// Hand-built snapshot buffer, mirrored in run_web_tests.py cross-check.
function buildSnapshot(over = {}) {
  const buf = new ArrayBuffer(SNAPSHOT_SIZE);
  const dv = new DataView(buf);
  'SSTR'.split('').forEach((c, i) => dv.setUint8(i, c.charCodeAt(0)));
  dv.setUint16(4, over.version ?? 1, true);
  dv.setUint16(6, over.type ?? 1, true);
  dv.setUint32(8, 7, true);
  dv.setUint32(12, 5, true);
  dv.setBigUint64(16, 123456789012n, true);
  for (let i = 0; i < 19; i++) dv.setFloat64(24 + 8 * i, 12.345 + i * 1.25, true);
  return buf;
}

test('decoder', () => {
  const r = decodeBinary(buildSnapshot());
  ok(r.kind === 'snapshot', 'kind snapshot');
  const s = r.snap;
  ok(s.runId === 7, 'runId');
  ok(s.flags === 5 && s.engineRunning && !s.touchdown && s.halted, 'flags');
  ok(typeof s.step === 'bigint' && s.step === 123456789012n, 'step bigint');
  const v = (i) => 12.345 + i * 1.25;
  approx(s.simTime, v(0), 'simTime');
  approxArr(s.pos, [v(1), v(2), v(3)], 'pos');
  approxArr(s.vel, [v(4), v(5), v(6)], 'vel');
  approxArr(s.quat, [v(7), v(8), v(9), v(10)], 'quat');
  approxArr(s.angRate, [v(11), v(12), v(13)], 'angRate');
  approx(s.throttle, v(14), 'throttle');
  approx(s.gimbalPitch, v(15), 'gimbalPitch');
  approx(s.gimbalYaw, v(16), 'gimbalYaw');
  approx(s.propellantMass, v(17), 'propellantMass');
  approx(s.totalMass, v(18), "totalMass");
  const dv = new DataView(buildSnapshot());
  ok(dv.byteLength === 176, 'size 176');
  const bad = new Uint8Array(buildSnapshot()); bad[0] = 0x58;
  ok(decodeBinary(bad.buffer).kind === 'error', 'bad magic');
  const v2 = decodeBinary(buildSnapshot({ version: 2 }));
  ok(v2.kind === 'error' && /version 2/.test(v2.reason), 'version 2');
  ok(decodeBinary(buildSnapshot().slice(0, 175)).kind === 'error', 'length 175');
  ok(decodeBinary(buildSnapshot({ type: 2 })).kind === 'unknown', 'type 2 unknown');
  ok(decodeBinary(new ArrayBuffer(4)).kind === 'error', 'short');
  ok(parseControl('{"a":1}').a === 1 && parseControl('nope') === null, 'parseControl');
  ok(makeRestartMessage() === '{"type":"restart"}', 'restart msg');
});

function mk(step, t, over = {}) {
  return Object.assign({
    runId: 1, flags: 0, engineRunning: false, touchdown: false, halted: false,
    step: BigInt(step), simTime: t, pos: [t, 0, 0], vel: [0, 0, 0], quat: [1, 0, 0, 0],
    angRate: [0, 0, 0], throttle: 0, gimbalPitch: 0, gimbalYaw: 0, propellantMass: 100, totalMass: 200,
  }, over);
}

test('interp midpoint', () => {
  const it = createInterpolator({ delaySnapshots: 0 });
  it.push(mk(1, 0, { throttle: 0 }), 10.0);
  it.push(mk(2, 0.01, { throttle: 1, pos: [10, 0, 0] }), 10.01);
  // offset = 10.0 (min of d). wall 10.005 -> sim time 0.005
  const s = it.sample(10.005);
  approx(s.throttle, 0.5, 'throttle mid');
  approx(s.pos[0], 5, 'pos mid');
});

test('interp slerp', () => {
  const it = createInterpolator({ delaySnapshots: 0 });
  const h = Math.SQRT1_2;
  it.push(mk(1, 0), 5.0);
  it.push(mk(2, 0.01, { quat: [h, h, 0, 0] }), 5.01);
  let s = it.sample(5.005);
  let ang = 2 * Math.acos(Math.abs(s.quat[0]));
  approx(ang, Math.PI / 4, 'slerp 45 deg', 1e-6);
  // negated target gives the same rotation
  const it2 = createInterpolator({ delaySnapshots: 0 });
  it2.push(mk(1, 0), 5.0);
  it2.push(mk(2, 0.01, { quat: [-h, -h, 0, 0] }), 5.01);
  s = it2.sample(5.005);
  ang = 2 * Math.acos(Math.abs(s.quat[0]));
  approx(ang, Math.PI / 4, 'slerp q vs -q', 1e-6);
  ok(s.quat[0] > 0, 'short path keeps w positive');
});

test('interp jitter monotonic', () => {
  const it = createInterpolator();
  let seed = 12345;
  const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
  let wallPush = 0, lastSim = -1, bad = 0, step = 1, nextRender = 0;
  for (let k = 0; k < 600; k++) {
    const arrive = k * 0.01 + (rnd() * 2 - 1) * 0.005;
    while (nextRender < arrive) {
      const s = it.sample(nextRender);
      if (s) { if (s.simTime < lastSim - 1e-12) bad++; lastSim = s.simTime; }
      nextRender += 1 / 60;
    }
    it.push(mk(step++, k * 0.01), arrive);
    wallPush = arrive;
  }
  ok(wallPush > 0, 'ran');
  ok(bad === 0, 'render time monotonic, violations: ' + bad);
  ok(lastSim > 4, 'render time advanced: ' + lastSim);
});

test('interp runId and halted', () => {
  const it = createInterpolator();
  for (let i = 1; i <= 5; i++) it.push(mk(i, i * 0.01), 1 + i * 0.01);
  it.push(mk(1, 0, { runId: 2, pos: [99, 0, 0] }), 3);
  const s = it.sample(3.0);
  ok(s.runId === 2 && s.pos[0] === 99, 'new run clears buffer');
  const h = createInterpolator();
  h.push(mk(1, 0), 1);
  h.push(mk(2, 0.01), 1.01);
  h.push(mk(3, 0.02, { halted: true, flags: 4, pos: [7, 0, 0] }), 1.02);
  const a = JSON.stringify(h.sample(1.5));
  for (let i = 0; i < 3; i++) h.push(mk(3, 0.02, { halted: true, flags: 4, pos: [7, 0, 0] }), 2 + i);
  const b = JSON.stringify(h.sample(5));
  ok(a === b, 'heartbeats give identical sample');
  ok(JSON.parse(a).pos[0] === 7 && JSON.parse(a).halted, 'halted holds newest');
});

test('interp timescale', () => {
  const it = createInterpolator({ delaySnapshots: 0 });
  it.setTimeScale(0.25);
  for (let i = 0; i < 20; i++) it.push(mk(i + 1, i * 0.01), 100 + i * 0.04);
  // offset 100; wall 100.08 -> sim 0.02
  const s = it.sample(100.08);
  approx(s.simTime, 0.02, 'sim time at 0.25x', 1e-9);
  it.setTimeScale(-3);
  const it2 = createInterpolator({ delaySnapshots: 0 });
  it2.setTimeScale(0);
  it2.push(mk(1, 0), 1); it2.push(mk(2, 0.01), 1.01);
  approx(it2.sample(1.005).simTime, 0.005, 'invalid timescale falls back to 1');
});

test('hud formatters', () => {
  const c = [[0, 'T+00:00.0'], [9.96, 'T+00:09.9'], [59.96, 'T+00:59.9'], [60, 'T+01:00.0'],
    [125.4, 'T+02:05.4'], [-1, 'T+--:--.-'], [NaN, 'T+--:--.-'], [Infinity, 'T+--:--.-']];
  for (const [t, e] of c) ok(formatMet(t) === e, 'formatMet(' + t + ') = ' + formatMet(t) + ' expected ' + e);
  ok(formatTimeScale(0.25) === 'x0.25', 'ts 0.25: ' + formatTimeScale(0.25));
  ok(formatTimeScale(1) === 'x1.0', 'ts 1');
  ok(formatTimeScale(0.3333333) === 'x0.333', 'ts third');
  ok(formatTimeScale(undefined) === 'x-' && formatTimeScale(NaN) === 'x-' && formatTimeScale(0) === 'x-', 'ts invalid');
  const half = Math.SQRT1_2;
  const xRotation = formatBodyAttitude([half, half, 0, 0]);
  ok(xRotation[0] === '+90.00' && xRotation[1] === '+0.00' && xRotation[2] === '+0.00',
     'body attitude x rotation');
  const identity = formatBodyAttitude([1, 0, 0, 0]);
  ok(identity.every((v) => v === '+0.00'), 'body attitude identity');
});

function fakeEnv() {
  const env = { sockets: [], timers: [], states: [], hellos: [], runs: [], snaps: [], errors: [] };
  class FakeWS {
    constructor(url) { this.url = url; this.sent = []; this.closed = false; env.sockets.push(this); }
    send(m) { this.sent.push(m); }
    close() { if (this.closed) return; this.closed = true; if (this.onclose) this.onclose(); }
    open() { this.onopen(); }
    msg(d) { this.onmessage({ data: d }); }
  }
  env.client = createTruthClient({
    url: 'ws://x/ws/truth', WebSocketImpl: FakeWS, nowFn: () => 0,
    setTimeoutFn: (fn, ms) => { env.timers.push({ fn, ms, live: true }); return env.timers.length - 1; },
    clearTimeoutFn: (id) => { if (env.timers[id]) env.timers[id].live = false; },
    onHello: (h) => env.hellos.push(h), onRunStarted: (m) => env.runs.push(m),
    onSnapshot: (s) => env.snaps.push(s), onState: (s, x) => env.states.push([s, x]),
    onProtocolError: (r) => env.errors.push(r),
  });
  env.liveTimers = () => env.timers.filter((t) => t.live);
  return env;
}

test('truthClient', () => {
  const e = fakeEnv();
  e.client.connect();
  const s0 = e.sockets[0];
  ok(s0.binaryType === 'arraybuffer', 'binaryType');
  e.client.sendRestart();
  ok(s0.sent.length === 0, 'no restart before open');
  s0.open();
  ok(e.client.getState() === 'open', 'open state');
  s0.msg(JSON.stringify({ type: 'hello', protocol: 1, scenario: 'hop', runId: 3, timeScale: 1 }));
  ok(e.hellos.length === 1 && e.hellos[0].runId === 3, 'hello dispatched');
  s0.msg(JSON.stringify({ type: 'run_started', runId: 4 }));
  ok(e.runs.length === 1 && e.runs[0].runId === 4, 'run_started dispatched');
  s0.msg(JSON.stringify({ type: 'future_thing' }));
  s0.msg('not json');
  ok(e.hellos.length === 1 && e.errors.length === 0, 'unknown and garbage ignored');
  s0.msg(buildSnapshot());
  ok(e.snaps.length === 1 && e.snaps[0].runId === 7, 'binary snapshot dispatched');
  s0.msg(buildSnapshot({ type: 2 }));
  ok(e.snaps.length === 1 && e.errors.length === 0, 'unknown binary type ignored');
  s0.msg(new ArrayBuffer(10));
  ok(e.errors.length === 1 && e.liveTimers().length === 0, 'non-version binary error: dropped, not fatal');
  e.client.sendRestart();
  ok(s0.sent.length === 1 && s0.sent[0] === '{"type":"restart"}', 'restart message exact');

  const delayOf = () => e.states[e.states.length - 1][1];
  const fireLast = () => { const t = e.timers[e.timers.length - 1]; t.live = false; t.fn(); };
  s0.close();
  ok(e.client.getState() === 'reconnecting' && e.liveTimers().length === 1, 'reconnect scheduled');
  let d = delayOf();
  ok(d >= 0.375 && d <= 0.625, 'first backoff in range: ' + d);
  fireLast();
  const s1 = e.sockets[1];
  ok(!!s1, 'new socket after timer');
  s1.close();
  d = delayOf();
  ok(d >= 0.75 && d <= 1.25, 'second backoff in range: ' + d);
  fireLast();
  const s2 = e.sockets[2];
  s2.open();
  s2.close();
  d = delayOf();
  ok(d >= 0.375 && d <= 0.625, 'backoff reset after open: ' + d);
  fireLast();
  e.sockets[e.sockets.length - 1].send = () => { throw new Error('should not send'); };
  e.client.sendRestart();
  ok(true, 'no restart while connecting');
  for (let i = 0; i < 8; i++) { e.sockets[e.sockets.length - 1].close(); fireLast(); }
  const mx = Math.max(...e.states.filter((x) => x[0] === 'reconnecting').map((x) => x[1]));
  ok(mx <= 5 * 1.25 + 1e-9 && mx > 3, 'backoff capped: ' + mx);
});

test('truthClient fatal', () => {
  const e = fakeEnv();
  e.client.connect();
  e.sockets[0].open();
  e.sockets[0].msg(JSON.stringify({ type: 'hello', protocol: 2 }));
  ok(e.errors.length === 1 && /version 2/.test(e.errors[0]), 'protocol 2 error');
  ok(e.hellos.length === 0, 'hello not dispatched');
  ok(e.liveTimers().length === 0 && e.sockets.length === 1, 'no reconnect after fatal hello');
  ok(e.client.getState().startsWith('error:'), 'error state');
  const f = fakeEnv();
  f.client.connect();
  f.sockets[0].open();
  f.sockets[0].msg(buildSnapshot({ version: 2 }));
  ok(f.errors.length === 1 && f.liveTimers().length === 0 && f.sockets.length === 1, 'binary version error fatal');
});

test('interp halt continuity', () => {
  const it = createInterpolator({ delaySnapshots: 2.5 });
  for (let i = 1; i <= 3; i++) it.push(mk(i, i * 0.01), 1 + i * 0.01);
  const before = it.sample(1.05).pos[0];
  it.push(mk(4, 0.04, { halted: true, flags: 4 }), 1.04);
  const after = it.sample(1.05).pos[0];
  ok(Math.abs(after - before) < 1e-9, 'no jump on halt: ' + before + ' vs ' + after);
  let prev = after, mono = true;
  for (let w = 1.05; w < 1.2; w += 0.005) { const p = it.sample(w).pos[0]; if (p < prev - 1e-12) mono = false; prev = p; }
  ok(mono && Math.abs(prev - 0.04) < 1e-9, 'reaches final snapshot then holds: ' + prev);
});

test('recorder mime selection', () => {
  const only = (set) => (t) => set.includes(t);
  const mp4 = pickMimeType(only(['video/mp4', 'video/webm']));
  ok(mp4 && mp4.mimeType === 'video/mp4' && mp4.ext === 'mp4', 'mp4 preferred over webm');
  const h264 = pickMimeType(only(['video/mp4;codecs=avc1.640028', 'video/mp4']));
  ok(h264 && h264.mimeType === 'video/mp4;codecs=avc1.640028', 'explicit H.264 first');
  const webm = pickMimeType(only(['video/webm;codecs=vp8', 'video/webm']));
  ok(webm && webm.mimeType === 'video/webm;codecs=vp8' && webm.ext === 'webm', 'webm fallback');
  ok(pickMimeType(only([])) === null, 'nothing supported');
  ok(pickMimeType(undefined) === null, 'no MediaRecorder');
  ok(pickMimeType(() => { throw new Error('x'); }) === null, 'throwing probe treated as unsupported');
});

test('recorder filename and bitrate', () => {
  const d = new Date(2026, 10, 22, 14, 5, 9);
  ok(makeFilename(12, d, 'mp4') === 'starship-run12-2026-11-22T14-05-09.mp4', 'filename: ' + makeFilename(12, d, 'mp4'));
  ok(makeFilename(null, d, 'webm') === 'starship-2026-11-22T14-05-09.webm', 'filename without run');
  approx(bitrateFor(1920, 1080, 60, 0.1), 12441600, 'bitrate 1080p60');
});

test('recorder frame pacer', () => {
  for (const fps of [30, 60]) {
    const pacer = createFramePacer(fps);
    let frames = 0;
    // 144 Hz display for 2 s
    for (let i = 0; i < 288; i++) if (pacer.due(1000 + i * (1000 / 144))) frames++;
    ok(Math.abs(frames - 2 * fps) <= 1, fps + ' fps from 144 Hz: ' + frames);
  }
  // After a 500 ms stall: one frame, then back on schedule (no burst).
  const pacer = createFramePacer(60);
  let t = 0;
  for (; t < 1000; t += 1000 / 144) pacer.due(t);
  t += 500;
  ok(pacer.due(t), 'frame right after stall');
  let burst = 0;
  for (let k = 1; k <= 3; k++) if (pacer.due(t + k * (1000 / 144))) burst++;
  ok(burst <= 1, 'no catch-up burst after stall: ' + burst);
  // 60 Hz display at 60 fps: every frame (timestamp jitter tolerated).
  const p60 = createFramePacer(60);
  let all = 0;
  for (let i = 0; i < 120; i++) if (p60.due(i * (1000 / 60) + (i % 2 ? 0.3 : -0.3))) all++;
  ok(all >= 119, '60 fps on a 60 Hz display renders every frame: ' + all);
});

test('hud model', () => {
  const none = buildTelemetry(null);
  ok(none.waiting && none.met === 'T+--:--.-' && TELEMETRY_ROWS.every((r) => none.values[r.key] === '--'), 'no sample');
  const s = {
    simTime: 83.25, pos: [1, 2, 49.96], vel: [0, 0, -1.04], quat: [1, 0, 0, 0], throttle: 0.684,
    gimbalPitch: 0.01, gimbalYaw: -0.02, propellantMass: 266757.2, totalMass: 416757.2,
    engineRunning: true, touchdown: false, halted: false,
  };
  const m = buildTelemetry(s);
  ok(!m.waiting && m.met === 'T+01:23.2', 'met ' + m.met);
  ok(m.values.alt === '50.0' && m.values.vs === '-1.0' && m.values.thr === '68', 'alt/vs/thr');
  ok(m.values.gp === '+0.57' && m.values.gy === '-1.15', 'gimbals ' + m.values.gp + ' ' + m.values.gy);
  ok(m.values.ax === '+0.00' && m.values.prop === '266757' && m.values.mass === '416757', 'attitude and mass');
  ok(m.engine && !m.touchdown && !m.finished, 'lamps');
  ok(vehicleNote(null) === '', 'no banner before hello');
  ok(vehicleNote({ name: 'Ship', length: 52, radius: 4.5 }) === 'Physics vehicle: Ship, length 52 m, radius 4.5 m.', 'banner text');
});

const results = { passed, failed: failures.length, failures };
window.__results = results;
document.getElementById('out').textContent =
  'passed ' + passed + ', failed ' + failures.length + '\n' + failures.join('\n');
