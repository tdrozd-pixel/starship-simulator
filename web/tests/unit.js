// Unit tests for the pure modules. Results go to window.__results and the page.
import { enuToThree, enuHeadingToThreeYaw, threeToEnu, bodyQuatToThree, bodyDirToThreeLocal, exhaustDirBody } from '../js/frames.js';
import { decodeBinary, parseControl, makeRestartMessage, SNAPSHOT_SIZE } from '../js/protocol.js';
import { validateTower, hingePosition, armDirection, armDirectionAngle, armEnd, armHeading } from '../js/towerGeometry.js';
import { enginePositionBody, ENGINE_LABELS } from '../js/engineLayout.js';
import { formatMet, formatTimeScale, formatBodyAttitude } from '../js/hud.js';
import { createTruthClient } from '../js/truthClient.js';
import { createInterpolator } from '../js/interpolator.js';
import { pickMimeType, makeFilename, createFramePacer, bitrateFor } from '../js/recorder.js';
import { buildTelemetry, vehicleNote, resultBanner, TELEMETRY_ROWS, formatEngineThrottles } from '../js/hudModel.js';
import { runGncTests } from './gncUnit.js';

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
  dv.setUint16(4, over.version ?? 3, true);
  dv.setUint16(6, over.type ?? 1, true);
  dv.setUint32(8, 7, true);
  // flags 5 plus engine running bits for engines 0, 2 and 5 (bits 3, 5, 8)
  dv.setUint32(12, 5 | (1 << 3) | (1 << 5) | (1 << 8), true);
  dv.setBigUint64(16, 123456789012n, true);
  for (let i = 0; i < 19; i++) dv.setFloat64(24 + 8 * i, 12.345 + i * 1.25, true);
  for (let i = 0; i < 12; i++) dv.setFloat64(176 + 8 * i, 0.05 * (i + 1), true); // 6 thr, 3 pitch, 3 yaw
  dv.setFloat64(272, over.arm0 ?? 0.1, true);
  dv.setFloat64(280, over.arm1 ?? 0.2, true);
  dv.setUint32(288, over.towerState ?? 3, true);
  dv.setUint32(292, over.catchResult ?? 1, true);
  if (over.flags !== undefined) dv.setUint32(12, over.flags, true);
  return buf;
}

test('decoder', () => {
  const r = decodeBinary(buildSnapshot());
  ok(r.kind === 'snapshot', 'kind snapshot');
  const s = r.snap;
  ok(s.runId === 7, 'runId');
  ok(s.flags === (5 | 8 | 32 | 256) && s.engineRunning && !s.touchdown && s.halted, 'flags');
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
  approxArr(s.engineThrottle, [0.05, 0.1, 0.15, 0.2, 0.25, 0.3], 'engineThrottle');
  approxArr(s.engineGimbalPitch, [0.35, 0.4, 0.45], 'engineGimbalPitch');
  approxArr(s.engineGimbalYaw, [0.5, 0.55, 0.6], 'engineGimbalYaw');
  ok(s.engineRun.join() === 'true,false,true,false,false,true', 'engineRun ' + s.engineRun.join());
  const dv = new DataView(buildSnapshot());
  ok(dv.byteLength === 296 && SNAPSHOT_SIZE === 296, 'size 296');
  approxArr(s.armAngle, [0.1, 0.2], 'armAngle');
  ok(s.towerState === 3 && s.catchResult === 1, 'tower state and result');
  ok(!s.caught && !s.crashed, 'caught/crashed clear');
  const cf = decodeBinary(buildSnapshot({ flags: 5 | (1 << 9) })).snap;
  ok(cf.caught && !cf.crashed, 'caught flag bit 9');
  const xf = decodeBinary(buildSnapshot({ flags: 1 << 10 })).snap;
  ok(xf.crashed && !xf.caught, 'crashed flag bit 10');
  const v1 = decodeBinary(buildSnapshot({ version: 1 }));
  ok(v1.kind === 'error' && /version 1/.test(v1.reason), 'version 1 rejected');
  const bad = new Uint8Array(buildSnapshot()); bad[0] = 0x58;
  ok(decodeBinary(bad.buffer).kind === 'error', 'bad magic');
  const v2 = decodeBinary(buildSnapshot({ version: 2 }));
  ok(v2.kind === 'error' && /version 2/.test(v2.reason), 'version 2 rejected');
  const v4 = decodeBinary(buildSnapshot({ version: 4 }));
  ok(v4.kind === 'error' && /version 4/.test(v4.reason), 'version 4 rejected');
  ok(decodeBinary(buildSnapshot().slice(0, 272)).kind === 'error', 'old v2 length 272 rejected');
  ok(decodeBinary(buildSnapshot().slice(0, 295)).kind === 'error', 'length 295');
  ok(decodeBinary(buildSnapshot({ type: 2 })).kind === 'unknown', 'type 2 unknown');
  ok(decodeBinary(new ArrayBuffer(4)).kind === 'error', 'short');
  ok(parseControl('{"a":1}').a === 1 && parseControl('nope') === null, 'parseControl');
  ok(makeRestartMessage() === '{"type":"restart"}', 'restart msg');
});

// Catch tower hello object (placeholder numbers for tests; the geometry math does not depend on them).
const TOWER = {
  position: [-60, 0], approachDir: [-1, 0], mastHalfWidth: 6, mastHeight: 140, hingeHalfSpacing: 8,
  hingeSetback: 10, armLength: 24, armHalfWidth: 3, armThickness: 2, railHeight: 100,
  armOpenAngle: 20 * Math.PI / 180, armClosedAngle: 14.5 * Math.PI / 180, pinHeight: 30, pinOffset: 5.5,
};

function mk(step, t, over = {}) {
  return Object.assign({
    runId: 1, flags: 0, engineRunning: false, touchdown: false, halted: false,
    step: BigInt(step), simTime: t, pos: [t, 0, 0], vel: [0, 0, 0], quat: [1, 0, 0, 0],
    angRate: [0, 0, 0], throttle: 0, gimbalPitch: 0, gimbalYaw: 0, propellantMass: 100, totalMass: 200,
    engineThrottle: [0, 0, 0, 0, 0, 0], engineGimbalPitch: [0, 0, 0], engineGimbalYaw: [0, 0, 0],
    engineRun: [false, false, false, false, false, false],
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
  s0.msg(JSON.stringify({ type: 'hello', protocol: 3, scenario: 'hop', runId: 3, timeScale: 1, tower: TOWER }));
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
  const g = fakeEnv();
  g.client.connect();
  g.sockets[0].open();
  g.sockets[0].msg(JSON.stringify({ type: 'hello', protocol: 3, runId: 1 }));
  ok(g.errors.length === 1 && /tower/.test(g.errors[0]) && g.hellos.length === 0, 'hello without tower fatal');
  const h2 = fakeEnv();
  h2.client.connect();
  h2.sockets[0].open();
  h2.sockets[0].msg(JSON.stringify({ type: 'hello', protocol: 2, runId: 1, tower: TOWER }));
  ok(h2.errors.length === 1 && /version 2/.test(h2.errors[0]), 'protocol 2 hello fatal');
  const e = fakeEnv();
  e.client.connect();
  e.sockets[0].open();
  e.sockets[0].msg(JSON.stringify({ type: 'hello', protocol: 1 }));
  ok(e.errors.length === 1 && /version 1/.test(e.errors[0]), 'protocol 1 error');
  ok(e.hellos.length === 0, 'hello not dispatched');
  ok(e.liveTimers().length === 0 && e.sockets.length === 1, 'no reconnect after fatal hello');
  ok(e.client.getState().startsWith('error:'), 'error state');
  const f = fakeEnv();
  f.client.connect();
  f.sockets[0].open();
  f.sockets[0].msg(buildSnapshot({ version: 1 }));
  ok(f.errors.length === 1 && f.liveTimers().length === 0 && f.sockets.length === 1, 'binary version error fatal');
});

test('interp per-engine fields', () => {
  const it = createInterpolator({ delaySnapshots: 0 });
  it.push(mk(1, 0, { engineThrottle: [0.4, 0.4, 0.4, 0, 0, 0], engineGimbalPitch: [0, 0.1, -0.1],
    engineGimbalYaw: [0.2, 0, 0], engineRun: [true, true, false, false, false, false] }), 10.0);
  it.push(mk(2, 0.01, { engineThrottle: [0.8, 0.6, 0.4, 0, 0, 0], engineGimbalPitch: [0.1, 0.1, 0.1],
    engineGimbalYaw: [0, 0, 0], engineRun: [false, true, true, false, false, false] }), 10.01);
  const o = it.sample(10.005);
  approxArr(o.engineThrottle, [0.6, 0.5, 0.4, 0, 0, 0], 'engineThrottle lerp');
  approxArr(o.engineGimbalPitch, [0.05, 0.1, 0], 'engineGimbalPitch lerp');
  approxArr(o.engineGimbalYaw, [0.1, 0, 0], 'engineGimbalYaw lerp');
  ok(o.engineRun.length === 6 && o.engineRun[1] === true, 'engineRun held');
  const o2 = it.sample(10.0099);
  ok(o2.engineRun[0] === false && o2.engineRun[2] === true, 'engineRun switches with the later sample: ' + o2.engineRun.join());
  const o3 = it.sample(10.0001);
  ok(o3.engineRun[0] === true && o3.engineRun[2] === false, 'engineRun from earlier sample');
});

test('engine layout', () => {
  const p = [0, 0, 0];
  approxArr(enginePositionBody(0, p), [0, 1.55, 0], 'SL1 at azimuth 90');
  approxArr(enginePositionBody(1, p), [1.55 * Math.cos(210 * Math.PI / 180), 1.55 * Math.sin(210 * Math.PI / 180), 0], 'SL2');
  approxArr(enginePositionBody(3, p), [3.6 * Math.cos(150 * Math.PI / 180), 3.6 * Math.sin(150 * Math.PI / 180), 0], 'RV1 at azimuth 150');
  approxArr(enginePositionBody(4, p), [0, -3.6, 0], 'RV2 at azimuth 270');
  approxArr(enginePositionBody(5, p), [3.6 * Math.cos(30 * Math.PI / 180), 3.6 * Math.sin(30 * Math.PI / 180), 0], 'RV3 at azimuth 30');
  ok(ENGINE_LABELS.join() === 'SL1,SL2,SL3,RV1,RV2,RV3', 'labels');
});

test('hud engine throttles', () => {
  const s = { engineThrottle: [0.85, 0.5, 0.9, 0, 0, 0], engineRun: [true, false, true, false, false, false] };
  ok(formatEngineThrottles(s, 0, 3) === '85 - 90', 'SL line ' + formatEngineThrottles(s, 0, 3));
  ok(formatEngineThrottles(s, 3, 3) === '- - -', 'RV line');
  ok(formatEngineThrottles({}, 0, 3) === '- - -', 'no per-engine data');
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

test('tower geometry', () => {
  ok(validateTower(TOWER) === null, 'valid tower');
  ok(validateTower(undefined) !== null && validateTower({ ...TOWER, armLength: NaN }) !== null &&
    validateTower({ ...TOWER, approachDir: [0, 0] }) !== null, 'invalid towers rejected');
  const h = [0, 0];
  approxArr(hingePosition(TOWER, 0, h), [-70, -8], 'hinge 0 on +b side (b = (0,-1))');
  approxArr(hingePosition(TOWER, 1, h), [-70, 8], 'hinge 1 on -b side');
  approx(armDirectionAngle(TOWER, 0), -TOWER.armOpenAngle, 'open: phi = -open');
  approx(armDirectionAngle(TOWER, TOWER.armOpenAngle + TOWER.armClosedAngle), TOWER.armClosedAngle, 'closed: phi = closed');
  const d = [0, 0];
  // Open: arm 0 points away from the axis (toward +b = south here), arm 1 mirrored (north).
  armDirection(TOWER, 0, 0, d);
  approxArr(d, [-Math.cos(TOWER.armOpenAngle), -Math.sin(TOWER.armOpenAngle)], 'arm 0 open direction');
  armDirection(TOWER, 1, 0, d);
  approxArr(d, [-Math.cos(TOWER.armOpenAngle), Math.sin(TOWER.armOpenAngle)], 'arm 1 open direction');
  // Closed: both converge toward the approach axis (arm 0 now heads toward -b = north).
  const psiC = TOWER.armOpenAngle + TOWER.armClosedAngle;
  armDirection(TOWER, 0, psiC, d);
  ok(d[1] > 0, 'arm 0 closed points inward (north)');
  armDirection(TOWER, 1, psiC, d);
  ok(d[1] < 0, 'arm 1 closed points inward (south)');
  const e = [0, 0];
  armEnd(TOWER, 0, psiC, e);
  approxArr(e, [-70 - 24 * Math.cos(TOWER.armClosedAngle), -8 + 24 * Math.sin(TOWER.armClosedAngle)], 'arm 0 end closed');
  armEnd(TOWER, 1, 0, e);
  approxArr(e, [-70 - 24 * Math.cos(TOWER.armOpenAngle), 8 + 24 * Math.sin(TOWER.armOpenAngle)], 'arm 1 end open');
  approx(armHeading(TOWER, 0, 0), Math.atan2(-Math.sin(TOWER.armOpenAngle), -Math.cos(TOWER.armOpenAngle)), 'heading arm 0 open');
  const hd = armHeading(TOWER, 1, psiC);
  armDirection(TOWER, 1, psiC, d);
  approxArr([Math.cos(hd), Math.sin(hd)], d, 'heading matches direction');
  // A rotated approach direction: hinge offsets rotate with it (a = (0,1) gives b = (-1,0)).
  const rot90 = { ...TOWER, position: [0, 0], approachDir: [0, 1] };
  approxArr(hingePosition(rot90, 0, h), [-8, 10], 'hinge 0 with a = (0,1)');
  approx(enuHeadingToThreeYaw(0.7), 0.7, 'heading to three yaw');
  // three: rotation about Y by the heading turns local +X onto ENU (e, n) = three (e, 0, -n).
  const a = 0.6;
  approxArr([Math.cos(a), -Math.sin(a)], [enuToThree(Math.cos(a), Math.sin(a), 0, [0, 0, 0])[0], enuToThree(Math.cos(a), Math.sin(a), 0, [0, 0, 0])[2]], 'yaw convention');
});

test('interp arm angles', () => {
  const it = createInterpolator({ adaptive: false });
  it.push(mk(1, 1.00, { armAngle: [0, 0.2], towerState: 1, catchResult: 0 }), 10.00);
  it.push(mk(2, 1.01, { armAngle: [0.1, 0.4], towerState: 2, catchResult: 1, caught: true }), 10.01);
  const mid = it.sample(10.0125 + 0.0);
  ok(mid.armAngle[0] >= 0 && mid.armAngle[0] <= 0.1 && mid.armAngle[1] >= 0.2 && mid.armAngle[1] <= 0.4, 'arm angle between samples');
  const q = createInterpolator({ adaptive: false, delaySnapshots: 0 });
  q.push(mk(1, 1.00, { armAngle: [0, 0] }), 10.00);
  q.push(mk(2, 1.02, { armAngle: [0.2, 0.4] }), 10.02);
  const m = q.sample(10.01);
  approx(m.armAngle[0], 0.1, 'arm 0 linear midpoint', 1e-9);
  approx(m.armAngle[1], 0.2, 'arm 1 linear midpoint', 1e-9);
  // Starved: holds the last value (no extrapolation of arm angles).
  const st = q.sample(10.5);
  approx(st.armAngle[0], 0.2, 'arm 0 held when starved');
  approx(st.armAngle[1], 0.4, 'arm 1 held when starved');
  ok(st.towerState === 0 && st.catchResult === 0, 'enum fields default');
  const w = createInterpolator({ adaptive: false, delaySnapshots: 0 });
  w.push(mk(1, 1.00, { towerState: 2, catchResult: 1, caught: true }), 10.00);
  w.push(mk(2, 1.02, { towerState: 3, catchResult: 1, caught: true }), 10.02);
  const e = w.sample(10.02);
  ok(e.towerState === 3 && e.catchResult === 1 && e.caught && !e.crashed, 'discrete tower fields follow nearest sample');
});

test('hud result banner', () => {
  ok(resultBanner(null) === '' && resultBanner({}) === '', 'no banner');
  ok(resultBanner({ caught: true }) === 'CAUGHT' && resultBanner({ crashed: true }) === 'CRASHED', 'banner text');
  const base = { simTime: 1, pos: [0, 0, 0], vel: [0, 0, 0], quat: [1, 0, 0, 0], throttle: 0, gimbalPitch: 0, gimbalYaw: 0, propellantMass: 1, totalMass: 2 };
  ok(buildTelemetry({ ...base, caught: true }).result === 'CAUGHT', 'telemetry result caught');
  ok(buildTelemetry({ ...base, crashed: true }).result === 'CRASHED', 'telemetry result crashed');
  ok(buildTelemetry(base).result === '' && buildTelemetry(null).result === '', 'telemetry result empty');
});

runGncTests({ ok, approx, approxArr, test });

// Feed snapshots with a given arrival-latency pattern and render at 60 Hz; count
// frames where the view stalls (moves < 25% of a normal step) or jumps (> 175%).
function runArrivals(latencyFn, opts = {}) {
  const it = createInterpolator({ delaySnapshots: 2.5 });
  it.reset(1);
  const accel = opts.accel || 0;
  const arrivals = [];
  for (let k = 0; k < 900; k++) arrivals.push({ k, arr: 1 + k * 0.01 + latencyFn(k) });
  for (let i = 1; i < arrivals.length; i++) arrivals[i].arr = Math.max(arrivals[i].arr, arrivals[i - 1].arr);
  let ai = 0, prev = null, stalls = 0, jumps = 0;
  for (let wall = 2; wall < 9.5; wall += 1 / 60) {
    while (ai < arrivals.length && arrivals[ai].arr <= wall) {
      const k = arrivals[ai].k, t = k * 0.01;
      it.push(mk(k + 1, t, { pos: [5 * t + 0.5 * accel * t * t, 0, 0], vel: [5 + accel * t, 0, 0] }), arrivals[ai].arr);
      ai++;
    }
    const s = it.sample(wall);
    if (!s) continue;
    if (prev !== null) {
      const ideal = (5 + accel * s.simTime) / 60, dx = s.pos[0] - prev;
      if (dx < 0.25 * ideal) stalls++;
      if (dx > 1.75 * ideal) jumps++;
    }
    prev = s.pos[0];
  }
  return { stalls, jumps, stats: it.stats() };
}

function lcg(seed) { let s = seed; return () => (s = (s * 1664525 + 1013904223) >>> 0) / 4294967296; }

test('interp adaptive buffer on even delivery stays at the floor', () => {
  const r = runArrivals(() => 0);
  ok(r.stalls === 0 && r.jumps === 0, 'localhost smooth: ' + JSON.stringify(r));
  approx(r.stats.delayMs, 25, 'delay stays at 25 ms', 1e-6);
});

test('interp adaptive buffer absorbs Wi-Fi bursts', () => {
  const rnd = lcg(12345);
  const r = runArrivals(() => (rnd() < 0.08 ? 0.03 + 0.05 * rnd() : 0.002 + 0.006 * rnd()));
  ok(r.stalls === 0 && r.jumps === 0, 'wifi smooth: ' + JSON.stringify(r));
  ok(r.stats.delayMs > 40 && r.stats.delayMs <= 250, 'delay grew: ' + r.stats.delayMs);
});

test('interp adaptive buffer absorbs power-save batching', () => {
  const r = runArrivals((k) => Math.ceil((1 + k * 0.01) / 0.1) * 0.1 + 0.002 - (1 + k * 0.01));
  ok(r.stalls === 0 && r.jumps === 0, 'power save smooth: ' + JSON.stringify(r));
});

test('interp extrapolation blends back without a jump', () => {
  // Accelerating vehicle, so constant-velocity dead reckoning is slightly wrong,
  // and a 90 ms hole in delivery forces extrapolation before the buffer adapts.
  const r = runArrivals((k) => (k >= 150 && k < 159 ? (159 - k) * 0.01 : 0), { accel: 3 });
  ok(r.stats.extrapolatedFrames > 0, 'extrapolated: ' + JSON.stringify(r.stats));
  ok(r.jumps === 0 && r.stalls === 0, 'no jump or stall around the hole: ' + JSON.stringify(r));
});

test('interp extrapolation follows the body rate', () => {
  const it = createInterpolator({ delaySnapshots: 0 });
  const w = 0.5; // [rad/s] about body x
  for (let k = 0; k < 5; k++) {
    const a = w * k * 0.01;
    it.push(mk(k + 1, k * 0.01, { quat: [Math.cos(a / 2), Math.sin(a / 2), 0, 0], angRate: [w, 0, 0] }), 10 + k * 0.01);
  }
  const s = it.sample(10.09); // 50 ms past the newest snapshot (0.04)
  const ang = 2 * Math.atan2(s.quat[1], s.quat[0]);
  approx(ang, w * 0.09, 'extrapolated rotation', 1e-6);
  ok(it.stats().extrapolatedFrames === 1, 'counted');
});

const results = { passed, failed: failures.length, failures };
window.__results = results;
document.getElementById('out').textContent =
  'passed ' + passed + ', failed ' + failures.length + '\n' + failures.join('\n');
