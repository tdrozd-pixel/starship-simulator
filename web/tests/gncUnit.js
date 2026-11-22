// Unit tests for the GNC console pure modules, run from unit.js.
import { decodeGroundFrame, decodeCuc } from '../js/gnc/ccsds.js';
import * as P from '../js/gnc/gncPackets.js';
import { makeEnuFrame, geodeticToEnu, enuToGeodetic, geodeticToEcef, bearingDeg, horizontalDistance } from '../js/gnc/geodesy.js';
import { createTelemetryStore } from '../js/gnc/telemetryStore.js';
import { buildModel, formatLatitude, formatLongitude, createModeTimeline } from '../js/gnc/gncModel.js';
import { createWsClient } from '../js/wsClient.js';
import * as K from './gncPack.js';

const D2R = Math.PI / 180;
const f32 = Math.fround;

export const NAV = {
  q: [0.9, 0.1, -0.2, 0.3], rate: [0.01, -0.02, 0.03], force: [0.1, 0.2, 9.8], alt: 12.5, vz: -1.25, range: 12.75, rangeValid: true,
  east: 3.5, north: -4.25, ve: 0.5, vn: -0.5, be: 0.01, bn: -0.02,
};
const GNC = {
  altSp: 50, vSp: 0.5, eSp: 1, nSp: -2, aE: 0.25, aN: -0.75, attErr: [0.001, 0.002, 0.003],
  engines: [{ on: true, thr: 0.5, pitch: 0.05, yaw: -0.04 }, { on: true, thr: 0.6, pitch: -0.1, yaw: 0.02 }, { on: false, thr: 0, pitch: 0, yaw: 0 }],
};
const FB = { engines: [{ valid: true, pitch: 0.04, yaw: -0.03 }, { valid: false, pitch: -0.09, yaw: 0.01 }, { valid: true, pitch: 0, yaw: 0 }] };
const CFG = { lat: 25.996603 * D2R, lon: -97.158251 * D2R, h: -22, targetAlt: 50, tol: 1.5, translateAlt: 30 };
const FSW = { mode: 3, flags: 0b10111, fixCount: 12, cycles: 123456, timeInMode: 4.5, prop: 200000, mass: 400000 };

export function runGncTests({ ok, approx, approxArr, test }) {
  test('ccsds decode', () => {
    const user = K.packFsw(FSW);
    const buf = K.packFrame({ runId: 7, groundTime: 12.5, apid: 0x100, seq: 5, time: 10.25, user });
    const r = decodeGroundFrame(buf);
    ok(r.kind === 'packet' && r.runId === 7 && r.apid === 0x100 && r.seq === 5, 'good frame header');
    approx(r.time, 10.25, 'cuc time'); approx(r.groundTime, 12.5, 'ground time');
    ok(r.payload.byteLength === 20, 'payload length');
    ok(decodeGroundFrame(K.packFrame({ apid: 0x100, user, magic: 'XXXX' })).reason === 'bad magic', 'bad magic');
    ok(decodeGroundFrame(new ArrayBuffer(10)).kind === 'error', 'short buffer');
    ok(/version/.test(decodeGroundFrame(K.packFrame({ apid: 0x100, user, version: 2 })).reason), 'wrong version is a version error');
    ok(decodeGroundFrame(K.packFrame({ apid: 0x100, user, ccsdsVersion: 1 })).kind === 'error', 'bad CCSDS version');
    ok(decodeGroundFrame(buf.slice(0, buf.byteLength - 1)).kind === 'error', 'truncated frame');
    const dv = new DataView(new ArrayBuffer(6));
    dv.setUint32(0, 100, false); dv.setUint16(4, 0x8000, false);
    approx(decodeCuc(dv, 0), 100.5, 'cuc fraction');
    // Secondary header flag cleared must be rejected.
    const bad = K.packFrame({ apid: 0x100, user });
    new DataView(bad, 24).setUint16(0, 0x100, false);
    ok(decodeGroundFrame(bad).kind === 'error', 'no secondary header');
  });

  const dec = (apid, user, extra = {}) => {
    const r = decodeGroundFrame(K.packFrame({ apid, user, ...extra }));
    return P.decodePayload(apid, r.payload);
  };

  test('gnc packet decoders', () => {
    const f = dec(0x100, K.packFsw(FSW));
    ok(f.mode === 3 && f.enginesOn && f.navValid && f.translating && !f.aligned && f.gnssReady, 'fsw flags');
    ok(f.gnssFixCount === 12 && f.cycleCount === 123456, 'fsw counts');
    approxArr([f.timeInMode, f.propellant, f.mass], [4.5, 200000, 400000], 'fsw floats');
    const c = dec(0x101, K.packConfig(CFG));
    approxArr([c.targetLatitude, c.targetLongitude, c.targetHeight], [CFG.lat, CFG.lon, -22], 'config f64', 0);
    approxArr([c.targetAltitude, c.alignTolerance, c.translateAltitude], [50, 1.5, 30], 'config f32');
    const n = dec(0x110, K.packNav(NAV));
    approxArr(n.attitude, NAV.q.map(f32), 'nav attitude', 0);
    approxArr(n.angularRate, NAV.rate.map(f32), 'nav rate', 0);
    approxArr(n.specificForce, NAV.force.map(f32), 'nav force', 0);
    approxArr([n.altitude, n.verticalVelocity, n.altimeterRange, n.east, n.north, n.velocityEast, n.velocityNorth, n.accelBiasEast, n.accelBiasNorth],
      [12.5, -1.25, 12.75, 3.5, -4.25, 0.5, -0.5, f32(0.01), f32(-0.02)], 'nav scalars', 0);
    ok(n.altimeterValid === true, 'nav altimeter valid');
    const g = dec(0x120, K.packGnc(GNC));
    approxArr([g.altitudeSetpoint, g.velocitySetpoint, g.eastSetpoint, g.northSetpoint, g.eastAccelCommand, g.northAccelCommand], [50, 0.5, 1, -2, 0.25, -0.75], 'gnc scalars');
    approxArr(g.attitudeError, GNC.attErr.map(f32), 'gnc att err', 0);
    ok(g.engines.length === 3 && g.engines[0].on && !g.engines[2].on, 'gnc engine on');
    approxArr([g.engines[1].throttle, g.engines[1].pitch, g.engines[1].yaw], [f32(0.6), f32(-0.1), f32(0.02)], 'gnc engine 2', 0);
    const b = dec(0x130, K.packFeedback(FB));
    ok(b.engines[0].valid && !b.engines[1].valid, 'feedback valid');
    approxArr([b.engines[1].pitch, b.engines[1].yaw], [f32(-0.09), f32(0.01)], 'feedback engine 2', 0);
    const s = dec(0x140, K.packGnss({ fix: 2, used: 18, visible: 24, fixTime: 99.5, lat: CFG.lat, lon: CFG.lon, h: 3.25, vel: [1, 2, 3], hSigma: 0.02, vSigma: 0.04 }));
    ok(s.fixType === 2 && s.satellitesUsed === 18 && s.satellitesVisible === 24, 'gnss counts');
    approxArr([s.fixTime, s.latitude, s.longitude, s.height], [99.5, CFG.lat, CFG.lon, 3.25], 'gnss f64', 0);
    approxArr(s.velocity, [1, 2, 3], 'gnss vel'); approxArr([s.horizontalSigma, s.verticalSigma], [f32(0.02), f32(0.04)], 'gnss sigma', 0);
    ok(P.decodePayload(0x100, new DataView(new ArrayBuffer(19))) === null, 'wrong user length rejected');
    ok(P.decodePayload(0x999, new DataView(new ArrayBuffer(8))) === null, 'unknown apid');
    ok(P.MODE_NAMES.length === 7 && P.MODE_NAMES[5] === 'FINAL DESCENT' && P.FIX_TYPE_NAMES[2] === 'RTK FIXED', 'name tables');
    const e0 = P.enginePosition(0);
    approx(e0[0], 0, 'engine 1 azimuth 90 x', 1e-12); approx(e0[1], 1.55, 'engine 1 y');
  });

  test('geodesy', () => {
    const f = makeEnuFrame(CFG.lat, CFG.lon, CFG.h);
    const o = geodeticToEnu(f, CFG.lat, CFG.lon, CFG.h, [0, 0, 0]);
    approxArr(o, [0, 0, 0], 'reference maps to zero', 1e-6);
    const g = enuToGeodetic(f, 30, 20, 50, [0, 0, 0]);
    const back = geodeticToEnu(f, g[0], g[1], g[2], [0, 0, 0]);
    approxArr(back, [30, 20, 50], 'round trip', 1e-3);
    // North of the reference by 20 m is a latitude increase of about 20 / 110.6 km per degree.
    ok(g[0] > CFG.lat && g[1] > CFG.lon, 'north and east increase lat and lon');
    approx((g[0] - CFG.lat) * 6335439 / (1 - 0.00669438 * Math.pow(Math.sin(CFG.lat), 2)) ** 1.5, 20, 'meridian arc', 0.05);
    const ecef = geodeticToEcef(0, 0, 0, [0, 0, 0]);
    approxArr(ecef, [6378137, 0, 0], 'equator ecef', 1e-6);
    approx(bearingDeg(0, 1), 0, 'bearing north'); approx(bearingDeg(1, 0), 90, 'bearing east');
    approx(bearingDeg(-1, 0), 270, 'bearing west'); approx(horizontalDistance(3, 4), 5, 'distance');
  });

  test('telemetry store', () => {
    const st = createTelemetryStore(10);
    const push = (apid, user, seq, time, runId = 1) => {
      const r = decodeGroundFrame(K.packFrame({ runId, apid, user, seq, time, groundTime: time + 0.2 }));
      return st.ingest(r, 5);
    };
    for (let i = 0; i < 15; i++) push(0x110, K.packNav(NAV), i, i * 0.02);
    ok(st.sampleCount(0x110) === 10, 'ring capped at capacity');
    const [x, y] = st.series(['alt'], -Infinity);
    ok(x.length === 10 && Math.abs(x[0] - 0.1) < 1e-4 && Math.abs(x[9] - 0.28) < 1e-4, 'ring keeps newest, ordered: ' + x[0] + ' ' + x[9]);
    approx(y[0], 12.5, 'ring value');
    const [xw] = st.series(['alt'], 0.19);
    ok(xw.length === 5 && xw[0] >= 0.19, 'window start');
    ok(st.counts[0x110].received === 15 && st.counts[0x110].lost === 0, 'no loss');
    ok(push(0x110, K.packNav(NAV), 14, 0.28) === false, 'duplicate ignored');
    push(0x110, K.packNav(NAV), 18, 0.36);
    ok(st.counts[0x110].lost === 3 && st.counts[0x110].received === 16, 'gap counted: ' + st.counts[0x110].lost);
    approx(st.latency, 0.2, 'latency', 1e-4);
    // 14-bit wrap.
    const w = createTelemetryStore(10);
    const pw = (seq) => w.ingest(decodeGroundFrame(K.packFrame({ apid: 0x100, user: K.packFsw(FSW), seq, time: 1 })), 0);
    pw(16382); pw(16383); pw(0); pw(2);
    ok(w.counts[0x100].received === 4 && w.counts[0x100].lost === 1, 'wrap gap: ' + JSON.stringify(w.counts[0x100]));
    ok(Math.abs(w.lossPercent() - 20) < 1e-9, 'loss percent');
    // Run change resets.
    ok(push(0x110, K.packNav(NAV), 0, 0.0, 2), 'new run accepted');
    ok(st.runId === 2 && st.sampleCount(0x110) === 1 && st.counts[0x110].received === 1 && st.counts[0x110].lost === 0, 'run id reset');
    // Joined chart columns keep order across APIDs.
    st.reset();
    ok(st.sampleCount(0x110) === 0 && st.totalReceived === 0, 'explicit reset');
  });

  test('gnc model', () => {
    const st = createTelemetryStore();
    const add = (apid, user, seq, time) => st.ingest(decodeGroundFrame(K.packFrame({ apid, user, seq, time, groundTime: time + 0.1 })), 10);
    const empty = buildModel(st, 10, { fps: NaN });
    ok(empty.text.mode === '--' && empty.text.met === 'T+--:--.-' && empty.raw === null, 'empty model');
    add(0x100, K.packFsw(FSW), 0, 83.25);
    add(0x101, K.packConfig(CFG), 0, 83.25);
    add(0x110, K.packNav({ ...NAV, east: 3, north: 4 }), 0, 83.3);
    add(0x120, K.packGnc(GNC), 0, 83.3);
    // 30 m east / 20 m north / 50 m up of the target, via the JS inverse.
    const fr = makeEnuFrame(CFG.lat, CFG.lon, CFG.h);
    const g = enuToGeodetic(fr, 30, 20, 50, [0, 0, 0]);
    add(0x140, K.packGnss({ fix: 2, used: 18, visible: 24, fixTime: 83.0, lat: g[0], lon: g[1], h: g[2], vel: [0, 0, 0], hSigma: 0.02, vSigma: 0.04 }), 0, 83.3);
    const m = buildModel(st, 10.2, { fps: 100 });
    ok(m.text.mode === 'HOVER' && m.text.met === 'T+01:23.3', 'mode and met ' + m.text.met);
    ok(m.text.gnssFix === 'RTK FIXED' && m.text.gnssSats === '18 / 24', 'gnss fix and sats');
    approx(m.raw.east, 30, 'raw east', 1e-3); approx(m.raw.north, 20, 'raw north', 1e-3);
    approx(m.raw.dist, Math.hypot(30, 20), 'raw dist', 1e-3);
    approx(m.raw.bearing, Math.atan2(30, 20) / D2R, 'raw bearing', 1e-3);
    approx(m.navDelta.dist, 5, 'nav dist');
    ok(m.text.gnssAge === '0.30', 'solution age ' + m.text.gnssAge);
    ok(m.lamps.enginesOn && m.lamps.gnssReady && !m.lamps.aligned, 'lamps');
    ok(m.stale.NAV === 'ok' && m.stale.FSW === 'ok', 'fresh');
    ok(buildModel(st, 11.5, null).stale.NAV === 'stale', 'stale after 1 s wall');
    ok(buildModel(st, 11.5, null).stale.CONFIG === 'ok', 'config allowed 2.5 s');
    ok(m.text.linkLat === '100', 'latency ms ' + m.text.linkLat);
    ok(formatLatitude(-0.1) === '5.7295780 S' && formatLongitude(-0.1) === '5.7295780 W' && formatLongitude(0.1).endsWith('E'), 'lat/lon format');
    const tl = createModeTimeline();
    tl.update(0, 0); tl.update(0, 1); tl.update(1, 2); tl.update(2, 3);
    ok(tl.segs.length === 3 && tl.segs[1].t0 === 2, 'timeline segments');
  });

  test('wsClient generic', () => {
    const socks = [], got = [], errs = [];
    class WS { constructor(u) { this.url = u; socks.push(this); } send() {} close() { if (this.onclose) this.onclose(); } }
    const c = createWsClient({
      url: 'ws://x/ws/telemetry', WebSocketImpl: WS, nowFn: () => 0, setTimeoutFn: () => 1, clearTimeoutFn: () => {},
      decodeBinary: decodeGroundFrame, acceptHello: (m) => (m.stream === 'telemetry' ? null : 'wrong stream'),
      onData: (r) => got.push(r), onProtocolError: (r) => errs.push(r), onHello: (h) => got.push(h),
    });
    c.connect(); socks[0].onopen();
    socks[0].onmessage({ data: JSON.stringify({ type: 'hello', protocol: 1, stream: 'telemetry' }) });
    socks[0].onmessage({ data: K.packFrame({ apid: 0x100, user: K.packFsw(FSW) }) });
    ok(got.length === 2 && got[1].apid === 0x100, 'hello and packet dispatched');
    socks[0].onmessage({ data: JSON.stringify({ type: 'hello', protocol: 1, stream: 'truth' }) });
    ok(errs.length === 1 && c.getState().startsWith('error:'), 'wrong stream is fatal');
  });
}
