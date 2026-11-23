// Unit tests for the GNC console pure modules, run from unit.js.
import { decodeGroundFrame, decodeCuc } from '../js/gnc/ccsds.js';
import * as P from '../js/gnc/gncPackets.js';
import { makeEnuFrame, geodeticToEnu, enuToGeodetic, geodeticToEcef, bearingDeg, horizontalDistance } from '../js/gnc/geodesy.js';
import { createTelemetryStore } from '../js/gnc/telemetryStore.js';
import { buildModel, formatLatitude, formatLongitude, createModeTimeline, attitudeAngles } from '../js/gnc/gncModel.js';
import { createDecimator, createAligner } from '../js/gnc/decimate.js';
import { createNavInterpolator } from '../js/gnc/navInterpolator.js';
import { expandSeries } from '../js/gnc/chartDefs.js';
import { PARAM_APID } from '../js/gnc/telemetryStore.js';
import { parseParam, isValidParam, formatParam, paramUnit } from '../js/gnc/paramFormat.js';
import * as R from '../js/gnc/regimes.js';
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
const FSW = { mode: 4, flags: 0b10111, fixCount: 12, cycles: 123456, timeInMode: 4.5, prop: 200000, mass: 400000 };

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
    ok(f.mode === 4 && f.enginesOn && f.navValid && f.translating && !f.aligned && f.gnssReady, 'fsw flags');
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
    const [x, y] = st.series(['navigation.altitude'], -Infinity);
    ok(x.length === 10 && Math.abs(x[0] - 0.1) < 1e-4 && Math.abs(x[9] - 0.28) < 1e-4, 'ring keeps newest, ordered: ' + x[0] + ' ' + x[9]);
    approx(y[0], 12.5, 'ring value');
    const [xw] = st.series(['navigation.altitude'], 0.19);
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
    ok(m.text.mode === 'DESCENT' && m.text.met === 'T+01:23.3', 'mode and met ' + m.text.met);
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

  test('regimes: modes, hash, follow', () => {
    for (const m of P.ENUMS.mode) {
      const r = R.regimeForMode(m.value);
      ok(r && R.isLive(r), 'mode ' + m.name + ' maps to a live regime: ' + r);
    }
    ok(R.regimeForMode(0) === 'prelaunch' && R.regimeForMode(1) === 'ascent' && R.regimeForMode(2) === 'ascent'
      && R.regimeForMode(4) === 'landing' && R.regimeForMode(6) === 'landing', 'mode table');
    ok(R.regimeForMode(99) === null, 'unknown mode');
    ok(!R.isLive('staging') && !R.isLive('entry') && R.getRegime('entry').note.includes('17 to 26'), 'planned regimes');
    ok(R.screensOf('staging').length === 0 && R.getScreen('entry', 'overview') === null, 'planned regimes have no screens');
    for (const r of R.REGIMES) if (r.status === 'live') {
      ok(r.screens.length >= 3 && r.screens.some((s) => s.id === 'fsw') && r.screens.some((s) => s.id === 'nav'), r.id + ' screens');
      ok(new Set(r.screens.map((s) => s.id)).size === r.screens.length, r.id + ' unique screen ids');
    }
    // Hash.
    ok(R.formatHash('landing', 'actuators') === '#/landing/actuators', 'format hash');
    const ph = R.parseHash('#/landing/actuators');
    ok(ph && ph.regime === 'landing' && ph.screen === 'actuators', 'parse hash');
    ok(R.parseHash('#/landing').screen === 'overview', 'regime only selects the first screen');
    ok(R.parseHash('#/entry/overview') === null && R.parseHash('#/staging') === null, 'planned regime hash rejected');
    ok(R.parseHash('#/prelaunch/actuators') === null && R.parseHash('#nav') === null && R.parseHash('') === null && R.parseHash(null) === null, 'bad hashes');
    ok(R.initialSelection('#/ascent/gnc', null).screen === 'gnc', 'hash wins');
    ok(R.initialSelection('', '#/landing/nav').regime === 'landing', 'stored selection');
    ok(R.initialSelection('#/bogus', '#/nope').regime === R.DEFAULT_REGIME, 'default');
    ok(R.screenFromKey('ascent', '3') === 'actuators' && R.screenFromKey('prelaunch', '4') === null && R.screenFromKey('ascent', 'x') === null, 'keys');
    // Follow and override.
    const s0 = { follow: true, regime: 'ascent', screen: 'gnc' };
    const f = R.followStep({ ...s0, screen: 'nav' }, 4);
    ok(f.state.regime === 'landing' && f.state.screen === 'nav' && f.notice === 'Phase: ASCENT -> LANDING', 'follow keeps the screen kind: ' + f.notice);
    ok(R.followStep(s0, 2).state === s0 && R.followStep(s0, 2).notice === null, 'same regime: no change');
    const f2 = R.followStep({ follow: true, regime: 'ascent', screen: 'gnc' }, 4);
    ok(f2.state.screen === 'overview', 'screen kind missing in the new regime falls back to the first screen');
    ok(R.followStep({ ...s0, follow: false }, 4).state.regime === 'ascent', 'follow off: no paging');
    ok(R.followStep(s0, 4, false).notice === null, 'quiet switch');
    const m1 = R.selectManual(s0, 'landing', 'nav');
    ok(m1.regime === 'landing' && m1.screen === 'nav' && m1.follow === false, 'manual regime turns follow off');
    ok(R.selectManual(s0, 'ascent', 'nav').follow === true, 'manual screen in the same regime keeps follow');
    ok(R.selectManual(s0, 'entry', 'overview') === s0, 'planned regime cannot be opened');
    const back = R.resumeFollow(m1, 4);
    ok(back.follow && back.regime === 'landing' && back.screen === 'nav', 'resume follow');
  });

  test('screens reference existing parameters', () => {
    let n = 0;
    for (const r of R.REGIMES) {
      for (const s of R.screensOf(r.id)) {
        for (const p of R.screenParams(s)) { n++; ok(isValidParam(p), r.id + '/' + s.id + ' parameter ' + p); }
        for (const p of R.screenChartParams(s)) ok(PARAM_APID[p] !== undefined, r.id + '/' + s.id + ' chart parameter stored: ' + p);
      }
    }
    ok(n > 80, 'walked ' + n + ' parameters');
    ok(!isValidParam('navigation.bogus') && !isValidParam('nope.altitude') && !isValidParam('navigation.attitude[4]')
      && !isValidParam('guidance_control.engines[3].pitch') && !isValidParam('guidance_control.engines[0].bogus')
      && !isValidParam('fsw_status.flags.bogus') && !isValidParam('derived.bogus') && !isValidParam('navigation.altitude[0]'), 'invalid parameters rejected');
    ok(isValidParam('fsw_status.flags.enginesOn') && isValidParam('guidance_control.engines[2].throttle') && isValidParam('derived.tilt'), 'valid parameters');
  });

  test('parameter formatting', () => {
    const st = createTelemetryStore();
    const add = (apid, user, seq, time) => st.ingest(decodeGroundFrame(K.packFrame({ apid, user, seq, time, groundTime: time + 0.1 })), 10);
    const model = () => buildModel(st, 10, { fps: NaN });
    ok(formatParam({ param: 'navigation.altitude' }, st, model()) === '--', 'no data');
    add(0x100, K.packFsw(FSW), 0, 5); add(0x110, K.packNav(NAV), 0, 5); add(0x120, K.packGnc(GNC), 0, 5); add(0x101, K.packConfig(CFG), 0, 5);
    const m = model();
    ok(formatParam({ param: 'navigation.altitude' }, st, m) === '12.50', 'scalar');
    ok(formatParam({ param: 'navigation.verticalVelocity', signed: true }, st, m) === '-1.25', 'signed');
    ok(formatParam({ param: 'fsw_status.mode' }, st, m) === 'DESCENT', 'enum label');
    ok(formatParam({ param: 'fsw_status.flags.aligned', bool: ['A', 'B'] }, st, m) === 'B' && formatParam({ param: 'fsw_status.flags.enginesOn' }, st, m) === 'YES', 'flag bits');
    ok(formatParam({ param: 'fsw_status.gnssFixCount' }, st, m) === '12', 'integer default dp');
    ok(formatParam({ param: 'guidance_control.engines[1].throttle' }, st, m) === '60.00', 'fraction shown as percent');
    ok(formatParam({ param: 'guidance_control.engines[0].pitch' }, st, m) === (f32(0.05) * 180 / Math.PI).toFixed(2), 'rad shown as deg');
    ok(formatParam({ param: 'navigation.angularRate[1]' }, st, m) === (f32(-0.02) * 180 / Math.PI).toFixed(2), 'array element, rad/s to deg/s');
    ok(formatParam({ param: 'mission_config.targetLatitude', fmt: 'lat' }, st, m).endsWith(' N'), 'latitude format');
    ok(formatParam({ param: 'derived.tilt' }, st, m) === m.text.tilt && formatParam({ param: 'derived.throttle' }, st, m) === '55', 'derived (mean throttle of engines on)');
    ok(paramUnit({ param: 'navigation.angularRate[0]' }) === 'deg/s' && paramUnit({ param: 'navigation.altitude' }) === 'm' && paramUnit({ param: 'x', unit: 'q' }) === 'q', 'units');
    ok(parseParam('guidance_control.engines[1].throttle').apid === 0x120, 'apid of a record field');
  });

  test('attitude angles', () => {
    const o = { tiltE: 0, tiltN: 0, roll: 0 };
    attitudeAngles([1, 0, 0, 0], o);
    approx(o.tiltE, 0, 'upright east tilt'); approx(o.tiltN, 0, 'upright north tilt'); approx(o.roll, 90, 'body x points east');
    const a = 10 * D2R; // rotate about +y (north) by +a: the nose (+z) leans toward +x (east)
    attitudeAngles([Math.cos(a / 2), 0, Math.sin(a / 2), 0], o);
    approx(o.tiltE, 10, 'tilt east', 1e-6); approx(o.tiltN, 0, 'no north tilt', 1e-6);
    attitudeAngles([Math.cos(a / 2), -Math.sin(a / 2), 0, 0], o); // about -x: nose leans north
    approx(o.tiltN, 10, 'tilt north', 1e-6);
    const h = Math.PI / 2; // 90 deg about +z: body x points north
    attitudeAngles([Math.cos(h / 2), 0, 0, Math.sin(h / 2)], o);
    approx(o.roll, 0, 'roll after 90 deg yaw', 1e-6);
  });

  test('NAV attitude interpolation', () => {
    const it = createNavInterpolator(0.05);
    const sample = (attitude, altitude) => ({
      attitude, angularRate: [0, 0, 0], specificForce: [0, 0, 9.8],
      altitude, verticalVelocity: 0, altimeterRange: altitude, east: 0, north: 0,
      velocityEast: 0, velocityNorth: 0, accelBiasEast: 0, accelBiasNorth: 0,
      altimeterValid: true,
    });
    const half = Math.SQRT1_2;
    it.push(sample([1, 0, 0, 0], 0), 1.0);
    it.push(sample([half, half, 0, 0], 10), 1.1);
    const mid = it.sample(1.1);
    approx(mid.altitude, 5, 'interpolated altitude');
    approx(mid.attitude[0], Math.cos(Math.PI / 8), 'slerp attitude w');
    approx(mid.attitude[1], Math.sin(Math.PI / 8), 'slerp attitude x');
    approx(it.sample(1.2).altitude, 10, 'newest held while waiting for next NAV packet');
    it.reset();
    ok(it.sample(2) === null, 'reset clears interpolation samples');
  });

  test('chart series and decimation', () => {
    const one = expandSeries([{ param: 'a.b[{e}].c', label: 'x', color: '#111', perEngine: true }, { param: 'plain', label: 'p' }], 1);
    ok(one.length === 2 && one[0].param === 'a.b[1].c' && one[0].label === 'E2 x' && one[0].color === '#111', 'single engine');
    const all = expandSeries([{ param: 'a.b[{e}].c', label: 'x', color: '#111', perEngine: true }], 'all');
    ok(all.length === 3 && all[2].param === 'a.b[2].c' && all[0].color !== all[1].color, 'all engines');
    ok(expandSeries([{ param: 'a[{e}]', perEngine: true, allEngines: true }], 0).length === 3, 'allEngines ignores the selector');
    // Decimation: spikes survive, newest sample kept, x monotonic, size bounded.
    const n = 3000, x = new Float64Array(n), y = new Float64Array(n), z = new Float64Array(n);
    for (let i = 0; i < n; i++) { x[i] = i * 0.02; y[i] = Math.sin(i / 40); z[i] = 5; }
    y[1234] = 9; y[2001] = -9;
    const dec = createDecimator();
    const out = dec([x, y, z], 200);
    ok(out.length === 3 && out[0].length <= 204 && out[0].length >= 100, 'decimated size ' + out[0].length);
    ok(Array.prototype.includes.call(out[1], 9) && Array.prototype.includes.call(out[1], -9), 'spikes kept');
    ok(out[0][out[0].length - 1] === x[n - 1], 'newest sample kept');
    let mono = true;
    for (let i = 1; i < out[0].length; i++) if (out[0][i] <= out[0][i - 1]) mono = false;
    ok(mono, 'x monotonic');
    const small = dec([x.subarray(0, 50), y.subarray(0, 50)], 200);
    ok(small[0].length === 50 && small[1][7] === y[7], 'short input passes through');
    // Aligner: tables with identical x concatenate; missing timestamps hold the previous value.
    const al = createAligner();
    const same = al([[new Float64Array([0, 1, 2]), new Float64Array([5, 6, 7])], [new Float64Array([0, 1, 2]), new Float64Array([8, 9, 10])]]);
    ok(same.length === 3 && same[2][1] === 9, 'aligned identical x');
    const gap = al([[new Float64Array([0, 1, 2]), new Float64Array([5, 6, 7])], [new Float64Array([0, 2]), new Float64Array([8, 10])]]);
    ok(gap[0].length === 3 && gap[2][1] === 8 && gap[2][2] === 10, 'missing timestamp holds the previous value');
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
