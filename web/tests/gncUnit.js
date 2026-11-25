// Unit tests for the GNC console pure modules, run from unit.js.
import { decodeGroundFrame, decodeCuc } from '../js/gnc/ccsds.js';
import * as P from '../js/gnc/gncPackets.js';
import { makeEnuFrame, geodeticToEnu, enuToGeodetic, geodeticToEcef, bearingDeg, horizontalDistance } from '../js/gnc/geodesy.js';
import { createTelemetryStore } from '../js/gnc/telemetryStore.js';
import { lateralSideText, updateAgeState, NAV_STALE_AMBER_S, NAV_STALE_RED_S, buildModel, formatLatitude, formatLongitude, createModeTimeline, attitudeAngles, attitudeErrorMagDeg, engineMargins, landingPrediction, differentialThrottle, rvacSummary, DIFF_THROTTLE_THRESHOLD_PP } from '../js/gnc/gncModel.js';
import { createDecimator, createAligner } from '../js/gnc/decimate.js';
import { createNavInterpolator } from '../js/gnc/navInterpolator.js';
import { expandSeries } from '../js/gnc/chartDefs.js';
import { PARAM_APID } from '../js/gnc/telemetryStore.js';
import { parseParam, isValidParam, formatParam, paramUnit } from '../js/gnc/paramFormat.js';
import * as R from '../js/gnc/regimes.js';
import * as CV from '../js/gnc/corridorView.js';
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
const ES = (state, on, valid, command, thr) => ({ on, state, valid, command, thr });
const ENG = {
  engines: [ES(2, true, true, 0.5, 0.48), ES(2, true, true, 0.6, 0.58), ES(1, true, false, 1.0, 0.9), ES(0, false, true, 0, 0), ES(0, false, true, 0, 0), ES(0, false, true, 0, 0)],
};
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
    ok(P.MODE_NAMES.length === 11 && P.ENUMS.mode.every((e) => P.MODE_NAMES[e.value] === e.label) && P.MODE_NAMES[5] === 'FINAL DESCENT' && P.MODE_NAMES[7] === 'CATCH APPROACH'
      && P.MODE_NAMES[10] === 'CAUGHT' && P.FIX_TYPE_NAMES[2] === 'RTK FIXED', 'name tables');
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
    ok([7, 8, 9, 10].every((v) => R.regimeForMode(v) === 'catch') && R.isLive('catch'), 'catch modes map to the catch regime');
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
    ok(R.screenFromKey('ascent', '3') === 'actuators' && R.screenFromKey('prelaunch', '5') === null && R.screenFromKey('ascent', '5') === 'navhealth' && R.screenFromKey('ascent', 'x') === null, 'keys');
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

  test('attitude error, actuator margins and landing prediction (derived)', () => {
    approx(attitudeErrorMagDeg([0.03, 0.04, 0]), 0.05 * 180 / Math.PI, 'attitude error magnitude', 1e-9);
    ok(Number.isNaN(attitudeErrorMagDeg(null)), 'no packet gives NaN');
    const lim = P.GIMBAL_LIMIT_DEG, d = Math.PI / 180;
    const m = engineMargins({ on: true, throttle: 1, pitch: 0.95 * lim * d, yaw: 0 }, { valid: true, pitch: 0.9 * lim * d, yaw: 0 });
    approx(m.cmdPct, 95, 'cmd percent of limit', 1e-9); approx(m.measPct, 90, 'meas percent', 1e-9);
    approx(m.errDeg, 0.05 * lim, 'tracking error', 1e-9);
    ok(m.warn && !m.atLimit && m.throttleMax && !m.invalid, 'flags: warn at 90 percent, throttle at 100 percent');
    ok(engineMargins({ on: true, throttle: 0.5, pitch: lim * d, yaw: 0 }, { valid: false, pitch: 0, yaw: 0 }).atLimit, 'at the limit');
    const n = engineMargins({ on: true, throttle: 0.5, pitch: 0, yaw: 0 }, { valid: false, pitch: 0, yaw: 0 });
    ok(n.invalid && !n.warn, 'invalid gimbal sensor lamp');
    const none = engineMargins(null, null);
    ok(!none.warn && !none.invalid && Number.isNaN(none.errDeg), 'no data');
    // Landing prediction: 20 m up, descending 4 m/s -> 5 s; drifting 1 m/s east from 2 m east -> 7 m east.
    const nav = { altitude: 20, verticalVelocity: -4, east: 2, north: 0, velocityEast: 1, velocityNorth: 0 };
    const p = landingPrediction(nav, { velocitySetpoint: -2, eastSetpoint: 6, northSetpoint: 0 }, { alignTolerance: 1.5 });
    approx(p.ttg, 5, 'time to go'); approx(p.ttgSp, 10, 'time to go at setpoint rate');
    approx(p.east, 7, 'predicted east'); approx(p.miss, 7, 'miss vs pad'); approx(p.missSp, 1, 'miss vs setpoint');
    ok(p.go === false, 'no-go outside tolerance');
    ok(landingPrediction({ ...nav, velocityEast: -0.3 }, null, { alignTolerance: 1.5 }).go === true, 'go inside tolerance');
    ok(Number.isNaN(landingPrediction({ ...nav, verticalVelocity: 1 }, null, null).ttg), 'ascending gives no time to go');
    approx(landingPrediction({ ...nav, altitude: 0 }, null, null).ttg, 0, 'on the ground');
    ok(landingPrediction(null, null, null) === null, 'no nav');
  });

  test('nav health: update ages, innovation magnitude, stale lamps, stored columns', () => {
    ok(updateAgeState(-1).text === 'NONE YET' && !updateAgeState(-1).amber && !updateAgeState(-1).red, 'no update yet');
    ok(updateAgeState(NaN).text === '--', 'non-finite');
    ok(updateAgeState(0.5).text === '0.50' && !updateAgeState(0.5).amber, 'fresh');
    ok(updateAgeState(NAV_STALE_AMBER_S + 0.1).amber && !updateAgeState(NAV_STALE_AMBER_S + 0.1).red, 'amber');
    ok(updateAgeState(NAV_STALE_RED_S + 0.1).amber && updateAgeState(NAV_STALE_RED_S + 0.1).red, 'red');
    const st = createTelemetryStore();
    const add = (nav, seq, time) => st.ingest(decodeGroundFrame(K.packFrame({ apid: 0x110, user: K.packNav(nav), seq, time, groundTime: time + 0.1 })), 10);
    ok(buildModel(st, 10, { fps: NaN }).text.gnssUpdAge === '--', 'no nav packet');
    add({ ...NAV, health: { psE: 1.5, psN: 2.5, giE: 3, giN: 4, ai: -0.5, gAge: 0.2, aAge: 4 } }, 0, 5);
    let m = buildModel(st, 10, { fps: NaN });
    ok(m.text.gnssInnovMag === '5.00' && m.text.gnssUpdAge === '0.20' && m.text.altUpdAge === '4.00', 'derived text');
    ok(!m.lamps.gnssStaleAmber && m.lamps.altStaleAmber && m.lamps.altStaleRed, 'stale lamps');
    ok(formatParam({ param: 'navigation.positionSigmaNorth', dp: 2 }, st, m) === '2.50' && formatParam({ param: 'derived.altUpdAge' }, st, m) === '4.00', 'formatted');
    add({ ...NAV }, 1, 5.1);
    m = buildModel(st, 10, { fps: NaN });
    ok(m.text.gnssUpdAge === 'NONE YET' && !m.lamps.gnssStaleRed, 'age -1');
    const w = st.series(['navigation.positionSigmaEast', 'navigation.gnssInnovationNorth', 'navigation.altimeterInnovation'], 0);
    ok(w[0].length === 2 && w[1][0] === 1.5 && w[2][0] === 4 && w[3][0] === -0.5, 'columns stored');
    // attitude error columns (deg) and magnitude
    const g = decodeGroundFrame(K.packFrame({ apid: 0x120, user: K.packGnc(GNC), seq: 0, time: 5, groundTime: 5.1 }));
    st.ingest(g, 10);
    const a = st.series(['guidance_control.attitudeError[2]', 'derived.attErrMag'], 0);
    approx(a[1][0], f32(0.003) * 180 / Math.PI, 'stored attitude error z [deg]', 1e-6);
    approx(a[2][0], Math.hypot(f32(0.001), f32(0.002), f32(0.003)) * 180 / Math.PI, 'stored attitude error magnitude', 1e-6);
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

  test('engine_status: decode, store, model, derived differential throttle', () => {
    ok(P.APID.ENGINE === 336 && P.APID_NAMES[336] === 'ENG' && P.USER_LENGTH[0x150] === 72, 'apid and length');
    ok(P.NUM_ENGINES === 3 && P.NUM_POWER_ENGINES === 6 && P.POWER_ENGINE_LABELS.join() === 'SL1,SL2,SL3,RV1,RV2,RV3', 'engine labels');
    const d = P.decodePayload(0x150, new DataView(K.packEngineStatus({ engines: ENG.engines.map((e, i) => (i === 1 ? ES(3, true, false, 0.25, 0.2) : e)) }).buffer));
    ok(d.engines.length === 6 && d.engines[1].state === 3 && d.engines[1].on && !d.engines[1].valid, 'record fields');
    approx(d.engines[1].command, 0.25, 'command'); approx(d.engines[1].throttle, 0.2, 'throttle', 1e-6);
    ok(P.decodePayload(0x150, new DataView(new ArrayBuffer(71))) === null, 'wrong length rejected');
    const st = createTelemetryStore();
    const add = (apid, user, seq, time) => st.ingest(decodeGroundFrame(K.packFrame({ apid, user, seq, time, groundTime: time + 0.1 })), 10);
    ok(buildModel(st, 10, null).engines[0].state === '--' && buildModel(st, 10, null).text.rvacStatus === '--', 'no packet');
    add(0x150, K.packEngineStatus(ENG), 0, 5);
    add(0x120, K.packGnc({ ...GNC, engines: [{ on: true, thr: 0.50, pitch: 0, yaw: 0 }, { on: true, thr: 0.54, pitch: 0, yaw: 0 }, { on: true, thr: 0.52, pitch: 0, yaw: 0 }] }), 0, 5);
    add(0x130, K.packFeedback(FB), 0, 5);
    ok(st.sampleCount(0x150) === 1, 'stored');
    const [x, y, c] = st.series(['engine_status.engines[0].throttle', 'engine_status.engines[2].command'], -Infinity);
    ok(x.length === 1, 'column window'); approx(y[0], 48, 'stored throttle percent', 1e-4); approx(c[0], 100, 'stored command percent');
    const m = buildModel(st, 10, null);
    ok(m.engines.length === 6 && m.engines[0].state === 'RUNNING' && m.engines[0].tone === 'green', 'running state');
    ok(m.engines[2].state === 'STARTING' && m.engines[2].tone === 'amber' && m.engines[2].invalid && m.engines[2].valid === 'INVALID', 'starting, sensor invalid');
    ok(m.engines[3].command === 'OFF' && m.engines[5].throttle === '0' && m.engines[3].tone === 'off', 'RVac off');
    ok(m.text.rvacStatus === 'OFF (not used in hop)', 'rvac status ' + m.text.rvacStatus);
    ok(m.text.thrAct[0] === '48' && m.text.thrAct[2] === '90', 'actual throttle from engine_status');
    ok(m.stale.ENG === 'ok', 'engine packet staleness tracked');
    // Differential throttle: 50, 54, 52 -> mean 52, spread 4 pp > 2 pp.
    approx(m.text.thrSpread === '4.0' ? 4 : -1, 4, 'spread text');
    ok(m.text.thrDev[0] === '-2.0' && m.text.thrDev[1] === '+2.0' && m.lamps.diffThrottle, 'deviation text and lamp');
    const sd = st.series(['derived.thrDev[1]', 'derived.thrSpread'], -Infinity);
    approx(sd[1][0], 2, 'stored deviation', 1e-4); approx(sd[2][0], 4, 'stored spread', 1e-4);
    const dt = differentialThrottle([{ on: true, throttle: 0.5 }, { on: true, throttle: 0.5 + DIFF_THROTTLE_THRESHOLD_PP / 100 }, { on: false, throttle: 0 }]);
    ok(!dt.active && dt.dev[2] === 0, 'exactly at the threshold is not active; off engine deviation 0');
    ok(differentialThrottle([{ on: true, throttle: 0.5 }, { on: true, throttle: 0.53 }, { on: true, throttle: 0.5 }]).active, 'above threshold');
    ok(!differentialThrottle([{ on: false, throttle: 0 }, { on: false, throttle: 0 }, { on: false, throttle: 0 }]).active, 'all off');
    ok(Number.isNaN(differentialThrottle(null).spread) && !differentialThrottle(null).active, 'no data');
    // Margins use the power record when present.
    const mg = engineMargins({ on: true, throttle: 0.5, pitch: 0, yaw: 0 }, null, P.GIMBAL_LIMIT_DEG, {}, { on: true, command: 1, throttle: 0.9 });
    ok(mg.throttleMax && Math.abs(mg.throttleActualPct - 90) < 1e-4, 'margin throttle flag from power command');
    ok(!engineMargins({ on: true, throttle: 1, pitch: 0, yaw: 0 }, null, P.GIMBAL_LIMIT_DEG, {}, { on: true, command: 0.5, throttle: 0.5 }).throttleMax, 'power command wins over guidance');
    const rv = (state, on) => ({ engines: [0, 1, 2, 3, 4, 5].map((i) => (i === 4 ? ES(state, on, true, 0, 0) : ES(0, false, true, 0, 0))) });
    ok(rvacSummary(rv(3, true)) === 'INHIBITED' && rvacSummary(rv(2, true)) === 'RUNNING' && rvacSummary(rv(1, true)) === 'STARTING' && rvacSummary(rv(0, true)) === 'COMMANDED ON', 'rvac summaries');
    // Screens: actuators carries the power panel and the six-engine chart; RVac chart series expand to six.
    const act = R.getScreen('landing', 'actuators');
    const types = [];
    const walk = (w) => { types.push(w.type); (w.children || []).forEach(walk); };
    act.rows.forEach((r) => r.forEach(walk));
    ok(types.includes('powerTable') && types.includes('chart'), 'actuators has the power table');
    const six = expandSeries([{ param: 'engine_status.engines[{e}].throttle', label: '', color: '#111', perEngine: true, powerEngines: true }], 0);
    ok(six.length === 6 && six[5].param === 'engine_status.engines[5].throttle' && six[3].label === 'RV1' && six[0].label === 'SL1', 'six-engine series');
    ok(R.screenFromKey('landing', '1') && R.screenFromKey('ascent', '6') && R.screenFromKey('landing', '5'), 'screen keys unchanged');
  });

  const CSTAT = { phase: 2, abort: 0, arms: 1, flags: 0b0110011, along: 12.5, lateral: -0.25, pinHeight: 3.5, alongSpeed: 1.5, lateralSpeed: 0.125,
    sink: 0.75, tilt: 0.05, limit: 1.6, ttSeat: 8 };
  const CCFG = { landingMode: 1, seat: [42.4, 0, 45.25], dir: [1, 0], staging: 20, tower: [60, 0], mastHalf: 6, mastHeight: 145, hingeHalf: 8,
    hingeSetback: 6, armLen: 16, armOpen: 20 * D2R, armClosed: 14.5 * D2R, winLat: 0.7, winAlong: 1.5, winSink: 1.5, winTilt: 3 * D2R };

  test('catch packets: decode, store, model, regime', () => {
    ok(P.APID.CATCH_STATUS === 352 && P.APID.CATCH_CONFIG === 353 && P.APID_NAMES[352] === 'CATCH' && P.APID_NAMES[353] === 'CATCHCFG'
      && P.USER_LENGTH[0x160] === 40 && P.USER_LENGTH[0x161] === 80, 'apids, names, lengths');
    const s = dec(0x160, K.packCatchStatus(CSTAT));
    ok(s.phase === 2 && s.abortReason === 0 && s.armsExpected === 1 && s.go && s.positionOk && !s.speedOk && !s.tiltOk && s.rtkOk && s.corridorOk && !s.aborted, 'status flags');
    approxArr([s.alongRemaining, s.lateralError, s.pinHeightAboveSeat, s.alongSpeed, s.lateralSpeed, s.sinkRate, s.tilt, s.lateralLimit, s.timeToSeat],
      [12.5, -0.25, 3.5, 1.5, 0.125, 0.75, f32(0.05), f32(1.6), 8], 'status floats', 1e-4);
    const c = dec(0x161, K.packCatchConfig(CCFG));
    ok(c.landingMode === 1, 'config landing mode');
    approxArr([...c.seatPoint, ...c.approachDir, c.stagingDistance, ...c.towerPosition, c.mastHalfWidth, c.mastHeight, c.hingeHalfSpacing, c.hingeSetback, c.armLength],
      [42.4, 0, 45.25, 1, 0, 20, 60, 0, 6, 145, 8, 6, 16], 'config geometry', 1e-4);
    approxArr([c.armOpenAngle, c.armClosedAngle, c.windowLateral, c.windowAlong, c.windowSink, c.windowTilt], [20 * D2R, 14.5 * D2R, 0.7, 1.5, 1.5, 3 * D2R], 'config window', 1e-4);
    ok(P.decodePayload(0x160, new DataView(new ArrayBuffer(39))) === null, 'wrong length rejected');

    const st = createTelemetryStore();
    const add = (apid, user, seq, time) => st.ingest(decodeGroundFrame(K.packFrame({ apid, user, seq, time, groundTime: time + 0.1 })), 10);
    const none = buildModel(st, 10, null);
    ok(none.text.catchSide === '--' && !none.lamps.catchNoGo && none.stale.CATCH === 'none', 'absent: dashes, no lamp, no data');
    ok(formatParam({ param: 'catch_status.alongRemaining' }, st, none) === '--' && formatParam({ param: 'catch_status.phase' }, st, none) === '--', 'absent params');
    add(0x100, K.packFsw({ ...FSW, mode: 7 }), 0, 5);
    add(0x160, K.packCatchStatus(CSTAT), 0, 5);
    add(0x161, K.packCatchConfig(CCFG), 0, 5);
    add(0x160, K.packCatchStatus({ ...CSTAT, along: 10, lateral: 0.5, flags: 0 }), 1, 5.1);
    const w = st.series(['catch_status.alongRemaining', 'catch_status.tilt', 'catch_status.lateralLimit', 'derived.negLateralLimit'], -Infinity);
    ok(w[0].length === 2 && w[1][0] === 12.5 && w[1][1] === 10, 'stored along');
    approx(w[2][0], f32(0.05) * 180 / Math.PI, 'stored tilt in deg', 1e-4);
    approx(w[3][0], f32(1.6), 'stored limit', 1e-4); approx(w[4][0], -f32(1.6), 'stored mirrored limit', 1e-4);
    const m = buildModel(st, 10, null);
    ok(m.modeValue === 7 && m.text.mode === 'CATCH APPROACH', 'mode label');
    ok(m.text.catchSide === '+b (SOUTH)' && m.lamps.catchNoGo && m.stale.CATCH === 'ok' && m.stale.CATCHCFG === 'ok', 'side text, no-go lamp, staleness: ' + m.text.catchSide);
    ok(buildModel(st, 11.5, null).stale.CATCHCFG === 'ok' && buildModel(st, 11.5, null).stale.CATCH === 'stale', 'catch config allowed 2.5 s');
    ok(formatParam({ param: 'catch_status.phase' }, st, m) === 'APPROACH' && formatParam({ param: 'catch_status.armsExpected' }, st, m) === 'CLOSING', 'enum labels');
    ok(formatParam({ param: 'catch_status.alongRemaining', signed: true }, st, m) === '+10.00' && formatParam({ param: 'catch_status.flags.go' }, st, m) === 'NO', 'values');
    ok(formatParam({ param: 'catch_config.windowTilt' }, st, m) === '3.00' && paramUnit({ param: 'catch_config.windowTilt' }) === 'deg', 'window tilt in deg');
    ok(lateralSideText(-1, [1, 0]) === '-b (NORTH)' && lateralSideText(0.01, [1, 0]) === 'CENTERED' && lateralSideText(1, null) === '+b' && lateralSideText(NaN, null) === '--', 'side text');
    // Regime: modes, screens and key handling.
    ok(R.getRegime('catch').status === 'live' && R.screensOf('catch').map((x) => x.id).join() === 'overview,actuators,nav,navhealth,fsw', 'catch screens');
    ok(R.screenFromKey('catch', '1') === 'overview' && R.screenFromKey('catch', '5') === 'fsw' && R.screenFromKey('catch', '6') === null, 'catch keys');
    ok(R.parseHash('#/catch/overview').regime === 'catch', 'catch hash');
    const f = R.followStep({ follow: true, regime: 'landing', screen: 'nav' }, 7);
    ok(f.state.regime === 'catch' && f.state.screen === 'nav' && f.notice === 'Phase: LANDING -> CATCH', 'follow into catch: ' + f.notice);
    ok(R.followStep(f.state, 4).state.regime === 'landing', 'abort returns to the landing regime');
    ok(R.followStep(f.state, 8).state === f.state && R.followStep(f.state, 10).state === f.state, 'catch modes stay in the catch regime');
    ok(R.screenParams(R.getScreen('catch', 'overview')).some((p) => p === 'catch_config.windowLateral'), 'overview binds the window');
    ok(R.screenChartParams(R.getScreen('catch', 'overview')).every((p) => PARAM_APID[p] === 0x160), 'catch charts are stored columns');
  });

  test('corridor geometry (pure)', () => {
    ok(CV.niceCeil(7, [5, 10, 20]) === 10 && CV.niceCeil(99, [5, 10]) === 10 && CV.niceCeil(5, [5, 10]) === 5, 'niceCeil');
    const fr = CV.approachFrame([1, 0]);
    ok(fr.aE === -1 && fr.bE === 0 && fr.bN === -1, 'frame: a west, b south');
    ok(CV.approachFrame(null).dE === 1 && CV.approachFrame([0, 0]).dE === 1, 'frame fallback');
    // Funnel: limit = 0.6 + 0.08 * along, sampled at 3 points.
    const fit = CV.funnelFit([20, 5, 12], [2.2, 1.0, 1.56], 3);
    approx(fit.base, 0.6, 'funnel base', 1e-9); approx(fit.slope, 0.08, 'funnel slope', 1e-9);
    approx(CV.funnelLimit(fit, -3), 0.6, 'funnel does not widen behind the seat'); approx(CV.funnelLimit(fit, 10), 1.4, 'funnel at 10 m');
    ok(CV.funnelFit([5, 5.1], [1, 1], 2) === null && CV.funnelFit([], [], 0) === null && CV.funnelFit([NaN, 4], [1, 2], 2) === null, 'no fit without span');
    const cfg = { approachDir: [1, 0], towerPosition: [60, 0], mastHalfWidth: 6, hingeHalfSpacing: 8, hingeSetback: 6, armLength: 16,
      armOpenAngle: 20 * D2R, armClosedAngle: 14.5 * D2R, seatPoint: [42.4, 0, 45.25], stagingDistance: 20, windowLateral: 0.7, windowAlong: 1.5 };
    const mast = CV.mastCorners(cfg);
    ok(mast.length === 4 && mast.every(([e, n]) => Math.abs(Math.abs(e - 60) - 6) < 1e-9 && Math.abs(Math.abs(n) - 6) < 1e-9), 'mast square');
    const a0 = CV.armSegment(cfg, 1, false), a1 = CV.armSegment(cfg, -1, false);
    approxArr(a0.hinge, [54, -8], 'arm 0 hinge on +b (south)', 1e-9); approxArr(a1.hinge, [54, 8], 'arm 1 hinge', 1e-9);
    approxArr(a0.tip, [54 - 16 * Math.cos(20 * D2R), -8 - 16 * Math.sin(20 * D2R)], 'open arm tip points outward', 1e-9);
    const c0 = CV.armSegment(cfg, 1, true);
    approxArr(c0.tip, [54 - 16 * Math.cos(14.5 * D2R), -8 + 16 * Math.sin(14.5 * D2R)], 'closed arm tip points inward', 1e-9);
    ok(CV.hasApproachLine(cfg) && !CV.hasApproachLine({ ...cfg, stagingDistance: 0 }) && !CV.hasApproachLine(null), 'approach line only with a staging distance');
    const ln0 = CV.approachLine({ ...cfg, stagingDistance: 0 });
    approxArr([...ln0.staging, ...ln0.seat], [42.4, 0, 42.4, 0], 'vertical catch: staging point is the seat', 1e-9);
    ok(CV.corridorExtents([95, 60, 10], [0.4, -0.2, 0.1], [0.1, -0.05, 0.02], 3, { ...cfg, stagingDistance: 0 }, null).alongMax === 200, 'height axis extent (95 m + margin)');
    const ln = CV.approachLine(cfg);
    approxArr([...ln.staging, ...ln.seat], [22.4, 0, 42.4, 0], 'approach line', 1e-9);
    const ex = CV.corridorExtents([12, 8], [0.4, -0.9], [3, 1], 2, cfg, fit);
    ok(ex.alongMax === 15 && ex.lateralMax === 3 && ex.heightMax === 4 && ex.alongMin === -3 && ex.heightMin === -0.5, 'extents ' + JSON.stringify(ex));
    ok(CV.corridorExtents([], [], [], 0, null, null).alongMax === 5, 'empty extents');
    ok(CV.windowBox(cfg).along === 1.5 && CV.windowBox(null) === null, 'window box');
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
