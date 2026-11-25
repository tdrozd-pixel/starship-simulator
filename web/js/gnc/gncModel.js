// Display model for the GNC console: telemetry store in, strings and flags out. Pure module
// (no DOM, no three) so the numbers on screen are unit tested. Time shown is onboard time from
// telemetry; wall-clock values (staleness, link rate) are passed in and used only for link health.
import { formatMet, signed } from '../hudModel.js';
import { APID, APID_NAMES, MODE_NAMES, FIX_TYPE_NAMES, ENGINE_STATE_NAMES, POWER_ENGINE_LABELS, NUM_ENGINES, NUM_POWER_ENGINES, GIMBAL_LIMIT_DEG, rotationVectorDeg, tiltDeg, RAD2DEG } from './gncPackets.js';
import { makeEnuFrame, geodeticToEnu, horizontalDistance, bearingDeg } from './geodesy.js';

export const STALE_AFTER = 1.0; // [s] wall clock without a packet of an APID
export const STALE_AFTER_SLOW = 2.5; // [s] for the 1 Hz packets (mission config, catch config)
export const MODE_COLORS = ['#667788', '#f08030', '#35a7ff', '#4ccf6a', '#b080ff', '#f0d030', '#2fc9b0',
  '#35d7c0', '#ffa63a', '#ff7a5a', '#8fe05a']; // catch: approach, hold, final, caught
const DASH = '--';

export function fmt(v, dp) { return Number.isFinite(v) ? v.toFixed(dp) : DASH; }
export function fmtSigned(v, dp) { return Number.isFinite(v) ? signed(v, dp) : DASH; }

// Latitude "26.0000000 N", longitude "97.1500000 W" from radians.
export function formatLatitude(rad) {
  if (!Number.isFinite(rad)) return DASH;
  const d = rad * RAD2DEG;
  return Math.abs(d).toFixed(7) + (d < 0 ? ' S' : ' N');
}
export function formatLongitude(rad) {
  if (!Number.isFinite(rad)) return DASH;
  const d = rad * RAD2DEG;
  return Math.abs(d).toFixed(7) + (d < 0 ? ' W' : ' E');
}

export function modeName(mode) { return MODE_NAMES[mode] !== undefined ? MODE_NAMES[mode] : 'MODE ' + mode; }

// ENU frame about the mission-config target, cached on the config object identity.
let cachedConfig = null, cachedFrame = null;
export function targetFrame(cfg) {
  if (!cfg) return null;
  if (cfg !== cachedConfig) {
    cachedConfig = cfg;
    cachedFrame = makeEnuFrame(cfg.targetLatitude, cfg.targetLongitude, cfg.targetHeight);
  }
  return cachedFrame;
}

// Tilt of the nose (+z) from vertical split into the east and north planes, and the bearing of
// body +x, from a w,x,y,z body-to-world quaternion in the ENU world. Angles [deg]; out is
// {tiltE, tiltN, roll} (roll: bearing of body +x projected on the horizontal, 0 = north, clockwise).
export function attitudeAngles(q, out) {
  const [w, x, y, z] = q;
  const n2 = w * w + x * x + y * y + z * z || 1;
  const noseE = 2 * (x * z + w * y) / n2, noseN = 2 * (y * z - w * x) / n2, noseU = 1 - 2 * (x * x + y * y) / n2;
  const bxE = 1 - 2 * (y * y + z * z) / n2, bxN = 2 * (x * y + w * z) / n2;
  out.tiltE = Math.atan2(noseE, noseU) * RAD2DEG;
  out.tiltN = Math.atan2(noseN, noseU) * RAD2DEG;
  out.roll = (Math.atan2(bxE, bxN) * RAD2DEG + 360) % 360;
  return out;
}

// ---- Attitude error (commanded vs actual) ----
// err: guidance_control.attitudeError [rad], body x/y/z. Returns the magnitude [deg] (NaN if absent).
export function attitudeErrorMagDeg(err) {
  if (!err) return NaN;
  return Math.hypot(err[0], err[1], err[2]) * RAD2DEG;
}

// ---- Navigation health ----
// PLACEHOLDER thresholds for the data-stale lamps. TODO(phase 10, item 48): replace with limits from
// the limit system (dictionary limits), per sensor.
export const NAV_STALE_AMBER_S = 1; // [s] age of the last accepted update: amber above
export const NAV_STALE_RED_S = 3; // [s] red above
export const NO_UPDATE_AGE = -1; // dictionary: age -1 means no accepted update yet

// Age of the last accepted measurement update [s] -> {text, amber, red}. -1 (or non-finite) is
// "NONE YET" and raises no lamp (the filter has not started, e.g. on the pad).
export function updateAgeState(age) {
  if (!Number.isFinite(age) || age < 0) return { text: Number.isFinite(age) ? 'NONE YET' : DASH, amber: false, red: false };
  return { text: age.toFixed(2), amber: age > NAV_STALE_AMBER_S, red: age > NAV_STALE_RED_S };
}

// ---- Actuator margins ----
export const GIMBAL_WARN_FRACTION = 0.9; // lamp at >= 90 % of the gimbal limit
export const GIMBAL_LIMIT_FRACTION = 0.99; // "at the limit" (the command is clamped to the limit)
export const THROTTLE_MAX_FRACTION = 0.995; // dictionary carries no min/max: flag a command at 100 %

// ---- Differential throttle ----
// The flight software differentiates the sea-level (SL) throttles when the gimbals saturate. The
// console flags it when the commanded SL throttles (guidance_control) of the engines that are on
// differ by more than this. PLACEHOLDER threshold. TODO(phase 10, item 48): limit system.
export const DIFF_THROTTLE_THRESHOLD_PP = 2; // [percentage points] max minus min commanded SL throttle

// engines: guidance_control.engines (on, throttle fraction). Fills out.dev[i] = deviation of
// engine i from the mean of the engines that are on [pp] (0 for an engine that is off), out.spread
// = max minus min over engines that are on [pp] and out.active. All NaN/false without data.
export function differentialThrottle(engines, out = { dev: [0, 0, 0], spread: NaN, active: false }) {
  let sum = 0, n = 0, lo = Infinity, hi = -Infinity;
  if (engines) {
    for (const e of engines) if (e.on) { const p = e.throttle * 100; sum += p; n++; if (p < lo) lo = p; if (p > hi) hi = p; }
  }
  const mean = n ? sum / n : 0;
  for (let i = 0; i < out.dev.length; i++) out.dev[i] = engines && engines[i].on ? engines[i].throttle * 100 - mean : (engines ? 0 : NaN);
  out.spread = !engines ? NaN : n ? hi - lo : 0;
  out.active = out.spread > DIFF_THROTTLE_THRESHOLD_PP; // false for NaN
  return out;
}

// ---- Engine power settings (engine_status packet) ----
// Tone of the state cell: green is the unmarked "on" look; amber for a transitional or refused state.
export function engineStateTone(state) {
  const name = ENGINE_STATE_NAMES[state];
  return name === 'STARTING' || name === 'INHIBITED' ? 'amber' : name === 'RUNNING' ? 'green' : 'off';
}

// Summary of the RVac engines (indices NUM_ENGINES..) for the one-line status.
export function rvacSummary(eng) {
  if (!eng) return DASH;
  let on = 0, inh = 0, run = 0, start = 0;
  for (let i = NUM_ENGINES; i < NUM_POWER_ENGINES; i++) {
    const e = eng.engines[i];
    if (e.on) on++;
    const n = ENGINE_STATE_NAMES[e.state];
    if (n === 'INHIBITED') inh++; else if (n === 'RUNNING') run++; else if (n === 'STARTING') start++;
  }
  if (inh) return 'INHIBITED';
  if (run) return 'RUNNING';
  if (start) return 'STARTING';
  return on ? 'COMMANDED ON' : 'OFF (not used in hop)';
}

// One engine. cmd: {on, throttle, pitch, yaw} (rad, fraction); meas: {valid, pitch, yaw} (rad).
// Deflection magnitude is the vector length of pitch and yaw as a percent of the gimbal limit
// (GIMBAL_LIMIT_DEG from the dictionary constants). Returns percent values (NaN when absent) and flags.
//
// power (optional): engine_status record {on, command, throttle} of the same engine. When present the
// throttle margin uses it: the 100 % flag follows the power setting command, and throttleActualPct
// is the measured throttle (not the command). Without it the guidance command is used.
export function engineMargins(cmd, meas, limitDeg = GIMBAL_LIMIT_DEG, out = {}, power = null) {
  const cmdDeg = cmd ? Math.hypot(cmd.pitch, cmd.yaw) * RAD2DEG : NaN;
  const measDeg = meas ? Math.hypot(meas.pitch, meas.yaw) * RAD2DEG : NaN;
  out.cmdPct = 100 * cmdDeg / limitDeg;
  out.measPct = 100 * measDeg / limitDeg;
  out.errDeg = cmd && meas ? Math.hypot(cmd.pitch - meas.pitch, cmd.yaw - meas.yaw) * RAD2DEG : NaN; // cmd minus measured
  out.throttlePct = cmd ? cmd.throttle * 100 : NaN;
  const atLeast = (f) => out.cmdPct >= 100 * f || out.measPct >= 100 * f; // false for NaN
  out.warn = atLeast(GIMBAL_WARN_FRACTION);
  out.atLimit = atLeast(GIMBAL_LIMIT_FRACTION);
  out.throttleActualPct = power ? power.throttle * 100 : NaN;
  out.throttleMax = power ? !!(power.on && power.command >= THROTTLE_MAX_FRACTION)
    : !!(cmd && cmd.on && cmd.throttle >= THROTTLE_MAX_FRACTION);
  out.invalid = !!(meas && !meas.valid); // gimbal position sensor invalid: engine out or sensor failure
  return out;
}

// ---- Catch ----
// Side of the approach axis for a lateral error (positive = +b side). b is the approach direction
// rotated -90 deg: for the ship moving east (1, 0) it is (0, -1), south. approachDir is the downlinked
// unit vector (east, north) or null; side names are compass points when the axis is east/west or
// north/south, else just "+b"/"-b". Returns e.g. "+b (SOUTH)", "CENTERED" within 0.05 m.
export const CATCH_CENTERED_M = 0.05; // [m] display threshold for "centered"
export function lateralSideText(lateral, approachDir) {
  if (!Number.isFinite(lateral)) return DASH;
  if (Math.abs(lateral) < CATCH_CENTERED_M) return 'CENTERED';
  const sign = lateral > 0 ? 1 : -1;
  let name = '';
  if (approachDir) {
    const bE = approachDir[1] * sign, bN = -approachDir[0] * sign; // direction of the error
    if (Math.abs(bE) > 0.99) name = bE > 0 ? 'EAST' : 'WEST';
    else if (Math.abs(bN) > 0.99) name = bN > 0 ? 'NORTH' : 'SOUTH';
  }
  return (sign > 0 ? '+b' : '-b') + (name ? ' (' + name + ')' : '');
}

// ---- Landing prediction (ESTIMATE) ----
// A simple ballistic-style extrapolation from the downlinked navigation estimate only; it is NOT
// the flight guidance. Assumptions: the current vertical velocity is held until altitude reaches
// zero (time to go = altitude / descent rate), and the current horizontal velocity is held for that
// time. The real descent brakes (setpoint profile), so this is pessimistic for horizontal drift
// and optimistic for time; ttgSp uses the commanded vertical velocity setpoint instead.
export const MIN_DESCENT_RATE = 0.1; // [m/s] below this the vehicle is not considered descending
export const MAX_TTG = 600; // [s] longer predictions are not shown
export function landingPrediction(nav, gnc, cfg) {
  if (!nav) return null;
  const alt = nav.altitude, vz = nav.verticalVelocity;
  const rate = (r) => (alt <= 0 ? 0 : r < -MIN_DESCENT_RATE ? alt / -r : NaN);
  let ttg = rate(vz);
  if (!(ttg <= MAX_TTG)) ttg = NaN;
  let ttgSp = gnc ? rate(gnc.velocitySetpoint) : NaN;
  if (!(ttgSp <= MAX_TTG)) ttgSp = NaN;
  const east = nav.east + nav.velocityEast * ttg, north = nav.north + nav.velocityNorth * ttg;
  const miss = Math.hypot(east, north); // versus pad center
  const missSp = gnc ? Math.hypot(east - gnc.eastSetpoint, north - gnc.northSetpoint) : NaN;
  const tol = cfg ? cfg.alignTolerance : NaN;
  const go = Number.isFinite(miss) && Number.isFinite(tol) ? miss <= tol : null;
  return { ttg, ttgSp, east, north, miss, missSp, tol, go };
}

const tmpEnu = [0, 0, 0];
const tmpRot = [0, 0, 0];
const tmpMargin = {};
const tmpDiff = { dev: [0, 0, 0], spread: NaN, active: false };

// Mode timeline: segments by onboard time, appended when the mode changes.
export function createModeTimeline() {
  const segs = [];
  return {
    segs,
    update(mode, t) {
      const last = segs[segs.length - 1];
      if (!last) segs.push({ mode, t0: t });
      else if (last.mode !== mode && t >= last.t0) segs.push({ mode, t0: t });
    },
    reset() { segs.length = 0; },
  };
}

function lampsNavHealth(lamps, gu, au) {
  lamps.gnssStaleAmber = !!(gu && gu.amber); lamps.gnssStaleRed = !!(gu && gu.red);
  lamps.altStaleAmber = !!(au && au.amber); lamps.altStaleRed = !!(au && au.red);
}

// link: {fps} computed by the page from wall time. wallNow: wall-clock seconds.
export function buildModel(store, wallNow, link, navOverride = null) {
  const fsw = store.latest[APID.FSW];
  const cfg = store.latest[APID.CONFIG];
  const nav = navOverride || store.latest[APID.NAV];
  const gnc = store.latest[APID.GNC];
  const gnss = store.latest[APID.GNSS];
  const t = {};
  const have = store.totalReceived > 0;

  t.met = have ? formatMet(store.met) : 'T+--:--.-';
  t.mode = fsw ? modeName(fsw.mode) : DASH;
  t.run = store.runId === null ? '#-' : '#' + store.runId;
  t.linkFps = fmt(link ? link.fps : NaN, 0);
  t.linkLoss = have ? fmt(store.lossPercent(), 1) : DASH;
  t.linkLat = Number.isFinite(store.latency) ? fmt(store.latency * 1000, 0) : DASH;

  const stale = {};
  for (const [apid, name] of Object.entries(APID_NAMES)) {
    const w = store.lastWall[apid];
    if (w === undefined) stale[name] = 'none';
    else stale[name] = wallNow - w > (Number(apid) === APID.CONFIG || Number(apid) === APID.CATCH_CONFIG ? STALE_AFTER_SLOW : STALE_AFTER) ? 'stale' : 'ok';
  }

  // Navigation readouts.
  let att = null;
  if (nav) {
    rotationVectorDeg(nav.attitude, tmpRot);
    t.tilt = fmt(tiltDeg(nav.attitude), 2);
    att = attitudeAngles(nav.attitude, { tiltE: 0, tiltN: 0, roll: 0 });
    t.roll = fmt(att.roll, 1);
    t.attX = signed(tmpRot[0], 2); t.attY = signed(tmpRot[1], 2); t.attZ = signed(tmpRot[2], 2);
    t.rateX = signed(nav.angularRate[0] * RAD2DEG, 2);
    t.rateY = signed(nav.angularRate[1] * RAD2DEG, 2);
    t.rateZ = signed(nav.angularRate[2] * RAD2DEG, 2);
  } else {
    t.tilt = t.roll = t.attX = t.attY = t.attZ = t.rateX = t.rateY = t.rateZ = DASH;
  }

  // Flight software.
  t.fswMode = t.mode;
  t.fswTime = fsw ? fmt(fsw.timeInMode, 1) : DASH;
  t.fswFixes = fsw ? String(fsw.gnssFixCount) : DASH;
  t.fswCycles = fsw ? String(fsw.cycleCount) : DASH;
  t.fswProp = fsw ? fmt(fsw.propellant, 0) : DASH;
  t.fswMass = fsw ? fmt(fsw.mass, 0) : DASH;
  t.fswAlt = nav ? fmt(nav.altitude, 2) : DASH;
  t.fswAltSp = gnc ? fmt(gnc.altitudeSetpoint, 2) : DASH;
  t.fswVz = nav ? fmtSigned(nav.verticalVelocity, 2) : DASH;
  t.fswVzSp = gnc ? fmtSigned(gnc.velocitySetpoint, 2) : DASH;
  t.fswRange = nav ? fmt(nav.altimeterRange, 2) : DASH;
  t.fswRangeValid = nav ? (nav.altimeterValid ? 'VALID' : 'INVALID') : DASH;
  t.cfgTol = cfg ? fmt(cfg.alignTolerance, 1) : DASH;
  t.cfgTarget = cfg ? formatLatitude(cfg.targetLatitude) + ' ' + formatLongitude(cfg.targetLongitude) : DASH;
  t.spEast = gnc ? fmtSigned(gnc.eastSetpoint, 2) : DASH;
  t.spNorth = gnc ? fmtSigned(gnc.northSetpoint, 2) : DASH;
  let thrSum = 0, thrOn = 0;
  if (gnc) for (const e of gnc.engines) if (e.on) { thrSum += e.throttle; thrOn++; }
  t.throttle = gnc ? fmt(thrOn ? 100 * thrSum / thrOn : 0, 0) : DASH; // mean over engines that are on
  t.cfgTargetAlt = cfg ? fmt(cfg.targetAltitude, 1) : DASH;
  t.cfgTranslateAlt = cfg ? fmt(cfg.translateAltitude, 1) : DASH;
  t.cfgTargetHeight = cfg ? fmt(cfg.targetHeight, 1) : DASH;
  const lamps = {
    enginesOn: !!(fsw && fsw.enginesOn), navValid: !!(fsw && fsw.navValid),
    translating: !!(fsw && fsw.translating), aligned: !!(fsw && fsw.aligned),
    gnssReady: !!(fsw && fsw.gnssReady),
  };

  // Attitude error (commanded vs actual), from the guidance packet.
  t.attErrMag = gnc ? fmt(attitudeErrorMagDeg(gnc.attitudeError), 2) : DASH;

  // Navigation health (filter covariance, innovations, update ages) from the navigation packet.
  const gu = nav ? updateAgeState(nav.gnssUpdateAge) : null, au = nav ? updateAgeState(nav.altimeterUpdateAge) : null;
  t.gnssUpdAge = gu ? gu.text : DASH;
  t.altUpdAge = au ? au.text : DASH;
  t.gnssInnovMag = nav ? fmt(Math.hypot(nav.gnssInnovationEast, nav.gnssInnovationNorth), 2) : DASH;
  lampsNavHealth(lamps, gu, au);

  // Actuator margins per engine (arrays indexed by engine).
  const fb = store.latest[APID.FEEDBACK];
  const pw = store.latest[APID.ENGINE];
  t.gimCmdPct = []; t.gimMeasPct = []; t.gimErr = []; t.thrCmd = []; t.thrAct = [];
  const act = { gimWarn: [], gimLimit: [], thrMax: [], engInvalid: [] };
  for (let i = 0; i < NUM_ENGINES; i++) {
    const m = engineMargins(gnc ? gnc.engines[i] : null, fb ? fb.engines[i] : null, GIMBAL_LIMIT_DEG, tmpMargin, pw ? pw.engines[i] : null);
    t.gimCmdPct.push(fmt(m.cmdPct, 0)); t.gimMeasPct.push(fmt(m.measPct, 0)); t.gimErr.push(fmtSigned(m.errDeg, 2));
    t.thrCmd.push(!gnc ? DASH : gnc.engines[i].on ? fmt(m.throttlePct, 0) : 'OFF');
    t.thrAct.push(fmt(m.throttleActualPct, 0));
    act.gimWarn.push(m.warn); act.gimLimit.push(m.atLimit); act.thrMax.push(m.throttleMax); act.engInvalid.push(m.invalid);
  }
  Object.assign(lamps, act);

  // Differential throttle of the SL engines (command deviation from the mean of the engines on).
  const dt = differentialThrottle(gnc ? gnc.engines : null, tmpDiff);
  t.thrDev = dt.dev.map((v) => fmtSigned(v, 1));
  t.thrSpread = fmt(dt.spread, 1);
  lamps.diffThrottle = dt.active;

  // Power settings of all six engines (engine_status): strings per engine for the Power settings panel.
  const engines = [];
  for (let i = 0; i < NUM_POWER_ENGINES; i++) {
    const e = pw ? pw.engines[i] : null;
    engines.push({
      label: POWER_ENGINE_LABELS[i],
      state: e ? (ENGINE_STATE_NAMES[e.state] || 'STATE ' + e.state) : DASH,
      tone: e ? engineStateTone(e.state) : 'off',
      command: !e ? DASH : e.on ? fmt(e.command * 100, 0) : 'OFF',
      throttle: e ? fmt(e.throttle * 100, 0) : DASH,
      valid: !e ? DASH : e.valid ? 'VALID' : 'INVALID',
      invalid: !!(e && !e.valid),
    });
  }
  t.rvacStatus = rvacSummary(pw);

  // Catch (catch_status, catch_config). Lamps follow catch_status.flags; NO-GO is only lit with data.
  const cs = store.latest[APID.CATCH_STATUS], cc = store.latest[APID.CATCH_CONFIG];
  t.catchSide = cs ? lateralSideText(cs.lateralError, cc ? cc.approachDir : null) : DASH;
  lamps.catchNoGo = !!(cs && !cs.go);

  // Landing prediction (ESTIMATE, see landingPrediction).
  const pred = landingPrediction(nav, gnc, cfg);
  t.predTtg = pred ? fmt(pred.ttg, 1) : DASH;
  t.predTtgSp = pred ? fmt(pred.ttgSp, 1) : DASH;
  t.predEast = pred ? fmtSigned(pred.east, 1) : DASH;
  t.predNorth = pred ? fmtSigned(pred.north, 1) : DASH;
  t.predMiss = pred ? fmt(pred.miss, 1) : DASH;
  t.predMissSp = pred ? fmt(pred.missSp, 1) : DASH;
  t.predGo = pred && pred.go !== null ? (pred.go ? 'GO' : 'NO-GO') : DASH;
  lamps.predGo = !!(pred && pred.go === true);
  lamps.predNoGo = !!(pred && pred.go === false);

  // GNSS and deltas from the pad center.
  const frame = targetFrame(cfg);
  let raw = null, navDelta = null;
  if (gnss) {
    t.gnssFix = FIX_TYPE_NAMES[gnss.fixType] || 'FIX ' + gnss.fixType;
    t.gnssSats = gnss.satellitesUsed + ' / ' + gnss.satellitesVisible;
    t.gnssLat = formatLatitude(gnss.latitude);
    t.gnssLon = formatLongitude(gnss.longitude);
    t.gnssHeight = fmt(gnss.height, 2);
    t.gnssHSigma = fmt(gnss.horizontalSigma, 2);
    t.gnssVSigma = fmt(gnss.verticalSigma, 2);
    t.gnssAge = Number.isFinite(store.latestTime[APID.GNSS]) ? fmt(store.met - gnss.fixTime, 2) : DASH;
    if (frame && gnss.fixType > 0) {
      geodeticToEnu(frame, gnss.latitude, gnss.longitude, gnss.height, tmpEnu);
      raw = {
        east: tmpEnu[0], north: tmpEnu[1],
        dist: horizontalDistance(tmpEnu[0], tmpEnu[1]), bearing: bearingDeg(tmpEnu[0], tmpEnu[1]),
      };
    }
  } else {
    t.gnssFix = t.gnssSats = t.gnssLat = t.gnssLon = t.gnssHeight = t.gnssHSigma = t.gnssVSigma = t.gnssAge = DASH;
  }
  t.rawEast = raw ? fmtSigned(raw.east, 2) : DASH;
  t.rawNorth = raw ? fmtSigned(raw.north, 2) : DASH;
  t.rawDist = raw ? fmt(raw.dist, 2) : DASH;
  t.rawBearing = raw ? fmt(raw.bearing, 1) : DASH;
  if (nav) {
    navDelta = { east: nav.east, north: nav.north, dist: horizontalDistance(nav.east, nav.north) };
  }
  t.navEast = navDelta ? fmtSigned(navDelta.east, 2) : DASH;
  t.navNorth = navDelta ? fmtSigned(navDelta.north, 2) : DASH;
  t.navDist = navDelta ? fmt(navDelta.dist, 2) : DASH;

  t.gnssChip = gnss ? t.gnssFix + '  ' + gnss.satellitesUsed + '/' + gnss.satellitesVisible + '  ' + (raw ? fmt(raw.dist, 1) : DASH) + ' m' : DASH;

  return {
    text: t, lamps, stale, engines, raw, navDelta, frame, have, pred,
    att,
    modeValue: fsw ? fsw.mode : -1, fixValue: gnss ? gnss.fixType : -1,
    modeColor: fsw ? MODE_COLORS[fsw.mode] || '#667788' : '#445',
  };
}
