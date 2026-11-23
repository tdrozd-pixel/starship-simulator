// Display model for the GNC console: telemetry store in, strings and flags out. Pure module
// (no DOM, no three) so the numbers on screen are unit tested. Time shown is onboard time from
// telemetry; wall-clock values (staleness, link rate) are passed in and used only for link health.
import { formatMet, signed } from '../hudModel.js';
import { APID, APID_NAMES, MODE_NAMES, FIX_TYPE_NAMES, rotationVectorDeg, tiltDeg, RAD2DEG } from './gncPackets.js';
import { makeEnuFrame, geodeticToEnu, horizontalDistance, bearingDeg } from './geodesy.js';

export const STALE_AFTER = 1.0; // [s] wall clock without a packet of an APID
export const STALE_AFTER_SLOW = 2.5; // [s] for the 1 Hz mission config packet
export const MODE_COLORS = ['#667788', '#f08030', '#35a7ff', '#4ccf6a', '#b080ff', '#f0d030', '#2fc9b0'];
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

const tmpEnu = [0, 0, 0];
const tmpRot = [0, 0, 0];

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
    else stale[name] = wallNow - w > (Number(apid) === APID.CONFIG ? STALE_AFTER_SLOW : STALE_AFTER) ? 'stale' : 'ok';
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
    text: t, lamps, stale, raw, navDelta, frame, have,
    att,
    modeValue: fsw ? fsw.mode : -1, fixValue: gnss ? gnss.fixType : -1,
    modeColor: fsw ? MODE_COLORS[fsw.mode] || '#667788' : '#445',
  };
}
