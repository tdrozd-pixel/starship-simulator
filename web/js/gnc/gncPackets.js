// GNC packet user-data decoders (big-endian), built on the generated gncPackets.generated.js.
// Layouts: telemetry/dictionary.json, docs/telemetry/gnc_packets.md. Pure module.

import { PACKETS, ENUMS, CONSTANTS, DECODERS, USER_LENGTH } from './gncPackets.generated.js';

export { PACKETS, ENUMS, USER_LENGTH };

// APIDs and short console names; the numbers come from the generated dictionary.
export const APID = {
  FSW: PACKETS.fsw_status.apid, CONFIG: PACKETS.mission_config.apid, NAV: PACKETS.navigation.apid,
  GNC: PACKETS.guidance_control.apid, FEEDBACK: PACKETS.actuator_feedback.apid, GNSS: PACKETS.gnss.apid,
  ENGINE: PACKETS.engine_status.apid,
  CATCH_STATUS: PACKETS.catch_status.apid, CATCH_CONFIG: PACKETS.catch_config.apid,
};
export const APID_NAMES = {
  [APID.FSW]: 'FSW', [APID.CONFIG]: 'CONFIG', [APID.NAV]: 'NAV', [APID.GNC]: 'GNC',
  [APID.FEEDBACK]: 'ACT', [APID.GNSS]: 'GNSS', [APID.ENGINE]: 'ENG',
  [APID.CATCH_STATUS]: 'CATCH', [APID.CATCH_CONFIG]: 'CATCHCFG',
};
// Indexed by enum value (values may have gaps, e.g. a retired mode).
const labelsByValue = (list) => { const a = []; for (const e of list) a[e.value] = e.label; return a; };
export const MODE_NAMES = labelsByValue(ENUMS.mode);
export const FIX_TYPE_NAMES = labelsByValue(ENUMS.gnssFixType);
export const ENGINE_STATE_NAMES = labelsByValue(ENUMS.engineState);
export const CATCH_PHASE_NAMES = labelsByValue(ENUMS.catchPhase);
export const CATCH_ABORT_NAMES = labelsByValue(ENUMS.catchAbort);
export const ARMS_EXPECTED_NAMES = labelsByValue(ENUMS.armsExpected);

// Vehicle constants (dictionary constants).
export const NUM_ENGINES = CONSTANTS.engineCount;
// Sea-level (gimbaled) engines come first, then the fixed Raptor Vacuum engines: engine_status
// carries all six, the gimbal records only the first NUM_ENGINES.
export const NUM_RVAC = CONSTANTS.rvacCount;
export const NUM_POWER_ENGINES = NUM_ENGINES + NUM_RVAC;
export const POWER_ENGINE_LABELS = [];
for (let i = 0; i < NUM_ENGINES; i++) POWER_ENGINE_LABELS.push('SL' + (i + 1));
for (let i = 0; i < NUM_RVAC; i++) POWER_ENGINE_LABELS.push('RV' + (i + 1));
export const ENGINE_RADIUS = CONSTANTS.engineRadius; // [m]
export const GIMBAL_LIMIT_DEG = CONSTANTS.gimbalLimitDeg; // [deg]
export const RAD2DEG = 180 / Math.PI;
// Body x, y of engine i: azimuth 90 + 120 i deg from +x toward +y.
export function enginePosition(i) {
  const az = (CONSTANTS.engineAzimuthFirstDeg + CONSTANTS.engineAzimuthStepDeg * i) * Math.PI / 180;
  return [ENGINE_RADIUS * Math.cos(az), ENGINE_RADIUS * Math.sin(az)];
}

// Decode user data (a DataView over it). Returns null for an unknown APID or a wrong length.
export function decodePayload(apid, dv) {
  const d = DECODERS[apid];
  if (!d || dv.byteLength !== USER_LENGTH[apid]) return null;
  return d(dv);
}

// Rotation vector (axis * angle) in degrees of a w,x,y,z quaternion, like hud formatBodyAttitude.
export function rotationVectorDeg(q, out) {
  let [w, x, y, z] = q;
  const n = Math.hypot(w, x, y, z);
  out[0] = out[1] = out[2] = 0;
  if (!Number.isFinite(n) || n <= 1e-12) return out;
  w /= n; x /= n; y /= n; z /= n;
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  const vn = Math.hypot(x, y, z);
  if (vn <= 1e-12) return out;
  const s = 2 * Math.atan2(vn, Math.min(1, w)) * RAD2DEG / vn;
  out[0] = x * s; out[1] = y * s; out[2] = z * s;
  return out;
}

// Tilt of body +z from world up [deg], from a w,x,y,z body-to-world quaternion.
export function tiltDeg(q) {
  const [w, x, y, z] = q;
  const n2 = w * w + x * x + y * y + z * z || 1;
  const zz = 1 - 2 * (x * x + y * y) / n2;
  return Math.acos(Math.max(-1, Math.min(1, zz))) * RAD2DEG;
}
