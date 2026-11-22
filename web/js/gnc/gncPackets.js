// GNC packet user-data decoders per docs/telemetry/gnc_packets.md (big-endian). Pure module.

export const APID = {
  FSW: 0x100, CONFIG: 0x101, NAV: 0x110, GNC: 0x120, FEEDBACK: 0x130, GNSS: 0x140,
};
export const APID_NAMES = {
  0x100: 'FSW', 0x101: 'CONFIG', 0x110: 'NAV', 0x120: 'GNC', 0x130: 'ACT', 0x140: 'GNSS',
};
export const USER_LENGTH = { 0x100: 20, 0x101: 36, 0x110: 80, 0x120: 84, 0x130: 36, 0x140: 56 };
export const MODE_NAMES = ['PRELAUNCH', 'IGNITION', 'ASCENT', 'HOVER', 'DESCENT', 'FINAL DESCENT', 'LANDED'];
export const FIX_TYPE_NAMES = ['NO FIX', 'SPS', 'RTK FIXED'];

// Vehicle constants (dictionary, "Vehicle constants the console needs").
export const NUM_ENGINES = 3;
export const ENGINE_RADIUS = 1.55; // [m]
export const GIMBAL_LIMIT_DEG = 15; // [deg]
export const RAD2DEG = 180 / Math.PI;
// Body x, y of engine i: azimuth 90 + 120 i deg from +x toward +y.
export function enginePosition(i) {
  const az = (90 + 120 * i) * Math.PI / 180;
  return [ENGINE_RADIUS * Math.cos(az), ENGINE_RADIUS * Math.sin(az)];
}

const f32 = (dv, o) => dv.getFloat32(o, false);
const f64 = (dv, o) => dv.getFloat64(o, false);
const f32s = (dv, o, n) => Array.from({ length: n }, (_, i) => f32(dv, o + 4 * i));

function decodeFsw(dv) {
  const flags = dv.getUint8(1);
  return {
    mode: dv.getUint8(0), flags,
    enginesOn: (flags & 1) !== 0, navValid: (flags & 2) !== 0, translating: (flags & 4) !== 0,
    aligned: (flags & 8) !== 0, gnssReady: (flags & 16) !== 0,
    gnssFixCount: dv.getUint16(2, false), cycleCount: dv.getUint32(4, false),
    timeInMode: f32(dv, 8), propellant: f32(dv, 12), mass: f32(dv, 16),
  };
}

function decodeConfig(dv) {
  return {
    targetLatitude: f64(dv, 0), targetLongitude: f64(dv, 8), targetHeight: f64(dv, 16),
    targetAltitude: f32(dv, 24), alignTolerance: f32(dv, 28), translateAltitude: f32(dv, 32),
  };
}

function decodeNav(dv) {
  return {
    attitude: f32s(dv, 0, 4), angularRate: f32s(dv, 16, 3), specificForce: f32s(dv, 28, 3),
    altitude: f32(dv, 40), verticalVelocity: f32(dv, 44), altimeterRange: f32(dv, 48),
    altimeterValid: dv.getUint8(52) !== 0,
    east: f32(dv, 56), north: f32(dv, 60), velocityEast: f32(dv, 64), velocityNorth: f32(dv, 68),
    accelBiasEast: f32(dv, 72), accelBiasNorth: f32(dv, 76),
  };
}

function decodeGnc(dv) {
  const engines = [];
  for (let i = 0; i < NUM_ENGINES; i++) {
    const o = 36 + 16 * i;
    engines.push({ on: dv.getUint8(o) !== 0, throttle: f32(dv, o + 4), pitch: f32(dv, o + 8), yaw: f32(dv, o + 12) });
  }
  return {
    altitudeSetpoint: f32(dv, 0), velocitySetpoint: f32(dv, 4),
    eastSetpoint: f32(dv, 8), northSetpoint: f32(dv, 12),
    eastAccelCommand: f32(dv, 16), northAccelCommand: f32(dv, 20),
    attitudeError: f32s(dv, 24, 3), engines,
  };
}

function decodeFeedback(dv) {
  const engines = [];
  for (let i = 0; i < NUM_ENGINES; i++) {
    const o = 12 * i;
    engines.push({ valid: dv.getUint8(o) !== 0, pitch: f32(dv, o + 4), yaw: f32(dv, o + 8) });
  }
  return { engines };
}

function decodeGnss(dv) {
  return {
    fixType: dv.getUint8(0), satellitesUsed: dv.getUint8(1), satellitesVisible: dv.getUint8(2),
    fixTime: f64(dv, 4), latitude: f64(dv, 12), longitude: f64(dv, 20), height: f64(dv, 28),
    velocity: f32s(dv, 36, 3), horizontalSigma: f32(dv, 48), verticalSigma: f32(dv, 52),
  };
}

const DECODERS = {
  0x100: decodeFsw, 0x101: decodeConfig, 0x110: decodeNav,
  0x120: decodeGnc, 0x130: decodeFeedback, 0x140: decodeGnss,
};

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
