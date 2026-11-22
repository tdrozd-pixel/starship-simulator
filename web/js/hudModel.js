// HUD text model: the single source of the strings shown by the DOM HUD (hud.js)
// and drawn into recorded frames (hudCanvas.js). Pure module: no DOM, no three.

const RAD2DEG = 180 / Math.PI;

export function formatMet(t) {
  if (!Number.isFinite(t) || t < 0) return 'T+--:--.-';
  const tenths = Math.floor(t * 10 + 1e-6); // round to tenths first, then split
  const m = Math.floor(tenths / 600);
  const rem = tenths - m * 600;
  const s = Math.floor(rem / 10);
  return 'T+' + String(m).padStart(2, '0') + ':' + String(s).padStart(2, '0') + '.' + (rem % 10);
}

export function formatTimeScale(ts) {
  if (typeof ts !== 'number' || !Number.isFinite(ts) || ts <= 0) return 'x-';
  const r = +ts.toFixed(3);
  return 'x' + (Number.isInteger(r) ? r.toFixed(1) : String(r));
}

export function signed(v, dp) {
  const s = v.toFixed(dp);
  return v >= 0 ? '+' + s : s;
}

export function formatBodyAttitude(quat) {
  let [w, x, y, z] = quat;
  const norm = Math.hypot(w, x, y, z);
  if (!Number.isFinite(norm) || norm <= 1e-12) return ['+0.00', '+0.00', '+0.00'];
  w /= norm; x /= norm; y /= norm; z /= norm;
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  const vectorNorm = Math.hypot(x, y, z);
  if (vectorNorm <= 1e-12) return ['+0.00', '+0.00', '+0.00'];
  const angleDeg = 2 * Math.atan2(vectorNorm, Math.max(-1, Math.min(1, w))) * RAD2DEG;
  const scale = angleDeg / vectorNorm;
  return [signed(x * scale, 2), signed(y * scale, 2), signed(z * scale, 2)];
}

// Bottom banner: the physics vehicle from the server's hello, or nothing before it.
export function vehicleNote(vehicle) {
  if (!vehicle) return '';
  return 'Physics vehicle: ' + vehicle.name + ', length ' + vehicle.length + ' m, radius ' + vehicle.radius + ' m.';
}

// Telemetry rows in display order. `key` matches the DOM element id suffix (t-<key>).
// `color` is set only where the DOM HUD colors a value (gimbal pitch cyan, yaw magenta).
export const TELEMETRY_ROWS = Object.freeze([
  { key: 'alt', label: 'ALT (m)' },
  { key: 'vs', label: 'V SPEED (m/s)' },
  { key: 'thr', label: 'THROTTLE (%)' },
  { key: 'gp', label: 'GIMBAL PITCH (deg)', color: '#35d7ff' },
  { key: 'gy', label: 'GIMBAL YAW (deg)', color: '#ff5bd6' },
  { key: 'ax', label: 'ATT X (deg)' },
  { key: 'ay', label: 'ATT Y (deg)' },
  { key: 'az', label: 'ATT Z (deg)' },
  { key: 'prop', label: 'PROP (kg)' },
  { key: 'mass', label: 'MASS (kg)' },
]);

// Display strings for one interpolated sample (or null: no data yet).
export function buildTelemetry(sample) {
  if (!sample) {
    const values = {};
    for (const r of TELEMETRY_ROWS) values[r.key] = '--';
    return { waiting: true, met: formatMet(NaN), values, engine: false, touchdown: false, finished: false };
  }
  const attitude = formatBodyAttitude(sample.quat);
  return {
    waiting: false,
    met: formatMet(sample.simTime),
    values: {
      alt: sample.pos[2].toFixed(1),
      vs: signed(sample.vel[2], 1),
      thr: (sample.throttle * 100).toFixed(0),
      gp: signed(sample.gimbalPitch * RAD2DEG, 2),
      gy: signed(sample.gimbalYaw * RAD2DEG, 2),
      ax: attitude[0],
      ay: attitude[1],
      az: attitude[2],
      prop: sample.propellantMass.toFixed(0),
      mass: sample.totalMass.toFixed(0),
    },
    engine: !!sample.engineRunning,
    touchdown: !!sample.touchdown,
    finished: !!sample.halted,
  };
}
