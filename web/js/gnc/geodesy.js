// WGS-84 geodetic <-> ECEF <-> local ENU, independent of the C++ implementation. Pure module.
// Angles in radians, lengths in metres.

export const WGS84_A = 6378137.0; // [m]
export const WGS84_F = 1 / 298.257223563;
export const WGS84_E2 = WGS84_F * (2 - WGS84_F);

export function geodeticToEcef(lat, lon, h, out) {
  const sl = Math.sin(lat), cl = Math.cos(lat);
  const n = WGS84_A / Math.sqrt(1 - WGS84_E2 * sl * sl);
  out[0] = (n + h) * cl * Math.cos(lon);
  out[1] = (n + h) * cl * Math.sin(lon);
  out[2] = (n * (1 - WGS84_E2) + h) * sl;
  return out;
}

// Iterative inverse (fixed point on latitude); converges far below 1 mm in a few iterations.
export function ecefToGeodetic(x, y, z, out) {
  const p = Math.hypot(x, y);
  const lon = Math.atan2(y, x);
  let lat = Math.atan2(z, p * (1 - WGS84_E2));
  let h = 0;
  for (let i = 0; i < 8; i++) {
    const sl = Math.sin(lat);
    const n = WGS84_A / Math.sqrt(1 - WGS84_E2 * sl * sl);
    h = p / Math.cos(lat) - n;
    lat = Math.atan2(z, p * (1 - WGS84_E2 * n / (n + h)));
  }
  out[0] = lat; out[1] = lon; out[2] = h;
  return out;
}

// A local ENU frame about a geodetic reference.
export function makeEnuFrame(lat, lon, h) {
  return {
    lat, lon, h, ecef: geodeticToEcef(lat, lon, h, [0, 0, 0]),
    sl: Math.sin(lat), cl: Math.cos(lat), so: Math.sin(lon), co: Math.cos(lon),
  };
}

export function ecefToEnu(f, x, y, z, out) {
  const dx = x - f.ecef[0], dy = y - f.ecef[1], dz = z - f.ecef[2];
  out[0] = -f.so * dx + f.co * dy;
  out[1] = -f.sl * f.co * dx - f.sl * f.so * dy + f.cl * dz;
  out[2] = f.cl * f.co * dx + f.cl * f.so * dy + f.sl * dz;
  return out;
}

export function enuToEcef(f, e, n, u, out) {
  out[0] = f.ecef[0] - f.so * e - f.sl * f.co * n + f.cl * f.co * u;
  out[1] = f.ecef[1] + f.co * e - f.sl * f.so * n + f.cl * f.so * u;
  out[2] = f.ecef[2] + f.cl * n + f.sl * u;
  return out;
}

export function geodeticToEnu(f, lat, lon, h, out) {
  const t = geodeticToEcef(lat, lon, h, [0, 0, 0]);
  return ecefToEnu(f, t[0], t[1], t[2], out);
}

export function enuToGeodetic(f, e, n, u, out) {
  const t = enuToEcef(f, e, n, u, [0, 0, 0]);
  return ecefToGeodetic(t[0], t[1], t[2], out);
}

export function horizontalDistance(e, n) { return Math.hypot(e, n); }

// Compass bearing [deg, 0..360) from the origin toward (e, n): 0 north, 90 east.
export function bearingDeg(e, n) {
  const b = Math.atan2(e, n) * 180 / Math.PI;
  return (b + 360) % 360;
}
