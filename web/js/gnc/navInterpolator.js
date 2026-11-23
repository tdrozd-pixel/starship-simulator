// Short wall-clock buffer for smooth attitude rendering between NAV telemetry packets.
export function createNavInterpolator(delaySeconds = 0.05, capacity = 8) {
  const samples = [];

  function push(nav, receivedAt) {
    const last = samples[samples.length - 1];
    if (last && receivedAt <= last.receivedAt) return;
    samples.push({ nav, receivedAt });
    if (samples.length > capacity) samples.shift();
  }

  function sample(now) {
    if (samples.length === 0) return null;
    const target = now - delaySeconds;
    if (samples.length === 1 || target <= samples[0].receivedAt) return samples[0].nav;
    const last = samples[samples.length - 1];
    if (target >= last.receivedAt) return last.nav;

    let i = 1;
    while (samples[i].receivedAt < target) i++;
    const a = samples[i - 1], b = samples[i];
    const fraction = (target - a.receivedAt) / (b.receivedAt - a.receivedAt);
    return interpolateNav(a.nav, b.nav, fraction);
  }

  function reset() { samples.length = 0; }
  return { push, sample, reset };
}

function interpolateNav(a, b, t) {
  const mix = (x, y) => x + (y - x) * t;
  return {
    ...b,
    attitude: slerp(a.attitude, b.attitude, t),
    angularRate: a.angularRate.map((v, i) => mix(v, b.angularRate[i])),
    specificForce: a.specificForce.map((v, i) => mix(v, b.specificForce[i])),
    altitude: mix(a.altitude, b.altitude),
    verticalVelocity: mix(a.verticalVelocity, b.verticalVelocity),
    altimeterRange: mix(a.altimeterRange, b.altimeterRange),
    east: mix(a.east, b.east),
    north: mix(a.north, b.north),
    velocityEast: mix(a.velocityEast, b.velocityEast),
    velocityNorth: mix(a.velocityNorth, b.velocityNorth),
    accelBiasEast: mix(a.accelBiasEast, b.accelBiasEast),
    accelBiasNorth: mix(a.accelBiasNorth, b.accelBiasNorth),
  };
}

function slerp(a, b, t) {
  let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let end = b;
  if (dot < 0) {
    end = b.map((v) => -v);
    dot = -dot;
  }
  if (dot > 0.9995) {
    const q = a.map((v, i) => v + (end[i] - v) * t);
    const norm = Math.hypot(...q);
    return q.map((v) => v / norm);
  }
  const angle = Math.acos(Math.max(-1, Math.min(1, dot)));
  const sinAngle = Math.sin(angle);
  const wa = Math.sin((1 - t) * angle) / sinAngle;
  const wb = Math.sin(t * angle) / sinAngle;
  return a.map((v, i) => wa * v + wb * end[i]);
}
