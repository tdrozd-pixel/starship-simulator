// Catch tower and arm kinematics in the sim ENU plane (docs/protocol/truth_stream.md, hello "tower").
// Pure module: no three, no DOM, no frame conversion (frames.js owns that).
//
// a = approachDir (unit, from the tower toward the approach side), b = a rotated +90 deg in the
// ENU plane (b = (-a_n, a_e); (0,-1) when a = (-1,0)). Hinge k sits at tower + setback * a
// +/- hingeHalfSpacing * b (arm 0 on +b, arm 1 on -b). The arm direction angle phi is measured
// from a, inward (toward the approach axis) positive, mirrored for arm 1:
//   phi = -armOpenAngle + psi,  dir_k = cos(phi) a - s_k sin(phi) b,  s_0 = +1, s_1 = -1.

export const TOWER_NUMBER_KEYS = Object.freeze([
  'mastHalfWidth', 'mastHeight', 'hingeHalfSpacing', 'hingeSetback', 'armLength', 'armHalfWidth',
  'armThickness', 'railHeight', 'armOpenAngle', 'armClosedAngle', 'pinHeight', 'pinOffset',
]);

// Returns a reason string if the hello tower object is unusable, else null.
export function validateTower(t) {
  if (!t || typeof t !== 'object') return 'hello has no tower';
  for (const k of ['position', 'approachDir']) {
    if (!Array.isArray(t[k]) || t[k].length !== 2 || !t[k].every(Number.isFinite)) return 'tower.' + k + ' invalid';
  }
  for (const k of TOWER_NUMBER_KEYS) if (!Number.isFinite(t[k])) return 'tower.' + k + ' invalid';
  if (!(Math.hypot(t.approachDir[0], t.approachDir[1]) > 0.5)) return 'tower.approachDir not a unit vector';
  return null;
}

function unitApproach(t) {
  const n = Math.hypot(t.approachDir[0], t.approachDir[1]);
  return [t.approachDir[0] / n, t.approachDir[1] / n];
}

export function hingePosition(t, k, out) {
  const [ae, an] = unitApproach(t);
  const be = -an, bn = ae;
  const s = k === 0 ? 1 : -1;
  out[0] = t.position[0] + t.hingeSetback * ae + s * t.hingeHalfSpacing * be;
  out[1] = t.position[1] + t.hingeSetback * an + s * t.hingeHalfSpacing * bn;
  return out;
}

// Direction angle from a (inward positive) for an actual arm angle psi [rad]; same for both arms.
export function armDirectionAngle(t, psi) { return -t.armOpenAngle + psi; }

export function armDirection(t, k, psi, out) {
  const [ae, an] = unitApproach(t);
  const be = -an, bn = ae;
  const s = k === 0 ? 1 : -1;
  const phi = armDirectionAngle(t, psi);
  const c = Math.cos(phi), sn = Math.sin(phi);
  out[0] = c * ae - s * sn * be;
  out[1] = c * an - s * sn * bn;
  return out;
}

export function armEnd(t, k, psi, out) {
  hingePosition(t, k, out);
  const d = [0, 0]; // setup-time and test use; per-frame code uses armHeading
  armDirection(t, k, psi, d);
  out[0] += t.armLength * d[0];
  out[1] += t.armLength * d[1];
  return out;
}

// ENU heading of the arm [rad], atan2(north, east), for orienting the arm mesh.
export function armHeading(t, k, psi) {
  const [ae, an] = unitApproach(t);
  const be = -an, bn = ae;
  const s = k === 0 ? 1 : -1;
  const phi = armDirectionAngle(t, psi);
  const c = Math.cos(phi), sn = Math.sin(phi);
  return Math.atan2(c * an - s * sn * bn, c * ae - s * sn * be);
}
