// The ONLY place sim frames are converted to renderer (three.js) frames.
// Pure module: no imports, no DOM.
//
// Sim world: ENU, x east, y north, z up. Body: +z toward the nose.
// three.js: Y up. (X, Y, Z) = (x, z, -y), i.e. M = [[1,0,0],[0,0,1],[0,-1,0]], det +1
// (a rotation of +90 deg about X). The vehicle Object3D local axes are M * body, and the
// cylinder is built along local +Y. A quaternion conjugated by M keeps w and maps the
// vector part (qx,qy,qz) -> (qx,qz,-qy).

export function enuToThree(x, y, z, out) {
  out[0] = x; out[1] = z; out[2] = -y;
  return out;
}

export function threeToEnu(X, Y, Z, out) {
  out[0] = X; out[1] = -Z; out[2] = Y;
  return out;
}

// A heading in the ENU plane, atan2(north, east) [rad], becomes a rotation about three's Y axis
// that turns local +X onto that direction: Ry(a) maps +X to (cos a, 0, -sin a) and ENU (e, n)
// maps to three (e, 0, -n), so the angle is unchanged.
export function enuHeadingToThreeYaw(heading) { return heading; }

// Output order is three's Quaternion.set(x, y, z, w).
export function bodyQuatToThree(w, x, y, z, out) {
  out[0] = x; out[1] = z; out[2] = -y; out[3] = w;
  return out;
}

export function bodyDirToThreeLocal(dx, dy, dz, out) {
  out[0] = dx; out[1] = dz; out[2] = -dy;
  return out;
}

// Thrust direction in the body frame per sim_vehicle/src/HopperDynamics.cpp is
// (cos p sin y, -sin p, cos p cos y); the exhaust points the opposite way.
export function exhaustDirBody(pitch, yaw, out) {
  const cp = Math.cos(pitch);
  out[0] = -cp * Math.sin(yaw);
  out[1] = Math.sin(pitch);
  out[2] = -cp * Math.cos(yaw);
  return out;
}
