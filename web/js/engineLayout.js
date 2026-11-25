// Ship engine layout for the outside view. Pure module: no three, no DOM.
//
// Source of truth is sim_vehicle/include/sim/vehicle/HopperParams.hpp (engineClusterRadius,
// engineAzimuth0, rvacClusterRadius, rvacAzimuth0). Azimuths are measured in the body xy plane
// from body +x toward +y; engines are spaced 120 deg apart. Keep these numbers in sync with it.
// Engine index: 0..2 sea-level (SL, gimbaled), 3..5 Raptor Vacuum (RVac, fixed).

const DEG = Math.PI / 180;

export const ENGINE_COUNT = 6;
export const SL_COUNT = 3;
export const SL_RING_RADIUS = 1.55; // [m] estimate (HopperParams engineClusterRadius)
export const SL_AZIMUTH0 = 90 * DEG; // [rad] estimate (HopperParams engineAzimuth0)
export const RVAC_RING_RADIUS = 3.6; // [m] estimate (HopperParams rvacClusterRadius)
export const RVAC_AZIMUTH0 = 150 * DEG; // [rad] estimate (HopperParams rvacAzimuth0)
export const ENGINE_AZIMUTH_STEP = 120 * DEG; // [rad]

export const ENGINE_LABELS = Object.freeze(['SL1', 'SL2', 'SL3', 'RV1', 'RV2', 'RV3']);

export function isSeaLevel(i) { return i < SL_COUNT; }

// Body-frame position of engine i on the base plane (z = 0). Output [x, y, z] in body axes.
export function enginePositionBody(i, out) {
  const sl = isSeaLevel(i);
  const r = sl ? SL_RING_RADIUS : RVAC_RING_RADIUS;
  const az = (sl ? SL_AZIMUTH0 : RVAC_AZIMUTH0) + (sl ? i : i - SL_COUNT) * ENGINE_AZIMUTH_STEP;
  out[0] = r * Math.cos(az);
  out[1] = r * Math.sin(az);
  out[2] = 0;
  return out;
}
