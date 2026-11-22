// Visual-only parameters. Nothing here feeds physics.
// Every vehicle number says where it came from: "public figure" or "estimate".

export const VISUAL = Object.freeze({
  shipDiameter: 9.0, // [m] public figure
  shipHeight: 52.0, // [m] public figure, approximate Ship height
  plumeRadius: 1.5, // [m] estimate (Raptor nozzle exit ~1.3 m, drawn wider)
  plumeMaxLength: 40.0, // [m] visual estimate
  plumeMinOpacity: 0.25, // style
  plumeMaxOpacity: 0.85, // style
  padRadius: 20.0, // [m] placeholder
  poleOffsetEast: 30.0, // [m] placeholder
  poleHeight: 100.0, // [m] placeholder
  tickSpacing: 10.0, // [m] placeholder
  fovDeg: 45, // style
  defaultCamPos: Object.freeze([80, -110, 35]), // sim ENU [m] (east, north, up), style
  near: 0.5, // [m]
  far: 5000, // [m]
  pixelRatioCap: 2,
  // Interpolation constants (see interpolator.js)
  interpDelaySnapshots: 2.5, // snapshots of render delay
  bufferSeconds: 2.0, // [s] sim time kept
  offsetWindowSeconds: 3.0, // [s] wall window for min-delay clock offset
  staleSeconds: 3.0, // [s] without any message before STALE
});

export function placeholderNote(vehicle) {
  let s = 'Visual: Ship-sized placeholder cylinder (' + VISUAL.shipDiameter + ' m x ' + VISUAL.shipHeight + ' m), not the physics shape.';
  if (vehicle) {
    s += ' Physics vehicle: ' + vehicle.name + ', length ' + vehicle.length + ' m, radius ' + vehicle.radius + ' m.';
  }
  return s;
}
