// Visual-only parameters. Nothing here feeds physics.
// Every vehicle number says where it came from: "public figure" or "estimate".

export const VISUAL = Object.freeze({
  shipDiameter: 9.0, // [m] public figure
  shipHeight: 52.0, // [m] public figure, approximate Ship height
  plumeRadiusSl: 0.85, // [m] estimate (SL nozzle exit 1.3 m, drawn a little wider)
  plumeRadiusRvac: 1.5, // [m] estimate (RVac nozzle exit 2.4 m, drawn a little wider)
  nozzleExitRadiusSl: 0.65, // [m] estimate, HopperParams nozzleExitDiameter / 2
  nozzleExitRadiusRvac: 1.2, // [m] estimate, HopperParams rvacNozzleExitDiameter / 2
  plumeMaxLength: 40.0, // [m] visual estimate
  plumeMinOpacity: 0.25, // style
  plumeMaxOpacity: 0.85, // style
  padRadius: 20.0, // [m] placeholder
  poleOffsetEast: 30.0, // [m] placeholder
  poleHeight: 100.0, // [m] placeholder
  tickSpacing: 10.0, // [m] placeholder
  pinRadius: 0.25, // [m] visual, per the catch plan
  fovDeg: 45, // style
  defaultCamPos: Object.freeze([80, -110, 35]), // sim ENU [m] (east, north, up), style
  near: 0.5, // [m]
  far: 5000, // [m]
  pixelRatioCap: 2,
  // Built-in recorder (see recorder.js)
  recordBitsPerPixel: 0.1, // [bit/px/frame] style: ~12 Mbit/s at 1080p60
  recordMemoryWarnBytes: 1e9, // [B] recordings are held in memory until saved
  // Interpolation constants (see interpolator.js)
  interpDelaySnapshots: 2.5, // snapshots of render delay
  bufferSeconds: 2.0, // [s] sim time kept
  offsetWindowSeconds: 3.0, // [s] wall window for min-delay clock offset
  staleSeconds: 3.0, // [s] without any message before STALE
});
