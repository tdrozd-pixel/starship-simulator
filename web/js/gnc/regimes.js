// Flight regimes and console screens of the GNC position, as data. Pure module (no DOM).
//
// A regime is a phase of flight (prelaunch, ascent, ...) with an ordered list of screens. A screen
// is a declarative spec: rows of widgets (see widgets.js for the renderers). Widgets bind to
// telemetry parameters by dictionary path (paramFormat.js), so adding a screen or a regime is a
// data change in this file; a unit test checks that every referenced parameter exists in the
// telemetry dictionary. Screen ids are shared across regimes ("nav", "gnc", ...), so paging to
// another regime can keep the same kind of screen.
//
// Widget types: readouts, lamps, chart, table, map, gimbals, attitude, timeline, details (collapsible),
// corridor (catch approach view), diagnostics, powerTable (six-engine power settings), note, stack (vertical group). A row is an array of widgets laid out side by side;
// `w` is the relative column width (default 1).
import { ENUMS, GIMBAL_LIMIT_DEG } from './gncPackets.js';
import { NAV_STALE_AMBER_S, NAV_STALE_RED_S, DIFF_THROTTLE_THRESHOLD_PP } from './gncModel.js';

// Series colors (same meaning on every screen: cyan estimate, magenta setpoint or command, dashed
// for command or setpoint, green/amber/red for axes or engines).
const CY = '#35d7ff', MG = '#ff5bd6', AM = '#f0b030', GR = '#4ccf6a', RD = '#ff6a5a', BL = '#6aa0ff';
const SP = { color: MG, dash: [5, 4] };

const chart = (id, title, unit, series, extra) => ({ type: 'chart', id, title, unit, series, ...extra });
const ro = (label, param, extra) => ({ label, param, ...extra });
const row = (label, cells, extra) => ({ label, cells, ...extra });

const C = {
  alt: chart('alt', 'Altitude estimate vs setpoint', 'm', [
    { param: 'navigation.altitude', label: 'estimate', color: CY }, { param: 'guidance_control.altitudeSetpoint', label: 'setpoint', ...SP }]),
  vz: chart('vz', 'Vertical velocity vs setpoint', 'm/s', [
    { param: 'navigation.verticalVelocity', label: 'estimate', color: CY }, { param: 'guidance_control.velocitySetpoint', label: 'setpoint', ...SP }]),
  hpos: chart('hpos', 'Horizontal position vs setpoint (from pad center)', 'm', [
    { param: 'navigation.east', label: 'east', color: CY }, { param: 'navigation.north', label: 'north', color: GR },
    { param: 'guidance_control.eastSetpoint', label: 'east sp', color: CY, dash: [5, 4] },
    { param: 'guidance_control.northSetpoint', label: 'north sp', color: GR, dash: [5, 4] }]),
  pos: chart('pos', 'East / north estimate (from pad center)', 'm', [
    { param: 'navigation.east', label: 'east', color: CY }, { param: 'navigation.north', label: 'north', color: GR }]),
  vel: chart('vel', 'Velocity estimate', 'm/s', [
    { param: 'navigation.velocityEast', label: 'east', color: CY }, { param: 'navigation.velocityNorth', label: 'north', color: GR },
    { param: 'navigation.verticalVelocity', label: 'up', color: AM }]),
  att: chart('att', 'Attitude estimate (rotation vector)', 'deg', [
    { param: 'derived.attX', label: 'X', color: RD }, { param: 'derived.attY', label: 'Y', color: GR }, { param: 'derived.attZ', label: 'Z', color: BL }]),
  rate: chart('rate', 'Body rates (IMU)', 'deg/s', [
    { param: 'navigation.angularRate[0]', label: 'X', color: RD }, { param: 'navigation.angularRate[1]', label: 'Y', color: GR },
    { param: 'navigation.angularRate[2]', label: 'Z', color: BL }]),
  atterr: chart('atterr', 'Attitude error (body)', 'deg', [
    { param: 'guidance_control.attitudeError[0]', label: 'X', color: RD }, { param: 'guidance_control.attitudeError[1]', label: 'Y', color: GR },
    { param: 'guidance_control.attitudeError[2]', label: 'Z', color: BL }]),
  sigpos: chart('sigpos', 'Position 1-sigma (horizontal)', 'm', [
    { param: 'navigation.positionSigmaEast', label: 'east', color: CY }, { param: 'navigation.positionSigmaNorth', label: 'north', color: GR }]),
  innov: chart('innov', 'GNSS innovation (measurement minus prediction)', 'm', [
    { param: 'navigation.gnssInnovationEast', label: 'east', color: CY }, { param: 'navigation.gnssInnovationNorth', label: 'north', color: GR }]),
  altinnov: chart('altinnov', 'Altimeter innovation', 'm', [
    { param: 'navigation.altimeterInnovation', label: 'altimeter', color: AM }]),
  accel: chart('accel', 'Horizontal acceleration commands', 'm/s2', [
    { param: 'guidance_control.eastAccelCommand', label: 'east', color: CY }, { param: 'guidance_control.northAccelCommand', label: 'north', color: GR }]),
  catchLat: chart('catchlat', 'Lateral error vs funnel limit (+b side positive)', 'm', [
    { param: 'catch_status.lateralError', label: 'lateral', color: CY },
    { param: 'catch_status.lateralLimit', label: 'funnel', color: AM, dash: [5, 4] },
    { param: 'derived.negLateralLimit', label: 'funnel', color: AM, dash: [5, 4] }]),
  catchAlong: chart('catchalong', 'Along-axis offset from the seat point', 'm', [
    { param: 'catch_status.alongRemaining', label: 'along remaining', color: CY }]),
  catchSink: chart('catchsink', 'Sink rate (positive down)', 'm/s', [
    { param: 'catch_status.sinkRate', label: 'sink rate', color: AM }]),
  catchPin: chart('catchpin', 'Pin height above seat', 'm', [
    { param: 'catch_status.pinHeightAboveSeat', label: 'pin height', color: GR }]),
  thr: chart('thr', 'Throttle command', '%', [
    { param: 'guidance_control.engines[{e}].throttle', label: 'throttle', color: CY, perEngine: true, allEngines: true }]),
  // All six engines (index 3 to 5 are the RVacs: zero in the hop, shown so the display is complete).
  engthr: chart('engthr', 'Actual throttle, all six engines (RVac zero in the hop)', '%', [
    { param: 'engine_status.engines[{e}].throttle', label: '', color: CY, perEngine: true, powerEngines: true }]),
  thrdev: chart('thrdev', 'SL throttle command deviation from mean', 'pp', [
    { param: 'derived.thrDev[{e}]', label: 'dev', color: CY, perEngine: true, allEngines: true }]),
  gimbal: chart('gimbal', 'Gimbal command (dashed) vs measured', 'deg', [
    { param: 'guidance_control.engines[{e}].pitch', label: 'pitch cmd', color: CY, dash: [5, 4], perEngine: true },
    { param: 'actuator_feedback.engines[{e}].pitch', label: 'pitch meas', color: CY, perEngine: true },
    { param: 'guidance_control.engines[{e}].yaw', label: 'yaw cmd', color: MG, dash: [5, 4], perEngine: true },
    { param: 'actuator_feedback.engines[{e}].yaw', label: 'yaw meas', color: MG, perEngine: true }]),
};

const MODE_ROW = row('MODE', [ro('', 'fsw_status.mode'), ro('', 'fsw_status.timeInMode', { dp: 1 })], { sep: ', ', unit: 's in mode' });
const RVAC_ROW = row('RVAC', [ro('', 'derived.rvacStatus')], { unit: '' }); // from engine_status
const BIG = {
  alt: ro('ALTITUDE', 'navigation.altitude', { unit: 'm' }),
  vz: ro('VERTICAL SPEED', 'navigation.verticalVelocity', { signed: true, unit: 'm/s' }),
  tilt: ro('TILT', 'derived.tilt'),
  throttle: ro('THROTTLE', 'derived.throttle', { unit: '%' }),
  prop: ro('PROPELLANT', 'fsw_status.propellant', { dp: 0 }),
};
const FLAGS = [
  ro('ENGINES ON', 'fsw_status.flags.enginesOn'), ro('NAV VALID', 'fsw_status.flags.navValid'),
  ro('TRANSLATING', 'fsw_status.flags.translating'), ro('ALIGNED', 'fsw_status.flags.aligned'), ro('GNSS READY', 'fsw_status.flags.gnssReady'),
];
const MAP_LEGEND = 'nav estimate (cyan), raw GNSS (amber), setpoint (magenta), align tolerance (green); pad circle: placeholder; catch: tower mast (grey), arms in the open pose from the downlinked geometry (blue, not live), seat point (cyan; the Ship descends vertically over it)';

// ---- Common screens (every regime that lists them shares the same spec) ----

const NAV = {
  id: 'nav', title: 'Navigation',
  rows: [[
    { type: 'stack', w: 1, children: [
      { type: 'table', title: 'GNSS', rows: [
        row('FIX', [ro('', 'gnss.fixType')]),
        row('SATS USED / VISIBLE', [ro('', 'gnss.satellitesUsed'), ro('', 'gnss.satellitesVisible')], { sep: ' / ', unit: '' }),
        row('LATITUDE', [ro('', 'gnss.latitude', { fmt: 'lat' })]),
        row('LONGITUDE', [ro('', 'gnss.longitude', { fmt: 'lon' })]),
        row('HEIGHT (ELLIPSOID)', [ro('', 'gnss.height')]),
        row('SIGMA H / V', [ro('', 'gnss.horizontalSigma'), ro('', 'gnss.verticalSigma')], { sep: ' / ', unit: 'm' }),
        row('SOLUTION AGE', [ro('', 'derived.gnssAge')]),
      ] },
      { type: 'table', title: 'Deltas from pad center [m, deg]', head: ['EAST', 'NORTH', 'DIST', 'BRG'], rows: [
        row('RAW GNSS', ['rawEast', 'rawNorth', 'rawDist', 'rawBearing'].map((k) => ro('', 'derived.' + k))),
        row('NAV EST.', ['navEast', 'navNorth', 'navDist'].map((k) => ro('', 'derived.' + k))),
      ] },
      { type: 'note', text: 'Raw GNSS is at the antenna; the nav estimate is at the IMU, so they differ by the lever arm and by filter error.' },
    ] },
    { type: 'map', title: 'Position (top-down, pad centered)', note: MAP_LEGEND, w: 1 },
    { type: 'stack', w: 1.3, children: [C.pos, C.vel] },
  ]],
};

const NAV_HEALTH = {
  id: 'navhealth', title: 'Nav Health',
  rows: [[
    { type: 'stack', w: 1, children: [
      { type: 'table', title: 'Filter covariance (1-sigma)', head: ['EAST', 'NORTH'], rows: [
        row('POSITION [m]', ['East', 'North'].map((k) => ro('', 'navigation.positionSigma' + k, { dp: 2 })), { unit: '' }),
        row('VELOCITY [m/s]', ['East', 'North'].map((k) => ro('', 'navigation.velocitySigma' + k, { dp: 3 })), { unit: '' }),
      ] },
      { type: 'table', title: 'Measurement innovations', rows: [
        row('GNSS EAST / NORTH', [ro('', 'navigation.gnssInnovationEast', { signed: true }), ro('', 'navigation.gnssInnovationNorth', { signed: true })], { sep: ' / ', unit: 'm' }),
        row('GNSS MAGNITUDE', [ro('', 'derived.gnssInnovMag')]),
        row('ALTIMETER', [ro('', 'navigation.altimeterInnovation', { signed: true })]),
      ] },
      { type: 'table', title: 'Time since last accepted update', rows: [
        row('GNSS', [ro('', 'derived.gnssUpdAge')]),
        row('ALTIMETER', [ro('', 'derived.altUpdAge')]),
      ] },
      { type: 'lamps', title: 'Data staleness (placeholder thresholds)', items: [
        ro('GNSS STALE > ' + NAV_STALE_AMBER_S + ' s', 'derived.gnssStaleAmber', { tone: 'amber' }), ro('GNSS STALE > ' + NAV_STALE_RED_S + ' s', 'derived.gnssStaleRed', { tone: 'red' }),
        ro('ALTIMETER STALE > ' + NAV_STALE_AMBER_S + ' s', 'derived.altStaleAmber', { tone: 'amber' }), ro('ALTIMETER STALE > ' + NAV_STALE_RED_S + ' s', 'derived.altStaleRed', { tone: 'red' })] },
      { type: 'note', text: 'Sigmas are the horizontal Kalman covariance diagonals (0 until the filter starts); age NONE YET means no accepted update. The vertical estimate is a complementary filter with no covariance, and no measurement gating exists, so no rejected-measurement count is shown. Stale thresholds are placeholders. TODO(phase 10, item 48): limit system.' },
    ] },
    { type: 'stack', w: 1.3, children: [C.sigpos, C.altinnov] },
    { type: 'stack', w: 1.3, children: [C.innov] },
  ]],
};

const GNC = {
  id: 'gnc', title: 'Guidance & Control',
  rows: [[
    { type: 'stack', w: 1, children: [
      { type: 'readouts', small: true, items: [ro('TILT', 'derived.tilt'), ro('ROLL (BODY X BEARING)', 'derived.roll'), ro('ATT ERROR (CMD VS ACTUAL)', 'derived.attErrMag')] },
      { type: 'table', title: 'Attitude error, command minus actual (body axes)', head: ['X', 'Y', 'Z', 'MAG'], rows: [
        row('ERROR [deg]', [0, 1, 2].map((i) => ro('', 'guidance_control.attitudeError[' + i + ']', { signed: true })).concat([ro('', 'derived.attErrMag')])),
      ] },
      { type: 'attitude', title: 'Attitude (estimate)' },
      { type: 'details', title: 'Rotation vector and body rates', children: [
        { type: 'table', rows: [
          row('ATT X / Y / Z', ['attX', 'attY', 'attZ'].map((k) => ro('', 'derived.' + k)), { sep: ' / ', unit: 'deg' }),
          row('RATE X / Y / Z', [0, 1, 2].map((i) => ro('', 'navigation.angularRate[' + i + ']')), { sep: ' / ', unit: 'deg/s' }),
        ] }] },
    ] },
    { type: 'stack', w: 1.3, children: [C.att, C.rate, C.atterr] },
    { type: 'stack', w: 1.3, children: [C.alt, C.vz, C.accel] },
  ]],
};

const ACTUATORS = {
  id: 'actuators', title: 'Actuators',
  rows: [
    [{ type: 'gimbals', title: 'Engine gimbals, sea-level engines (+/-' + GIMBAL_LIMIT_DEG + ' deg)' }],
    [{ type: 'note', text: 'Only the three sea-level engines (SL1 to SL3) gimbal. The three Raptor Vacuum engines (RV1 to RV3) are fixed and have no gimbal display.' }],
    [
      { type: 'table', title: 'Actuator margins', w: 1.4, head: ['SL1', 'SL2', 'SL3'], rows: [
        row('GIMBAL CMD [% OF ' + GIMBAL_LIMIT_DEG + ' DEG]', [0, 1, 2].map((i) => ro('', 'derived.gimCmdPct[' + i + ']'))),
        row('GIMBAL MEASURED [% OF LIMIT]', [0, 1, 2].map((i) => ro('', 'derived.gimMeasPct[' + i + ']'))),
        row('TRACKING ERROR CMD-MEAS [deg]', [0, 1, 2].map((i) => ro('', 'derived.gimErr[' + i + ']'))),
        row('THROTTLE CMD, GUIDANCE [%]', [0, 1, 2].map((i) => ro('', 'derived.thrCmd[' + i + ']'))),
        row('THROTTLE ACTUAL, ENGINE FEEDBACK [%]', [0, 1, 2].map((i) => ro('', 'derived.thrAct[' + i + ']'))),
      ] },
      { type: 'stack', w: 1, children: [0, 1, 2].map((i) => ({ type: 'lamps', title: 'SL' + (i + 1) + ' flags', items: [
        ro('GIMBAL >= 90%', 'derived.gimWarn[' + i + ']', { tone: 'amber' }), ro('GIMBAL AT LIMIT', 'derived.gimLimit[' + i + ']', { tone: 'red' }),
        ro('THROTTLE CMD 100%', 'derived.thrMax[' + i + ']', { tone: 'amber' }),
        ro('ENGINE OUT / GIMBAL SENSOR INVALID', 'derived.engInvalid[' + i + ']', { tone: 'red' })] })) },
    ],
    [
      { type: 'powerTable', title: 'Power settings', w: 1.6 },
      { type: 'stack', w: 1, children: [
        { type: 'table', title: 'SL throttle command, deviation from mean', head: ['SL1', 'SL2', 'SL3'], rows: [
          row('DEVIATION [pp]', [0, 1, 2].map((i) => ro('', 'derived.thrDev[' + i + ']'))),
          row('SPREAD MAX-MIN [pp]', [ro('', 'derived.thrSpread')]),
        ] },
        { type: 'lamps', items: [ro('DIFF THROTTLE ACTIVE', 'derived.diffThrottle', { tone: 'amber' })] },
        { type: 'note', text: 'Differential throttle: lamp when the commanded SL throttles differ by more than ' + DIFF_THROTTLE_THRESHOLD_PP + ' percentage points (placeholder threshold). TODO(phase 10, item 48): limit system.' },
      ] },
    ],
    [{ ...C.thr, w: 1 }, { ...C.engthr, w: 1 }, { ...C.thrdev, w: 1 }],
    [{ ...C.gimbal, w: 1 }],
  ],
};

const FSW = {
  id: 'fsw', title: 'Flight Software',
  rows: [[
    { type: 'stack', w: 1, children: [
      { type: 'table', title: 'Flight software', rows: [
        MODE_ROW,
        row('GNSS FIXES / CYCLES', [ro('', 'fsw_status.gnssFixCount'), ro('', 'fsw_status.cycleCount')], { sep: ' / ', unit: '' }),
        row('PROPELLANT EST.', [ro('', 'fsw_status.propellant', { dp: 0 })]),
        row('MASS EST.', [ro('', 'fsw_status.mass', { dp: 0 })]),
        row('ALTITUDE EST. / SP', [ro('', 'navigation.altitude'), ro('', 'guidance_control.altitudeSetpoint')], { sep: ' / ', unit: 'm' }),
        row('V SPEED EST. / SP', [ro('', 'navigation.verticalVelocity', { signed: true }), ro('', 'guidance_control.velocitySetpoint', { signed: true })], { sep: ' / ', unit: 'm/s' }),
        row('HORIZ. SETPOINT E / N', [ro('', 'guidance_control.eastSetpoint', { signed: true }), ro('', 'guidance_control.northSetpoint', { signed: true })], { sep: ' / ', unit: 'm' }),
        row('ALTIMETER', [ro('', 'navigation.altimeterRange'), ro('', 'navigation.altimeterValid', { bool: ['VALID', 'INVALID'] })], { sep: ' m ', unit: '' }),
        RVAC_ROW,
      ] },
      { type: 'lamps', items: FLAGS },
      { type: 'details', title: 'Mission configuration', children: [
        { type: 'table', rows: [
          row('TARGET (LAT / LON)', [ro('', 'mission_config.targetLatitude', { fmt: 'lat' }), ro('', 'mission_config.targetLongitude', { fmt: 'lon' })], { sep: ' ', unit: '' }),
          row('TARGET HEIGHT (ELLIPSOID)', [ro('', 'mission_config.targetHeight', { dp: 1 })]),
          row('APEX ALTITUDE', [ro('', 'mission_config.targetAltitude', { dp: 1 })]),
          row('TRANSLATE ALTITUDE', [ro('', 'mission_config.translateAltitude', { dp: 1 })]),
          row('ALIGN TOLERANCE', [ro('', 'mission_config.alignTolerance', { dp: 1 })]),
        ] }] },
    ] },
    { type: 'diagnostics', title: 'Packet diagnostics (wall clock)', w: 1.3 },
  ]],
};

// ---- Regime-specific first screens (id "overview") ----

const READINESS = {
  id: 'overview', title: 'Readiness',
  rows: [
    [{ type: 'readouts', items: [
      ro('GNSS FIX', 'gnss.fixType'), ro('SATS USED', 'gnss.satellitesUsed'), ro('SATS VISIBLE', 'gnss.satellitesVisible'),
      ro('SIGMA H', 'gnss.horizontalSigma', { unit: 'm' }), ro('MODE', 'fsw_status.mode'),
    ] }],
    [
      { type: 'stack', w: 1, children: [
        { type: 'lamps', title: 'Readiness', items: [ro('GNSS READY', 'fsw_status.flags.gnssReady'), ro('NAV VALID', 'fsw_status.flags.navValid'), ro('ENGINES ON', 'fsw_status.flags.enginesOn')] },
        { type: 'table', title: 'Sensors', rows: [
          row('GNSS SOLUTION AGE', [ro('', 'derived.gnssAge')]),
          row('GNSS FIXES (COUNT)', [ro('', 'fsw_status.gnssFixCount')], { unit: '' }),
          row('ALTIMETER', [ro('', 'navigation.altimeterValid', { bool: ['VALID', 'INVALID'] })], { unit: '' }),
          row('GNSS DIST FROM PAD', [ro('', 'derived.rawDist')]),
        ] },
      ] },
      { type: 'table', title: 'Mission configuration', w: 1, rows: [
        row('TARGET LATITUDE', [ro('', 'mission_config.targetLatitude', { fmt: 'lat' })], { unit: '' }),
        row('TARGET LONGITUDE', [ro('', 'mission_config.targetLongitude', { fmt: 'lon' })], { unit: '' }),
        row('TARGET HEIGHT (ELLIPSOID)', [ro('', 'mission_config.targetHeight', { dp: 1 })]),
        row('APEX ALTITUDE', [ro('', 'mission_config.targetAltitude', { dp: 1 })]),
        row('TRANSLATE ALTITUDE', [ro('', 'mission_config.translateAltitude', { dp: 1 })]),
        row('ALIGN TOLERANCE', [ro('', 'mission_config.alignTolerance', { dp: 1 })]),
      ] },
    ],
  ],
};

const ASCENT = {
  id: 'overview', title: 'Ascent',
  rows: [
    [{ type: 'readouts', items: [BIG.alt, BIG.vz, BIG.throttle, BIG.tilt, BIG.prop] }],
    [
      { type: 'stack', w: 1, children: [{ type: 'table', title: 'Flight computer', rows: [MODE_ROW] }, { type: 'timeline', title: 'Mode timeline (onboard time)' }] },
      { ...C.alt, w: 1.3 }, { ...C.vz, w: 1.3 },
    ],
  ],
};

const LANDING = {
  id: 'overview', title: 'Landing',
  rows: [
    [{ type: 'readouts', items: [
      BIG.alt, BIG.vz, ro('ALTITUDE SETPOINT', 'guidance_control.altitudeSetpoint', { unit: 'm' }),
      ro('HORIZ ERROR TO PAD (EST.)', 'derived.navDist', { unit: 'm' }), ro('MODE', 'fsw_status.mode'),
    ] }],
    [{ type: 'readouts', small: true, items: [
      ro('TIME TO GO (ESTIMATE)', 'derived.predTtg', { unit: 's' }), ro('PRED. MISS AT TOUCHDOWN (ESTIMATE)', 'derived.predMiss', { unit: 'm' }),
      ro('TOUCHDOWN GO/NO-GO (ESTIMATE)', 'derived.predGo'),
    ] }],
    [
      { type: 'stack', w: 1, children: [
        { type: 'table', title: 'Touchdown', rows: [
          MODE_ROW,
          row('ALTIMETER', [ro('', 'navigation.altimeterRange'), ro('', 'navigation.altimeterValid', { bool: ['VALID', 'INVALID'] })], { sep: ' m ', unit: '' }),
          RVAC_ROW,
        ] },
        { type: 'table', title: 'Touchdown prediction (ESTIMATE)', rows: [
          row('TIME TO GO (AT CURRENT VZ)', [ro('', 'derived.predTtg')]),
          row('TIME TO GO (AT VZ SETPOINT)', [ro('', 'derived.predTtgSp')]),
          row('PRED. EAST / NORTH', [ro('', 'derived.predEast'), ro('', 'derived.predNorth')], { sep: ' / ', unit: 'm' }),
          row('MISS VS PAD CENTER', [ro('', 'derived.predMiss')]),
          row('MISS VS HORIZ. SETPOINT', [ro('', 'derived.predMissSp')]),
          row('ALIGN TOLERANCE', [ro('', 'mission_config.alignTolerance', { dp: 1 })]),
        ] },
        { type: 'lamps', items: [ro('PRED. GO', 'derived.predGo'), ro('PRED. NO-GO', 'derived.predNoGo', { tone: 'red' }),
          ro('ENGINES ON', 'fsw_status.flags.enginesOn'), ro('ALIGNED', 'fsw_status.flags.aligned'), ro('NAV VALID', 'fsw_status.flags.navValid')] },
        { type: 'note', text: 'ESTIMATE: simple ballistic-style extrapolation of the nav estimate (current vertical and horizontal velocity held until altitude 0), not the flight guidance. GO means predicted miss vs pad center is within the align tolerance.' },
        { type: 'map', title: 'Position (top-down, pad centered)', note: MAP_LEGEND },
      ] },
      { type: 'stack', w: 1.3, children: [C.alt, C.vz] },
      { type: 'stack', w: 1.3, children: [C.hpos, C.thr] },
    ],
  ],
};

const CATCH = {
  id: 'overview', title: 'Catch',
  rows: [
    [{ type: 'readouts', items: [
      ro('ALONG OFFSET (+ = SHORT OF SEAT)', 'catch_status.alongRemaining', { signed: true, unit: 'm' }),
      ro('LATERAL ERROR (+ = +b SIDE)', 'catch_status.lateralError', { signed: true, unit: 'm' }),
      ro('PIN HEIGHT ABOVE SEAT', 'catch_status.pinHeightAboveSeat', { signed: true, unit: 'm' }),
      ro('TIME TO SEAT', 'catch_status.timeToSeat', { unit: 's' }), ro('MODE', 'fsw_status.mode'),
    ] }],
    [{ type: 'readouts', small: true, items: [
      ro('ALONG SPEED', 'catch_status.alongSpeed', { signed: true, unit: 'm/s' }),
      ro('LATERAL SPEED', 'catch_status.lateralSpeed', { signed: true, unit: 'm/s' }),
      ro('SINK RATE (+ = DOWN)', 'catch_status.sinkRate', { signed: true, unit: 'm/s' }),
      ro('TILT', 'catch_status.tilt'), ro('SIDE OF AXIS', 'derived.catchSide', { unit: '' }),
    ] }],
    [
      { type: 'stack', w: 1, children: [
        { type: 'table', title: 'Catch state', rows: [
          row('PHASE', [ro('', 'catch_status.phase')]),
          row('ABORT REASON', [ro('', 'catch_status.abortReason')]),
          MODE_ROW,
          row('ARMS (EXPECTED PER SEQUENCE, NOT MEASURED)', [ro('', 'catch_status.armsExpected')]),
          row('LANDING MODE', [ro('', 'catch_config.landingMode')]),
        ] },
        { type: 'lamps', title: 'Go / no-go', items: [
          ro('GO', 'catch_status.flags.go'), ro('NO-GO', 'derived.catchNoGo', { tone: 'red' }),
          ro('POSITION', 'catch_status.flags.positionOk'), ro('SPEED', 'catch_status.flags.speedOk'), ro('TILT', 'catch_status.flags.tiltOk'),
          ro('RTK', 'catch_status.flags.rtkOk'), ro('IN FUNNEL', 'catch_status.flags.corridorOk'),
          ro('ABORTED', 'catch_status.flags.aborted', { tone: 'red' })] },
        { type: 'table', title: 'Catch window (mission data)', rows: [
          row('LATERAL, +/-', [ro('', 'catch_config.windowLateral')]),
          row('ALONG, +/-', [ro('', 'catch_config.windowAlong')]),
          row('SINK RATE, MAX', [ro('', 'catch_config.windowSink')]),
          row('TILT, MAX', [ro('', 'catch_config.windowTilt')]),
          row('FUNNEL LIMIT NOW', [ro('', 'catch_status.lateralLimit')]),
        ] },
        { type: 'note', text: 'Arm state is what the flight sequence expects (time since the arms-closing height); the arm position is not downlinked, so it is never measured here. The Ship descends vertically over the seat: along and lateral are the horizontal offsets from the seat point in the tower approach frame, and the funnel is the flight software gate at the current height above the seat (about 2x the physical catch window at the seat). The pin is hypothetical (estimate).' },
      ] },
      { type: 'corridor', title: 'Descent corridor', w: 1.4 },
      { type: 'stack', w: 1.3, children: [C.catchLat, C.catchAlong] },
      { type: 'stack', w: 1.3, children: [C.catchSink, C.catchPin] },
    ],
  ],
};

// FSW modes (ENUMS.mode names) per regime. TODO(phase 10, item 48): move the mode to regime
// mapping into the telemetry dictionary mode enum so flight, ground and consoles share one table.
export const REGIMES = [
  { id: 'prelaunch', label: 'Prelaunch', status: 'live', modes: ['Prelaunch'],
    description: 'On the pad before ignition: readiness of navigation and sensors.', screens: [READINESS, NAV, NAV_HEALTH, FSW] },
  { id: 'ascent', label: 'Ascent', status: 'live', modes: ['Ignition', 'Ascent'],
    description: 'Engine start and climb to the apex altitude.', screens: [ASCENT, GNC, ACTUATORS, NAV, NAV_HEALTH, FSW] },
  { id: 'staging', label: 'Staging', status: 'planned', modes: [], plannedScreens: ['Separation', 'Booster', 'Ship'],
    description: 'Hot staging and booster separation.', note: 'Not yet simulated (build order item 41).' },
  { id: 'entry', label: 'Entry', status: 'planned', modes: [], plannedScreens: ['Entry corridor', 'Heating', 'Aerosurfaces'],
    description: 'Atmospheric entry and aerodynamic descent.', note: 'Not yet simulated (build order items 17 to 26).' },
  { id: 'landing', label: 'Landing', status: 'live', modes: ['Descent', 'FinalDescent', 'Landed'],
    description: 'Descent, final descent and touchdown.', screens: [LANDING, ACTUATORS, NAV, NAV_HEALTH, FSW],
    note: 'Landing and catch: build order item 40.' },
  { id: 'catch', label: 'Catch', status: 'live', modes: ['CatchApproach', 'CatchHold', 'CatchFinal', 'Caught'],
    description: 'Vertical descent over the seat and between the arms (approach, arms closing, final), and the caught state.', screens: [CATCH, ACTUATORS, NAV, NAV_HEALTH, FSW] },
];

export const DEFAULT_REGIME = 'prelaunch';

const byId = new Map(REGIMES.map((r) => [r.id, r]));
const modeRegime = new Map(); // mode value -> regime id
for (const m of ENUMS.mode) {
  const r = REGIMES.find((x) => x.modes.includes(m.name));
  if (r) modeRegime.set(m.value, r.id);
}

export function getRegime(id) { return byId.get(id) || null; }
export function isLive(id) { const r = byId.get(id); return !!r && r.status === 'live'; }
export function regimeForMode(modeValue) { return modeRegime.get(modeValue) || null; }
export function screensOf(regimeId) { return isLive(regimeId) ? byId.get(regimeId).screens : []; }
export function getScreen(regimeId, screenId) { return screensOf(regimeId).find((s) => s.id === screenId) || null; }
// The screen to show in a regime: the preferred kind if the regime has it, else its first screen.
export function resolveScreen(regimeId, preferred) {
  const list = screensOf(regimeId);
  if (!list.length) return null;
  return list.some((s) => s.id === preferred) ? preferred : list[0].id;
}
// Keys '1'..'9' pick the screens of a regime in order.
export function screenFromKey(regimeId, key) {
  const n = Number(key);
  const list = screensOf(regimeId);
  return Number.isInteger(n) && n >= 1 && n <= list.length ? list[n - 1].id : null;
}

// ---- URL hash: #/<regime>/<screen> ----
export function formatHash(regimeId, screenId) { return '#/' + regimeId + '/' + screenId; }
// Returns {regime, screen} for a live regime ("#/landing" alone selects its first screen) or null.
export function parseHash(hash) {
  const m = /^#?\/([a-z]+)(?:\/([a-z]+))?\/?$/.exec(String(hash || '').toLowerCase());
  if (!m || !isLive(m[1])) return null;
  const screen = resolveScreen(m[1], m[2]);
  if (m[2] && screen !== m[2]) return null;
  return { regime: m[1], screen };
}
// The URL hash wins (deep link), then the remembered manual selection, then the default.
export function initialSelection(hash, stored) {
  const sel = parseHash(hash) || parseHash(stored);
  return sel || { regime: DEFAULT_REGIME, screen: screensOf(DEFAULT_REGIME)[0].id };
}

// ---- Follow phase ----
// state: {follow, regime, screen}. All functions return new states and never mutate.
// Following: a mode that maps to another live regime pages there, keeping the same screen kind
// when the new regime has it. notice is the text for the header, or null.
export function followStep(state, modeValue, announce = true) {
  const next = regimeForMode(modeValue);
  if (!state.follow || !next || next === state.regime) return { state, notice: null };
  const notice = announce ? 'Phase: ' + byId.get(state.regime).label.toUpperCase() + ' -> ' + byId.get(next).label.toUpperCase() : null;
  return { state: { follow: true, regime: next, screen: resolveScreen(next, state.screen) }, notice };
}
// Operator picks a regime and/or screen: planned regimes cannot be opened; a regime change turns follow off.
export function selectManual(state, regimeId, screenId) {
  if (!isLive(regimeId)) return state;
  const screen = resolveScreen(regimeId, screenId);
  return { follow: regimeId === state.regime ? state.follow : false, regime: regimeId, screen };
}
// One-click way back: follow on and jump to the regime of the current mode (if known).
export function resumeFollow(state, modeValue) {
  const next = regimeForMode(modeValue);
  if (!next) return { ...state, follow: true };
  return { follow: true, regime: next, screen: resolveScreen(next, state.screen) };
}

// ---- Parameter references (tests, validation) ----
// Every dictionary parameter path a screen binds to. Per-engine series are listed for engine 0.
export function screenParams(screen) {
  const out = [];
  const walk = (w) => {
    for (const it of w.items || []) out.push(it.param);
    for (const r of w.rows || []) for (const c of r.cells || []) out.push(c.param);
    for (const s of w.series || []) out.push(s.param.replace('{e}', '0'));
    for (const c of w.children || []) walk(c);
  };
  for (const r of screen.rows) for (const w of r) walk(w);
  return out;
}
// Chart series parameters only (they must also be stored columns).
export function screenChartParams(screen) {
  const out = [];
  const walk = (w) => {
    if (w.type === 'chart') for (const s of w.series) out.push(s.param.replace('{e}', '0'));
    for (const c of w.children || []) walk(c);
  };
  for (const r of screen.rows) for (const w of r) walk(w);
  return out;
}
