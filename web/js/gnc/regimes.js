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
// diagnostics, note, stack (vertical group). A row is an array of widgets laid out side by side;
// `w` is the relative column width (default 1).
import { ENUMS } from './gncPackets.js';

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
  accel: chart('accel', 'Horizontal acceleration commands', 'm/s2', [
    { param: 'guidance_control.eastAccelCommand', label: 'east', color: CY }, { param: 'guidance_control.northAccelCommand', label: 'north', color: GR }]),
  thr: chart('thr', 'Throttle command', '%', [
    { param: 'guidance_control.engines[{e}].throttle', label: 'throttle', color: CY, perEngine: true, allEngines: true }]),
  gimbal: chart('gimbal', 'Gimbal command (dashed) vs measured', 'deg', [
    { param: 'guidance_control.engines[{e}].pitch', label: 'pitch cmd', color: CY, dash: [5, 4], perEngine: true },
    { param: 'actuator_feedback.engines[{e}].pitch', label: 'pitch meas', color: CY, perEngine: true },
    { param: 'guidance_control.engines[{e}].yaw', label: 'yaw cmd', color: MG, dash: [5, 4], perEngine: true },
    { param: 'actuator_feedback.engines[{e}].yaw', label: 'yaw meas', color: MG, perEngine: true }]),
};

const MODE_ROW = row('MODE', [ro('', 'fsw_status.mode'), ro('', 'fsw_status.timeInMode', { dp: 1 })], { sep: ', ', unit: 's in mode' });
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
const MAP_LEGEND = 'nav estimate (cyan), raw GNSS (amber), setpoint (magenta), align tolerance (green); pad circle: placeholder';

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

const GNC = {
  id: 'gnc', title: 'Guidance & Control',
  rows: [[
    { type: 'stack', w: 1, children: [
      { type: 'readouts', small: true, items: [ro('TILT', 'derived.tilt'), ro('ROLL (BODY X BEARING)', 'derived.roll')] },
      { type: 'attitude', title: 'Attitude (estimate)' },
      { type: 'details', title: 'Rotation vector and body rates', children: [
        { type: 'table', rows: [
          row('ATT X / Y / Z', ['attX', 'attY', 'attZ'].map((k) => ro('', 'derived.' + k)), { sep: ' / ', unit: 'deg' }),
          row('RATE X / Y / Z', [0, 1, 2].map((i) => ro('', 'navigation.angularRate[' + i + ']')), { sep: ' / ', unit: 'deg/s' }),
        ] }] },
    ] },
    { type: 'stack', w: 1.3, children: [C.att, C.rate] },
    { type: 'stack', w: 1.3, children: [C.alt, C.vz, C.accel] },
  ]],
};

const ACTUATORS = {
  id: 'actuators', title: 'Actuators',
  rows: [
    [{ type: 'gimbals', title: 'Engine gimbals (+/-15 deg)' }],
    [{ ...C.thr, w: 1 }, { ...C.gimbal, w: 2 }],
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
    [
      { type: 'stack', w: 1, children: [
        { type: 'table', title: 'Touchdown', rows: [
          MODE_ROW,
          row('ALTIMETER', [ro('', 'navigation.altimeterRange'), ro('', 'navigation.altimeterValid', { bool: ['VALID', 'INVALID'] })], { sep: ' m ', unit: '' }),
        ] },
        { type: 'lamps', items: [ro('ENGINES ON', 'fsw_status.flags.enginesOn'), ro('ALIGNED', 'fsw_status.flags.aligned'), ro('NAV VALID', 'fsw_status.flags.navValid')] },
        { type: 'map', title: 'Position (top-down, pad centered)', note: MAP_LEGEND },
      ] },
      { type: 'stack', w: 1.3, children: [C.alt, C.vz] },
      { type: 'stack', w: 1.3, children: [C.hpos, C.thr] },
    ],
  ],
};

// FSW modes (ENUMS.mode names) per regime. TODO(phase 10, item 48): move the mode to regime
// mapping into the telemetry dictionary mode enum so flight, ground and consoles share one table.
export const REGIMES = [
  { id: 'prelaunch', label: 'Prelaunch', status: 'live', modes: ['Prelaunch'],
    description: 'On the pad before ignition: readiness of navigation and sensors.', screens: [READINESS, NAV, FSW] },
  { id: 'ascent', label: 'Ascent', status: 'live', modes: ['Ignition', 'Ascent'],
    description: 'Engine start and climb to the apex altitude.', screens: [ASCENT, GNC, ACTUATORS, NAV, FSW] },
  { id: 'staging', label: 'Staging', status: 'planned', modes: [], plannedScreens: ['Separation', 'Booster', 'Ship'],
    description: 'Hot staging and booster separation.', note: 'Not yet simulated (build order item 41).' },
  { id: 'entry', label: 'Entry', status: 'planned', modes: [], plannedScreens: ['Entry corridor', 'Heating', 'Aerosurfaces'],
    description: 'Atmospheric entry and aerodynamic descent.', note: 'Not yet simulated (build order items 17 to 26).' },
  { id: 'landing', label: 'Landing', status: 'live', modes: ['Descent', 'FinalDescent', 'Landed'],
    description: 'Descent, final descent and touchdown.', screens: [LANDING, ACTUATORS, NAV, FSW],
    note: 'Landing and catch: build order item 40.' },
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
