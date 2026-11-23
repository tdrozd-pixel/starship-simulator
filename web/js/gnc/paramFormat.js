// Binds screen widgets to telemetry parameters named by dictionary path. Pure module (no DOM).
//   "navigation.altitude"                      scalar field of a packet
//   "navigation.angularRate[0]"                array element
//   "guidance_control.engines[1].pitch"        record field of an array element
//   "fsw_status.flags.enginesOn"               flag bit
//   "derived.tilt"                             computed by gncModel (not a dictionary field)
// Packet and field names come from the generated dictionary tables and are validated by
// parseParam, so a dictionary change cannot silently break a screen (a unit test walks every screen).
import { PACKETS, ENUMS, RAD2DEG } from './gncPackets.js';
import { RECORDS } from './gncPackets.generated.js';
import { fmt, fmtSigned, formatLatitude, formatLongitude } from './gncModel.js';

// Derived quantities: name -> unit. The value is model.text[name] from gncModel.buildModel.
export const DERIVED = {
  tilt: 'deg', roll: 'deg', throttle: '%', attX: 'deg', attY: 'deg', attZ: 'deg',
  rawEast: 'm', rawNorth: 'm', rawDist: 'm', rawBearing: 'deg', navEast: 'm', navNorth: 'm', navDist: 'm',
  gnssAge: 's',
};

const RE = /^([a-z_]+)\.([A-Za-z]+)(?:\[(\d+)\])?(?:\.([A-Za-z]+))?$/;
const parsed = new Map();

// Returns a descriptor or null when the path does not exist in the dictionary / derived table.
export function parseParam(path) {
  if (parsed.has(path)) return parsed.get(path);
  const r = parseUncached(path);
  parsed.set(path, r);
  return r;
}

function parseUncached(path) {
  const m = RE.exec(path);
  if (!m) return null;
  const [, pk, name, idxText, sub] = m;
  const index = idxText === undefined ? null : Number(idxText);
  if (pk === 'derived') return name in DERIVED && index === null && !sub ? { path, derived: true, key: name, unit: DERIVED[name] } : null;
  const packet = PACKETS[pk];
  const field = packet && packet.fields.find((f) => f.name === name);
  if (!field || field.type === 'pad') return null;
  const base = { path, derived: false, apid: packet.apid, field, name, index, sub, bit: false, def: field };
  if (field.bits) return index === null && field.bits.some((b) => b.name === sub) ? { ...base, bit: true } : null;
  if (field.type.startsWith('record:')) {
    const rec = RECORDS[field.type.slice(7)];
    const sf = rec && rec.fields.find((f) => f.name === sub && f.type !== 'pad');
    return sf && index !== null && index < field.count ? { ...base, def: sf } : null;
  }
  if (field.count) return index !== null && index < field.count && !sub ? base : null;
  return index === null && !sub ? base : null;
}

export function isValidParam(path) { return parseParam(path) !== null; }

// Raw decoded value from the store, or undefined when that packet has not arrived.
export function resolveParam(store, p) {
  if (p.derived) return undefined;
  const d = store.latest[p.apid];
  if (!d) return undefined;
  if (p.bit) return d[p.sub];
  let v = d[p.name];
  if (p.index !== null) v = v[p.index];
  if (p.sub) v = v[p.sub];
  return v;
}

// Display unit: the dictionary unit with rad and rad/s shown in degrees and fractions in percent.
const UNIT_MAP = { rad: 'deg', 'rad/s': 'deg/s', fraction: '%', 'm/s^2': 'm/s2' };
export function paramUnit(spec) {
  if (spec.unit !== undefined) return spec.unit;
  if (spec.fmt) return ''; // formatted text carries its own hemisphere letter
  const p = parseParam(spec.param);
  if (!p) return '';
  if (p.derived) return p.unit;
  const u = p.def.unit || '';
  return UNIT_MAP[u] || u;
}

// spec: {param, dp?, signed?, fmt?: 'lat'|'lon', bool?: [textTrue, textFalse]}; model: buildModel result.
export function formatParam(spec, store, model) {
  const p = parseParam(spec.param);
  if (!p) return '--';
  if (p.derived) { const t = model.text[p.key]; return t === undefined ? '--' : t; }
  const v = resolveParam(store, p);
  if (v === undefined) return '--';
  if (p.bit || typeof v === 'boolean') {
    const b = spec.bool || ['YES', 'NO'];
    return v ? b[0] : b[1];
  }
  if (p.def.enum) {
    const e = ENUMS[p.def.enum].find((x) => x.value === v);
    return e ? e.label : String(v);
  }
  if (spec.fmt === 'lat') return formatLatitude(v);
  if (spec.fmt === 'lon') return formatLongitude(v);
  let x = v;
  const u = p.def.unit;
  if (u === 'rad' || u === 'rad/s') x = v * RAD2DEG;
  else if (u === 'fraction') x = v * 100;
  const integer = /^[iu](8|16|32)$/.test(p.def.type);
  const dp = spec.dp !== undefined ? spec.dp : integer ? 0 : 2;
  return spec.signed ? fmtSigned(x, dp) : fmt(x, dp);
}
