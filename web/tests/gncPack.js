// Test-only big-endian packers for GNC telemetry packets and ground frames (docs/telemetry/gnc_packets.md).
import { USER_LENGTH } from '../js/gnc/gncPackets.js';

// Build a ground frame. user is a Uint8Array of the packet user data.
export function packFrame({ runId = 1, groundTime = 0, apid, seq = 0, time = 0, user, magic = 'STLM', version = 1, ccsdsVersion = 0 }) {
  const packetLength = 12 + user.length;
  const buf = new ArrayBuffer(24 + packetLength);
  const le = new DataView(buf);
  for (let i = 0; i < 4; i++) le.setUint8(i, magic.charCodeAt(i));
  le.setUint16(4, version, true);
  le.setUint16(6, 1, true);
  le.setUint32(8, runId, true);
  le.setUint16(12, packetLength, true);
  le.setFloat64(16, groundTime, true);
  const be = new DataView(buf, 24);
  be.setUint16(0, (ccsdsVersion << 13) | (1 << 11) | apid, false);
  be.setUint16(2, (3 << 14) | (seq & 0x3fff), false);
  be.setUint16(4, user.length + 6 - 1, false);
  const coarse = Math.floor(time);
  be.setUint32(6, coarse, false);
  be.setUint16(10, Math.round((time - coarse) * 65536), false);
  new Uint8Array(buf, 24 + 12).set(user);
  return buf;
}

function writer(len) {
  const buf = new ArrayBuffer(len);
  const dv = new DataView(buf);
  return { buf, dv, u8: (o, v) => dv.setUint8(o, v), u16: (o, v) => dv.setUint16(o, v, false), u32: (o, v) => dv.setUint32(o, v, false),
    f32: (o, v) => dv.setFloat32(o, v, false), f64: (o, v) => dv.setFloat64(o, v, false) };
}

export function packFsw(o) {
  const w = writer(USER_LENGTH[0x100]);
  w.u8(0, o.mode); w.u8(1, o.flags); w.u16(2, o.fixCount); w.u32(4, o.cycles);
  w.f32(8, o.timeInMode); w.f32(12, o.prop); w.f32(16, o.mass);
  return new Uint8Array(w.buf);
}
export function packConfig(o) {
  const w = writer(USER_LENGTH[0x101]);
  w.f64(0, o.lat); w.f64(8, o.lon); w.f64(16, o.h); w.f32(24, o.targetAlt); w.f32(28, o.tol); w.f32(32, o.translateAlt);
  return new Uint8Array(w.buf);
}
export function packNav(o) {
  const w = writer(USER_LENGTH[0x110]);
  o.q.forEach((v, i) => w.f32(4 * i, v));
  o.rate.forEach((v, i) => w.f32(16 + 4 * i, v));
  o.force.forEach((v, i) => w.f32(28 + 4 * i, v));
  w.f32(40, o.alt); w.f32(44, o.vz); w.f32(48, o.range); w.u8(52, o.rangeValid ? 1 : 0);
  w.f32(56, o.east); w.f32(60, o.north); w.f32(64, o.ve); w.f32(68, o.vn); w.f32(72, o.be); w.f32(76, o.bn);
  // Filter health (optional in test fixtures; ages default to -1 = no accepted update yet).
  const h = o.health || {};
  w.f32(80, h.psE || 0); w.f32(84, h.psN || 0); w.f32(88, h.vsE || 0); w.f32(92, h.vsN || 0);
  w.f32(96, h.giE || 0); w.f32(100, h.giN || 0); w.f32(104, h.ai || 0);
  w.f32(108, h.gAge === undefined ? -1 : h.gAge); w.f32(112, h.aAge === undefined ? -1 : h.aAge);
  return new Uint8Array(w.buf);
}
export function packGnc(o) {
  const w = writer(USER_LENGTH[0x120]);
  w.f32(0, o.altSp); w.f32(4, o.vSp); w.f32(8, o.eSp); w.f32(12, o.nSp); w.f32(16, o.aE); w.f32(20, o.aN);
  o.attErr.forEach((v, i) => w.f32(24 + 4 * i, v));
  o.engines.forEach((e, i) => { const b = 36 + 16 * i; w.u8(b, e.on ? 1 : 0); w.f32(b + 4, e.thr); w.f32(b + 8, e.pitch); w.f32(b + 12, e.yaw); });
  return new Uint8Array(w.buf);
}
export function packFeedback(o) {
  const w = writer(USER_LENGTH[0x130]);
  o.engines.forEach((e, i) => { const b = 12 * i; w.u8(b, e.valid ? 1 : 0); w.f32(b + 4, e.pitch); w.f32(b + 8, e.yaw); });
  return new Uint8Array(w.buf);
}
// engine_status (0x150): six engine_power records {on, state, valid, command, throttle} (12 bytes each).
export function packEngineStatus(o) {
  const w = writer(USER_LENGTH[0x150]);
  o.engines.forEach((e, i) => {
    const b = 12 * i;
    w.u8(b, e.on ? 1 : 0); w.u8(b + 1, e.state); w.u8(b + 2, e.valid ? 1 : 0); w.f32(b + 4, e.command); w.f32(b + 8, e.thr);
  });
  return new Uint8Array(w.buf);
}
export function packGnss(o) {
  const w = writer(USER_LENGTH[0x140]);
  w.u8(0, o.fix); w.u8(1, o.used); w.u8(2, o.visible);
  w.f64(4, o.fixTime); w.f64(12, o.lat); w.f64(20, o.lon); w.f64(28, o.h);
  o.vel.forEach((v, i) => w.f32(36 + 4 * i, v));
  w.f32(48, o.hSigma); w.f32(52, o.vSigma);
  return new Uint8Array(w.buf);
}
// catch_status (0x160): u8 phase, abortReason, armsExpected, flags; then nine f32.
export function packCatchStatus(o) {
  const w = writer(USER_LENGTH[0x160]);
  w.u8(0, o.phase); w.u8(1, o.abort); w.u8(2, o.arms); w.u8(3, o.flags);
  [o.along, o.lateral, o.pinHeight, o.alongSpeed, o.lateralSpeed, o.sink, o.tilt, o.limit, o.ttSeat].forEach((v, i) => w.f32(4 + 4 * i, v));
  return new Uint8Array(w.buf);
}
// catch_config (0x161): u8 landingMode, 3 pad, then 19 f32 (seat 3, dir 2, staging, tower 2, mast 2, hinge 2, arm 3, window 4).
export function packCatchConfig(o) {
  const w = writer(USER_LENGTH[0x161]);
  w.u8(0, o.landingMode);
  [...o.seat, ...o.dir, o.staging, ...o.tower, o.mastHalf, o.mastHeight, o.hingeHalf, o.hingeSetback, o.armLen, o.armOpen, o.armClosed,
    o.winLat, o.winAlong, o.winSink, o.winTilt].forEach((v, i) => w.f32(4 + 4 * i, v));
  return new Uint8Array(w.buf);
}
