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
export function packGnss(o) {
  const w = writer(USER_LENGTH[0x140]);
  w.u8(0, o.fix); w.u8(1, o.used); w.u8(2, o.visible);
  w.f64(4, o.fixTime); w.f64(12, o.lat); w.f64(20, o.lon); w.f64(28, o.h);
  o.vel.forEach((v, i) => w.f32(36 + 4 * i, v));
  w.f32(48, o.hSigma); w.f32(52, o.vSigma);
  return new Uint8Array(w.buf);
}
