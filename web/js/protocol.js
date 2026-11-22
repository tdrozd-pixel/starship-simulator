// Truth stream wire format, version 1 (docs/protocol/truth_stream.md). Pure module.

export const MAGIC = 'SSTR';
export const VERSION = 1;
export const TYPE_SNAPSHOT = 1;
export const SNAPSHOT_SIZE = 176;
export const FLAG_ENGINE = 1;
export const FLAG_TOUCHDOWN = 2;
export const FLAG_HALTED = 4;

export function decodeBinary(buf) {
  if (!buf || buf.byteLength < 8) return { kind: 'error', reason: 'message too short' };
  const dv = new DataView(buf);
  for (let i = 0; i < 4; i++) {
    if (dv.getUint8(i) !== MAGIC.charCodeAt(i)) return { kind: 'error', reason: 'bad magic' };
  }
  const version = dv.getUint16(4, true);
  if (version !== VERSION) return { kind: 'error', reason: 'protocol version ' + version + ' unsupported' };
  if (dv.getUint16(6, true) !== TYPE_SNAPSHOT) return { kind: 'unknown' };
  if (buf.byteLength !== SNAPSHOT_SIZE) {
    return { kind: 'error', reason: 'bad snapshot length ' + buf.byteLength };
  }
  const f = (o) => dv.getFloat64(o, true);
  const flags = dv.getUint32(12, true);
  return {
    kind: 'snapshot',
    snap: {
      runId: dv.getUint32(8, true),
      flags,
      engineRunning: (flags & FLAG_ENGINE) !== 0,
      touchdown: (flags & FLAG_TOUCHDOWN) !== 0,
      halted: (flags & FLAG_HALTED) !== 0,
      step: dv.getBigUint64(16, true),
      simTime: f(24),
      pos: [f(32), f(40), f(48)],
      vel: [f(56), f(64), f(72)],
      quat: [f(80), f(88), f(96), f(104)], // w, x, y, z
      angRate: [f(112), f(120), f(128)],
      throttle: f(136),
      gimbalPitch: f(144),
      gimbalYaw: f(152),
      propellantMass: f(160),
      totalMass: f(168),
    },
  };
}

export function parseControl(text) {
  try {
    const o = JSON.parse(text);
    return o !== null && typeof o === 'object' ? o : null;
  } catch (e) {
    return null;
  }
}

export function makeRestartMessage() {
  return '{"type":"restart"}';
}
