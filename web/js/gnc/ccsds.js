// Ground frame (docs/protocol/telemetry_stream.md) and CCSDS Space Packet (docs/telemetry/gnc_packets.md)
// decoding. Pure module: no imports, no DOM. Ground header is little-endian, CCSDS is big-endian.

export const GROUND_MAGIC = 'STLM';
export const GROUND_VERSION = 1;
export const GROUND_TYPE_PACKET = 1;
export const GROUND_HEADER_SIZE = 24;
export const PRIMARY_HEADER_SIZE = 6;
export const SECONDARY_HEADER_SIZE = 6;
export const USER_DATA_OFFSET = PRIMARY_HEADER_SIZE + SECONDARY_HEADER_SIZE;

// CUC time code: u32 whole seconds, u16 fraction in 1/65536 s. Returns seconds.
export function decodeCuc(dv, offset) {
  return dv.getUint32(offset, false) + dv.getUint16(offset + 4, false) / 65536;
}

// Decode the CCSDS packet occupying buf[offset .. offset + length). Returns
// {kind:'packet', apid, seq, time, payload} or {kind:'error', reason}.
export function decodeCcsds(buf, offset, length) {
  if (length < USER_DATA_OFFSET) return { kind: 'error', reason: 'packet too short' };
  const dv = new DataView(buf, offset, length);
  const w0 = dv.getUint16(0, false);
  const w1 = dv.getUint16(2, false);
  const dataLength = dv.getUint16(4, false) + 1;
  const version = w0 >> 13;
  const type = (w0 >> 12) & 1;
  const secHdr = (w0 >> 11) & 1;
  const apid = w0 & 0x7ff;
  const seqFlags = w1 >> 14;
  const seq = w1 & 0x3fff;
  if (version !== 0) return { kind: 'error', reason: 'bad CCSDS version ' + version };
  if (type !== 0) return { kind: 'error', reason: 'not a telemetry packet' };
  if (!secHdr) return { kind: 'error', reason: 'missing secondary header' };
  if (dataLength + PRIMARY_HEADER_SIZE !== length) {
    return { kind: 'error', reason: 'bad data length ' + dataLength + ' for packet of ' + length };
  }
  return {
    kind: 'packet', apid, seq, seqFlags, time: decodeCuc(dv, PRIMARY_HEADER_SIZE),
    payload: new DataView(buf, offset + USER_DATA_OFFSET, length - USER_DATA_OFFSET),
  };
}

// Decode a whole ground frame. Returns {kind:'packet', runId, groundTime, apid, seq, time, payload}
// or {kind:'error', reason}. An error whose reason mentions "version" is fatal to the client.
export function decodeGroundFrame(buf) {
  if (!buf || buf.byteLength < GROUND_HEADER_SIZE) return { kind: 'error', reason: 'frame too short' };
  const dv = new DataView(buf);
  for (let i = 0; i < 4; i++) {
    if (dv.getUint8(i) !== GROUND_MAGIC.charCodeAt(i)) return { kind: 'error', reason: 'bad magic' };
  }
  const version = dv.getUint16(4, true);
  if (version !== GROUND_VERSION) return { kind: 'error', reason: 'protocol version ' + version + ' unsupported' };
  if (dv.getUint16(6, true) !== GROUND_TYPE_PACKET) return { kind: 'unknown' };
  const packetLength = dv.getUint16(12, true);
  if (buf.byteLength !== GROUND_HEADER_SIZE + packetLength) {
    return { kind: 'error', reason: 'bad frame length ' + buf.byteLength + ' for packet length ' + packetLength };
  }
  const p = decodeCcsds(buf, GROUND_HEADER_SIZE, packetLength);
  if (p.kind !== 'packet') return p;
  p.runId = dv.getUint32(8, true);
  p.groundTime = dv.getFloat64(16, true);
  return p;
}
