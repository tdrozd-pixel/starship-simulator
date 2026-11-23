// Per-parameter ring buffers and latest values for downlinked GNC telemetry. Pure module.
// x is onboard time [s]. Rings are allocated up front; ingest allocates only the decoded packet.
import { APID, decodePayload, rotationVectorDeg, RAD2DEG, NUM_ENGINES } from './gncPackets.js';

export const DEFAULT_CAPACITY = 6000;

// Chart parameters per APID, in column order, named by dictionary path ("navigation.altitude",
// "guidance_control.engines[0].pitch"; see paramFormat.js). Columns are stored in display units
// (angles in deg, throttle in %); "derived.*" columns are computed from several fields.
const tmp3 = [0, 0, 0];
const ENG = (pk, f) => [0, 1, 2].map((i) => pk + '.engines[' + i + '].' + f);
const GROUPS = {
  [APID.FSW]: {
    names: ['fsw_status.timeInMode', 'fsw_status.propellant', 'fsw_status.mass'],
    fill: (d, r) => { r[0] = d.timeInMode; r[1] = d.propellant; r[2] = d.mass; },
  },
  [APID.NAV]: {
    names: ['derived.attX', 'derived.attY', 'derived.attZ',
      'navigation.angularRate[0]', 'navigation.angularRate[1]', 'navigation.angularRate[2]',
      'navigation.altitude', 'navigation.verticalVelocity', 'navigation.east', 'navigation.north',
      'navigation.velocityEast', 'navigation.velocityNorth'],
    fill: (d, r) => {
      rotationVectorDeg(d.attitude, tmp3);
      r[0] = tmp3[0]; r[1] = tmp3[1]; r[2] = tmp3[2];
      r[3] = d.angularRate[0] * RAD2DEG; r[4] = d.angularRate[1] * RAD2DEG; r[5] = d.angularRate[2] * RAD2DEG;
      r[6] = d.altitude; r[7] = d.verticalVelocity; r[8] = d.east; r[9] = d.north;
      r[10] = d.velocityEast; r[11] = d.velocityNorth;
    },
  },
  [APID.GNC]: {
    names: ['guidance_control.altitudeSetpoint', 'guidance_control.velocitySetpoint',
      'guidance_control.eastSetpoint', 'guidance_control.northSetpoint',
      'guidance_control.eastAccelCommand', 'guidance_control.northAccelCommand',
      ...ENG('guidance_control', 'throttle'), ...ENG('guidance_control', 'pitch'), ...ENG('guidance_control', 'yaw')],
    fill: (d, r) => {
      r[0] = d.altitudeSetpoint; r[1] = d.velocitySetpoint; r[2] = d.eastSetpoint; r[3] = d.northSetpoint;
      r[4] = d.eastAccelCommand; r[5] = d.northAccelCommand;
      for (let i = 0; i < NUM_ENGINES; i++) {
        const e = d.engines[i];
        r[6 + i] = e.throttle * 100; r[9 + i] = e.pitch * RAD2DEG; r[12 + i] = e.yaw * RAD2DEG;
      }
    },
  },
  [APID.FEEDBACK]: {
    names: [...ENG('actuator_feedback', 'pitch'), ...ENG('actuator_feedback', 'yaw')],
    fill: (d, r) => {
      for (let i = 0; i < NUM_ENGINES; i++) {
        r[i] = d.engines[i].pitch * RAD2DEG; r[3 + i] = d.engines[i].yaw * RAD2DEG;
      }
    },
  },
  [APID.GNSS]: {
    names: ['gnss.latitude', 'gnss.longitude', 'gnss.height', 'gnss.horizontalSigma'],
    fill: (d, r) => {
      r[0] = d.latitude * RAD2DEG; r[1] = d.longitude * RAD2DEG; r[2] = d.height; r[3] = d.horizontalSigma;
    },
  },
};

export const PARAM_APID = {};
for (const [apid, g] of Object.entries(GROUPS)) for (const n of g.names) PARAM_APID[n] = Number(apid);

class Group {
  constructor(spec, capacity) {
    this.spec = spec;
    this.capacity = capacity;
    this.t = new Float64Array(capacity);
    this.cols = spec.names.map(() => new Float64Array(capacity));
    this.row = new Float64Array(spec.names.length);
    this.clear();
  }
  clear() { this.head = 0; this.count = 0; }
  push(t, decoded) {
    this.spec.fill(decoded, this.row);
    const h = this.head;
    this.t[h] = t;
    for (let c = 0; c < this.cols.length; c++) this.cols[c][h] = this.row[c];
    this.head = (h + 1) % this.capacity;
    if (this.count < this.capacity) this.count++;
  }
  // Samples with t >= tMin as [x, y...] for the requested names: Float64Array views into per-group
  // scratch buffers (no per-call array allocation beyond the small view objects). Valid until the
  // next window() call on this group; consumers must copy or consume them immediately.
  window(tMin, names) {
    const first = (this.head - this.count + this.capacity) % this.capacity;
    let lo = 0, hi = this.count;
    while (lo < hi) { // samples are time ordered: binary search the first t >= tMin
      const mid = (lo + hi) >> 1;
      if (this.t[(first + mid) % this.capacity] < tMin) lo = mid + 1; else hi = mid;
    }
    const n = this.count - lo;
    const start = (first + lo) % this.capacity;
    if (!this.scratchT) {
      this.scratchT = new Float64Array(this.capacity);
      this.scratch = this.cols.map(() => null);
    }
    copyRing(this.t, this.scratchT, start, n, this.capacity);
    const out = [this.scratchT.subarray(0, n)];
    for (const nm of names) {
      const c = this.spec.names.indexOf(nm);
      if (!this.scratch[c]) this.scratch[c] = new Float64Array(this.capacity);
      copyRing(this.cols[c], this.scratch[c], start, n, this.capacity);
      out.push(this.scratch[c].subarray(0, n));
    }
    return out;
  }
}

function copyRing(src, dst, start, n, cap) {
  if (start + n <= cap) { dst.set(src.subarray(start, start + n)); return; }
  const a = cap - start;
  dst.set(src.subarray(start, cap));
  dst.set(src.subarray(0, n - a), a);
}

export function createTelemetryStore(capacity = DEFAULT_CAPACITY) {
  const groups = {};
  for (const [apid, spec] of Object.entries(GROUPS)) groups[apid] = new Group(spec, capacity);
  const s = {
    runId: null,
    latest: {}, // apid -> decoded object
    latestTime: {}, // apid -> onboard time [s]
    lastWall: {}, // apid -> wall-clock seconds of the last accepted packet (staleness only)
    counts: {}, // apid -> {received, lost}
    lastSeq: {},
    met: 0, // newest onboard time seen [s]
    latency: NaN, // latest groundTime - onboard time [s]
    totalReceived: 0,
    badPackets: 0,
    version: 0, // bumps on every accepted packet and reset: cheap "data changed" check for redraws
  };

  function reset() {
    s.latest = {}; s.latestTime = {}; s.lastWall = {}; s.counts = {}; s.lastSeq = {};
    s.met = 0; s.latency = NaN; s.totalReceived = 0; s.badPackets = 0; s.version++;
    for (const g of Object.values(groups)) g.clear();
  }

  // Feed a decoded ground frame ({kind:'packet', ...}). Returns true if accepted as a new packet.
  s.ingest = (pkt, wallNow = 0) => {
    if (s.runId !== pkt.runId) { reset(); s.runId = pkt.runId; }
    const apid = pkt.apid;
    const last = s.lastSeq[apid];
    let gap = 0;
    if (last !== undefined) {
      if (pkt.seq === last) return false; // replayed duplicate
      gap = (pkt.seq - ((last + 1) & 0x3fff)) & 0x3fff;
      if (gap > 0x2000) return false; // older than the newest: out of order, ignore
    }
    const d = decodePayload(apid, pkt.payload);
    if (!d) { s.badPackets++; return false; }
    const c = s.counts[apid] || (s.counts[apid] = { received: 0, lost: 0 });
    c.lost += gap;
    c.received++;
    s.lastSeq[apid] = pkt.seq;
    s.totalReceived++;
    s.version++;
    s.latest[apid] = d;
    s.latestTime[apid] = pkt.time;
    s.lastWall[apid] = wallNow;
    if (pkt.time > s.met) s.met = pkt.time;
    s.latency = pkt.groundTime - pkt.time;
    const g = groups[apid];
    if (g) g.push(pkt.time, d);
    return true;
  };

  s.reset = reset;
  s.sampleCount = (apid) => (groups[apid] ? groups[apid].count : 0);
  // Series for charts: all names must belong to one APID. Returns [x, y...].
  s.series = (names, tMin) => groups[PARAM_APID[names[0]]].window(tMin, names);
  s.lossPercent = () => {
    let r = 0, l = 0;
    for (const c of Object.values(s.counts)) { r += c.received; l += c.lost; }
    return r + l > 0 ? 100 * l / (r + l) : 0;
  };
  return s;
}
