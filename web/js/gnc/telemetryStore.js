// Per-parameter ring buffers and latest values for downlinked GNC telemetry. Pure module.
// x is onboard time [s]. Rings are allocated up front; ingest allocates only the decoded packet.
import { APID, decodePayload, rotationVectorDeg, RAD2DEG, NUM_ENGINES } from './gncPackets.js';

export const DEFAULT_CAPACITY = 6000;

// Parameter names per APID, in column order, and the extractor that fills a row.
const tmp3 = [0, 0, 0];
const GROUPS = {
  [APID.FSW]: {
    names: ['timeInMode', 'propellant', 'mass'],
    fill: (d, r) => { r[0] = d.timeInMode; r[1] = d.propellant; r[2] = d.mass; },
  },
  [APID.NAV]: {
    names: ['attX', 'attY', 'attZ', 'rateX', 'rateY', 'rateZ', 'alt', 'vz', 'east', 'north'],
    fill: (d, r) => {
      rotationVectorDeg(d.attitude, tmp3);
      r[0] = tmp3[0]; r[1] = tmp3[1]; r[2] = tmp3[2];
      r[3] = d.angularRate[0] * RAD2DEG; r[4] = d.angularRate[1] * RAD2DEG; r[5] = d.angularRate[2] * RAD2DEG;
      r[6] = d.altitude; r[7] = d.verticalVelocity; r[8] = d.east; r[9] = d.north;
    },
  },
  [APID.GNC]: {
    names: ['altSp', 'vSp', 'eSp', 'nSp', 'aE', 'aN',
      'thr0', 'thr1', 'thr2', 'cgp0', 'cgp1', 'cgp2', 'cgy0', 'cgy1', 'cgy2'],
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
    names: ['mgp0', 'mgp1', 'mgp2', 'mgy0', 'mgy1', 'mgy2'],
    fill: (d, r) => {
      for (let i = 0; i < NUM_ENGINES; i++) {
        r[i] = d.engines[i].pitch * RAD2DEG; r[3 + i] = d.engines[i].yaw * RAD2DEG;
      }
    },
  },
  [APID.GNSS]: {
    names: ['gLat', 'gLon', 'gH', 'gSigmaH'],
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
  // Copy samples with t >= tMin into plain arrays: returns [x, y...] for the requested names.
  window(tMin, names) {
    const first = (this.head - this.count + this.capacity) % this.capacity;
    let lo = 0, hi = this.count;
    while (lo < hi) { // samples are time ordered: binary search the first t >= tMin
      const mid = (lo + hi) >> 1;
      if (this.t[(first + mid) % this.capacity] < tMin) lo = mid + 1; else hi = mid;
    }
    const n = this.count - lo;
    const x = new Array(n);
    const ys = names.map(() => new Array(n));
    const idx = names.map((nm) => this.spec.names.indexOf(nm));
    for (let i = 0; i < n; i++) {
      const p = (first + lo + i) % this.capacity;
      x[i] = this.t[p];
      for (let k = 0; k < idx.length; k++) ys[k][i] = this.cols[idx[k]][p];
    }
    return [x, ...ys];
  }
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
  };

  function reset() {
    s.latest = {}; s.latestTime = {}; s.lastWall = {}; s.counts = {}; s.lastSeq = {};
    s.met = 0; s.latency = NaN; s.totalReceived = 0; s.badPackets = 0;
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
