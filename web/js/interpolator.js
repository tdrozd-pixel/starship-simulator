// Snapshot interpolation with a fixed render delay and a min-delay wall/sim clock offset.
// Pure module: no DOM, no three.

const MAX_OFFSET_STEP = 0.002; // [s] offset slew per push

export function createInterpolator(opts = {}) {
  let snapshotRateHz = opts.snapshotRateHz ?? 100;
  const delaySnapshots = opts.delaySnapshots ?? 2.5;
  const bufferSeconds = opts.bufferSeconds ?? 2;
  const offsetWindowSeconds = opts.offsetWindowSeconds ?? 3;

  let runId = null;
  let timeScale = 1;
  let buf = [];
  let halted = false;
  let dSamples = []; // {wall, d}
  let offset = 0;
  let haveOffset = false;

  // Reused output (avoid per-frame allocation).
  const out = {
    ready: true, pos: [0, 0, 0], vel: [0, 0, 0], quat: [1, 0, 0, 0],
    throttle: 0, gimbalPitch: 0, gimbalYaw: 0, propellantMass: 0, totalMass: 0,
    flags: 0, engineRunning: false, touchdown: false, halted: false, simTime: 0, runId: 0,
  };

  function reset(id) {
    runId = id;
    buf = [];
    halted = false;
    dSamples = [];
    haveOffset = false;
    offset = 0;
  }

  function push(s, wall) {
    if (s.runId !== runId) reset(s.runId);
    let last = buf.length ? buf[buf.length - 1] : null;
    if (last && s.step <= last.step) return;
    if (last && s.simTime < last.simTime) { reset(s.runId); last = null; }
    buf.push(s);
    while (buf.length > 2 && s.simTime - buf[0].simTime > bufferSeconds) buf.shift();
    if (s.halted) { halted = true; return; }
    halted = false;
    const d = wall - s.simTime / timeScale;
    dSamples.push({ wall, d });
    while (dSamples.length && wall - dSamples[0].wall > offsetWindowSeconds) dSamples.shift();
    let target = Infinity;
    for (const x of dSamples) if (x.d < target) target = x.d;
    if (!haveOffset) { offset = target; haveOffset = true; return; }
    const diff = target - offset;
    offset += Math.max(-MAX_OFFSET_STEP, Math.min(MAX_OFFSET_STEP, diff));
  }

  function fill(a, b, u) {
    const l = (x, y) => x + (y - x) * u;
    for (let i = 0; i < 3; i++) {
      out.pos[i] = l(a.pos[i], b.pos[i]);
      out.vel[i] = l(a.vel[i], b.vel[i]);
    }
    out.throttle = l(a.throttle, b.throttle);
    out.gimbalPitch = l(a.gimbalPitch, b.gimbalPitch);
    out.gimbalYaw = l(a.gimbalYaw, b.gimbalYaw);
    out.propellantMass = l(a.propellantMass, b.propellantMass);
    out.totalMass = l(a.totalMass, b.totalMass);
    out.simTime = l(a.simTime, b.simTime);
    // quaternion slerp
    const qa = a.quat, qb = b.quat;
    let dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
    const sgn = dot < 0 ? -1 : 1;
    dot *= sgn;
    let wa, wb;
    if (dot > 0.9995) { wa = 1 - u; wb = u * sgn; }
    else {
      const th = Math.acos(Math.min(1, dot));
      const sn = Math.sin(th);
      wa = Math.sin((1 - u) * th) / sn;
      wb = (Math.sin(u * th) / sn) * sgn;
    }
    let n = 0;
    for (let i = 0; i < 4; i++) { out.quat[i] = wa * qa[i] + wb * qb[i]; n += out.quat[i] * out.quat[i]; }
    n = 1 / Math.sqrt(n);
    for (let i = 0; i < 4; i++) out.quat[i] *= n;
    const d = u < 0.5 ? a : b;
    out.flags = d.flags; out.engineRunning = d.engineRunning;
    out.touchdown = d.touchdown; out.halted = d.halted; out.runId = d.runId;
  }

  function sample(wall) {
    if (!buf.length) return null;
    if (buf.length === 1) {
      const s = buf[buf.length - 1];
      fill(s, s, 0);
      return out;
    }
    const head = buf[0], tail = buf[buf.length - 1];
    let tr = (wall - offset) * timeScale - delaySnapshots / snapshotRateHz;
    tr = Math.max(head.simTime, Math.min(tail.simTime, tr));
    let i = buf.length - 1;
    while (i > 1 && buf[i - 1].simTime > tr) i--;
    const a = buf[i - 1], b = buf[i];
    const span = b.simTime - a.simTime;
    const u = span > 0 ? Math.max(0, Math.min(1, (tr - a.simTime) / span)) : 0;
    fill(a, b, u);
    return out;
  }

  return {
    reset,
    setTimeScale(ts) { timeScale = ts > 0 && Number.isFinite(ts) ? ts : 1; },
    setSnapshotRate(hz) { if (hz > 0 && Number.isFinite(hz)) snapshotRateHz = hz; },
    push, sample,
  };
}
