// Snapshot interpolation with an adaptive jitter buffer. Pure module: no DOM, no three.
//
// Rendering runs a little behind the newest snapshot so there is always a pair to
// interpolate between. How far behind is adaptive: on localhost or a wire, delivery
// is even and the delay stays at its floor (delaySnapshots, 25 ms by default); on
// Wi-Fi, snapshots arrive in bursts after stalls of 30 to 100 ms, and a fixed 25 ms
// buffer runs dry, so the view froze and then jumped forward several times a second.
// The buffer now sizes itself to the measured arrival jitter, changes smoothly (as a
// small playback speed change, never a jump), and as a last resort extrapolates
// briefly along the last velocity and rate, blending back when data resumes.

// Clock offset (wall - sim/timeScale for the fastest arrivals): max change per push.
// 0.3 ms per push at 100 Hz = 3% playback speed, invisible.
const MAX_OFFSET_STEP = 0.0003; // [s]
const JITTER_PERCENTILE = 0.95;
const JITTER_MARGIN = 0.015; // [s] added to the measured jitter
const MAX_DELAY = 0.25; // [s]
const DELAY_RISE_RATE = 0.15; // [s of delay per s of wall time] slow down up to 15% to grow the buffer
const DELAY_FALL_RATE = 0.02; // [s/s] speed up at most 2% to shrink it
const MAX_EXTRAPOLATION = 0.1; // [s] beyond the newest snapshot
const BLEND_TIME_CONSTANT = 0.05; // [s] decay of the correction after extrapolating

export function createInterpolator(opts = {}) {
  let snapshotRateHz = opts.snapshotRateHz ?? 100;
  const delaySnapshots = opts.delaySnapshots ?? 2.5;
  const bufferSeconds = opts.bufferSeconds ?? 2;
  const offsetWindowSeconds = opts.offsetWindowSeconds ?? 3;
  const adaptive = opts.adaptive ?? true;

  let runId = null;
  let timeScale = 1;
  let buf = [];
  let halted = false;
  let dSamples = []; // {wall, d}
  let offset = 0;
  let haveOffset = false;
  let delay = null; // [s] applied render delay (sim time)
  let targetDelay = 0;
  let jitter = 0; // [s] measured
  let lastSampleWall = null;
  // Correction applied after extrapolation, decaying to zero.
  const corrPos = [0, 0, 0];
  const corrQuat = [1, 0, 0, 0];
  let corrWeight = 0;
  let wasExtrapolating = false;
  let extrapBase = null; // snapshot the last extrapolation started from
  const extPos = [0, 0, 0];
  const extQuat = [1, 0, 0, 0];
  const stats = { starvedFrames: 0, extrapolatedFrames: 0, heldFrames: 0 };

  // Reused output (avoid per-frame allocation).
  const out = {
    ready: true, pos: [0, 0, 0], vel: [0, 0, 0], quat: [1, 0, 0, 0],
    engineThrottle: [0, 0, 0, 0, 0, 0], engineGimbalPitch: [0, 0, 0], engineGimbalYaw: [0, 0, 0],
    engineRun: [false, false, false, false, false, false],
    throttle: 0, gimbalPitch: 0, gimbalYaw: 0, propellantMass: 0, totalMass: 0,
    armAngle: [0, 0], towerState: 0, catchResult: 0, caught: false, crashed: false,
    flags: 0, engineRunning: false, touchdown: false, halted: false, simTime: 0, runId: 0,
  };

  function floorDelay() { return delaySnapshots / snapshotRateHz; }

  function reset(id) {
    runId = id;
    buf = [];
    halted = false;
    dSamples = [];
    haveOffset = false;
    offset = 0;
    delay = null;
    jitter = 0;
    lastSampleWall = null;
    corrWeight = 0;
    wasExtrapolating = false;
    extrapBase = null;
  }

  function updateJitter() {
    if (!adaptive || dSamples.length < 10) return;
    let min = Infinity;
    for (const x of dSamples) if (x.d < min) min = x.d;
    const late = dSamples.map((x) => x.d - min).sort((a, b) => a - b);
    jitter = late[Math.min(late.length - 1, Math.floor(JITTER_PERCENTILE * late.length))];
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
    if (dSamples.length % 10 === 0) updateJitter();
    let target = Infinity;
    for (const x of dSamples) if (x.d < target) target = x.d;
    if (!haveOffset) { offset = target; haveOffset = true; return; }
    const diff = target - offset;
    offset += Math.max(-MAX_OFFSET_STEP, Math.min(MAX_OFFSET_STEP, diff));
  }

  function lerpFields(a, b, u) {
    const l = (x, y) => x + (y - x) * u;
    for (let i = 0; i < 3; i++) {
      out.pos[i] = l(a.pos[i], b.pos[i]);
      out.vel[i] = l(a.vel[i], b.vel[i]);
    }
    for (let i = 0; i < 6; i++) out.engineThrottle[i] = l(a.engineThrottle[i], b.engineThrottle[i]);
    for (let i = 0; i < 3; i++) {
      out.engineGimbalPitch[i] = l(a.engineGimbalPitch[i], b.engineGimbalPitch[i]);
      out.engineGimbalYaw[i] = l(a.engineGimbalYaw[i], b.engineGimbalYaw[i]);
    }
    // Arm angles are linear; a sample without them (older tests) counts as open. Held when starved
    // because extrapolate() feeds lerpFields(s, s, 0).
    for (let i = 0; i < 2; i++) {
      const x = a.armAngle ? a.armAngle[i] : 0, y = b.armAngle ? b.armAngle[i] : 0;
      out.armAngle[i] = l(x, y);
    }
    out.throttle = l(a.throttle, b.throttle);
    out.gimbalPitch = l(a.gimbalPitch, b.gimbalPitch);
    out.gimbalYaw = l(a.gimbalYaw, b.gimbalYaw);
    out.propellantMass = l(a.propellantMass, b.propellantMass);
    out.totalMass = l(a.totalMass, b.totalMass);
    out.simTime = l(a.simTime, b.simTime);
    slerp(a.quat, b.quat, u, out.quat);
    const d = u < 0.5 ? a : b;
    for (let i = 0; i < 6; i++) out.engineRun[i] = d.engineRun[i];
    out.flags = d.flags; out.engineRunning = d.engineRunning;
    out.towerState = d.towerState || 0; out.catchResult = d.catchResult || 0;
    out.caught = !!d.caught; out.crashed = !!d.crashed;
    out.touchdown = d.touchdown; out.halted = d.halted; out.runId = d.runId;
  }

  // Dead reckoning past snapshot s: position along the velocity, attitude along the
  // body rate (q_body_to_world advanced by exp(w dt / 2) on the right).
  function deadReckon(s, dt, pos, quat) {
    for (let i = 0; i < 3; i++) pos[i] = s.pos[i] + s.vel[i] * dt;
    for (let i = 0; i < 4; i++) quat[i] = s.quat[i];
    const w = s.angRate || [0, 0, 0];
    const mag = Math.hypot(w[0], w[1], w[2]);
    if (mag > 1e-12) {
      const half = 0.5 * mag * dt, k = Math.sin(half) / mag;
      quatMul(s.quat, [Math.cos(half), w[0] * k, w[1] * k, w[2] * k], quat);
      normalize(quat);
    }
  }

  function extrapolate(s, dt) {
    lerpFields(s, s, 0);
    deadReckon(s, dt, out.pos, out.quat);
    out.simTime = s.simTime + dt;
    extrapBase = s;
  }

  function applyDelayRate(wall) {
    const floor = floorDelay();
    // Perfectly even delivery (no measurable jitter) needs no margin beyond the floor.
    const wanted = adaptive && jitter > 0.001 ? jitter + JITTER_MARGIN : floor;
    targetDelay = Math.min(MAX_DELAY, Math.max(floor, wanted));
    if (delay === null) { delay = targetDelay; lastSampleWall = wall; return 0; }
    const dtWall = Math.max(0, Math.min(0.25, wall - lastSampleWall));
    lastSampleWall = wall;
    if (targetDelay > delay) delay = Math.min(targetDelay, delay + DELAY_RISE_RATE * dtWall);
    else delay = Math.max(targetDelay, delay - DELAY_FALL_RATE * dtWall);
    return dtWall;
  }

  function sample(wall) {
    if (!buf.length) return null;
    if (buf.length === 1) {
      const s = buf[buf.length - 1];
      lerpFields(s, s, 0);
      return out;
    }
    const frameDt = applyDelayRate(wall);
    const head = buf[0], tail = buf[buf.length - 1];
    const tRender = (wall - offset) * timeScale - delay;
    let extrapolating = false;
    if (tRender > tail.simTime && !tail.halted && !halted) {
      stats.starvedFrames++;
      const ahead = Math.min(tRender - tail.simTime, MAX_EXTRAPOLATION);
      if (tRender - tail.simTime > MAX_EXTRAPOLATION) stats.heldFrames++;
      else stats.extrapolatedFrames++;
      extrapolate(tail, ahead);
      extrapolating = true;
    } else {
      const tr = Math.max(head.simTime, Math.min(tail.simTime, tRender));
      let i = buf.length - 1;
      while (i > 1 && buf[i - 1].simTime > tr) i--;
      const a = buf[i - 1], b = buf[i];
      const span = b.simTime - a.simTime;
      const u = span > 0 ? Math.max(0, Math.min(1, (tr - a.simTime) / span)) : 0;
      lerpFields(a, b, u);
    }
    // Leaving extrapolation: the dead-reckoned pose at this render time may differ
    // from the data that just arrived; start from the dead-reckoned pose and decay
    // into the data instead of snapping.
    if (wasExtrapolating && !extrapolating && extrapBase) {
      const ahead = Math.max(0, Math.min(MAX_EXTRAPOLATION, out.simTime - extrapBase.simTime));
      deadReckon(extrapBase, ahead, extPos, extQuat);
      for (let i = 0; i < 3; i++) corrPos[i] = extPos[i] - out.pos[i];
      quatMul(extQuat, conj(out.quat), corrQuat);
      corrWeight = 1;
    }
    wasExtrapolating = extrapolating;
    if (corrWeight > 1e-3) {
      for (let i = 0; i < 3; i++) out.pos[i] += corrPos[i] * corrWeight;
      const c = [1, 0, 0, 0];
      slerp([1, 0, 0, 0], corrQuat, corrWeight, c);
      quatMul(c, out.quat.slice(), out.quat);
      normalize(out.quat);
      corrWeight *= Math.exp(-frameDt / BLEND_TIME_CONSTANT);
    } else {
      corrWeight = 0;
    }
    return out;
  }

  return {
    reset,
    setTimeScale(ts) { timeScale = ts > 0 && Number.isFinite(ts) ? ts : 1; },
    setSnapshotRate(hz) { if (hz > 0 && Number.isFinite(hz)) snapshotRateHz = hz; },
    push, sample,
    // Link health for the ?debug=1 readout and tests.
    stats() {
      return { delayMs: (delay ?? floorDelay()) * 1000, targetDelayMs: targetDelay * 1000, jitterMs: jitter * 1000, ...stats };
    },
  };
}

function quatMul(a, b, o) {
  const w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  const x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  const y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  const z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
  o[0] = w; o[1] = x; o[2] = y; o[3] = z;
  return o;
}

function conj(q) { return [q[0], -q[1], -q[2], -q[3]]; }

function normalize(q) {
  const n = 1 / Math.hypot(q[0], q[1], q[2], q[3]);
  for (let i = 0; i < 4; i++) q[i] *= n;
}

function slerp(qa, qb, u, o) {
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
  for (let i = 0; i < 4; i++) o[i] = wa * qa[i] + wb * qb[i];
  normalize(o);
  return o;
}
