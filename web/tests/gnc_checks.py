"""GNC console checks for run_web_tests.py: a Python cross-check of the packet decoders and a
page smoke test driven through window.__gncInject (no server needed)."""
import math
import struct

SHOT = "C:/Users/Taras/AppData/Local/Temp/claude/b512f2bc3954477c/9e5da832-41ed-4e6d-8b57-396e9d227ae1/scratchpad/gnc_shot.png"
TARGET = (math.radians(25.996603), math.radians(-97.158251), -22.0)  # lat [rad], lon [rad], height [m]
E2 = 0.00669437999014

INJECT = """(hex) => { const b = new Uint8Array(hex.match(/../g).map(h => parseInt(h, 16))); window.__gncInject(b.buffer); }"""
INJECT_MANY = """(hexes) => { for (const hex of hexes) {
  const b = new Uint8Array(hex.match(/../g).map(h => parseInt(h, 16)));
  window.__gncInject(b.buffer); } }"""


def ground_frame(run, ground_time, apid, seq, t, user):
    """Ground frame (little-endian header) around a CCSDS packet (big-endian), per the dictionary."""
    coarse = int(t)
    fine = int(round((t - coarse) * 65536))
    ccsds = struct.pack(">HHH", (1 << 11) | apid, (3 << 14) | (seq & 0x3FFF), len(user) + 6 - 1)
    ccsds += struct.pack(">IH", coarse, fine) + user
    return struct.pack("<4sHHIHHd", b"STLM", 1, 1, run, len(ccsds), 0, ground_time) + ccsds


def pack_nav(q, rate, force, alt, vz, rng, valid, east, north, ve, vn, be, bn):
    return struct.pack(">4f3f3f3fB3x6f", *q, *rate, *force, alt, vz, rng, 1 if valid else 0, east, north, ve, vn, be, bn)


def pack_fsw(mode, flags, fixes, cycles, tim, prop, mass):
    return struct.pack(">BBHIfff", mode, flags, fixes, cycles, tim, prop, mass)


def pack_gnc(alt_sp, v_sp, e_sp, n_sp, ae, an, err, engines):
    out = struct.pack(">6f3f", alt_sp, v_sp, e_sp, n_sp, ae, an, *err)
    for on, thr, p, y in engines:
        out += struct.pack(">B3xfff", 1 if on else 0, thr, p, y)
    return out


def pack_fb(engines):
    return b"".join(struct.pack(">B3xff", 1 if v else 0, p, y) for v, p, y in engines)


def pack_gnss(fix, used, vis, fix_time, lat, lon, h, vel, hs, vs):
    return struct.pack(">BBBxdddd3fff", fix, used, vis, fix_time, lat, lon, h, *vel, hs, vs)


def pack_config(lat, lon, h, alt, tol, trans):
    return struct.pack(">dddfff", lat, lon, h, alt, tol, trans)


def cross_check(page, fails):
    """Pack a navigation packet with struct, decode it in the page, compare every field."""
    nav = dict(q=(0.9, 0.1, -0.2, 0.3), rate=(0.01, -0.02, 0.03), force=(0.1, 0.2, 9.8), alt=12.5, vz=-1.25,
               rng=12.75, valid=True, east=3.5, north=-4.25, ve=0.5, vn=-0.5, be=0.01, bn=-0.02)
    user = pack_nav(**nav)
    assert len(user) == 80, len(user)
    got = page.evaluate(
        """async (hex) => {
          const c = await import('/js/gnc/ccsds.js');
          const g = await import('/js/gnc/gncPackets.js');
          const b = new Uint8Array(hex.match(/../g).map(h => parseInt(h, 16)));
          const r = c.decodeGroundFrame(b.buffer);
          return {r: {kind: r.kind, runId: r.runId, apid: r.apid, seq: r.seq, time: r.time, gt: r.groundTime},
                  n: g.decodePayload(r.apid, r.payload)};
        }""",
        ground_frame(9, 101.5, 0x110, 77, 100.25, user).hex(),
    )
    r, n = got["r"], got["n"]
    if (r["kind"], r["runId"], r["apid"], r["seq"], r["time"], r["gt"]) != ("packet", 9, 0x110, 77, 100.25, 101.5):
        fails.append(f"gnc cross-check header: {r}")
    exp = {
        "attitude": nav["q"], "angularRate": nav["rate"], "specificForce": nav["force"],
        "altitude": nav["alt"], "verticalVelocity": nav["vz"], "altimeterRange": nav["rng"],
        "east": nav["east"], "north": nav["north"], "velocityEast": nav["ve"],
        "velocityNorth": nav["vn"], "accelBiasEast": nav["be"], "accelBiasNorth": nav["bn"],
    }
    for k, v in exp.items():
        vs = list(v) if isinstance(v, tuple) else [v]
        gs = n[k] if isinstance(n[k], list) else [n[k]]
        for a, b in zip(gs, vs):
            if not math.isclose(a, struct.unpack(">f", struct.pack(">f", b))[0], rel_tol=0, abs_tol=1e-12):
                fails.append(f"gnc cross-check {k}: js {a} py {b}")
    if n["altimeterValid"] is not True:
        fails.append("gnc cross-check altimeterValid")
    print("gnc cross-check: done")


def build_flight(run=5):
    """Synthetic 30 s hop as ground frames (hex), plus the final truth values to check against."""
    lat0, lon0, h0 = TARGET
    m_rad = 6378137.0 * (1 - E2) / (1 - E2 * math.sin(lat0) ** 2) ** 1.5
    n_rad = 6378137.0 / math.sqrt(1 - E2 * math.sin(lat0) ** 2)
    frames = []
    seq = {a: 0 for a in (0x100, 0x101, 0x110, 0x120, 0x130, 0x140)}

    def add(apid, t, user):
        frames.append(ground_frame(run, t + 0.15, apid, seq[apid], t, user).hex())
        seq[apid] += 1

    mode_edges = [2, 4, 10, 20, 26, 29]
    steps = 1500
    for k in range(steps):
        t = k * 0.02
        mode = sum(1 for e in mode_edges if t >= e)
        alt = 50 * (1 - math.exp(-t / 5))
        east, north = 20 * math.exp(-t / 8), -12 * math.exp(-t / 8)
        ang = 0.06 * math.sin(t)
        q = (math.cos(ang / 2), math.sin(ang / 2), 0.0, 0.0)
        eng_cmd = [(1 <= mode <= 5, 0.5 + 0.1 * i, 0.12 * math.sin(t + i), 0.1 * math.cos(1.3 * t + i)) for i in range(3)]
        eng_fb = [(not (i == 1 and t > 12), 0.12 * math.sin(t - 0.1 + i), 0.1 * math.cos(1.3 * (t - 0.1) + i)) for i in range(3)]
        add(0x110, t, pack_nav(q, (ang * 0.5, 0.0, 0.01), (0.0, 0.0, 9.8), alt, 10 * math.exp(-t / 5) / 2, alt + 0.1, True,
                               east, north, -east / 8, -north / 8, 0.01, -0.02))
        add(0x120, t, pack_gnc(50.0, 0.0, 0.0, 0.0, -east / 20, -north / 20, (0.001, 0.002, 0.0), eng_cmd))
        add(0x130, t, pack_fb(eng_fb))
        if k % 5 == 0:
            add(0x100, t, pack_fsw(mode, 0b10111 if mode in (3, 4) else 0b00011, min(k // 5, 60), k // 2, t % 10,
                                   250000 - 40 * t, 420000 - 40 * t))
            lat = lat0 + (north + 0.4) / m_rad
            lon = lon0 + (east + 0.3) / (n_rad * math.cos(lat0))
            add(0x140, t, pack_gnss(2, 17, 22, t - 0.05, lat, lon, h0 + alt + 0.5, (0.0, 0.0, 0.0), 0.02, 0.04))
        if k % 50 == 0:
            add(0x101, t, pack_config(lat0, lon0, h0, 50.0, 1.5, 30.0))
    t_end = (steps - 1) * 0.02
    return frames, seq, steps, (20 * math.exp(-t_end / 8), -12 * math.exp(-t_end / 8))


def smoke(browser, base, fails):
    ctx = browser.new_context(viewport={"width": 1920, "height": 1080})
    gp = ctx.new_page()
    errors, sockets = [], []
    gp.on("console", lambda m: errors.append("console: " + m.text) if m.type == "error" else None)
    gp.on("pageerror", lambda e: errors.append("pageerror: " + str(e)))
    gp.on("websocket", lambda w: sockets.append(w.url))
    gp.on("response", lambda r: errors.append(f"http {r.status}: {r.url}") if r.status >= 400 else None)
    gp.goto(base + "/gnc/")
    gp.wait_for_function("typeof window.__gncInject === 'function'", timeout=10000)
    frames, seq, steps, (e_end, n_end) = build_flight()
    gp.evaluate(INJECT_MANY, frames)
    gp.wait_for_timeout(700)
    st = gp.evaluate("window.__gncState()")
    if st["mode"] != "LANDED":
        fails.append(f"gnc smoke: mode {st['mode']}")
    if st["fixType"] != "RTK FIXED" or st["satsUsed"] != 17 or st["satsVisible"] != 22:
        fails.append(f"gnc smoke: gnss {st['fixType']} {st['sats']}")
    raw, nv = st["rawDelta"], st["navDelta"]
    if raw is None or abs(raw["east"] - (e_end + 0.3)) > 0.05 or abs(raw["north"] - (n_end + 0.4)) > 0.05:
        fails.append(f"gnc smoke: raw delta {raw}")
    elif abs(raw["dist"] - math.hypot(raw["east"], raw["north"])) > 1e-6:
        fails.append("gnc smoke: raw distance")
    if nv is None or abs(nv["east"] - e_end) > 1e-3 or abs(nv["north"] - n_end) > 1e-3:
        fails.append(f"gnc smoke: nav delta {nv}")
    if len(st["segments"]) != 7:
        fails.append(f"gnc smoke: {len(st['segments'])} mode segments")
    if st["received"]["272"]["received"] != steps or st["received"]["272"]["lost"] != 0:
        fails.append(f"gnc smoke: nav counts {st['received']['272']}")
    if any(v != "ok" for v in st["stale"].values()):
        fails.append(f"gnc smoke: stale flags {st['stale']}")
    if not st["met"].startswith("T+00:29"):
        fails.append(f"gnc smoke: met {st['met']}")
    gp.screenshot(path=SHOT)

    # Loss accounting (two packets missing), then run change, then staleness.
    nav_user = pack_nav((1, 0, 0, 0), (0, 0, 0), (0, 0, 9.8), 1, 0, 1, True, 0, 0, 0, 0, 0, 0)
    gp.evaluate(INJECT, ground_frame(5, 31.0, 0x110, (seq[0x110] + 2) & 0x3FFF, 30.0, nav_user).hex())
    lost = gp.evaluate("window.__gncState().received['272'].lost")
    if lost != 2:
        fails.append(f"gnc smoke: lost {lost}, expected 2")
    gp.evaluate(INJECT, ground_frame(6, 1.0, 0x110, 0, 0.5, nav_user).hex())
    st2 = gp.evaluate("window.__gncState()")
    if st2["runId"] != 6 or st2["mode"] != "--" or st2["segments"]:
        fails.append(f"gnc smoke: run change did not reset ({st2['runId']}, {st2['mode']})")
    gp.wait_for_timeout(1500)
    if gp.evaluate("window.__gncState().stale")["NAV"] != "stale":
        fails.append("gnc smoke: NAV not stale after 1.5 s without packets")
    if any("/ws/truth" in u for u in sockets):
        fails.append(f"gnc smoke: opened the truth stream: {sockets}")
    if not any("/ws/telemetry" in u for u in sockets):
        fails.append(f"gnc smoke: never opened /ws/telemetry: {sockets}")
    fails += ["gnc smoke: " + e for e in errors if "websocket" not in e.lower()]
    print(f"gnc smoke: injected {len(frames)} frames, final mode {st['mode']}, sockets {sorted(set(sockets))}")
    ctx.close()
