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

    mode_edges = [2, 4, 10, 20, 26]
    steps = 1500
    for k in range(steps):
        t = k * 0.02
        mode = [0, 1, 2, 4, 5, 6][sum(1 for e in mode_edges if t >= e)]
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
            add(0x100, t, pack_fsw(mode, 0b10111 if mode == 4 else 0b00011, min(k // 5, 60), k // 2, t % 10,
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
    if len(st["segments"]) != 6:
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


SCREEN_CHARTS = {
    "overview": ["alt", "vz"], "gnc": ["att", "rate", "alt", "vz", "accel"], "actuators": ["thr", "gimbal"],
    "nav": ["pos", "vel"], "fsw": [],
}
SHOWN = "[...document.querySelectorAll('.screen')].map(e => e.dataset.screen)"
STATE = "(() => { const s = window.__gncState(); return {regime: s.regime, screen: s.screen, follow: s.follow, mode: s.mode}; })()"


def extra_frames(seq, t0, n, run=5):
    """n more 50 Hz navigation, guidance and feedback packets continuing the sequence counters."""
    out = []
    for k in range(n):
        t = t0 + k * 0.02
        nav = pack_nav((1, 0, 0, 0), (0, 0, 0), (0, 0, 9.8), 3 + math.sin(t), 0.1, 3, True, 1.0, 2.0, 0, 0, 0, 0)
        gnc = pack_gnc(3.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0, 0, 0), [(True, 0.5, 0.01, 0.01)] * 3)
        fb = pack_fb([(True, 0.01, 0.01)] * 3)
        for apid, user in ((0x110, nav), (0x120, gnc), (0x130, fb)):
            out.append(ground_frame(run, t + 0.15, apid, seq[apid] & 0x3FFF, t, user).hex())
            seq[apid] += 1
    return out


def screens(browser, base, fails):
    """Screen routing, per-screen drawing, follow-phase paging, planned regimes, no three.js."""
    ctx = browser.new_context(viewport={"width": 1920, "height": 1080})
    page = ctx.new_page()
    requests, sockets, errors = [], [], []
    page.on("request", lambda r: requests.append(r.url))
    page.on("websocket", lambda w: sockets.append(w.url))
    page.on("pageerror", lambda e: errors.append("pageerror: " + str(e)))
    page.on("console", lambda m: errors.append("console: " + m.text) if m.type == "error" and "websocket" not in m.text.lower() else None)
    page.goto(base + "/gnc/#/ascent/overview")
    page.wait_for_function("typeof window.__gncInject === 'function'", timeout=10000)
    frames, seq, _, _ = build_flight()
    page.evaluate(INJECT_MANY, frames)
    st = page.evaluate(STATE)
    if st["regime"] != "ascent" or st["follow"] or st["mode"] != "LANDED":
        fails.append(f"gnc screens: a deep link must pin its screen even when the flight moved on: {st}")
    if page.evaluate(SHOWN) != ["overview"]:
        fails.append(f"gnc screens: deep link shows {page.evaluate(SHOWN)}")

    def expect(screen, how):
        shown = page.evaluate(SHOWN)
        if shown != [screen]:
            fails.append(f"gnc screens: {how}: shown {shown}, expected [{screen}]")
            return
        if not page.locator("#attitude-mini").is_visible() or not page.locator("#attitude-mini-canvas").is_visible():
            fails.append(f"gnc screens: pitch/roll indicator missing on {screen}")
        ids = page.evaluate("window.__gncCharts.ids()")
        if ids != SCREEN_CHARTS[screen]:
            fails.append(f"gnc screens: {how}: charts {ids}, expected {SCREEN_CHARTS[screen]}")
        n_plots = page.evaluate("document.querySelectorAll('.uplot').length")
        if n_plots != len(SCREEN_CHARTS[screen]):
            fails.append(f"gnc screens: {how}: {n_plots} uPlot instances in the DOM")

    page.click('#tabs button[data-screen="gnc"]')
    expect("gnc", "click")
    page.keyboard.press("3")
    expect("actuators", "key 3")
    page.keyboard.press("5")
    expect("fsw", "key 5")
    page.evaluate("location.hash = '#/ascent/nav'")
    page.wait_for_function("window.__gncState().screen === 'nav'")
    expect("nav", "hash")
    page.keyboard.press("9")  # no ninth screen: ignored
    expect("nav", "key 9 ignored")
    # Switching builds the screen from history at once and draws its charts.
    t0 = page.evaluate("performance.now()")
    page.keyboard.press("2")
    page.wait_for_function("window.__gncCharts.ids().length === 5")
    build_ms = page.evaluate(f"performance.now() - {t0}")
    if build_ms > 400:  # generous: includes the harness round trips
        fails.append(f"gnc screens: screen switch took {build_ms:.0f} ms")
    if (page.evaluate("window.__gncCharts.draws().byId.att") or 0) < 1:
        fails.append("gnc screens: a new screen must draw its charts from history immediately")

    # Only the visible screen draws: stream more packets while on nav, then on a screen without charts.
    page.keyboard.press("4")  # nav
    expect("nav", "key 4")
    before = page.evaluate("window.__gncCharts.draws()")
    page.evaluate(INJECT_MANY, extra_frames(seq, 40.0, 100))
    page.wait_for_timeout(700)
    after = page.evaluate("window.__gncCharts.draws()")
    grew = {k for k, v in after["byId"].items() if v > before["byId"].get(k, 0)}
    if grew != {"pos", "vel"}:
        fails.append(f"gnc screens: charts drawn while on nav: {sorted(grew)}")
    page.keyboard.press("5")  # fsw: no charts at all
    before = page.evaluate("window.__gncCharts.draws()")["total"]
    page.evaluate(INJECT_MANY, extra_frames(seq, 42.0, 100))
    page.wait_for_timeout(700)
    if page.evaluate("window.__gncCharts.draws()")["total"] != before:
        fails.append("gnc screens: charts drawn on a screen without charts")
    if page.evaluate("window.__gncState().sampleCounts.nav") < 100:
        fails.append("gnc screens: the store must keep collecting while a screen is not shown")

    # Reload without a hash restores the last manual selection.
    page.keyboard.press("2")
    page2 = ctx.new_page()
    page2.on("websocket", lambda w: sockets.append(w.url))
    page2.goto(base + "/gnc/")
    page2.wait_for_function("typeof window.__gncInject === 'function'")
    st2 = page2.evaluate(STATE)
    if (st2["regime"], st2["screen"]) != ("ascent", "gnc"):
        fails.append(f"gnc screens: reload did not restore the last screen: {st2}")
    page2.close()

    # Planned regimes cannot be opened.
    page.evaluate("location.hash = '#/entry/overview'")
    page.wait_for_timeout(300)
    if page.evaluate(STATE)["regime"] != "ascent":
        fails.append("gnc screens: a planned regime was opened through the URL")
    page.click("#regime-btn")
    for rid in ("staging", "entry"):
        if not page.locator(f'.regime-item[data-regime="{rid}"]').is_disabled():
            fails.append(f"gnc screens: planned regime {rid} is selectable")
    if page.locator('.regime-item[data-regime="ascent"]').is_disabled():
        fails.append("gnc screens: live regime ascent is disabled")
    page.keyboard.press("Escape")
    if not page.locator("#regime-pop").is_hidden():
        fails.append("gnc screens: Escape did not close the regime menu")

    # Menu and staleness popover open and close cleanly.
    page.click("#menu-btn")
    if page.locator("#menu").is_hidden():
        fails.append("gnc screens: menu did not open")
    page.click("#stale-btn")  # opens the staleness popover and closes the menu
    if not page.locator("#menu").is_hidden() or page.locator("#stale-pop").is_hidden():
        fails.append("gnc screens: popovers must be exclusive")
    page.mouse.click(600, 600)
    if not page.locator("#stale-pop").is_hidden():
        fails.append("gnc screens: outside click did not close the popover")
    page.click("#menu-btn")
    page.select_option("#win", "30")
    page.select_option("#eng", "all")
    if page.evaluate("window.__gncState().chartWindow") != 30:
        fails.append("gnc screens: chart window selector")
    page.click("#clear-btn")
    if page.evaluate("window.__gncState().sampleCounts.nav") != 0:
        fails.append("gnc screens: clear history")
    ctx.close()

    # Follow phase: walk the hop through its modes.
    ctx = browser.new_context(viewport={"width": 1920, "height": 1080})
    page = ctx.new_page()
    page.on("websocket", lambda w: sockets.append(w.url))
    page.goto(base + "/gnc/")
    page.wait_for_function("typeof window.__gncInject === 'function'")
    st = page.evaluate(STATE)
    if (st["regime"], st["follow"]) != ("prelaunch", True):
        fails.append(f"gnc follow: initial state {st}")
    page.keyboard.press("2")  # Navigation: the screen kind must survive regime changes
    seen, notices = [], []
    for i in range(0, len(frames), 60):
        page.evaluate(INJECT_MANY, frames[i:i + 60])
        s = page.evaluate(STATE)
        if not seen or seen[-1] != s["regime"]:
            seen.append(s["regime"])
            notices.append(page.evaluate("document.querySelector('#notice').textContent"))
        if s["screen"] != "nav":
            fails.append(f"gnc follow: screen kind lost in {s['regime']}: {s['screen']}")
            break
    if seen != ["prelaunch", "ascent", "landing"]:
        fails.append(f"gnc follow: regimes {seen}")
    if "Phase: ASCENT -> LANDING" not in notices:
        fails.append(f"gnc follow: notices {notices}")
    if not page.evaluate(STATE)["follow"]:
        fails.append("gnc follow: follow turned itself off")
    # Manual regime: follow off; the one-click way back resumes.
    page.click("#regime-btn")
    page.click('.regime-item[data-regime="ascent"]')
    s = page.evaluate(STATE)
    if (s["regime"], s["follow"]) != ("ascent", False):
        fails.append(f"gnc follow: manual regime {s}")
    page.evaluate(INJECT_MANY, extra_frames(seq, 50.0, 10))
    if page.evaluate(STATE)["regime"] != "ascent":
        fails.append("gnc follow: paged while follow was off")
    if "OFF" not in page.locator("#follow-btn").inner_text():
        fails.append("gnc follow: button does not say OFF")
    page.click("#follow-btn")
    s = page.evaluate(STATE)
    if (s["regime"], s["follow"]) != ("landing", True):
        fails.append(f"gnc follow: resume {s}")
    ctx.close()

    if any("three" in u for u in requests):
        fails.append("gnc screens: the GNC page loaded three.js: " + str([u for u in requests if "three" in u]))
    if any("/ws/truth" in u for u in sockets) or not sockets:
        fails.append(f"gnc screens: sockets {sockets}")
    fails += ["gnc screens: " + e for e in errors]
    print(f"gnc screens: routing, drawing, reload, planned regimes, follow ({' > '.join(seen)}), popovers checked")


PERF_DRIVER = """async () => {
  const K = await import('/tests/gncPack.js');
  const seq = {}, tStep = { t: 0 };
  const put = (apid, user) => {
    const s = seq[apid] = (seq[apid] === undefined ? 0 : seq[apid] + 1);
    window.__gncInject(K.packFrame({ runId: 7, groundTime: tStep.t + 0.15, apid, seq: s, time: tStep.t, user }));
  };
  const step = (k) => {
    const t = tStep.t = k * 0.02;
    const w = Math.sin(t);
    put(0x110, K.packNav({ q: [1, 0.02 * w, 0.01 * w, 0], rate: [0.01 * w, 0, 0], force: [0, 0, 9.8], alt: 20 + 10 * w, vz: w, range: 20, rangeValid: true,
      east: 5 * w, north: -4 * w, ve: w, vn: -w, be: 0, bn: 0 }));
    put(0x120, K.packGnc({ altSp: 25, vSp: 0, eSp: 0, nSp: 0, aE: 0.2 * w, aN: 0, attErr: [0, 0, 0],
      engines: [0, 1, 2].map((i) => ({ on: true, thr: 0.6 + 0.1 * w, pitch: 0.05 * w, yaw: 0.04 * Math.cos(t + i) })) }));
    put(0x130, K.packFeedback({ engines: [0, 1, 2].map((i) => ({ valid: true, pitch: 0.05 * Math.sin(t - 0.1), yaw: 0.04 * Math.cos(t + i - 0.1) })) }));
    if (k % 5 === 0) {
      put(0x100, K.packFsw({ mode: 4, flags: 0b10111, fixCount: k / 5, cycles: k * 20, timeInMode: t, prop: 3e5 - t, mass: 5e5 - t }));
      put(0x140, K.packGnss({ fix: 2, used: 20, visible: 24, fixTime: t, lat: 0.4537, lon: -1.6957, h: 30, vel: [0, 0, 0], hSigma: 0.02, vSigma: 0.04 }));
    }
    if (k % 50 === 0) put(0x101, K.packConfig({ lat: 0.4537, lon: -1.6957, h: 0, targetAlt: 25, tol: 1.5, translateAlt: 15 }));
  };
  window.__perf = { k: 0, raf: [], longTasks: [] };
  for (let k = 0; k < 3000; k++) step(k); // 60 s of history, as late in a flight
  window.__perf.k = 3000;
  new PerformanceObserver((l) => { for (const e of l.getEntries()) window.__perf.longTasks.push(e.duration); }).observe({ entryTypes: ['longtask'] });
  (function loop(t) { window.__perf.raf.push(t); requestAnimationFrame(loop); })(0);
  // Live rate: one 50 Hz tick per 20 ms (3 packets) plus the 10 Hz and 1 Hz packets, about 170 packets/s.
  setInterval(() => { step(window.__perf.k++); }, 20);
}"""

# Measured at 4x CPU throttle on the development machine (Edge, headless): busy 21 to 26%, p95 frame
# 16.8 ms (one 60 Hz frame), no long task. The thresholds are about 2x those values, so the test
# catches regressions (for example a screen that draws everything again) without being flaky on a
# loaded machine.
PERF_MAX_BUSY = 0.55
PERF_MAX_P95_MS = 33.4
PERF_MAX_LONG_TASK_MS = 100


def perf(browser, base, fails, screen_ids=("gnc", "overview", "actuators", "nav")):
    """4x CPU throttle, live-rate packets for 5 s: busy share, frame pacing, long tasks."""
    out = []
    for screen in screen_ids:
        ctx = browser.new_context(viewport={"width": 1920, "height": 1080})
        page = ctx.new_page()
        page.goto(f"{base}/gnc/#/ascent/{screen}")
        page.wait_for_function("typeof window.__gncInject === 'function'", timeout=10000)
        page.evaluate(PERF_DRIVER)
        page.wait_for_timeout(1500)
        cdp = ctx.new_cdp_session(page)
        cdp.send("Emulation.setCPUThrottlingRate", {"rate": 4})
        cdp.send("Profiler.enable")
        cdp.send("Profiler.start")
        page.evaluate("window.__perf.raf = []; window.__perf.longTasks = []")
        page.wait_for_timeout(5000)
        prof = cdp.send("Profiler.stop")["profile"]
        res = page.evaluate("""() => {
          const r = window.__perf.raf, d = [];
          for (let i = 1; i < r.length; i++) d.push(r[i] - r[i - 1]);
          d.sort((a, b) => a - b);
          return { n: r.length, p95: d[Math.floor(d.length * 0.95)] || 0, worst: Math.max(0, ...window.__perf.longTasks) };
        }""")
        nodes = {n["id"]: n for n in prof["nodes"]}
        total = idle = 0.0
        for nid, dt in zip(prof["samples"], prof["timeDeltas"]):
            total += dt
            if nodes[nid]["callFrame"]["functionName"] == "(idle)":
                idle += dt
        busy = 1 - idle / total if total else 1
        out.append(f"{screen} busy {busy * 100:.0f}% p95 {res['p95']:.1f} ms long task max {res['worst']:.0f} ms")
        if busy > PERF_MAX_BUSY:
            fails.append(f"gnc perf {screen}: busy {busy:.2f} > {PERF_MAX_BUSY}")
        if res["p95"] > PERF_MAX_P95_MS:
            fails.append(f"gnc perf {screen}: p95 frame {res['p95']:.1f} ms > {PERF_MAX_P95_MS}")
        if res["worst"] > PERF_MAX_LONG_TASK_MS:
            fails.append(f"gnc perf {screen}: long task {res['worst']:.0f} ms > {PERF_MAX_LONG_TASK_MS}")
        ctx.close()
    print("gnc perf (4x throttle, 5 s, 170 packets/s): " + "; ".join(out))
