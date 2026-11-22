"""Web client test runner. Run with: python -I web/tests/run_web_tests.py

Serves web/ on a free port, runs the browser unit tests, cross-checks the JS
decoder against a Python-packed buffer, smoke-loads index.html, and records a
short clip with the built-in recorder.
"""
import functools
import http.server
import math
import os
import socketserver
import struct
import sys
import threading

from playwright.sync_api import sync_playwright

WEB_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class Handler(http.server.SimpleHTTPRequestHandler):
    # The Windows registry can map .js to text/plain, which breaks ES modules.
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".js": "text/javascript",
        ".mjs": "text/javascript",
        ".html": "text/html",
        ".css": "text/css",
        ".json": "application/json",
        ".txt": "text/plain",
    }

    def log_message(self, *a):
        pass


def start_server():
    handler = functools.partial(Handler, directory=WEB_ROOT)
    srv = socketserver.ThreadingTCPServer(("127.0.0.1", 0), handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


def launch(p):
    tried = []
    for kwargs in ({}, {"channel": "msedge"}):
        try:
            b = p.chromium.launch(headless=True, **kwargs)
            print("browser:", kwargs.get("channel", "bundled chromium"))
            return b
        except Exception as e:  # noqa: BLE001
            tried.append(str(e).splitlines()[0])
    raise RuntimeError("no browser available: " + "; ".join(tried))


def main():
    fails = []
    srv, port = start_server()
    base = f"http://127.0.0.1:{port}"
    with sync_playwright() as p:
        browser = launch(p)
        page = browser.new_page()

        # 1. Unit tests
        page.goto(base + "/tests/test.html")
        page.wait_for_function("window.__results !== undefined", timeout=15000)
        res = page.evaluate("window.__results")
        print(f"unit tests: {res['passed']} passed, {res['failed']} failed")
        fails += res["failures"]

        # 2. Cross-check decoder against Python packing
        # 24 byte header + 19 doubles = 176 (the plan said 20 doubles, which would be 184)
        fmt = "<4sHHIIQ" + "d" * 19
        assert struct.calcsize(fmt) == 176
        vals = [12.345 + i * 1.7 for i in range(19)]
        raw = struct.pack(fmt, b"SSTR", 1, 1, 7, 5, 123456789012, *vals)
        assert len(raw) == 176
        got = page.evaluate(
            """async (hex) => {
              const m = await import('/js/protocol.js');
              const b = new Uint8Array(hex.match(/../g).map(h => parseInt(h, 16)));
              const r = m.decodeBinary(b.buffer);
              const s = r.snap;
              return {kind: r.kind, runId: s.runId, flags: s.flags, step: s.step.toString(),
                      isBig: typeof s.step === 'bigint',
                      f: [s.simTime, ...s.pos, ...s.vel, ...s.quat, ...s.angRate, s.throttle,
                          s.gimbalPitch, s.gimbalYaw, s.propellantMass, s.totalMass]};
            }""",
            raw.hex(),
        )
        if got["kind"] != "snapshot" or got["runId"] != 7 or got["flags"] != 5 or got["step"] != "123456789012" or not got["isBig"]:
            fails.append(f"cross-check header mismatch: {got}")
        for i, (a, b) in enumerate(zip(got["f"], vals)):
            if not math.isclose(a, b, rel_tol=0, abs_tol=1e-12):
                fails.append(f"cross-check field {i}: js {a} py {b}")
        print("cross-check: done")

        # 3. Smoke test of index.html
        smoke = browser.new_page()
        errors = []
        smoke.on("console", lambda m: errors.append("console: " + m.text) if m.type == "error" else None)
        smoke.on("pageerror", lambda e: errors.append("pageerror: " + str(e)))
        smoke.on("requestfailed", lambda r: errors.append("requestfailed: " + r.url))
        smoke.on("response", lambda r: errors.append(f"http {r.status}: {r.url}") if r.status >= 400 else None)
        smoke.goto(base + "/index.html")
        smoke.wait_for_timeout(1500)
        if smoke.query_selector("canvas") is None:
            fails.append("smoke: no canvas")
        real = [e for e in errors if "websocket" not in e.lower() and "/ws/truth" not in e]
        if real:
            fails += ["smoke: " + e for e in real]
        print("smoke: ignored", len(errors) - len(real), "websocket errors")

        # 4. Built-in recorder: record ~2 s at 1080p30, check pacing, size and the file.
        rec = browser.new_page(accept_downloads=True)
        rec_errors = []
        rec.on("pageerror", lambda e: rec_errors.append("pageerror: " + str(e)))
        rec.goto(base + "/index.html")
        rec.wait_for_timeout(500)
        if rec.is_disabled("#btn-rec"):
            fails.append("recorder: Record button disabled (" + str(rec.get_attribute("#btn-rec", "title")) + ")")
        else:
            rec.select_option("#rec-res", "1920x1080")
            rec.select_option("#rec-fps", "30")
            rec.click("#btn-rec")
            rec.wait_for_timeout(300)
            size = rec.evaluate("[document.getElementById('view').width, document.getElementById('view').height]")
            if size != [1920, 1080]:
                fails.append(f"recorder: render size {size}, expected [1920, 1080]")
            start = rec.evaluate("window.__viewStats.renders")
            rec.wait_for_timeout(2000)
            rate = (rec.evaluate("window.__viewStats.renders") - start) / 2.0
            if not 24 <= rate <= 33:
                fails.append(f"recorder: {rate:.1f} renders/s while recording at 30 fps")
            with rec.expect_download(timeout=15000) as dl_info:
                rec.click("#btn-rec")
            dl = dl_info.value
            path = dl.path()
            nbytes = os.path.getsize(path) if path else 0
            name = dl.suggested_filename
            if nbytes <= 0 or not (name.endswith(".mp4") or name.endswith(".webm")) or not name.startswith("starship-"):
                fails.append(f"recorder: bad download {name!r}, {nbytes} bytes")
            rec.wait_for_timeout(200)
            after = rec.evaluate("[document.getElementById('view').width, document.getElementById('view').height]")
            if after == [1920, 1080] and rec.viewport_size != {"width": 1920, "height": 1080}:
                fails.append("recorder: fixed render size not cleared after stop")
            print(f"recorder: {rate:.1f} renders/s at 30 fps, saved {name} ({nbytes} bytes)")
        fails += ["recorder: " + e for e in rec_errors]
        browser.close()
    srv.shutdown()
    if fails:
        print("FAILURES:")
        for f in fails:
            print(" -", f)
        return 1
    print("ALL WEB TESTS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
