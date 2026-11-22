"""End-to-end smoke test for sim_server.

Run (after building Release):

    python -I tools/e2e/server_smoke.py [path/to/sim_server(.exe)]

Needs the `websocket-client` package. Starts the server on 127.0.0.1 and a free
port, checks the protocol described in docs/protocol/truth_stream.md (hello,
frames, restart, halted frame and heartbeat, a stalled client that must not
disturb a healthy one, the GNC page and /ws/telemetry (docs/protocol/telemetry_stream.md), and a clean shutdown on Ctrl-Break / SIGINT), and kills
the server in all cases. Takes about a minute. Exit code 0 means every check passed.
"""

import os
import signal
import socket
import struct
import subprocess
import sys
import http.client
import tempfile
import time
import json

import websocket  # pip install websocket-client

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
IS_WINDOWS = os.name == "nt"
failures = []


def check(cond, message):
    print(("  ok   " if cond else "  FAIL ") + message)
    if not cond:
        failures.append(message)


def default_server_path():
    exe = "sim_server.exe" if IS_WINDOWS else "sim_server"
    for sub in (os.path.join("build", "sim_server", "Release"), os.path.join("build", "sim_server")):
        candidate = os.path.join(ROOT, sub, exe)
        if os.path.exists(candidate):
            return candidate
    raise SystemExit("sim_server not found; build it first or pass its path")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Server:
    def __init__(self, exe, *args):
        self.port = free_port()
        self.log = tempfile.NamedTemporaryFile("w+", delete=False, suffix=".log")
        flags = subprocess.CREATE_NEW_PROCESS_GROUP if IS_WINDOWS else 0
        self.proc = subprocess.Popen(
            [exe, "--host", "127.0.0.1", "--port", str(self.port), *args],
            stdout=self.log, stderr=subprocess.STDOUT, creationflags=flags)
        deadline = time.time() + 10
        while time.time() < deadline:
            try:
                socket.create_connection(("127.0.0.1", self.port), timeout=0.2).close()
                return
            except OSError:
                time.sleep(0.05)
        raise RuntimeError("server did not start")

    def url(self):
        return f"ws://127.0.0.1:{self.port}/ws/truth"

    def tlm_url(self):
        return f"ws://127.0.0.1:{self.port}/ws/telemetry"

    def output(self):
        self.log.flush()
        with open(self.log.name) as f:
            return f.read()

    def stop_gracefully(self, timeout=2.0):
        """Ctrl-Break (Windows) or SIGINT; returns (exit_code, seconds)."""
        t0 = time.time()
        self.proc.send_signal(signal.CTRL_BREAK_EVENT if IS_WINDOWS else signal.SIGINT)
        try:
            code = self.proc.wait(timeout=timeout + 8)
        except subprocess.TimeoutExpired:
            code = None
        return code, time.time() - t0

    def kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()
        self.log.close()
        try:
            os.unlink(self.log.name)
        except OSError:
            pass


def decode(data):
    assert len(data) == 176, f"frame is {len(data)} bytes"
    assert data[:4] == b"SSTR", data[:4]
    version, mtype, run_id, flags, step = struct.unpack_from("<HHIIQ", data, 4)
    sim_time, = struct.unpack_from("<d", data, 24)
    return {"version": version, "type": mtype, "run": run_id, "flags": flags, "step": step, "t": sim_time}


def recv(ws):
    """Returns ('text', dict) or ('bin', frame dict)."""
    op, data = ws.recv_data()
    if op == websocket.ABNF.OPCODE_TEXT:
        return "text", json.loads(data)
    return "bin", decode(data)


def test_basic_and_restart(exe):
    print("basic stream, restart ordering and rate limit")
    srv = Server(exe)
    try:
        ws = websocket.create_connection(srv.url(), timeout=10)
        kind, hello = recv(ws)
        check(kind == "text" and hello["type"] == "hello" and hello["protocol"] == 1, "hello received")
        check(hello["scenario"] == "hop" and hello["baseRateHz"] == 1000 and hello["snapshotRateHz"] == 100,
              "hello rates")
        check(hello["vehicle"]["name"].startswith("Starship") and hello["vehicle"]["length"] > 0,
              "hello vehicle")
        frames = []
        while len(frames) < 100:
            kind, m = recv(ws)
            if kind == "bin":
                frames.append(m)
        steps = [f["step"] for f in frames]
        check(all(f["version"] == 1 and f["type"] == 1 for f in frames), "version/type fields")
        check(all(b - a == 10 for a, b in zip(steps, steps[1:])), "100 frames, step strictly +10")
        check(steps[0] == 0, "run waited for the client: first step is 0")
        run = frames[0]["run"]
        check(run == hello["runId"] or hello["runId"] == 0, "hello runId consistent")

        # Two restarts back to back: the second is rate limited.
        ws.send(json.dumps({"type": "restart"}))
        ws.send(json.dumps({"type": "restart"}))
        started = []
        new_frames = []
        old_after_new = False
        t0 = time.time()
        while time.time() - t0 < 4:
            kind, m = recv(ws)
            if kind == "text" and m["type"] == "run_started":
                started.append(m["runId"])
            elif kind == "bin":
                if m["run"] == run + 1:
                    new_frames.append(m["step"])
                elif new_frames:
                    old_after_new = True
        check(started == [run + 1], f"exactly one run_started, runId+1 (got {started})")
        check(new_frames and new_frames[0] == 0, "new run steps restart at 0")
        check(not old_after_new, "no old-run frame after the first new-run frame")
        ws.close()
    finally:
        srv.kill()


def test_halted_and_heartbeat(exe):
    print("halted frame and heartbeat at --time-scale 20")
    srv = Server(exe, "--time-scale", "20")
    try:
        ws = websocket.create_connection(srv.url(), timeout=10)
        recv(ws)
        last_periodic = None
        halted_times = []
        halted_step = None
        t0 = time.time()
        while time.time() - t0 < 30 and len(halted_times) < 4:
            kind, m = recv(ws)
            if kind != "bin":
                continue
            if m["flags"] & 4:
                halted_times.append(time.time())
                halted_step = m["step"] if halted_step is None else halted_step
            else:
                last_periodic = m["step"]
        check(halted_step is not None, "halted frame (flag bit 2) arrived")
        check(halted_step is not None and halted_step > last_periodic, "halted step > last periodic step")
        gaps = [b - a for a, b in zip(halted_times, halted_times[1:])]
        check(len(gaps) >= 3 and all(0.8 < g < 1.3 for g in gaps), f"~1 Hz heartbeat (gaps {[round(g, 2) for g in gaps]})")
        ws.close()
    finally:
        srv.kill()


def test_stalled_client_and_shutdown(exe):
    print("stalled client must not disturb a healthy client; clean shutdown")
    srv = Server(exe)
    try:
        # Stalled: connects, completes the handshake, never reads, tiny receive buffer.
        stalled = websocket.create_connection(
            srv.url(), timeout=10, sockopt=[(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)])
        good = websocket.create_connection(srv.url(), timeout=10)
        stamps = []
        steps = []
        t0 = time.time()
        while time.time() - t0 < 15:
            kind, m = recv(good)
            if kind == "bin" and not m["flags"] & 4:
                stamps.append(time.time())
                steps.append(m["step"])
        rate = len(stamps) / (stamps[-1] - stamps[0]) if len(stamps) > 1 else 0
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        check(95 <= rate <= 105, f"healthy client sees ~100 Hz for 15 s (got {rate:.1f} Hz)")
        check(max(gaps) < 0.5, f"no stall longer than 0.5 s (max gap {max(gaps):.3f} s)")
        check(all(b - a == 10 for a, b in zip(steps, steps[1:])), "healthy client lost no frames")

        code, seconds = srv.stop_gracefully()
        out = srv.output()
        check(code == 0, f"exit code 0 (got {code})")
        check(seconds < 2.0, f"shutdown within 2 s with a stalled client (took {seconds:.2f} s)")
        check("Shutting down" in out, "shutdown line printed")
        print("  server output tail:", " | ".join(out.strip().splitlines()[-3:]))
        stalled.close()
    finally:
        srv.kill()


APIDS = {"fsw_status": 256, "mission_config": 257, "navigation": 272,
         "guidance_control": 288, "actuator_feedback": 304, "gnss": 320}


def decode_ground(data):
    """Returns the ground frame as a dict; asserts the header contract."""
    assert len(data) >= 24, f"frame is {len(data)} bytes"
    assert data[:4] == b"STLM", data[:4]
    version, ftype, run_id, plen, reserved = struct.unpack_from("<HHIHH", data, 4)
    ground_time, = struct.unpack_from("<d", data, 16)
    assert version == 1 and ftype == 1 and reserved == 0, (version, ftype, reserved)
    assert len(data) == 24 + plen, (len(data), plen)
    pkt = data[24:]
    apid = ((pkt[0] & 0x07) << 8) | pkt[1]
    seq = ((pkt[2] & 0x3F) << 8) | pkt[3]
    data_len, = struct.unpack_from(">H", pkt, 4)
    return {"run": run_id, "t": ground_time, "apid": apid, "seq": seq, "version": pkt[0] >> 5,
            "sec_hdr": (pkt[0] >> 3) & 1, "len_ok": len(pkt) == 7 + data_len}


def recv_tlm(ws):
    op, data = ws.recv_data()
    if op == websocket.ABNF.OPCODE_TEXT:
        return "text", json.loads(data)
    return "bin", decode_ground(data)


def http_get(port, path):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", path)
    resp = conn.getresponse()
    body = resp.read()
    result = (resp.status, resp.getheader("Location"), body)
    conn.close()
    return result


def test_telemetry_stream(exe):
    print("gnc page and telemetry stream")
    srv = Server(exe)
    try:
        status, _, body = http_get(srv.port, "/gnc/")
        check(status == 200 and len(body) > 0, f"GET /gnc/ is 200 (got {status})")
        status, location, _ = http_get(srv.port, "/gnc")
        print(f"  info: /gnc answers {status}" + (f" -> {location}" if location else ""))
        check(status in (301, 302, 307, 308) and location == "/gnc/", "GET /gnc redirects to /gnc/ (relative)")

        ws = websocket.create_connection(srv.tlm_url(), timeout=10)
        kind, hello = recv_tlm(ws)
        check(kind == "text" and hello["type"] == "hello" and hello["stream"] == "telemetry"
              and hello["protocol"] == 1, "telemetry hello")
        check(hello["apids"] == APIDS, "hello apids map")
        frames = []
        started = []
        t0 = time.time()
        while time.time() - t0 < 3.0:
            kind, m = recv_tlm(ws)
            if kind == "bin":
                frames.append(m)
            elif m["type"] == "run_started":
                started.append(m["runId"])
        check(len(frames) > 100, f"ground frames arrive ({len(frames)} in 3 s)")
        check(all(f["version"] == 0 and f["sec_hdr"] == 1 and f["len_ok"] for f in frames),
              "valid CCSDS primary headers and length fields")
        seen = {f["apid"] for f in frames}
        check(set(APIDS.values()) <= seen, f"all six APIDs seen (got {sorted(seen)})")
        run = frames[0]["run"]
        check(all(f["run"] == run for f in frames) and (not started or started == [run]), "single run id")
        check(all(b["t"] >= a["t"] for a, b in zip(frames, frames[1:])), "ground time nondecreasing")
        gaps = 0
        for apid in APIDS.values():
            seqs = [f["seq"] for f in frames if f["apid"] == apid]
            ok = all(0 < (b - a) % 16384 < 8192 for a, b in zip(seqs, seqs[1:]))
            check(ok, f"APID {apid}: sequence counts increase")
            gaps += sum(((b - a) % 16384) - 1 for a, b in zip(seqs, seqs[1:]))
        check(gaps <= 2, f"sequence gaps are rare ({gaps})")

        # A second client joining mid-run gets hello, then the current value table at once.
        ws2 = websocket.create_connection(srv.tlm_url(), timeout=10)
        t_join = time.time()
        kind, hello2 = recv_tlm(ws2)
        check(kind == "text" and hello2["stream"] == "telemetry" and hello2["runId"] == run,
              "second client: hello with the current run id")
        replay = []
        while len(replay) < 6:
            kind, m = recv_tlm(ws2)
            if kind == "bin":
                replay.append(m)
        check(time.time() - t_join < 1.0, "replay arrives immediately")
        check(len({f["apid"] for f in replay}) >= 4,
              f"replay holds several distinct APIDs ({sorted({f['apid'] for f in replay})})")
        check(all(f["run"] == run for f in replay), "replay is the current run")
        ws2.close()
        ws.close()
    finally:
        srv.kill()


def test_stalled_telemetry_client(exe):
    print("stalled telemetry client must not disturb healthy truth and telemetry clients")
    srv = Server(exe)
    try:
        stalled = websocket.create_connection(
            srv.tlm_url(), timeout=10, sockopt=[(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)])
        truth = websocket.create_connection(srv.url(), timeout=10)
        tlm = websocket.create_connection(srv.tlm_url(), timeout=10)
        truth.settimeout(0.01)
        tlm.settimeout(0.01)
        truth_stamps, tlm_stamps, truth_steps = [], [], []
        t0 = time.time()
        while time.time() - t0 < 12:
            for ws, stamps, kind_fn in ((truth, truth_stamps, recv), (tlm, tlm_stamps, recv_tlm)):
                try:
                    kind, m = kind_fn(ws)
                except (websocket.WebSocketTimeoutException, socket.timeout):
                    continue
                if kind == "bin":
                    stamps.append(time.time())
                    if ws is truth and not m["flags"] & 4:
                        truth_steps.append(m["step"])

        def worst_gap(stamps):
            return max(b - a for a, b in zip(stamps, stamps[1:]))

        truth_rate = len(truth_stamps) / (truth_stamps[-1] - truth_stamps[0])
        check(95 <= truth_rate <= 105, f"healthy truth client ~100 Hz (got {truth_rate:.1f})")
        check(worst_gap(truth_stamps) < 0.5, f"truth: no stall longer than 0.5 s ({worst_gap(truth_stamps):.3f})")
        check(all(b - a == 10 for a, b in zip(truth_steps, truth_steps[1:])), "truth client lost no frames")
        check(len(tlm_stamps) > 12 * 100, f"healthy telemetry client keeps receiving ({len(tlm_stamps)} frames)")
        check(worst_gap(tlm_stamps) < 0.5, f"telemetry: no stall longer than 0.5 s ({worst_gap(tlm_stamps):.3f})")
        code, seconds = srv.stop_gracefully()
        # Healthy clients that stop reading at shutdown make civetweb take about 2 s to close
        # (same for two truth clients), so the bound here is looser than the 2 s one-client case.
        check(code == 0 and seconds < 3.5,
              f"clean shutdown with a stalled telemetry client (code {code}, {seconds:.2f} s)")
        print("  server output tail:", " | ".join(srv.output().strip().splitlines()[-3:]))
        stalled.close()
        truth.close()
        tlm.close()
    finally:
        srv.kill()


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else default_server_path()
    print("server:", exe)
    test_basic_and_restart(exe)
    test_halted_and_heartbeat(exe)
    test_stalled_client_and_shutdown(exe)
    test_telemetry_stream(exe)
    test_stalled_telemetry_client(exe)
    print()
    if failures:
        print(f"{len(failures)} check(s) FAILED")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
