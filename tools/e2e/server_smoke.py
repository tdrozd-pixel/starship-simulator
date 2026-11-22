"""End-to-end smoke test for sim_server.

Run (after building Release):

    python -I tools/e2e/server_smoke.py [path/to/sim_server(.exe)]

Needs the `websocket-client` package. Starts the server on 127.0.0.1 and a free
port, checks the protocol described in docs/protocol/truth_stream.md (hello,
frames, restart, halted frame and heartbeat, a stalled client that must not
disturb a healthy one, and a clean shutdown on Ctrl-Break / SIGINT), and kills
the server in all cases. Takes about a minute. Exit code 0 means every check passed.
"""

import os
import signal
import socket
import struct
import subprocess
import sys
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


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else default_server_path()
    print("server:", exe)
    test_basic_and_restart(exe)
    test_halted_and_heartbeat(exe)
    test_stalled_client_and_shutdown(exe)
    print()
    if failures:
        print(f"{len(failures)} check(s) FAILED")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
