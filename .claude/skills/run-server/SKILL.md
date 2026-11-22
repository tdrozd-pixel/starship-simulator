---
name: run-server
description: Launch sim_server and view the hop in a browser, locally or for laptops on the LAN. Use when asked to run, demo or screenshot the simulator, or to confirm a server or web client change works in the real app.
---

# Run the simulation server

Build first (see the `build-and-test` skill). The server binary is `build/sim_server/Release/sim_server.exe`. It serves the web client from `web/` and the truth stream at `/ws/truth` (protocol: `docs/protocol/truth_stream.md`).

## Local only (no firewall prompt)

```bash
build/sim_server/Release/sim_server.exe --host 127.0.0.1 --port 8080
```
Then open http://127.0.0.1:8080/. Run it in the background when an agent needs to keep working, and stop it when done.

## For laptops on the LAN

```bash
build/sim_server/Release/sim_server.exe            # binds 0.0.0.0:8080
```
The server prints a URL per LAN address. The first run on Windows shows a firewall prompt: allow **Private networks** only.

Useful options: `--time-scale X` (finite, 0 < X <= 100; 0.25 is slow motion), `--seed 42`, `--ideal-sensors`, `--port N`, `--web-root DIR`, `--autostart`. Each run waits on the pad until a client connects unless `--autostart` is given. The hop takes about 40 s of sim time; the Restart button in the page (or `{"type":"restart"}` on the WebSocket, at most once per second) starts a new run. Up to 24 WebSocket clients.

Stop it with Ctrl-C (or Ctrl-Break); it exits within about 1 s and prints drop counters.

## Automated end-to-end check

```bash
python -I tools/e2e/server_smoke.py
```
Starts the built server on a free 127.0.0.1 port and checks the protocol, restart ordering, the halted frame and heartbeat, a stalled client not disturbing a healthy one, and clean shutdown. Prefer this over ad hoc scripts.

## Verify without a human

- Stream check: a Python `websocket-client` script that connects to `ws://127.0.0.1:PORT/ws/truth`, reads the `hello` text message, then checks binary frames are 176 bytes with magic `SSTR` and increasing `step`. Put throwaway scripts in a temp directory, not the repo.
- Visual check: Python Playwright headless, open the page, wait a few seconds, take a screenshot into a temp directory and look at it. Expect a cylinder on the pad that climbs to about 50 m and lands, with the HUD showing CONNECTED.

Always stop servers you started.
