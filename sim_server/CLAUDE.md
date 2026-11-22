# sim_server

Real-time sim process on the LAN. The only code in the repo that does networking (civetweb v1.16, HTTP + WebSocket).

- `SimRunner`: sim thread. Builds the scenario, runs it paced, holds on the pad until a client connects (`--autostart` skips), restarts.
- `SnapshotRing`: lock-free SPSC ring, sim thread to broadcaster.
- `TruthServer`: serves static files from `web/`, `/ws/truth`, a broadcaster thread fanning out to per-client bounded queues, each drained by its own sender thread.
- `WireFormat`, `JsonMessages`: truth stream encoding. The contract is `docs/protocol/truth_stream.md`; change the doc (and bump its version) with the code.
- Planned: telemetry downlink, commands, sessions and roles, command log (see `docs/architecture.md`).

Rules: the sim thread never blocks on networking. Truth snapshots go only to the outside view and HUD, never to consoles. Bind to the LAN only. Tests and agents use `--host 127.0.0.1`.

## Lessons learned

- One stalled client must never stall the others. The first server wrote to every client from one thread with blocking writes; one client that stopped reading froze everyone after about 9 s. Per-client queues and sender threads fixed it; `tools/e2e/server_smoke.py` keeps it fixed.
- A one-shot message (the final halted snapshot) needs a strictly increasing `step` and a guaranteed path through the ring, or clients that de-duplicate by step silently miss it.
- civetweb matches MIME types by path suffix (`.js=text/javascript`, with the dot), and browsers refuse ES modules served with the wrong type.
