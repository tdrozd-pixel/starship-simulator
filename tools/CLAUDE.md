# tools

- `e2e/server_smoke.py`: end-to-end test against the real Release `sim_server` on 127.0.0.1 and a free port. Checks the truth stream protocol (hello, frames, restart, halted frame, heartbeat), that a stalled client does not disturb a healthy one, and clean shutdown. Needs `websocket-client`. Takes about a minute. Run with `python -I tools/e2e/server_smoke.py [path/to/sim_server]`.
- Python tools run with `-I`. They talk to the server only over the network, like any client.
