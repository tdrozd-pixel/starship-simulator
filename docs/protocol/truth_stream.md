# Truth stream protocol, version 1

The truth stream carries simulated **truth** (not telemetry) from `sim_server` to visual clients: the outside view and the webcast HUD. Consoles never subscribe to it (see "Runtime architecture" in the top-level CLAUDE.md).

Status: version 1, used by the vertical hop scenario (placeholder test hopper).

## Transport

- `sim_server` serves HTTP and WebSocket on one port (default `8080`, bound to `0.0.0.0` so laptops on the LAN can join; `--host 127.0.0.1` for local only).
- `GET /` and other paths: static files from the web root (`web/`).
- WebSocket endpoint: `/ws/truth`.
- Binary frames carry snapshots. Text frames carry JSON control messages.
- Clients must ignore unknown JSON `type` values and unknown binary message types, so the protocol can grow.

## Frames and units

SI units, radians.

- World frame: flat local ENU, z up, origin at the pad center (placeholder until WGS-84 / site frames, phase 0 item 2).
- Body frame: +z along the vehicle axis toward the nose. Origin at the vehicle base on the centerline (z = 0 when standing on the pad).
- Attitude quaternion rotates body vectors into the world frame.
- Conversion from these frames to the renderer's convention (three.js is Y-up) happens in exactly one module in `web/`.

## Binary message: TruthSnapshot (type 1)

Sent at `snapshotRateHz` (100 Hz of sim time; must divide the 1 kHz base rate, so the wall-clock rate is `snapshotRateHz * timeScale`) while a run is in progress. Every snapshot produced by the sim is sent, in order, with no duplicates, unless a client falls too far behind.

Slow clients: each client has its own bounded send queue, so a slow or stalled client never delays the sim or other clients. When a client's queue is full, the server drops that client's oldest queued snapshots (never JSON control messages, and never the final halted snapshot). A client that cannot drain at all for several seconds is disconnected; it may reconnect and will get `hello` plus the latest snapshot.

After the run ends (`halted` flag set) the server keeps sending the final snapshot at 1 Hz as a heartbeat. Heartbeats repeat the same `step`, so a client that drops `step <= lastStep` ignores them; they exist for liveness. The final halted snapshot's `step` is always strictly greater than the last periodic snapshot's.

Run start: by default a run waits on the pad (no sim steps, no snapshots) until at least one client is connected, so the first viewer sees the run from T+0. `sim_server --autostart` starts runs immediately.

Little-endian. Fixed size **176 bytes**. Every `f64` sits at a multiple of 8.

| Offset | Type | Field | Units / notes |
|---:|---|---|---|
| 0 | u8[4] | magic | ASCII `SSTR` (0x53 0x53 0x54 0x52) |
| 4 | u16 | protocolVersion | 1 |
| 6 | u16 | messageType | 1 = TruthSnapshot |
| 8 | u32 | runId | increments each time a run (re)starts |
| 12 | u32 | flags | bit 0 engineRunning, bit 1 touchdownOccurred, bit 2 halted (run finished); other bits 0 |
| 16 | u64 | step | sim step count since run start |
| 24 | f64 | simTime | [s] since run start |
| 32 | f64[3] | position x, y, z | world [m] |
| 56 | f64[3] | velocity x, y, z | world [m/s] |
| 80 | f64[4] | attitude w, x, y, z | unit quaternion, body to world |
| 112 | f64[3] | angularRate x, y, z | body [rad/s] |
| 136 | f64 | throttle | **actual** throttle actuator position, fraction of max thrust. It can be nonzero while the engine is off (the actuator has not returned to 0); thrust is zero whenever `engineRunning` is clear |
| 144 | f64 | gimbalPitch | **actual** gimbal angle [rad] |
| 152 | f64 | gimbalYaw | **actual** gimbal angle [rad] |
| 160 | f64 | propellantMass | [kg] |
| 168 | f64 | totalMass | [kg] |

Actuator fields are actual positions from the actuator models, never commanded values.

Gimbal sign convention (from `sim_vehicle/src/HopperDynamics.cpp`): with p = gimbalPitch and y = gimbalYaw, the thrust direction in the body frame is `(cos p * sin y, -sin p, cos p * cos y)`. The exhaust points the opposite way. Zero gimbal thrusts along body +z.

The server must serve `.js` as `text/javascript` (browsers refuse ES modules otherwise), plus `.html`, `.css`, `.json`, `.txt` and `.glb` with their standard types.

## JSON messages, server to client

`hello`, sent once right after the WebSocket opens, followed immediately by the latest snapshot if one exists:

```json
{
  "type": "hello",
  "protocol": 1,
  "scenario": "hop",
  "runId": 3,
  "baseRateHz": 1000,
  "snapshotRateHz": 100,
  "timeScale": 1.0,
  "vehicle": { "name": "test hopper (placeholder)", "length": 20.0, "radius": 4.5 }
}
```

`vehicle` describes the **physics** model. A client may draw a different visual model (for example a Ship-sized cylinder) but should say so.

`run_started`, sent to every client when a new run begins:

```json
{ "type": "run_started", "runId": 4 }
```

Clients drop their interpolation buffer when `runId` changes.

## JSON messages, client to server

`restart`: end the current run (if any) and start a new one from the pad with the same options.

```json
{ "type": "restart" }
```

This is a session-control action. Today any client may send it. Restarts are coalesced and rate limited (at most one per second); extra requests inside that window are ignored. Only an unfragmented text frame whose top-level `type` is `"restart"` counts. TODO(phase 10, item 48): restrict to the supervisor role once sessions and roles exist.

## Versioning

Any change to the binary layout or to required JSON fields increments `protocolVersion`. Clients that see an unknown version show an error instead of guessing.
