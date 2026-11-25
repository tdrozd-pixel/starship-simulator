# Truth stream protocol, version 3

The truth stream carries simulated **truth** (not telemetry) from `sim_server` to visual clients: the outside view and the webcast HUD. Consoles never subscribe to it (see "Runtime architecture" in the top-level CLAUDE.md).

Status: version 3, used by the hop scenario (Starship V3 Ship in hop configuration, three sea-level Raptors; the three Raptor Vacuum engines are in the model and in the frame but are not fired in the hop) with the catch tower (a hypothetical estimate, see `sim_vehicle/include/sim/vehicle/TowerParams.hpp`). Version 2 added per-engine actual throttle, gimbal and running state to version 1; version 3 adds the catch tower state (actual arm angles, tower state machine, catch result, caught and crashed flags) and a `tower` geometry object in `hello`. See "Versioning" for the migration.

## Transport

- `sim_server` serves HTTP and WebSocket on one port (default `8080`, bound to `0.0.0.0` so laptops on the LAN can join; `--host 127.0.0.1` for local only).
- `GET /` and other paths: static files from the web root (`web/`).
- WebSocket endpoint: `/ws/truth`.
- Binary frames carry snapshots. Text frames carry JSON control messages.
- Clients must ignore unknown JSON `type` values and unknown binary message types, so the protocol can grow.

## Frames and units

SI units, radians.

- World frame: flat local ENU, z up, origin at the landing target (Starbase Pad 2 center, the placeholder for the catch point). It is the tangent plane at the target's WGS-84 coordinates (`LaunchSite` in `sim_vehicle/include/sim/vehicle/HopperParams.hpp`); the vehicle lifts off from a point offset from the origin.
- Body frame: +z along the vehicle axis toward the nose. `position` and `velocity` are the vehicle base on the centerline (z = 0 when standing on the pad), not the center of mass.
- Attitude quaternion rotates body vectors into the world frame.
- Conversion from these frames to the renderer's convention (three.js is Y-up) happens in exactly one module in `web/`.

## Binary message: TruthSnapshot (type 1)

Sent at `snapshotRateHz` (100 Hz of sim time; must divide the 1 kHz base rate, so the wall-clock rate is `snapshotRateHz * timeScale`) while a run is in progress. Every snapshot produced by the sim is sent, in order, with no duplicates, unless a client falls too far behind.

Slow clients: each client has its own bounded send queue, so a slow or stalled client never delays the sim or other clients. When a client's queue is full, the server drops that client's oldest queued snapshots (never JSON control messages, and never the final halted snapshot). A client that cannot drain at all for several seconds is disconnected; it may reconnect and will get `hello` plus the latest snapshot.

After the run ends (`halted` flag set) the server keeps sending the final snapshot at 1 Hz as a heartbeat. Heartbeats repeat the same `step`, so a client that drops `step <= lastStep` ignores them; they exist for liveness. The final halted snapshot's `step` is always strictly greater than the last periodic snapshot's.

Run start: by default a run waits on the pad (no sim steps, no snapshots) until at least one client is connected, so the first viewer sees the run from T+0. `sim_server --autostart` starts runs immediately.

Little-endian. Fixed size **296 bytes** (about 29.6 kB/s, 237 kbit/s per client at 100 Hz of sim time, plus WebSocket framing of about 2 to 6 bytes per message; version 1 was 176 bytes, version 2 was 272 bytes). Every `f64` sits at a multiple of 8.

| Offset | Type | Field | Units / notes |
|---:|---|---|---|
| 0 | u8[4] | magic | ASCII `SSTR` (0x53 0x53 0x54 0x52) |
| 4 | u16 | protocolVersion | 3 |
| 6 | u16 | messageType | 1 = TruthSnapshot |
| 8 | u32 | runId | increments each time a run (re)starts |
| 12 | u32 | flags | bit 0 engineRunning (any engine producing thrust), bit 1 touchdownOccurred, bit 2 halted (run finished), bits 3 to 8 per engine running flags (engine `i` is bit `3 + i`, engines numbered as below), bit 9 caught (the tower arms hold the Ship), bit 10 crashed (the Ship hit the tower or an arm; placeholder, the sim freezes in place); other bits 0 |
| 16 | u64 | step | sim step count since run start |
| 24 | f64 | simTime | [s] since run start |
| 32 | f64[3] | position x, y, z | world [m] |
| 56 | f64[3] | velocity x, y, z | world [m/s] |
| 80 | f64[4] | attitude w, x, y, z | unit quaternion, body to world |
| 112 | f64[3] | angularRate x, y, z | body [rad/s] |
| 136 | f64 | throttle | **actual** throttle actuator position, fraction of rated flow, averaged over the engines. It can be nonzero while the engine is off (the actuator has not returned to 0); thrust is zero whenever `engineRunning` is clear |
| 144 | f64 | gimbalPitch | **actual** gimbal angle [rad], averaged over the engines |
| 152 | f64 | gimbalYaw | **actual** gimbal angle [rad], averaged over the engines |
| 160 | f64 | propellantMass | [kg] |
| 168 | f64 | totalMass | [kg] |
| 176 | f64[6] | engineThrottle | **actual** throttle of each engine, fraction of rated flow, engines 0 to 5 at offsets 176, 184, ..., 216. Same caveat as `throttle`: can be nonzero while that engine is not running |
| 224 | f64[3] | engineGimbalPitch | **actual** gimbal pitch [rad] of sea-level engines 0 to 2 (offsets 224, 232, 240) |
| 248 | f64[3] | engineGimbalYaw | **actual** gimbal yaw [rad] of sea-level engines 0 to 2 (offsets 248, 256, 264) |
| 272 | f64[2] | armAngle | **actual** catch arm angle psi [rad] of arm 0 (offset 272) and arm 1 (offset 280), measured from fully open (0) toward closed; see "Catch tower" |
| 288 | u32 | towerState | 0 Open, 1 Closing, 2 Closed, 3 Holding, 4 Opening. 0 when the scenario has no tower |
| 292 | u32 | catchResult | 0 None, 1 Caught, 2 ArmStrike, 3 TowerStrike. Latched at first contact; 0 when there is no tower |

The frame ends at byte 296 (every `f64` still sits at a multiple of 8). Engine numbering: 0 to 2 are the sea-level Raptors (SL1 to SL3, inner ring, gimbaled), 3 to 5 are the Raptor Vacuum engines (RV1 to RV3, outer ring, fixed, so there are no gimbal fields for them). Layout (body xy plane, azimuth from body +x toward +y, 120 degrees apart; values are estimates from `sim_vehicle/include/sim/vehicle/HopperParams.hpp`): SL ring radius 1.55 m at 90, 210, 330 degrees; RVac ring radius 3.6 m at 150, 270, 30 degrees. Engine `i` produces thrust only while its running flag is set.

Actuator fields are actual positions from the actuator models, never commanded values.

The scalar `throttle`, `gimbalPitch` and `gimbalYaw` fields (offsets 136 to 152) are kept for HUDs and simple clients: each is the mean over the three sea-level engines. The per-engine fields at 176 and above are the primary data for the outside view. The sea-level engines get the same commands apart from a small differential gimbal for roll control, so the means are close to each engine.

Gimbal sign convention (from `sim_vehicle/src/HopperDynamics.cpp`): with p = gimbalPitch and y = gimbalYaw, the thrust direction in the body frame is `(cos p * sin y, -sin p, cos p * cos y)`. The exhaust points the opposite way. Zero gimbal thrusts along body +z.

The browser truth view draws all six engines. Each sea-level nozzle and its plume follow that engine's own actual gimbal angles; each plume is scaled by that engine's own actual throttle and drawn only while its running flag is set. The Raptor Vacuum nozzles are fixed and drawn dark; their plumes appear only if their running flag is set. None of this uses flight-software commands.

The server must serve `.js` as `text/javascript` (browsers refuse ES modules otherwise), plus `.html`, `.css`, `.json`, `.txt` and `.glb` with their standard types.

## Catch tower

The tower is ground infrastructure with two chopstick arms (hypothetical estimate; the real Ship has no catch pins, the Ship pins are a placeholder, see `docs/vehicle_reference.md`). Its fixed geometry comes in `hello.tower`; the time varying state is `armAngle`, `towerState`, `catchResult` and flags bits 9 and 10 in every snapshot. `armAngle` values are actual positions from the servo model, never commands.

Tower frame (all in the world ENU frame of this document): `a = approachDir` is the unit vector from the tower center toward the side the Ship approaches from, `b` is `a` rotated +90 degrees about z (`b = (-a_n, a_e)`). The Ship flies along `-a` into the arms. Arm 0 hinges on the `+b` side and arm 1 on the `-b` side, so for a client the hinge of arm `k` is at

`hinge_k = position + hingeSetback * a + s_k * hingeHalfSpacing * b`, with `s_0 = +1`, `s_1 = -1`.

Arm direction convention. The arm direction angle `phi` is measured from `a` and is **positive inward** (toward the approach axis). In tower-frame coordinates (along `a`, lateral `b`) the arm extends from its hinge along `(cos(phi), -s_k * sin(phi))`, so for arm 0 (`s_0 = +1`, hinge at lateral `+hingeHalfSpacing`) a positive `phi` moves the tip toward lateral 0, and a negative `phi` swings it outward, away from the axis. Arm 1 mirrors arm 0 (`s_1 = -1`). With the arm angle psi from the snapshot (0 = fully open, increasing toward closed, same sign for both arms):

`phi(psi) = -armOpenAngle + psi`

so psi = 0 gives `phi = -armOpenAngle` (open, pointing outward) and the closed pose is `psi = armOpenAngle + armClosedAngle`, where `phi = +armClosedAngle`. The tip of arm 0 at angle psi is at tower-frame (along, lateral) = `(hingeSetback + armLength * cos(phi), hingeHalfSpacing - armLength * sin(phi))`.

In `hello.tower`, `armOpenAngle` is a magnitude measured **outward** (20 degrees; the open direction is `phi = -armOpenAngle`) and `armClosedAngle` is a magnitude measured **inward** (about 4.8 degrees with the default 7 m pin offset; the closed direction is `phi = +armClosedAngle`). The closed angle is chosen so the cradle pads (12 m from each hinge, not in `hello`) sit exactly `pinOffset` from the approach axis: `armClosedAngle = asin((hingeHalfSpacing - pinOffset) / 12)`. Clients draw each arm as a box of `armLength` by `2 * armHalfWidth` by `armThickness` whose top surface is at `railHeight`, pivoting about its hinge (no vertical arm motion; documented simplification).

The Ship pins (`pinHeight` above the base along body +z, `pinOffset` lateral from the axis along body +y and -y, pin radius 0.25 m, not in `hello`) are drawn by the client from `hello.tower` and the vehicle model.

## JSON messages, server to client

`hello`, sent once right after the WebSocket opens, followed immediately by the latest snapshot if one exists:

```json
{
  "type": "hello",
  "protocol": 3,
  "scenario": "hop",
  "runId": 3,
  "baseRateHz": 1000,
  "snapshotRateHz": 100,
  "timeScale": 1.0,
  "vehicle": { "name": "Starship V3 Ship, hop configuration", "length": 52.0, "radius": 4.5 },
  "tower": {
    "position": [60, 0],
    "approachDir": [-1, 0],
    "mastHalfWidth": 6,
    "mastHeight": 145,
    "hingeHalfSpacing": 8,
    "hingeSetback": 6,
    "armLength": 16,
    "armHalfWidth": 1,
    "armThickness": 1.5,
    "railHeight": 75,
    "armOpenAngle": 0.349066,
    "armClosedAngle": 0.083427,
    "pinHeight": 30,
    "pinOffset": 7
  }
}
```

`tower` is required in version 3 (a server with no tower sends a degenerate object rather than omitting it). Keys, all in the world ENU frame:

| Key | Units | Meaning |
|---|---|---|
| `position` | [e, n] m | tower center (mast center) |
| `approachDir` | [e, n] unit | `a`, from the tower toward the approach side (axis-aligned today: east or north, positive or negative) |
| `mastHalfWidth` | m | the mast is a square of side `2 * mastHalfWidth`, vertical, centered on `position` |
| `mastHeight` | m | mast top above the origin plane |
| `hingeHalfSpacing` | m | each hinge is this far from the approach axis, along `+b` (arm 0) or `-b` (arm 1) |
| `hingeSetback` | m | hinge distance from the tower center along `a` |
| `armLength` | m | arm length from the hinge |
| `armHalfWidth` | m | arm half width |
| `armThickness` | m | arm vertical thickness; the top surface is at `railHeight` |
| `railHeight` | m | arm top surface above the origin plane |
| `armOpenAngle` | rad | open arm direction from `a`, **positive outward** (away from the approach axis); `armAngle` 0 is this pose |
| `armClosedAngle` | rad | closed arm direction from `a`, **positive inward** (toward the approach axis); the closed `armAngle` is `armOpenAngle + armClosedAngle` |
| `pinHeight` | m | Ship catch pin center above the base, along body +z |
| `pinOffset` | m | Ship catch pin lateral offset from the vehicle axis (pins on body +y and -y) |

Numbers are JSON numbers printed with 6 significant digits; values are hypothetical estimates, not SpaceX data.

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

Version 3 (this document) appends 24 bytes after `engineGimbalYaw` (`armAngle` f64[2], `towerState` u32, `catchResult` u32), defines flags bits 9 and 10, sets `hello.protocol` to 3 and adds the required `hello.tower` object. Every version 1 and 2 field keeps its offset, type and meaning. Version 2 appended 96 bytes after `totalMass` and defined flag bits 3 to 8. There is no negotiation and no compatibility mode: the server speaks only version 3 (`hello.protocol` and the binary `protocolVersion` field are both 3), the C++ decoder (`decodeSnapshot`) rejects frames with another version or length, and the bundled web client accepts only version 3 (a `hello` with another `protocol`, or a binary frame with another `protocolVersion`, is a fatal error shown on screen, with no reconnect loop). Migration: a version 2 client must read the new 24 bytes (or ignore them), accept version 3 in both places and read `hello.tower` before it can draw the tower; bandwidth rises from about 27 kB/s to about 29.6 kB/s per client (296 bytes at 100 Hz). A client and server from different versions are expected to be served together because `sim_server` serves the client from `web/`. The telemetry stream (`/ws/telemetry`) has its own version (still 1); its hello `apids` map gained `engine_status` (336), `catch_status` (352) and `catch_config` (353).
