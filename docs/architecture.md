# Architecture

The binding rules (layering, the step, determinism, no allocation in the step loop) are in the root `CLAUDE.md`. This file holds the longer design: rate plan, runtime architecture, rendering and mission control. Most of it is planned, not built; see `build_order.md` for status.

## Rate groups

Base rate 1 kHz (dt = 1 ms). Every model rate must divide the base rate exactly (the scheduler throws otherwise).

| Model | Rate |
|---|---|
| Rigid body, slosh, bending, actuators | 1 kHz in RK4 |
| Engine internals (turbopumps, chamber) | subcycled 5 to 10 kHz, or implicit |
| Aero, thrust, low-order gravity | inside `derivatives()` (table lookups) |
| High-order gravity harmonics | 10 to 100 Hz correction |
| Tank thermodynamics | 50 to 100 Hz |
| Tile thermal | 10 to 50 Hz, multithreaded |
| IMU | 500 Hz (current) |
| Radar altimeter | 50 Hz (current) |
| GNSS | 5 to 10 Hz |
| Flight software | 100 Hz (current) |
| Snapshot to renderer | 100 Hz (current; browser interpolates to display rate) |
| Logging | 100 Hz (current) |

Heavy physics is precomputed offline into tables: aero coefficients (CFD with OpenFOAM or SU2), combustion properties (NASA CEA), upper atmosphere (NRLMSISE-00).

## Runtime architecture (planned)

Built so far (interim): the flight computer downlinks CCSDS packets through a placeholder link model, `sim_server` forwards them on `/ws/telemetry` (`docs/protocol/telemetry_stream.md`), and a GNC console at `/gnc/` decodes them directly. When Yamcs arrives, the vehicle side and the packet dictionary (`docs/telemetry/gnc_packets.md`) stay; Yamcs takes over from the WebSocket forwarding.

Flight controllers at consoles watch live telemetry and make real-time decisions, with visuals alongside. One machine runs the server; everyone else joins from a browser on the same local network. The sim is the authority and everything else is a client.

```
                               one host machine                          other laptops on the LAN
  +------------------------------------------------------------------+
  |  sim_server (sim_core + sim_vehicle, flight software in lockstep)|
  |  owns sim time and truth; session and roles; command log         |-- truth snapshots ---> browser outside view, webcast HUD
  +------------------------------------------------------------------+   (WebSocket, binary)
        |                                                             \-- camera feeds -----> camera views inside consoles
        |                                                                 (via video downlink model)
        | downlink (via comms model)        ^ uplink (via comms model)
        v CCSDS packets                     | commands                  <--- supervisor station (browser,
  +------------------------------------------------------------------+         privileged: faults, time, weather)
  |  Yamcs (XTCE telemetry/command database, archive, web API)       |
  +------------------------------------------------------------------+
        | WebSocket                         ^ commands
        v                                   |
     Open MCT consoles in the browser: FLIGHT, PROP, GNC, POWER, THERMAL, STRUCT, COMMS, ...
```

- **`sim_server` is a standalone real-time process.** It owns sim time and truth, and `Pacer` still sets the pace. A client crashing, stalling, losing Wi-Fi or dropping frames never affects the sim. A client that reconnects catches up from current state (and from the Yamcs archive for history), not by asking the sim to wait.
- **Browser first.** Every client is a web page served on the LAN: lobby, consoles, 3D views, HUD, supervisor. Nothing to install on participant laptops; any OS works. There is no native renderer: 3D is rendered in the browser so it can be embedded directly in console layouts.
- **Sessions and roles.** The host starts a scenario; participants open the server URL, pick a console position and a name. Each position sees the displays and holds the command permissions of that discipline (PROP can command propulsion, not GNC), enforced on the server, the way real consoles have command authority per position. The flight director position can see everything downlinked and gives the go/no-go calls.
- **Configurable crew size, from one person to a full room.** The position list and how each position is staffed come from a crew configuration chosen in the lobby (presets plus a custom option, stored as a file so sessions are repeatable). Each position is in one of these states:
  - *Staffed*: a person holds it.
  - *Consolidated*: merged into another staffed position, which then gets its displays and command authority (for example PROP and POWER consolidated into one seat). Real control rooms consolidate positions during quiet phases, so this is realistic, not a shortcut.
  - *Automated* (optional, later; build order item 51): a virtual controller holds it. It watches the same downlinked telemetry a person would (never truth), makes the routine calls, follows the flight rules, and escalates anomalies to the flight director rather than silently fixing them. It is deliberately no better than a competent human, and its calls and commands go in the session log like anyone else's.

  Example presets: *Solo* (one person holds every position consolidated, with scripted faults in place of a human supervisor), *Small crew* (2 to 4 people on consolidated positions), *Full room* (one person per position plus a human supervisor). Once virtual controllers exist, Solo and Small crew can hand some positions to automation instead. The crew configuration can change mid-session (a friend joins or leaves and positions are consolidated or split again); the handover is logged. The flight software flies the vehicle regardless of staffing. Positions only monitor, decide and command, as in a real control room.
- **LAN scope and security.** The server binds to the local network only and is never exposed to the internet by default. A session code to join and a separate password for the supervisor role are enough for friends; this is not a hardened public service.
- **Time.** Sim time from the server is the only clock. Clients show mission elapsed time from telemetry timestamps, never their local clock. Only the supervisor can pause or change the time scale, and it applies to everyone.
- **Three output streams, with different trust levels.**
  - *Truth snapshots* (60 to 120 Hz, binary over WebSocket) go only to the outside view and webcast HUD, which play the "outside camera" and may see truth. Consoles never subscribe to this stream.
  - *Camera feeds* go to the camera views embedded in consoles. Each onboard camera is a sensor model (phase 6, item 37) with a mount point, field of view and exposure, and its feed passes through a video downlink model (frame rate, latency, bandwidth, dropouts, blackout) before it leaves the server. The browser renders the camera's view from that degraded stream and adds noise and compression artifacts driven by link quality. If the link drops, the view freezes or goes dark, exactly as a real feed would.
  - *Downlinked telemetry* (CCSDS) goes through the comms model, with its latency, bandwidth limits, dropouts and plasma blackout, before it reaches Yamcs. **Consoles see only downlinked telemetry, never truth**, the same way flight software sees only `fsw::Hal`. A telemetry gap should look like a telemetry gap, not like perfect data.
- **Commands take the real path.** A console sends a command; Yamcs checks it against the XTCE command database; it goes uplink through the comms model (latency, blackout); flight software receives and executes it. Nothing on the ground writes sim state directly.
- **Determinism with humans in the loop.** Every command is recorded with the sim step at which it is applied on board. The seed plus the command log reproduces the run bit for bit, for replay and debrief.
- **Supervisor (instructor) station.** A separate privileged client that injects faults, changes weather and controls time (pause, slow motion). Controllers do not see its actions except through their effects on telemetry. The supervisor can be a person or a script: solo or without a volunteer, a fault script picked in the lobby (or randomized from a seed, without telling the crew which faults it contains) plays the role. Supervisor actions go in the same log so replays include them.
- **Visual clients are only stream consumers.** Any number of 3D views (outside view, and camera views inside each console) can run at once, and the rendering library can change without touching the sim.
- **Documented protocols.** The snapshot format and the session/role API are versioned and documented, like the XTCE database, so clients can be written in any language.

## Rendering plan (not started)

All rendering is in the browser (WebGL, WebGPU later if useful), in `web/`. No native renderer. One shared rendering module draws the vehicle and scene; it is used in two ways:

- **Outside view and webcast HUD.** Driven by the truth snapshot stream. CesiumJS is the leading candidate because it gives a WGS-84 globe, terrain and imagery for free, which fits ascent, entry and the planned ECEF frames (phase 0, item 2); three.js for close-in scenes (pad, tower catch, hot staging) if Cesium is too limiting there. Large distances: camera-relative rendering (Cesium does this already).
- **Camera views embedded in consoles.** Each position's layout includes the onboard cameras that discipline would watch: PROP gets engine bay and plume cameras, GNC gets views that show attitude and grid fin or flap motion, STRUCT gets cameras on the interstage and tank domes, and so on. They are panels inside the Open MCT layout (a custom Open MCT view), driven by the degraded camera feed stream, never the truth stream. The camera list per position lives in the crew configuration.

Both interpolate between the two latest samples per frame and must run on an ordinary laptop: several camera panels plus plots on one screen. Budget for that with LODs, a capped resolution per panel, and frame rate tied to the feed's own frame rate (a 30 fps camera feed does not need 60 fps rendering).

Models and effects:
- Blender models: each moving part (gimbaled engines, grid fins, flaps, legs) is its own object with its origin at the pivot and local axes aligned with its rotation axes. Export glTF, with web-weight LODs so laptops on Wi-Fi load it quickly.
- Animate actual actuator positions from the stream, never commanded ones.
- Plumes driven by throttle and ambient pressure; entry glow driven by the heating model.

Detailed analysis by plotting CSV logs in Python.

## Mission control plan (phase 10)

CCSDS packets and an XTCE telemetry and command database between vehicle and ground, served by Yamcs (Java, runs on the host). Operator consoles are an Open MCT web app served on the LAN and connected to Yamcs, with one layout per position (flight director, propulsion, GNC, power, thermal, structures, comms): live plots, limit alarms, event logs and command panels, all fed by downlinked telemetry only. Commanding follows the uplink path described above, with command permissions per position. The webcast-style HUD on the 3D view is the public-facing display, separate from the consoles.

Later, optional: voice loops between positions (WebRTC in the browser), since real mission control coordinates by voice; flight rules and procedures shown alongside the consoles.
