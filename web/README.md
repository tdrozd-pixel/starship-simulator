# Web truth view

Browser client for the sim server's truth stream (`docs/protocol/truth_stream.md`). Plain ES modules, no build step. three.js 0.180.0 is vendored in `vendor/three/` (unmodified, MIT, see `VERSION.txt`).

## Run

Start `sim_server` (it serves this directory and the WebSocket on one port, default 8080) and open `http://localhost:8080/`. The client connects to `/ws/truth` on the same host.

Controls: drag to orbit, wheel to zoom, `R` or "Reset view (R)" to reset the camera, "Restart" to start a new run.

The visual is a Ship-sized cylinder (9 m x 52 m); the bottom banner names the physics vehicle from the server. The HUD shows simulated truth, not telemetry.

## Smooth motion on remote devices

Snapshots arrive unevenly over Wi-Fi (bursts after 30 to 100 ms stalls, or batched on ~100 ms beacons when a laptop or phone is in power save). The view renders slightly behind the newest snapshot, and `js/interpolator.js` sizes that delay to the measured arrival jitter: 25 ms on localhost or a wire, about 75 to 110 ms on poor Wi-Fi, never more than 250 ms. The delay changes as a small playback speed change, never a jump. If data still runs out, the vehicle is dead-reckoned along its last velocity and body rate for up to 100 ms and blended back when data resumes. Open `/?debug=1` to see the buffer, measured jitter and starved frames on a given device.

## Recording

Use the built-in recorder instead of an OS screen recorder: pick a resolution (720p, 1080p, 1440p) and frame rate (30 or 60 fps), choose whether to include the HUD, and press Record; Stop saves `starship-run<N>-<time>.mp4` (or `.webm` where the browser cannot record MP4). While recording, the view renders at exactly the recording size and frame rate (letterboxed in the window), and each frame is composed with the HUD drawn into it, so the cost is bounded instead of full-window, full-DPI rendering at the display refresh plus an OS capture on top. Keep the tab visible: background tabs stop rendering, which freezes the recording. The video is held in memory until you stop (about 90 MB per minute at 1080p60).

## Layout

- `js/frames.js`: the only place sim ENU frames are converted to three.js frames.
- `js/engineLayout.js`: the six-engine layout (SL ring, RVac ring) in one place, with `HopperParams.hpp` as its source; `js/vehicleView.js` draws all six engines from the per-engine actual fields of protocol v2 and the catch pins from `hello.tower`.
- `js/towerView.js` (three) builds the catch tower mast and two arms once from `hello.tower` and rotates each arm every frame by its actual angle (protocol v3 `armAngle`); the arm math is the pure `js/towerGeometry.js` (hinge positions, arm direction angle, arm end point). The HUD shows a CAUGHT or CRASHED banner from the snapshot flags (text in `hudModel.js`, drawn by `hud.js` and `hudCanvas.js`).
- `js/protocol.js`, `js/interpolator.js`, `js/frames.js`, `js/engineLayout.js`, `js/towerGeometry.js`: pure modules (no three, no DOM access), unit tested.
- `js/truthClient.js`: WebSocket with reconnect and backoff.
- `js/sceneBuilder.js`, `js/vehicleView.js`, `js/cameraRig.js`: the only modules importing three.
- `js/hudModel.js`: pure; the HUD's display strings, shared by the DOM HUD (`js/hud.js`) and the recorded frames (`js/hudCanvas.js`).
- `js/recorder.js`: built-in recorder (pure helpers for format choice, file name and frame pacing, plus the MediaRecorder part).
- `js/hud.js`, `js/main.js`, `css/hud.css`, `index.html`.

## GNC console

`/gnc/` is the GNC position of the mission control simulation. It connects only to `/ws/telemetry` (never the truth stream) and shows what the flight computer downlinks: CCSDS packets per `docs/telemetry/gnc_packets.md`, wrapped in the ground frames of `docs/protocol/telemetry_stream.md`. There is no 3D view and the page does not load three.js. Time shown is always onboard time from the packets; the browser clock is used only for staleness and link statistics.

**Header** (always visible, one row): onboard mission elapsed time, flight computer mode with a mode timeline, the current regime and a Follow phase button, a notice slot ("Phase: ASCENT -> MANEUVER"), an empty alarm slot (reserved for the ground service alarms, build order item 48), link health (connection, loss from sequence gaps, latency), a GNSS summary chip (fix, satellites used/visible, horizontal distance from pad center), one staleness indicator (opens a popover listing every packet; STALE means no packet for more than 1 s of wall clock, 2.5 s for the 1 Hz config packet) and a menu button (chart window 30/60/120 s, gimbal chart engine 1/2/3/all, performance readout, clear history).

**Regimes and screens.** A regime is a phase of flight; each has an ordered list of screens shown as tabs (keys 1 to 9 pick a screen). Screen ids are shared between regimes, so paging to another regime keeps the same kind of screen when the new regime has it.

| Regime | FSW modes | Status | Screens |
|---|---|---|---|
| Prelaunch | Prelaunch | live | Readiness, Navigation, Flight Software |
| Ascent | Ignition, Ascent | live | Ascent, Guidance & Control, Actuators, Navigation, Flight Software |
| Staging | | planned (build order item 41) | |
| Entry | | planned (build order items 17 to 26) | |
| Landing | Descent, Final descent, Landed | live | Landing, Actuators, Navigation, Flight Software |

Planned regimes are greyed out in the regime menu and cannot be opened; they show no data because nothing is downlinked for them yet.

**Following the flight phase.** Follow phase is on by default: when the FSW mode maps to another regime the console switches to it (same screen kind if available) and shows a short notice. Picking a regime from the regime menu turns Follow off (the button then reads "OFF (click to resume)"; one click returns to the current phase). Picking a screen inside the current regime keeps Follow on.

**URLs and memory.** `#/<regime>/<screen>`, for example `#/landing/actuators`. Opening a URL with a hash pins that screen (Follow off); while following, the hash is not written, so a reload keeps following. The last manual selection and the Follow state are kept in `localStorage` (every access is guarded), and used when the URL has no hash. `?debug=1` (or Performance readout in the menu) shows frame times, per-feature cost, packets per second and long tasks.

**Only the visible screen costs anything.** The telemetry store collects every packet whatever is shown. Switching screens destroys the old screen (DOM, canvases, uPlot charts) and builds the new one from the store history at once. Text and lamps update at most 10 Hz, canvases at 5 Hz, charts at 4 Hz on min/max decimated data (about two points per pixel), canvas sizes are cached by a ResizeObserver (no layout reads in the frame loop), and nothing runs while the tab is hidden. Collapsible sections inside screens are closed by default and are not updated while closed.

**Adding a screen or a regime is a data change in `js/gnc/regimes.js`.** A screen is a spec made of rows of widgets (`readouts`, `lamps`, `chart`, `table`, `map`, `gimbals`, `attitude`, `timeline`, `details`, `diagnostics`, `note`, `stack`; renderers in `js/gnc/widgets.js`). Widgets bind to telemetry by dictionary path (`navigation.altitude`, `guidance_control.engines[0].pitch`, `fsw_status.flags.enginesOn`, or `derived.tilt` for a quantity computed in `gncModel.js`); `js/gnc/paramFormat.js` resolves, converts (rad to deg, fraction to percent) and formats them, and a unit test checks that every parameter any live screen references exists in the generated dictionary tables and that chart parameters are stored columns. To add a regime, append an entry with its FSW modes (the mode to regime table is one place; it should later move into the dictionary mode enum) and its screens.

Modules in `js/gnc/`: pure `ccsds.js`, `gncPackets.js`, `geodesy.js`, `telemetryStore.js`, `gncModel.js`, `regimes.js`, `paramFormat.js`, `chartDefs.js`, `decimate.js` (unit tested); DOM-bound `widgets.js`, `charts.js`, `gimbalView.js`, `attitudeView.js`, `mapView.js`, `timelineView.js`, `canvasUtil.js`, `perfMonitor.js`, `gncMain.js`. None imports three. `js/wsClient.js` is the generic reconnecting client that `js/truthClient.js` wraps. Test hooks: `window.__gncInject(arrayBuffer)` feeds a ground frame, `window.__gncState()` returns the model values (computed from the store, whatever screen is shown) and the current regime and screen, `window.__gncCharts` lists the charts on screen and how often each chart id was drawn, `window.__gncPerf()` returns the perf monitor snapshot when enabled. uPlot 1.6.31 is vendored unmodified in `vendor/uplot/`.

## Tests

```bash
python -I web/tests/run_web_tests.py
```

Needs Python with `playwright` (Chromium or Edge). It serves `web/`, runs `tests/test.html` in a headless browser, cross-checks the JS decoder against a Python-packed 296 byte (protocol v3) snapshot, smoke-loads `index.html`, and records a 2 s clip with the built-in recorder (checks 1080p render size, 30 fps pacing and the saved file). The GNC checks cross-check a Python-packed navigation packet against the JS decoders, then load `/gnc/`, assert it never opens `/ws/truth` and never loads three.js, inject a synthetic hop for every APID through `__gncInject`, and check mode, fix, satellites, deltas, loss counting, run reset and staleness; screen routing by click, key and hash, that only the shown screen draws charts, reload restore, follow-phase paging (on and off), planned regimes, menu and popovers; and a perf check at 4x CPU throttle with live-rate packets.
