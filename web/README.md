# Web truth view

Browser client for the sim server's truth stream (`docs/protocol/truth_stream.md`). Plain ES modules, no build step. three.js 0.180.0 is vendored in `vendor/three/` (unmodified, MIT, see `VERSION.txt`).

## Run

Start `sim_server` (it serves this directory and the WebSocket on one port, default 8080) and open `http://localhost:8080/`. The client connects to `/ws/truth` on the same host.

Controls: drag to orbit, wheel to zoom, `R` or "Reset view (R)" to reset the camera, "Restart" to start a new run.

The visual is a Ship-sized placeholder cylinder, not the physics shape. The HUD shows simulated truth, not telemetry.

## Layout

- `js/frames.js`: the only place sim ENU frames are converted to three.js frames.
- `js/protocol.js`, `js/interpolator.js`, `js/frames.js`: pure modules (no three, no DOM access), unit tested.
- `js/truthClient.js`: WebSocket with reconnect and backoff.
- `js/sceneBuilder.js`, `js/vehicleView.js`, `js/cameraRig.js`: the only modules importing three.
- `js/hud.js`, `js/main.js`, `css/hud.css`, `index.html`.

## Tests

```bash
python -I web/tests/run_web_tests.py
```

Needs Python with `playwright` (Chromium or Edge). It serves `web/`, runs `tests/test.html` in a headless browser, cross-checks the JS decoder against a Python-packed 176 byte snapshot, and smoke-loads `index.html`.
