# Web truth view

Browser client for the sim server's truth stream (`docs/protocol/truth_stream.md`). Plain ES modules, no build step. three.js 0.180.0 is vendored in `vendor/three/` (unmodified, MIT, see `VERSION.txt`).

## Run

Start `sim_server` (it serves this directory and the WebSocket on one port, default 8080) and open `http://localhost:8080/`. The client connects to `/ws/truth` on the same host.

Controls: drag to orbit, wheel to zoom, `R` or "Reset view (R)" to reset the camera, "Restart" to start a new run.

The visual is a Ship-sized cylinder (9 m x 52 m); the bottom banner names the physics vehicle from the server. The HUD shows simulated truth, not telemetry.

## Recording

Use the built-in recorder instead of an OS screen recorder: pick a resolution (720p, 1080p, 1440p) and frame rate (30 or 60 fps), choose whether to include the HUD, and press Record; Stop saves `starship-run<N>-<time>.mp4` (or `.webm` where the browser cannot record MP4). While recording, the view renders at exactly the recording size and frame rate (letterboxed in the window), and each frame is composed with the HUD drawn into it, so the cost is bounded instead of full-window, full-DPI rendering at the display refresh plus an OS capture on top. Keep the tab visible: background tabs stop rendering, which freezes the recording. The video is held in memory until you stop (about 90 MB per minute at 1080p60).

## Layout

- `js/frames.js`: the only place sim ENU frames are converted to three.js frames.
- `js/protocol.js`, `js/interpolator.js`, `js/frames.js`: pure modules (no three, no DOM access), unit tested.
- `js/truthClient.js`: WebSocket with reconnect and backoff.
- `js/sceneBuilder.js`, `js/vehicleView.js`, `js/cameraRig.js`: the only modules importing three.
- `js/hudModel.js`: pure; the HUD's display strings, shared by the DOM HUD (`js/hud.js`) and the recorded frames (`js/hudCanvas.js`).
- `js/recorder.js`: built-in recorder (pure helpers for format choice, file name and frame pacing, plus the MediaRecorder part).
- `js/hud.js`, `js/main.js`, `css/hud.css`, `index.html`.

## Tests

```bash
python -I web/tests/run_web_tests.py
```

Needs Python with `playwright` (Chromium or Edge). It serves `web/`, runs `tests/test.html` in a headless browser, cross-checks the JS decoder against a Python-packed 176 byte snapshot, smoke-loads `index.html`, and records a 2 s clip with the built-in recorder (checks 1080p render size, 30 fps pacing and the saved file).
