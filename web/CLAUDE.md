# web

Browser clients, plain ES modules, no build step. Served by `sim_server`. Module layout and test details: `web/README.md`. Rendering plan: `docs/architecture.md`.

- `js/frames.js` is the only place sim ENU frames convert to renderer frames (three.js is Y-up). Do all conversion there.
- `protocol.js`, `interpolator.js`, `frames.js` are pure (no three, no DOM) and unit tested. Keep them that way. Only `sceneBuilder.js`, `vehicleView.js`, `cameraRig.js` import three.
- `vendor/three/` is three.js 0.180.0, vendored unmodified. Never edit it.
- Clients talk to the sim only through documented protocols (`docs/protocol/`). The outside view and HUD may show truth; consoles (planned) must use downlinked telemetry only.
- Show time from the stream's sim time, never the browser clock. Interpolate between the two latest samples; the stream is 100 Hz.
- Animate actual actuator positions from the stream, never commanded ones.
- Must run on an ordinary laptop over Wi-Fi.
- HUD text lives in `hudModel.js` only: the DOM HUD and the recorder's canvas HUD (`hudCanvas.js`) both render it, so a recorded frame always matches the screen. Recorder controls and the REC indicator are DOM only and never appear in the video.
- Tests: `python -I web/tests/run_web_tests.py`.
