import * as THREE from 'three';
import { VISUAL } from './visualParams.js';
import { createInterpolator } from './interpolator.js';
import { createTruthClient } from './truthClient.js';
import { buildScene } from './sceneBuilder.js';
import { createVehicleView } from './vehicleView.js';
import { createCameraRig } from './cameraRig.js';
import { createHud } from './hud.js';

const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
const scene = buildScene();
const vehicle = createVehicleView();
scene.add(vehicle.root);
const rig = createCameraRig(renderer, canvas);

const interp = createInterpolator({
  delaySnapshots: VISUAL.interpDelaySnapshots,
  bufferSeconds: VISUAL.bufferSeconds,
  offsetWindowSeconds: VISUAL.offsetWindowSeconds,
});

let client = null;
const hud = createHud({
  onRestart: () => client && client.sendRestart(),
  onResetView: () => rig.reset(),
});

const url = (location.protocol === 'https:' ? 'wss' : 'ws') + '://' + location.host + '/ws/truth';
client = createTruthClient({
  url,
  onHello: (h) => {
    hud.setHello(h);
    interp.setTimeScale(h.timeScale);
    interp.setSnapshotRate(h.snapshotRateHz);
    interp.reset(h.runId);
  },
  onRunStarted: (m) => { interp.reset(m.runId); hud.setRun(m.runId); },
  onSnapshot: (s) => interp.push(s, performance.now() / 1000),
  onState: (s, extra) => hud.setConnection(s, extra),
  onProtocolError: (r) => hud.setError(r),
});
hud.setStaleSource(() => client.secondsSinceMessage());
client.connect();

let last = performance.now();
function frame(now) {
  const dt = Math.min(0.1, (now - last) / 1000);
  last = now;
  const s = interp.sample(now / 1000);
  if (s) {
    vehicle.apply(s);
    rig.update(vehicle.root.position, dt);
  } else {
    vehicle.hide();
    rig.update(null, dt);
  }
  hud.update(s, now);
  renderer.render(scene, rig.camera);
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
