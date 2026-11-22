import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { VISUAL } from './visualParams.js';
import { enuToThree } from './frames.js';

export function createCameraRig(renderer, canvas) {
  const camera = new THREE.PerspectiveCamera(VISUAL.fovDeg, 1, VISUAL.near, VISUAL.far);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.maxPolarAngle = Math.PI * 0.495;
  controls.minDistance = 5;
  controls.maxDistance = 1500;

  const off = enuToThree(VISUAL.defaultCamPos[0], VISUAL.defaultCamPos[1], VISUAL.defaultCamPos[2], [0, 0, 0]);
  const defaultOffset = new THREE.Vector3(off[0], off[1], off[2]);
  const desired = new THREE.Vector3();
  const delta = new THREE.Vector3();

  function reset() {
    controls.target.set(0, VISUAL.shipHeight / 2, 0);
    camera.position.copy(controls.target).add(defaultOffset);
    controls.update();
  }

  // Follow the vehicle while preserving the user's orbit offset.
  function update(vehiclePos, dt) {
    if (vehiclePos) {
      desired.set(vehiclePos.x, vehiclePos.y + VISUAL.shipHeight / 2, vehiclePos.z);
      const k = 1 - Math.exp(-dt * 6);
      delta.copy(desired).sub(controls.target).multiplyScalar(k);
      controls.target.add(delta);
      camera.position.add(delta);
    }
    controls.update();
  }

  // While recording, render at exactly the recording size (pixel ratio 1) whatever the
  // window and display density, so the GPU cost is bounded; CSS letterboxes it.
  let fixed = null;

  function resize() {
    if (fixed) {
      renderer.setPixelRatio(1);
      renderer.setSize(fixed.w, fixed.h, false);
      camera.aspect = fixed.w / fixed.h;
    } else {
      const w = Math.max(1, window.innerWidth), h = Math.max(1, window.innerHeight);
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, VISUAL.pixelRatioCap));
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
    }
    camera.updateProjectionMatrix();
  }

  function setFixedSize(w, h) {
    fixed = { w, h };
    canvas.classList.add('fixed-size');
    resize();
  }

  function clearFixedSize() {
    fixed = null;
    canvas.classList.remove('fixed-size');
    resize();
  }

  window.addEventListener('resize', resize);
  window.addEventListener('keydown', (e) => {
    const t = e.target;
    if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.isContentEditable)) return;
    if (e.key === 'r' || e.key === 'R') reset();
  });
  resize();
  reset();
  return { camera, controls, update, reset, resize, setFixedSize, clearFixedSize };
}
