// Placeholder vehicle visual: a Ship-sized cylinder, not the physics shape.
// Uses ACTUAL actuator positions from the snapshot (throttle, gimbal), never commanded ones.
import * as THREE from 'three';
import { VISUAL } from './visualParams.js';
import { enuToThree, bodyQuatToThree, bodyDirToThreeLocal, exhaustDirBody } from './frames.js';

export function createVehicleView() {
  const R = VISUAL.shipDiameter / 2;
  const H = VISUAL.shipHeight;
  const root = new THREE.Group(); // origin = cylinder base on the centerline
  root.visible = false;

  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(R, R, H, 32),
    new THREE.MeshLambertMaterial({ color: 0xc8ccd0 }));
  body.position.y = H / 2;
  root.add(body);

  // Dark stripe so roll is visible.
  const stripe = new THREE.Mesh(
    new THREE.BoxGeometry(1.2, H * 0.9, 0.2),
    new THREE.MeshLambertMaterial({ color: 0x202428 }));
  stripe.position.set(0, H / 2, R + 0.05);
  root.add(stripe);

  const nozzlePivot = new THREE.Group();
  const nozzle = new THREE.Mesh(
    new THREE.CylinderGeometry(0.3, 0.65, 0.9, 24),
    new THREE.MeshLambertMaterial({ color: 0x555b60 }));
  nozzle.position.y = -0.4;
  nozzlePivot.add(nozzle);
  root.add(nozzlePivot);

  const engineMount = new THREE.Mesh(
    new THREE.TorusGeometry(0.72, 0.1, 8, 24),
    new THREE.MeshLambertMaterial({ color: 0x30363b }));
  engineMount.rotation.x = Math.PI / 2;
  engineMount.position.y = -0.05;
  root.add(engineMount);

  const pitchIndicator = new THREE.ArrowHelper(
    new THREE.Vector3(0, -1, 0), new THREE.Vector3(-0.85, 0, 0), 1.8, 0x35d7ff, 0.35, 0.18);
  const yawIndicator = new THREE.ArrowHelper(
    new THREE.Vector3(0, -1, 0), new THREE.Vector3(0.85, 0, 0), 1.8, 0xff5bd6, 0.35, 0.18);
  root.add(pitchIndicator, yawIndicator);

  // Plume: unit cone spanning y 0..-1, wide end at the engine.
  const plume = new THREE.Group();
  const coneGeo = new THREE.ConeGeometry(VISUAL.plumeRadius, 1, 24, 1, true);
  coneGeo.rotateX(Math.PI);
  coneGeo.translate(0, -0.5, 0);
  const outerMat = new THREE.MeshBasicMaterial({
    color: 0xffb060, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide });
  plume.add(new THREE.Mesh(coneGeo, outerMat));
  const innerGeo = new THREE.ConeGeometry(VISUAL.plumeRadius * 0.5, 0.7, 24, 1, true);
  innerGeo.rotateX(Math.PI);
  innerGeo.translate(0, -0.35, 0);
  const innerMat = new THREE.MeshBasicMaterial({
    color: 0xfff0c0, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide });
  plume.add(new THREE.Mesh(innerGeo, innerMat));
  plume.visible = false;
  root.add(plume);

  const p3 = [0, 0, 0], q4 = [0, 0, 0, 0], eb = [0, 0, 0], dl = [0, 0, 0];
  const dir = new THREE.Vector3();
  const down = new THREE.Vector3(0, -1, 0);

  function apply(s) {
    root.visible = true;
    enuToThree(s.pos[0], s.pos[1], s.pos[2], p3);
    root.position.set(p3[0], p3[1], p3[2]);
    bodyQuatToThree(s.quat[0], s.quat[1], s.quat[2], s.quat[3], q4);
    root.quaternion.set(q4[0], q4[1], q4[2], q4[3]);

    exhaustDirBody(s.gimbalPitch, s.gimbalYaw, eb);
    bodyDirToThreeLocal(eb[0], eb[1], eb[2], dl);
    dir.set(dl[0], dl[1], dl[2]).normalize();
    plume.quaternion.setFromUnitVectors(down, dir);
    nozzlePivot.quaternion.copy(plume.quaternion);

    exhaustDirBody(s.gimbalPitch, 0, eb);
    bodyDirToThreeLocal(eb[0], eb[1], eb[2], dl);
    pitchIndicator.setDirection(dir.set(dl[0], dl[1], dl[2]).normalize());
    exhaustDirBody(0, s.gimbalYaw, eb);
    bodyDirToThreeLocal(eb[0], eb[1], eb[2], dl);
    yawIndicator.setDirection(dir.set(dl[0], dl[1], dl[2]).normalize());

    const on = s.engineRunning && s.throttle > 0.001;
    plume.visible = on;
    if (!on) return;
    plume.scale.set(1, VISUAL.plumeMaxLength * (0.2 + 0.8 * s.throttle), 1);
    const op = VISUAL.plumeMinOpacity + (VISUAL.plumeMaxOpacity - VISUAL.plumeMinOpacity) * Math.min(1, s.throttle);
    outerMat.opacity = op * 0.6;
    innerMat.opacity = op;
  }

  function hide() { root.visible = false; }
  return { root, apply, hide };
}
