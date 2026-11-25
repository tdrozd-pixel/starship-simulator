// Placeholder vehicle visual: a Ship-sized cylinder, not the physics shape.
// Draws six engines (3 SL, 3 RVac, engineLayout.js) from the ACTUAL per-engine actuator positions
// in the snapshot (throttle, gimbal, running), never commanded ones.
import * as THREE from 'three';
import { VISUAL } from './visualParams.js';
import { ENGINE_COUNT, isSeaLevel, enginePositionBody } from './engineLayout.js';
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

  // Six engines: 0..2 sea-level (gimbaled), 3..5 Raptor Vacuum (fixed). Layout from engineLayout.js.
  // Geometries and materials are shared or built once; apply() only sets transforms and opacities.
  const nozzleMat = new THREE.MeshLambertMaterial({ color: 0x555b60 });
  const rvacNozzleMat = new THREE.MeshLambertMaterial({ color: 0x2a2e32 }); // dark: never fired in the hop
  const mountMat = new THREE.MeshLambertMaterial({ color: 0x30363b });
  const slNozzleGeo = new THREE.CylinderGeometry(0.3, VISUAL.nozzleExitRadiusSl, 0.9, 24);
  slNozzleGeo.translate(0, -0.4, 0);
  const rvacNozzleGeo = new THREE.CylinderGeometry(0.45, VISUAL.nozzleExitRadiusRvac, 1.5, 24);
  rvacNozzleGeo.translate(0, -0.7, 0);
  const slMountGeo = new THREE.TorusGeometry(VISUAL.nozzleExitRadiusSl + 0.07, 0.1, 8, 24);
  slMountGeo.rotateX(Math.PI / 2);
  slMountGeo.translate(0, -0.05, 0);
  const rvacMountGeo = new THREE.TorusGeometry(VISUAL.nozzleExitRadiusRvac + 0.07, 0.12, 8, 24);
  rvacMountGeo.rotateX(Math.PI / 2);
  rvacMountGeo.translate(0, -0.05, 0);

  // Plume: unit-radius cone spanning y 0..-1, wide end at the engine; scaled per engine.
  const coneGeo = new THREE.ConeGeometry(1, 1, 24, 1, true);
  coneGeo.rotateX(Math.PI);
  coneGeo.translate(0, -0.5, 0);
  const innerGeo = new THREE.ConeGeometry(0.5, 0.7, 24, 1, true);
  innerGeo.rotateX(Math.PI);
  innerGeo.translate(0, -0.35, 0);
  const plumeMat = (color) => new THREE.MeshBasicMaterial({
    color, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending, side: THREE.DoubleSide });

  const engines = [];
  const pos = [0, 0, 0], loc = [0, 0, 0];
  for (let i = 0; i < ENGINE_COUNT; i++) {
    const sl = isSeaLevel(i);
    enginePositionBody(i, pos);
    bodyDirToThreeLocal(pos[0], pos[1], pos[2], loc); // a position on the base plane maps like a direction
    // The mount ring is fixed to the hull; the pivot (nozzle and plume) rotates with the gimbal.
    const mount = new THREE.Mesh(sl ? slMountGeo : rvacMountGeo, mountMat);
    mount.position.set(loc[0], loc[1], loc[2]);
    root.add(mount);
    const pivot = new THREE.Group();
    pivot.position.set(loc[0], loc[1], loc[2]);
    pivot.add(new THREE.Mesh(sl ? slNozzleGeo : rvacNozzleGeo, sl ? nozzleMat : rvacNozzleMat));
    const plume = new THREE.Group();
    const outerMat = plumeMat(0xffb060);
    const innerMat = plumeMat(0xfff0c0);
    plume.add(new THREE.Mesh(coneGeo, outerMat), new THREE.Mesh(innerGeo, innerMat));
    plume.visible = false;
    pivot.add(plume);
    root.add(pivot);
    engines.push({ pivot, plume, outerMat, innerMat, radius: sl ? VISUAL.plumeRadiusSl : VISUAL.plumeRadiusRvac });
  }

  // Catch pins: cylinders along body +/- y (pin 0 at +y, pin 1 at -y) at pinHeight above the base,
  // pinOffset from the axis. Built once from hello.tower (setTower); static, no per-frame work.
  const pinMat = new THREE.MeshLambertMaterial({ color: 0xd8a020 });
  let pinGroup = null, pinGeo = null;
  function setTower(tower) {
    if (pinGroup) { root.remove(pinGroup); pinGeo.dispose(); pinGroup = null; }
    if (!tower) return;
    const len = Math.max(0.6, tower.pinOffset - R); // outboard of the hull, [m]
    pinGeo = new THREE.CylinderGeometry(VISUAL.pinRadius, VISUAL.pinRadius, len, 12);
    pinGroup = new THREE.Group();
    const c = [0, 0, 0];
    for (const side of [1, -1]) {
      const m = new THREE.Mesh(pinGeo, pinMat);
      bodyDirToThreeLocal(0, side * (tower.pinOffset - len / 2), 0, c); // body +/- y: pin centre
      m.position.set(c[0], tower.pinHeight, c[2]);
      bodyDirToThreeLocal(0, side, 0, c); // cylinder axis (+Y) turned onto the pin direction
      m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(c[0], c[1], c[2]));
      pinGroup.add(m);
    }
    root.add(pinGroup);
  }

  const p3 = [0, 0, 0], q4 = [0, 0, 0, 0], eb = [0, 0, 0], dl = [0, 0, 0];
  const dir = new THREE.Vector3();
  const down = new THREE.Vector3(0, -1, 0);

  function apply(s) {
    root.visible = true;
    enuToThree(s.pos[0], s.pos[1], s.pos[2], p3);
    root.position.set(p3[0], p3[1], p3[2]);
    bodyQuatToThree(s.quat[0], s.quat[1], s.quat[2], s.quat[3], q4);
    root.quaternion.set(q4[0], q4[1], q4[2], q4[3]);

    for (let i = 0; i < ENGINE_COUNT; i++) {
      const e = engines[i];
      if (isSeaLevel(i)) {
        // Actual per-engine gimbal angles; the plume rides in the pivot so it follows the nozzle.
        exhaustDirBody(s.engineGimbalPitch[i], s.engineGimbalYaw[i], eb);
        bodyDirToThreeLocal(eb[0], eb[1], eb[2], dl);
        dir.set(dl[0], dl[1], dl[2]).normalize();
        e.pivot.quaternion.setFromUnitVectors(down, dir);
      }
      const thr = s.engineThrottle[i];
      const on = s.engineRun[i] && thr > 0.001;
      e.plume.visible = on;
      if (!on) continue;
      const t = Math.min(1, thr);
      e.plume.scale.set(e.radius, VISUAL.plumeMaxLength * (0.2 + 0.8 * t), e.radius);
      const op = VISUAL.plumeMinOpacity + (VISUAL.plumeMaxOpacity - VISUAL.plumeMinOpacity) * t;
      e.outerMat.opacity = op * 0.6;
      e.innerMat.opacity = op;
    }
  }

  function hide() { root.visible = false; }
  return { root, apply, hide, setTower };
}
