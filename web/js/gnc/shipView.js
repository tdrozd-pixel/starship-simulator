// GNC vehicle view: the Ship at the ESTIMATED attitude, nozzles at the MEASURED gimbal with a
// ghost at the COMMANDED gimbal. Imports three. All sim-to-three conversion goes through frames.js.
import * as THREE from 'three';
import { bodyQuatToThree, bodyDirToThreeLocal, exhaustDirBody } from '../frames.js';
import { NUM_ENGINES, enginePosition } from './gncPackets.js';

const SHIP_LENGTH = 52; // [m] Ship height (docs/vehicle_reference.md)
const SHIP_RADIUS = 4.5; // [m]
const NOZZLE_LENGTH = 2.4; // [m] drawn length, a display choice
const NOZZLE_RADIUS = 0.7; // [m] drawn exit radius, a display choice
const MEASURED = 0x4ccf6a, INVALID = 0xf0b030, GHOST = 0xff5bd6;

export function createShipView(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  renderer.setClearColor(0x0b0f16, 1);
  renderer.autoClear = false;
  const scene = new THREE.Scene();

  // Ship group: local axes are M * body (frames.js), origin at mid-length, base at local -Y.
  const ship = new THREE.Group();
  ship.quaternion.set(0, 0, 0, 1);
  scene.add(ship);
  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(SHIP_RADIUS, SHIP_RADIUS, SHIP_LENGTH, 28),
    new THREE.MeshBasicMaterial({ color: 0x1b2430 }));
  ship.add(body);
  const edges = new THREE.LineSegments(
    new THREE.EdgesGeometry(body.geometry, 20), new THREE.LineBasicMaterial({ color: 0x4a5a70 }));
  ship.add(edges);
  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(SHIP_RADIUS, 6, 28), new THREE.MeshBasicMaterial({ color: 0x2c3a4e }));
  nose.position.y = SHIP_LENGTH / 2 + 3;
  ship.add(nose);
  // Body axes: x red, y green, z (nose) blue, in body terms via bodyDirToThreeLocal.
  const tmp3 = [0, 0, 0];
  const axes = [[1, 0, 0, 0xff5050], [0, 1, 0, 0x50ff50], [0, 0, 1, 0x5090ff]];
  for (const [x, y, z, col] of axes) {
    bodyDirToThreeLocal(x, y, z, tmp3);
    const len = z === 1 ? 14 : 9;
    const g = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0), new THREE.Vector3(tmp3[0] * len, tmp3[1] * len, tmp3[2] * len)]);
    ship.add(new THREE.Line(g, new THREE.LineBasicMaterial({ color: col })));
  }
  // World up reference through the ship origin.
  scene.add(new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, -40, 0), new THREE.Vector3(0, 40, 0)]),
    new THREE.LineDashedMaterial({ color: 0x556677, dashSize: 1, gapSize: 1 })).computeLineDistances());

  // Nozzles: pivot at the engine mount (base), cone apex at the pivot, exhaust along local -Y at zero gimbal.
  const nozzleGeo = new THREE.ConeGeometry(NOZZLE_RADIUS, NOZZLE_LENGTH, 16, 1, true);
  nozzleGeo.translate(0, -NOZZLE_LENGTH / 2, 0);
  const measuredMat = [], ghostMat = [];
  const measured = [], ghost = [];
  for (let i = 0; i < NUM_ENGINES; i++) {
    const [ex, ey] = enginePosition(i);
    bodyDirToThreeLocal(ex, ey, 0, tmp3);
    const mm = new THREE.MeshBasicMaterial({ color: MEASURED, side: THREE.DoubleSide });
    const gm = new THREE.MeshBasicMaterial({
      color: GHOST, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false });
    measuredMat.push(mm); ghostMat.push(gm);
    const mp = new THREE.Group(), gp = new THREE.Group();
    mp.position.set(tmp3[0], -SHIP_LENGTH / 2 + tmp3[1], tmp3[2]);
    gp.position.copy(mp.position);
    mp.add(new THREE.Mesh(nozzleGeo, mm));
    const gmesh = new THREE.Mesh(nozzleGeo, gm);
    gmesh.scale.set(1.15, 1.15, 1.15);
    gp.add(gmesh);
    ship.add(mp, gp);
    measured.push(mp); ghost.push(gp);
  }

  const mainCam = new THREE.PerspectiveCamera(30, 1, 1, 500);
  mainCam.position.set(0, 4, 120);
  mainCam.lookAt(0, 0, 0);
  // Base camera is a child of the ship so the nozzles stay framed: looks at the base from the side and below.
  const baseCam = new THREE.PerspectiveCamera(35, 1, 0.2, 100);
  baseCam.position.set(0, -SHIP_LENGTH / 2 - 5.5, 12);
  baseCam.lookAt(0, -SHIP_LENGTH / 2 - 1.2, 0);
  ship.add(baseCam);

  const up = new THREE.Vector3(0, -1, 0), dir = new THREE.Vector3(), q4 = [0, 0, 0, 1], d3 = [0, 0, 0];
  function aim(group, pitch, yaw) {
    exhaustDirBody(pitch, yaw, d3);
    bodyDirToThreeLocal(d3[0], d3[1], d3[2], tmp3);
    dir.set(tmp3[0], tmp3[1], tmp3[2]);
    group.quaternion.setFromUnitVectors(up, dir);
  }

  let w = 0, h = 0;
  function resize() {
    const cw = Math.max(1, canvas.clientWidth), ch = Math.max(1, canvas.clientHeight);
    if (cw === w && ch === h) return;
    w = cw; h = ch;
    renderer.setSize(w, h, false);
    mainCam.aspect = w / h;
    mainCam.updateProjectionMatrix();
  }

  // attitude: [w,x,y,z] estimated, or null; cmd/fb: arrays of {pitch,yaw,valid} per engine (rad) or null.
  function render(attitude, cmd, fb) {
    resize();
    if (attitude) {
      bodyQuatToThree(attitude[0], attitude[1], attitude[2], attitude[3], q4);
      ship.quaternion.set(q4[0], q4[1], q4[2], q4[3]).normalize();
    }
    for (let i = 0; i < NUM_ENGINES; i++) {
      measured[i].visible = !!fb;
      ghost[i].visible = !!cmd;
      if (fb) {
        aim(measured[i], fb.engines[i].pitch, fb.engines[i].yaw);
        measuredMat[i].color.setHex(fb.engines[i].valid ? MEASURED : INVALID);
      }
      if (cmd) aim(ghost[i], cmd.engines[i].pitch, cmd.engines[i].yaw);
    }
    ship.updateMatrixWorld(true);
    renderer.setScissorTest(true);
    renderer.setViewport(0, 0, w, h);
    renderer.setScissor(0, 0, w, h);
    renderer.clear();
    renderer.render(scene, mainCam);
    // Inset: nozzle close-up, bottom-right.
    const iw = Math.floor(w * 0.42), ih = Math.floor(h * 0.42);
    const ix = w - iw - 6, iy = 6;
    baseCam.aspect = iw / ih;
    baseCam.updateProjectionMatrix();
    renderer.setViewport(ix, iy, iw, ih);
    renderer.setScissor(ix, iy, iw, ih);
    renderer.setClearColor(0x131a26, 1);
    renderer.clear();
    renderer.render(scene, baseCam);
    renderer.setClearColor(0x0b0f16, 1);
  }

  return { render, dispose() { renderer.dispose(); } };
}
