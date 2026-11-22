// Placeholder scene. TODO(phase 0, item 6): replace with real site geometry (Starbase pad, tower, terrain).
import * as THREE from 'three';
import { VISUAL } from './visualParams.js';
import { enuToThree } from './frames.js';

function makeLabel(text) {
  const c = document.createElement('canvas');
  c.width = 128; c.height = 48;
  const g = c.getContext('2d');
  g.fillStyle = 'rgba(0,0,0,0.55)';
  g.fillRect(0, 0, 128, 48);
  g.fillStyle = '#fff';
  g.font = 'bold 28px monospace';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(text, 64, 26);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
  spr.scale.set(8, 3, 1);
  spr.renderOrder = 10;
  return spr;
}

function buildPole(scene) {
  const e = [0, 0, 0];
  const pole = new THREE.Group();
  enuToThree(VISUAL.poleOffsetEast, 0, 0, e);
  pole.position.set(e[0], e[1], e[2]);
  const mat = new THREE.MeshLambertMaterial({ color: 0xf0f0f0 });
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, VISUAL.poleHeight, 12), mat);
  shaft.position.y = VISUAL.poleHeight / 2;
  pole.add(shaft);
  const tickMat = new THREE.MeshLambertMaterial({ color: 0xd03030 });
  for (let h = VISUAL.tickSpacing; h <= VISUAL.poleHeight + 1e-6; h += VISUAL.tickSpacing) {
    const major = Math.abs(h / 50 - Math.round(h / 50)) < 1e-9;
    const len = major ? 5 : 2.5;
    const tick = new THREE.Mesh(new THREE.BoxGeometry(len, 0.3, 0.3), tickMat);
    tick.position.set(len / 2, h, 0);
    pole.add(tick);
    if (major || h === VISUAL.tickSpacing) {
      const lab = makeLabel(h + ' m');
      lab.position.set(len + 4.5, h, 0);
      pole.add(lab);
    }
  }
  scene.add(pole);
}

export function buildScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x87b6e8);
  scene.fog = new THREE.Fog(0x87b6e8, 800, 3500);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x556b2f, 1.0));
  const sun = new THREE.DirectionalLight(0xffffff, 1.5);
  sun.position.set(100, 200, 50);
  scene.add(sun);

  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(4000, 4000),
    new THREE.MeshLambertMaterial({ color: 0x8a7458 }));
  ground.rotation.x = -Math.PI / 2;
  scene.add(ground);

  const grid = new THREE.GridHelper(1000, 100, 0x555555, 0x777777);
  grid.position.y = 0.05;
  scene.add(grid);

  const ring = new THREE.Mesh(
    new THREE.RingGeometry(VISUAL.padRadius - 0.5, VISUAL.padRadius, 64),
    new THREE.MeshBasicMaterial({ color: 0xffd040, side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2;
  ring.position.y = 0.02;
  scene.add(ring);
  const crossMat = new THREE.MeshBasicMaterial({ color: 0xffd040, side: THREE.DoubleSide });
  for (const rotY of [0, Math.PI / 2]) {
    const bar = new THREE.Mesh(new THREE.PlaneGeometry(6, 0.4), crossMat);
    bar.rotation.x = -Math.PI / 2;
    bar.rotation.z = rotY;
    bar.position.y = 0.03;
    scene.add(bar);
  }

  buildPole(scene);
  return scene;
}
