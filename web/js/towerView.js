// Catch tower visual: a static mast plus two arms that rotate about their hinges by the ACTUAL arm
// angles from the snapshot. Geometry comes from hello.tower (build once); apply() only sets arm
// rotations (no allocation). Pure arm math lives in towerGeometry.js; frames via frames.js.
import * as THREE from 'three';
import { enuToThree, enuHeadingToThreeYaw } from './frames.js';
import { hingePosition, armHeading } from './towerGeometry.js';

export function createTowerView() {
  const root = new THREE.Group();
  root.visible = false;
  const mastMat = new THREE.MeshLambertMaterial({ color: 0x9aa0a6 });
  const armMat = new THREE.MeshLambertMaterial({ color: 0xc84a2a });
  let tower = null;
  let geos = [];
  const arms = [];

  function build(t) {
    for (const g of geos) g.dispose();
    geos = [];
    arms.length = 0;
    root.clear();
    tower = t;
    if (!t) { root.visible = false; return; }
    const p = [0, 0, 0];
    const mastGeo = new THREE.BoxGeometry(2 * t.mastHalfWidth, t.mastHeight, 2 * t.mastHalfWidth);
    // The mast is square, so its ENU axes need no orientation.
    const mast = new THREE.Mesh(mastGeo, mastMat);
    enuToThree(t.position[0], t.position[1], t.mastHeight / 2, p);
    mast.position.set(p[0], p[1], p[2]);
    root.add(mast);
    // Arm box along local +X from the hinge; its top surface is railHeight above the origin plane.
    const armGeo = new THREE.BoxGeometry(t.armLength, t.armThickness, 2 * t.armHalfWidth);
    armGeo.translate(t.armLength / 2, t.railHeight - t.armThickness / 2, 0);
    geos.push(mastGeo, armGeo);
    const h = [0, 0];
    for (let k = 0; k < 2; k++) {
      const g = new THREE.Group();
      hingePosition(t, k, h);
      enuToThree(h[0], h[1], 0, p);
      g.position.set(p[0], p[1], p[2]);
      g.add(new THREE.Mesh(armGeo, armMat));
      root.add(g);
      arms.push(g);
    }
    root.visible = true;
  }

  function apply(s) {
    if (!tower) return;
    for (let k = 0; k < 2; k++) {
      arms[k].rotation.y = enuHeadingToThreeYaw(armHeading(tower, k, s.armAngle[k]));
    }
  }

  return { root, build, apply };
}
