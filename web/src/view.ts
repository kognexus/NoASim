/**
 * Draws one arm with three.js, straight from the MuJoCo model: every geom becomes a mesh
 * whose pose is copied from the simulation each frame. Joints are drawn as discs on
 * their axes, coloured by what the joint reports, and can be clicked to select them.
 */
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { GeomInfo, Plant } from "../../sim/src/plant/plant.ts";
import { type JointState, jointState } from "./status.ts";

/** MuJoCo geom type codes (`mjtGeom`). */
const GEOM = { plane: 0, sphere: 2, capsule: 3, cylinder: 5, box: 6 } as const;

/** Disc colours by joint state (the CSS custom properties of the page match them). */
const JOINT_COLOURS: Record<JointState, number> = {
  ready: 0x3f8f6b,
  idle: 0x8a8f98,
  network: 0x2f6fd6,
  holding: 0xd99a22,
  fault: 0xd64545,
  torqueOff: 0x222222,
  commissioning: 0x8a5cd6,
};

function geomMesh(info: GeomInfo, material: THREE.Material): THREE.Object3D | undefined {
  const [a, b, c] = info.size;
  let geometry: THREE.BufferGeometry;
  switch (info.type) {
    case GEOM.sphere:
      geometry = new THREE.SphereGeometry(a, 24, 16);
      break;
    case GEOM.capsule:
      // MuJoCo capsules lie along local z; three.js builds them along y.
      geometry = new THREE.CapsuleGeometry(a, 2 * b, 8, 20).rotateX(Math.PI / 2);
      break;
    case GEOM.cylinder:
      geometry = new THREE.CylinderGeometry(a, a, 2 * b, 24).rotateX(Math.PI / 2);
      break;
    case GEOM.box:
      geometry = new THREE.BoxGeometry(2 * a, 2 * b, 2 * c);
      break;
    default:
      return undefined; // the floor is drawn as a grid instead
  }
  const mesh = new THREE.Mesh(geometry, material);
  mesh.matrixAutoUpdate = false;
  return mesh;
}

export class ArmView {
  readonly #renderer: THREE.WebGLRenderer;
  readonly #scene = new THREE.Scene();
  readonly #camera: THREE.PerspectiveCamera;
  readonly #controls: OrbitControls;
  readonly #geoms: { index: number; object: THREE.Object3D }[] = [];
  readonly #discs: THREE.Mesh<THREE.CylinderGeometry, THREE.MeshStandardMaterial>[] = [];
  readonly #canvas: HTMLCanvasElement;
  #plant: Plant;
  #selected = -1;
  /** Called with the joint index (0-based) the user clicked, or −1. */
  onSelect: (joint: number) => void = () => undefined;

  constructor(canvas: HTMLCanvasElement, plant: Plant) {
    this.#canvas = canvas;
    this.#plant = plant;
    this.#renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.#renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
    this.#camera = new THREE.PerspectiveCamera(40, 1, 0.01, 20);
    this.#camera.up.set(0, 0, 1);
    this.#camera.position.set(1.45, -1.55, 1.0);
    this.#controls = new OrbitControls(this.#camera, canvas);
    this.#controls.target.set(0.28, 0, 0.3);
    this.#controls.update();
    this.#scene.background = new THREE.Color(
      getComputedStyle(canvas).getPropertyValue("--surface").trim() || "#ffffff",
    );
    this.#scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f98, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(1, -2, 3);
    this.#scene.add(sun);
    const gridColour = getComputedStyle(canvas).getPropertyValue("--grid").trim() || "#e4e7eb";
    const grid = new THREE.GridHelper(2, 20, gridColour, gridColour);
    grid.rotateX(Math.PI / 2);
    this.#scene.add(grid);
    this.#build();
    canvas.addEventListener("pointerdown", (e) => this.#pick(e));
    this.resize();
  }

  /** Points the view at another plant (after a rebuild). */
  attach(plant: Plant): void {
    for (const { object } of this.#geoms) this.#scene.remove(object);
    for (const disc of this.#discs) this.#scene.remove(disc);
    this.#geoms.length = 0;
    this.#discs.length = 0;
    this.#plant = plant;
    this.#build();
  }

  #build(): void {
    this.#plant.geoms().forEach((info, index) => {
      const [r, g, b, alpha] = info.rgba;
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(r, g, b),
        roughness: 0.55,
        metalness: 0.1,
        transparent: alpha < 1,
        opacity: alpha,
      });
      const object = geomMesh(info, material);
      if (object) {
        this.#scene.add(object);
        this.#geoms.push({ index, object });
      }
    });
    for (let j = 0; j < this.#plant.jointCount; j++) {
      const disc = new THREE.Mesh(
        new THREE.CylinderGeometry(0.05, 0.05, 0.07, 32),
        new THREE.MeshStandardMaterial({ color: JOINT_COLOURS.idle, roughness: 0.4 }),
      );
      disc.userData["joint"] = j;
      this.#scene.add(disc);
      this.#discs.push(disc);
    }
  }

  /** Marks joint `joint` as selected (−1 for none). */
  select(joint: number): void {
    this.#selected = joint;
  }

  resize(): void {
    const { clientWidth: w, clientHeight: h } = this.#canvas;
    if (w === 0 || h === 0) return;
    this.#renderer.setSize(w, h, false);
    this.#camera.aspect = w / h;
    this.#camera.updateProjectionMatrix();
  }

  /** Copies the simulation's poses and the joints' reported status, then draws. */
  render(status: ReadonlyMap<number, number>): void {
    const { positions: p, rotations: r } = this.#plant.geomPoses();
    for (const { index: g, object } of this.#geoms) {
      const m = (k: number) => r[9 * g + k] ?? 0;
      // prettier-ignore
      object.matrix.set(
        m(0), m(1), m(2), p[3 * g] ?? 0,
        m(3), m(4), m(5), p[3 * g + 1] ?? 0,
        m(6), m(7), m(8), p[3 * g + 2] ?? 0,
        0, 0, 0, 1,
      );
    }
    const up = new THREE.Vector3(0, 1, 0);
    this.#plant.jointAnchors().forEach((anchor, j) => {
      const disc = this.#discs[j];
      if (!disc) return;
      disc.position.set(...anchor.position);
      disc.quaternion.setFromUnitVectors(up, new THREE.Vector3(...anchor.axis));
      disc.material.color.setHex(JOINT_COLOURS[jointState(status.get(j + 1))]);
      disc.material.emissive.setHex(j === this.#selected ? 0x555555 : 0x000000);
      disc.scale.setScalar(j === this.#selected ? 1.25 : 1);
    });
    this.#renderer.render(this.#scene, this.#camera);
  }

  #pick(event: PointerEvent): void {
    const rect = this.#canvas.getBoundingClientRect();
    const pointer = new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
    const ray = new THREE.Raycaster();
    ray.setFromCamera(pointer, this.#camera);
    const hit = ray.intersectObjects(this.#discs)[0];
    if (hit) this.onSelect(Number(hit.object.userData["joint"]));
  }

  dispose(): void {
    this.#controls.dispose();
    this.#renderer.dispose();
  }
}
