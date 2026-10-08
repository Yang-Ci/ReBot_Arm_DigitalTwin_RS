(function (root) {
  'use strict';
  const DEG = Math.PI / 180;
  const CENTERS = [0, 84.5, -99.5, 5, 0, 0, 135];
  // Official LD display directions, independent of the ROS follower mapping.
  const DISPLAY_DIRECTIONS = [1, 1, 1, -1, 1, 1];
  function mapLeaderModelAngles(values) {
    if (!Array.isArray(values) || values.length !== 7 || !values.every(Number.isFinite)) return null;
    const angles = values.map((v, i) => ((v - CENTERS[i] + 180) % 360 + 360) % 360 - 180 + CENTERS[i]);
    const joints = {};
    for (let i = 0; i < 6; i++) joints['joint' + (i + 1)] = angles[i] * DEG * DISPLAY_DIRECTIONS[i];
    joints.joint7_left = angles[6] * DEG;
    joints.joint7_right = -joints.joint7_left;
    return joints;
  }
  if (typeof module !== 'undefined' && module.exports) module.exports = { mapLeaderModelAngles };
  if (!root?.document) return;

  class LeaderModelPreview {
    constructor(element) {
      this.element = element;
      this.host = element.querySelector('.leader-model-host');
      this.note = element.querySelector('.leader-model-note');
      this.robot = null;
      this.loaded = false;
      this.loading = false;
      this.failed = false;
      this.connected = false;
      this.sampleFresh = false;
      this.joints = null;
      this.statusSeenAt = 0;
      this.drawPending = false;
      this.language = () => root.rebotI18n?.getLang() === 'en' ? 1 : 0;
      element.querySelector('.leader-model-reset').addEventListener('click', () => this.resetView());
    }
    update(status, fresh, visible) {
      if (status !== this.lastStatus) { this.lastStatus = status; this.statusSeenAt = Date.now(); }
      this.connected = Boolean(status?.port) && status.state !== 'DISCONNECTED';
      this.element.hidden = !this.connected || !visible;
      const sampleAge = Number(status?.sample_age) + (Date.now() - this.statusSeenAt) / 1000;
      this.sampleFresh = Boolean(fresh && Number.isFinite(status?.sample_age) && status.sample_age >= 0 && sampleAge < 0.3);
      const joints = this.sampleFresh ? mapLeaderModelAngles(status.angles_deg) : null;
      this.sampleFresh = Boolean(joints);
      if (joints) { this.joints = joints; this.applyPose(); }
      if (this.connected && visible && !this.loading && !this.loaded && !this.failed) this.load();
      const lang = this.language();
      this.element.querySelector('.leader-model-title').textContent = lang ? 'Arm102 Leader · live pose' : 'Arm102 Leader · 实测姿态';
      this.element.querySelector('.leader-model-reset').textContent = lang ? 'Reset view' : '重置视角';
      this.note.textContent = this.failed
        ? (lang ? 'Model could not load. Check local files; reconnect to retry.' : '模型加载失败，请检查本地文件，重新连接后重试。')
        : !this.loaded ? (lang ? 'Loading official LD model…' : '正在加载官方 LD 模型…')
        : this.sampleFresh ? (lang ? 'Drag to orbit · scroll to zoom · raw leader angles' : '拖动旋转 · 滚轮缩放 · 主臂原始角度')
        : (lang ? 'Sample stale · holding last displayed pose' : '样本过期 · 已冻结最后显示姿态');
      this.element.classList.toggle('is-stale', !this.sampleFresh);
      if (!this.connected) { this.joints = null; this.failed = false; }
      this.draw();
    }
    load() {
      this.loading = true;
      try {
        if (!this.renderer) this.setupScene();
        const manager = new THREE.LoadingManager();
        let meshFailed = false;
        manager.onError = () => { meshFailed = true; };
        manager.onLoad = () => {
          this.loading = false;
          if (meshFailed || !this.robot) { this.failed = true; return; }
          this.loaded = true;
          this.robot.traverse((child) => {
            if (!child.isMesh) return;
            let link = child;
            while (link && !link.isURDFLink) link = link.parent;
            child.material = new THREE.MeshStandardMaterial({ color: link?.name === 'base_link' ? 0x4a5550 : 0xb5d0bf,
              metalness: 0.16, roughness: 0.48, side: THREE.DoubleSide });
            child.material.color.convertSRGBToLinear();
          });
          Object.values(this.robot.joints).forEach((joint) => { joint.ignoreLimits = true; });
          this.applyPose(); this.resetView();
        };
        const loader = new URDFLoader(manager);
        loader.load(new URL('models/leader-arm102/urdf/leader.urdf', document.baseURI).href, (robot) => {
          if (this.robot) this.frame.remove(this.robot);
          this.robot = robot;
          this.frame.add(robot);
        }, undefined, () => { this.loading = false; this.failed = true; });
      } catch (error) {
        this.loading = false; this.failed = true;
        console.warn('Leader model preview unavailable:', error);
      }
    }
    setupScene() {
      this.scene = new THREE.Scene();
      this.scene.background = new THREE.Color(0x161d19);
      this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 5);
      this.renderer = new THREE.WebGLRenderer({ antialias: true });
      this.renderer.setPixelRatio(Math.min(root.devicePixelRatio || 1, 2));
      this.renderer.outputEncoding = THREE.sRGBEncoding;
      this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
      this.host.appendChild(this.renderer.domElement);
      this.renderer.domElement.setAttribute('aria-label', 'Arm102 Leader 3D pose');
      this.renderer.domElement.tabIndex = 0;
      this.scene.add(new THREE.HemisphereLight(0xf1fff6, 0x3c4941, 0.8));
      const light = new THREE.DirectionalLight(0xffffff, 1.1);
      light.position.set(1, 1.7, 1.2); this.scene.add(light);
      this.frame = new THREE.Group(); this.frame.rotation.x = -Math.PI / 2; this.scene.add(this.frame);
      this.scene.add(new THREE.GridHelper(0.8, 16, 0x536b5d, 0x2b3e32));
      this.target = new THREE.Vector3(0, 0.18, 0);
      this.orbit = new THREE.Spherical(0.8, 1.15, 0.75);
      const canvas = this.renderer.domElement;
      const pointers = new Map();
      let pinchDistance = 0;
      canvas.addEventListener('pointerdown', (event) => {
        if (event.pointerType === 'mouse' && event.button !== 0) return;
        canvas.setPointerCapture(event.pointerId);
        pointers.set(event.pointerId, [event.clientX, event.clientY]); pinchDistance = 0;
      });
      canvas.addEventListener('pointermove', (event) => {
        const old = pointers.get(event.pointerId);
        if (!old) return;
        pointers.set(event.pointerId, [event.clientX, event.clientY]);
        if (pointers.size === 1) {
          this.orbit.theta -= (event.clientX - old[0]) * 0.008;
          this.orbit.phi = Math.max(0.12, Math.min(Math.PI - 0.12, this.orbit.phi - (event.clientY - old[1]) * 0.008));
        } else {
          const [a, b] = Array.from(pointers.values());
          const distance = Math.hypot(a[0]-b[0], a[1]-b[1]);
          if (pinchDistance && distance) this.orbit.radius = Math.max(0.2, Math.min(2, this.orbit.radius * pinchDistance / distance));
          pinchDistance = distance;
        }
        this.draw();
      });
      const release = (event) => { pointers.delete(event.pointerId); pinchDistance = 0; };
      canvas.addEventListener('pointerup', release);
      canvas.addEventListener('pointercancel', release);
      canvas.addEventListener('lostpointercapture', release);
      canvas.addEventListener('wheel', (event) => {
        event.preventDefault();
        this.orbit.radius = Math.max(0.2, Math.min(2, this.orbit.radius * (event.deltaY > 0 ? 1.1 : 0.9))); this.draw();
      }, { passive: false });
      canvas.addEventListener('keydown', (event) => {
        const changes = { ArrowLeft: [-0.12, 0], ArrowRight: [0.12, 0], ArrowUp: [0, -0.12], ArrowDown: [0, 0.12] };
        if (!changes[event.key]) return;
        event.preventDefault(); this.orbit.theta += changes[event.key][0];
        this.orbit.phi = Math.max(0.12, Math.min(Math.PI - 0.12, this.orbit.phi + changes[event.key][1])); this.draw();
      });
      this.resizeObserver = new ResizeObserver(() => this.draw()); this.resizeObserver.observe(this.host);
    }
    applyPose() {
      if (!this.robot || !this.joints) return;
      // Right handle follows the URDF mimic. This path never sends ROS commands.
      for (const [name, value] of Object.entries(this.joints)) {
        if (name !== 'joint7_right') this.robot.setJointValue(name, value);
      }
      this.robot.updateMatrixWorld(true);
    }
    resetView() {
      if (!this.orbit) return;
      if (this.robot) {
        this.robot.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(this.robot); box.getCenter(this.target);
        this.viewRadius = Math.max(0.45, box.getSize(new THREE.Vector3()).length() * 1.6);
      }
      this.orbit.set(this.viewRadius || 0.8, 1.15, 0.75); this.draw();
    }
    draw() {
      if (this.drawPending || !this.renderer || this.element.hidden || document.hidden) return;
      this.drawPending = true;
      requestAnimationFrame(() => {
        this.drawPending = false;
        const width = this.host.clientWidth, height = this.host.clientHeight;
        if (!width || !height || this.element.hidden) return;
        if (width !== this.width || height !== this.height) {
          this.width = width; this.height = height;
          this.renderer.setSize(width, height, false); this.camera.aspect = width / height;
          this.camera.updateProjectionMatrix();
        }
        this.camera.position.setFromSpherical(this.orbit).add(this.target); this.camera.lookAt(this.target);
        this.renderer.render(this.scene, this.camera);
      });
    }
  }
  root.LeaderModelPreview = LeaderModelPreview;
})(typeof window === 'undefined' ? null : window);
