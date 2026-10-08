(function () {
  const MODELS = Object.freeze({ none: '', d405: 'D405', d435i: 'D435i', gemini2: 'Gemini 2' });
  const STORAGE_KEY = 'rebotarm.rs.wristCamera';

  // Geometry is shared by the solid assembly and its ghost. Dispose each
  // resource once, only after both have been detached.
  function disposeAssemblies(...roots) {
    const resources = new Set();
    roots.filter(Boolean).forEach((root) => {
      if (root.parent) root.parent.remove(root);
      root.traverse((child) => {
        if (child.geometry) resources.add(child.geometry);
        const materials = Array.isArray(child.material) ? child.material : [child.material];
        materials.filter(Boolean).forEach((material) => {
          resources.add(material);
          Object.values(material).forEach((value) => {
            if (value?.isTexture) resources.add(value);
          });
        });
      });
    });
    resources.forEach((resource) => resource.dispose());
  }

  function loadAssembly(model) {
    return new Promise((resolve, reject) => {
      let assembly;
      let failure;
      const manager = new THREE.LoadingManager();
      manager.onError = (url) => { failure = new Error(`Camera asset failed: ${url}`); };
      manager.onLoad = () => {
        if (failure || !assembly) {
          disposeAssemblies(assembly);
          reject(failure || new Error('Camera URDF did not load'));
        } else {
          resolve(assembly);
        }
      };
      const loader = new URDFLoader(manager);
      // Supply error callbacks for both formats: an incomplete assembly must
      // never replace the previously visible camera.
      loader.loadMeshCb = (url, loadingManager, done) => {
        const onError = (error) => {
          failure = error || new Error(`Camera mesh failed: ${url}`);
          done(null, failure);
        };
        if (/\.dae$/i.test(url)) {
          new THREE.ColladaLoader(loadingManager).load(url, (dae) => done(dae.scene), undefined, onError);
        } else {
          new THREE.STLLoader(loadingManager).load(url, (geometry) => {
            done(new THREE.Mesh(geometry, new THREE.MeshPhongMaterial()));
          }, undefined, onError);
        }
      };
      loader.load(new URL(`models/wrist-cameras/urdf/${model}.urdf`, document.baseURI).href,
        (loaded) => { assembly = loaded; }, undefined,
        (error) => { failure = error; });
    });
  }

  class WristCamera {
    constructor(select, status, styleRobot) {
      this.select = select;
      this.status = status;
      this.styleRobot = styleRobot;
      this.active = 'none';
      this.revision = 0;
      this.statusKey = 'wrist.none';
      this.statusModel = 'none';
      let saved;
      try { saved = localStorage.getItem(STORAGE_KEY); } catch (_) {}
      this.requested = Object.hasOwn(MODELS, saved) ? saved : 'none';
      this.select.value = this.requested;
      this.select.disabled = true;
      this.select.addEventListener('change', () => this.setModel(this.select.value));
      window.rebotI18n?.onLangChange(() => this.renderStatus());
      this.renderStatus();
    }

    attach(robot, ghostRobot) {
      this.wrist = robot.getObjectByName('gripper_end');
      this.ghostWrist = ghostRobot.getObjectByName('gripper_end');
      this.select.disabled = !this.wrist || !this.ghostWrist;
      if (this.select.disabled) {
        this.setStatus('wrist.failed', this.requested);
        return;
      }
      this.setModel(this.requested);
    }

    setStatus(key, model) {
      this.statusKey = key;
      this.statusModel = model;
      this.renderStatus();
    }

    renderStatus() {
      this.status.textContent = window.rebotI18n.t(this.statusKey, { model: MODELS[this.statusModel] });
    }

    async setModel(model) {
      if (!Object.hasOwn(MODELS, model) || !this.wrist) return false;
      const revision = ++this.revision;
      this.requested = model;
      this.select.value = model;
      this.select.setAttribute('aria-busy', model === 'none' ? 'false' : 'true');
      this.setStatus(model === 'none' ? 'wrist.none' : 'wrist.loading', model);
      let next;
      let nextGhost;
      try {
        if (model !== 'none') {
          next = await loadAssembly(model);
          if (revision !== this.revision) {
            disposeAssemblies(next);
            return false;
          }
          this.styleRobot(next, false);
          nextGhost = next.clone(true);
          this.styleRobot(nextGhost, true);
        }
        disposeAssemblies(this.assembly, this.ghostAssembly);
        this.assembly = next;
        this.ghostAssembly = nextGhost;
        if (next) this.wrist.add(next);
        if (nextGhost) this.ghostWrist.add(nextGhost);
        this.active = model;
        try { localStorage.setItem(STORAGE_KEY, model); } catch (_) {}
        this.setStatus(model === 'none' ? 'wrist.none' : 'wrist.ready', model);
        return true;
      } catch (error) {
        disposeAssemblies(next, nextGhost);
        if (revision !== this.revision) return false;
        console.warn('Wrist camera switch failed:', error);
        this.requested = this.active;
        this.select.value = this.active;
        this.setStatus('wrist.failed', model);
        return false;
      } finally {
        if (revision === this.revision) this.select.setAttribute('aria-busy', 'false');
      }
    }

    getDiagnostics() {
      const pose = (root, name) => {
        const link = root?.getObjectByName(name);
        if (!link) return null;
        link.updateWorldMatrix(true, false);
        const position = link.getWorldPosition(new THREE.Vector3());
        const rotation = link.getWorldQuaternion(new THREE.Quaternion());
        return { position: position.toArray(), quaternion: rotation.toArray() };
      };
      return {
        active: this.active,
        requested: this.requested,
        loading: this.select.getAttribute('aria-busy') === 'true',
        mount: pose(this.assembly, 'camera_mount_link'),
        camera: pose(this.assembly, 'camera_link'),
        ghostCamera: pose(this.ghostAssembly, 'camera_link')
      };
    }
  }

  window.RebotWristCamera = WristCamera;
})();
