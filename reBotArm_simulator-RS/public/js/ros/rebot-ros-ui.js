(function () {
  const TARGET_STORAGE_KEY = 'rebotarm.rs.ros.target';
  const t = window.rebotI18n ? window.rebotI18n.t : (k) => k;
  const ROS_TARGETS = {
    hardware: { namespace: 'rebotarm', label: 'RS 真机' },
    simulation: { namespace: 'rebotarm_rs', label: 'RS 仿真' }
  };
  function loadTargetKey() {
    try {
      const value = localStorage.getItem(TARGET_STORAGE_KEY);
      return ROS_TARGETS[value] ? value : 'simulation';
    } catch (_) {
      return 'simulation';
    }
  }
  const TARGET_KEY = loadTargetKey();
  const TARGET = ROS_TARGETS[TARGET_KEY];
  const NS = TARGET.namespace;
  const URL_STORAGE_KEY = 'rebotarm.ros.url';
  function loadSavedUrl() { try { return localStorage.getItem(URL_STORAGE_KEY) || ''; } catch (_) { return ''; } }
  function saveUrl(url) { try { localStorage.setItem(URL_STORAGE_KEY, url); } catch (_) {} }
 const OPEN_GRIPPER_M = 0.0715;
  const CLOSE_GRIPPER_M = 0;
  const OPEN_GRIPPER_MOTOR_RAD = 5.0;
  const CLOSE_GRIPPER_MOTOR_RAD = 0;
  // The RS MuJoCo fingers each travel 50 mm, so their relative opening spans
  // roughly 100 mm.  71.5 mm is the UI/motor command range, not the physical
  // distance between the two fingers.
  const GRIPPER_BASE_GAP_M = 0;
  const GRIPPER_FINGER_TRAVEL_M = 0.05;
  const GRIPPER_PHYSICAL_TRAVEL_M = 0.10;
  const GRIPPER_EFFECTIVE_GAP_M = 0.098;
  const GRASP_SQUEEZE_M = 0.004;
  const MIN_OBJECT_GRASP_M = 0.018;
  const DEFAULT_JOINT_VLIM_RAD_S = 1.20;
  const MAX_JOINT_VLIM_RAD_S = 1.50;
  const GRIPPER_VLIM_RAD_S = 5.0;
  const TEACH_ENDPOINT_DURATION_S = 3;
  const VISION_TRANSIT_Z_M = 0.190;
  const VISION_TRANSIT_Z_BY_COLOR_M = {
    blue: 0.185,
    yellow: 0.195
  };
  const VISION_FIRST_LIFT_CLEARANCE_M = 0.040;
  const VISION_FIRST_LIFT_MIN_M = 0.180;
  const VISION_POSE_SKIP_M = 0.006;
  const VISION_VERTICAL_ALIGN_CLEARANCE_M = 0.035;
  const VISION_PREGRASP_CLEARANCE_M = 0.020;
  const VISION_MIN_VERTICAL_ALIGN_Z_M = 0.170;
  const VISION_FIRST_LIFT_MIN_BY_COLOR_M = {};
  const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
  const REQUIRED_TOPICS = {
    jointStates: `/${NS}/joint_states`,
    mujocoJointStates: `/${NS}/mujoco/joint_states`,
    armStatus: `/${NS}/arm_status`,
    gripper: `/${NS}/gripper/state`,
    cameraImage: `/${NS}/mujoco/overhead_rgb/image_raw`,
    objectStates: `/${NS}/mujoco/object_states`,
    visionDetections: `/${NS}/vision/color_blocks/detections`,
    simAnimation: `/${NS}/sim/animation_event`
  };
  const REQUIRED_SERVICES = {
    gravityStart: `/${NS}/gravity_compensation/start`,
    gravityStop: `/${NS}/gravity_compensation/stop`,
    gravityStatus: `/${NS}/gravity_compensation/status`,
    recordStart: `/${NS}/mujoco/record/start`,
    recordStop: `/${NS}/mujoco/record/stop`,
    recordReplay: `/${NS}/mujoco/record/replay`,
    recordClear: `/${NS}/mujoco/record/clear`
  };

  const els = {
    target: document.getElementById('ros-target'),
    targetTitle: document.getElementById('ros-target-title'),
    url: document.getElementById('ros-url'),
    connect: document.getElementById('ros-connect'),
    disconnect: document.getElementById('ros-disconnect'),
    safeDisconnect: document.getElementById('ros-safe-disconnect'),
    mirror: document.getElementById('ros-mirror'),
    mirrorLabel: document.getElementById('ros-mirror-label'),
    control: document.getElementById('ros-control-enable'),
    controlLabel: document.getElementById('ros-control-label'),
    status: document.getElementById('ros-status'),
    message: document.getElementById('ros-message'),
    feedbackError: document.getElementById('ros-feedback-error'),
    enable: document.getElementById('ros-enable'),
    disable: document.getElementById('ros-disable'),
    safeHome: document.getElementById('ros-safe-home'),
    gravityStatus: document.getElementById('ros-gravity-status'),
    gravityStart: document.getElementById('ros-gravity-start'),
    gravityStop: document.getElementById('ros-gravity-stop'),
    gravityQuery: document.getElementById('ros-gravity-status-query'),
   rosOpenGripper: document.getElementById('ros-open-gripper'),
   closeGripper: document.getElementById('ros-close-gripper'),
   clearLog: document.getElementById('ros-clear-log'),
   log: document.getElementById('ros-log'),
   cameraCanvas: document.getElementById('ros-camera-canvas'),
    cameraStatus: document.getElementById('ros-camera-status'),
    cameraTopic: document.getElementById('ros-camera-topic'),
    visionStatus: document.getElementById('ros-vision-status'),
    visionTarget: document.getElementById('ros-vision-target'),
    visionColor: document.getElementById('ros-vision-color'),
    visionApproachZ: document.getElementById('ros-vision-approach-z'),
    visionGraspZ: document.getElementById('ros-vision-grasp-z'),
    visionFillPose: document.getElementById('ros-vision-fill-pose'),
    visionMoveAbove: document.getElementById('ros-vision-move-above'),
    visionPickDemo: document.getElementById('ros-vision-pick-demo'),
    visionPlaceDemo: document.getElementById('ros-vision-place-demo'),
    vlim: document.getElementById('ros-vlim'),
    sliderDamping: document.getElementById('ros-slider-damping'),
    trajectoryDuration: document.getElementById('ros-trajectory-duration'),
    poseX: document.getElementById('ros-pose-x'),
    poseY: document.getElementById('ros-pose-y'),
    poseZ: document.getElementById('ros-pose-z'),
    poseDuration: document.getElementById('ros-pose-duration'),
   checkIk: document.getElementById('ros-check-ik'),
   stopPath: document.getElementById('stop-path'),
   teachHardwareRecord: document.getElementById('teach-hardware-record'),
   teachHardwareModeRow: document.getElementById('teach-hardware-mode-row')
  };

 if (!window.ReBotRosClient || !els.connect) return;

  if (els.target) {
    els.target.value = TARGET_KEY;
    els.target.addEventListener('change', () => {
      const next = els.target.value;
      if (!ROS_TARGETS[next] || next === TARGET_KEY) return;
      if (client.connected || (client.socket && client.socket.readyState === WebSocket.CONNECTING)) {
        els.target.value = TARGET_KEY;
        setMessage(t('msg.switchTargetFirst'));
        writeLog('连接期间禁止切换真机/仿真目标', 'warn');
        return;
      }
      try { localStorage.setItem(TARGET_STORAGE_KEY, next); } catch (_) {}
      window.location.reload();
    });
  }
  if (els.targetTitle) els.targetTitle.textContent = `${TARGET.label}连接`;
  if (els.teachHardwareRecord) els.teachHardwareRecord.hidden = TARGET_KEY !== 'hardware';
  if (els.teachHardwareModeRow) els.teachHardwareModeRow.hidden = TARGET_KEY !== 'hardware';
  if (els.mirrorLabel) {
    els.mirrorLabel.textContent = TARGET_KEY === 'simulation'
      ? '镜像 RS MuJoCo 实际状态到网页'
      : `镜像${TARGET.label}关节状态到网页`;
  }
  if (els.controlLabel) {
    els.controlLabel.textContent = TARGET_KEY === 'hardware'
      ? '允许网页向 RS 真机发控制'
      : '等待识别 RS Fake Driver';
  }

  if (els.url && !els.url.value) {
    const saved = loadSavedUrl();
    if (saved) els.url.value = saved;
  }
  const client = new window.ReBotRosClient({ namespace: NS, url: els.url ? els.url.value : '' });
 window.reBotRos = client;

  const lastSent = new Map();
  const lastPublishedCommandValues = new Map();
  const dampedSliderCommands = new Map();
  const lastObservedSimCommand = new Map();
  const simTargetAngles = new Map();
  const mirrorHoldUntil = new Map();
  const hardwareSliderJoints = new Set();
  const sliderFeedbackTransitions = new Map();
  // Match a typical browser render cadence.  This caps each joint stream at
  // 60 Hz while the controller continues to run its 125 Hz MIT loop.
  const COMMAND_INTERVAL_MS = 1000 / 60;
 const SLIDER_INPUT_DEADBAND_RAD = Math.PI / 180;
 const FEEDBACK_RENDER_MIN_MS = 32;
 const FEEDBACK_RENDER_MAX_MS = 120;
 const JOINT_FEEDBACK_DEADBAND_RAD = 0.0025;
 const GRIPPER_FEEDBACK_DEADBAND_M = 0.00025;
 const HARDWARE_FEEDBACK_MIN_CUTOFF_HZ = 3.0;
 const HARDWARE_FEEDBACK_SPEED_BETA = 4.0;
 const HARDWARE_FEEDBACK_DERIVATIVE_CUTOFF_HZ = 1.0;
 const MIRROR_HOLD_MS = 1800;
 let latestJointPositions = null;
 let rosBackend = 'unknown';
 let latestArmEnabled = false;
  let latestJointStateAt = 0;
  let latestMujocoStateAt = 0;
  let latestGripperPosition = null;
  let latestGripperVelocity = null;
  let latestMujocoGripperPosition = null;
  let latestMujocoGripperVelocity = null;
  let latestMujocoGripperAt = 0;
  let latestGripperAt = 0;
  let pendingGripperSliderWidth = null;
  let gripperSliderPublishFrame = 0;
  let listedTopics = new Set();
  let listedServices = new Set();
  let lowLevelPlayback = null;
  let lastTargetPoseSent = 0;
  let latestVisionPayload = null;
  let latestVisionAt = 0;
  let latestMujocoSimulationMode = '';
  let selectedVisionTarget = null;
  let lastVisionTarget = null;
  let heldVisionTarget = null;
  let autoVisionTargetColor = '';
  let visionSequenceBusy = false;
  let activeVisionOperation = '';
  let queuedVisionOperation = null;
  let safeDisconnectBusy = false;
  let hardwareTeachActive = false;
  let hardwareTeachBusy = false;
  let hardwareTeachGravityStarted = false;
  let hardwareTeachModeConfirmed = false;
  let gravityCompensationActive = false;
  let gravityStatusSource = 'initial';
  let gravityStatusPollInFlight = false;
  // Multi-joint hardware motions use one shared requestAnimationFrame
  // renderer.  Keeping all links on the same feedback timeline prevents a
  // preset/replay from looking like seven independent, stepped animations.
  let hardwareBatchFeedbackActive = false;
  let hardwareBatchFinishTimer = 0;
  const hardwareFeedbackFilters = new Map();
  let feedbackRenderFrame = 0;
  let feedbackRenderCurrent = null;
  let feedbackRenderStart = null;
  let feedbackRenderTarget = null;
  let feedbackRenderStartedAt = 0;
  let feedbackRenderDuration = 1;
  let feedbackLastSampleAt = 0;
  let gripperFeedbackFrame = 0;
  let gripperFeedbackCurrent = null;
  let gripperFeedbackStart = null;
  let gripperFeedbackTarget = null;
  let gripperFeedbackStartedAt = 0;
  let gripperFeedbackDuration = 1;
  let gripperFeedbackLastSampleAt = 0;
  let sliderDampingFrame = 0;
  let sliderFeedbackFrame = 0;
  let gripperRepublishTimer = 0;

  // Hardware teaching needs the full driver publication rate; throttling here
  // would silently discard encoder samples.
  client.subscribe(REQUIRED_TOPICS.jointStates, 'sensor_msgs/msg/JointState', handleJointStates, { throttleRate: 0 });
  if (TARGET_KEY === 'simulation') {
    client.subscribe(REQUIRED_TOPICS.mujocoJointStates, 'sensor_msgs/msg/JointState', handleMujocoJointStates, { throttleRate: 40 });
  }
  client.subscribe(REQUIRED_TOPICS.gripper, 'rebotarm_msgs/msg/JointMotorState', handleGripperState, { throttleRate: 80 });
  client.subscribe(REQUIRED_TOPICS.armStatus, 'rebotarm_msgs/msg/ArmStatus', handleArmStatus, { throttleRate: 200 });
  client.subscribe(REQUIRED_TOPICS.cameraImage, 'sensor_msgs/msg/Image', handleCameraImage, { throttleRate: 120 });
  client.subscribe(REQUIRED_TOPICS.visionDetections, 'std_msgs/msg/String', handleVisionDetections, { throttleRate: 180 });
  client.subscribe(REQUIRED_TOPICS.simAnimation, 'std_msgs/msg/String', handleSimAnimationEvent, { throttleRate: 0 });
  client.subscribe(REQUIRED_TOPICS.objectStates, 'std_msgs/msg/String', handleMujocoObjectStates, { throttleRate: 33 });
  if (els.cameraTopic) els.cameraTopic.textContent = REQUIRED_TOPICS.cameraImage;

  client.addEventListener('status', (event) => {
    const detail = event.detail || {};
   setStatus(detail.state, detail.message);
    els.connect.disabled = detail.state === 'connecting' || detail.state === 'open';
    els.disconnect.disabled = detail.state === 'connecting' || detail.state === 'closed' || detail.state === 'error';
   if (detail.state !== 'connecting') {
      writeLog(detail.message || detail.state, detail.state === 'error' ? 'error' : detail.state === 'open' ? 'ok' : 'info');
    }
    updateDiagnostics();
   if (detail.state === 'closed' || detail.state === 'error') {
     finishHardwareTeachLocal(detail.state === 'error' ? 'ROS 连接异常，真机示教已停止' : 'ROS 连接断开，真机示教已停止');
     resetFeedbackRenderer();
     updateGravityStatus(false, t('msg.rosNotConnected'), 'connection');
   }
   if (detail.state === 'open') {
     window.setTimeout(() => {
       runDiagnostics();
     }, 250);
   }
  });

  els.connect.addEventListener('click', () => {
   const nextUrl = els.url.value.trim();
   if (!canConnectWebSocketUrl(nextUrl)) return;
    saveUrl(nextUrl);
   client.autoReconnect = true;
    client.connect(nextUrl);
 });
 els.disconnect.addEventListener('click', disconnectRos);
 window.addEventListener('pagehide', () => {
   client.autoReconnect = false;
   if (client.socket) client.socket.close();
 });
 els.teachHardwareRecord?.addEventListener('click', toggleHardwareTeaching);
 els.enable.addEventListener('click', () => guardedCall(() => client.enable(), t('msg.reqEnable')));
 els.disable.addEventListener('click', () => {
   void stopHardwareTeaching(true).finally(() => {
   cancelLowLevelPlayback();
   resetWebControlState();
   void guardedCall(() => client.disable(), t('msg.reqDisable'), true)
     .finally(resetWebControlState);
   });
 });
 els.safeHome.addEventListener('click', () => {
   void stopHardwareTeaching(true).finally(() => {
   cancelLowLevelPlayback();
   resetWebControlState();
   void guardedCall(() => client.safeHome(), t('msg.reqSafeHome'))
     .finally(resetWebControlState);
   });
 });
 els.gravityStart.addEventListener('click', () => {
   cancelLowLevelPlayback();
   cancelPendingWebMotionCommands();
   guardedOptionalService(
     REQUIRED_SERVICES.gravityStart,
     () => client.startGravityCompensation(),
     t('msg.reqGravityStart')
   );
 });
 els.gravityStop.addEventListener('click', () => {
    cancelLowLevelPlayback();
    if (hardwareTeachActive) void stopHardwareTeaching(false);
    guardedOptionalService(
      REQUIRED_SERVICES.gravityStop,
      () => client.stopGravityCompensation(),
      t('msg.reqGravityStop'),
      true
    );
  });
  els.gravityQuery.addEventListener('click', queryGravityCompensation);
 els.rosOpenGripper.addEventListener('click', () => sendGripper(OPEN_GRIPPER_M, { requireControl: true }));
 els.closeGripper.addEventListener('click', () => sendGripper(CLOSE_GRIPPER_M, { requireControl: true }));
 els.clearLog.addEventListener('click', () => { els.log.innerHTML = ''; });
  els.checkIk.addEventListener('click', checkIk);
  document.getElementById('ros-help-top')?.addEventListener('click', () => document.getElementById('ros-help-dialog')?.showModal());
  document.getElementById('ros-help-close')?.addEventListener('click', () => document.getElementById('ros-help-dialog')?.close());
  const sidebar = document.querySelector('.control-panel');
  const appShell = document.querySelector('.app-shell');
  const collapseBtn = document.getElementById('sidebar-collapse');
  collapseBtn?.addEventListener('click', () => {
    if (!sidebar) return;
    sidebar.classList.toggle('collapsed');
    const collapsed = sidebar.classList.contains('collapsed');
    collapseBtn.textContent = collapsed ? '▶' : '◀';
    collapseBtn.title = collapsed ? t('panel.expand') : t('panel.collapse');
 });
 if (els.visionColor) els.visionColor.addEventListener('change', () => {
    if (els.visionColor.value === 'auto') autoVisionTargetColor = '';
    updateSelectedVisionTarget();
  });
  if (els.visionFillPose) els.visionFillPose.addEventListener('click', fillPoseFromVisionTarget);
  if (els.visionMoveAbove) els.visionMoveAbove.addEventListener('click', moveAboveVisionTarget);
  if (els.visionPickDemo) els.visionPickDemo.addEventListener('click', requestVisionPick);
  if (els.visionPlaceDemo) els.visionPlaceDemo.addEventListener('click', requestVisionPlace);
  if (els.stopPath) {
    els.stopPath.addEventListener('click', () => {
      cancelLowLevelPlayback();
      writeLog('已请求停止低层回放', 'warn');
    });
  }

  els.control.addEventListener('change', () => {
    if (els.control.checked) {
      writeLog('控制锁已打开', 'info');
    } else if (hardwareTeachActive) {
      void stopHardwareTeaching(true);
    }
  });
  els.mirror.addEventListener('change', () => {
    resetFeedbackRenderer();
    if (
      els.mirror.checked &&
      TARGET_KEY === 'hardware' &&
      hardwareBatchFeedbackActive &&
      latestJointPositions
    ) {
      queueHardwareFeedbackFrame(latestJointPositions);
    }
  });

  waitForSimApi((sim) => {
    if (typeof sim.setHardwareFeedbackDriven === 'function') {
      sim.setHardwareFeedbackDriven(TARGET_KEY === 'hardware');
    }
    sim.onCommand((command) => forwardSimCommand(command));
    bindJointSliderFallback();
  });

  setStatus('closed', t('msg.rosNotConnected'));
  updateDiagnostics();
  window.setInterval(updateDiagnostics, 1000);
  window.setInterval(pollGravityCompensationStatus, 500);

  function handleJointStates(msg) {
    if (!window.reBotSim || !Array.isArray(msg.name) || !Array.isArray(msg.position)) return;
    const next = jointAnglesFromState(msg);
    const useDriverState = TARGET_KEY !== 'simulation' || !mujocoStateIsFresh();

    if (hardwareTeachActive && typeof window.reBotSim.appendHardwareTeachingSample === 'function') {
      window.reBotSim.appendHardwareTeachingSample(next, msg.header && msg.header.stamp);
    }

    if (useDriverState && Object.keys(next).length) {
      latestJointPositions = { ...(latestJointPositions || {}), ...next };
      latestJointStateAt = performance.now();
    }
    if (useDriverState) updateFeedbackError(next);

    if (useDriverState) {
      const displayAngles = TARGET_KEY === 'hardware'
        ? filterHardwareJointAngles(next, performance.now())
        : next;
      mirrorJointAngles(displayAngles);
    }
    updateDiagnostics();
  }

  function lowPassAlpha(cutoffHz, dtSeconds) {
    const cutoff = Math.max(0.01, Number(cutoffHz) || 0.01);
    const dt = Math.max(0.001, Number(dtSeconds) || 0.001);
    return 1 - Math.exp(-2 * Math.PI * cutoff * dt);
  }

  function filterHardwareJointAngles(next, timestamp) {
    const filtered = {};
    Object.entries(next || {}).forEach(([name, rawValue]) => {
      const raw = Number(rawValue);
      if (!Number.isFinite(raw)) return;
      let state = hardwareFeedbackFilters.get(name);
      if (!state) {
        state = { raw, value: raw, derivative: 0, at: timestamp };
        hardwareFeedbackFilters.set(name, state);
        filtered[name] = raw;
        return;
      }

      const dt = Math.max(0.005, Math.min(0.2, (timestamp - state.at) / 1000));
      const rawDerivative = (raw - state.raw) / dt;
      const derivativeAlpha = lowPassAlpha(
        HARDWARE_FEEDBACK_DERIVATIVE_CUTOFF_HZ,
        dt
      );
      state.derivative += derivativeAlpha * (rawDerivative - state.derivative);
      const cutoff = HARDWARE_FEEDBACK_MIN_CUTOFF_HZ +
        HARDWARE_FEEDBACK_SPEED_BETA * Math.abs(state.derivative);
      const valueAlpha = lowPassAlpha(cutoff, dt);
      state.value += valueAlpha * (raw - state.value);
      state.raw = raw;
      state.at = timestamp;
      filtered[name] = state.value;
    });
    return filtered;
  }

  function handleMujocoJointStates(msg) {
    if (!window.reBotSim || !Array.isArray(msg.name) || !Array.isArray(msg.position)) return;
    const next = jointAnglesFromState(msg);
    latestMujocoStateAt = performance.now();
    if (Object.keys(next).length) {
      latestJointPositions = { ...(latestJointPositions || {}), ...next };
      latestJointStateAt = latestMujocoStateAt;
      updateFeedbackError(next);
      mirrorJointAngles(next);
    }

    const gripperIndex = msg.name.findIndex((name) => String(name).toLowerCase().endsWith('gripper_joint1'));
    const fingerPosition = gripperIndex >= 0 ? Number(msg.position[gripperIndex]) : NaN;
    if (Number.isFinite(fingerPosition)) {
      const width = mujocoFingerToWidth(fingerPosition);
      const fingerVelocity = Array.isArray(msg.velocity)
        ? Number(msg.velocity[gripperIndex])
        : NaN;
      const widthVelocity = Number.isFinite(fingerVelocity)
        ? Math.abs(fingerVelocity) * OPEN_GRIPPER_M / GRIPPER_FINGER_TRAVEL_M
        : null;
      // Keep an independent physics-feedback channel.  The fake driver also
      // publishes gripper/state and must not overwrite the MuJoCo contact
      // samples used by the grasp completion state machine.
      latestMujocoGripperPosition = width;
      latestMujocoGripperVelocity = widthVelocity;
      latestMujocoGripperAt = latestMujocoStateAt;
      latestGripperPosition = width;
      latestGripperAt = latestMujocoStateAt;
      latestGripperVelocity = widthVelocity;
      if (els.mirror.checked) {
        const now = performance.now();
        const holdUntil = mirrorHoldUntil.get('gripper') || 0;
        const target = simTargetAngles.get('gripper');
        const reachedTarget = typeof target === 'number' && Math.abs(target - width) < 0.003;
        const targetExpired = typeof target === 'number' && holdUntil > 0 && now > holdUntil;
        if (reachedTarget || targetExpired) {
          simTargetAngles.delete('gripper');
          mirrorHoldUntil.delete('gripper');
        }
        if (reachedTarget || targetExpired || typeof target !== 'number' || now > holdUntil) {
          window.reBotSim.setGripperWidth(width, { source: 'ros', animate: false });
        }
        hideGhostWhenTargetsRetired();
      }
    }
    updateDiagnostics();
  }

  function jointAnglesFromState(msg) {
    const next = {};
    msg.name.forEach((name, index) => {
      const simName = normalizeJointName(name);
      const value = Number(msg.position[index]);
      if (!simName || !Number.isFinite(value)) return;
      next[simName] = value;
    });
    return next;
  }

  function mirrorJointAngles(next) {
    if (!els.mirror.checked || !Object.keys(next).length) return;
    if (TARGET_KEY === 'hardware' && hardwareBatchFeedbackActive) {
      Object.entries(next).forEach(([name, value]) => {
        const target = simTargetAngles.get(name);
        if (typeof target === 'number' && Math.abs(target - value) < 0.025) {
          simTargetAngles.delete(name);
          mirrorHoldUntil.delete(name);
        }
      });
      queueHardwareFeedbackFrame(next);
      scheduleHardwareBatchFeedbackFinish();
      return;
    }
    const mirrored = {};
    const now = performance.now();
    Object.entries(next).forEach(([name, value]) => {
      const holdUntil = mirrorHoldUntil.get(name) || 0;
      const target = simTargetAngles.get(name);
      const reachedTarget = typeof target === 'number' && Math.abs(target - value) < 0.025;
      const targetExpired = typeof target === 'number' && holdUntil > 0 && now > holdUntil;
      const targetRetired = reachedTarget || targetExpired;
      if (targetRetired) {
        simTargetAngles.delete(name);
        mirrorHoldUntil.delete(name);
      }
      if (TARGET_KEY === 'hardware') {
        queueHardwareSliderFeedback(name, value);
        if (targetRetired) {
          if (hardwareSliderJoints.has(name)) hardwareSliderJoints.delete(name);
        }
        return;
      }
      if (targetRetired || typeof target !== 'number' || now > holdUntil) {
        mirrored[name] = value;
      }
    });
    if (
      simTargetAngles.size === 0 &&
      window.reBotSim &&
      typeof window.reBotSim.setGhostVisible === 'function'
    ) {
      window.reBotSim.setGhostVisible(false);
    }
    if (Object.keys(mirrored).length) window.reBotSim.setAngles(mirrored, { source: 'ros' });
  }

  function queueHardwareSliderFeedback(name, value) {
    if (!window.reBotSim || TARGET_KEY !== 'hardware' || !Number.isFinite(Number(value))) return;
    const now = performance.now();
    const numeric = Number(value);
    let state = sliderFeedbackTransitions.get(name);
    if (!state) {
      const current = typeof window.reBotSim.getAngles === 'function'
        ? Number(window.reBotSim.getAngles()[name])
        : numeric;
      if (
        !hardwareSliderJoints.has(name) &&
        Number.isFinite(current) &&
        Math.abs(numeric - current) <= JOINT_FEEDBACK_DEADBAND_RAD
      ) return;
      state = {
        current: Number.isFinite(current) ? current : numeric,
        start: Number.isFinite(current) ? current : numeric,
        target: numeric,
        startedAt: now,
        duration: FEEDBACK_RENDER_MIN_MS,
        lastSampleAt: now
      };
      sliderFeedbackTransitions.set(name, state);
    } else {
      const ratio = Math.max(0, Math.min(1, (now - state.startedAt) / Math.max(state.duration, 1)));
      state.current = state.start + (state.target - state.start) * ratio;
      if (
        !hardwareSliderJoints.has(name) &&
        Math.abs(numeric - state.target) <= JOINT_FEEDBACK_DEADBAND_RAD
      ) {
        return;
      }
      state.start = state.current;
      state.target = numeric;
      state.duration = Math.max(
        FEEDBACK_RENDER_MIN_MS,
        Math.min(FEEDBACK_RENDER_MAX_MS, now - state.lastSampleAt || 80)
      );
      state.startedAt = now;
      state.lastSampleAt = now;
    }
    if (!sliderFeedbackFrame) {
      sliderFeedbackFrame = window.requestAnimationFrame(renderHardwareSliderFeedback);
    }
  }

  function renderHardwareSliderFeedback(timestamp) {
    sliderFeedbackFrame = 0;
    if (!client.connected || !els.mirror.checked || TARGET_KEY !== 'hardware') {
      sliderFeedbackTransitions.clear();
      return;
    }
    const controlled = {};
    const observed = {};
    const preserveTarget = simTargetAngles.size > 0;
    let animating = false;
    sliderFeedbackTransitions.forEach((state, name) => {
      const ratio = Math.max(0, Math.min(1, (timestamp - state.startedAt) / Math.max(state.duration, 1)));
      state.current = state.start + (state.target - state.start) * ratio;
      if (hardwareSliderJoints.has(name)) controlled[name] = state.current;
      else observed[name] = state.current;
      if (ratio < 1) animating = true;
    });
    if (Object.keys(observed).length && window.reBotSim) {
      window.reBotSim.setAngles(observed, {
        source: 'ros',
        preserveControls: false,
        preserveGhost: preserveTarget
      });
    }
    if (Object.keys(controlled).length && window.reBotSim) {
      window.reBotSim.setAngles(controlled, {
        source: 'ros',
        preserveControls: true,
        preserveGhost: preserveTarget
      });
    }
    if (
      preserveTarget &&
      typeof window.reBotSim.setFeedbackTarget === 'function'
    ) {
      window.reBotSim.setFeedbackTarget(Object.fromEntries(simTargetAngles));
    }
    if (animating) {
      sliderFeedbackFrame = window.requestAnimationFrame(renderHardwareSliderFeedback);
    }
  }

  function cancelHardwareSliderFeedback(name) {
    if (name) {
      hardwareSliderJoints.delete(name);
      sliderFeedbackTransitions.delete(name);
    } else {
      hardwareSliderJoints.clear();
      sliderFeedbackTransitions.clear();
    }
    if (!sliderFeedbackTransitions.size && sliderFeedbackFrame) {
      window.cancelAnimationFrame(sliderFeedbackFrame);
      sliderFeedbackFrame = 0;
    }
  }

  function queueHardwareFeedbackFrame(next) {
    if (!window.reBotSim || TARGET_KEY !== 'hardware') return;
    const sample = {};
    Object.entries(next || {}).forEach(([name, value]) => {
      const numeric = Number(value);
      if (Number.isFinite(numeric)) sample[name] = numeric;
    });
    if (!Object.keys(sample).length) return;

    const now = performance.now();
    if (!feedbackRenderCurrent) {
      const current = typeof window.reBotSim.getAngles === 'function'
        ? window.reBotSim.getAngles()
        : {};
      feedbackRenderCurrent = { ...current, ...sample };
      feedbackRenderStart = { ...feedbackRenderCurrent };
      feedbackRenderTarget = { ...feedbackRenderCurrent };
      feedbackLastSampleAt = now;
      applyHardwareFeedbackFrame(feedbackRenderCurrent);
      feedbackRenderFrame = window.requestAnimationFrame(renderHardwareFeedbackFrame);
      return;
    }

    feedbackRenderStart = { ...feedbackRenderCurrent };
    feedbackRenderTarget = { ...feedbackRenderTarget, ...sample };
    feedbackRenderDuration = Math.max(
      FEEDBACK_RENDER_MIN_MS,
      Math.min(FEEDBACK_RENDER_MAX_MS, feedbackLastSampleAt ? now - feedbackLastSampleAt : 80)
    );
    feedbackRenderStartedAt = now;
    feedbackLastSampleAt = now;
    if (!feedbackRenderFrame) {
      feedbackRenderFrame = window.requestAnimationFrame(renderHardwareFeedbackFrame);
    }
  }

  function renderHardwareFeedbackFrame(timestamp) {
    feedbackRenderFrame = 0;
    if (
      !client.connected ||
      !els.mirror.checked ||
      TARGET_KEY !== 'hardware' ||
      !feedbackRenderStart ||
      !feedbackRenderTarget
    ) return;

    const ratio = Math.max(
      0,
      Math.min(1, (timestamp - feedbackRenderStartedAt) / Math.max(feedbackRenderDuration, 1))
    );
    const rendered = {};
    Object.entries(feedbackRenderTarget).forEach(([name, target]) => {
      const start = Number(feedbackRenderStart[name]);
      rendered[name] = Number.isFinite(start)
        ? start + (target - start) * ratio
        : target;
    });
    feedbackRenderCurrent = rendered;
    applyHardwareFeedbackFrame(rendered);
    // Keep rendering at display cadence even after reaching the latest
    // sample. Local slider/IK animations also run per frame; continuously
    // reapplying actual feedback guarantees the solid robot remains physical.
    feedbackRenderFrame = window.requestAnimationFrame(renderHardwareFeedbackFrame);
  }

  function applyHardwareFeedbackFrame(angles) {
    if (!window.reBotSim || typeof window.reBotSim.setAngles !== 'function') return;
    const preserveTarget = simTargetAngles.size > 0;
    window.reBotSim.setAngles(angles, {
      source: 'ros',
      forceFeedback: true,
      preserveGhost: preserveTarget,
      preserveControls: preserveTarget
    });
    if (preserveTarget && typeof window.reBotSim.setFeedbackTarget === 'function') {
      window.reBotSim.setFeedbackTarget(Object.fromEntries(simTargetAngles));
    }
  }

  function resetFeedbackRenderer() {
    cancelHardwareSliderFeedback();
    if (feedbackRenderFrame) window.cancelAnimationFrame(feedbackRenderFrame);
    feedbackRenderFrame = 0;
    feedbackRenderCurrent = null;
    feedbackRenderStart = null;
    feedbackRenderTarget = null;
    feedbackRenderStartedAt = 0;
    feedbackRenderDuration = 1;
    feedbackLastSampleAt = 0;
    if (gripperFeedbackFrame) window.cancelAnimationFrame(gripperFeedbackFrame);
    gripperFeedbackFrame = 0;
    gripperFeedbackCurrent = null;
    gripperFeedbackStart = null;
    gripperFeedbackTarget = null;
    gripperFeedbackStartedAt = 0;
    gripperFeedbackDuration = 1;
    gripperFeedbackLastSampleAt = 0;
  }

  function scheduleHardwareBatchFeedbackFinish() {
    if (simTargetAngles.size > 0 || !hardwareBatchFeedbackActive) return;
    if (hardwareBatchFinishTimer) window.clearTimeout(hardwareBatchFinishTimer);
    hardwareBatchFinishTimer = window.setTimeout(() => {
      hardwareBatchFinishTimer = 0;
      if (simTargetAngles.size > 0) return;
      hardwareBatchFeedbackActive = false;
      resetFeedbackRenderer();
      if (window.reBotSim && typeof window.reBotSim.setGhostVisible === 'function') {
        window.reBotSim.setGhostVisible(false);
      }
    }, FEEDBACK_RENDER_MAX_MS + 20);
  }

  function queueHardwareGripperFeedback(width) {
    if (
      TARGET_KEY !== 'hardware' ||
      !window.reBotSim ||
      !Number.isFinite(Number(width))
    ) return;

    const target = Number(width);
    const now = performance.now();
    if (!Number.isFinite(gripperFeedbackCurrent)) {
      const current = typeof window.reBotSim.getAngles === 'function'
        ? Number(window.reBotSim.getAngles().gripper)
        : NaN;
      gripperFeedbackCurrent = Number.isFinite(current) ? current : target;
    }

    if (
      Number.isFinite(gripperFeedbackStart) &&
      Number.isFinite(gripperFeedbackTarget)
    ) {
      const ratio = Math.max(
        0,
        Math.min(1, (now - gripperFeedbackStartedAt) / Math.max(gripperFeedbackDuration, 1))
      );
      gripperFeedbackCurrent = gripperFeedbackStart +
        (gripperFeedbackTarget - gripperFeedbackStart) * ratio;
    }

    if (
      !simTargetAngles.has('gripper') &&
      Math.abs(target - gripperFeedbackCurrent) <= GRIPPER_FEEDBACK_DEADBAND_M
    ) {
      if (gripperFeedbackFrame) window.cancelAnimationFrame(gripperFeedbackFrame);
      gripperFeedbackFrame = 0;
      gripperFeedbackStart = gripperFeedbackCurrent;
      gripperFeedbackTarget = gripperFeedbackCurrent;
      applyHardwareGripperFeedback(gripperFeedbackCurrent);
      return;
    }

    gripperFeedbackStart = gripperFeedbackCurrent;
    gripperFeedbackTarget = target;
    gripperFeedbackDuration = Math.max(
      FEEDBACK_RENDER_MIN_MS,
      Math.min(
        FEEDBACK_RENDER_MAX_MS,
        gripperFeedbackLastSampleAt ? now - gripperFeedbackLastSampleAt : 80
      )
    );
    gripperFeedbackStartedAt = now;
    gripperFeedbackLastSampleAt = now;
    if (!gripperFeedbackFrame) {
      gripperFeedbackFrame = window.requestAnimationFrame(renderHardwareGripperFeedback);
    }
  }

  function renderHardwareGripperFeedback(timestamp) {
    gripperFeedbackFrame = 0;
    if (
      !client.connected ||
      !els.mirror.checked ||
      TARGET_KEY !== 'hardware' ||
      !Number.isFinite(gripperFeedbackStart) ||
      !Number.isFinite(gripperFeedbackTarget)
    ) return;

    const ratio = Math.max(
      0,
      Math.min(1, (timestamp - gripperFeedbackStartedAt) / Math.max(gripperFeedbackDuration, 1))
    );
    gripperFeedbackCurrent = gripperFeedbackStart +
      (gripperFeedbackTarget - gripperFeedbackStart) * ratio;
    applyHardwareGripperFeedback(gripperFeedbackCurrent);
    if (ratio < 1) {
      gripperFeedbackFrame = window.requestAnimationFrame(renderHardwareGripperFeedback);
    }
  }

  function applyHardwareGripperFeedback(width) {
    if (!window.reBotSim || typeof window.reBotSim.setGripperWidth !== 'function') return;
    const armTargets = Object.fromEntries(
      Array.from(simTargetAngles.entries()).filter(([name]) => name !== 'gripper')
    );
    const preserveTarget = Object.keys(armTargets).length > 0;
    window.reBotSim.setGripperWidth(width, {
      source: 'ros',
      animate: false,
      forceFeedback: true,
      preserveGhost: preserveTarget,
      preserveControls: simTargetAngles.has('gripper')
    });
    if (
      hardwareBatchFeedbackActive &&
      preserveTarget &&
      typeof window.reBotSim.setFeedbackTarget === 'function'
    ) {
      window.reBotSim.setFeedbackTarget(armTargets);
    }
  }

  function mujocoStateIsFresh() {
    return latestMujocoStateAt > 0 && performance.now() - latestMujocoStateAt < 600;
  }

  function mujocoFingerToWidth(position) {
    const ratio = Math.max(0, Math.min(1, Number(position) / GRIPPER_FINGER_TRAVEL_M));
    return ratio * OPEN_GRIPPER_M;
  }

  function handleGripperState(msg) {
    const useDriverState = TARGET_KEY !== 'simulation' || !mujocoStateIsFresh();
    if (useDriverState && typeof msg.position === 'number') {
      latestGripperPosition = gripperMotorToWidth(msg.position);
      latestGripperAt = performance.now();
    }
    if (useDriverState && typeof msg.velocity === 'number') {
      latestGripperVelocity = Math.abs(msg.velocity) * OPEN_GRIPPER_M / OPEN_GRIPPER_MOTOR_RAD;
    }
    if (useDriverState && els.mirror.checked && window.reBotSim && typeof msg.position === 'number') {
      const width = gripperMotorToWidth(msg.position);
      if (TARGET_KEY === 'hardware' && hardwareBatchFeedbackActive) {
        const target = simTargetAngles.get('gripper');
        const holdUntil = mirrorHoldUntil.get('gripper') || 0;
        const targetRetired = typeof target === 'number' && (
          Math.abs(target - width) < 0.003 ||
          (holdUntil > 0 && performance.now() > holdUntil)
        );
        if (targetRetired) {
          simTargetAngles.delete('gripper');
          mirrorHoldUntil.delete('gripper');
        }
        queueHardwareFeedbackFrame({ gripper: width });
        scheduleHardwareBatchFeedbackFinish();
      } else if (TARGET_KEY === 'hardware') {
        const target = simTargetAngles.get('gripper');
        const reachedTarget = typeof target === 'number' && Math.abs(target - width) < 0.003;
        const holdUntil = mirrorHoldUntil.get('gripper') || 0;
        const targetExpired = typeof target === 'number' && holdUntil > 0 && performance.now() > holdUntil;
        if (reachedTarget || targetExpired) {
          // Once the measured gripper reaches the command, stop drawing the
          // translucent target on top of the solid gripper.  Leaving both at
          // almost the same width makes normal encoder noise look like flicker.
          simTargetAngles.delete('gripper');
          mirrorHoldUntil.delete('gripper');
          if (reachedTarget) setMessage(t('msg.gripperArrived', { mm: Math.round(width * 1000) }));
          hideGhostWhenTargetsRetired();
        }
        // The solid gripper represents measured hardware state.  Interpolate
        // only this scalar between ROS samples so TCP/arm state is untouched.
        queueHardwareGripperFeedback(width);
      } else {
        const now = performance.now();
        const holdUntil = mirrorHoldUntil.get('gripper') || 0;
        const target = simTargetAngles.get('gripper');
        const reachedTarget = typeof target === 'number' && Math.abs(target - width) < 0.003;
        const targetExpired = typeof target === 'number' && holdUntil > 0 && now > holdUntil;
        if (reachedTarget || targetExpired) {
          simTargetAngles.delete('gripper');
          mirrorHoldUntil.delete('gripper');
        }
        if (reachedTarget || targetExpired || typeof target !== 'number' || now > holdUntil) {
          window.reBotSim.setGripperWidth(width, { source: 'ros', animate: false });
        }
        hideGhostWhenTargetsRetired();
      }
    }
    if (typeof msg.position === 'number' && simTargetAngles.has('gripper')) {
      const width = gripperMotorToWidth(msg.position);
      const target = simTargetAngles.get('gripper');
      const err = Math.abs(target - width);
      if (err < 0.003) {
        setMessage(t('msg.gripperArrived', { mm: Math.round(width * 1000) }));
      } else {
        setMessage(t('msg.gripperMoving', { cmd: Math.round(target * 1000), mm: Math.round(width * 1000) }));
      }
    }
    updateDiagnostics();
  }

  function hideGhostWhenTargetsRetired() {
    if (
      simTargetAngles.size === 0 &&
      window.reBotSim &&
      typeof window.reBotSim.setGhostVisible === 'function'
    ) {
      window.reBotSim.setGhostVisible(false);
    }
  }

  function handleArmStatus(msg) {
    latestArmEnabled = Boolean(msg.enabled);
    const enabled = msg.enabled ? t('st.enabled') : t('st.disabled');
    const mode = msg.mode || 'unknown';
    updateRosBackend(mode);
    const machine = msg.state_machine || 'unknown';
    const errors = Array.isArray(msg.error_codes) && msg.error_codes.length ? t('fb.errors', { codes: msg.error_codes.join(', ') }) : '';
    setMessage(`${enabled}，模式 ${mode}，状态 ${machine}${errors}`);
    updateGravityStatus(machine === 'GRAVITY_COMP', machine, 'arm');
    if (machine === 'GRAVITY_COMP') hardwareTeachModeConfirmed = true;
    if (hardwareTeachActive && hardwareTeachModeConfirmed && machine !== 'GRAVITY_COMP') {
      void stopHardwareTeaching(false);
    }
    updateDiagnostics();
  }

  async function toggleHardwareTeaching() {
    if (hardwareTeachBusy) return;
    if (hardwareTeachActive) {
      // An explicit teaching finish owns the post-teach home. Cleanup paths
      // such as Disable and Safe Home already perform their own final action.
      await stopHardwareTeaching(true, { returnHome: true });
      return;
    }
    await startHardwareTeaching();
  }

  async function startHardwareTeaching() {
    if (TARGET_KEY !== 'hardware') {
      setMessage('推动示教仅支持 RS 真机目标');
      return;
    }
    if (!client.connected) {
      setStatus('closed', t('msg.rosNotConnected'));
      return;
    }
    if (!controlAllowed(true)) return;
    if (!latestJointPositions || performance.now() - latestJointStateAt > 500) {
      setMessage('等待 /joint_states 反馈后再开始真机示教');
      return;
    }

    hardwareTeachBusy = true;
    hardwareTeachModeConfirmed = false;
    updateHardwareTeachUi('正在进入重力补偿');
    try {
      const result = await guardedCall(
        () => client.startGravityCompensation(),
        '正在进入重力补偿以开始推动示教',
        false
      );
      if (!result || result.success === false || result.accepted === false) return;
      hardwareTeachActive = true;
      hardwareTeachGravityStarted = true;
      if (!els.mirror.checked) els.mirror.checked = true;
      window.reBotSim.beginHardwareTeaching();
      writeLog('真机推动示教开始：直接记录 /joint_states 原始位置', 'ok');
    } finally {
      hardwareTeachBusy = false;
      updateHardwareTeachUi();
    }
  }

  async function stopHardwareTeaching(stopGravity, options) {
    if (!hardwareTeachActive) return;
    hardwareTeachActive = false;
    hardwareTeachModeConfirmed = false;
    if (window.reBotSim && typeof window.reBotSim.endHardwareTeaching === 'function') {
      window.reBotSim.endHardwareTeaching();
    }
    const shouldStopGravity = Boolean(stopGravity) && hardwareTeachGravityStarted && client.connected;
    const shouldReturnHome = Boolean(options && options.returnHome) && client.connected;
    hardwareTeachGravityStarted = false;
    hardwareTeachBusy = shouldStopGravity || shouldReturnHome;
    updateHardwareTeachUi(shouldReturnHome ? t('sim.hardwareTeachHoming') : undefined);
    writeLog('真机推动示教录制已停止', 'info');
    try {
      let gravityStopped = true;
      if (shouldStopGravity) {
        const result = await guardedCall(
          () => client.stopGravityCompensation(),
          '正在退出重力补偿',
          true,
          { keepConnectionStatus: true }
        );
        gravityStopped = Boolean(
          result && result.success !== false && result.accepted !== false
        );
      }
      if (shouldReturnHome && gravityStopped && client.connected) {
        cancelLowLevelPlayback();
        resetWebControlState();
        await guardedCall(
          () => client.safeHome(),
          t('msg.teachSafeHome'),
          true,
          { keepConnectionStatus: true }
        );
        resetWebControlState();
      }
    } finally {
      hardwareTeachBusy = false;
      updateHardwareTeachUi();
    }
  }

  function finishHardwareTeachLocal(message) {
    if (!hardwareTeachActive) return;
    hardwareTeachActive = false;
    hardwareTeachGravityStarted = false;
    hardwareTeachModeConfirmed = false;
    if (window.reBotSim && typeof window.reBotSim.endHardwareTeaching === 'function') {
      window.reBotSim.endHardwareTeaching();
    }
    updateHardwareTeachUi();
    writeLog(message || '真机推动示教已停止', 'warn');
  }

  function updateHardwareTeachUi(busyText) {
    if (!els.teachHardwareRecord) return;
    els.teachHardwareRecord.disabled = hardwareTeachBusy;
    els.teachHardwareRecord.classList.toggle('active', hardwareTeachActive);
    els.teachHardwareRecord.textContent = hardwareTeachActive
      ? t('sim.stopHardwareRecord')
      : (busyText || t('teach.hardwareRecord'));
  }

 function forwardSimCommand(command) {
   if (hardwareTeachActive) {
     writeLog('真机推动示教中，已忽略网页控制指令', 'warn');
     return;
   }
   if (command && command.type === 'execute-current-pose') {
     void executeCurrentPoseCommand(command);
     return;
   }
   if (command && command.type === 'teaching-replay') {
     forwardTeachingReplay(command);
     return;
   }
   if (command && command.type === 'tcp-drag-start') {
     beginTcpDragHandoff();
     return;
   }
   if (command && command.type === 'tcp-target') {
     forwardTcpTarget(command);
     return;
   }
   if (command && command.type === 'joint-batch') {
     forwardJointBatch(command);
     return;
   }
   if (!command || command.type !== 'joint') return;
   lastObservedSimCommand.set(command.name, {
     value: command.value,
     phase: command.phase || 'commit',
     at: performance.now()
   });
   simTargetAngles.set(command.name, command.value);
   mirrorHoldUntil.set(command.name, performance.now() + MIRROR_HOLD_MS);

   if (els.mirror.checked && !els.control.checked && command.source === 'slider') {
     els.mirror.checked = false;
     writeLog('已暂停 ROS 镜像，避免旧反馈把滑块拉回', 'warn');
   }

  if (!controlAllowed(false)) {
    return;
  }

   if (String(command.source || '').startsWith('slider')) {
     if (command.name === 'gripper') {
       forwardGripperSliderCommand(command);
       if (command.phase === 'commit') {
         writeLog(`gripper 直接目标 ${(command.value * 1000).toFixed(0)} mm`, 'info');
       }
       return;
     }
     if (TARGET_KEY === 'hardware' && command.name !== 'gripper') {
       hardwareSliderJoints.add(command.name);
     }
     queueDampedSliderCommand(command);
     if (command.phase === 'commit') {
       const value = command.name === 'gripper'
         ? `${(command.value * 1000).toFixed(0)} mm`
         : `${(command.value * 180 / Math.PI).toFixed(1)}°`;
       writeLog(`${command.name} 阻尼目标 ${value}`, 'info');
     }
     return;
   }
   cancelHardwareSliderFeedback(command.name);
   dampedSliderCommands.delete(command.name);

   const now = performance.now();
   const last = lastSent.get(command.name) || 0;
   // The change event is the final slider value and must never be throttled.
   // Otherwise ROS can stop at the penultimate input event's position.
   if (command.phase !== 'commit' && now - last < COMMAND_INTERVAL_MS) return;
   lastSent.set(command.name, now);

   if (command.name === 'gripper') {
     publishGripperWidthCommand(command.value);
     lastPublishedCommandValues.set(command.name, command.value);
     writeLog(t('log.gripperCmdShort', { mm: (command.value * 1000).toFixed(0) }), 'info');
     return;
   }
   client.publishJointCommand(command.name, command.value, { vlim: getVlim() });
   lastPublishedCommandValues.set(command.name, command.value);
   if (command.phase === 'commit') {
     writeLog(`${command.name} 指令 ${(command.value * 180 / Math.PI).toFixed(1)} 度`, 'info');
   }
 }

  async function executeCurrentPoseCommand(command) {
    if (!controlAllowed(true)) return;
    cancelLowLevelPlayback();
    cancelPendingWebMotionCommands();
    cancelDampedSliderCommands();
    const requested = command && command.joints && typeof command.joints === 'object'
      ? command.joints
      : {};
    const start = getCurrentRosPositions();
    const goal = JOINT_NAMES.map((name, index) => {
      const value = Number(requested[name]);
      return Number.isFinite(value) ? value : start[index];
    });
    const label = command.label || t('adv.plan');
    const points = buildZeroToCurrentPosePoints(start, goal, getTrajectoryDuration());
    await sendTrajectory(points, label);
  }

  function buildZeroToCurrentPosePoints(start, goal, segmentDuration) {
    const duration = clamp(Number(segmentDuration) || 2, 1, 30);
    const zero = JOINT_NAMES.map(() => 0);
    const toZero = buildSmoothJointMovePoints(start, zero, duration);
    const fromZero = buildSmoothJointMovePoints(zero, goal, duration)
      .slice(1)
      .map((point) => ({
        ...point,
        time_from_start: secondsToRosTime(
          duration + rosTimeToSeconds(point.time_from_start)
        )
      }));
    return [...toZero, ...fromZero];
  }

  function beginTcpDragHandoff() {
    if (TARGET_KEY !== 'hardware') return;
    // A preset and a teaching replay use the shared batch renderer, whose
    // forceFeedback frames intentionally bypass the normal "dragging" guard.
    // Stop that owner before TCP IK takes control, otherwise every display
    // frame can overwrite the solver's newly calculated joint angles.
    hardwareBatchFeedbackActive = false;
    if (hardwareBatchFinishTimer) window.clearTimeout(hardwareBatchFinishTimer);
    hardwareBatchFinishTimer = 0;
    resetFeedbackRenderer();
    cancelDampedSliderCommands();
    simTargetAngles.clear();
    mirrorHoldUntil.clear();
    // The command event is synchronous.  Apply the newest measured pose now,
    // before rebot-sim captures currentAngles and creates the new drag plane.
    // This matters when TCP mode stayed enabled across a preset motion.
    if (
      latestJointPositions &&
      window.reBotSim &&
      typeof window.reBotSim.setAngles === 'function'
    ) {
      window.reBotSim.setAngles(latestJointPositions, {
        source: 'ros',
        forceFeedback: true,
        preserveGhost: false,
        preserveControls: false
      });
    }
    // Do not let a low-pass state from before the preset pull the second drag
    // back toward the old pose when the next hardware sample arrives.
    hardwareFeedbackFilters.clear();
    if (window.reBotSim && typeof window.reBotSim.setGhostVisible === 'function') {
      window.reBotSim.setGhostVisible(false);
    }
  }

  function forwardGripperSliderCommand(command) {
    const width = clamp(Number(command && command.value), CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
    if (!Number.isFinite(width)) return;
    pendingGripperSliderWidth = width;

    // Pointer events may arrive much faster than ROS/CAN can consume them.
    // Keep only the newest J7 width and publish at most once per display
    // frame.  There is no value deadband or millimetre quantisation, and the
    // final change event is always sent immediately.
    if ((command.phase || 'input') === 'commit') {
      flushGripperSliderCommand();
      return;
    }
    if (!gripperSliderPublishFrame) {
      gripperSliderPublishFrame = window.requestAnimationFrame(() => {
        gripperSliderPublishFrame = 0;
        flushGripperSliderCommand();
      });
    }
  }

  function flushGripperSliderCommand() {
    if (gripperSliderPublishFrame) {
      window.cancelAnimationFrame(gripperSliderPublishFrame);
      gripperSliderPublishFrame = 0;
    }
    const width = pendingGripperSliderWidth;
    pendingGripperSliderWidth = null;
    if (!Number.isFinite(width) || !client.connected || !controlAllowed(false)) return;
    const now = performance.now();
    publishGripperWidthCommand(width);
    lastSent.set('gripper', now);
    lastPublishedCommandValues.set('gripper', width);
  }

  function forwardTeachingReplay(command) {
    const waypoints = Array.isArray(command && command.waypoints)
      ? command.waypoints.filter((point) => point && point.joints)
      : [];
    const endpointOnly = command && command.mode === 'endpoint';
    if ((!endpointOnly && waypoints.length < 2) || (endpointOnly && !waypoints.length) || !controlAllowed(false)) return;
    const finalJoints = waypoints[waypoints.length - 1].joints;
    if (TARGET_KEY === 'hardware') {
      Object.entries(finalJoints).forEach(([name, value]) => {
        if (JOINT_NAMES.includes(name) && Number.isFinite(Number(value))) {
          simTargetAngles.set(name, Number(value));
        }
      });
    }
    setHardwareTargetGhost(finalJoints);
    if (hardwareBatchFinishTimer) window.clearTimeout(hardwareBatchFinishTimer);
    hardwareBatchFinishTimer = 0;
    hardwareBatchFeedbackActive = TARGET_KEY === 'hardware' && els.mirror.checked;
    if (!hardwareBatchFeedbackActive) resetFeedbackRenderer();
    if (typeof command.claim === 'function') {
      command.claim({
        feedbackDriven: hardwareBatchFeedbackActive
      });
    }
    void runTeachingReplayOnRos(command, waypoints);
  }

  function setHardwareTargetGhost(joints) {
    if (
      TARGET_KEY !== 'hardware' ||
      !window.reBotSim ||
      typeof window.reBotSim.setFeedbackTarget !== 'function'
    ) return;
    window.reBotSim.setFeedbackTarget(joints);
  }

  async function runTeachingReplayOnRos(command, waypoints) {
    cancelLowLevelPlayback();
    cancelPendingWebMotionCommands();
    const endpointOnly = command && command.mode === 'endpoint';
    const lastTime = Number(waypoints[waypoints.length - 1].t) || 0;
    const recordedDuration = endpointOnly ? TEACH_ENDPOINT_DURATION_S : Math.max(0.4, lastTime / 1000);
    let points;
    if (endpointOnly) {
      const start = getCurrentRosPositions();
      const finalJoints = waypoints[waypoints.length - 1].joints;
      const goal = JOINT_NAMES.map((name, index) => {
        const value = Number(finalJoints[name]);
        return Number.isFinite(value) ? value : start[index];
      });
      points = buildSmoothJointMovePoints(start, goal, TEACH_ENDPOINT_DURATION_S);
    } else {
      points = buildTeachingTrajectoryPoints(waypoints);
    }
    const label = endpointOnly
      ? '真机示教最终位置（3.0 秒）'
      : `TCP 示教回放（${waypoints.length} 点 / ${recordedDuration.toFixed(1)} 秒）`;
    let success = false;
    let message = '真机示教回放失败，请查看 ROS 日志';
    try {
      const result = await sendTrajectory(
        points,
        label,
        endpointOnly ? undefined : { profile: 'teaching-replay' }
      );
      success = result === true || Boolean(
        result && result.success !== false && result.accepted !== false
      );
      if (success) message = endpointOnly ? '已用 3 秒运动到示教最终位置' : '真机示教回放完成';
    } catch (error) {
      writeLog(`真机示教回放失败：${error && error.message ? error.message : error}`, 'error');
    } finally {
      hardwareBatchFeedbackActive = false;
      if (hardwareBatchFinishTimer) window.clearTimeout(hardwareBatchFinishTimer);
      hardwareBatchFinishTimer = 0;
      resetFeedbackRenderer();
      if (window.reBotSim && typeof window.reBotSim.setGhostVisible === 'function') {
        window.reBotSim.setGhostVisible(false);
      }
    }
    if (typeof command.complete === 'function') command.complete(success, message);
  }

  function queueDampedSliderCommand(command) {
    const name = command.name;
    const desired = Number(command.value);
    if (!Number.isFinite(desired)) return;
    const now = performance.now();
    let state = dampedSliderCommands.get(name);
    if (
      command.phase !== 'commit' &&
      name !== 'gripper' &&
      state &&
      Math.abs(desired - state.desired) < SLIDER_INPUT_DEADBAND_RAD
    ) return;
    if (!state) {
      const feedback = name === 'gripper'
        ? latestGripperPosition
        : latestJointPositions && latestJointPositions[name];
      const lastPublished = lastPublishedCommandValues.get(name);
      const initial = Number.isFinite(lastPublished)
        ? lastPublished
        : Number.isFinite(feedback)
        ? feedback
        : desired;
      state = {
        current: initial,
        desired,
        lastTick: now,
        lastPublish: 0,
        committedAt: command.phase === 'commit' ? now : 0
      };
      dampedSliderCommands.set(name, state);
    } else {
      state.desired = desired;
      if (command.phase === 'commit') state.committedAt = now;
    }
    if (!sliderDampingFrame) {
      sliderDampingFrame = window.requestAnimationFrame(runSliderDampingFrame);
    }
  }

  function runSliderDampingFrame(timestamp) {
    sliderDampingFrame = 0;
    if (!client.connected || !controlAllowed(false)) {
      cancelDampedSliderCommands();
      return;
    }

    const dampingMs = getSliderDampingMs();
    dampedSliderCommands.forEach((state, name) => {
      const dtMs = Math.max(1, Math.min(50, timestamp - state.lastTick));
      state.lastTick = timestamp;
      const alpha = dampingMs <= 0 ? 1 : 1 - Math.exp(-dtMs / dampingMs);
      state.current += (state.desired - state.current) * alpha;
      const epsilon = name === 'gripper' ? 0.0002 : 0.0015;
      const commitDeadlineReached = state.committedAt > 0 &&
        timestamp - state.committedAt >= Math.max(120, dampingMs * 4);
      const settled = Math.abs(state.desired - state.current) <= epsilon || commitDeadlineReached;
      if (settled) state.current = state.desired;

      if (timestamp - state.lastPublish >= COMMAND_INTERVAL_MS || settled) {
        if (name === 'gripper') {
          publishGripperWidthCommand(state.current);
        } else {
          client.publishJointCommand(name, state.current, { vlim: getVlim() });
        }
        state.lastPublish = timestamp;
        lastSent.set(name, timestamp);
        lastPublishedCommandValues.set(name, state.current);
      }
      if (settled) dampedSliderCommands.delete(name);
    });

    if (dampedSliderCommands.size) {
      sliderDampingFrame = window.requestAnimationFrame(runSliderDampingFrame);
    }
  }

  function cancelDampedSliderCommands() {
    dampedSliderCommands.clear();
    if (sliderDampingFrame) {
      window.cancelAnimationFrame(sliderDampingFrame);
      sliderDampingFrame = 0;
    }
  }

  function cancelPendingWebMotionCommands() {
    cancelDampedSliderCommands();
    pendingGripperSliderWidth = null;
    if (gripperSliderPublishFrame) {
      window.cancelAnimationFrame(gripperSliderPublishFrame);
      gripperSliderPublishFrame = 0;
    }
    if (gripperRepublishTimer) {
      window.clearTimeout(gripperRepublishTimer);
      gripperRepublishTimer = 0;
    }
  }

  function resetWebControlState() {
    cancelPendingWebMotionCommands();
    cancelHardwareSliderFeedback();
    simTargetAngles.clear();
    mirrorHoldUntil.clear();
    lastPublishedCommandValues.clear();
    lastObservedSimCommand.clear();
    lastSent.clear();
    hardwareBatchFeedbackActive = false;
    if (hardwareBatchFinishTimer) window.clearTimeout(hardwareBatchFinishTimer);
    hardwareBatchFinishTimer = 0;
    hardwareFeedbackFilters.clear();
    if (window.reBotSim && typeof window.reBotSim.setGhostVisible === 'function') {
      window.reBotSim.setGhostVisible(false);
    }
  }

  function bindJointSliderFallback() {
    const names = [...JOINT_NAMES, 'gripper'];
    const sliders = names.map((name) => document.getElementById(name));
    if (sliders.some((slider) => !slider)) {
      window.setTimeout(bindJointSliderFallback, 50);
      return;
    }

    names.forEach((name, index) => {
      const slider = sliders[index];
      if (slider.dataset.rosFallbackBound === '1') return;
      slider.dataset.rosFallbackBound = '1';

      const forward = (phase) => {
        const raw = Number(slider.value);
        if (!Number.isFinite(raw)) return;
        const value = name === 'gripper' ? raw / 1000 : raw * Math.PI / 180;
        const observed = lastObservedSimCommand.get(name);
        const alreadyForwarded = observed &&
          performance.now() - observed.at < 20 &&
          observed.phase === phase &&
          Math.abs(observed.value - value) < 1e-9;
        if (alreadyForwarded) return;

        forwardSimCommand({
          type: 'joint',
          name,
          value,
          source: 'slider-dom-fallback',
          phase,
          stamp: performance.now()
        });
      };

      slider.addEventListener('input', () => forward('input'));
      slider.addEventListener('change', () => forward('commit'));
    });

    writeLog('关节滑块 ROS 发布通道已就绪', 'ok');
  }

  function forwardTcpTarget(command) {
    if (!command || !command.target_ros || !controlAllowed(false)) return;
    cancelHardwareSliderFeedback();
    cancelDampedSliderCommands();
    const now = performance.now();
    if (now - lastTargetPoseSent < COMMAND_INTERVAL_MS) return;
    lastTargetPoseSent = now;
    client.publishTargetPose({
      position: command.target_ros,
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    });
  }

  function forwardJointBatch(command) {
    cancelDampedSliderCommands();
    const joints = command && command.joints && typeof command.joints === 'object' ? command.joints : {};
    const names = [...JOINT_NAMES, 'gripper'].filter((name) => typeof joints[name] === 'number' && Number.isFinite(joints[name]));
    if (!names.length) return;

    const handoffToFeedback = Boolean(
      command.handoffToFeedback && TARGET_KEY === 'hardware'
    );
    if (!handoffToFeedback) {
      const holdUntil = performance.now() + MIRROR_HOLD_MS;
      names.forEach((name) => {
        simTargetAngles.set(name, joints[name]);
        mirrorHoldUntil.set(name, holdUntil);
      });
    }

    if (!controlAllowed(false)) return;

    if (
      TARGET_KEY === 'hardware' &&
      els.mirror.checked &&
      command.source === 'preset'
    ) {
      if (hardwareBatchFinishTimer) window.clearTimeout(hardwareBatchFinishTimer);
      hardwareBatchFinishTimer = 0;
      resetFeedbackRenderer();
      hardwareBatchFeedbackActive = true;
      if (latestJointPositions) queueHardwareFeedbackFrame(latestJointPositions);
    }

    names.forEach((name) => {
      lastSent.set(name, 0);
      if (name === 'gripper') {
        publishGripperWidthCommand(joints[name]);
      } else {
        client.publishJointCommand(name, joints[name], { vlim: getVlim() });
      }
    });
    if (handoffToFeedback) {
      // Do not reset the feedback interpolator here: it is already rendering
      // the solid robot smoothly.  Only retire the transient TCP target so a
      // following feedback frame cannot make the ghost visible again.
      names.forEach((name) => {
        simTargetAngles.delete(name);
        mirrorHoldUntil.delete(name);
      });
      // Feedback received while the pointer was down describes older physical
      // positions.  Discard those transitions so they cannot replay once the
      // drag guard is removed; the next ROS sample starts a fresh transition.
      cancelHardwareSliderFeedback();
      if (window.reBotSim && typeof window.reBotSim.setGhostVisible === 'function') {
        window.reBotSim.setGhostVisible(false);
      }
    }
    writeLog(t('log.jointBatch', { label: command.label || command.source || t('log.batchDefault'), n: names.length }), 'ok');
  }

 async function checkIk() {
   if (!controlAllowed(true)) return;
   cancelDampedSliderCommands();
   const pose = readPose();
    const duration = getPoseDuration();
    client.publishTargetPose(pose);
    await sendVisionMoveGoal(
      pose,
      duration,
      t('msg.reqIkMove', { sec: duration.toFixed(1) })
    );
  }

  async function disconnectRos() {
    if (safeDisconnectBusy) return;
    await stopHardwareTeaching(true);
    cancelLowLevelPlayback();
    resetWebControlState();

    if (!els.safeDisconnect || !els.safeDisconnect.checked || !client.connected) {
      client.disconnect();
      return;
    }

    safeDisconnectBusy = true;
    els.disconnect.disabled = true;
    try {
      setMessage(t('msg.disconnectHome'));
      writeLog(t('log.disconnectHomeStart'), 'info');
      // The controller's disable service performs a fresh six-joint zero-pose
      // check, safe-homes only when needed, and aborts disable on home failure.
      const disableResult = await client.disable();
      if (!disableResult || disableResult.success === false) {
        throw new Error(disableResult && disableResult.message
          ? disableResult.message
          : t('msg.disableFail'));
      }
      writeLog(disableResult.message || t('log.disconnectDisableDone'), 'ok');
      setMessage(t('msg.disconnecting'));
      client.disconnect();
    } catch (error) {
      const message = t('msg.disconnectGuardFail', {
        err: error && error.message ? error.message : error
      });
      setMessage(message);
      writeLog(message, 'error');
    } finally {
      safeDisconnectBusy = false;
      if (client.connected) els.disconnect.disabled = false;
    }
  }

  async function queryGravityCompensation(options) {
    const result = await guardedOptionalService(
      REQUIRED_SERVICES.gravityStatus,
      () => client.gravityCompensationStatus(),
      t('msg.reqGravityQuery'),
      true,
      options
    );
    if (result) updateGravityStatus(Boolean(result.success), result.message || '', 'service');
  }

  async function pollGravityCompensationStatus() {
    if (
      !client.connected ||
      !gravityCompensationActive ||
      visionSequenceBusy ||
      gravityStatusPollInFlight
    ) return;

    gravityStatusPollInFlight = true;
    try {
      await queryGravityCompensation({ silent: true });
    } finally {
      gravityStatusPollInFlight = false;
    }
  }

  async function runDiagnostics() {
    updateDiagnostics();
    if (!client.connected) {
      writeLog('rosbridge 离线，请先连接', 'warn');
      return;
    }
    try {
      const topics = await client.getRosTopics();
      const services = await client.getRosServices();
      const topicList = topics.topics || [];
      const serviceList = services.services || [];
      listedTopics = new Set(topicList);
      listedServices = new Set(serviceList);
      writeLog(`rosapi: ${topicList.length} topics, ${serviceList.length} services`, 'ok');
      if (els.visionStatus && !topicList.includes(REQUIRED_TOPICS.visionDetections)) {
        els.visionStatus.textContent = t('st.waitNode');
      }
      if (!listedServices.has(REQUIRED_SERVICES.gravityStatus)) {
        updateGravityStatus(false, t('st.serviceUnavailable'));
      }
      if (TARGET_KEY === 'simulation' && !hasActionService(`/${NS}/follow_joint_trajectory`)) {
        writeLog('已检测到仿真驱动，轨迹按钮将使用低层回放', 'info');
      }
    } catch (error) {
      writeLog(t('log.rosapiFallback', { err: error.message || error }), 'warn');
    }
  }

  function buildTrajectoryPoints(waypoints, totalDuration) {
    const firstT = waypoints[0].t || 0;
    const lastT = waypoints[waypoints.length - 1].t || firstT + 1;
    const span = Math.max(lastT - firstT, 1);
    const points = [makeTrajectoryPoint(getCurrentRosPositions(), 0.05)];
    let previousSeconds = 0.05;
    waypoints.forEach((point, index) => {
      const ratio = waypoints.length === 1 ? 1 : Math.max(0, (point.t - firstT) / span);
      const desiredSeconds = 0.15 + (
        index === waypoints.length - 1 ? totalDuration : ratio * totalDuration
      );
      const seconds = Math.max(previousSeconds + 0.02, desiredSeconds);
      points.push(makeTrajectoryPoint(JOINT_NAMES.map((name) => Number(point.joints[name] || 0)), seconds));
      previousSeconds = seconds;
    });
    return points;
  }

  function buildTeachingTrajectoryPoints(waypoints) {
    // The browser playback clock starts at t=0 from its current pose. Use the
    // same origin for the action instead of adding a hidden 50 ms hold.
    const points = [makeTrajectoryPointAtNs(getCurrentRosPositions(), 0n)];
    const firstStampNs = validRosStamp(waypoints[0] && waypoints[0].stamp)
      ? rosStampToNs(waypoints[0].stamp)
      : null;
    const leadNs = millisecondsToNs(waypoints[0] ? waypoints[0].t : 0);
    let previousNs = 0n;
    waypoints.forEach((point) => {
      const pointStampNs = validRosStamp(point.stamp) ? rosStampToNs(point.stamp) : null;
      let relativeNs;
      if (pointStampNs !== null && firstStampNs !== null) {
        relativeNs = pointStampNs - firstStampNs;
      } else if (validRosStamp(point.time_from_start)) {
        relativeNs = rosStampToNs(point.time_from_start);
      } else {
        relativeNs = millisecondsToNs(point.t);
      }
      const desiredNs = leadNs + relativeNs;
      const timeNs = desiredNs > previousNs ? desiredNs : previousNs + 1n;
      points.push(makeTrajectoryPointAtNs(
        JOINT_NAMES.map((name) => Number(point.joints[name]) || 0),
        timeNs
      ));
      previousNs = timeNs;
    });
    return points;
  }

  async function sendTrajectory(points, optimisticMessage, options) {
    if (!points.length) return;
    if (shouldUseLowLevelTrajectory()) {
      setMessage(t('msg.simLowLevelSuffix', { label: optimisticMessage }));
      writeLog(t('log.lowLevelSuffix', { label: optimisticMessage }), 'info');
      return replayTrajectoryLowLevel(points);
    }
    if (TARGET_KEY !== 'hardware' && !hasActionService(`/${NS}/follow_joint_trajectory`)) {
      writeLog('未发现 FollowJointTrajectory 动作，改用低层回放', 'warn');
      return replayTrajectoryLowLevel(points);
    }
    return guardedCall(
      () => client.followJointTrajectory(JOINT_NAMES, points, options),
      optimisticMessage
    );
  }

  async function replayTrajectoryLowLevel(points) {
    cancelLowLevelPlayback();
    const playback = { cancelled: false };
    lowLevelPlayback = playback;
    const started = performance.now();
    writeLog(t('log.lowLevelStart', { n: points.length }), 'ok');
    for (const point of points) {
      if (playback.cancelled || !controlAllowed(false)) break;
      const targetMs = rosTimeToSeconds(point.time_from_start) * 1000;
      const waitMs = Math.max(0, targetMs - (performance.now() - started));
      if (waitMs > 0) await sleep(waitMs);
      JOINT_NAMES.forEach((name, index) => {
        const pos = Number(point.positions[index]);
        if (Number.isFinite(pos)) {
          simTargetAngles.set(name, pos);
          client.publishJointCommand(name, pos, { vlim: getVlim() });
        }
      });
      syncSimArmFromTrajectoryPoint(point);
    }
    if (lowLevelPlayback === playback) lowLevelPlayback = null;
    writeLog(playback.cancelled ? t('log.lowLevelCancelled') : t('log.lowLevelDone'), playback.cancelled ? 'warn' : 'ok');
    return !playback.cancelled;
  }

  function syncSimArmFromTrajectoryPoint(point) {
    if (!window.reBotSim || typeof window.reBotSim.setAngles !== 'function') return;
    // While MuJoCo feedback is fresh in simulation, let that feedback drive
    // the web arm; writing every playback waypoint on top of it causes jitter.
    if (TARGET_KEY === 'simulation' && els.mirror.checked && mujocoStateIsFresh()) return;
    const angles = {};
    JOINT_NAMES.forEach((name, index) => {
      const pos = Number(point.positions[index]);
      if (Number.isFinite(pos)) angles[name] = pos;
    });
    if (Object.keys(angles).length) {
      window.reBotSim.setAngles(angles, { source: 'trajectory-playback' });
    }
  }

  function cancelLowLevelPlayback() {
    if (lowLevelPlayback) lowLevelPlayback.cancelled = true;
  }

  function shouldUseLowLevelTrajectory() {
    // rosapi/services may omit ROS 2 hidden action services.  The real RS
    // controller exposes FollowJointTrajectory even when rosapi omits it.
    if (TARGET_KEY === 'hardware') return false;
    return !hasActionService(`/${NS}/follow_joint_trajectory`);
  }

  function hasActionService(actionName) {
    return listedServices.has(`${actionName}/_action/send_goal`);
  }

  function makeTrajectoryPoint(positions, seconds) {
    return makeTrajectoryPointAtNs(positions, secondsToNs(seconds));
  }

  function makeTrajectoryPointAtNs(positions, timeNs) {
    return {
      positions,
      velocities: JOINT_NAMES.map(() => 0),
      accelerations: [],
      effort: [],
      time_from_start: nsToRosStamp(timeNs)
    };
  }

  function validRosStamp(stamp) {
    return rosStampToNs(stamp) !== null;
  }

  function rosStampToNs(stamp) {
    const sec = Number(stamp && stamp.sec);
    const nanosec = Number(stamp && stamp.nanosec);
    if (!Number.isInteger(sec) || !Number.isInteger(nanosec) || sec < 0 || nanosec < 0 || nanosec > 999999999) {
      return null;
    }
    return BigInt(sec) * 1000000000n + BigInt(nanosec);
  }

  function nsToRosStamp(ns) {
    const normalized = ns < 0n ? 0n : ns;
    return {
      sec: Number(normalized / 1000000000n),
      nanosec: Number(normalized % 1000000000n)
    };
  }

  function millisecondsToNs(milliseconds) {
    const value = Math.max(0, Number(milliseconds) || 0);
    return BigInt(Math.round(value * 1e6));
  }

  function getCurrentRosPositions() {
    const source = latestJointPositions || (window.reBotSim && window.reBotSim.getAngles ? window.reBotSim.getAngles() : {});
    return JOINT_NAMES.map((name) => Number(source[name] || 0));
  }

  function getTeachWaypoints() {
    if (!window.reBotSim || typeof window.reBotSim.getTeachingWaypoints !== 'function') return [];
    return window.reBotSim.getTeachingWaypoints().filter((point) => point && point.joints);
  }

  function readPose() {
    return {
      position: {
        x: Number(els.poseX.value) || 0,
        y: Number(els.poseY.value) || 0,
        z: Number(els.poseZ.value) || 0
      },
      orientation: { x: 0, y: 0, z: 0, w: 1 }
    };
  }

 function controlAllowed(interactive) {
   if (!client.connected) {
     if (interactive) setStatus('closed', t('msg.rosNotConnected'));
     return false;
   }
   // RS Fake Driver is auto-allowed in simulation mode.
   if (TARGET_KEY === 'simulation' && rosBackend === 'fake-rs') return true;
    if (!els.control.checked) {
      if (interactive) setMessage(t('msg.controlLockClosed'));
      return false;
    }
    return true;
  }

  function updateRosBackend(mode) {
    const text = String(mode || '').toLowerCase();
    let nextBackend = 'real-or-unknown';
    if (text.startsWith('fake_rs_')) {
      nextBackend = TARGET_KEY === 'simulation' ? 'fake-rs' : 'fake-other';
    } else if (text.startsWith('fake_') || TARGET_KEY === 'simulation') {
      nextBackend = 'fake-other';
    }
    if (nextBackend === rosBackend) return;

    rosBackend = nextBackend;
    if (nextBackend === 'fake-rs') {
      els.control.checked = true;
      els.control.disabled = true;
      if (els.controlLabel) els.controlLabel.textContent = 'RS 仿真：已自动允许网页控制';
      writeLog('已识别 RS Fake Driver，滑块控制已自动解锁', 'ok');
      return;
    }

    // For hardware mode, respect the user's manual choice — never auto-uncheck.
    // Only disable control if the target/driver is mismatched (fake-other).
    els.control.disabled = nextBackend === 'fake-other';
    if (els.controlLabel) {
      els.controlLabel.textContent = nextBackend === 'fake-other'
        ? '控制目标与 ROS 驱动不匹配，已禁止控制'
        : '允许网页向 RS 真机发控制';
    }
    if (nextBackend === 'fake-other') {
      writeLog('控制目标与 ROS 驱动不匹配，已阻止滑块命令', 'error');
    }
  }

  function canConnectWebSocketUrl(url) {
    if (window.location.protocol === 'https:' && /^ws:\/\//i.test(url)) {
      const message = t('msg.httpsWsBlocked');
      setStatus('error', message);
      writeLog(message, 'error');
      return false;
    }
    return true;
  }

  async function guardedOptionalService(serviceName, call, optimisticMessage, allowWithoutControl, options) {
    if (listedServices.size && !listedServices.has(serviceName)) {
      const message = t('msg.serviceNotFound', { name: serviceName });
      updateGravityStatus(false, t('st.serviceUnavailable'));
      setMessage(message);
      if (!(options && (options.auto || options.silent))) writeLog(message, 'warn');
      return null;
    }
    return guardedCall(call, optimisticMessage, allowWithoutControl, {
      keepConnectionStatus: true,
      silent: Boolean(options && options.silent)
    });
  }

 async function guardedCall(call, optimisticMessage, allowWithoutControl, options) {
   if (!client.connected) {
     setStatus('closed', t('msg.rosNotConnected'));
     return null;
   }
    if (!allowWithoutControl && !controlAllowed(false)) {
      setMessage(t('msg.controlLockClosed'));
      return null;
    }
    try {
      if (!(options && options.silent)) {
        setMessage(optimisticMessage);
        writeLog(optimisticMessage, 'info');
      }
      const result = await call();
      const message = formatServiceResult(result);
      if (!(options && options.silent)) {
        setMessage(message);
        const failed = result && (result.accepted === false || result.success === false);
        writeLog(message, failed ? 'error' : 'ok');
      }
      return result;
    } catch (error) {
      const message = error && error.message ? error.message : t('log.rosCallFail');
      if (options && options.silent) return null;
      if (options && options.keepConnectionStatus && client.connected) {
        setMessage(message);
      } else {
        setStatus('error', message);
      }
      writeLog(message, 'error');
      return null;
    }
  }

  function formatServiceResult(result) {
    if (!result) return t('log.rosCallDone');
    if (typeof result.accepted === 'boolean') return result.accepted ? t('msg.goalAccepted') : t('msg.goalRejected');
    if (typeof result.message === 'string' && result.message) return result.message;
    if (typeof result.reached_position === 'number') return t('msg.gripperReached', { mm: Math.round(result.reached_position * 1000) });
    if (Array.isArray(result.q_solution)) return t('log.ikResult', { result: result.success ? '成功' : '失败', q: result.q_solution.map((v) => Number(v).toFixed(3)).join(', ') });
    if (typeof result.success === 'boolean') return result.success ? t('log.rosCallSuccess') : t('log.rosCallFail');
    return t('log.rosCallDone');
  }

  function updateDiagnostics() {
    updateCameraStatusFromTopic();
  }

  function markTopicDiag(el, topic) {
    const last = client.getLastMessageAt(topic);
    if (!client.connected) {
      markDiag(el, false, '--');
      return;
    }
    if (!last) {
      markDiag(el, null, listedTopics.has(topic) ? (topic === REQUIRED_TOPICS.armStatus ? t('st.diagFound') : t('st.diagFoundWait')) : '等待');
      return;
    }
    const age = (Date.now() - last) / 1000;
    const liveLimit = topic === REQUIRED_TOPICS.armStatus ? 90 : topic === REQUIRED_TOPICS.cameraImage ? 3.0 : 2.5;
    markDiag(el, age < liveLimit, `${age.toFixed(1)}s`);
  }

  function markDiag(el, ok, text) {
    if (!el) return;
    const box = el.closest('.diag-item');
    if (box) {
      box.classList.toggle('ok', ok === true);
      box.classList.toggle('warn', ok === null);
      box.classList.toggle('bad', ok === false);
    }
    el.textContent = text;
  }

  function normalizeJointName(name) {
    const text = String(name || '').toLowerCase();
    if (text.endsWith('finger_left') || text.endsWith('/finger_left')) return 'finger_left';
    if (text.endsWith('finger_right') || text.endsWith('/finger_right')) return 'finger_right';
    // These are gripper fingers, not arm joint1/joint2. Check them before the
    // generic arm-joint suffix matcher below.
    if (text.endsWith('gripper_joint1') || text.endsWith('gripper_joint2')) return null;
    const match = text.match(/joint[_-]?([1-6])$/) || text.match(/j([1-6])$/);
    return match ? `joint${match[1]}` : null;
  }

  function handleCameraImage(msg) {
    if (!els.cameraCanvas || !msg) return;
    const width = Number(msg.width) || 0;
    const height = Number(msg.height) || 0;
    if (width <= 0 || height <= 0) {
      setCameraStatus('错误', 'error');
      return;
    }

    const bytes = rosImageBytes(msg.data);
    if (!bytes) {
      setCameraStatus('数据异常', 'error');
      return;
    }

    const encoding = String(msg.encoding || 'rgb8').toLowerCase();
    const channels = encoding === 'rgba8' || encoding === 'bgra8' ? 4 : 3;
    const supported = encoding === 'rgb8' || encoding === 'bgr8' || encoding === 'rgba8' || encoding === 'bgra8';
    if (!supported) {
      setCameraStatus(encoding || t('st.encodingUnsupported'), 'warn');
      return;
    }

    if (els.cameraCanvas.width !== width) els.cameraCanvas.width = width;
    if (els.cameraCanvas.height !== height) els.cameraCanvas.height = height;
    const ctx = els.cameraCanvas.getContext('2d');
    const frame = ctx.createImageData(width, height);
    const dst = frame.data;
    const step = Number(msg.step) || width * channels;
    const bgr = encoding === 'bgr8' || encoding === 'bgra8';

    for (let y = 0; y < height; y += 1) {
      const row = y * step;
      for (let x = 0; x < width; x += 1) {
        const src = row + x * channels;
        const out = (y * width + x) * 4;
        dst[out] = bgr ? bytes[src + 2] : bytes[src];
        dst[out + 1] = bytes[src + 1];
        dst[out + 2] = bgr ? bytes[src] : bytes[src + 2];
        dst[out + 3] = channels === 4 ? bytes[src + 3] : 255;
      }
    }

    ctx.putImageData(frame, 0, 0);
    setCameraStatus(`${width}x${height}`, 'online');
    updateDiagnostics();
  }

  function rosImageBytes(data) {
    if (!data) return null;
    if (Array.isArray(data)) return data;
    if (data instanceof Uint8Array) return data;
    if (typeof data === 'string') {
      try {
        const binary = window.atob(data);
        const bytes = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i += 1) bytes[i] = binary.charCodeAt(i);
        return bytes;
      } catch (error) {
        return null;
      }
    }
    if (data.buffer instanceof ArrayBuffer) return new Uint8Array(data.buffer);
    return null;
  }

  function updateCameraStatusFromTopic() {
    if (!els.cameraStatus) return;
    if (!client.connected) {
      setCameraStatus('离线', 'error');
      return;
    }
    const last = client.getLastMessageAt(REQUIRED_TOPICS.cameraImage);
    if (!last) {
      setCameraStatus(listedTopics.has(REQUIRED_TOPICS.cameraImage) ? t('st.cameraWaitFrame') : t('st.cameraWaitTopic'), 'warn');
      return;
    }
    const age = (Date.now() - last) / 1000;
    if (age > 3.0) {
      setCameraStatus(`${age.toFixed(1)}s`, 'warn');
    }
  }

  function setCameraStatus(text, state) {
    if (!els.cameraStatus) return;
    els.cameraStatus.textContent = text;
    els.cameraStatus.classList.toggle('online', state === 'online');
    els.cameraStatus.classList.toggle('warn', state === 'warn');
    els.cameraStatus.classList.toggle('error', state === 'error');
  }

  function handleVisionDetections(msg) {
    if (!els.visionStatus && !els.visionTarget) return;
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      if (els.visionStatus) els.visionStatus.textContent = t('st.cameraDataError');
      return;
    }

    latestVisionPayload = payload;
    latestVisionAt = performance.now();
    updateSelectedVisionTarget();
    const count = Number(payload.count) || 0;
    const displayedColor = selectedVisionTarget && selectedVisionTarget.color
      ? selectedVisionTarget.color
      : payload.target_color;
    if (els.visionStatus) {
      els.visionStatus.textContent = count
        ? t('fb.visionCount', { count, color: displayedColor || '--' })
        : t('st.visionNone');
    }
  }

  function handleMujocoObjectStates(msg) {
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      return;
    }
    latestMujocoSimulationMode = String(payload.simulation_mode || '').toLowerCase();
    if (window.reBotSim && typeof window.reBotSim.syncMujocoObjectStates === 'function') {
      window.reBotSim.syncMujocoObjectStates(payload.objects || []);
    }
  }

  function handleSimAnimationEvent(msg) {
    let payload = null;
    try {
      payload = JSON.parse(msg && msg.data ? msg.data : '{}');
    } catch (error) {
      writeLog('MCP 动画事件解析失败', 'warn');
      return;
    }

    const event = String(payload.event || payload.action || '').toLowerCase();
    if (event === 'attach_object') {
      const target = payload.target && typeof payload.target === 'object'
        ? payload.target
        : { color: payload.color };
      attachSimCarriedObject(target);
    } else if (event === 'release_object') {
      releaseSimCarriedObject();
    }
  }

  function updateSelectedVisionTarget() {
    const mode = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    const target = mode === 'auto'
      ? (chooseVisionTarget(autoVisionTargetColor) || chooseRandomVisionTarget())
      : chooseVisionTarget(mode);
    selectedVisionTarget = target;
    renderVisionTarget(target);
  }

  function renderVisionTarget(target) {
    if (!els.visionTarget) return;
    if (!target) {
      els.visionTarget.textContent = '--';
      return;
    }
    const approachZ = getVisionApproachZ(target);
    const graspPlan = estimateVisionGraspPlan(target);
    els.visionTarget.textContent = t('fb.visionTarget', { color: target.color, x: Number(target.x).toFixed(3), y: Number(target.y).toFixed(3), z: approachZ.toFixed(3), mm: Math.round(graspPlan.physicalGap * 1000), yaw: Math.round(graspPlan.yawRad * 180 / Math.PI) });
  }

  function chooseRandomVisionTarget() {
    const detections = latestVisionPayload && Array.isArray(latestVisionPayload.detections)
      ? latestVisionPayload.detections
      : [];
    const colors = [...new Set(
      detections
        .filter((item) => item && item.color)
        .map((item) => String(item.color))
    )];
    if (!colors.length) return null;
    const alternatives = colors.filter((color) => color !== autoVisionTargetColor);
    const pool = alternatives.length ? alternatives : colors;
    autoVisionTargetColor = pool[Math.floor(Math.random() * pool.length)];
    return chooseVisionTarget(autoVisionTargetColor);
  }

  function chooseVisionTarget(preferredColor) {
    const detections = latestVisionPayload && Array.isArray(latestVisionPayload.detections)
      ? latestVisionPayload.detections
      : [];
    if (!detections.length) return null;
    const color = preferredColor || (els.visionColor ? String(els.visionColor.value || 'auto') : 'auto');
    if (color && color !== 'auto') {
      return detections.find((item) => item && item.color === color) || null;
    }
    return latestVisionPayload.target || detections[0] || null;
  }

  async function waitForFreshVisionTarget(preferredColor, timeoutMs) {
    const start = performance.now();
    const initialVisionAt = latestVisionAt;
    let target = chooseVisionTarget(preferredColor);
    while (performance.now() - start < timeoutMs) {
      const fresh = latestVisionAt > initialVisionAt || performance.now() - latestVisionAt < 450;
      target = chooseVisionTarget(preferredColor) || target;
      if (target && fresh) return cloneVisionTarget(target);
      await sleep(80);
    }
    return target ? cloneVisionTarget(target) : null;
  }

  function fillPoseFromVisionTarget() {
    const target = selectedVisionTarget || chooseVisionTarget();
    const pose = poseFromVisionTarget(getVisionApproachZ(target), target);
    if (!pose) return;
    writePoseInputs(pose);
    if (client.connected) client.publishTargetPose(pose);
    setMessage(t('msg.visionFillPose'));
    writeLog('视觉目标 -> Pose 输入框', 'ok');
  }

  async function moveAboveVisionTarget() {
    if (!controlAllowed(true)) return;
    const mode = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    const target = mode === 'auto'
      ? (chooseRandomVisionTarget() || selectedVisionTarget || chooseVisionTarget())
      : (selectedVisionTarget || chooseVisionTarget(mode));
    const pose = poseFromVisionTarget(getVisionApproachZ(target), target);
    if (!pose) return;
    if (mode === 'auto') {
      selectedVisionTarget = target;
      renderVisionTarget(target);
      writeLog(`自动模式已选择目标：${target.color}`, 'info');
    }
    const duration = getPoseDuration();
    setVisionBusy(true, 'move');
    try {
      const route = buildVisionTransitRoute(target);
      for (const waypoint of route) {
        await runVisionMoveStep(waypoint.pose, Math.max(1.1, duration * 0.60), waypoint.label);
      }
      await runVisionMoveStep(pose, duration, t('msg.moveAboveTarget', { color: target.color }));
      lastVisionTarget = cloneVisionTarget(target);
    } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionMoveAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      finishVisionSequence();
    }
  }

  function requestedVisionPickColor() {
    const mode = els.visionColor ? String(els.visionColor.value || 'auto') : 'auto';
    if (mode !== 'auto') return mode;
    const target = chooseRandomVisionTarget() || selectedVisionTarget || chooseVisionTarget();
    if (!target) return 'auto';
    selectedVisionTarget = target;
    renderVisionTarget(target);
    return String(target.color || 'auto');
  }

  function queueVisionOperation(operation) {
    queuedVisionOperation = operation;
    const color = operation.type === 'pick' && operation.color && operation.color !== 'auto'
      ? ` ${operation.color}`
      : '';
    const nextLabel = operation.type === 'pick' ? `抓取${color}` : '放置';
    const currentLabel = activeVisionOperation === 'place' ? '放置' : (activeVisionOperation === 'move' ? '移动' : '抓取');
    const message = `当前正在${currentLabel}，已排队下一步：${nextLabel}`;
    setMessage(message);
    writeLog(message, 'info');
    const activeButton = activeVisionOperation === 'place' ? els.visionPlaceDemo : els.visionPickDemo;
    if (activeButton) {
      activeButton.textContent = `${currentLabel}中 · 已排队${nextLabel}`;
      activeButton.title = message;
    }
  }

  function requestVisionPick() {
    const color = requestedVisionPickColor();
    if (visionSequenceBusy) {
      queueVisionOperation({ type: 'pick', color });
      return;
    }
    runVisionPickDemo(color);
  }

  function requestVisionPlace() {
    if (visionSequenceBusy) {
      queueVisionOperation({ type: 'place' });
      return;
    }
    runVisionPlaceDemo();
  }

  function runQueuedVisionOperation() {
    if (visionSequenceBusy || !queuedVisionOperation) return;
    const operation = queuedVisionOperation;
    queuedVisionOperation = null;
    if (operation.type === 'pick') {
      runVisionPickDemo(operation.color);
    } else {
      runVisionPlaceDemo();
    }
  }

  async function runVisionPickDemo(requestedColor) {
    if (visionSequenceBusy) {
      queueVisionOperation({ type: 'pick', color: requestedColor || requestedVisionPickColor() });
      return;
    }
    if (!controlAllowed(true)) return;
    // Claim the sequence before the first await. Previously the 700 ms vision
    // wait left the button active, so repeated clicks launched concurrent IK
    // service calls that all fought over the arm and timed out together.
    setVisionBusy(true, 'pick');
    if (TARGET_KEY === 'simulation' && latestMujocoSimulationMode === 'kinematic') {
      const message = '当前 MuJoCo 为 kinematic 模式，物体没有接触动力学，无法执行物理抓取；请使用 physics 模式重启仿真。';
      setMessage(message);
      writeLog(message, 'warn');
      finishVisionSequence();
      return;
    }
    const preferredColor = requestedColor || (els.visionColor ? String(els.visionColor.value || 'auto') : 'auto');
    let target = await waitForFreshVisionTarget(preferredColor, 700);
    if (preferredColor === 'auto') {
      target = chooseRandomVisionTarget() || target;
      if (target) {
        selectedVisionTarget = target;
        renderVisionTarget(target);
        writeLog(`自动模式已选择目标：${target.color}`, 'info');
      }
    }
    if (!target) {
      setMessage(t('msg.noVisionTarget'));
      finishVisionSequence();
      return;
    }

    let plan = buildVisionPickPlan(target);
    if (!plan) {
      finishVisionSequence();
      return;
    }
    writeLog(
      t('log.visionGraspPose', { color: target.color, mm: Math.round(plan.graspPlan.physicalGap * 1000), yaw: Math.round(plan.graspPlan.yawRad * 180 / Math.PI), lift: plan.firstLiftPose.position.z.toFixed(3), transit: plan.transitPose.position.z.toFixed(3) }),
      'info'
    );

    const duration = getPoseDuration();
    let lastPose = null;
    const runIfNeeded = async (pose, moveDuration, label) => {
      if (lastPose && poseDistance(lastPose, pose) < VISION_POSE_SKIP_M) {
        writeLog(`${label}：已在目标附近，跳过重复移动`, 'info');
        return { success: true, skipped: true };
      }
      const result = await runVisionMoveStep(pose, moveDuration, label);
      lastPose = pose;
      return result;
    };

    try {
      const carriedColor = window.reBotSim && typeof window.reBotSim.getCarriedObject === 'function'
        ? window.reBotSim.getCarriedObject()
        : '';
      const carriedTarget = heldVisionTarget
        || (carriedColor && lastVisionTarget && String(lastVisionTarget.color) === String(carriedColor)
          ? lastVisionTarget
          : null);
      if (carriedTarget && sameVisionTarget(carriedTarget, target)) {
        const message = `当前已经夹持 ${target.color}，如需放下请点击“放置物体”`;
        setMessage(message);
        writeLog(message, 'info');
        return;
      }
      const openGripper = async () => {
        const settings = {
          timeoutMs: 4500,
          minWaitMs: 850,
          tolerance: 0.0035,
          requireReached: true,
          afterMs: 180
        };
        try {
          await commandGripperAndWait(OPEN_GRIPPER_M, t('msg.pickOpenGripper'), settings);
        } catch (error) {
          const measured = readGripperFeedbackPosition(NaN);
          if (Number.isFinite(measured) && Math.abs(measured - OPEN_GRIPPER_M) <= 0.005) {
            writeLog('视觉抓取：夹爪实际已打开，忽略丢失的到位反馈并继续', 'warn');
            return;
          }
          writeLog('视觉抓取：夹爪未打开，当前序列自动重试一次', 'warn');
          await commandGripperAndWait(OPEN_GRIPPER_M, t('msg.pickOpenGripper'), {
            ...settings,
            timeoutMs: 3000,
            minWaitMs: 500
          });
        }
      };

      // Plan from the arm's measured current pose directly to the new target's
      // safe high waypoint.  lastVisionTarget describes history, not a
      // mandatory waypoint; using it here made every target switch revisit the
      // previous object before moving to the requested one.
      const route = buildVisionTransitRoute(target);
      const moveHighRoute = async () => {
        for (const waypoint of route) {
          await runIfNeeded(waypoint.pose, Math.max(1.2, duration * 0.65), waypoint.label);
        }
      };

      if (carriedTarget) {
        // Match the DM hand-off flow: release the previous object at the
        // current safe height, then travel directly to the next target. The
        // explicit Place button remains responsible for descend-and-place.
        writeLog(`准备抓取 ${target.color}：先在当前位置释放 ${carriedTarget.color}，再直接切换目标`, 'info');
        await openGripper();
        releaseSimCarriedObject();
        await moveHighRoute();
      } else {
        // On a normal pick, start opening and moving to the safe high waypoint
        // together. Both must complete before any descent toward the object.
        writeLog('视觉抓取：夹爪打开与高位关节运动并行开始', 'info');
        await Promise.all([openGripper(), moveHighRoute()]);
      }

      const refinedTarget = await waitForFreshVisionTarget(target.color, 420);
      if (refinedTarget && visionTargetShifted(refinedTarget, target, 0.008)) {
        target = refinedTarget;
        plan = buildVisionPickPlan(target);
        if (!plan) return;
        writeLog(t('log.visionRelocate', { color: target.color }), 'info');
        writeLog(
          t('log.visionGraspPose', { color: target.color, mm: Math.round(plan.graspPlan.physicalGap * 1000), yaw: Math.round(plan.graspPlan.yawRad * 180 / Math.PI), lift: plan.firstLiftPose.position.z.toFixed(3), transit: plan.transitPose.position.z.toFixed(3) }),
          'info'
        );
      }

      writePoseInputs(plan.approachPose);
      await runIfNeeded(plan.approachPose, duration, t('msg.pickMoveAbove', { color: target.color }));

      const alignDuration = Math.max(1.0, duration * 0.55);
      await runIfNeeded(plan.verticalAlignPose, alignDuration, t('msg.pickAlign', { color: target.color }));

      const pregraspDuration = Math.max(0.85, duration * 0.45);
      await runIfNeeded(plan.pregraspPose, pregraspDuration, t('msg.pickPreDescend', { color: target.color }));

      const descendDuration = Math.max(1.1, duration * 0.65);
      await runIfNeeded(plan.graspPose, descendDuration, t('msg.pickDescend', { color: target.color }));

      await commandGripperAndWait(plan.graspPlan.command, t('msg.pickSqueeze', { color: target.color }), {
        // The physics gripper closes under actuator force instead of jumping
        // directly to qpos.  A full-open grasp can take more than 4.5 s once
        // the finger collision pieces begin touching the object, so keep the
        // sequence alive until the measured fingers have actually settled.
        timeoutMs: 9000,
        minWaitMs: 850,
        tolerance: 0.006,
        allowContactStop: true,
        // The requested width already includes GRASP_SQUEEZE_M.  Requiring the
        // measured width to stop another 2.5 mm above that target rejects a
        // valid grasp that settles near the requested squeeze.  Physical pick
        // success is verified immediately after the first lift by checking the
        // detected object's real z displacement.
        requireContactStop: false,
        contactTolerance: 0.0025,
        requireSettled: true,
        settleMs: 420,
        // Contact introduces sub-millimetre MuJoCo oscillation even though the
        // object is already firmly pinched. 1.5 mm is still well below the
        // per-sample travel while the 5 rad/s gripper is actually closing.
        positionStableTolerance: 0.0015,
        afterMs: 220
      });
      writeLog(`夹爪已稳定夹紧 ${target.color}，开始离桌抬升`, 'ok');
      const firstLiftDuration = Math.max(1.25, duration * 0.75);
      if (String(target.color || '') === 'blue') {
        await runVisionMoveStep(plan.firstLiftPose, firstLiftDuration, t('msg.pickBlueLift', { z: plan.firstLiftPose.position.z.toFixed(3) }));
        lastPose = plan.firstLiftPose;
      } else {
        await runIfNeeded(plan.firstLiftPose, firstLiftDuration, t('msg.pickLift', { color: target.color }));
      }

      const liftedTarget = await waitForFreshVisionTarget(target.color, 800);
      const liftCheck = verifyVisionLift(target, liftedTarget);
      if (!liftCheck.ok) {
        throw new Error(`视觉抓取失败：物体没有随夹爪抬起（${liftCheck.message}）`);
      }
      attachSimCarriedObject(liftedTarget);
      writeLog(`物理抓取验证通过：物体抬升 ${(liftCheck.deltaZ * 1000).toFixed(0)}mm`, 'ok');

      if (String(target.color || '') === 'blue') {
        await runVisionMoveStep(plan.transitPose, Math.max(1.45, duration * 0.70), t('msg.pickBlueTransit', { z: plan.transitPose.position.z.toFixed(3) }));
        lastPose = plan.transitPose;
      }

      const liftDuration = Math.max(1.8, duration * 0.85);
      await runIfNeeded(plan.approachPose, liftDuration, t('msg.pickRaise', { color: target.color }));

      const finalTransitDuration = Math.max(1.1, duration * 0.60);
      await runIfNeeded(plan.transitPose, finalTransitDuration, t('msg.pickTransit', { color: target.color }));

      lastVisionTarget = cloneVisionTarget(target);
      setMessage(t('msg.graspDemoDone', { mm: Math.round(plan.graspPlan.physicalGap * 1000) }));
      writeLog(t('log.graspDone', { mm: Math.round(plan.graspPlan.command * 1000) }), 'ok');
    } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionPickAbort');
     setMessage(message);
     writeLog(message, 'warn');
   } finally {
     finishVisionSequence();
   }
 }

  async function safelyPlaceVisionTarget(target, duration, reason) {
    const plan = buildVisionPickPlan(target);
    if (!plan) throw new Error(`${reason || '视觉放置'}：目标位姿无效`);
    await runVisionMoveStep(
      plan.approachPose,
      Math.max(1.1, duration * 0.65),
      `${reason || '视觉放置'}：移动到 ${target.color} 上方`
    );
    await runVisionMoveStep(
      plan.graspPose,
      Math.max(1.1, duration * 0.65),
      `${reason || '视觉放置'}：下探 ${target.color}`
    );
    await commandGripperAndWait(OPEN_GRIPPER_M, `${reason || '视觉放置'}：松开 ${target.color}`, {
      // MuJoCo's force-driven fingers need the same physical travel time when
      // opening as when closing.  The previous 4.5 s timeout expired while the
      // fingers were still moving and prevented release/retreat.
      timeoutMs: 9000,
      minWaitMs: 850,
      tolerance: 0.006,
      requireReached: true,
      requireSettled: true,
      settleMs: 320,
      afterMs: 220
    });
    releaseSimCarriedObject();
    await runVisionMoveStep(
      plan.approachPose,
      Math.max(1.5, duration * 0.8),
      `${reason || '视觉放置'}：抬高 ${target.color}`
    );
    lastVisionTarget = cloneVisionTarget(target);
  }

  async function runVisionPlaceDemo() {
    if (visionSequenceBusy) {
      queueVisionOperation({ type: 'place' });
      return;
    }
    if (!controlAllowed(true)) return;

    const simCarriedColor = window.reBotSim && typeof window.reBotSim.getCarriedObject === 'function'
      ? window.reBotSim.getCarriedObject()
      : '';
    const target = heldVisionTarget
      || (simCarriedColor && lastVisionTarget && String(lastVisionTarget.color) === String(simCarriedColor)
        ? lastVisionTarget
        : null);
    if (!target) {
      setMessage(t('msg.noCarriedObject'));
      writeLog('放置被忽略：没有已抓取的物体', 'warn');
      return;
    }

    const plan = buildVisionPickPlan(target);
    if (!plan) return;
    const duration = getPoseDuration();
    setVisionBusy(true, 'place');
    try {
      await safelyPlaceVisionTarget(target, duration, '视觉放置');
      setMessage(`视觉放置演示完成，${target.color} 物体已放下`);
      writeLog(`视觉放置演示完成，${target.color} 物体已放下`, 'ok');
    } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionPlaceAbort');
      setMessage(message);
      writeLog(message, 'warn');
    } finally {
      finishVisionSequence();
    }
  }

  async function runVisionMoveStep(pose, duration, label) {
    writePoseInputs(pose);
    client.publishTargetPose(pose);
    const result = await sendVisionMoveGoal(pose, duration, label);
    if (!movementSucceeded(result)) {
      throw new Error(`${label}失败，已停止后续动作`);
    }
    if (!(result && (result.localPlayback || result.completed))) {
      await sleep(duration * 1000 + 300);
    }
    await waitForArmMotionSettled(label, result && result.jointGoal);
    return result;
  }

  async function waitForArmMotionSettled(label, jointGoal) {
    if (TARGET_KEY !== 'simulation' || !mujocoStateIsFresh()) return;
    const timeoutMs = 2600;
    const started = performance.now();
    let previousStamp = latestJointStateAt;
    let previous = getCurrentRosPositions();
    let stableSince = 0;
    let sawFreshFeedback = false;

    while (performance.now() - started < timeoutMs) {
      await sleep(80);
      if (latestJointStateAt === previousStamp) continue;
      previousStamp = latestJointStateAt;
      sawFreshFeedback = true;
      const current = getCurrentRosPositions();
      const sampleDelta = current.reduce((largest, value, index) => (
        Math.max(largest, Math.abs(value - previous[index]))
      ), 0);
      const goalError = Array.isArray(jointGoal) && jointGoal.length === current.length
        ? current.reduce((largest, value, index) => (
          Math.max(largest, Math.abs(value - Number(jointGoal[index])))
        ), 0)
        : 0;
      const stable = sampleDelta < 0.0012 && goalError < 0.025;
      if (stable) {
        if (!stableSince) stableSince = performance.now();
        if (performance.now() - stableSince >= 260) {
          writeLog(`${label}：MuJoCo 实际反馈已到位`, 'ok');
          return;
        }
      } else {
        stableSince = 0;
      }
      previous = current;
    }

    if (sawFreshFeedback) {
      // The fake driver and MuJoCo physics loop can retain a small steady-state
      // joint error under gravity. This settling check is a sequencing delay,
      // not a motion result; the ROS action result above remains authoritative.
      // Do not abort a valid pick at the safe high waypoint.
      writeLog(`${label}：MuJoCo 反馈仍有微小跟随误差，继续下一步`, 'warn');
    }
  }

  function movementSucceeded(result) {
    if (!result) return false;
    if (result.success === false || result.accepted === false) return false;
    if (result.completed && Number(result.status) !== 4) return false;
    return true;
  }

  function buildVisionPickPlan(target) {
    const approachZ = getVisionApproachZ(target);
    const graspZ = getVisionGraspZ(target);
    const transitPose = poseFromVisionTarget(getVisionTransitZ(target), target);
    const approachPose = poseFromVisionTarget(approachZ, target);
    const verticalAlignZ = Math.min(
      approachZ,
      Math.max(graspZ + VISION_VERTICAL_ALIGN_CLEARANCE_M, VISION_MIN_VERTICAL_ALIGN_Z_M)
    );
    const pregraspZ = Math.min(
      verticalAlignZ,
      Math.max(graspZ + VISION_PREGRASP_CLEARANCE_M, graspZ)
    );
    const verticalAlignPose = poseFromVisionTarget(verticalAlignZ, target);
    const pregraspPose = poseFromVisionTarget(pregraspZ, target);
    const graspPose = poseFromVisionTarget(graspZ, target);
    const firstLiftZ = Math.min(
      approachZ,
      Math.max(graspZ + VISION_FIRST_LIFT_CLEARANCE_M, getVisionFirstLiftMinZ(target))
    );
    const firstLiftPose = poseFromVisionTarget(firstLiftZ, target);
    if (!transitPose || !approachPose || !verticalAlignPose || !pregraspPose || !graspPose || !firstLiftPose) return null;
    return {
      target,
      approachZ,
      graspZ,
      transitPose,
      approachPose,
      verticalAlignPose,
      pregraspPose,
      graspPose,
      firstLiftPose,
      graspPlan: estimateVisionGraspPlan(target)
    };
  }

  function buildVisionTransitRoute(target) {
    const route = [];
    appendVisionRoutePose(
      route,
      poseFromVisionTarget(getVisionTransitZ(target), target),
      t('msg.avoidMove', { color: target.color })
    );
    return route;
  }

  function appendVisionRoutePose(route, pose, label) {
    if (!pose) return;
    const last = route.length ? route[route.length - 1].pose : null;
    if (last && poseDistance(last, pose) < 0.025) return;
    route.push({ pose, label });
  }

  function poseDistance(left, right) {
    const values = [
      left && left.position && left.position.x,
      left && left.position && left.position.y,
      left && left.position && left.position.z,
      right && right.position && right.position.x,
      right && right.position && right.position.y,
      right && right.position && right.position.z
    ].map(Number);
    if (!values.every(Number.isFinite)) return Infinity;
    return Math.hypot(values[0] - values[3], values[1] - values[4], values[2] - values[5]);
  }

  function poseFromVisionTarget(zOverride, targetOverride) {
    const target = targetOverride || selectedVisionTarget || chooseVisionTarget();
    if (!target) {
      setMessage(t('msg.noVisionTarget'));
      return null;
    }
    const x = Number(target.x);
    const y = Number(target.y);
    const z = Number(zOverride);
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      setMessage(t('msg.visionPoseInvalid'));
      return null;
    }
    selectedVisionTarget = target;
    const graspPlan = estimateVisionGraspPlan(target);
    return {
      position: { x, y, z },
      orientation: topDownOrientationWithYaw(graspPlan.yawRad)
    };
  }

  async function sendVisionMoveGoal(pose, duration, optimisticMessage) {
    // rosapi may hide ROS 2 action services. The hardware controller always
    // exposes MoveToPose, so only use the IK/trajectory fallback in simulation.
    if (TARGET_KEY !== 'hardware' && !hasActionService(`/${NS}/move_to_pose`)) {
      return moveToPoseViaIkTrajectory(pose, duration, optimisticMessage);
    }

    try {
      setMessage(optimisticMessage);
      writeLog(optimisticMessage, 'info');
      const result = await client.moveToPose(pose, duration);
      const message = formatServiceResult(result);
      setMessage(message);
      writeLog(message, result && result.accepted === false ? 'warn' : 'ok');
      return { ...(result || {}), localPlayback: false };
    } catch (error) {
      const message = error && error.message ? error.message : t('msg.visionMoveFail');
      setStatus('error', message);
      writeLog(message, 'error');
      return { success: false, localPlayback: false };
    }
  }

  async function moveToPoseViaIkTrajectory(pose, duration, optimisticMessage) {
    setMessage(t('msg.ikSolving', { label: optimisticMessage }));
    writeLog(t('msg.ikSolving', { label: optimisticMessage }), 'info');
    const ik = await guardedCall(
      () => client.solveMoveToPoseIK(pose),
      t('msg.ikSolving', { label: optimisticMessage }),
      true,
      { keepConnectionStatus: true }
    );
    if (!ik || !Array.isArray(ik.q_solution) || !ik.q_solution.length) {
      setMessage(t('msg.ikNoSolution'));
      return { success: false, localPlayback: true };
    }
    const ikBestEffort = ik.success === false;
    if (ikBestEffort) {
      const message = ik.message || t('msg.ikApproxFallback');
      setMessage(`${message}，继续执行近似解。`);
      writeLog(`${message}，继续执行近似解。`, 'warn');
    }

    const start = getCurrentRosPositions();
    const goal = JOINT_NAMES.map((name, index) => {
      const value = Number(ik.q_solution[index]);
      return Number.isFinite(value) ? value : start[index];
    });
    const points = buildSmoothJointMovePoints(start, goal, duration);
    await sendTrajectory(points, t('msg.ikLowLevelSuffix', { label: optimisticMessage }));
    return {
      success: true,
      localPlayback: true,
      bestEffort: ikBestEffort,
      jointGoal: goal
    };
  }

  function buildSmoothJointMovePoints(start, goal, duration) {
    const seconds = clamp(Number(duration) || 2, 0.35, 30);
    const count = Math.max(10, Math.ceil(seconds * 30));
    const points = [makeTrajectoryPoint(start, 0.05)];
    for (let index = 1; index <= count; index += 1) {
      const ratio = index / count;
      const eased = ratio * ratio * (3 - 2 * ratio);
      const positions = goal.map((value, jointIndex) => {
        const from = Number(start[jointIndex]) || 0;
        return from + (value - from) * eased;
      });
      points.push(makeTrajectoryPoint(positions, Math.max(0.06, seconds * ratio)));
    }
    return points;
  }

  function writePoseInputs(pose) {
    if (els.poseX) els.poseX.value = Number(pose.position.x).toFixed(3);
    if (els.poseY) els.poseY.value = Number(pose.position.y).toFixed(3);
    if (els.poseZ) els.poseZ.value = Number(pose.position.z).toFixed(3);
  }

  function getVisionApproachZ(target) {
    const transitZ = getVisionTransitZ(target);
    const detected = target && Number.isFinite(Number(target.z)) ? Number(target.z) : transitZ;
    const requested = Number(els.visionApproachZ && els.visionApproachZ.value);
    const value = Number.isFinite(requested) ? requested : detected;
    return clamp(Math.max(value, transitZ), 0.08, 0.42);
  }

  function getVisionTransitZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const value = VISION_TRANSIT_Z_BY_COLOR_M[color];
    return Number.isFinite(value) ? value : VISION_TRANSIT_Z_M;
  }

  function getVisionGraspZ(target) {
    const safe = estimateVisionGraspZ(target);
    const requested = Number(els.visionGraspZ && els.visionGraspZ.value);
    const value = Number.isFinite(requested) ? Math.max(requested, safe) : safe;
    return clamp(value, 0.06, 0.25);
  }

  function estimateVisionGraspZ(target) {
    // The TCP is at the centre of the 79 mm-long finger pads, not at the
    // object's centre. z=0.140 keeps the pad tips just above the 0.100 m table.
    return 0.140;
  }

  function getVisionFirstLiftMinZ(target) {
    const color = target && target.color ? String(target.color) : '';
    const value = VISION_FIRST_LIFT_MIN_BY_COLOR_M[color];
    return Number.isFinite(value) ? value : VISION_FIRST_LIFT_MIN_M;
  }

  function sameVisionTarget(left, right) {
    if (!left || !right) return false;
    if (String(left.color || '') !== String(right.color || '')) return false;
    const dx = Number(left.x) - Number(right.x);
    const dy = Number(left.y) - Number(right.y);
    return Number.isFinite(dx) && Number.isFinite(dy) && Math.hypot(dx, dy) < 0.035;
  }

  function visionTargetShifted(left, right, threshold) {
    if (!left || !right) return false;
    if (String(left.color || '') !== String(right.color || '')) return false;
    const dx = Number(left.x) - Number(right.x);
    const dy = Number(left.y) - Number(right.y);
    const dz = Number(left.z || 0) - Number(right.z || 0);
    if (![dx, dy, dz].every(Number.isFinite)) return false;
    return Math.hypot(dx, dy, dz) > threshold;
  }

  function verifyVisionLift(before, after) {
    if (!before || !after || String(before.color || '') !== String(after.color || '')) {
      return { ok: false, deltaZ: 0, xyError: Infinity, message: '没有收到同一物体的最新位置' };
    }
    const x0 = Number(before.x);
    const y0 = Number(before.y);
    const z0 = Number(before.z);
    const x1 = Number(after.x);
    const y1 = Number(after.y);
    const z1 = Number(after.z);
    if (![x0, y0, z0, x1, y1, z1].every(Number.isFinite)) {
      return { ok: false, deltaZ: 0, xyError: Infinity, message: '物体坐标无效' };
    }
    const deltaZ = z1 - z0;
    const xyError = Math.hypot(x1 - x0, y1 - y0);
    // The commanded first lift is 40 mm at the TCP. A pinched object's centre
    // can lag by roughly 10 mm while the pads settle, so 20 mm is a clear
    // lift-off from the table without rejecting a valid physical grasp.
    const ok = deltaZ >= 0.020 && xyError <= 0.03;
    const message = `抬升 ${(deltaZ * 1000).toFixed(0)}mm，水平偏移 ${(xyError * 1000).toFixed(0)}mm`;
    return { ok, deltaZ, xyError, message };
  }

  function cloneVisionTarget(target) {
    if (!target || typeof target !== 'object') return null;
    return { ...target };
  }

  function attachSimCarriedObject(target) {
    const color = target && target.color ? String(target.color) : '';
    if (!color) return;
    heldVisionTarget = cloneVisionTarget(target);
    if (!window.reBotSim || typeof window.reBotSim.attachObject !== 'function') return;
    if (window.reBotSim.attachObject(color)) {
      writeLog(`网页动画：${color} 已绑定到夹爪跟随`, 'ok');
    }
  }

  function releaseSimCarriedObject() {
    heldVisionTarget = null;
    if (!window.reBotSim || typeof window.reBotSim.releaseObject !== 'function') return;
    if (window.reBotSim.releaseObject({ settleOnTable: true })) {
      writeLog('网页动画：已释放上一件跟随物体', 'info');
    }
  }

  function estimateVisionGraspPlan(target) {
    const fallbackByColor = {
      red: 0.05,
      yellow: 0.044,
      blue: 0.044
    };
    const width = Number(target && target.width_m);
    const height = Number(target && target.height_m);
    let crossSection = Number(target && target.shortest_m);
    const reportedYaw = Number(target && target.grasp_yaw_rad);
    const objectYaw = Number.isFinite(reportedYaw) ? reportedYaw : 0;
    let yawRad = objectYaw;

    const color = target && target.color ? String(target.color) : '';
    if (Number.isFinite(width) && Number.isFinite(height) && width > 0 && height > 0) {
      const candidates = [
        { crossSection: height, yawRad: objectYaw },
        { crossSection: width, yawRad: objectYaw - Math.PI / 2 }
      ].sort((left, right) => {
        const leftFits = left.crossSection <= GRIPPER_EFFECTIVE_GAP_M;
        const rightFits = right.crossSection <= GRIPPER_EFFECTIVE_GAP_M;
        if (leftFits !== rightFits) return leftFits ? -1 : 1;
        return left.crossSection - right.crossSection;
      });
      crossSection = candidates[0].crossSection;
      yawRad = candidates[0].yawRad;
    } else {
      if (!Number.isFinite(crossSection) || crossSection <= 0) {
        crossSection = fallbackByColor[color] || 0.05;
      }
      if (Number.isFinite(reportedYaw)) {
        yawRad = reportedYaw;
      }
    }

    const physicalGap = clamp(
      crossSection - GRASP_SQUEEZE_M,
      MIN_OBJECT_GRASP_M,
      GRIPPER_EFFECTIVE_GAP_M
    );
    return {
      command: physicalGapToGripperCommand(physicalGap),
      physicalGap,
      yawRad
    };
  }

  function physicalGapToGripperCommand(physicalGap) {
    const travel = Math.max(GRIPPER_PHYSICAL_TRAVEL_M, 0.001);
    return clamp(((physicalGap - GRIPPER_BASE_GAP_M) / travel) * OPEN_GRIPPER_M, CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
  }

  function topDownOrientationWithYaw(yawRad) {
    const yaw = Number.isFinite(yawRad) ? yawRad : 0;
    const sy = Math.sin(yaw / 2);
    const cy = Math.cos(yaw / 2);
    const s90 = Math.SQRT1_2;
    return {
      x: -sy * s90,
      y: cy * s90,
      z: sy * s90,
      w: cy * s90
    };
  }

  function getPoseDuration() {
    return clamp(Number(els.poseDuration && els.poseDuration.value) || 2, 0.4, 8);
  }

  function setVisionBusy(busy, operation) {
    visionSequenceBusy = busy;
    activeVisionOperation = busy ? String(operation || '') : '';
    if (els.visionPickDemo) {
      // Keep the action buttons clickable while a sequence is running. Their
      // handlers serialize the latest requested action instead of silently
      // discarding it or starting concurrent ROS service calls.
      els.visionPickDemo.disabled = false;
      els.visionPickDemo.setAttribute('aria-busy', busy && operation === 'pick' ? 'true' : 'false');
      els.visionPickDemo.textContent = busy && operation === 'pick'
        ? t('btn.pickBusy')
        : t('camera.pick');
      if (!busy) els.visionPickDemo.removeAttribute('title');
    }
    if (els.visionPlaceDemo) {
      els.visionPlaceDemo.disabled = false;
      els.visionPlaceDemo.setAttribute('aria-busy', busy && operation === 'place' ? 'true' : 'false');
      els.visionPlaceDemo.textContent = busy && operation === 'place'
        ? t('btn.placeBusy')
        : t('camera.place');
      if (!busy) els.visionPlaceDemo.removeAttribute('title');
    }
    if (els.visionMoveAbove) els.visionMoveAbove.disabled = busy;
    if (els.visionFillPose) els.visionFillPose.disabled = busy;
  }

  function finishVisionSequence() {
    setVisionBusy(false);
    // Start the queued action synchronously up to its first await so there is
    // no idle frame in which another click can strand an older queued action.
    runQueuedVisionOperation();
  }

  function updateFeedbackError(feedback) {
    if (!els.feedbackError || !window.reBotSim || !feedback || !Object.keys(feedback).length) return;
    const simAngles = typeof window.reBotSim.getAngles === 'function' ? window.reBotSim.getAngles() : {};
    let maxError = 0;
    let sumSq = 0;
    let count = 0;
    let worstJoint = '';

    Object.entries(feedback).forEach(([name, value]) => {
      const target = simTargetAngles.has(name) ? simTargetAngles.get(name) : simAngles[name];
      if (typeof target !== 'number') return;
      const error = Math.abs(target - value);
      if (error > maxError) {
        maxError = error;
        worstJoint = name;
      }
      sumSq += error * error;
      count += 1;
    });

    if (!count) return;
    const rms = Math.sqrt(sumSq / count);
    els.feedbackError.textContent = t('fb.errorMax', { max: (maxError * 180 / Math.PI).toFixed(2), joint: worstJoint || '', rms: (rms * 180 / Math.PI).toFixed(2) });
    els.feedbackError.style.color = maxError < 0.035 ? '#d7fff4' : (maxError < 0.12 ? '#ffe0b0' : '#ffd1c9');
  }

  function updateGravityStatus(active, detail, source) {
    const nextActive = Boolean(active);
    const nextSource = source || 'system';
    // Keep the detailed status-service text visible instead of allowing the
    // faster ArmStatus stream to replace it with only "GRAVITY_COMP".
    if (
      nextSource === 'arm' &&
      gravityStatusSource === 'service' &&
      nextActive === gravityCompensationActive
    ) return;

    gravityCompensationActive = nextActive;
    gravityStatusSource = nextSource;
    if (!els.gravityStatus) return;
    els.gravityStatus.textContent = nextActive ? t('st.running') : t('st.notRunning');
    if (detail && detail !== 'GRAVITY_COMP') {
      els.gravityStatus.textContent += ` / ${detail}`;
    }
    els.gravityStatus.style.color = nextActive ? '#d7fff4' : '#ffe0b0';
  }

  function maybeSendGripper(position) {
    syncSimGripper(position);
    if (!client.connected) {
      setMessage(t('msg.gripperSimOnly'));
      return;
    }
    if (!controlAllowed(false)) {
      setMessage(t('msg.controlLockClosed'));
      return;
    }
    publishGripper(position);
  }

  function sendGripper(position, options) {
    if (options && options.requireControl && !controlAllowed(true)) return;
    if (!client.connected) {
      setStatus('closed', t('msg.rosNotConnected'));
      return;
    }
    publishGripper(position);
  }

  async function commandGripperAndWait(position, label, options) {
    const settings = {
      timeoutMs: 1800,
      minWaitMs: 500,
      tolerance: 0.006,
      settleMs: 260,
      afterMs: 0,
      allowContactStop: false,
      requireContactStop: false,
      contactTolerance: 0.0025,
      requireReached: false,
      requireSettled: false,
      positionStableTolerance: 0.0008,
      ...(options || {})
    };
    publishGripper(position);
    setMessage(label);

    const start = performance.now();
    const initialFeedbackAt = latestGripperAt;
    const initialMujocoFeedbackAt = latestMujocoGripperAt;
    const initialJointFeedback = gripperJointFeedback();
    let lastPosition = readGripperFeedbackPosition(position);
    const initialPosition = lastPosition;
    let lastRepublishAt = start;
    let stableSince = start;
    let sawFreshFeedback = false;
    let reached = false;
    let settled = false;
    let contactStopped = false;
    let current = lastPosition;
    let source = latestGripperAt > 0 ? 'gripper/state' : '';

    while (performance.now() - start < settings.timeoutMs) {
      await sleep(80);
      const now = performance.now();
      if (now - lastRepublishAt > 520) {
        publishGripperWidthCommand(position);
        lastRepublishAt = now;
      }

      const jointFeedback = gripperJointFeedback();
      const hasFreshMujocoState = TARGET_KEY === 'simulation' &&
        latestMujocoGripperAt > initialMujocoFeedbackAt &&
        now - latestMujocoGripperAt < 700;
      const hasFreshGripperState = latestGripperAt > initialFeedbackAt && now - latestGripperAt < 700;
      const hasFreshJointState = jointFeedback.fresh && jointFeedback.stamp !== initialJointFeedback.stamp;
      const hasFreshFeedback = hasFreshMujocoState || hasFreshJointState || hasFreshGripperState;
      if (!hasFreshFeedback) {
        if (!settings.requireReached && now - start > Math.max(settings.minWaitMs, 900)) break;
        continue;
      }

      sawFreshFeedback = true;
      current = hasFreshMujocoState
        ? Number(latestMujocoGripperPosition)
        : (hasFreshJointState ? jointFeedback.widthCommand : Number(latestGripperPosition));
      source = hasFreshMujocoState
        ? 'mujoco/joint_states/gripper_joint1'
        : (hasFreshJointState ? 'joint_states/gripper_joint1' : 'gripper/state');
      // Number(null) is zero, which previously made simulation feedback look
      // stationary before a velocity sample had ever arrived.
      const velocitySource = hasFreshMujocoState
        ? latestMujocoGripperVelocity
        : latestGripperVelocity;
      const velocity = typeof velocitySource === 'number'
        ? velocitySource
        : NaN;
      const closeEnough = Number.isFinite(current) && Math.abs(current - position) <= settings.tolerance;
      const closingProgress = Number.isFinite(initialPosition) && Number.isFinite(current)
        ? initialPosition - current
        : 0;
      // During a closing command, a measured opening that remains above the
      // requested opening proves that the fingers were blocked by an object.
      // Reaching the command exactly is an empty grasp and must not be lifted.
      const contactBlocked = settings.allowContactStop &&
        Number.isFinite(current) &&
        closingProgress >= Math.max(settings.contactTolerance, 0.003) &&
        current - position >= settings.contactTolerance;
      // MuJoCo kinematic mode intentionally publishes qvel=0 while qpos is
      // being smoothed, so velocity alone cannot prove that the fingers have
      // stopped. Require consecutive position samples to be stable as well.
      const positionStable = Number.isFinite(current) &&
        Number.isFinite(lastPosition) &&
        Math.abs(current - lastPosition) < settings.positionStableTolerance;
      const velocityStable = !Number.isFinite(velocity) || Math.abs(velocity) < 0.0025;
      // Physics mode publishes the real MuJoCo qvel, so it must participate in
      // the stop decision.  Ignoring it made a slowly closing gripper look
      // stationary after several small position increments and allowed the arm
      // to lift before the fingers had finished squeezing.  Kinematic mode is
      // the only simulation mode whose qvel is intentionally always zero.
      const hasPhysicsVelocity = TARGET_KEY === 'simulation' &&
        latestMujocoSimulationMode === 'physics' &&
        hasFreshMujocoState &&
        Number.isFinite(velocity);
      const barelyMoving = positionStable && (
        hasPhysicsVelocity ? (velocityStable || (contactBlocked && Math.abs(velocity) < 0.004)) : (
          TARGET_KEY === 'simulation' || velocityStable
        )
      );
      reached = reached || closeEnough;

      if (barelyMoving) {
        if (now - stableSince >= settings.settleMs && now - start >= settings.minWaitMs) {
          if (closeEnough || (
            !settings.requireReached && settings.allowContactStop && contactBlocked
          )) {
            settled = true;
            contactStopped = contactStopped || contactBlocked;
            break;
          }
        }
      } else {
        stableSince = now;
      }

      if (
        closeEnough &&
        now - start >= settings.minWaitMs &&
        (!settings.requireSettled || (
          barelyMoving && now - stableSince >= settings.settleMs
        ))
      ) break;
      lastPosition = current;
    }

    if (settings.afterMs > 0) await sleep(settings.afterMs);
    if (settings.requireReached && !reached) {
      const message = `${label}未确认到位，已停止本轮动作`;
      setMessage(message);
      writeLog(message, 'warn');
      throw new Error(message);
    }
    if (settings.requireSettled && !settled) {
      const message = `${label}未确认稳定停止，已停止本轮动作`;
      setMessage(message);
      writeLog(message, 'warn');
      throw new Error(message);
    }
    if (settings.requireContactStop && !contactStopped) {
      const message = `${label}未检测到物体接触，夹爪为空，已停止抬升`;
      setMessage(message);
      writeLog(message, 'warn');
      throw new Error(message);
    }
    const feedback = sawFreshFeedback && Number.isFinite(Number(current))
      ? t('fb.gripperSrcFb', { src: source, mm: Math.round(current * 1000) })
      : '';
    writeLog(`${label}完成${feedback}`, 'ok');
  }

  function readGripperFeedbackPosition(commandPosition) {
    if (
      TARGET_KEY === 'simulation' &&
      latestMujocoGripperAt > 0 &&
      Number.isFinite(Number(latestMujocoGripperPosition))
    ) {
      return Number(latestMujocoGripperPosition);
    }
    const jointFeedback = gripperJointFeedback();
    if (jointFeedback.fresh && Number.isFinite(jointFeedback.widthCommand)) {
      return jointFeedback.widthCommand;
    }
    if (Number.isFinite(Number(latestGripperPosition))) {
      return Number(latestGripperPosition);
    }
    return Number(commandPosition);
  }

  function gripperJointFeedback() {
    const source = latestJointPositions || {};
    const hasRsJoint = Number.isFinite(Number(source.gripper_joint1));
    const left = hasRsJoint ? Number(source.gripper_joint1) : Number(source.finger_left);
    if (!Number.isFinite(left)) {
      return { fresh: false, widthCommand: NaN, stamp: 0 };
    }
    // /mujoco/joint_states reports the actual MuJoCo prismatic joint, whose
    // per-finger travel is 50 mm.  The fake/real RS driver reports the 45 mm
    // URDF visual joint.  Treating both as 45 mm inflated MuJoCo feedback by
    // 11%, producing a false contact-stop before physical closure completed.
    const visualOpen = hasRsJoint
      ? (TARGET_KEY === 'simulation' && mujocoStateIsFresh()
        ? GRIPPER_FINGER_TRAVEL_M
        : 0.045)
      : 0.0285;
    return {
      fresh: true,
      widthCommand: fingerOpeningToGripperCommand(left, visualOpen),
      stamp: latestJointStateAt || 0
    };
  }

  function fingerOpeningToGripperCommand(opening, visualOpen) {
    return clamp((Number(opening) / visualOpen) * OPEN_GRIPPER_M, CLOSE_GRIPPER_M, OPEN_GRIPPER_M);
  }

  function gripperWidthToMotor(width) {
    const ratio = clamp(
      (Number(width) - CLOSE_GRIPPER_M) / (OPEN_GRIPPER_M - CLOSE_GRIPPER_M),
      0,
      1
    );
    return CLOSE_GRIPPER_MOTOR_RAD + ratio * (OPEN_GRIPPER_MOTOR_RAD - CLOSE_GRIPPER_MOTOR_RAD);
  }

  function gripperMotorToWidth(position) {
    const ratio = clamp(
      (Number(position) - CLOSE_GRIPPER_MOTOR_RAD) /
        (OPEN_GRIPPER_MOTOR_RAD - CLOSE_GRIPPER_MOTOR_RAD),
      0,
      1
    );
    return CLOSE_GRIPPER_M + ratio * (OPEN_GRIPPER_M - CLOSE_GRIPPER_M);
  }

  function publishGripperWidthCommand(width) {
    client.publishGripperCommand(gripperWidthToMotor(width), GRIPPER_VLIM_RAD_S);
  }

  function publishGripper(position) {
    dampedSliderCommands.delete('gripper');
    if (gripperRepublishTimer) window.clearTimeout(gripperRepublishTimer);
    syncSimGripper(position);
    publishGripperWidthCommand(position);
    simTargetAngles.set('gripper', position);
    mirrorHoldUntil.set('gripper', performance.now() + 1200);
    const feedback = typeof latestGripperPosition === 'number' ? t('fb.gripperFb', { mm: Math.round(latestGripperPosition * 1000) }) : '';
    setMessage(t('msg.gripperCmdPublished', { mm: Math.round(position * 1000), fb: feedback }));
    writeLog(`夹爪指令 ${Math.round(position * 1000)} 毫米 -> /${NS}/gripper/cmd/mit`, 'ok');
    gripperRepublishTimer = window.setTimeout(() => {
      gripperRepublishTimer = 0;
      if (client.connected) publishGripperWidthCommand(position);
    }, 120);
  }

  function syncSimGripper(position) {
    if (!window.reBotSim) return;
    if (TARGET_KEY === 'hardware') {
      // In hardware mode the gripper has no target ghost: the solid fingers
      // move only with measured feedback from /gripper/state.
      return;
    }
    if (typeof window.reBotSim.setGripperWidth !== 'function') return;
    window.reBotSim.setGripperWidth(position, { source: 'ui', animate: true });
  }

  function getVlim() {
    return clamp(
      Number(els.vlim.value) || DEFAULT_JOINT_VLIM_RAD_S,
      0.05,
      MAX_JOINT_VLIM_RAD_S
    );
  }

  function getSliderDampingMs() {
    return clamp(Number(els.sliderDamping && els.sliderDamping.value) || 0, 0, 300);
  }

  function getTrajectoryDuration() {
    return clamp(Number(els.trajectoryDuration.value) || 2, 1, 30);
  }

  function secondsToRosTime(seconds) {
    return nsToRosStamp(secondsToNs(seconds));
  }

  function secondsToNs(seconds) {
    const value = Math.max(0, Number(seconds) || 0);
    return BigInt(Math.round(value * 1e9));
  }

  function rosTimeToSeconds(time) {
    return Number(time && time.sec ? time.sec : 0) + Number(time && time.nanosec ? time.nanosec : 0) * 1e-9;
  }

  function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function sleep(ms) {
    return new Promise((resolve) => window.setTimeout(resolve, ms));
  }

 function waitForSimApi(callback) {
   if (window.reBotSim && typeof window.reBotSim.onCommand === 'function') {
     callback(window.reBotSim);
     return;
   }
   window.setTimeout(() => waitForSimApi(callback), 50);
 }

  function setStatus(state, message) {
    els.status.className = 'mini-pill';
    if (state === 'open') {
      els.status.classList.add('online');
      els.status.textContent = t('st.online');
    } else if (state === 'connecting') {
      els.status.classList.add('warn');
      els.status.textContent = t('st.connecting');
    } else if (state === 'error') {
      els.status.classList.add('error');
      els.status.textContent = t('st.error');
    } else {
      els.status.textContent = t('st.offline');
    }
    setMessage(message);
  }

  function setMessage(message) {
    if (els.message) els.message.textContent = message || '';
  }

  function writeLog(message, level) {
    if (!els.log || !message) return;
    const line = document.createElement('div');
    line.className = `ros-log-line ${level || 'info'}`;
    const now = new Date();
    line.innerHTML = `<time>${now.toLocaleTimeString()}</time><span></span>`;
    line.querySelector('span').textContent = String(message);
    els.log.prepend(line);
    while (els.log.children.length > 80) els.log.lastElementChild.remove();
  }

  if (window.rebotI18n) {
    window.rebotI18n.onLangChange(() => {
      if (els.visionPickDemo && !visionSequenceBusy) {
        els.visionPickDemo.textContent = t('camera.pick');
      }
      if (els.visionPlaceDemo && !visionSequenceBusy) {
        els.visionPlaceDemo.textContent = t('camera.place');
      }
      if (els.status) {
        if (client.connected) {
          els.status.textContent = t('st.online');
        } else {
          els.status.textContent = t('st.offline');
        }
      }
    });
  }

})();
