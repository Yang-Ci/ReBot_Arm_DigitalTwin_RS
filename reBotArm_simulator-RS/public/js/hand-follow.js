(function () {
  const scriptUrl = document.currentScript.src;
  const asset = (relative) => new URL(relative, scriptUrl).href;
  const $ = (id) => document.getElementById(id);
  const video = $('hand-video');
  const canvas = $('hand-overlay');
  const context = canvas.getContext('2d');
  const preview = $('hand-preview');
  const depthPreview = $('hand-depth-preview');
  const depthContext = depthPreview.getContext('2d');
  const depthController = new window.RebotHandDepth.Controller();
  const { MIN_DISTANCE_M, NEAR_RELEASE_M } = window.RebotHandDepth;
  const depthMotion = new window.RebotHandDepth.Motion();
  let depthPacket = null, depthSequence = 0, depthOwned = false;
  let depthStop = Promise.resolve(), depthFeedback = null, lastDepth = null;
  let lastDepthQuality = null, nearDistanceBlocked = false;
  let lastDepthTransition = null;
  const depthHistory = [];
  let lastDepthFrameAt = 0;
  const { Session, PinchGripper, measureHand, visiblePoint, SAMPLE_MAX_AGE_MS } = window.RebotHandPolicy;
  const pinchGripper = new PinchGripper();
  const RESULT_MAX_GAP_MS = 400;
  const safety = new Session();
  const connections = [[0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[5,9],[9,10],[10,11],[11,12],[9,13],[13,14],[14,15],[15,16],[13,17],[0,17],[17,18],[18,19],[19,20]];
  let mode = 'off';
  let busy = false;
  let session = 0;
  let worker = null;
  let stream = null;
  let frame = 0;
  let pendingFrame = false;
  let target = null;
  let filtered = null;
  let lastValidResultAt = 0;
  let lastTick = 0;
  let lastCapture = 0;
  let lastVideoTime = -1;
  let filteredGrip = null;
  let lastUiUpdate = 0;
  let readyReported = false;
  let lastObservation = null;
  let lastFollowResult = null;
  let standbyReason = '机械臂保持零点，张开手掌保持 1 秒唤醒。';
  let workerInitCancel = null;
  let lastRecognition = null;
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  const status = (text, active = false) => {
    $('hand-follow-status').textContent = text;
    $('hand-follow-status').dataset.active = String(active);
  };
  const message = (text) => { $('hand-follow-message').textContent = text; };
  const sim = () => window.reBotSim;

  const simpleView = new URLSearchParams(location.search).get('view') === 'hand-follow';
  if (simpleView && new URLSearchParams(location.search).get('embed') === 'mujoco') document.body.classList.add('hand-follow-embedded');
  if (simpleView) {
    document.body.classList.add('hand-follow-page');
    document.title = 'ReBot Arm · 手势跟随';
    // Keep the existing model and DOM, but show only the controls for this demo.
    ['.top-hud h1', '.panel-title h2'].forEach((selector, index) => {
      const el = document.querySelector(selector);
      el.removeAttribute('data-i18n');
      el.textContent = index ? '手势跟随' : '机械臂手势跟随';
    });
    const eyebrow = document.querySelector('.top-hud .eyebrow');
    eyebrow.removeAttribute('data-i18n');
    eyebrow.textContent = 'REBOT ARM / CAMERA INTERACTION';
    ['toggle-envelope', 'toggle-mujoco-scene'].forEach((id) => {
      $(id).checked = false;
      $(id).dispatchEvent(new Event('change'));
    });
  }

  function updateButtons() {
    const ready = Boolean(sim()?.isReady());
    $('hand-start').disabled = busy || !ready;
    $('hand-mouse').disabled = busy || !ready;
    $('hand-stop').disabled = !busy && mode === 'off';
    $('hand-recenter').disabled = busy || safety.state !== 'active';
    $('hand-wake').hidden = mode !== 'mouse';
    $('hand-wake').disabled = busy || mode !== 'mouse' || safety.state !== 'standby';
    $('hand-sleep').disabled = busy || !['active', 'waking'].includes(safety.state);
    $('hand-camera').disabled = busy;
    $('hand-start').textContent = busy ? '正在开启…' : ['camera', 'depth'].includes(mode) ? '重启相机跟随' : '开启相机跟随';
    $('hand-mouse').classList.toggle('active', mode === 'mouse');
  }

  function pauseTracking(resetDepth = true) {
    target = filtered = filteredGrip = null;
    depthMotion.reset();
    if (resetDepth) { depthController.reset(); depthFeedback = null; }
    lastFollowResult = null;
    sim()?.recenterWebFollow();
  }

  function holdDepthTracking(feedback) {
    target = null; lastFollowResult = null;
    depthFeedback = feedback;
    depthMotion.suspend();
    sim()?.holdWebFollow();
  }

  function reportDepthTransition() {
    const key = `${depthController.active}:${depthController.paused}:${depthController.baseline}:${depthFeedback?.status || depthFeedback?.mode || 'idle'}:${lastDepthQuality?.status || 'no-hand'}`;
    if (key === lastDepthTransition) return;
    lastDepthTransition = key;
    const entry = { at: Math.round(performance.now()), handVisible: Boolean(lastRecognition?.handVisible),
      active: depthController.active, paused: depthController.paused, quality: lastDepthQuality?.status || 'no-hand',
      distanceMeters: lastDepth, baselineMeters: depthController.baseline,
      robotReferenceX: sim()?.getWebFollowState().origin?.x ?? null, rebased: Boolean(depthFeedback?.rebased),
      status: depthFeedback?.status || depthFeedback?.mode || 'idle' };
    depthHistory.push(entry);
    if (depthHistory.length > 16) depthHistory.shift();
    console.debug('[hand-follow:depth]', entry);
  }

  function returnHome(reason) {
    standbyReason = reason;
    pinchGripper.reset();
    pauseTracking();
    safety.clearWake();
    safety.requestHome();
    sim()?.endWebFollow();
    renderSafety(performance.now());
  }

  function handleSafetyEvent(event) {
    if (event) console.debug('[hand-follow:safety]', { event, state: safety.state, recognition: lastRecognition });
    if (event === 'home') returnHome('连续 15 秒没有有效手势，已归零。张开手掌保持 1 秒重新唤醒。');
    if (event === 'wake') {
      pauseTracking();
      if (!sim()?.beginWebFollow()) returnHome('正在准备零点，请稍后重新唤醒。');
    }
  }

  function renderSafety(now) {
    const labels = { standby: '零点待机', waking: '正在进入中位', active: '交互中', returning: '平滑归零中' };
    if (!simpleView && mode === 'off') labels.standby = '手势未开启';
    $('hand-interaction-state').textContent = labels[safety.state];
    $('hand-wake-progress').value = safety.wakeProgress(now);
    $('hand-idle-countdown').textContent = safety.state === 'active'
      ? `${(safety.remainingMs(now) / 1000).toFixed(1)} 秒后归零` : safety.state === 'standby' ? '待唤醒' : '请稍候';
    if (!busy) {
      status(safety.state === 'active' ? (target ? (target.partial ? '部分遮挡 · 继续跟随' : '正在跟随')
        : lastRecognition?.handVisible ? '跟随暂停' : '丢手暂停') : labels[safety.state], safety.state === 'active');
      if (safety.state === 'waking') message('已唤醒，正在平滑进入中位；到位后开始跟随。');
      else if (safety.state === 'returning') message('正在平滑归零，期间不接收跟随输入。');
      else if (safety.state === 'standby') {
        if (nearDistanceBlocked && mode === 'depth') message('距离过近，请退到 30 cm 以上，再张开手掌唤醒。');
        else if (safety.wakeProgress() > 0) message(safety.wakePaused
          ? '识别暂时中断，保持进度已暂停。请继续张开手掌。'
          : `保持张开手掌… ${(safety.wakeProgress() * 100).toFixed(0)}%`);
        else if ((mode === 'camera' || mode === 'depth') && safety.wakeArmed && lastRecognition?.handVisible) {
          message(lastRecognition.palmOpen ? '已识别到张开的手掌，请保持 1 秒。'
            : '已识别到手部，请自然摊开四指，手掌正对相机。');
        } else message(standbyReason);
      }
      else if (!target) message(depthFeedback?.reason || '手部交互已暂停；15 秒内恢复有效交互可继续，否则自动归零。');
      else if (lastFollowResult) {
        const { x, y, z } = lastFollowResult.tcp;
        message(`${mode === 'mouse' ? '鼠标' : '手掌'}跟随 · X ${(x * 1000).toFixed(0)} / Y ${(y * 1000).toFixed(0)} / Z ${(z * 1000).toFixed(0)} mm${lastFollowResult.clamped ? ' · 已到安全边界' : ''}`);
      }
    }
    if (mode === 'depth') {
      const reading = Number.isFinite(lastDepth) ? `手部距离 ${(lastDepth * 1000).toFixed(0)} mm`
        : lastRecognition?.handVisible ? '已识别手部，距离暂不可用' : '等待手部进入画面';
      const reference = depthController.active ? ` · 起点 ${(depthController.baseline * 1000).toFixed(0)} mm` : '';
      const hint = depthFeedback?.reason || (depthController.active ? '握拳前后跟随 · 张开退出'
        : lastDepthQuality?.reason || '握拳进入前后跟随');
      $('hand-depth-status').textContent = `${reading}${reference} · ${hint}`;
    } else $('hand-depth-status').textContent = '336 深度模式支持握拳前后跟随；普通 RGB 模式只跟随平面位置。';
    updateButtons();
  }

  async function listCameras() {
    if (!navigator.mediaDevices?.enumerateDevices) return;
    try {
      const previous = $('hand-camera').value;
      const devices = (await navigator.mediaDevices.enumerateDevices()).filter((device) => device.kind === 'videoinput');
      const options = [new Option('336 RGB-D（握拳前后跟随）', 'depth-336'), new Option('默认 RGB 相机', '')];
      devices.forEach((device, index) => options.push(new Option(device.label || `相机 ${index + 1}`, device.deviceId)));
      $('hand-camera').replaceChildren(...options);
      if (previous === 'depth-336' || previous === '' || devices.some((device) => device.deviceId === previous)) $('hand-camera').value = previous;
    } catch (_) {
      // Device labels become available after the user grants camera access.
    }
  }

  function stop(reason = '相机已关闭，机械臂归零待机。') {
    session += 1;
    busy = false;
    mode = 'off';
    if (depthOwned) {
      depthOwned = false;
      depthStop = fetch('/api/hand-depth/stop', { method: 'POST', keepalive: true }).catch(() => {});
    }
    depthPacket = null; depthSequence = 0; lastDepth = lastDepthQuality = null; nearDistanceBlocked = false;
    lastDepthTransition = null; depthHistory.length = 0; lastRecognition = null;
    depthPreview.hidden = true;
    workerInitCancel?.();
    workerInitCancel = null;
    worker?.terminate();
    worker = null;
    stream?.getTracks().forEach((track) => track.stop());
    stream = null;
    video.pause();
    video.srcObject = null;
    video.hidden = canvas.hidden = true;
    $('hand-preview-empty').hidden = false;
    $('hand-preview-empty').lastElementChild.textContent = '开启相机，把手放进画面';
    $('hand-pointer').hidden = $('hand-preview-mode').hidden = true;
    context.clearRect(0, 0, canvas.width, canvas.height);
    target = filtered = filteredGrip = null;
    lastObservation = lastFollowResult = null;
    lastValidResultAt = lastTick = lastCapture = 0;
    lastVideoTime = -1;
    pendingFrame = false;
    returnHome(reason);
  }

  function initializeWorker(token) {
    return new Promise((resolve, reject) => {
      const detectorWorker = new Worker(asset('./hand-follow-worker.js?v=20261005-hand10'), { type: 'module' });
      worker = detectorWorker;
      const timeout = setTimeout(() => reject(new Error('手部模型加载超时，请重试。')), 30000);
      workerInitCancel = () => {
        clearTimeout(timeout);
        reject(new DOMException('Camera initialization cancelled', 'AbortError'));
      };
      detectorWorker.onerror = () => {
        clearTimeout(timeout);
        const error = new Error('手部识别模块加载失败，请检查本地模型文件。');
        if (token === session && !busy) stop(error.message);
        reject(error);
      };
      detectorWorker.onmessage = ({ data }) => {
        if (token !== session) return;
        if (data.type === 'ready') {
          clearTimeout(timeout);
          workerInitCancel = null;
          resolve();
        } else if (data.type === 'error') {
          clearTimeout(timeout);
          const error = new Error(`手部识别失败：${data.message}`);
          if (!busy) stop(error.message);
          reject(error);
        } else if (data.type === 'result') {
          pendingFrame = false;
          const paired = depthPacket?.timestamp === data.timestamp ? depthPacket : null;
          depthPacket = null;
          acceptLandmarks(data.landmarks, data.timestamp, paired);
        }
      };
      detectorWorker.postMessage({
        type: 'init',
        wasmUrl: asset('../lib/mediapipe/wasm'),
        modelUrl: asset('../models/hand_landmarker.task')
      });
    });
  }

  function cameraError(error) {
    if (error.name === 'NotAllowedError') return '相机权限未开启，请在浏览器地址栏允许相机后重试。';
    if (error.name === 'NotFoundError') return '没有找到相机。连接 336 后选择 RGB 相机，也可以先用鼠标试跟随。';
    if (error.name === 'NotReadableError') return '相机被其他程序占用，请关闭占用程序后重试。';
    if (error.name === 'OverconstrainedError') return '所选相机已断开，请重新选择相机。';
    return error.message || '相机开启失败，请重试。';
  }

  async function startCamera() {
    if ($('hand-camera').value === 'depth-336') return startDepthCamera();
    stop('正在开启相机…');
    const token = session;
    busy = true;
    status('正在开启');
    updateButtons();
    await depthStop;
    if (token !== session) return;
    if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) {
      stop('请使用 localhost 或 HTTPS 打开网页，浏览器才能访问相机。');
      return;
    }
    try {
      const deviceId = $('hand-camera').value;
      const nextStream = await navigator.mediaDevices.getUserMedia({
        audio: false,
        video: { width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30 }, ...(deviceId ? { deviceId: { exact: deviceId } } : {}) }
      });
      if (token !== session) {
        nextStream.getTracks().forEach((track) => track.stop());
        return;
      }
      stream = nextStream;
      if (!deviceId) {
        // The 336 exposes separate depth and RGB UVC devices. After permission
        // reveals device labels, prefer its RGB stream over the depth device.
        const devices = await navigator.mediaDevices.enumerateDevices();
        if (token !== session) return;
        const rgb = devices.find((device) => device.kind === 'videoinput' && /336.*rgb|rgb.*336/i.test(device.label));
        if (rgb && stream.getVideoTracks()[0].getSettings().deviceId !== rgb.deviceId) {
          stream.getTracks().forEach((track) => track.stop());
          const rgbStream = await navigator.mediaDevices.getUserMedia({
            audio: false,
            video: { deviceId: { exact: rgb.deviceId }, width: { ideal: 640 }, height: { ideal: 480 }, frameRate: { ideal: 30 } }
          });
          if (token !== session) {
            rgbStream.getTracks().forEach((track) => track.stop());
            return;
          }
          stream = rgbStream;
        }
      }
      video.srcObject = stream;
      await video.play();
      if (token !== session) return;
      canvas.width = video.videoWidth || 640;
      canvas.height = video.videoHeight || 480;
      preview.style.aspectRatio = `${canvas.width} / ${canvas.height}`;
      video.hidden = canvas.hidden = false;
      $('hand-preview-empty').hidden = true;
      message('正在加载手部识别模型…');
      await listCameras();
      if (token !== session) return;
      $('hand-camera').value = stream.getVideoTracks()[0].getSettings().deviceId;
      await initializeWorker(token);
      if (token !== session) return;
      if (!sim().prepareWebFollow()) throw new Error('机械臂模型尚未加载完成。');
      busy = false;
      mode = 'camera';
      $('hand-preview-mode').hidden = false;
      $('hand-preview-mode').textContent = '相机跟随';
      standbyReason = '机械臂保持零点。正对相机张开手掌保持 1 秒，唤醒后进入中位交互。';
      stream.getVideoTracks()[0].addEventListener('ended', () => {
        if (token === session) stop('相机已断开，请重新连接后开启跟随。');
      });
      renderSafety(performance.now());
    } catch (error) {
      if (token === session) stop(cameraError(error));
    }
  }

  async function depthRequest(path, options = {}) {
    const response = await fetch(`/api/hand-depth/${path}`, { ...options, signal: AbortSignal.timeout(18000) });
    if (!response.ok) {
      const error = await response.json().catch(() => ({}));
      throw new Error(error.error || '336 深度服务连接失败。');
    }
    return response;
  }

  async function startDepthCamera() {
    stop('正在开启 336 RGB-D…');
    const token = session;
    busy = true; updateButtons(); status('正在开启深度');
    try {
      await depthStop;
      if (token !== session) return;
      message('正在加载手部识别模型…');
      await initializeWorker(token);
      if (token !== session) return;
      depthOwned = true;
      await depthRequest('start', { method: 'POST' });
      if (token !== session) {
        if (!depthOwned) await fetch('/api/hand-depth/stop', { method: 'POST' });
        return;
      }
      const deadline = performance.now() + 12000;
      let state;
      do {
        state = await (await depthRequest('status')).json();
        if (token !== session) return;
        if (state.state === 'error') throw new Error(state.error);
        if (state.state === 'running') break;
        if (performance.now() > deadline) throw new Error('336 开启超时，请检查 USB 连接和相机占用。');
        await new Promise(resolve => setTimeout(resolve, 150));
      } while (true);
      if (!sim().prepareWebFollow()) throw new Error('机械臂模型尚未加载完成。');
      mode = 'depth'; busy = false; lastDepthFrameAt = performance.now();
      depthPreview.hidden = canvas.hidden = false;
      $('hand-preview-empty').hidden = true;
      $('hand-preview-mode').hidden = false;
      $('hand-preview-mode').textContent = '336 RGB-D 深度跟随';
      standbyReason = '张开手掌保持 1 秒唤醒。进入中位后，握拳控制前后距离，张开手掌退出。';
      renderSafety(performance.now());
    } catch (error) { if (token === session) stop(error.message); }
  }

  async function captureDepthFrame(now) {
    if (pendingFrame || !worker || now - lastCapture < 45) return;
    const token = session, detectorWorker = worker;
    pendingFrame = true; lastCapture = now;
    try {
      const response = await depthRequest('frame');
      if (token !== session) return;
      if (response.status === 204) {
        pendingFrame = false;
        if (now - lastDepthFrameAt > 2000) throw new Error('336 图像中断，已停止跟随。');
        return;
      }
      const packet = window.RebotHandDepth.decodePacket(await response.arrayBuffer());
      if (token !== session) return;
      if (packet.seq <= depthSequence) {
        pendingFrame = false;
        if (now - lastDepthFrameAt > 2000) throw new Error('336 图像停滞，已停止跟随。');
        return;
      }
      depthSequence = packet.seq;
      const age = Date.now() - packet.capturedUnixMs;
      if (age < 0 || age > SAMPLE_MAX_AGE_MS) { pendingFrame = false; return; }
      const timestamp = performance.now() - age;
      const bitmap = await createImageBitmap(packet.jpeg);
      if (token !== session) { bitmap.close(); return; }
      lastDepthFrameAt = performance.now();
      if (canvas.width !== packet.width || canvas.height !== packet.height) {
        canvas.width = depthPreview.width = packet.width;
        canvas.height = depthPreview.height = packet.height;
        preview.style.aspectRatio = `${packet.width} / ${packet.height}`;
      }
      depthContext.drawImage(bitmap, 0, 0, depthPreview.width, depthPreview.height);
      depthPacket = { ...packet, timestamp };
      detectorWorker.postMessage({ type: 'frame', bitmap, timestamp }, [bitmap]);
    } catch (error) { if (token === session) stop(error.message); }
  }

  function startMouse() {
    stop();
    if (!sim()?.prepareWebFollow()) return;
    mode = 'mouse';
    preview.style.aspectRatio = '4 / 3';
    $('hand-preview-empty').lastElementChild.textContent = '在这里移动鼠标或手指';
    $('hand-preview-mode').hidden = false;
    $('hand-preview-mode').textContent = '鼠标试跟随';
    standbyReason = '点击“鼠标唤醒”进入中位，再在预览区域移动鼠标；按住鼠标可闭合夹爪。';
    renderSafety(performance.now());
  }

  function resetAnchor() {
    pauseTracking();
    message('已重新定位，下一次检测到的手掌位置作为起点。');
  }

  function acceptLandmarks(landmarks, capturedAt, pairedDepth) {
    if (mode !== 'camera' && mode !== 'depth') return;
    const now = performance.now();
    context.clearRect(0, 0, canvas.width, canvas.height);
    let sample = measureHand(landmarks, canvas.width / canvas.height);
    if (!Number.isFinite(capturedAt) || capturedAt > now || capturedAt <= safety.lastSampleAt
      || now - capturedAt > SAMPLE_MAX_AGE_MS || document.hidden) sample = null;
    if (sample && lastObservation && sample.anchor === lastObservation.anchor && capturedAt - lastObservation.stamp < 150
      && distance(sample, lastObservation) > 0.20) {
      lastObservation = { ...sample, stamp: capturedAt };
      sample = null;
    } else if (sample) lastObservation = { ...sample, stamp: capturedAt };
    lastRecognition = { receivedAt: now, latencyMs: Number.isFinite(capturedAt) ? Math.round(now - capturedAt) : null,
      handVisible: Boolean(sample), palmOpen: Boolean(sample?.palmOpen), palmOpenHeld: Boolean(sample?.palmOpenHeld),
      partial: Boolean(sample?.partial), visibleFingers: sample?.visibleFingers ?? 0, anchor: sample?.anchor ?? null };
    lastDepthQuality = mode === 'depth' && sample ? window.RebotHandDepth.inspectDepth(pairedDepth, landmarks) : null;
    lastDepth = lastDepthQuality?.distanceMeters ?? null;
    if (mode === 'depth' && Number.isFinite(lastDepth)) {
      if (lastDepth < MIN_DISTANCE_M) nearDistanceBlocked = true;
      else if (lastDepth >= NEAR_RELEASE_M && lastDepthQuality.usable) nearDistanceBlocked = false;
    }
    const needsDepth = mode === 'depth' && (depthController.active || sample?.fistClosed)
      && !sample?.palmOpen && !sample?.palmOpenHeld;
    const interaction = nearDistanceBlocked || (needsDepth && (!lastDepthQuality?.usable || sample?.gestureReliable === false)) ? null : sample;
    const previousProgress = safety.wakeProgress();
    handleSafetyEvent(safety.observe(interaction, now, Number.isFinite(capturedAt) ? capturedAt : now));
    if (previousProgress > 0 && safety.wakeProgress() === 0 && safety.state === 'standby') {
      console.debug('[hand-follow:wake-reset]', lastRecognition);
    }
    if (!sample) {
      const fresh = Number.isFinite(capturedAt) && capturedAt <= now && now - capturedAt <= SAMPLE_MAX_AGE_MS && !document.hidden;
      const missing = mode === 'depth' && depthController.active
        ? fresh ? depthController.observe(null, null, capturedAt)
          : depthController.missing(now, '识别结果已过期，前后跟随已暂停。', 'stale-frame') : null;
      if (missing?.suspended) holdDepthTracking(missing);
      else if (target || depthController.active || depthController.candidateAt !== null || missing?.reset) pauseTracking();
      if (mode === 'depth') reportDepthTransition();
      return;
    }
    sample.grip = pinchGripper.observe(sample.pinchWidthRatio);
    lastRecognition.pinchWidthRatio = sample.pinchWidthRatio;
    lastRecognition.gripperTargetMm = sample.grip === null ? null : sample.grip * 1000;
    context.strokeStyle = '#33d6b0';
    context.lineWidth = 3;
    context.beginPath();
    connections.forEach(([a, b]) => {
      if (!visiblePoint(landmarks[a]) || !visiblePoint(landmarks[b])) return;
      context.moveTo(landmarks[a].x * canvas.width, landmarks[a].y * canvas.height);
      context.lineTo(landmarks[b].x * canvas.width, landmarks[b].y * canvas.height);
    });
    context.stroke();
    context.fillStyle = '#f4f1ea';
    landmarks.forEach((point) => {
      if (!visiblePoint(point)) return;
      context.beginPath();
      context.arc(point.x * canvas.width, point.y * canvas.height, 4, 0, Math.PI * 2);
      context.fill();
    });
    const palm = sample.palm;
    // Accepted packets are already checked for capture age. Track the gap
    // between results separately so healthy, slower inference does not keep
    // discarding the input anchor between consecutive results.
    lastValidResultAt = now;
    lastRecognition.fistClosed = sample.fistClosed;
    lastRecognition.depthMeters = lastDepth;
    lastRecognition.depthQuality = lastDepthQuality?.status ?? null;
    if (safety.state === 'active') {
      if (mode === 'depth') {
        let next = depthController.observe(sample, lastDepth, capturedAt, lastDepthQuality);
        if (nearDistanceBlocked && next.mode === 'planar') next = { ...next, mode: 'hold', status: 'too-near',
          reason: '距离过近，已暂停；请退到 30 cm 以上。' };
        if (next.entered || next.cancelled || next.reset) pauseTracking(false);
        if (next.rebased) {
          if (!sim()?.rebaseWebDepth()) { returnHome('跟随参考未就绪，已归零。请重新唤醒。'); return; }
          depthMotion.reset();
          filtered = filteredGrip = null;
        }
        depthFeedback = next;
        if (next.mode === 'depth') {
          depthMotion.update(next.offset, capturedAt);
          target = { ...sample, depthOffset: next.offset };
        }
        else if (next.mode === 'planar') {
          if (target && target.anchor !== sample.anchor) pauseTracking(false);
          target = sample;
        }
        else if (next.suspended) holdDepthTracking(next);
        else {
          if (target) pauseTracking(false);
          target = null;
        }
      } else {
        if (target && target.anchor !== sample.anchor) pauseTracking(false);
        target = sample;
      }
    }
    if (mode === 'depth') reportDepthTransition();
    context.fillStyle = '#f2a541';
    context.beginPath();
    context.arc(palm.x * canvas.width, palm.y * canvas.height, 9, 0, Math.PI * 2);
    context.fill();
  }

  async function captureFrame(now) {
    if (pendingFrame || !worker || now - lastCapture < 45 || video.readyState < 2 || video.currentTime === lastVideoTime) return;
    const token = session;
    const detectorWorker = worker;
    pendingFrame = true;
    lastCapture = now;
    lastVideoTime = video.currentTime;
    try {
      const bitmap = await createImageBitmap(video);
      if (token !== session || mode !== 'camera') {
        bitmap.close();
        return;
      }
      detectorWorker.postMessage({ type: 'frame', bitmap, timestamp: now }, [bitmap]);
    } catch (error) {
      if (token === session) stop(`相机画面读取失败：${error.message}`);
    }
  }

  function tick(now) {
    const dt = lastTick ? clamp((now - lastTick) / 1000, 0.001, 0.04) : 1 / 60;
    lastTick = now;
    const phase = sim()?.getWebFollowState().phase;
    if (safety.state === 'waking' && phase === 'active') {
      safety.middleReached(now);
      pauseTracking();
    } else if (safety.state === 'returning' && phase === 'standby') safety.homeReached();
    handleSafetyEvent(safety.tick(now));
    if ((mode === 'camera' || mode === 'depth') && !busy && !document.hidden) {
      if (lastValidResultAt && now - lastValidResultAt > RESULT_MAX_GAP_MS && (target || depthController.active)) {
        if (mode === 'depth' && depthController.active) {
          holdDepthTracking(depthController.missing(now, '识别暂时中断，已保持机械臂位置。', 'frame-gap'));
          reportDepthTransition();
        } else pauseTracking();
      }
      if (pendingFrame && now - lastCapture > 2000) stop('识别模块响应超时，相机已关闭并归零。');
      if (mode === 'depth') captureDepthFrame(now);
      else captureFrame(now);
    }
    if (safety.state === 'active' && target && !document.hidden) {
      const blend = 1 - Math.exp(-dt / 0.065);
      if (Number.isFinite(target.depthOffset)) {
        lastFollowResult = sim().followWebDepth(depthMotion.value(now) * (Number($('hand-gain').value) / .4), dt);
      } else {
      if (!filtered) filtered = { x: target.x, y: target.y };
      filtered.x += (target.x - filtered.x) * blend;
      filtered.y += (target.y - filtered.y) * blend;
      if (filteredGrip === null) filteredGrip = sim().getAngles().gripper;
      const gripKnown = Number.isFinite(target.grip);
      if (gripKnown) {
        filteredGrip += (target.grip - filteredGrip) * blend;
        if (Math.abs(target.grip - filteredGrip) <= .00025) filteredGrip = target.grip;
      }
      else filteredGrip = sim().getAngles().gripper;
      lastFollowResult = sim().followWebTarget(filtered, dt, Number($('hand-gain').value), $('hand-gripper').checked && gripKnown ? filteredGrip : undefined);
      }
    }
    if (now - lastUiUpdate > 150) { lastUiUpdate = now; renderSafety(now); }
    frame = requestAnimationFrame(tick);
  }

  function pointerInput(event) {
    if (mode !== 'mouse') return;
    const rect = preview.getBoundingClientRect();
    const x = clamp((event.clientX - rect.left) / rect.width, 0, 1);
    const y = clamp((event.clientY - rect.top) / rect.height, 0, 1);
    const sample = { x, y, grip: event.buttons ? 0 : 0.0715, palmOpen: false };
    handleSafetyEvent(safety.observe(sample, performance.now()));
    if (safety.state === 'active') target = sample;
    $('hand-pointer').hidden = false;
    $('hand-pointer').style.left = `${x * 100}%`;
    $('hand-pointer').style.top = `${y * 100}%`;
  }

  preview.addEventListener('pointermove', pointerInput);
  preview.addEventListener('pointerdown', (event) => {
    if (mode !== 'mouse') return;
    preview.setPointerCapture(event.pointerId);
    pointerInput(event);
  });
  preview.addEventListener('pointerup', pointerInput);
  preview.addEventListener('pointercancel', () => { if (mode === 'mouse') resetAnchor(); });
  preview.addEventListener('pointerleave', (event) => {
    if (mode !== 'mouse' || preview.hasPointerCapture(event.pointerId)) return;
    resetAnchor();
    $('hand-pointer').hidden = true;
  });
  $('hand-start').addEventListener('click', startCamera);
  $('hand-stop').addEventListener('click', () => stop());
  $('hand-mouse').addEventListener('click', startMouse);
  $('hand-recenter').addEventListener('click', resetAnchor);
  $('hand-wake').addEventListener('click', () => {
    if (mode === 'mouse' && safety.wakeMouse()) handleSafetyEvent('wake');
  });
  $('hand-sleep').addEventListener('click', () => returnHome('已休眠归零。先收起手掌，再张开保持 1 秒可重新唤醒。'));
  $('hand-camera').addEventListener('change', () => { if (mode === 'camera' || mode === 'depth') startCamera(); });
  $('hand-gain').addEventListener('input', () => {
    $('hand-gain-value').textContent = `${(Number($('hand-gain').value) / 0.4).toFixed(1)}×`;
    if (mode !== 'off') resetAnchor();
  });
  document.addEventListener('keydown', (event) => { if (event.key === 'Escape' && (mode !== 'off' || busy)) stop(); });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      safety.observe(null, performance.now());
      returnHome('页面离开前台，已归零待机。返回后重新唤醒即可交互。');
    }
  });
  window.addEventListener('pagehide', () => stop());
  navigator.mediaDevices?.addEventListener('devicechange', listCameras);
  listCameras();
  const readyTimer = setInterval(() => {
    updateButtons();
    if (sim()?.isReady() && !readyReported) {
      readyReported = true;
      if (simpleView) sim().prepareWebFollow();
      standbyReason = simpleView ? '机械臂保持零点。开启相机后，张开手掌保持 1 秒唤醒。'
        : '开启手势跟随后归零待机，张开手掌保持 1 秒唤醒。';
      renderSafety(performance.now());
      frame = requestAnimationFrame(tick);
      clearInterval(readyTimer);
    }
  }, 250);
  window.rebotHandFollow = { getState: () => ({ state: safety.state, mode, wakeArmed: safety.wakeArmed,
    tickAt: lastTick,
    depth: { active: depthController.active, paused: depthController.paused, baselineMeters: depthController.baseline,
      distanceMeters: lastDepth, quality: lastDepthQuality ? { ...lastDepthQuality } : null,
      feedback: depthFeedback ? { ...depthFeedback } : null, history: depthHistory.map(entry => ({ ...entry })) },
    wakePaused: safety.wakePaused,
    recognition: lastRecognition ? { ...lastRecognition } : null,
    remainingMs: safety.remainingMs(performance.now()), wakeProgress: safety.wakeProgress(performance.now()) }) };
})();
