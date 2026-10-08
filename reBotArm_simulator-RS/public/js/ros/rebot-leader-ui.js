(function () {
  'use strict';
  const client = window.reBotRos;
  const card = document.getElementById('leader-panel');
  if (!client || !card) return;
  const $ = (id) => document.getElementById('leader-' + id);
  const controlLock = document.getElementById('ros-control-enable');
  const service = '/' + client.namespace + '/leader/control';
  const topic = '/' + client.namespace + '/leader/status';
  const model = window.LeaderModelPreview ? new window.LeaderModelPreview(document.getElementById('leader-model')) : null;
  window.reBotLeaderModel = model;
  const speedInput = $('speed');
  if (speedInput) {
    speedInput.max = '1';
  }
  const words = {
    title: ['Leader 遥操作', 'Leader teleoperation'],
    scan: ['扫描主臂', 'Scan leader'], connect: ['连接', 'Connect'], disconnect: ['断开主臂', 'Disconnect leader'],
    port: ['主臂串口（ROS 主机）', 'Leader port (ROS host)'],
    device: ['扫描结果', 'Scan results'], zero: ['已摆好主臂零位并闭合夹爪', 'Leader is at reference zero with gripper closed'],
    unlock: ['解锁主臂', 'Unlock leader'],
    calibrate: ['确认并设置主臂零位', 'Confirm and set leader zero'],
    speed: ['目标速度（rad/s）', 'Target speed (rad/s)'],
    gripper: ['跟随夹爪', 'Follow gripper'], absolute: ['绝对跟随（需主从对齐）', 'Absolute follow (aligned arms required)'],
    start: ['开始跟随', 'Start following'], pause: ['暂停保持', 'Pause and hold'], resume: ['继续跟随', 'Resume'],
    stop: ['停止并释放控制', 'Stop and release control'],
    tip: ['扫描 ROS 主机的串口。连接预览后解锁主臂、摆好零位，再确认校准；默认相对跟随。',
          'Scan ports on the ROS host. Preview, unlock and align the leader, then confirm zero. Relative follow is the default.'],
    waiting: ['等待 ROS 主臂服务', 'Waiting for ROS leader service'],
    stale: ['主臂状态已过期；请检查 ROS 连接', 'Leader status is stale; check ROS connection'],
    lock: ['先打开 ROS 控制锁', 'Enable the ROS control lock first'],
    pending: ['正在处理…', 'Working…'],
    unknown: ['操作结果待确认；已停止心跳，等待控制器超时保持',
              'Result unconfirmed; heartbeat stopped, waiting for controller hold'],
    owned: ['另一页面正在控制主臂', 'Another page owns the leader session'],
    partial: ['不完整', 'Incomplete'], full: ['完整 Leader', 'Complete leader'],
    missing: ['未发现串口；检查 USB 透传和权限，也可手动填端口',
              'No ports found; check USB passthrough/permissions, or enter a port manually'],
    error: ['操作失败：', 'Operation failed: '],
    states: {
      DISCONNECTED: ['未连接', 'Disconnected'], CONNECTING: ['连接中', 'Connecting'],
      CALIBRATING: ['校准中', 'Calibrating'], PREVIEW: ['只读预览', 'Read-only preview'],
      READY: ['已就绪', 'Ready'], FOLLOWING: ['跟随中', 'Following'], PAUSED: ['已暂停', 'Paused'], FAULT: ['故障', 'Fault']
    }
  };
  let status = null;
  let statusAt = 0;
  let session = '';
  let busy = false;
  let heartbeatBusy = false;
  let generation = 0;
  let message = '';
  let messageUntil = 0;
  const language = () => window.rebotI18n?.getLang() === 'en' ? 1 : 0;
  const t = (key) => words[key][language()];
  const active = () => Boolean(status?.session_id);
  const ours = () => Boolean(session && session === status?.session_id);
  const fresh = () => client.connected && statusAt && Date.now() - statusAt < 1200;
  const inRos = () => !window.reBotControlMode || window.reBotControlMode.is('ros');

  function render() {
    card.querySelectorAll('[data-leader-text]').forEach((element) => {
      element.textContent = t(element.dataset.leaderText);
    });
    $('status').textContent = fresh()
      ? (words.states[status.state]?.[language()] || status.state) : t('waiting');
    if (!busy && Date.now() > messageUntil) message = '';
    $('message').textContent = message || (fresh() ? status.message : (statusAt ? t('stale') : t('waiting')));
    const occupied = active();
    const connected = Boolean(status?.port) && status?.state !== 'DISCONNECTED';
    $('scan').disabled = busy || !client.connected || connected || occupied;
    $('connect').disabled = busy || !client.connected || occupied || !inRos();
    $('calibrate').disabled = busy || !fresh() || !connected || occupied || !$('zero').checked;
    $('unlock').disabled = busy || !fresh() || !connected || occupied;
    $('disconnect').disabled = busy || !client.connected || !connected || (occupied && !ours());
    $('start').disabled = busy || !fresh() || !status.calibrated || occupied || !inRos() || !controlLock?.checked || status.sample_age < 0 || status.sample_age >= 0.3;
    $('pause').disabled = busy || !fresh() || !ours() || !status.following;
    $('resume').disabled = busy || !fresh() || !ours() || !status.paused || !controlLock?.checked || !inRos();
    $('stop').disabled = busy || !client.connected || !session;
    ['port', 'devices', 'speed', 'absolute', 'gripper', 'zero'].forEach((id) => { $(id).disabled = busy || occupied; });
    const values = status?.angles_deg || [];
    $('angles').textContent = values.length === 7 ? values.map((v, i) =>
      (i < 6 ? 'J' + (i + 1) : 'G') + ': ' + Number(v).toFixed(1) + '°').join('  ') : 'J1–J6 / G: —';
    $('rate').textContent = fresh() && status.sample_age >= 0
      ? `${Number(status.sample_hz).toFixed(1)} Hz · ${Math.round(status.sample_age * 1000)} ms` : '—';
    window.reBotLeaderActive = occupied;
    model?.update(status, fresh(), inRos());
    if (occupied && !ours()) $('message').textContent = t('owned');
  }

  async function call(operation, args) {
    const result = await client.callService(service, 'rebotarm_msgs/srv/LeaderControl', {
      operation, port: $('port').value.trim(), session_id: session,
      speed: Number($('speed').value), follow_gripper: $('gripper').checked,
      absolute: $('absolute').checked, confirm_zero: $('zero').checked, ...args
    }, { timeoutMs: operation === 'scan' ? 35000 : 8000, retryOnTimeout: false });
    if (!result.success) throw new Error(result.message || operation + ' failed');
    return result;
  }

  async function perform(operation) {
    if (busy) return;
    if ((operation === 'start' || operation === 'resume') && !controlLock?.checked) {
      message = t('lock'); render(); return;
    }
    if (operation === 'calibrate' && !$('zero').checked) return;
    if (operation === 'start') {
      if (window.rebotHandFollow?.getState().mode && window.rebotHandFollow.getState().mode !== 'off') {
        message = language() ? 'Stop hand following before starting the leader' : '请先关闭手势跟随，再启动主臂';
        messageUntil = Date.now() + 8000; render(); return;
      }
      window.reBotSim?.stopMotion?.();
      window.dispatchEvent(new CustomEvent('rebot-leader-starting'));
    }
    const requestGeneration = generation;
    busy = true; message = t('pending'); render();
    try {
      const result = await call(operation);
      if (requestGeneration !== generation || !inRos()) {
        if (result.session_id && client.connected) {
          await call('stop', { session_id: result.session_id });
        }
        return;
      }
      if (operation === 'start' || operation === 'resume') session = result.session_id;
      if (operation === 'stop' || operation === 'disconnect') session = '';
      if (['connect', 'unlock', 'disconnect'].includes(operation)) $('zero').checked = false;
      if (operation === 'scan') {
        $('devices').replaceChildren();
        for (const device of result.devices || []) {
          const option = document.createElement('option');
          option.value = device.port;
          option.textContent = `${device.port} · ${device.description} · ${device.complete ? t('full') : t('partial')}${device.error ? ' · ' + device.error : ''}`;
          $('devices').appendChild(option);
        }
        if ($('devices').options.length) $('port').value = $('devices').value;
        else message = t('missing');
      }
      if (!message || message === t('pending')) message = result.message;
    } catch (error) {
      message = t('error') + error.message;
      if (['start', 'resume', 'stop', 'disconnect'].includes(operation)) {
        session = ''; message += ' · ' + t('unknown');
      }
    } finally {
      messageUntil = Date.now() + 8000;
      busy = false; render();
    }
  }

  client.subscribe(topic, 'rebotarm_msgs/msg/LeaderStatus', (value) => {
    status = value; statusAt = Date.now();
    if (session && !value.session_id) session = '';
    render();
  }, { throttleRate: 100 });
  client.addEventListener('status', (event) => {
    if (event.detail?.state !== 'open') {
      generation++; session = ''; statusAt = 0;
      // The controller's heartbeat watchdog ends the previous page session.
    }
    render();
  });
  for (const operation of ['scan', 'connect', 'unlock', 'calibrate', 'disconnect', 'start', 'pause', 'resume', 'stop']) {
    $(operation).addEventListener('click', () => void perform(operation));
  }
  $('devices').addEventListener('change', () => { $('port').value = $('devices').value; });
  $('zero').addEventListener('change', render);
  const endSession = () => {
    generation++;
    if (!session) { render(); return; }
    const old = session;
    session = '';
    if (client.connected) void call('stop', { session_id: old }).catch((error) => {
      message = t('error') + error.message + ' · ' + t('unknown');
      messageUntil = Date.now() + 8000; render();
    });
    render();
  };
  controlLock?.addEventListener('change', () => { if (!controlLock.checked) endSession(); render(); });
  window.addEventListener('rebot-control-mode-change', (event) => {
    if (event.detail.mode !== 'ros') endSession();
    render();
  });
  window.addEventListener('pagehide', endSession);
  window.rebotI18n?.onLangChange(render);
  window.reBotLeader = { stop: endSession, owned: active };
  setInterval(async () => {
    render();
    if (!session || !client.connected || !fresh() || !inRos() || !controlLock?.checked || heartbeatBusy) return;
    heartbeatBusy = true;
    const heartbeatSession = session;
    try { await call('heartbeat'); } catch (error) {
      if (session === heartbeatSession) {
        session = ''; message = t('error') + error.message;
        messageUntil = Date.now() + 8000;
      }
    } finally { heartbeatBusy = false; render(); }
  }, 250);
  render();
})();
