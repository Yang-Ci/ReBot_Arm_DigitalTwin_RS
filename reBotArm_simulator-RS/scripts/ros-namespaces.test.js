const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

class BrowserCustomEvent extends Event {
  constructor(type, options = {}) {
    super(type);
    this.detail = options.detail;
  }
}

function consoleFixture(target, withPlacementHooks = false) {
  const elements = new Map();
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      value: ({ 'ros-vlim': '1.20', 'ros-slider-damping': '30', 'ros-trajectory-duration': '2' })[id] || '',
      checked: false, style: {}, dataset: {}, children: [],
      classList: { add() {}, remove() {}, toggle() {}, contains() { return false; } },
      addEventListener() {}, querySelectorAll() { return []; },
      querySelector() { return element(`${id}-child`); },
      prepend() {}, getAttribute() { return null; }
    });
    return elements.get(id);
  }
  const displayed = [];
  const presetFeedbackModes = [], gripperAnimations = [];
  let clock = 1000;
  let visualAngles = {};
  let nextFrame = 1;
  const frames = new Map();
  const window = {
    location: { protocol: 'http:' }, addEventListener() {},
    setTimeout() {}, setInterval() {},
    requestAnimationFrame(callback) { const id = nextFrame++; frames.set(id, callback); return id; },
    cancelAnimationFrame(id) { frames.delete(id); },
    reBotSim: {
      onCommand() {}, setHardwareFeedbackDriven() {}, getAngles() { return { ...visualAngles }; },
      setAngles(angles) { visualAngles = { ...visualAngles, ...angles }; displayed.push(angles); },
      setSimulationPresetFeedbackDriven(enabled) { presetFeedbackModes.push(enabled); },
      setGripperWidth(width, options) { gripperAnimations.push({ width, options }); }
    }
  };
  const context = vm.createContext({
    window, EventTarget, Event, CustomEvent: BrowserCustomEvent, console,
    document: { getElementById: element, createElement: element,
      querySelector() { return null; }, querySelectorAll() { return []; } },
    localStorage: { getItem(key) { return key === 'rebotarm.rs.ros.target' ? target : null; } },
    performance: { now() { return clock; } },
    setTimeout, clearTimeout, requestAnimationFrame() { return 1; }, cancelAnimationFrame() {},
    WebSocket: { OPEN: 1, CONNECTING: 0 }
  });
  for (const name of ['rebot-ros-client.js', 'rebot-ros-ui.js']) {
    const filename = path.join(__dirname, '../public/js/ros', name);
    let source = fs.readFileSync(filename, 'utf8');
    if (name === 'rebot-ros-ui.js' && withPlacementHooks) {
      source = source.replace(/\n\}\)\(\);\s*$/, `
        window.__placementTest = {
          safelyPlaceVisionTarget, moveToPoseViaIkTrajectory, buildVisionCarryOffset, buildVisionPickPlan, buildVisionTransitRoute,
          runVisionMoveStep, publishTeachingReplayGripper, publishGripper, forwardJointBatch, forwardSimCommand,
          getVlim, getSliderDampingMs, getTrajectoryDuration,
          setMocks(m) {
            if (m.move) runVisionMoveStep = m.move;
            if (m.detect) waitForNewVisionTarget = m.detect;
            if (m.tcp) getVisionGraspTcpPosition = m.tcp;
            if (m.support) waitForStableTableSupport = m.support;
            if (m.grip) commandGripperAndWait = m.grip;
            if (m.width) readGripperFeedbackPosition = m.width;
            if (m.release) releaseSimCarriedObject = m.release;
            if (m.call) guardedCall = m.call;
            if (m.trajectory) sendTrajectory = m.trajectory;
            if (m.goal) sendVisionMoveGoal = m.goal;
            if (m.settle) waitForArmMotionSettled = m.settle;
            if (m.allow) controlAllowed = m.allow;
          }
        };
      })();`);
    }
    vm.runInContext(source, context, { filename });
  }
  return { client: window.reBotRos, Client: window.ReBotRosClient, displayed, presetFeedbackModes, gripperAnimations,
    element, placement: window.__placementTest,
    setVisual(angles) { visualAngles = { ...angles }; },
    frame(ms) { clock += ms; const queued = [...frames.values()]; frames.clear(); queued.forEach(callback => callback(clock)); },
    advance(ms) { clock += ms; } };
}

for (const [target, namespace] of [['hardware', 'rebot_arm_rs'], ['simulation', 'rebot_arm_rs_sim']]) {
  test(`${target}: feedback and all command types stay in /${namespace}`, async () => {
    const { client } = consoleFixture(target);
    assert.equal(client.namespace, namespace);
    const subscriptions = [...client._subscriptions.keys()];
    assert(subscriptions.includes(`/${namespace}/joint_states`));
    assert.equal(subscriptions.includes(`/${namespace}/mujoco/joint_states`), target === 'simulation');
    assert(subscriptions.every(topic => topic.startsWith(`/${namespace}/`)));

    const messages = [];
    client.connected = true;
    client.socket = { readyState: 1, send(data) {
      const message = JSON.parse(data);
      messages.push(message);
      if (message.op === 'call_service') queueMicrotask(() => client._handleMessage({
        data: JSON.stringify({ op: 'service_response', id: message.id, result: true, values: { success: true } })
      }));
    } };
    await client.enable();
    await client.disable();
    await client.safeHome();
    await client.setGripper(0.02);
    const actions = [];
    client.sendActionGoal = async (name) => { actions.push(name); return { success: true }; };
    await client.moveToPose({ position: { x: 0.3, y: 0, z: 0.2 } }, 2);
    await client.followJointTrajectory(['joint1'], [{ positions: [0.1], time_from_start: { sec: 1, nanosec: 0 } }]);
    assert.deepEqual(actions, [`/${namespace}/move_to_pose`, `/${namespace}/follow_joint_trajectory`]);
    client.publishJointCommand('joint1', 0.1, { vlim: 0.2 });
    assert.deepEqual(messages.filter(m => m.op === 'call_service').map(m => m.service), [
      `/${namespace}/enable`, `/${namespace}/disable`, `/${namespace}/safe_home`, `/${namespace}/gripper/set`
    ]);
    assert(messages.some(m => m.op === 'publish' && m.topic === `/${namespace}/joints/joint1/cmd/mit`));
    assert(messages.every(m => (m.service || m.topic || '').startsWith(`/${namespace}/`)));
  });
}

test('simulation mirrors MuJoCo actual state, ignores driver targets while fresh, then falls back', () => {
  const { client, displayed, advance } = consoleFixture('simulation');
  const deliver = (suffix, position) => client._subscriptions.get(`/rebot_arm_rs_sim/${suffix}`).callback({
    name: ['joint1'], position: [position]
  });
  deliver('joint_states', 0.1);
  assert.equal(displayed.at(-1).joint1, 0.1);
  deliver('mujoco/joint_states', 0.2);
  assert.equal(displayed.at(-1).joint1, 0.2);
  const count = displayed.length;
  deliver('joint_states', 0.8);
  assert.equal(displayed.length, count);
  advance(601);
  deliver('joint_states', 0.3);
  assert.equal(displayed.at(-1).joint1, 0.3);
});

test('fresh browser preferences select simulation; a standalone client defaults to hardware', () => {
  const { client, Client } = consoleFixture(null);
  assert.equal(client.namespace, 'rebot_arm_rs_sim');
  assert.equal(new Client().namespace, 'rebot_arm_rs');
});

for (const target of ['hardware', 'simulation']) {
  test(`${target}: safety panel visibility and command defaults`, () => {
    const { client, placement, element } = consoleFixture(target, true);
    assert.equal(element('ros-safety-section').hidden, target === 'simulation');
    client.connected = true;
    placement.setMocks({ allow: () => true });
    const commands = [];
    client.publishJointCommand = (name, position, options) => commands.push({ name, position, options });
    placement.forwardSimCommand({ type: 'joint', name: 'joint1', value: 0.1, source: 'drag', phase: 'commit' });
    assert.equal(commands.length, 1);
    assert.equal(commands[0].options.vlim, 1.2);
    assert.equal(placement.getVlim(), 1.2);
    assert.equal(placement.getSliderDampingMs(), 30);
    assert.equal(placement.getTrajectoryDuration(), 2);
    if (target === 'hardware') {
      element('ros-vlim').value = '0.3';
      placement.forwardSimCommand({ type: 'joint', name: 'joint1', value: 0.2, source: 'drag', phase: 'commit' });
      assert.equal(commands.at(-1).options.vlim, 0.3);
      element('ros-slider-damping').value = '60';
      element('ros-trajectory-duration').value = '3';
      assert.equal(placement.getSliderDampingMs(), 60);
      assert.equal(placement.getTrajectoryDuration(), 3);
    }
  });
}

test('simulation presets use connected feedback and gripper replay does not mask it', () => {
  const { client, element, presetFeedbackModes, gripperAnimations, placement, displayed } = consoleFixture('simulation', true);
  assert.equal(element('ros-mirror').checked, true);
  client.connected = true;
  client.dispatchEvent(new BrowserCustomEvent('status', { detail: { state: 'open' } }));
  assert.equal(presetFeedbackModes.at(-1), true);
  client.publishGripperCommand = () => {};
  const deliver = client._subscriptions.get('/rebot_arm_rs_sim/mujoco/joint_states').callback;
  deliver({ name: ['joint1', 'gripper_joint1'], position: [0.4, 0.02] });
  gripperAnimations.length = 0;
  placement.publishTeachingReplayGripper(0.01);
  assert.equal(gripperAnimations.length, 0);
  deliver({ name: ['joint1', 'gripper_joint1'], position: [0.2, 0.015] });
  assert.equal(displayed.at(-1).joint1, 0.2);
  assert.equal(gripperAnimations.length, 1);
  assert.equal(gripperAnimations[0].options.source, 'ros');
  client.connected = false;
  client.dispatchEvent(new BrowserCustomEvent('status', { detail: { state: 'closed' } }));
  assert.equal(presetFeedbackModes.at(-1), false);
});

test('simulation gripper follows each actual opening and closing sample while a target is pending', () => {
  const { client, placement, gripperAnimations, advance } = consoleFixture('simulation', true);
  client.connected = true;
  client.publishGripperCommand = () => {};
  const deliver = client._subscriptions.get('/rebot_arm_rs_sim/mujoco/joint_states').callback;
  const sample = (position) => deliver({ name: ['gripper_joint1'], position: [position] });
  sample(0);
  placement.publishGripper(0.0715);
  gripperAnimations.length = 0;
  sample(0.01);
  sample(0.02);
  assert.equal(gripperAnimations.length, 2);
  assert(Math.abs(gripperAnimations[0].width - 0.0143) < 1e-9);
  assert(Math.abs(gripperAnimations[1].width - 0.0286) < 1e-9);
  assert(gripperAnimations.every(item => item.options.source === 'ros' && item.options.forceFeedback));

  placement.publishGripper(0);
  sample(0.015);
  assert(Math.abs(gripperAnimations.at(-1).width - 0.02145) < 1e-9);

  advance(601);
  client._subscriptions.get('/rebot_arm_rs_sim/gripper/state').callback({ position: 1.0 });
  assert(Math.abs(gripperAnimations.at(-1).width - 0.0143) < 1e-9);
});

test('vision motion steps do not publish target markers', async () => {
  const { client, placement } = consoleFixture('simulation', true);
  let markers = 0, motions = 0;
  client.publishTargetPose = () => { markers += 1; };
  placement.setMocks({ goal: async () => { motions += 1; return { success: true, localPlayback: true }; }, settle: async () => {} });
  await placement.runVisionMoveStep({ position: { x: 0.3, y: 0, z: 0.2 } }, 1, 'test');
  assert.equal(motions, 1);
  assert.equal(markers, 0);
});

test('stored simulation objects use reachable approach, grasp and lift poses', () => {
  for (const [color, y, z] of [['red', -0.16, 0.1225], ['yellow', 0.16, 0.126], ['blue', 0, 0.119]]) {
    const { placement } = consoleFixture('simulation', true);
    const target = { color, x: 0.498, y, z, width_m: 0.04, height_m: 0.04, grasp_yaw_rad: 0 };
    const plan = placement.buildVisionPickPlan(target);
    assert.equal(plan.storagePick, true);
    const pitch = pose => 2 * Math.atan2(pose.orientation.y, pose.orientation.w) * 180 / Math.PI;
    assert(Math.abs(pitch(plan.approachPose) - 50) < 1e-9);
    assert(Math.abs(pitch(plan.graspPose) - 65) < 1e-9);
    assert(Math.abs(pitch(plan.firstLiftPose) - 50) < 1e-9);
    assert.equal(placement.buildVisionTransitRoute(target, plan)[0].pose.orientation.y, plan.approachPose.orientation.y);
  }
  const target = { color: 'yellow', x: 0.33, y: -0.2, z: 0.126 };
  assert.equal(consoleFixture('simulation', true).placement.buildVisionPickPlan(target).storagePick, false);
  target.x = 0.49; target.y = 0.16;
  assert.equal(consoleFixture('hardware', true).placement.buildVisionPickPlan(target).storagePick, false);
});

test('simulation drag release blends all joints into live feedback and can be interrupted', () => {
  const { client, placement, displayed, setVisual, frame } = consoleFixture('simulation', true);
  client.connected = true;
  const commands = [];
  client.publishJointCommand = (name, value) => commands.push({ name, value });
  placement.setMocks({ allow: () => true });
  const names = Array.from({ length: 6 }, (_, i) => `joint${i + 1}`);
  const positions = (value) => names.map(() => value);
  const joints = (value) => Object.fromEntries(names.map(name => [name, value]));
  const deliver = client._subscriptions.get('/rebot_arm_rs_sim/mujoco/joint_states').callback;
  deliver({ name: names, position: positions(0.2) });
  setVisual(joints(1));
  placement.forwardJointBatch({ joints: joints(1), source: 'tcp-drag-commit', handoffToFeedback: true });
  assert.deepEqual(commands.map(command => command.name), names);
  assert(commands.every(command => command.value === 1));
  const count = displayed.length;
  deliver({ name: names, position: positions(0.3) });
  assert.equal(displayed.length, count);
  frame(175);
  assert(Math.abs(displayed.at(-1).joint1 - 0.65) < 1e-9);
  deliver({ name: names, position: positions(0.4) });
  frame(175);
  assert(Math.abs(displayed.at(-1).joint1 - 0.4) < 1e-9);
  deliver({ name: names, position: positions(0.5) });
  assert.equal(displayed.at(-1).joint1, 0.5);

  setVisual(joints(1));
  placement.forwardJointBatch({ joints: joints(1), source: 'tcp-drag-commit', handoffToFeedback: true });
  placement.forwardSimCommand({ type: 'tcp-drag-start' });
  const beforeFrame = displayed.length;
  frame(175);
  assert.equal(displayed.length, beforeFrame);
});

test('storage release opens in small steps and first lifts straight from the measured position', async () => {
  const { placement } = consoleFixture('simulation', true);
  const moves = [], openings = [];
  const object = { color: 'yellow', x: 0.49, y: 0.16, z: 0.126, width_m: 0.04, height_m: 0.04 };
  const readings = [ { ...object, z: 0.25 }, object, object, object, object ];
  let released = false;
  placement.setMocks({
    move: async (pose, duration, label, options) => moves.push({ pose, label, options }),
    detect: async () => readings.shift(),
    tcp: () => ({ x: 0.482, y: 0.158, z: moves.length < 2 ? 0.25 : 0.14 }),
    support: async () => true,
    width: () => 0.03,
    grip: async (width) => openings.push(width),
    release: () => { released = true; }
  });
  const transfer = { position: { x: 0.49, y: 0.16, z: 0.25 }, orientation: { w: 1 } };
  await placement.safelyPlaceVisionTarget(object, 2, 'test', {
    storageZone: { x: 0.49, y: 0.16, z: 0.126 }, transferPose: transfer, retreatPose: transfer
  });
  assert.equal(released, true);
  assert.deepEqual(openings.map(v => Math.round(v * 1000)), [34, 38, 42, 72]);
  const clear = moves.find(move => move.label.includes('竖直抬起夹爪'));
  assert(clear);
  assert.equal(clear.pose.position.x, 0.482);
  assert.equal(clear.pose.position.y, 0.158);
  assert(Math.abs(clear.pose.position.z - 0.19) < 1e-9);
  assert.equal(clear.options.strictIk, true);
});

test('repeat placement keeps the reachable pitch and avoids nudging an already aligned object', async () => {
  const { placement } = consoleFixture('simulation', true);
  const moves = [];
  const object = { color: 'yellow', x: 0.49, y: 0.16, z: 0.126, width_m: 0.04, height_m: 0.04 };
  const settled = { ...object, x: 0.482 };
  const readings = [{ ...object, z: 0.25 }, settled, settled, settled, settled];
  placement.setMocks({
    move: async (pose, duration, label) => moves.push({ pose, label }),
    detect: async () => readings.shift(),
    tcp: () => ({ x: 0.49, y: 0.16, z: moves.length < 2 ? 0.25 : 0.14 }),
    support: async () => true, width: () => 0.03, grip: async () => {}, release: () => {}
  });
  const transfer = { position: { x: 0.49, y: 0.16, z: 0.25 }, orientation: { w: 1 } };
  await placement.safelyPlaceVisionTarget(object, 2, 'test', {
    storageZone: { x: 0.49, y: 0.16, z: 0.126 }, transferPose: transfer, retreatPose: transfer,
    placePitch: 65 * Math.PI / 180, placementTolerance: 0.012
  });
  const descent = moves.find(move => move.label.includes('下探'));
  assert(Math.abs(2 * Math.atan2(descent.pose.orientation.y, descent.pose.orientation.w) * 180 / Math.PI - 65) < 1e-9);
  assert(!moves.some(move => move.label.includes('下探后微调')));
});

test('storage does not open the gripper without stable table support', async () => {
  const { placement } = consoleFixture('simulation', true);
  const openings = [], moves = [];
  const object = { color: 'yellow', x: 0.49, y: 0.16, z: 0.126, width_m: 0.04, height_m: 0.04 };
  const readings = [{ ...object, z: 0.25 }, object];
  placement.setMocks({
    move: async (pose, duration, label) => moves.push(label),
    detect: async () => readings.shift(),
    tcp: () => ({ x: 0.49, y: 0.16, z: 0.14 }),
    support: async () => false,
    grip: async (width) => openings.push(width)
  });
  const transfer = { position: { x: 0.49, y: 0.16, z: 0.25 }, orientation: { w: 1 } };
  await assert.rejects(placement.safelyPlaceVisionTarget(object, 2, 'test', {
    storageZone: { x: 0.49, y: 0.16, z: 0.126 }, transferPose: transfer, retreatPose: transfer
  }), /未确认物体接触桌面/);
  assert.equal(moves.filter(label => label.includes('缓慢落到桌面')).length, 4);
  assert.equal(openings.length, 0);
});

test('an inaccurate storage IK solution is not executed', async () => {
  const { placement } = consoleFixture('simulation', true);
  let trajectories = 0;
  placement.setMocks({
    call: async () => ({ success: false, message: 'IK error 50mm', q_solution: [0, 0, 0, 0, 0, 0] }),
    trajectory: async () => { trajectories += 1; }
  });
  const result = await placement.moveToPoseViaIkTrajectory({ position: { x: 0.49, y: 0, z: 0.15 } }, 2, 'test', { strictIk: true });
  assert.equal(result.success, false);
  assert.equal(trajectories, 0);
});
