'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');
const vm = require('node:vm');

const root = path.join(__dirname, '..', 'public', 'js');
const simSource = fs.readFileSync(path.join(root, 'rebot-sim.js'), 'utf8');
const rosSource = fs.readFileSync(path.join(root, 'ros', 'rebot-ros-ui.js'), 'utf8');

function section(source, start, end) {
  const first = source.indexOf(start);
  const last = source.indexOf(end, first + start.length);
  assert.ok(first >= 0 && last > first, `missing section: ${start}`);
  return source.slice(first, last);
}

async function finishFromRecordButton(mode, stopResult) {
  const calls = [];
  const listeners = new Map();
  const window = {
    addEventListener: (name, listener) => listeners.set(name, listener),
    dispatchEvent: (event) => listeners.get(event.type)?.(event),
    reBotSim: { endHardwareTeaching: () => calls.push('end recording') }
  };
  const context = vm.createContext({
    window, CustomEvent: class { constructor(type) { this.type = type; } },
    teachingRecording: true, teachingSource: 'hardware', teachingMode: mode,
    teachingPlayback: null, hardwareTeachActive: true, hardwareTeachBusy: false,
    hardwareTeachModeConfirmed: true, hardwareTeachGravityStarted: true,
    client: { connected: true, stopGravityCompensation: () => { calls.push('stop gravity'); return stopResult; },
      safeHome: () => { calls.push('home'); return { success: true }; } },
    els: { teachHomeGripperMode: { value: 'hold' } },
    guardedCall: async (call) => call(), updateHardwareTeachUi: () => {},
    writeLog: () => {}, cancelLowLevelPlayback: () => {}, resetWebControlState: () => {},
    t: (key) => key
  });
  vm.runInContext(section(simSource, '  function toggleTeachingRecord()', '  function recordTeachingWaypoint('), context);
  vm.runInContext(section(rosSource, '  async function toggleHardwareTeaching()', '  function finishHardwareTeachLocal('), context);
  vm.runInContext(section(rosSource, " window.addEventListener('rebot-hardware-teach-finish'", " els.enable.addEventListener("), context);
  vm.runInContext('toggleTeachingRecord()', context);
  // The event listener invokes the async stop without awaiting it.
  await new Promise((resolve) => setImmediate(resolve));
  return calls;
}

for (const mode of ['endpoint', 'path']) {
  test(`record button ends hardware ${mode} teaching and homes`, async () => {
    assert.deepEqual(await finishFromRecordButton(mode, { success: true }),
      ['end recording', 'stop gravity', 'home']);
  });
}

test('failed gravity stop does not command home', async () => {
  assert.deepEqual(await finishFromRecordButton('endpoint', { success: false }),
    ['end recording', 'stop gravity']);
});
