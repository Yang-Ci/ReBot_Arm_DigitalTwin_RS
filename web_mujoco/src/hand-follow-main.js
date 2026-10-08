import './hand-follow.css';
import { loadMujocoModule, loadRsScene } from './load-model.js';
import { bindJoints, readAngles } from './kinematics.js';
import { createPhysicsController } from './pd-control.js';
import { createSceneView } from './scene-view.js';
import { mapHandPose } from './hand-joint-mapping.js';

const iframe = document.getElementById('hand-controller');
const loading = document.getElementById('hand-loading');
const status = document.getElementById('mapping-status');
const errorLabel = document.getElementById('tracking-error');
let ready = false, physics, data, joints, sourceState = null, lastSourceAt = 0;
let lastSourceTick = -Infinity, holdApplied = false;
let sourceApi = null;

function source() {
  try { return iframe.contentWindow; } catch { return null; }
}
function closeCamera() {
  try { source()?.document.getElementById('hand-stop')?.click(); } catch { /* Frame already gone. */ }
}
window.addEventListener('pagehide', closeCamera);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) {
    try { source()?.document.getElementById('hand-sleep')?.click(); } catch { /* Source unavailable. */ }
  }
});

async function main() {
  const scene = await loadRsScene(loadMujocoModule(), progress => {
    loading.textContent = progress?.key === 'status.loadingAssets'
      ? `正在加载机械臂模型 ${progress.vars.done}/${progress.vars.total} · ${progress.vars.mb} MB`
      : '正在加载 MuJoCo 和机械臂模型…';
  });
  const { mujoco, model, materialProps } = scene;
  const tcpBodyId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, 'gripper_end');
  if (tcpBodyId < 0) throw new Error('缺少 MuJoCo 末端 gripper_end。');
  data = scene.data; joints = bindJoints(mujoco, model);
  physics = createPhysicsController(mujoco, model, data, joints);
  physics.reset();
  const view = createSceneView(document.getElementById('viewport'));
  view.build(mujoco, model, materialProps); view.sync(data); view.render();
  ready = true; loading.hidden = true;
  iframe.src = `${import.meta.env.BASE_URL}hand-source/?view=hand-follow&embed=mujoco`;
  let previousAt = performance.now(), accumulator = 0, lastUiAt = 0;
  const loop = now => {
    const dt = Math.min(.05, Math.max(0, (now - previousAt) / 1000)); previousAt = now;
    let connected = false;
    try {
      const child = source();
      if (child?.rebotHandFollow !== sourceApi) { sourceApi = child?.rebotHandFollow; lastSourceTick = -Infinity; }
      const state = sourceApi?.getState();
      const tickAt = state?.tickAt;
      if (Number.isFinite(tickAt) && tickAt > lastSourceTick) {
        lastSourceTick = tickAt; lastSourceAt = now; sourceState = state;
      }
      connected = Boolean(child?.reBotSim?.isReady() && now - lastSourceAt < 600);
      const pose = connected && mapHandPose(child.reBotSim.getAngles());
      if (pose) { physics.setTargets(pose); holdApplied = false; }
      else if (!holdApplied) { physics.setTargets(readAngles(data, joints)); holdApplied = true; }
    } catch {
      if (!holdApplied) { physics.setTargets(readAngles(data, joints)); holdApplied = true; }
    }
    accumulator = Math.min(.05, accumulator + dt);
    const steps = Math.floor(accumulator / model.opt.timestep);
    if (steps) { physics.step(steps); accumulator -= steps * model.opt.timestep; }
    view.sync(data); view.render();
    if (now - lastUiAt > 150) {
      lastUiAt = now;
      const labels = { standby:'零点待机', waking:'正在进入中位', active:'手势跟随', returning:'平滑归零' };
      status.textContent = connected ? `MuJoCo · ${labels[sourceState?.state] || '等待手势控制'}` : 'MuJoCo 已就绪 · 等待手势控制';
      const samples = physics.telemetry().joints;
      const armError = Math.max(...samples.filter(joint => joint.name !== 'joint7').map(joint => Math.abs(joint.error)));
      errorLabel.textContent = `关节误差 ${(armError * 180 / Math.PI).toFixed(2)}° · 夹爪误差 ${(Math.abs(samples.at(-1).error) * 1000).toFixed(1)} mm`;
    }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
  window.rebotMujocoHand = { getState: () => ({ ready, source: sourceState,
    targets: { ...physics.targets }, actual: readAngles(data, joints), telemetry: physics.telemetry(),
    tcp: { x: data.xpos[tcpBodyId * 3], y: data.xpos[tcpBodyId * 3 + 1], z: data.xpos[tcpBodyId * 3 + 2] },
    simulationTime: data.time, sourceAgeMs: performance.now() - lastSourceAt }) };
}
main().catch(error => {
  console.error(error); closeCamera();
  loading.textContent = `MuJoCo 加载失败：${error.message}`;
});
