const fs = require('fs');
const path = require('path');
const { execFile } = require('child_process');
const { promisify } = require('util');
const runFile = promisify(execFile);

const root = path.resolve(__dirname, '..');
const output = path.join(root, 'public', 'lib', 'mediapipe');
const model = path.join(root, 'public', 'models', 'hand_landmarker.task');
const modelUrl = 'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task';

async function prepare() {
  const sdk = path.dirname(require.resolve('@mediapipe/tasks-vision'));
  fs.mkdirSync(output, { recursive: true });
  fs.copyFileSync(path.join(sdk, 'vision_bundle.mjs'), path.join(output, 'vision_bundle.mjs'));
  fs.cpSync(path.join(sdk, 'wasm'), path.join(output, 'wasm'), { recursive: true });
  if (!fs.existsSync(model)) {
    console.log('Downloading the hand landmark model...');
    fs.mkdirSync(path.dirname(model), { recursive: true });
    try {
      // curl also works with the HTTP(S) proxy environment commonly used by
      // local development machines; Node fetch does not do so on Node 18/20.
      await runFile(process.platform === 'win32' ? 'curl.exe' : 'curl', [
        '--fail', '--location', '--connect-timeout', '10', '--max-time', '60',
        '--output', `${model}.tmp`, modelUrl
      ], { windowsHide: true });
    } catch (error) {
      if (error.code !== 'ENOENT') throw error;
      const response = await fetch(modelUrl, { signal: AbortSignal.timeout(30000) });
      if (!response.ok) throw new Error(`Hand model download failed: HTTP ${response.status}`);
      fs.writeFileSync(`${model}.tmp`, Buffer.from(await response.arrayBuffer()));
    }
    if (fs.statSync(`${model}.tmp`).size < 1000000) throw new Error('Hand model download is incomplete');
    fs.renameSync(`${model}.tmp`, model);
  }
  console.log('Hand tracking assets are ready (served locally).');
}

prepare().catch((error) => {
  console.error(error.message);
  process.exitCode = 1;
});
