const { spawnSync } = require('node:child_process');
const path = require('node:path');
const root = path.resolve(__dirname, '..');
function run(command, args) {
  const result = spawnSync(command, args, { cwd: root, windowsHide: true, stdio: 'inherit' });
  if (result.error) throw result.error;
  if (result.status !== 0) process.exit(result.status || 1);
}
run(process.env.REBOT_DEPTH_PYTHON || 'python', ['-m', 'venv', '.venv-depth']);
const python = path.join(root, '.venv-depth', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');
run(python, ['-m', 'pip', 'install', '--disable-pip-version-check', '--no-deps', 'pyorbbecsdk2==2.1.2']);
run(python, ['-m', 'pip', 'install', '--disable-pip-version-check', 'numpy>=1.24,<2']);
console.log('336 深度采集已安装。打开网页，选择 336 RGB-D 深度模式。');
