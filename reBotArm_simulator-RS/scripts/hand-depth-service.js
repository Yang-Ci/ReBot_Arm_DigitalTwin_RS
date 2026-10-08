const { spawn } = require('node:child_process');
const { randomBytes } = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

module.exports = function createDepthService(root) {
  const python = path.join(root, '.venv-depth', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python');
  let child, ready, endpoint, token;
  function launch() {
    if (ready) return ready;
    if (!fs.existsSync(python)) throw new Error('请先运行 npm run prepare:depth 安装 336 深度采集。');
    token = randomBytes(32).toString('hex');
    ready = new Promise((resolve, reject) => {
      child = spawn(python, ['-u', path.join(root, 'scripts/hand-depth-bridge.py')], {
        cwd: root, windowsHide: true, env: { ...process.env, REBOT_DEPTH_TOKEN: token }, stdio: ['pipe', 'pipe', 'pipe']
      });
      let output = '';
      const timeout = setTimeout(() => { child?.stdin.destroy(); child?.kill(); reject(new Error('深度采集服务启动超时。')); }, 15000);
      child.stdout.on('data', (data) => {
        output = (output + data.toString()).slice(-2000);
        const match = output.match(/REBOT_DEPTH_READY (\d+)/);
        if (match) { clearTimeout(timeout); endpoint = `http://127.0.0.1:${match[1]}`; resolve(); }
      });
      child.stderr.on('data', () => {});
      child.on('error', (error) => { clearTimeout(timeout); ready = endpoint = null; reject(error); });
      child.on('exit', () => {
        clearTimeout(timeout); child = endpoint = ready = null;
        reject(new Error('深度采集服务已退出，请检查安装后重试。'));
      });
    });
    return ready;
  }
  return async function handle(req, res) {
    const route = req.url.split('?')[0].replace('/api/hand-depth', '');
    const allowed = { '/start': 'POST', '/stop': 'POST', '/status': 'GET', '/frame': 'GET' };
    const json = (status, data) => {
      res.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store' });
      res.end(JSON.stringify(data));
    };
    const host = req.headers.host;
    let hostname;
    try { hostname = new URL(`http://${host}`).hostname; } catch (_) { return json(403, { error: 'Invalid local host' }); }
    const localHost = ['localhost', '127.0.0.1', '[::1]'].includes(hostname);
    const loopback = ['127.0.0.1', '::1', '::ffff:127.0.0.1'].includes(req.socket.remoteAddress);
    const sameOrigin = !req.headers.origin || req.headers.origin === `${req.socket.encrypted ? 'https' : 'http'}://${host}`;
    if (!localHost || !loopback || !sameOrigin || req.headers['sec-fetch-site'] === 'cross-site') return json(403, { error: '请在本机 localhost 网页使用深度相机。' });
    if (allowed[route] !== req.method) return json(405, { error: 'Unsupported depth request' });
    try {
      if (!ready && route !== '/start') return json(200, { state: 'stopped', installed: fs.existsSync(python) });
      await launch();
      const response = await fetch(endpoint + route, {
        method: req.method, headers: { Authorization: `Bearer ${token}` }, signal: AbortSignal.timeout(6000)
      });
      res.writeHead(response.status, { 'Content-Type': response.headers.get('content-type') || 'application/json', 'Cache-Control': 'no-store' });
      res.end(Buffer.from(await response.arrayBuffer()));
    } catch (error) { json(503, { error: error.message }); }
  };
};
