import { spawn } from 'node:child_process';
import path from 'node:path';

export function handFollowSource(root) {
  let child = null, startup = null;
  async function available() {
    try {
      const response = await fetch('http://localhost:3002/api/hand-depth/status', { signal: AbortSignal.timeout(1200) });
      return response.ok && typeof (await response.json()).state === 'string';
    } catch { return false; }
  }
  async function ensureSource() {
    if (startup) return startup;
    startup = (async () => {
      if (await available()) return;
      const sourceRoot = path.resolve(root, '../reBotArm_simulator-RS');
      child = spawn(process.execPath, [path.join(sourceRoot, 'server.js')], {
        cwd: sourceRoot, windowsHide: true, env: { ...process.env, PORT: '3002', HTTPS: '0' }, stdio: ['ignore', 'pipe', 'pipe']
      });
      let failure = null;
      child.on('error', error => { failure = error; });
      child.on('exit', code => { failure = new Error(`手势服务退出 (${code})`); child = null; startup = null; });
      child.stdout.on('data', data => process.stdout.write(data));
      child.stderr.on('data', data => process.stderr.write(data));
      const deadline = Date.now() + 12000;
      while (!await available()) {
        if (failure) throw failure;
        if (Date.now() > deadline) { child?.kill(); throw new Error('手势服务启动超时，请检查 3002 端口。'); }
        await new Promise(resolve => setTimeout(resolve, 150));
      }
    })().catch(error => { startup = null; throw error; });
    return startup;
  }
  function configure(server) {
    server.middlewares.use(async (req, res, next) => {
      if (!req.url?.startsWith('/hand-source') && !req.url?.startsWith('/api/hand-depth')) return next();
      try { await ensureSource(); next(); }
      catch (error) { res.statusCode = 503; res.end(`手势服务无法启动：${error.message}`); }
    });
    // Only a backend started by this Vite instance belongs to this lifecycle.
    server.httpServer?.once('close', () => { child?.kill(); child = startup = null; });
  }
  return { name: 'hand-follow-source', configureServer: configure, configurePreviewServer: configure };
}
