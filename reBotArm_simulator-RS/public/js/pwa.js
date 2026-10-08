(function () {
  if (new URLSearchParams(location.search).get('embed') === 'mujoco') return;
  const installButton = document.getElementById('pwa-install');
  const installStatus = document.getElementById('pwa-install-status');
  let deferredPrompt = null;

  function setInstallStatus(text) {
    if (installStatus) installStatus.textContent = text;
  }

  function isStandalone() {
    return window.matchMedia('(display-mode: standalone)').matches || window.navigator.standalone;
  }

  if (!window.isSecureContext) {
    setInstallStatus('手机安装需要 HTTPS；localhost 可安装，局域网 HTTP 只能浏览');
  } else if ('serviceWorker' in navigator) {
    window.addEventListener('load', () => {
      navigator.serviceWorker.register('/service-worker.js', { updateViaCache: 'none' })
        .then((registration) => registration.update().then(() => registration))
        .then(() => setInstallStatus(isStandalone() ? '已作为 App 打开' : '可从浏览器菜单添加到桌面'))
        .catch(() => setInstallStatus('Service Worker 注册失败，暂时只能浏览'));
    });
  } else {
    setInstallStatus('当前浏览器未开放 PWA 所需的 Service Worker');
  }

  window.addEventListener('beforeinstallprompt', (event) => {
    event.preventDefault();
    deferredPrompt = event;
    if (installButton) installButton.hidden = false;
    setInstallStatus('可以安装到桌面');
  });

  window.addEventListener('appinstalled', () => {
    deferredPrompt = null;
    if (installButton) installButton.hidden = true;
    setInstallStatus('已安装到桌面');
  });

  if (installButton) {
    installButton.addEventListener('click', async () => {
      if (!deferredPrompt) {
        setInstallStatus('请使用浏览器菜单添加到桌面');
        return;
      }

      deferredPrompt.prompt();
      const choice = await deferredPrompt.userChoice;
      deferredPrompt = null;
      installButton.hidden = true;
      setInstallStatus(choice.outcome === 'accepted' ? '正在安装' : '已取消安装');
    });
  }
})();
