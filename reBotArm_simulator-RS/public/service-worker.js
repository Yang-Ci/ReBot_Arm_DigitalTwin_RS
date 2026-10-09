const CACHE_NAME = 'rebot-arm-rs-pwa-v108-uvc32-hand';
const APP_SHELL = [
  '/',
  '/index.html',
  '/manifest.webmanifest',
  '/favicon.png',
  '/css/rebot-sim.css?v=20260813-rs-guide43',
  '/js/pwa.js?v=20260904-rs-pages01',
  '/js/i18n.js?v=20261008-rs-camera01',
  '/js/rebot-sim.js?v=20261009-uvc-hand01',
  '/js/wrist-camera.js?v=20261009-rs-camera02',
  '/js/hand-follow-policy.js?v=20261005-hand10',
  '/js/hand-follow.js?v=20261005-hand10',
  '/js/hand-depth.js?v=20261005-hand10',
  '/js/hand-follow-worker.js?v=20261005-hand10',
  '/css/hand-follow.css?v=20261005-hand10',
  '/js/ros/rebot-ros-client.js?v=20261008-rs-grasp-fast01',
  '/js/control-mode.js?v=20260812-rs-ctrl32',
  '/js/ros/rebot-ros-ui.js?v=20261008-rs-grasp-fast01',
  '/js/ros/leader-model.js?v=20261007-leader02',
  '/js/ros/rebot-leader-ui.js?v=20261007-leader02',
  '/css/leader-teleop.css?v=20261007-leader02',
  '/models/leader-arm102/urdf/leader.urdf',
  '/models/leader-arm102/meshes/base_link.STL',
  '/models/leader-arm102/meshes/link1.STL',
  '/models/leader-arm102/meshes/link2.STL',
  '/models/leader-arm102/meshes/link3.STL',
  '/models/leader-arm102/meshes/link4.STL',
  '/models/leader-arm102/meshes/link5.STL',
  '/models/leader-arm102/meshes/link6.STL',
  '/models/leader-arm102/meshes/link7_left.STL',
  '/models/leader-arm102/meshes/link7_right.STL',
  '/js/rebot-llm.js?v=20260904-rs-pages01',
  '/lib/three-r128.min.js',
  '/lib/STLLoader-umd.js',
  '/lib/ColladaLoader.js',
  '/models/wrist-cameras/urdf/d405.urdf',
  '/models/wrist-cameras/urdf/d435i.urdf',
  '/models/wrist-cameras/urdf/gemini2.urdf',
  '/models/wrist-cameras/urdf/uvc32.urdf',
  '/models/wrist-cameras/meshes/mounts/UVC32_mount.stl',
  '/models/wrist-cameras/meshes/uvc32/board.stl',
  '/models/wrist-cameras/meshes/uvc32/lens.stl',
  '/models/wrist-cameras/meshes/uvc32/glass.stl',
  '/models/wrist-cameras/meshes/gemini2/base_link.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_botter_screw_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_color_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_color_optical_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_depth_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra1_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra1_optical_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra2_frame.STL',
  '/models/wrist-cameras/meshes/gemini2/camera_infra2_optical_frame.STL',
  '/models/wrist-cameras/meshes/mounts/D405_305_Mount.stl',
  '/models/wrist-cameras/meshes/mounts/D435_Gemini2_Mount.stl',
  '/models/wrist-cameras/meshes/realsense/d405.stl',
  '/models/wrist-cameras/meshes/realsense/d435.dae',
  '/lib/URDFLoader.js',
  '/js/motorbridge/rebot-motorbridge-client.js?v=20260812-rs-ctrl32',
  '/js/motorbridge/rebot-motorbridge-ui.js?v=20260812-rs-ctrl32'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(APP_SHELL))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(
        keys.filter((key) => key !== CACHE_NAME).map((key) => caches.delete(key))
      ))
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (event) => {
  const request = event.request;
  const url = new URL(request.url);

  if (request.method !== 'GET' || url.origin !== self.location.origin) return;

  if (url.pathname.startsWith('/api/')) {
    event.respondWith(fetch(request).catch(function () {
      return new Response('{"error":"network error"}', {
        status: 502,
        headers: { 'Content-Type': 'application/json' }
      });
    }));
    return;
  }

  const isControlAsset = request.mode === 'navigate' ||
    url.pathname.endsWith('.html') ||
    url.pathname.endsWith('.js') ||
    url.pathname.endsWith('.css');

  if (isControlAsset) {
    event.respondWith(
      fetch(request)
        .then((response) => {
          if (!response || response.status !== 200) return response;
          const copy = response.clone();
          caches.open(CACHE_NAME).then((cache) => cache.put(request, copy));
          return response;
        })
        .catch(() => caches.match(request).then((cached) => {
          if (cached) return cached;
          if (request.mode === 'navigate') return caches.match('/index.html');
          return Response.error();
        }))
    );
    return;
  }

  event.respondWith(
    caches.match(request).then((cached) => {
      if (cached) return cached;
      return fetch(request).then((response) => {
        if (!response || response.status !== 200) return response;
        const copy = response.clone();
        caches.open(CACHE_NAME).then((cache) => cache.put(request, copy));
        return response;
      });
    })
  );
});
