import { FilesetResolver, HandLandmarker } from '../lib/mediapipe/vision_bundle.mjs';

let detector;
self.onmessage = async ({ data }) => {
  if (data.type === 'init') {
    try {
      const files = await FilesetResolver.forVisionTasks(data.wasmUrl, true);
      detector = await HandLandmarker.createFromOptions(files, {
        baseOptions: { modelAssetPath: data.modelUrl, delegate: 'CPU' },
        runningMode: 'VIDEO',
        numHands: 1,
        minHandDetectionConfidence: 0.55,
        minHandPresenceConfidence: 0.45,
        minTrackingConfidence: 0.5
      });
      self.postMessage({ type: 'ready' });
    } catch (error) {
      self.postMessage({ type: 'error', message: error.message });
    }
  } else if (data.type === 'frame') {
    try {
      const result = detector.detectForVideo(data.bitmap, data.timestamp);
      self.postMessage({ type: 'result', landmarks: result.landmarks[0] || null, timestamp: data.timestamp });
    } catch (error) {
      self.postMessage({ type: 'error', message: error.message });
    } finally {
      data.bitmap.close();
    }
  }
};
