(function (root) {
  const WAKE_HOLD_MS = 1000;
  const IDLE_MS = 15000;
  const SAMPLE_MAX_AGE_MS = 300;
  const WAKE_MAX_GAP_MS = 650;
  const WAKE_MAX_MISSES = 3;
  const GRIPPER_MAX = .0715;
  const PINCH_CLOSE_RATIO = .30;
  const PINCH_RELEASE_RATIO = .38;
  const PINCH_OPEN_RATIO = .80;
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  const MCP_INDICES = [5, 9, 13, 17];
  const visiblePoint = (point) => Boolean(point && Number.isFinite(point.x) && Number.isFinite(point.y)
    && point.x >= 0 && point.x <= 1 && point.y >= 0 && point.y <= 1);

  function palmGeometry(landmarks, aspect = 4 / 3) {
    if (!Array.isArray(landmarks) || landmarks.length !== 21 || !Number.isFinite(aspect) || aspect <= 0) return null;
    const indices = MCP_INDICES.filter(index => visiblePoint(landmarks[index]));
    if (indices.length < 3) return null;
    const points = landmarks.map(point => visiblePoint(point) ? { x: point.x * aspect, y: point.y } : null);
    // Infer the width from several finger-root pairs. The wrist can disappear
    // at the image edge without invalidating a well-observed palm.
    const widths = [];
    for (let a = 0; a < indices.length; a++) for (let b = a + 1; b < indices.length; b++) {
      widths.push(distance(points[indices[a]], points[indices[b]]) * 12 / (indices[b] - indices[a]));
    }
    widths.sort((a, b) => a - b);
    const width = widths[Math.floor(widths.length / 2)];
    const size = width * .45;
    if (size < .035 || size > .65 || widths.some(value => value < width * .35 || value > width * 2.5)) return null;
    const center = indices.reduce((sum, index) => ({ x: sum.x + landmarks[index].x / indices.length,
      y: sum.y + landmarks[index].y / indices.length }), { x: 0, y: 0 });
    // The middle root stays fixed when the wrist appears/disappears. If that
    // root is missing, the caller reanchors before using the remaining roots.
    const anchor = visiblePoint(landmarks[9]) ? 'middle-root' : `roots-${indices.join('-')}`;
    const palm = anchor === 'middle-root' ? { x: landmarks[9].x, y: landmarks[9].y } : center;
    const axes = indices.filter(index => points[index + 1]).map(index => {
      const dx = points[index + 1].x - points[index].x, dy = points[index + 1].y - points[index].y;
      const length = Math.hypot(dx, dy);
      return length > .01 ? { x: dx / length, y: dy / length } : null;
    }).filter(Boolean);
    const axis = axes.reduce((sum, value) => ({ x: sum.x + value.x / axes.length, y: sum.y + value.y / axes.length }), { x: 0, y: 0 });
    // Move slightly inward from the roots, independent of wrist visibility.
    const depthCenter = { x: center.x - axis.x * width * .18 / aspect, y: center.y - axis.y * width * .18 };
    const patches = [depthCenter, ...indices.map(index => ({ x: depthCenter.x * .55 + landmarks[index].x * .45,
      y: depthCenter.y * .55 + landmarks[index].y * .45 }))].filter(visiblePoint);
    return { points, indices, size, width, palm, anchor, patches, depthCenter };
  }

  function pinchWidthToGrip(ratio) {
    if (!Number.isFinite(ratio) || ratio < 0) return null;
    // Landmark centers can remain separated even when the fingertips touch.
    // Use a close region scaled by palm width, independent of the wrist.
    return clamp((ratio - PINCH_CLOSE_RATIO) / (PINCH_OPEN_RATIO - PINCH_CLOSE_RATIO), 0, 1) * GRIPPER_MAX;
  }

  class PinchGripper {
    constructor() { this.reset(); }
    reset() { this.closed = false; }
    observe(ratio) {
      const width = pinchWidthToGrip(ratio);
      if (width === null) return null;
      if (ratio <= PINCH_CLOSE_RATIO) this.closed = true;
      else if (ratio >= PINCH_RELEASE_RATIO) this.closed = false;
      return this.closed ? 0 : width;
    }
  }

  function angle(a, b, c) {
    const ux = a.x - b.x, uy = a.y - b.y;
    const vx = c.x - b.x, vy = c.y - b.y;
    const length = Math.hypot(ux, uy) * Math.hypot(vx, vy);
    return length > 1e-8 ? Math.acos(clamp((ux * vx + uy * vy) / length, -1, 1)) * 180 / Math.PI : 0;
  }

  function measureHand(landmarks, aspect = 4 / 3) {
    const geometry = palmGeometry(landmarks, aspect);
    if (!geometry) return null;
    const { points, width, palm, anchor } = geometry;
    const fingers = MCP_INDICES.filter(mcp => points[mcp] && points[mcp + 1] && points[mcp + 3]
      && distance(points[mcp], points[mcp + 1]) > .01);
    const extended = (mcp, minAngle, minReach) => angle(points[mcp], points[mcp + 1], points[mcp + 3]) > minAngle
      && distance(points[mcp + 3], points[mcp]) > distance(points[mcp + 1], points[mcp]) * minReach;
    const pinchWidthRatio = points[4] && points[8] ? distance(points[4], points[8]) / width : null;
    const gestureReliable = fingers.length >= 3;
    // Missing fingers are unknown, never fabricated. Every observed finger
    // must agree; a known curled finger still vetoes an open-palm wake.
    const separated = pinchWidthRatio !== null ? pinchWidthRatio > PINCH_RELEASE_RATIO : fingers.length === 4;
    const palmOpen = gestureReliable && separated && fingers.every(mcp => extended(mcp, 135, 1.25));
    const palmOpenHeld = gestureReliable && separated && fingers.every(mcp => extended(mcp, 120, 1.15));
    const fistClosed = gestureReliable && fingers.every(mcp => distance(points[mcp + 3], points[mcp])
      < distance(points[mcp + 1], points[mcp]) * 1.15 && angle(points[mcp], points[mcp + 1], points[mcp + 3]) < 125);
    const partial = Array.from(landmarks).some(point => !visiblePoint(point));
    return { x: 1 - palm.x, y: palm.y, grip: pinchWidthToGrip(pinchWidthRatio), pinchWidthRatio,
      palmOpen, palmOpenHeld, fistClosed, gestureReliable, partial, visibleFingers: fingers.length, palm, anchor };
  }

  class Session {
    constructor() {
      this.state = 'standby';
      this.wakeStarted = this.lastWakeSample = this.lastInteraction = null;
      this.lastWakeCaptured = null;
      this.wakeHeldMs = this.wakeSamples = this.wakeMisses = 0;
      this.wakePaused = false;
      this.lastSampleAt = -Infinity;
      this.wakeArmed = true;
    }
    clearWake() {
      this.wakeStarted = this.lastWakeSample = this.lastWakeCaptured = null;
      this.wakeHeldMs = this.wakeSamples = this.wakeMisses = 0;
      this.wakePaused = false;
    }
    observe(sample, now, capturedAt = now) {
      if (!Number.isFinite(now) || !Number.isFinite(capturedAt)) return null;
      if (this.tick(now) === 'home') return 'home';
      if (capturedAt > now || now - capturedAt > SAMPLE_MAX_AGE_MS || capturedAt <= this.lastSampleAt) return null;
      this.lastSampleAt = capturedAt;
      if (!sample || (!sample.palmOpen && !sample.palmOpenHeld)) this.wakeArmed = true;
      if (this.state === 'active') {
        if (sample) this.lastInteraction = capturedAt;
        return null;
      }
      if (this.state !== 'standby' || !this.wakeArmed) return null;
      const open = sample?.palmOpen || (this.wakeStarted !== null && sample?.palmOpenHeld);
      if (!open) {
        if (this.wakeStarted !== null) {
          this.wakePaused = true;
          if (++this.wakeMisses >= WAKE_MAX_MISSES) this.clearWake();
        }
        return null;
      }
      if (this.lastWakeCaptured !== null && capturedAt - this.lastWakeCaptured > WAKE_MAX_GAP_MS) this.clearWake();
      if (this.wakeStarted === null) this.wakeStarted = capturedAt;
      else if (!this.wakePaused) this.wakeHeldMs += capturedAt - this.lastWakeCaptured;
      // Expiry uses arrival time: valid inference can take up to 300 ms.
      // Only confirmed open-palm capture intervals contribute to the hold.
      this.lastWakeSample = now;
      this.lastWakeCaptured = capturedAt;
      this.wakePaused = false;
      this.wakeMisses = 0;
      this.wakeSamples++;
      if (this.wakeHeldMs >= WAKE_HOLD_MS && this.wakeSamples >= 3) {
        this.state = 'waking';
        this.clearWake();
        return 'wake';
      }
      return null;
    }
    wakeMouse() {
      if (this.state !== 'standby') return false;
      this.state = 'waking';
      this.clearWake();
      return true;
    }
    middleReached(now) {
      if (this.state !== 'waking') return;
      this.state = 'active';
      this.lastInteraction = now;
    }
    requestHome() {
      if (this.state === 'standby' || this.state === 'returning') return false;
      this.state = 'returning';
      this.wakeArmed = false;
      this.lastInteraction = null;
      this.clearWake();
      return true;
    }
    homeReached() { if (this.state === 'returning') this.state = 'standby'; }
    tick(now) {
      if (this.lastWakeSample !== null && now - this.lastWakeSample > WAKE_MAX_GAP_MS) this.clearWake();
      if (this.state === 'active' && now - this.lastInteraction >= IDLE_MS && this.requestHome()) return 'home';
      return null;
    }
    wakeProgress() { return clamp(this.wakeHeldMs / WAKE_HOLD_MS, 0, 1); }
    remainingMs(now) { return this.state === 'active' ? Math.max(0, IDLE_MS - (now - this.lastInteraction)) : IDLE_MS; }
  }
  const policy = { Session, PinchGripper, measureHand, palmGeometry, visiblePoint, WAKE_HOLD_MS, IDLE_MS, SAMPLE_MAX_AGE_MS };
  if (typeof module !== 'undefined' && module.exports) module.exports = policy;
  else root.RebotHandPolicy = policy;
})(typeof window !== 'undefined' ? window : globalThis);
