(function (root) {
  const { palmGeometry, visiblePoint } = typeof module !== 'undefined' && module.exports ? require('./hand-follow-policy') : root.RebotHandPolicy;
  const TRACKING_HOLD_MS = 650;
  const MIN_DISTANCE_M = .26;
  const NEAR_RELEASE_M = .30;
  const MAX_DISTANCE_M = 1.5;
  const littleEndian = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;
  function decodePacket(buffer) {
    const bytes = new Uint8Array(buffer);
    if (bytes.length < 8 || String.fromCharCode(...bytes.subarray(0, 4)) !== 'RBD1') throw new Error('336 深度帧格式错误。');
    const metaLength = new DataView(buffer).getUint32(4, true);
    if (metaLength > 4096 || metaLength + 8 > bytes.length) throw new Error('336 深度帧头无效。');
    const meta = JSON.parse(new TextDecoder().decode(bytes.subarray(8, 8 + metaLength)));
    const { width, height, jpegBytes, seq, capturedUnixMs } = meta;
    if (![width, height, jpegBytes, seq].every(Number.isSafeInteger) || width < 1 || height < 1 || width > 1280 || height > 960
      || jpegBytes < 1 || seq < 1 || !Number.isFinite(capturedUnixMs)
      || bytes.length !== 8 + metaLength + jpegBytes + width * height * 2) throw new Error('336 RGB-D 帧尺寸无效。');
    const imageStart = 8 + metaLength;
    const depthBytes = bytes.slice(imageStart + jpegBytes);
    const depth = new Uint16Array(depthBytes.buffer);
    if (!littleEndian) {
      const view = new DataView(depthBytes.buffer);
      for (let i = 0; i < depth.length; i++) depth[i] = view.getUint16(i * 2, true);
    }
    return { ...meta, jpeg: new Blob([bytes.subarray(imageStart, imageStart + jpegBytes)], { type: 'image/jpeg' }), depth };
  }

  const median = (values) => { values.sort((a, b) => a - b); return values[Math.floor(values.length / 2)]; };
  const depthReading = (status, distanceMeters = null) => ({ status, distanceMeters, usable: status === 'valid', reason: {
    unavailable: '暂未取得同帧深度，前后跟随暂停。',
    incomplete: '手部未完整入画，前后跟随暂停。',
    insufficient: '掌内深度不足，请把手放在相机前 30–80 cm。',
    inconsistent: '掌内深度不一致，前后跟随暂停。',
    'too-near': '距离过近，已暂停；请退到 30 cm 以上。',
    'too-far': '距离过远，已暂停；请回到相机前 30–80 cm。'
  }[status] || null });
  function inspectDepth(frame, landmarks) {
    if (!frame) return depthReading('unavailable');
    if (!Array.isArray(landmarks) || landmarks.length !== 21) return depthReading('incomplete');
    const { width, height, depth } = frame;
    if (!Number.isSafeInteger(width) || !Number.isSafeInteger(height) || depth?.length !== width * height) return depthReading('unavailable');
    const geometry = palmGeometry(landmarks, width / height);
    if (!geometry || !visiblePoint(geometry.depthCenter)) return depthReading('incomplete');
    // Sample inside the palm rather than along silhouette edges, which often
    // contain background pixels. Require several consistent, populated patches.
    const patches = geometry.patches;
    const radius = Math.max(3, Math.min(6, Math.round(geometry.width / (width / height) * width * .04)));
    const values = [];
    let centerMm = null;
    let centerCoverage = 0;
    for (const p of patches) {
      const cx = Math.round(p.x * (width - 1)), cy = Math.round(p.y * (height - 1));
      const patch = [];
      for (let y = Math.max(0, cy - radius); y <= Math.min(height - 1, cy + radius); y++) {
        for (let x = Math.max(0, cx - radius); x <= Math.min(width - 1, cx + radius); x++) {
          const mm = depth[y * width + x];
          // Read near/far values for the warning; only the operating range
          // will be allowed to produce motion.
          if (mm >= 80 && mm <= 5000) patch.push(mm);
        }
      }
      const area = (Math.min(height - 1, cy + radius) - Math.max(0, cy - radius) + 1)
        * (Math.min(width - 1, cx + radius) - Math.max(0, cx - radius) + 1);
      if (p === patches[0]) centerCoverage = patch.length / area;
      // Small stereo holes are common during movement. Retain a populated
      // neighborhood, while still requiring a strong depth consensus.
      if (patch.length >= 20 && patch.length >= area * .6) {
        const mid = median(patch);
        if (patch.filter(v => Math.abs(v - mid) <= 35).length >= patch.length * .8) {
          values.push(mid);
          if (p === patches[0]) centerMm = mid;
        }
      }
    }
    // A valid background majority cannot replace missing palm-center depth.
    if (centerMm === null) return { ...depthReading('insufficient'), centerCoverage };
    if (centerMm < MIN_DISTANCE_M * 1000) return { ...depthReading('too-near', centerMm / 1000), centerCoverage };
    const consistent = values.filter(v => Math.abs(v - centerMm) <= 45);
    if (consistent.length < 3) return { ...depthReading('inconsistent'), centerCoverage };
    const meters = median(consistent) / 1000;
    const status = meters < MIN_DISTANCE_M ? 'too-near' : meters > MAX_DISTANCE_M ? 'too-far' : 'valid';
    return { ...depthReading(status, meters), centerCoverage };
  }
  function sampleDepth(frame, landmarks) { const reading = inspectDepth(frame, landmarks); return reading.usable ? reading.distanceMeters : null; }

  class Controller {
    constructor() { this.reset(); }
    reset() {
      this.active = this.paused = this.resumeRequired = this.nearBlocked = this.reacquiringHand = false;
      this.candidateAt = this.baseline = this.lastDepth = this.lastAt = null;
      this.lastObservedAt = -Infinity;
      this.clearResume();
    }
    clearResume() { this.resumeAt = this.resumeLastAt = this.resumeDepth = null; this.resumeSamples = 0; }
    hold(reason, status = 'missing', requireResume = false) {
      this.clearResume();
      if (this.active) {
        this.paused = true;
        this.resumeRequired ||= requireResume;
        this.reacquiringHand ||= ['hand-missing', 'frame-gap', 'stale-frame'].includes(status);
        return { mode: 'hold', suspended: true, status, reason, baselineMeters: this.baseline };
      }
      this.candidateAt = this.baseline = this.lastDepth = this.lastAt = null;
      return { mode: 'hold', reset: true, status, reason };
    }
    observe(hand, depth, capturedAt, quality = null) {
      if (!Number.isFinite(capturedAt) || capturedAt <= this.lastObservedAt) return this.hold('识别帧顺序无效，前后跟随暂停。', 'invalid-frame', true);
      this.lastObservedAt = capturedAt;
      if (!hand) return this.missing(capturedAt, '手部暂时丢失，已保持机械臂位置。', 'hand-missing');
      if (hand.palmOpen || hand.palmOpenHeld) {
        const cancelled = this.active || this.candidateAt !== null;
        this.reset();
        return { mode: 'planar', cancelled };
      }
      if (hand.gestureReliable === false) return this.missing(capturedAt, '可见手指不足，已保持机械臂位置。');
      if (!this.active && !hand.fistClosed) {
        this.candidateAt = this.baseline = this.lastAt = this.lastDepth = null;
        return { mode: 'planar' };
      }
      if (Number.isFinite(depth) && (depth < MIN_DISTANCE_M || (this.nearBlocked && depth < NEAR_RELEASE_M))) {
        this.nearBlocked = true;
        return this.hold('距离过近，已暂停；请退到 30 cm 以上，握拳保持后恢复。', 'too-near', true);
      }
      if (!Number.isFinite(depth) || depth > MAX_DISTANCE_M) return this.missing(capturedAt,
        quality?.reason || '掌内深度无效，已保持机械臂位置。', quality?.status || 'invalid-depth');
      this.nearBlocked = false;
      const elapsed = this.lastAt === null ? 0 : (capturedAt - this.lastAt) / 1000;
      const maxChange = Math.min(.20, .018 + .8 * Math.max(0, elapsed));
      if (this.active && this.lastAt !== null && capturedAt - this.lastAt > TRACKING_HOLD_MS) {
        this.paused = this.resumeRequired = true;
      }
      // A redetected hand can enter at a completely different distance. Do
      // not chase the old absolute target or sit saturated at its stroke cap.
      if (this.active && this.paused && this.reacquiringHand && Math.abs(depth - this.lastDepth) > .04) this.resumeRequired = true;
      if (this.lastAt !== null && !this.resumeRequired && (Math.abs(depth - this.lastDepth) > maxChange
        || (!this.active && capturedAt - this.lastAt > 400))) {
        return this.hold('距离变化异常，已暂停；保持握拳可确认接续跟随。', 'depth-jump', true);
      }
      let resumed = false, rebased = false;
      if (this.active && this.paused) {
        if (this.resumeRequired) {
          if (!hand.fistClosed) return this.hold('机械臂位置已保持，请握拳保持以确认恢复。', 'awaiting-fist', true);
          const dt = this.resumeLastAt === null ? 0 : capturedAt - this.resumeLastAt;
          if (this.resumeLastAt !== null && (dt > 400 || Math.abs(depth - this.resumeDepth) > Math.min(.2, .018 + .8 * dt / 1000))) this.clearResume();
          if (this.resumeAt === null) this.resumeAt = capturedAt;
          this.resumeLastAt = capturedAt; this.resumeDepth = depth; this.resumeSamples++;
          if (capturedAt - this.resumeAt < 120 || this.resumeSamples < 3) return {
            mode: 'hold', suspended: true, status: 'confirming-resume', baselineMeters: this.baseline,
            reason: '机械臂位置已保持，保持握拳，正在确认恢复…'
          };
        }
        if (this.resumeRequired) {
          // The UI must pair this distance reference with the held robot X
          // reference in the same callback, then restart the motion filter.
          this.baseline = depth;
          rebased = true;
        }
        this.paused = this.resumeRequired = false;
        this.reacquiringHand = false;
        this.clearResume(); resumed = true;
      }
      this.lastAt = capturedAt; this.lastDepth = depth;
      if (!this.active) {
        if (this.candidateAt === null) { this.candidateAt = capturedAt; this.baseline = depth; }
        if (capturedAt - this.candidateAt < 120) return { mode: 'hold', reason: '保持握拳，正在定位距离…' };
        this.active = true;
        return { ...this.depthTarget(depth), entered: true };
      }
      return { ...this.depthTarget(depth), ...(resumed ? { resumed: true } : {}), ...(rebased ? { rebased: true } : {}) };
    }
    depthTarget(depth) {
      const delta = depth - this.baseline;
      const offset = Math.sign(delta) * Math.max(0, Math.abs(delta) - .002);
      return { mode: 'depth', offset: Math.max(-.12, Math.min(.12, offset)), clamped: Math.abs(offset) > .12, baselineMeters: this.baseline };
    }
    missing(capturedAt, reason, status = 'missing') {
      return this.hold(reason, status, this.lastAt !== null && capturedAt - this.lastAt > TRACKING_HOLD_MS);
    }
  }

  class Motion {
    constructor() { this.reset(); }
    reset() { this.position = this.previous = null; this.velocity = 0; this.capturedAt = null; }
    suspend() { this.velocity = 0; this.previous = null; }
    update(offset, capturedAt) {
      if (!Number.isFinite(offset) || !Number.isFinite(capturedAt)) return false;
      if (this.capturedAt !== null && capturedAt <= this.capturedAt) return false;
      if (this.position === null || this.previous === null) {
        this.position = offset; this.velocity = 0;
      } else {
        const dt = Math.min(.4, (capturedAt - this.capturedAt) / 1000);
        const speed = Math.max(-.8, Math.min(.8, (offset - this.previous) / dt));
        this.velocity += (speed - this.velocity) * (1 - Math.exp(-dt / .045));
        // Stronger damping for tiny movements; a fast hand gets a shorter
        // filter time constant instead of stacking another animation filter.
        const tau = Math.max(.018, .065 / (1 + 14 * Math.abs(this.velocity)));
        this.position += (offset - this.position) * (1 - Math.exp(-dt / tau));
      }
      this.previous = offset; this.capturedAt = capturedAt;
      return true;
    }
    value(now) {
      if (this.position === null || !Number.isFinite(now)) return null;
      const age = Math.max(0, Math.min(.13, (now - this.capturedAt) / 1000));
      const lead = Math.max(-.012, Math.min(.012, this.velocity * age));
      return Math.max(-.12, Math.min(.12, this.position + lead));
    }
  }
  const api = { decodePacket, sampleDepth, inspectDepth, Controller, Motion, TRACKING_HOLD_MS, MIN_DISTANCE_M, NEAR_RELEASE_M, MAX_DISTANCE_M };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.RebotHandDepth = api;
})(typeof window !== 'undefined' ? window : globalThis);
