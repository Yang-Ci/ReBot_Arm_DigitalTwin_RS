const test = require('node:test');
const assert = require('node:assert/strict');
const { measureHand, palmGeometry } = require('../public/js/hand-follow-policy');
const { Controller, Motion, sampleDepth, inspectDepth, decodePacket } = require('../public/js/hand-depth');

function palm() {
  return [[.5,.8],[.44,.72],[.33,.67],[.22,.61],[.14,.54],
    [.36,.60],[.35,.43],[.35,.30],[.35,.19], [.50,.57],[.50,.37],[.50,.23],[.50,.10],
    [.64,.60],[.65,.40],[.66,.29],[.67,.19], [.77,.65],[.79,.50],[.81,.40],[.83,.30]].map(([x,y]) => ({x,y}));
}
function fist() {
  const p = palm();
  for (const mcp of [5,9,13,17]) p[mcp+3] = { x:p[mcp].x, y:p[mcp].y+.025 };
  return p;
}
const closed = { fistClosed:true, palmOpen:false, palmOpenHeld:false };
const opened = { fistClosed:false, palmOpen:true };
function engage(c) {
  assert.equal(c.observe(closed,.5,0).mode,'hold');
  c.observe(closed,.5,100);
  assert.equal(c.observe(closed,.5,200).entered,true);
}

test('distinguishes a four-finger fist from open palm and thumb/index pinch', () => {
  assert.equal(measureHand(fist()).fistClosed,true);
  assert.equal(measureHand(palm()).fistClosed,false);
  const pinch = palm(); pinch[4] = {...pinch[8]};
  assert.equal(measureHand(pinch).fistClosed,false);
  const oneFold = palm(); oneFold[12] = {...oneFold[9]};
  assert.equal(measureHand(oneFold).fistClosed,false);
});

test('fist must persist; depth deltas have a deadband and correct forward/back signs', () => {
  const c = new Controller(); engage(c);
  assert.equal(c.observe(closed,.501,300).offset,0);
  assert.ok(Math.abs(c.observe(closed,.55,400).offset-.048)<1e-9);
  assert.ok(c.observe(closed,.49,500).offset<0);
});

test('open palm cancels immediately and new fist takes a new baseline', () => {
  const c = new Controller(); engage(c);
  c.observe(closed,.55,300);
  assert.equal(c.observe(opened,null,400).cancelled,true);
  assert.equal(c.active,false);
  c.observe(closed,.7,500); c.observe(closed,.7,600);
  assert.equal(c.observe(closed,.7,700).offset,0);
});

test('invalid depth, jumps, stale gaps and reordered samples freeze motion while retaining the reference', () => {
  for (const [depth,t] of [[null,900],[NaN,900],[.1,900],[1.6,900],[.7,300],[.5,1000],[.5,200]]) {
    const c=new Controller();engage(c);
    assert.equal(c.observe(closed,depth,t).mode,'hold');
    assert.equal(c.active,true); assert.equal(c.paused,true); assert.equal(c.baseline,.5);
  }
  const c=new Controller();engage(c);
  assert.equal(c.observe(null,.5,900).suspended,true);
});

test('brief detection and depth gaps pause motion without moving the original baseline', () => {
  const c=new Controller();engage(c);
  c.observe(closed,.54,300);
  const missing=c.observe(null,null,400);
  assert.equal(missing.mode,'hold');assert.equal(missing.suspended,true);assert.equal(missing.offset,undefined);
  assert.equal(c.active,true);
  assert.ok(Math.abs(c.observe(closed,.55,450).offset-.048)<1e-9);
  assert.equal(c.observe(closed,0,550).suspended,true);
  assert.equal(c.observe(closed,0,1200).suspended,true);
  assert.equal(c.active,true); assert.equal(c.baseline,.5);
});

test('a short nearby hand reentry preserves the original reference and can then reverse', () => {
  const c = new Controller(); engage(c);
  c.observe(closed, .55, 300);
  assert.equal(c.observe(null, null, 520).suspended, true);
  assert.equal(c.observe(null, null, 740).suspended, true);
  const recovered = c.observe(closed, .52, 920);
  assert.equal(recovered.mode, 'depth');
  assert.equal(recovered.entered, undefined);
  assert.ok(Math.abs(recovered.offset - .018) < 1e-9);
  assert.ok(Math.abs(c.observe(closed, .47, 1020).offset + .028) < 1e-9);
  assert.equal(c.baseline, .5);
  assert.equal(c.observe(null, null, 1671).suspended, true);
  assert.equal(c.resumeRequired, true);
});

test('fist confirmation preserves movement and accepts a plausible fast stroke at a slower camera cadence', () => {
  const c=new Controller();c.observe(closed,.5,0);
  const entered=c.observe(closed,.53,150);
  assert.equal(entered.entered,true);assert.ok(Math.abs(entered.offset-.028)<1e-9);
  assert.equal(c.observe(closed,.65,350).mode,'depth');
  assert.equal(c.active,true);
  assert.equal(c.observe(closed,.85,390).status,'depth-jump');
  assert.equal(c.baseline,.5);
});

test('movement interleaved with small depth holes can reverse around the same origin', () => {
  const c = new Controller(); engage(c);
  const positions = [.52, .55, .54, .51, .48, .45, .49, .53];
  let at = 200;
  for (const depth of positions) {
    assert.equal(c.observe(closed, null, at += 50).suspended, true);
    const good = c.observe(closed, depth, at += 50);
    assert.equal(good.mode, 'depth');
    assert.equal(good.resumed, true);
    assert.equal(c.baseline, .5);
    assert.ok(Math.abs(good.offset - Math.sign(depth - .5) * (Math.abs(depth - .5) - .002)) < 1e-9);
  }
});

test('long loss confirms a new reference at the held position instead of chasing the old mapping', () => {
  const c = new Controller(); engage(c);
  c.observe(closed, .56, 300);
  assert.equal(c.missing(1800, 'missing').suspended, true);
  const ambiguous = { ...closed, fistClosed: false };
  assert.equal(c.observe(ambiguous, .46, 2000).status, 'awaiting-fist');
  assert.equal(c.observe(closed, .46, 2100).status, 'confirming-resume');
  assert.equal(c.observe(closed, .46, 2170).mode, 'hold');
  const recovered = c.observe(closed, .46, 2240);
  assert.equal(recovered.resumed, true); assert.equal(recovered.mode, 'depth');
  assert.equal(recovered.entered, undefined); assert.equal(recovered.rebased, true);
  assert.equal(recovered.offset, 0); assert.equal(c.baseline, .46);
  assert.ok(Math.abs(c.observe(closed, .5, 2340).offset - .038) < 1e-9);
});

test('reentry at a different distance immediately regains both stroke directions after confirmation', () => {
  const c = new Controller(); engage(c);
  c.observe(closed, .55, 300); c.observe(null, null, 1600);
  c.observe(closed, .8, 1800); c.observe(closed, .8, 1870);
  const resumed = c.observe(closed, .8, 1940);
  assert.equal(resumed.rebased, true); assert.equal(resumed.offset, 0);
  assert.equal(c.baseline, .8);
  assert.ok(Math.abs(c.observe(closed, .77, 2040).offset + .028) < 1e-9);
  assert.ok(Math.abs(c.observe(closed, .74, 2140).offset + .058) < 1e-9);
  assert.ok(Math.abs(c.observe(closed, .71, 2240).offset + .088) < 1e-9);
  assert.ok(c.observe(closed, .74, 2340).offset > -.088);
});

test('a displaced hand returning after a short loss also requires a paired reference', () => {
  const c = new Controller(); engage(c);
  c.observe(closed, .55, 300); c.observe(null, null, 400);
  assert.equal(c.observe(closed, .68, 500).status, 'confirming-resume');
  c.observe(closed, .68, 570);
  const result = c.observe(closed, .68, 640);
  assert.equal(result.rebased, true); assert.equal(result.offset, 0);
  assert.equal(c.baseline, .68);
});

test('short depth holes during a fast stroke do not discard movement or rebase the reference', () => {
  const c = new Controller(); engage(c);
  c.observe(closed, .54, 300); c.observe(closed, null, 400);
  const result = c.observe(closed, .62, 500);
  assert.equal(result.mode, 'depth'); assert.equal(result.rebased, undefined);
  assert.equal(c.baseline, .5); assert.ok(Math.abs(result.offset - .118) < 1e-9);
});

test('recovery interruptions restart confirmation and leave the reference untouched until confirmed', () => {
  const c = new Controller(); engage(c); c.missing(1000, 'missing');
  c.observe(closed, .6, 1100); c.observe(closed, .6, 1170);
  c.observe(closed, null, 1200);
  assert.equal(c.baseline, .5);
  assert.equal(c.observe(closed, .6, 1240).status, 'confirming-resume');
  assert.equal(c.observe(closed, .6, 1310).mode, 'hold');
  const confirmed = c.observe(closed, .6, 1380);
  assert.equal(confirmed.mode, 'depth'); assert.equal(confirmed.rebased, true);
  assert.equal(confirmed.offset, 0); assert.equal(c.baseline, .6);
});

test('near-distance protection holds until thirty centimeters, then resumes from the held pose', () => {
  const c = new Controller(); engage(c);
  assert.equal(c.observe(closed, .25, 300).status, 'too-near');
  assert.equal(c.observe(closed, .28, 400).status, 'too-near');
  assert.equal(c.observe(closed, .31, 500).status, 'confirming-resume');
  c.observe(closed, .32, 570);
  const result = c.observe(closed, .33, 640);
  assert.equal(result.resumed, true); assert.equal(result.rebased, true);
  assert.equal(result.offset, 0); assert.equal(c.baseline, .33);
  const fresh = new Controller();
  assert.equal(fresh.observe(closed, .20, 0).status, 'too-near');
  assert.equal(fresh.observe(closed, .28, 100).status, 'too-near');
  fresh.observe(closed, .31, 200);
  assert.equal(fresh.observe(closed, .31, 350).entered, true);
  assert.equal(fresh.baseline, .31);
});

test('adaptive motion damps jitter, responds to a stroke, and bounds inference-latency prediction', () => {
  const motion=new Motion();motion.update(0,0);
  for(let i=1;i<=12;i++)motion.update(i%2?.001:-.001,i*30);
  assert.ok(Math.abs(motion.value(370))<.002);
  motion.reset();motion.update(0,0);
  motion.update(.01,100);motion.update(.02,200);motion.update(.03,300);
  const predicted=motion.value(380);
  assert.ok(predicted>.033&&predicted<.044);
  assert.ok(motion.value(2000)<=motion.position+.012+1e-10);
  motion.suspend();assert.equal(motion.value(380),motion.position);
  motion.update(.035,400);assert.equal(motion.value(450),.035);
  assert.equal(motion.update(.1,350),false);
  motion.update(.12,500);assert.ok(motion.value(600)<=.12);
  motion.reset();assert.equal(motion.value(700),null);
});

test('depth offset cannot exceed twelve centimeters after slow cumulative movement', () => {
  const c=new Controller();engage(c);
  for(let i=1;i<=6;i++) c.observe(closed,.5+i*.04,200+i*100);
  assert.equal(c.observe(closed,.75,900).offset,.12);
  c.reset();engage(c);
  for(let i=1;i<=6;i++) c.observe(closed,.5-i*.04,200+i*100);
  assert.equal(c.observe(closed,.27,900).offset,-.12);
});

test('depth quality distinguishes range, tolerates scattered holes, and rejects a missing center or background', () => {
  const points = palm();
  const frame = { width: 640, height: 480, depth: new Uint16Array(640 * 480).fill(510) };
  for (let i = 0; i < frame.depth.length; i++) if (i % 3 === 0) frame.depth[i] = 0;
  assert.equal(inspectDepth(frame, points).status, 'valid');
  assert.equal(sampleDepth(frame, points), .510);
  const { depthCenter } = palmGeometry(points);
  const cx = Math.round(depthCenter.x * 639), cy = Math.round(depthCenter.y * 479);
  const fillCenter = (mm) => {
    for (let y = cy - 7; y <= cy + 7; y++) for (let x = cx - 7; x <= cx + 7; x++) frame.depth[y * 640 + x] = mm;
  };
  frame.depth.fill(1000); fillCenter(0);
  assert.equal(inspectDepth(frame, points).status, 'insufficient');
  assert.equal(sampleDepth(frame, points), null);
  fillCenter(510);
  assert.equal(inspectDepth(frame, points).status, 'inconsistent');
  frame.depth.fill(0);
  // A handful of valid pixels cannot substitute for the missing palm.
  for (let i = 0; i < 20; i++) frame.depth[(cy - 6 + Math.floor(i / 13)) * 640 + cx - 6 + i % 13] = 510;
  assert.equal(inspectDepth(frame, points).status, 'insufficient');
  frame.depth.fill(200);
  assert.equal(inspectDepth(frame, points).status, 'too-near');
  assert.equal(inspectDepth(frame, points).distanceMeters, .2);
  assert.equal(sampleDepth(frame, points), null);
  frame.depth.fill(2500);
  assert.equal(inspectDepth(frame, points).status, 'too-far');
});

test('uses consistent palm depth and rejects empty or background-conflicted data', () => {
  const frame={width:640,height:480,depth:new Uint16Array(640*480).fill(510)};
  assert.equal(sampleDepth(frame,palm()),.510);
  frame.depth.fill(0); assert.equal(sampleDepth(frame,palm()),null);
  frame.depth.fill(2500); assert.equal(sampleDepth(frame,palm()),null);
  for(let i=0;i<frame.depth.length;i++)frame.depth[i]=i%2?350:800;
  assert.equal(sampleDepth(frame,palm()),null);
  assert.equal(sampleDepth({...frame,depth:frame.depth.slice(1)},palm()),null);
});

test('visible root regions provide depth when the wrist or one root is obscured', () => {
  const frame = { width: 640, height: 480, depth: new Uint16Array(640 * 480) };
  // Only the palm interior has depth; the wrist and background are invalid.
  for (let y = 292; y <= 365; y++) for (let x = 224; x <= 500; x++) frame.depth[y * 640 + x] = 510;
  const partial = fist(); partial[0] = null;
  assert.equal(measureHand(partial).fistClosed, true);
  assert.equal(sampleDepth(frame, partial), .510);
  partial[0] = { x: .5, y: 1.01 };
  assert.equal(sampleDepth(frame, partial), .510);
  partial[9] = null;
  assert.equal(measureHand(partial).fistClosed, true);
  assert.equal(sampleDepth(frame, partial), .510);
  const c = new Controller();
  c.observe(measureHand(partial), .510, 0);
  assert.equal(c.observe(measureHand(partial), .525, 150).entered, true);
  assert.ok(c.observe(measureHand(partial), .54, 250).offset > .02);
  const openedPartial = palm(); openedPartial[0] = null;
  assert.equal(c.observe(measureHand(openedPartial), .54, 300).cancelled, true);
});

test('insufficient visible fingers and long hand loss freeze motion with the reference retained', () => {
  const c = new Controller(); engage(c);
  const partial = fist(); partial[8] = null; partial[12] = null;
  const held = c.observe(measureHand(partial), .55, 250);
  assert.equal(held.suspended, true); assert.equal(held.offset, undefined);
  assert.equal(c.observe(null, null, 900).suspended, true);
  assert.equal(c.active, true); assert.equal(c.baseline, .5);
});

test('binary RGB-D frame validates lengths and decodes little-endian millimeters', () => {
  const header=new TextEncoder().encode(JSON.stringify({width:2,height:1,jpegBytes:3,seq:1,capturedUnixMs:1234}));
  const bytes=new Uint8Array(8+header.length+3+4);
  bytes.set(new TextEncoder().encode('RBD1'));new DataView(bytes.buffer).setUint32(4,header.length,true);
  bytes.set(header,8);bytes.set([255,216,255],8+header.length);
  new DataView(bytes.buffer).setUint16(bytes.length-4,420,true);
  new DataView(bytes.buffer).setUint16(bytes.length-2,701,true);
  const frame=decodePacket(bytes.buffer);
  assert.deepEqual([...frame.depth],[420,701]);assert.equal(frame.jpeg.size,3);
  assert.throws(()=>decodePacket(bytes.buffer.slice(0,-1)));
  bytes[0]=0;assert.throws(()=>decodePacket(bytes.buffer));
});
