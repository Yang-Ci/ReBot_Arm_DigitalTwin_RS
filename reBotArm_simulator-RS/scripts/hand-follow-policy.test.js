const test = require('node:test');
const assert = require('node:assert/strict');
const { Session, PinchGripper, measureHand } = require('../public/js/hand-follow-policy.js');

function openPalm() {
  return [[.5,.8],[.44,.72],[.33,.67],[.22,.61],[.14,.54],
    [.36,.60],[.35,.43],[.35,.30],[.35,.19],
    [.50,.57],[.50,.37],[.50,.23],[.50,.10],
    [.64,.60],[.65,.40],[.66,.29],[.67,.19],
    [.77,.65],[.79,.50],[.81,.40],[.83,.30]].map(([x,y]) => ({ x,y }));
}
const palm = { x: .5, y: .5, grip: .07, palmOpen: true };
const pinch = { ...palm, palmOpen: false };

test('recognizes an open palm, but not a folded finger or a pinch', () => {
  assert.equal(measureHand(openPalm()).palmOpen, true);
  const folded = openPalm();
  folded[12] = { x: .50, y: .65 };
  assert.equal(measureHand(folded).palmOpen, false);
  const pinched = openPalm();
  pinched[4] = { ...pinched[8] };
  assert.equal(measureHand(pinched).palmOpen, false);
  assert.equal(measureHand(pinched).grip, 0);
});

test('a natural pinch with separated landmark centers reaches zero at different hand sizes', () => {
  const pinched = openPalm(); pinched[4] = { x:.43, y:.19 };
  for (const scale of [.55, .8, 1]) {
    const scaled = pinched.map(p => ({ x:.5 + (p.x - .5) * scale, y:.5 + (p.y - .5) * scale }));
    assert.notDeepEqual(scaled[4], scaled[8]);
    const sample = measureHand(scaled);
    assert.equal(sample.grip, 0); assert.equal(sample.fistClosed, false); assert.equal(sample.palmOpen, false);
    scaled[0] = null;
    assert.equal(measureHand(scaled).grip, 0);
  }
});

test('a near-touch pinch cannot wake or count as an open palm', () => {
  const pinched = openPalm(); pinched[4] = { x:.47, y:.19 };
  const sample = measureHand(pinched);
  assert.equal(sample.grip, 0);
  assert.equal(sample.palmOpen, false); assert.equal(sample.palmOpenHeld, false);
  const session = new Session();
  for (let t = 0; t <= 1200; t += 50) session.observe(sample, t);
  assert.equal(session.state, 'standby');
});

test('closed pinch stays at zero through jitter and unknown tips, and releases with a wider separation', () => {
  const gripper = new PinchGripper();
  assert.equal(gripper.observe(.28), 0);
  for (const ratio of [.31, .36, .29, .37]) assert.equal(gripper.observe(ratio), 0);
  assert.equal(gripper.observe(null), null);
  assert.equal(gripper.observe(NaN), null);
  assert.equal(gripper.observe(.35), 0);
  assert.ok(gripper.observe(.45) > 0);
  assert.ok(Math.abs(gripper.observe(.55) - .03575) < 1e-9);
  assert.equal(gripper.observe(.9), .0715);
  assert.equal(gripper.observe(-1), null);
  gripper.observe(.28); gripper.reset();
  assert.ok(gripper.observe(.35) > 0);
});

test('accepts a naturally bent thumb on an otherwise open palm', () => {
  const natural = openPalm();
  natural[4] = { x: .30, y: .48 };
  assert.equal(measureHand(natural).palmOpen, true);
});

test('slightly curved fingers can maintain a hold; a folded finger and pinch cannot', () => {
  const curved = openPalm(); curved[8] = { x:.44,y:.33 };
  const measured = measureHand(curved);
  assert.equal(measured.palmOpen, false);
  assert.equal(measured.palmOpenHeld, true);
  const folded = openPalm(); folded[12] = { x:.50,y:.65 };
  assert.equal(measureHand(folded).palmOpenHeld,false);
  const pinched = openPalm(); pinched[4] = { ...pinched[8] };
  assert.equal(measureHand(pinched).palmOpenHeld,false);
});

test('rejects shifted indices, insufficient roots, tiny and fully off-frame hand data', () => {
  assert.equal(measureHand(openPalm().slice(1)), null);
  const insufficient = openPalm(); insufficient[5] = null; insufficient[9] = { x: NaN, y: .5 };
  assert.equal(measureHand(insufficient), null);
  const outside = openPalm().map(p => ({ x: p.x + 1, y: p.y }));
  assert.equal(measureHand(outside), null);
  const tiny = openPalm().map(p => ({ x: .5 + p.x * .01, y: .5 + p.y * .01 }));
  assert.equal(measureHand(tiny), null);
});

test('a missing or off-frame wrist does not interrupt open-palm wake or shift its position', () => {
  const full = measureHand(openPalm());
  const session = new Session();
  for (let t = 0; t <= 1000; t += 50) {
    const partial = openPalm(); partial[0] = t % 100 ? null : { x: .5, y: 1.01 };
    const sample = measureHand(partial);
    assert.equal(sample.palmOpen, true); assert.equal(sample.partial, true);
    assert.equal(sample.x, full.x); assert.equal(sample.y, full.y); assert.equal(sample.anchor, full.anchor);
    assert.equal(session.observe(sample, t), t === 1000 ? 'wake' : null);
  }
});

test('tolerates one unknown finger but a known folded finger still vetoes an open palm', () => {
  const partial = openPalm(); partial[20] = null;
  assert.equal(measureHand(partial).palmOpen, true);
  partial[12] = { x: .5, y: .65 };
  assert.equal(measureHand(partial).palmOpen, false);
  assert.equal(measureHand(partial).palmOpenHeld, false);
  partial[8] = { x: NaN, y: .19 };
  assert.equal(measureHand(partial).gestureReliable, false);
  assert.equal(measureHand(partial).fistClosed, false);
});

test('missing pinch tips leave the gripper unknown while visible roots still locate the hand', () => {
  for (const index of [4, 8]) {
    const partial = openPalm(); partial[index] = null;
    const sample = measureHand(partial);
    assert.ok(sample); assert.equal(sample.grip, null);
  }
  const partial = openPalm(); partial[9] = null;
  assert.equal(measureHand(partial).anchor, 'roots-5-13-17');
  assert.equal(measureHand(partial).palmOpen, true);
});

test('root-based gestures work for a rotated palm', () => {
  const rotated = openPalm().map(p => ({ x: .5 + (.5 - p.y) / (4 / 3), y: .5 + (p.x - .5) * (4 / 3) }));
  rotated[0] = null;
  assert.equal(measureHand(rotated).palmOpen, true);
});

test('requires a continuous one-second hold and waits for the middle pose', () => {
  const session = new Session();
  for (let t = 0; t < 1000; t += 50) assert.equal(session.observe(palm, t), null);
  assert.equal(session.state, 'standby');
  assert.equal(session.observe(palm, 1000), 'wake');
  assert.equal(session.state, 'waking');
  session.observe(palm, 1050);
  assert.equal(session.lastInteraction, null);
  session.middleReached(2500);
  assert.equal(session.state, 'active');
  assert.equal(session.remainingMs(2500), 15000);
});

test('a prolonged missing frame gap or a sustained closed gesture resets wake progress', () => {
  const session = new Session();
  session.observe(palm, 0); session.observe(palm, 100);
  session.tick(751);
  assert.equal(session.wakeProgress(), 0);
  session.observe(palm, 800); session.observe(palm, 850);
  for(let t=900;t<=1000;t+=50) session.observe(pinch,t);
  assert.equal(session.wakeProgress(), 0);
  for(let t=1100;t<2100;t+=50) session.observe(palm,t);
  assert.equal(session.state, 'standby');
  assert.equal(session.observe(palm,2100), 'wake');
});

test('slow but fresh inference can wake while the animation timer runs', () => {
  const session = new Session();
  let wokeAt = null;
  for(let now=220;now<=1800;now+=16) {
    if((now-220)%240===0 && session.observe(palm,now,now-220)==='wake') wokeAt=now;
    session.tick(now);
  }
  assert.equal(session.state, 'waking');
  assert.equal(wokeAt, 1420);
});

test('brief missed detections pause progress and do not contribute to the one-second hold', () => {
  const session = new Session();
  for(let t=0;t<=400;t+=50) session.observe(palm,t);
  session.observe(null,450); session.observe(pinch,500);
  assert.equal(session.wakeProgress(), .4);
  session.observe(palm,550);
  assert.equal(session.wakeProgress(), .4);
  for(let t=600;t<=1100;t+=50) session.observe(palm,t);
  assert.equal(session.state, 'standby');
  assert.equal(session.observe(palm,1150), 'wake');
});

test('relaxed open-palm classification can maintain a hold but cannot start one', () => {
  const relaxed = { ...palm, palmOpen:false, palmOpenHeld:true };
  const session = new Session();
  for(let t=0;t<=1200;t+=50) session.observe(relaxed,t);
  assert.equal(session.state,'standby');
  assert.equal(session.wakeProgress(),0);
  session.observe(palm,1250);
  for(let t=1300;t<2250;t+=50) session.observe(relaxed,t);
  assert.equal(session.observe(relaxed,2250),'wake');
});

test('stale or reordered inference cannot add wake hold time', () => {
  const session = new Session();
  for(let t=0;t<=400;t+=50) session.observe(palm,t);
  session.observe(palm,450,100);
  session.observe(palm,500,350);
  assert.equal(session.wakeProgress(),.4);
  assert.equal(session.state,'standby');
});

test('returns exactly at 15 seconds of inactivity; short losses remain active', () => {
  const session = new Session(); session.wakeMouse(); session.middleReached(2000);
  session.observe(pinch, 3000);
  session.observe(null, 4000);
  assert.equal(session.tick(17999), null);
  assert.equal(session.state, 'active');
  assert.equal(session.tick(18000), 'home');
  assert.equal(session.state, 'returning');
});

test('valid hand presence extends activity; stale, future and reordered results do not', () => {
  const session = new Session(); session.wakeMouse(); session.middleReached(2000);
  session.observe(pinch, 3000);
  session.observe(pinch, 4000, 3600);
  session.observe(pinch, 4100, 4200);
  session.observe(pinch, 5000, 2900);
  assert.equal(session.lastInteraction, 3000);
  session.observe(pinch, 6000, 5900);
  assert.equal(session.lastInteraction, 5900);
  assert.equal(session.tick(20899), null);
  assert.equal(session.tick(20900), 'home');
});

test('a result arriving at the expired deadline cannot revive the active session', () => {
  const session = new Session(); session.wakeMouse(); session.middleReached(2000);
  assert.equal(session.observe(palm, 17000), 'home');
  assert.equal(session.state, 'returning');
});

test('ignores wake while returning and requires releasing a held wake gesture after sleep', () => {
  const session = new Session(); session.wakeMouse(); session.middleReached(2000);
  session.requestHome();
  for(let t=2100;t<=3200;t+=50) assert.equal(session.observe(palm,t), null);
  assert.equal(session.wakeMouse(), false);
  session.homeReached();
  for(let t=3300;t<=4400;t+=50) assert.equal(session.observe(palm,t), null);
  assert.equal(session.state, 'standby');
  session.observe(pinch,4450);
  for(let t=4500;t<5500;t+=50) session.observe(palm,t);
  assert.equal(session.observe(palm,5500), 'wake');
});
