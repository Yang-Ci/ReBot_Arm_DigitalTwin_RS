const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const { mapLeaderModelAngles } = require('../public/js/ros/leader-model.js');

test('Leader display keeps its own directions and handle angle, independent of follower transmission', () => {
  const pose = mapLeaderModelAngles([10, 20, -30, 40, 50, 60, 25]);
  const deg = 180 / Math.PI;
  for (const [name, expected] of Object.entries({joint1:10,joint2:20,joint3:-30,joint4:-40,joint5:50,joint6:60,joint7_left:25,joint7_right:-25})) {
    assert(Math.abs(pose[name] * deg - expected) < 1e-10);
  }
});

test('UART whole turns show the same physical pose', () => {
  const first = mapLeaderModelAngles([10,20,-30,40,50,60,25]);
  const turned = mapLeaderModelAngles([370,380,330,400,410,420,385]);
  for (const name in first) assert(Math.abs(first[name] - turned[name]) < 1e-10);
});

test('Malformed samples cannot change displayed pose', () => {
  for (const value of [null, [], [0,0,0,0,0,0], [0,0,0,0,0,0,NaN], [0,0,0,0,0,0,Infinity], ['0',0,0,0,0,0,0]]) {
    assert.equal(mapLeaderModelAngles(value), null);
  }
});

test('Vendored official model assets are complete and unchanged', () => {
  const dir = path.join(__dirname,'../public/models/leader-arm102');
  const source = JSON.parse(fs.readFileSync(path.join(dir,'source.json'),'utf8'));
  assert.equal(source.assets.filter(a => a.file.endsWith('.STL')).length,9);
  for (const asset of source.assets) {
    const data = fs.readFileSync(path.join(dir,asset.file));
    assert.equal(data.length,asset.bytes);
    assert.equal(crypto.createHash('sha256').update(data).digest('hex'),asset.sha256);
  }
  const urdf = fs.readFileSync(path.join(dir,'urdf/leader.urdf'),'utf8');
  for (const match of urdf.matchAll(/filename="([^"]+)"/g)) assert(fs.existsSync(path.resolve(dir,'urdf',match[1])));
});
