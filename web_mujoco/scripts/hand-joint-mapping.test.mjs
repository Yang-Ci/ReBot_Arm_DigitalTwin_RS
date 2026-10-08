import test from 'node:test';
import assert from 'node:assert/strict';
import { mapHandPose } from '../src/hand-joint-mapping.js';

const middle = { joint1:0, joint2:.45, joint3:.65, joint4:0, joint5:0, joint6:0, gripper:.035 };
test('RS arm targets keep joint axes and radians; gripper preserves the whole normalized stroke', () => {
  const pose = mapHandPose(middle);
  for (const joint of ['joint1','joint2','joint3','joint4','joint5','joint6']) assert.equal(pose[joint],middle[joint]);
  assert.equal(mapHandPose({...middle,gripper:0}).joint7,0);
  assert.equal(mapHandPose({...middle,gripper:.0715}).joint7,.05);
  assert.equal(mapHandPose({...middle,gripper:.03575}).joint7,.025);
});
test('incomplete or corrupt frames never produce partial targets, and extreme inputs stay bounded', () => {
  assert.equal(mapHandPose({...middle,joint3:NaN}),null);
  assert.equal(mapHandPose({...middle,gripper:undefined}),null);
  assert.equal(mapHandPose(null),null);
  const pose = mapHandPose({...middle,joint1:10,joint2:-10,gripper:1});
  assert.equal(pose.joint1,2.8); assert.equal(pose.joint2,0); assert.equal(pose.joint7,.05);
});
