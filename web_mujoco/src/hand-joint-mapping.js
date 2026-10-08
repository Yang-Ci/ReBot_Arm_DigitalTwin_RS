import { ARM_JOINTS } from './kinematics.js';

export const SOURCE_GRIPPER_MAX = .0715;
export const MUJOCO_GRIPPER_MAX = .05;

export function mapHandPose(angles) {
  if (!angles || !Number.isFinite(angles.gripper)) return null;
  const pose = {};
  for (const joint of ARM_JOINTS) {
    if (!Number.isFinite(angles[joint.name])) return null;
    pose[joint.name] = Math.min(joint.max, Math.max(joint.min, angles[joint.name]));
  }
  pose.joint7 = Math.min(1, Math.max(0, angles.gripper / SOURCE_GRIPPER_MAX)) * MUJOCO_GRIPPER_MAX;
  return pose;
}
