# rebotarm_mujoco_rs

ROS 2 integration for the B601-RS MuJoCo model. The required model XML and STL
meshes are tracked directly in `models/`, based on
`LAN-GER/reBot-B601-RS-for-mujoco_sim` revision
`1249cb6efdf393ba636056fc41df30dc6ba389aa` with local gripper, material, and
Seeed-badge updates. The upstream repository had no LICENSE file at that
revision.

Only model assets are used. This package does not import the upstream
`rebot_b601_rs_sim` algorithms, QP solver, examples, or tests. ROS integration,
scene detection, IK, and task execution are implemented in this package.

The wrapper does not open SocketCAN or send hardware commands. It subscribes to
ROS `JointState`, so the real arm continues to have a single owner: the
`reBotArmController` node.

The RS package now also includes a physics grasp environment with red, blue,
and yellow objects, overhead and switchable UVC32 / D405 / D435i / Gemini 2 wrist ROS cameras, object detections, Cartesian IK,
trajectory actions, and task recording services used by `rebotarm_agent`.

Modes:

- `kinematic`: directly synchronizes ROS joint state into MuJoCo.
- `physics`: tracks ROS targets with conservative PD plus MuJoCo bias forces.

```bash
ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
  arm_namespace:=rebotarm_rs simulation_mode:=physics use_viewer:=true
```

## TCP marker visibility

The red `tcp` site is an optional position marker attached to `gripper_end`.
It has no collision or dynamics role. Task IK uses its configured body and
offset independently. The marker is hidden by default, including in ROS camera
images. In the selection window, check **Show TCP marker**, or press **T** in
the MuJoCo viewer to show/hide it. The checkbox follows keyboard changes.
This changes only the marker's opacity and preserves simulation state.

Direct ROS launch supports `show_tcp_marker:=true`. To toggle it while running:

```bash
ros2 param set /rebotarm_rs_mujoco show_tcp_marker true
ros2 param set /rebotarm_rs_mujoco show_tcp_marker false
```

The viewer publishes the current setting on the transient-local
`/<arm_namespace>/mujoco/tcp_marker_visible` topic. This display option applies
to the native viewer; ROS camera images keep the marker hidden.

## Wrist camera selection

With `use_viewer:=true`, a small **MuJoCo · Wrist Camera** selection window opens
alongside the native viewer. Select 32×32 UVC, RealSense D405, RealSense D435i or Orbbec
Gemini 2 there, or press **C** in the MuJoCo viewer to cycle through them. The
selection window follows keyboard changes. The old 30-degree mount and camera
proxy have been removed; these assemblies reuse the RS console's upstream URDF
mount, body and color optical transforms.

The default is D405. Choose a different model at startup:

```bash
REBOTARM_WRIST_CAMERA_MODEL=uvc32 ./rebotarm start rs_sim
```

Direct ROS launch supports `wrist_camera_model:=d435i`. To change a running
simulation without the selection window:

```bash
ros2 param set /rebotarm_rs_mujoco wrist_camera_model uvc32
```

Switching changes only visual camera geometry and the wrist RGB pose. It
preserves joint positions, targets, grasp objects and simulation time. Both ROS
image renderers receive the selected model through the transient-local
`/rebotarm_rs/mujoco/wrist_camera_model` topic, including renderers started later.
Wrist image and CameraInfo topic names remain stable across selections.

Use `enable_camera_selector:=false` (or `REBOTARM_CAMERA_SELECTOR=false`) to
hide the extra selection window; the ROS parameter and C key still work.
Headless launches do not open the selector by default. The camera models are
visual only, with no added collision, joint or mass terms. The simulation uses
a shared 62.82° vertical field of view; device calibration is separate.
See [model sources and regeneration](models/THIRD_PARTY_MODELS.md).

Important topics:

- `/rebotarm_rs/mujoco/object_states`
- `/rebotarm_rs/mujoco/overhead_rgb/image_raw`
- `/rebotarm_rs/mujoco/wrist_rgb/image_raw`
- `/rebotarm_rs/mujoco/wrist_rgb/camera_info`
- `/rebotarm_rs/vision/color_blocks/detections`

Task endpoints:

- `/rebotarm_rs/move_to_pose_ik`
- `/rebotarm_rs/move_to_pose`
- `/rebotarm_rs/follow_joint_trajectory`

The UVC32 bracket is generated from official STEP CAD. Its 32×32 mm board
and lens are schematic simulation geometry; the nominal optical frame and
shared simulation FOV are not physical-device calibration.
