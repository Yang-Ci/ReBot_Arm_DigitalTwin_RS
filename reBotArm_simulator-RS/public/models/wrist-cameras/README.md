# B601-RS wrist camera assemblies

These are fixed URDF attachments rooted at an empty `gripper_end` link. The
browser attaches that root to the existing RS wrist, so changing cameras does
not reload the arm or send motion commands. The generated URDFs retain the
vendor camera bodies, mounting screw offsets and nominal optical frames.

## Assembly parameters

All values are copied from the bundled upstream assembly Xacros. XYZ is in
metres; RPY is in radians. D405 and D435i camera origins refer to their bottom
screw frames. Gemini 2's camera origin refers to `camera_link`.

| Model | Mount XYZ | Mount RPY | Camera XYZ | Camera RPY |
| --- | --- | --- | --- | --- |
| D405 | -0.1201 0.0003 0.0507 | 1.5827 0.0024 1.5515 | -0.0003 0.0443 -0.0093 | -0.0225 -1.0448 -1.5460 |
| D435i | -0.1201 0.0003 0.0450 | 1.5827 0.0024 1.5515 | -0.0003 0.0445 -0.0095 | 0.0126 -1.0489 -1.5981 |
| Gemini 2 | -0.1201 0.0003 0.0450 | 1.5827 0.0024 1.5515 | 0.0249 0.0487 0.0090 | 0.0042 -1.0502 -1.5718 |

## Sources and licensing

- Assembly Xacros and converted mount meshes:
  [xiehuangbao888/rebot_visual_grasp](https://github.com/xiehuangbao888/rebot_visual_grasp/tree/dd28d65598deec767cf95fa45521d69b38155833/description),
  commit `dd28d65598deec767cf95fa45521d69b38155833`. Its package declares Apache-2.0.
- Mount CAD provenance:
  [Yang-Ci/Camera-Mounts](https://github.com/Yang-Ci/Camera-Mounts), which attributes
  the B601 mount designs to Seeed-Projects/reBot-DevArm under CERN-OHL-W-2.0.
  The STL files here are the converted meshes from `rebot_visual_grasp`.
- RealSense camera Xacros and D405 STL / D435 Collada body:
  [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros/tree/9215f26e8348ad5922a608b77882a9bfe05940f0/realsense2_description),
  commit `9215f26e8348ad5922a608b77882a9bfe05940f0`, Apache-2.0.
- Gemini 2 Xacro and STL meshes:
  [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2/tree/c153462518ad674650bafa4464fda72c27ab797a/orbbec_description),
  commit `c153462518ad674650bafa4464fda72c27ab797a`, Apache-2.0.
- `../../lib/ColladaLoader.js` is from Three.js r128, MIT; its license is
  included in `licenses/Three-MIT.txt`.

License texts are in `licenses/`. `source.json` records original asset paths,
source revisions, byte sizes and SHA-256 hashes. Files in `source/` and `meshes/`
are unchanged upstream copies.

## Regenerate

With the ROS Xacro Python package available, from the simulator directory:

```bash
python3 scripts/build-wrist-camera-urdfs.py
```

The script uses only the bundled sources. It replaces the upstream arm include
with the attachment root, expands camera macros and resolves package mesh URIs
to relative local URLs. Browser runtime requires only the generated URDFs and
meshes, including Collada support for the D435i body.
