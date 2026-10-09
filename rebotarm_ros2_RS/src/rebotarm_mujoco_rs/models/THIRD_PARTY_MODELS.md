# Third-party model assets

## B601-RS UVC32 / D405 / D435i / Gemini 2 wrist assemblies

The former 30-degree D405 mount and primitive camera proxy have been removed.
The `wrist_*` mesh assets and generated sections in `rs_arm.xml` use the same
four assemblies as the RS web console. The original three use vendor bodies;
UVC32 uses the official bracket CAD with a generic camera-board schematic:

- Mounts and assembly positions: `xiehuangbao888/rebot_visual_grasp`, commit
  `dd28d65598deec767cf95fa45521d69b38155833` (package declares Apache-2.0).
- Mount CAD: Seeed-Projects/reBot-DevArm via Yang-Ci/Camera-Mounts,
  CERN-OHL-W-2.0.
- RealSense camera bodies: realsenseai/realsense-ros, commit
  `9215f26e8348ad5922a608b77882a9bfe05940f0`, Apache-2.0.
- Gemini 2 body: orbbec/OrbbecSDK_ROS2, commit
  `c153462518ad674650bafa4464fda72c27ab797a`, Apache-2.0.

Original sources, hashes, upstream links and full attribution are retained in
`reBotArm_simulator-RS/public/models/wrist-cameras/README.md` and `source.json`
at the repository root. License texts are also installed in `models/licenses/`.

`scripts/build_wrist_cameras.py` converts those URDF visual origins into fixed
geoms on `gripper_end`, retaining the source mount and camera transforms. STL
meshes are copied unchanged. The D435 Collada body is converted to binary STL
parts with its material colors and triangles preserved; large parts are split
to respect MuJoCo's 200,000-triangle STL limit.

The wrist RGB camera uses the selected model's color optical frame, converted
from ROS optical axes to MuJoCo camera axes. The common 62.82-degree simulation
vertical field of view is retained for all variants; it is not a calibration
of a physical device. Camera geoms are visual only and do not add collision,
mass or joints. Inactive variants have zero alpha and use hidden geom group 4.

To regenerate, install developer dependencies `numpy`, `scipy` and `pycollada`,
then run `python scripts/build_wrist_cameras.py` from this ROS package.

## UVC32 generated assembly

The bracket mesh is tessellated from Seeed-Projects/reBot-DevArm
`hardware/camera-mounts/b601-camera-mounts/UVC32_mount.step`, CERN-OHL-W-2.0.
The camera-board mounting plane and 28×28 mm hole spacing are extracted from
CAD; the common clamp interface is aligned using the official D435 reference
CAD and the existing wrist attachment. The generated 32×32×1.6 mm board, lens
and glass are simulation schematics, not a vendor device model. Its nominal
optical origin and shared FOV are not a physical-device calibration.

Original STEP files, generated mesh hashes and the CAD generation script are
retained with the canonical wrist camera assets.
