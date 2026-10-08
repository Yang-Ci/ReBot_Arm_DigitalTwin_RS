# Arm102 Leader model

This preview uses the **Star Arm 102-LD** model published with FashionStar's official browser controller, including the handle and finger rings. It is a display model, not a follower controller or a collision model.

- Manufacturer: https://fashionstar.com.hk/robot-arm/star-arm-102/
- Official viewer: https://fashionstar.com.hk/wiki/zh/software/robot-arm/data/102-web-controller/Browser_SDK/
- URDF source: https://fashionstar.com.hk/wiki/zh/software/robot-arm/data/102-web-controller/Browser_SDK/assets/urdf/ld/model.urdf
- Mesh source folder: the same viewer's `assets/models/ld/`.
- Official LD/HD archive: https://github.com/servodevelop/Star-Arm-102/blob/5d066333c65ac2c8aadcf0a36ee297e9521139be/hardware/102-ld/robot-description/star-arm-102-ld-hd-urdf.zip

The archive and shared GitHub model differ from the LD browser model. The bundled geometry comes from the official browser's LD directory. The Git commit above identifies the reference archive, not the web assets; exact web URLs, retrieval date, byte sizes and SHA-256 hashes are recorded in `source.json`. `urdf/upstream-model.urdf` preserves the original web URDF.

`urdf/leader.urdf` adapts only:

1. Mesh references to local relative paths.
2. J5 from fixed to revolute on axis `0 0 -1`, matching the official viewer's `LD_JOINTS` in `src/robot_model_configs.js`.
3. The right handle joint to mimic the left with multiplier −1, matching the viewer's paired handle motion.

The display converts the ROS leader's seven raw degree samples to radians. J4 has direction −1 as in the official `src/joint_mapper.js`; other arm joints have direction +1. The handle uses its own angle at scale 1, with opposing ring motion. The follower's direction corrections and ×6 gripper transmission are **not** applied here. Whole turns are normalized to the same branches as the ROS leader policy. Display limits are ignored so the mesh cannot silently clip the actual reported pose. These settings do not alter hardware commands, zero calibration or follower limits.

As the official mapper notes, direction/zero offsets still require comparison with a calibrated physical arm. This preview assumes the project's leader reference zero. Mesh colours are a local display finish, not part of the hardware model.

Assets are local so company testing does not need access to the manufacturer's site. Preserve the upstream notices in `UPSTREAM_LICENSE_SCOPE.md`; upstream's current scope notice does not establish a uniform licence for model assets.
