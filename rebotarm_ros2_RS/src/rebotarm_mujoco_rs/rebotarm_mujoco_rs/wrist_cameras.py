"""Switch visual wrist assemblies and the RGB optical pose without reloading."""
from pathlib import Path
import json

import mujoco

CAMERA_MODELS = ("d405", "d435i", "gemini2", "uvc32")


class WristCameraAssemblies:
    def __init__(self, model, model_path):
        self.model = model
        self.definitions = json.loads((Path(model_path).parent / "wrist_cameras.json").read_text())
        self.camera_id = self._id(mujoco.mjtObj.mjOBJ_CAMERA, "wrist_rgb")
        self.geoms = {
            name: [self._id(mujoco.mjtObj.mjOBJ_GEOM, geom) for geom in self.definitions[name]["geoms"]]
            for name in CAMERA_MODELS
        }
        self.materials = {
            name: [self._id(mujoco.mjtObj.mjOBJ_MATERIAL, material) for material in self.definitions[name]["materials"]]
            for name in CAMERA_MODELS
        }
        self.active = "d405"

    def _id(self, kind, name):
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise ValueError(f"Wrist camera model is missing {name!r}")
        return value

    def select(self, name):
        if name not in CAMERA_MODELS:
            raise ValueError("wrist_camera_model must be d405, d435i, gemini2 or uvc32")
        for variant in CAMERA_MODELS:
            selected = variant == name
            for geom_id in self.geoms[variant]:
                self.model.geom_group[geom_id] = 1 if selected else 4
                self.model.geom_rgba[geom_id, 3] = 1 if selected else 0
            for material_id in self.materials[variant]:
                self.model.mat_rgba[material_id, 3] = 1 if selected else 0
        camera = self.definitions[name]["camera"]
        self.model.cam_pos[self.camera_id] = camera["pos"]
        self.model.cam_quat[self.camera_id] = camera["quat"]
        self.model.cam_fovy[self.camera_id] = camera["fovy"]
        self.active = name

    @property
    def label(self):
        return self.definitions[self.active]["label"]
