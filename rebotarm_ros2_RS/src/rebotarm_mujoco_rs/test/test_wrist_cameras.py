from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from rebotarm_mujoco_rs.wrist_cameras import CAMERA_MODELS, WristCameraAssemblies

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / "models/rs_grasp_scene.xml"
URDFS = PACKAGE.parents[2] / "reBotArm_simulator-RS/public/models/wrist-cameras/urdf"


@pytest.fixture
def scene():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    return model, data, WristCameraAssemblies(model, MODEL)


def optical_transform(model_name):
    root = ET.parse(URDFS / f"{model_name}.urdf").getroot()
    joints = {joint.find("child").get("link"): joint for joint in root.findall("joint")}

    def pose(name):
        if name == "gripper_end":
            return np.eye(4)
        joint = joints[name]
        origin = joint.find("origin")
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
        transform[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        return pose(joint.find("parent").get("link")) @ transform

    result = pose("camera_color_optical_frame")
    result[:3, :3] = result[:3, :3] @ np.diag([1, -1, -1])
    return result


@pytest.mark.parametrize("name", CAMERA_MODELS)
def test_switch_preserves_simulation_and_uses_urdf_optical_frame(scene, name):
    model, data, cameras = scene
    data.qpos[0] = 0.4
    data.qvel[0] = 0.2
    data.time = 3.25
    state = (data.qpos.copy(), data.qvel.copy(), data.ctrl.copy(), model.body_mass.copy(), model.body_inertia.copy())
    cameras.select(name)
    mujoco.mj_forward(model, data)
    for before, after in zip(state, (data.qpos, data.qvel, data.ctrl, model.body_mass, model.body_inertia)):
        np.testing.assert_array_equal(before, after)
    assert data.time == 3.25
    for variant in CAMERA_MODELS:
        for geom_id in cameras.geoms[variant]:
            assert model.geom_group[geom_id] == (1 if variant == name else 4)
            assert model.geom_rgba[geom_id, 3] == (1 if variant == name else 0)
            assert model.geom_contype[geom_id] == model.geom_conaffinity[geom_id] == 0
        for material_id in cameras.materials[variant]:
            assert model.mat_rgba[material_id, 3] == (1 if variant == name else 0)
    expected = optical_transform(name)
    np.testing.assert_allclose(model.cam_pos[cameras.camera_id], expected[:3, 3], atol=1e-11)
    wxyz = model.cam_quat[cameras.camera_id]
    np.testing.assert_allclose(Rotation.from_quat(wxyz[[1, 2, 3, 0]]).as_matrix(), expected[:3, :3], atol=1e-11)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "d405_camera_body") == -1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, "d405_wrist_mount") == -1


def test_invalid_selection_preserves_previous_camera(scene):
    model, _, cameras = scene
    cameras.select("gemini2")
    before = model.cam_pos.copy(), model.cam_quat.copy(), model.geom_group.copy(), model.mat_rgba.copy()
    with pytest.raises(ValueError, match="wrist_camera_model"):
        cameras.select("old_camera")
    for first, second in zip(before, (model.cam_pos, model.cam_quat, model.geom_group, model.mat_rgba)):
        np.testing.assert_array_equal(first, second)
    assert cameras.active == "gemini2"
