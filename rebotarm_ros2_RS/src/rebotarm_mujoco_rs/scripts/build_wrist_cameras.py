#!/usr/bin/env python3
"""Convert the console's canonical wrist URDFs to visual-only MuJoCo geoms.

Developer dependencies: numpy, scipy, pycollada. Runtime needs none of the
conversion tools. Rerunning replaces only the generated camera sections.
"""
from pathlib import Path
import json
import re
import shutil
import struct
import xml.etree.ElementTree as ET

import collada
import numpy as np
from scipy.spatial.transform import Rotation

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
SOURCE = REPO / "reBotArm_simulator-RS/public/models/wrist-cameras"
MODELS = PACKAGE / "models"


def transform(origin):
    result = np.eye(4)
    if origin is not None:
        result[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        result[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
    return result


def pose(matrix):
    xyzw = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return {"pos": matrix[:3, 3].tolist(), "quat": xyzw[[3, 0, 1, 2]].tolist()}


def numbers(values):
    return " ".join(f"{value:.12g}" for value in values)


def write_stl(path, triangles):
    triangles = np.asarray(triangles, dtype=np.float32)
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    length = np.linalg.norm(normals, axis=1)
    normals /= np.where(length > 0, length, 1)[:, None]
    records = np.zeros(len(triangles), dtype=[("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    records["normal"] = normals
    records["vertices"] = triangles
    path.write_bytes(b"Converted from RealSense d435.dae".ljust(80, b"\0") + struct.pack("<I", len(records)) + records.tobytes())


def build():
    assets, geoms, definitions = [], [], {}
    copied = {}
    for variant in ("d405", "d435i", "gemini2"):
        urdf = SOURCE / "urdf" / f"{variant}.urdf"
        root = ET.parse(urdf).getroot()
        links = {link.get("name"): link for link in root.findall("link")}
        joints = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
        frames = {"gripper_end": np.eye(4)}

        def frame(name):
            if name not in frames:
                joint = joints[name]
                frames[name] = frame(joint.find("parent").get("link")) @ transform(joint.find("origin"))
            return frames[name]

        names, materials = [], []
        for link_name, link in links.items():
            for index, visual in enumerate(link.findall("visual")):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    continue
                src = (urdf.parent / mesh.get("filename")).resolve()
                scale = mesh.get("scale", "1 1 1")
                local = frame(link_name) @ transform(visual.find("origin"))
                pieces = []
                if src.suffix.lower() == ".dae":
                    # Collada meshes retain their native Z-up coordinates, as
                    # used by ROS/RViz. URDF visual origins supply the rotation.
                    document = collada.Collada(str(src))
                    for geometry in document.scene.objects("geometry"):
                        for primitive in geometry.primitives():
                            triangles = primitive.triangleset() if hasattr(primitive, "triangleset") else primitive
                            color = primitive.material.effect.diffuse
                            vertices = triangles.vertex[triangles.vertex_index] * (document.assetInfo.unitmeter or 1)
                            # MuJoCo's STL decoder limits each mesh to 200k
                            # triangles. Preserve all triangles in smaller parts.
                            for start in range(0, len(vertices), 150000):
                                filename = f"wrist_{variant}_body_{len(pieces):02d}.stl"
                                write_stl(MODELS / "meshes" / filename, vertices[start:start + 150000])
                                pieces.append((filename, color if isinstance(color, tuple) else (0.7, 0.7, 0.7, 1)))
                else:
                    if src not in copied:
                        filename = f"wrist_{src.parent.name}_{src.stem}.stl"
                        shutil.copyfile(src, MODELS / "meshes" / filename)
                        copied[src] = filename
                    color_node = visual.find("material/color")
                    color = np.fromstring(color_node.get("rgba"), sep=" ") if color_node is not None else [0.7, 0.73, 0.75, 1]
                    pieces.append((copied[src], color))
                for piece_index, (filename, color) in enumerate(pieces):
                    mesh_name = Path(filename).stem
                    asset = f'    <mesh name="{mesh_name}" file="{filename}" scale="{scale}" />'
                    if asset not in assets:
                        assets.append(asset)
                    name = f"wrist_{variant}_{link_name}_{index}_{piece_index}"
                    material = f"{name}_mat"
                    rgba = [*color[:3], 1 if variant == "d405" else 0]
                    assets.append(f'    <material name="{material}" rgba="{numbers(rgba)}" specular="0.35" shininess="0.5" />')
                    location = pose(local)
                    geoms.append(f'                    <geom name="{name}" type="mesh" mesh="{mesh_name}" pos="{numbers(location["pos"])}" quat="{numbers(location["quat"])}" material="{material}" group="{1 if variant == "d405" else 4}" contype="0" conaffinity="0" mass="0" />')
                    names.append(name)
                    materials.append(material)
        optical = frame("camera_color_optical_frame")
        # ROS optical +Z looks forward / +Y down; MuJoCo camera -Z looks
        # forward / +Y up. Reverse Y and Z without changing the optical centre.
        optical[:3, :3] = optical[:3, :3] @ np.diag([1, -1, -1])
        definitions[variant] = {"label": {"d405": "RealSense D405", "d435i": "RealSense D435i", "gemini2": "Orbbec Gemini 2"}[variant],
                                "geoms": names, "materials": materials, "camera": {**pose(optical), "fovy": 62.82}}

    default = definitions["d405"]["camera"]
    geoms.append(f'                    <camera name="wrist_rgb" pos="{numbers(default["pos"])}" quat="{numbers(default["quat"])}" fovy="{default["fovy"]}" />')
    arm_path = MODELS / "rs_arm.xml"
    arm = arm_path.read_text()
    arm = re.sub(r'    <!-- B601-RS / RealSense D405.*?<mesh name="d405_wrist_mount"[^>]+/>\n', "", arm, flags=re.S)
    arm = re.sub(r'    <!-- The mount uses the upstream.*?<material name="d405_lens_mat"[^>]+/>\n', "", arm, flags=re.S)
    asset_section = "    <!-- BEGIN GENERATED WRIST CAMERA ASSETS -->\n" + "\n".join(assets) + "\n    <!-- END GENERATED WRIST CAMERA ASSETS -->"
    if "BEGIN GENERATED WRIST CAMERA ASSETS" in arm:
        arm = re.sub(r'    <!-- BEGIN GENERATED WRIST CAMERA ASSETS -->.*?    <!-- END GENERATED WRIST CAMERA ASSETS -->', asset_section, arm, flags=re.S)
    else:
        arm = arm.replace("  </asset>", asset_section + "\n  </asset>")
    geom_section = "                    <!-- BEGIN GENERATED WRIST CAMERA GEOMS -->\n" + "\n".join(geoms) + "\n                    <!-- END GENERATED WRIST CAMERA GEOMS -->"
    if "BEGIN GENERATED WRIST CAMERA GEOMS" in arm:
        arm = re.sub(r'                    <!-- BEGIN GENERATED WRIST CAMERA GEOMS -->.*?                    <!-- END GENERATED WRIST CAMERA GEOMS -->', geom_section, arm, flags=re.S)
    else:
        arm = re.sub(r'                    <!-- Validated D405 eye-in-hand assembly\..*?<camera name="wrist_rgb".*?/>', geom_section, arm, flags=re.S)
    arm_path.write_text(arm)
    (MODELS / "wrist_cameras.json").write_text(json.dumps(definitions, indent=2) + "\n")
    print({key: len(value["geoms"]) for key, value in definitions.items()})


if __name__ == "__main__":
    build()
