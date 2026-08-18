#!/usr/bin/env python3
"""Generate the ShDog MuJoCo model from the normalized URDF and model facts."""

from __future__ import annotations

import argparse
import math
import tempfile
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPO_ROOT / "assets/sh_dog/urdf/sh_dog.urdf"
MODEL_PATH = REPO_ROOT / "assets/sh_dog/model.toml"
OUTPUT_DIR = REPO_ROOT / "assets/sh_dog/mujoco"


def _values(element: ET.Element | None, name: str, default: str) -> str:
    return element.get(name, default) if element is not None else default


def _half(values: str) -> str:
    return " ".join(f"{float(value) / 2.0:.12g}" for value in values.split())


def _quat(rpy: str) -> str:
    roll, pitch, yaw = (float(value) for value in rpy.split())
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return " ".join(f"{value:.12g}" for value in (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ))


def _origin_attributes(element: ET.Element | None) -> dict[str, str]:
    if element is None:
        return {}
    attributes = {"pos": _values(element, "xyz", "0 0 0")}
    rpy = _values(element, "rpy", "0 0 0")
    if any(abs(float(value)) > 1.0e-15 for value in rpy.split()):
        attributes["quat"] = _quat(rpy)
    return attributes


def _add_link_contents(body: ET.Element, link: ET.Element, mesh_assets: ET.Element) -> None:
    inertial = link.find("inertial")
    if inertial is not None:
        inertia = inertial.find("inertia")
        attributes = _origin_attributes(inertial.find("origin"))
        attributes["mass"] = inertial.find("mass").get("value")
        attributes["fullinertia"] = " ".join(
            inertia.get(key) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        )
        ET.SubElement(body, "inertial", attributes)

    visual = link.find("visual")
    if visual is not None:
        mesh = visual.find("geometry/mesh")
        if mesh is not None:
            name = link.get("name")
            filename = Path(mesh.get("filename")).name
            ET.SubElement(mesh_assets, "mesh", {"name": name, "file": filename})
            attributes = _origin_attributes(visual.find("origin"))
            attributes.update({"type": "mesh", "mesh": name, "class": "visual"})
            color = visual.find("material/color")
            if color is not None:
                attributes["rgba"] = color.get("rgba")
            ET.SubElement(body, "geom", attributes)

    collision = link.find("collision")
    if collision is None:
        return
    attributes = _origin_attributes(collision.find("origin"))
    attributes.update({"name": f"{link.get('name')}_collision", "class": "collision"})
    geometry = collision.find("geometry")
    box = geometry.find("box")
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    if box is not None:
        attributes.update({"type": "box", "size": _half(box.get("size"))})
    elif cylinder is not None:
        attributes.update({
            "type": "cylinder",
            "size": f"{cylinder.get('radius')} {float(cylinder.get('length')) / 2.0:.12g}",
        })
    elif sphere is not None:
        attributes.update({"type": "sphere", "size": sphere.get("radius")})
    else:
        raise ValueError(f"unsupported collision geometry on {link.get('name')}")
    ET.SubElement(body, "geom", attributes)


def build(output_dir: Path = OUTPUT_DIR) -> tuple[Path, Path]:
    model_cfg = tomllib.loads(MODEL_PATH.read_text(encoding="utf-8"))
    urdf = ET.parse(URDF_PATH).getroot()
    links = {link.get("name"): link for link in urdf.findall("link")}
    child_joints = {joint.find("child").get("link"): joint for joint in urdf.findall("joint")}
    children: dict[str, list[ET.Element]] = {name: [] for name in links}
    for joint in urdf.findall("joint"):
        children[joint.find("parent").get("link")].append(joint)
    root_names = set(links) - set(child_joints)
    if root_names != {"base_link"}:
        raise ValueError(f"expected base_link as the only root, got {sorted(root_names)}")

    mujoco = ET.Element("mujoco", {"model": model_cfg["robot"]["name"]})
    ET.SubElement(mujoco, "compiler", {
        "angle": "radian", "meshdir": "../raw/meshes", "autolimits": "true",
    })
    ET.SubElement(mujoco, "option", {"timestep": "0.005", "integrator": "RK4"})
    defaults = ET.SubElement(mujoco, "default")
    ET.SubElement(defaults, "geom", {"friction": "0.8 0.02 0.001", "condim": "3"})
    visual_default = ET.SubElement(defaults, "default", {"class": "visual"})
    ET.SubElement(visual_default, "geom", {"contype": "0", "conaffinity": "0", "group": "2"})
    collision_default = ET.SubElement(defaults, "default", {"class": "collision"})
    # Robot geoms only collide with world geoms, matching disabled articulation self-collision.
    ET.SubElement(collision_default, "geom", {"contype": "1", "conaffinity": "2", "group": "3"})
    assets = ET.SubElement(mujoco, "asset")
    worldbody = ET.SubElement(mujoco, "worldbody")

    def add_body(parent: ET.Element, link_name: str, joint: ET.Element | None = None) -> None:
        attributes = {"name": link_name}
        if joint is not None:
            attributes.update(_origin_attributes(joint.find("origin")))
        elif link_name == "base_link":
            attributes["pos"] = f"0 0 {model_cfg['default_state']['base_height_m']}"
        body = ET.SubElement(parent, "body", attributes)
        if joint is None:
            ET.SubElement(body, "freejoint", {"name": "floating_base"})
        elif joint.get("type") != "fixed":
            limit = joint.find("limit")
            joint_type = next(kind for kind in ("abad", "hip", "knee") if f"_{kind}_" in joint.get("name"))
            armature = (
                model_cfg["actuator"]["low_speed_equivalent_inertia_kg_m2"]
                * model_cfg["additional_reduction"][joint_type] ** 2
            )
            ET.SubElement(body, "joint", {
                "name": joint.get("name"),
                "type": "hinge",
                "axis": _values(joint.find("axis"), "xyz", "1 0 0"),
                "range": f"{limit.get('lower')} {limit.get('upper')}",
                "armature": f"{armature:.12g}",
            })
        _add_link_contents(body, links[link_name], assets)
        for child_joint in children[link_name]:
            add_body(body, child_joint.find("child").get("link"), child_joint)

    add_body(worldbody, "base_link")
    actuators = ET.SubElement(mujoco, "actuator")
    torque_factor = model_cfg["actuator"]["torque_speed"]["torque_safety_factor"]
    for joint_name in model_cfg["robot"]["joint_order"]:
        joint_type = next(kind for kind in ("abad", "hip", "knee") if f"_{kind}_" in joint_name)
        limit = (
            model_cfg["actuator"]["peak_torque_nm"]
            * model_cfg["additional_reduction"][joint_type]
            * torque_factor
        )
        ET.SubElement(actuators, "motor", {
            "name": joint_name, "joint": joint_name, "ctrllimited": "true",
            "ctrlrange": f"{-limit:.12g} {limit:.12g}",
        })

    ET.indent(mujoco, space="  ")
    output_dir.mkdir(parents=True, exist_ok=True)
    model_output = output_dir / "sh_dog.xml"
    ET.ElementTree(mujoco).write(model_output, encoding="utf-8", xml_declaration=True)

    scene = ET.Element("mujoco", {"model": "sh_dog_scene"})
    ET.SubElement(scene, "include", {"file": "sh_dog.xml"})
    scene_world = ET.SubElement(scene, "worldbody")
    ET.SubElement(scene_world, "light", {"pos": "0 0 3", "dir": "0 0 -1", "directional": "true"})
    ET.SubElement(scene_world, "geom", {
        "name": "floor", "type": "plane", "size": "20 20 0.05",
        "friction": "0.8 0.02 0.001", "contype": "2", "conaffinity": "1",
        "rgba": "0.18 0.2 0.22 1",
    })
    ET.indent(scene, space="  ")
    scene_output = output_dir / "scene.xml"
    ET.ElementTree(scene).write(scene_output, encoding="utf-8", xml_declaration=True)
    return model_output, scene_output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Fail if generated files differ.")
    args = parser.parse_args()
    if args.check:
        expected = (OUTPUT_DIR / "sh_dog.xml", OUTPUT_DIR / "scene.xml")
        with tempfile.TemporaryDirectory() as directory:
            generated = build(Path(directory))
            changed = [
                str(path.relative_to(REPO_ROOT))
                for path, candidate in zip(expected, generated, strict=True)
                if not path.exists() or path.read_bytes() != candidate.read_bytes()
            ]
        if changed:
            raise SystemExit("generated files were stale: " + ", ".join(changed))
        outputs = expected
    else:
        outputs = build()
    for output in outputs:
        print(output.relative_to(REPO_ROOT))


if __name__ == "__main__":
    main()
