#!/usr/bin/env python3

"""Generate the normalized ShDog URDF package from immutable CAD assets."""

from __future__ import annotations

import argparse
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:
    raise RuntimeError("normalize_urdf.py requires Python 3.11 or newer") from None


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "assets/sh_dog/model.toml"
DEFAULT_OUTPUT = REPO_ROOT / "assets/sh_dog/urdf"
SHAPE_ATTRIBUTES = {
    "box": ("size",),
    "cylinder": ("radius", "length"),
    "sphere": ("radius",),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--check", action="store_true", help="verify the committed URDF is current")
    return parser.parse_args()


def load_config(path: Path) -> dict[str, Any]:
    with path.open("rb") as stream:
        config = tomllib.load(stream)
    if config.get("schema_version") != 1:
        raise ValueError(f"unsupported schema_version in {path}")
    return config


def format_number(value: float) -> str:
    if value == 0.0:
        return "0"
    return f"{value:.12g}"


def format_vector(values: list[float]) -> str:
    return " ".join(format_number(float(value)) for value in values)


def build_collision(link_name: str, config: dict[str, Any]) -> ET.Element:
    prefix = None
    role = link_name
    if link_name != "base_link":
        prefix, role = link_name.split("_", maxsplit=1)
        if prefix not in {"fl", "fr", "rl", "rr"}:
            raise ValueError(f"unsupported leg prefix in {link_name}")
    primitive = config["collision"][role]
    xyz = [float(value) for value in primitive["xyz"]]
    rpy = [float(value) for value in primitive.get("rpy", [0.0, 0.0, 0.0])]
    if prefix is not None:
        if role == "abad_link" and prefix.startswith("r"):
            xyz[0] = -xyz[0]
        if prefix.endswith("r"):
            xyz[1] = -xyz[1]

    collision = ET.Element("collision")
    ET.SubElement(collision, "origin", xyz=format_vector(xyz), rpy=format_vector(rpy))
    geometry = ET.SubElement(collision, "geometry")
    shape = primitive["shape"]
    if shape not in SHAPE_ATTRIBUTES:
        raise ValueError(f"unsupported collision shape for {link_name}: {shape}")
    attributes = {
        name: format_vector(primitive[name]) if name == "size" else format_number(float(primitive[name]))
        for name in SHAPE_ATTRIBUTES[shape]
    }
    ET.SubElement(geometry, shape, attributes)
    return collision


def build_joint_limits(config: dict[str, Any]) -> dict[str, tuple[float, float]]:
    actuator = config["actuator"]
    peak_torque = float(actuator["peak_torque_nm"])
    no_load_speed = float(actuator["no_load_speed_rpm"]) * 2.0 * math.pi / 60.0
    reductions = config["additional_reduction"]
    limits = {}
    for joint_name in config["robot"]["joint_order"]:
        joint_type = joint_name.removesuffix("_joint").rsplit("_", maxsplit=1)[-1]
        reduction = float(reductions[joint_type])
        limits[joint_name] = (peak_torque * reduction, no_load_speed / reduction)
    return limits


def normalize(config_path: Path, output_dir: Path, *, check: bool = False) -> Path:
    config_path = config_path.resolve()
    output_dir = output_dir.resolve()
    config = load_config(config_path)
    robot_config = config["robot"]
    source_root = config_path.parent
    source_urdf = source_root / robot_config["source_urdf"]
    source_mesh_dir = source_root / "raw/meshes"

    tree = ET.parse(source_urdf)
    robot = tree.getroot()
    if robot.tag != "robot" or robot.get("name") != robot_config["source_robot_name"]:
        raise ValueError("source URDF robot name does not match model config")
    robot.set("name", robot_config["name"])

    actuated_joints = []
    for joint in robot.findall("joint"):
        joint_name = joint.get("name")
        if joint_name in config["normalization"]["joint_renames"]:
            joint.set("name", config["normalization"]["joint_renames"][joint_name])
        if joint.get("type") != "fixed":
            actuated_joints.append(joint)

    limits = build_joint_limits(config)
    if [joint.get("name") for joint in actuated_joints] != robot_config["joint_order"]:
        raise ValueError("actuated joint order in source URDF does not match model config")

    for joint in actuated_joints:
        joint_name = joint.get("name")
        limit = joint.find("limit")
        if joint_name is None or limit is None:
            raise ValueError("actuated joint is missing a name or limit")
        effort, velocity = limits[joint_name]
        limit.set("effort", format_number(effort))
        limit.set("velocity", format_number(velocity))

    links = robot.findall("link")
    for link in links:
        link_name = link.get("name")
        collision = link.find("collision")
        if link_name is None or collision is None:
            raise ValueError("link is missing a name or collision")
        link.remove(collision)
        link.append(build_collision(link_name, config))

    referenced_meshes: set[str] = set()
    for mesh in robot.findall(".//mesh"):
        source_uri = mesh.get("filename")
        if source_uri is None:
            raise ValueError("mesh is missing filename")
        source_name = Path(source_uri).name
        source_mesh = source_mesh_dir / source_name
        if not source_mesh.is_file():
            raise FileNotFoundError(source_mesh)
        referenced_meshes.add(source_name)
        relative_mesh = Path(os.path.relpath(source_mesh, output_dir)).as_posix()
        mesh.set("filename", relative_mesh)

    ET.indent(tree, space="  ")
    output_urdf = output_dir / f"{robot_config['name']}.urdf"
    output_bytes = ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=True)
    if check:
        if not output_urdf.is_file() or output_urdf.read_bytes() != output_bytes:
            raise RuntimeError(f"normalized URDF is missing or stale: {output_urdf}")
    else:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_urdf.write_bytes(output_bytes)

    print(f"urdf={output_urdf}")
    print(f"status={'current' if check else 'generated'}")
    print(f"actuated_joints={len(actuated_joints)}")
    print(f"primitive_collisions={len(links)}")
    print(f"unique_meshes={len(referenced_meshes)}")
    return output_urdf


def main() -> None:
    args = parse_args()
    normalize(args.config, args.output, check=args.check)


if __name__ == "__main__":
    main()
