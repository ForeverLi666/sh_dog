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
        raise ValueError("unsupported model schema_version")
    return config


def format_number(value: float) -> str:
    return f"{value:.12g}"


def build_joint_limits(config: dict[str, Any]) -> dict[str, tuple[float, float]]:
    actuator = config["actuator"]
    actuator_model = str(actuator["model"]).strip()
    rated_torque = float(actuator["rated_torque_nm"])
    peak_torque = float(actuator["peak_torque_nm"])
    no_load_speed = float(actuator["no_load_speed_rpm"]) * 2.0 * math.pi / 60.0
    if not actuator_model:
        raise ValueError("actuator model must not be empty")
    if not 0.0 < rated_torque <= peak_torque:
        raise ValueError("actuator torque limits are invalid")
    if no_load_speed <= 0.0:
        raise ValueError("actuator no-load speed must be positive")

    limits: dict[str, tuple[float, float]] = {}
    reductions = config["additional_reduction"]
    for joint_name in config["robot"]["joint_order"]:
        joint_type = joint_name.removesuffix("_joint").rsplit("_", maxsplit=1)[-1]
        if joint_type not in reductions:
            raise ValueError(f"missing additional_reduction for {joint_name}")
        reduction = float(reductions[joint_type])
        if reduction <= 0.0:
            raise ValueError(f"additional_reduction must be positive for {joint_type}")
        if joint_name in limits:
            raise ValueError(f"duplicate joint in model config: {joint_name}")
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

    joint_renames = config["normalization"]["joint_renames"]
    for joint in robot.findall("joint"):
        joint_name = joint.get("name")
        if joint_name in joint_renames:
            joint.set("name", joint_renames[joint_name])

    limits = build_joint_limits(config)
    expected_joint_order = robot_config["joint_order"]
    actuated_joints = [joint for joint in robot.findall("joint") if joint.get("type") != "fixed"]
    actual_joint_names = [joint.get("name") for joint in actuated_joints]
    if actual_joint_names != expected_joint_order:
        raise ValueError("actuated joint order in source URDF does not match model config")

    for joint in actuated_joints:
        joint_name = joint.get("name")
        limit = joint.find("limit")
        if joint_name is None or limit is None:
            raise ValueError("actuated joint is missing a name or limit")
        effort, velocity = limits[joint_name]
        limit.set("effort", format_number(effort))
        limit.set("velocity", format_number(velocity))

    mesh_count = 0
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
        mesh_count += 1

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
    print(f"mesh_references={mesh_count}")
    print(f"unique_meshes={len(referenced_meshes)}")
    return output_urdf


def main() -> None:
    args = parse_args()
    normalize(args.config, args.output, check=args.check)


if __name__ == "__main__":
    main()
