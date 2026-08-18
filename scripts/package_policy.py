#!/usr/bin/env python3
"""Create a validated deployment policy bundle from an exported ONNX policy."""

from __future__ import annotations

import argparse
import hashlib
import shutil
import tomllib
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_CONFIG = REPO_ROOT / "assets/sh_dog/model.toml"


def _yaml_sequence(values: list[object]) -> str:
    def render(value: object) -> str:
        if isinstance(value, bool):
            return str(value).lower()
        if isinstance(value, str):
            return f'"{value}"'
        return str(value)

    return "[" + ", ".join(render(value) for value in values) + "]"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", type=Path, help="ONNX file exported by Isaac Lab/RSL-RL.")
    parser.add_argument("output", type=Path, help="New or empty policy-bundle directory.")
    parser.add_argument("--recurrent", choices=("none", "gru"), required=True)
    parser.add_argument("--hidden-size", type=int, default=0)
    parser.add_argument("--num-layers", type=int, default=1)
    args = parser.parse_args()
    if not args.policy.is_file():
        parser.error(f"policy does not exist: {args.policy}")
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error(f"output must be a new or empty directory: {args.output}")
    if (args.recurrent == "gru") != (args.hidden_size > 0):
        parser.error("GRU requires --hidden-size > 0; non-recurrent requires the default 0")

    config = tomllib.loads(MODEL_CONFIG.read_text(encoding="utf-8"))
    joints = config["robot"]["joint_order"]
    default_q = config["default_state"]["joint_positions_rad"]
    kp = [25.0 if "_abad_" in name else 30.0 if "_hip_" in name else 40.0 for name in joints]
    kd = [1.0 if "_abad_" in name else 1.2 if "_hip_" in name else 2.0 for name in joints]
    stand_kp = [40.0 if "_abad_" in name else 60.0 if "_hip_" in name else 80.0 for name in joints]
    stand_kd = [1.5 if "_abad_" in name else 2.0 if "_hip_" in name else 3.0 for name in joints]
    recurrent_tensors = (
        f"    h_in: [{args.num_layers}, 1, {args.hidden_size}]\n"
        if args.recurrent == "gru" else ""
    )
    recurrent_outputs = (
        f"    h_out: [{args.num_layers}, 1, {args.hidden_size}]\n"
        if args.recurrent == "gru" else ""
    )
    manifest = f'''schema_version: 1
model:
  format: onnx
  file: policy.onnx
  inputs:
    obs: [1, 45]
{recurrent_tensors}  outputs:
    actions: [1, 12]
{recurrent_outputs}  recurrent: {str(args.recurrent == "gru").lower()}
robot:
  name: {config["robot"]["name"]}
  joint_order: {_yaml_sequence(joints)}
observation:
  dimension: 45
  history_length: 1
  corruption: false
  terms:
    - {{name: base_ang_vel, size: 3, clip: [-100.0, 100.0], scale: 0.2}}
    - {{name: projected_gravity, size: 3, clip: [-100.0, 100.0], scale: 1.0}}
    - {{name: velocity_command, size: 3, clip: [-100.0, 100.0], scale: 1.0}}
    - {{name: joint_position_error, size: 12, clip: [-100.0, 100.0], scale: 1.0}}
    - {{name: joint_velocity, size: 12, clip: [-100.0, 100.0], scale: 0.05}}
    - {{name: last_action, size: 12, clip: [-100.0, 100.0], scale: 1.0}}
action:
  dimension: 12
  scale: {_yaml_sequence([0.25] * 12)}
  clip: null
  default_joint_positions: {_yaml_sequence(default_q)}
control:
  period_s: 0.02
  motion:
    kp: {_yaml_sequence(kp)}
    kd: {_yaml_sequence(kd)}
  stand:
    kp: {_yaml_sequence(stand_kp)}
    kd: {_yaml_sequence(stand_kd)}
coordinates:
  frame: right_handed_x_forward_y_left_z_up
  quaternion: wxyz_base_to_world
  joint_units: rad_rad_per_s_nm
'''
    args.output.mkdir(parents=True, exist_ok=True)
    policy_output = args.output / "policy.onnx"
    manifest_output = args.output / "manifest.yaml"
    shutil.copyfile(args.policy, policy_output)
    manifest_output.write_text(manifest, encoding="utf-8")
    checksum_lines = []
    for path in (policy_output, manifest_output):
        checksum_lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n")
    (args.output / "checksum.sha256").write_text("".join(checksum_lines), encoding="ascii")
    print(args.output)


if __name__ == "__main__":
    main()
