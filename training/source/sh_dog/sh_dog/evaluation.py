"""Flat velocity-policy evaluation protocol and metrics."""

import csv
import hashlib
import json
import math
import subprocess
from itertools import product
from pathlib import Path

import torch
import yaml

PROTOCOL_VERSION = 1
COMMAND_AXES = ("lin_vel_x", "lin_vel_y", "ang_vel_z")


def _resolved_ranges(command_cfg) -> dict[str, list[float]]:
    ranges_cfg = getattr(command_cfg, "limit_ranges", None) or command_cfg.ranges
    ranges = {}
    for axis in COMMAND_AXES:
        lower, upper = getattr(ranges_cfg, axis)
        lower = float(lower)
        upper = float(upper)
        if lower > 0.0 or upper < 0.0 or lower > upper:
            raise ValueError(f"command range must contain zero: {axis}=({lower}, {upper})")
        ranges[axis] = [lower, upper]
    return ranges


def _scale_command(ranges: dict[str, list[float]], normalized: tuple[float, float, float]) -> list[float]:
    command = []
    for axis, value in zip(COMMAND_AXES, normalized):
        lower, upper = ranges[axis]
        command.append(abs(value) * lower if value < 0.0 else value * upper)
    return command


def _generate_cases(ranges: dict[str, list[float]]) -> list[dict]:
    cases = [{"id": "stand", "normalized": [0.0, 0.0, 0.0], "command": [0.0, 0.0, 0.0]}]
    for axis_index, axis in enumerate(COMMAND_AXES):
        lower, upper = ranges[axis]
        for fraction, label in ((-1.0, "neg_100"), (-0.5, "neg_50"), (0.5, "pos_50"), (1.0, "pos_100")):
            if (fraction < 0.0 and lower == 0.0) or (fraction > 0.0 and upper == 0.0):
                continue
            normalized = [0.0, 0.0, 0.0]
            normalized[axis_index] = fraction
            cases.append(
                {
                    "id": f"axis_{axis}_{label}",
                    "normalized": normalized,
                    "command": _scale_command(ranges, tuple(normalized)),
                }
            )

    if all(lower < 0.0 < upper for lower, upper in ranges.values()):
        for signs in product((-0.5, 0.5), repeat=3):
            label = "_".join("neg_50" if value < 0.0 else "pos_50" for value in signs)
            cases.append(
                {
                    "id": f"combined_{label}",
                    "normalized": list(signs),
                    "command": _scale_command(ranges, signs),
                }
            )
    return cases


def _steps(seconds: float, step_dt: float, name: str) -> int:
    if seconds <= 0.0:
        raise ValueError(f"{name} must be positive")
    steps = round(seconds / step_dt)
    if not math.isclose(steps * step_dt, seconds, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError(f"{name} must be divisible by the environment step_dt ({step_dt})")
    return steps


def generate_protocol(
    env_cfg,
    task: str,
    repeats: int = 8,
    seed: int = 42,
    warmup_s: float = 2.0,
    command_s: float = 8.0,
    recovery_s: float = 2.0,
) -> dict:
    """Resolve normalized command cases against the task command envelope."""
    if repeats <= 0:
        raise ValueError("repeats must be positive")
    step_dt = float(env_cfg.sim.dt * env_cfg.decimation)
    timing = {
        "warmup_s": warmup_s,
        "command_s": command_s,
        "recovery_s": recovery_s,
        "warmup_steps": _steps(warmup_s, step_dt, "warmup-s"),
        "command_steps": _steps(command_s, step_dt, "command-s"),
        "recovery_steps": _steps(recovery_s, step_dt, "recovery-s"),
    }
    ranges = _resolved_ranges(env_cfg.commands.base_velocity)
    return {
        "version": PROTOCOL_VERSION,
        "task": task,
        "step_dt": step_dt,
        "seed": seed,
        "repeats": repeats,
        "command_source": "limit_ranges" if hasattr(env_cfg.commands.base_velocity, "limit_ranges") else "ranges",
        "command_ranges": ranges,
        "timing": timing,
        "cases": _generate_cases(ranges),
    }


def load_protocol(path: Path, task: str, step_dt: float) -> dict:
    """Load and validate a previously resolved protocol."""
    with path.open(encoding="utf-8") as file:
        protocol = yaml.safe_load(file)
    if not isinstance(protocol, dict):
        raise ValueError("protocol root must be a mapping")
    if protocol.get("version") != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version: {protocol.get('version')}")
    if protocol.get("task") != task:
        raise ValueError(f"protocol task is {protocol.get('task')}, requested task is {task}")
    if not math.isclose(float(protocol.get("step_dt", -1.0)), step_dt, rel_tol=0.0, abs_tol=1.0e-12):
        raise ValueError(f"protocol step_dt is {protocol.get('step_dt')}, task step_dt is {step_dt}")
    if not isinstance(protocol.get("repeats"), int) or protocol["repeats"] <= 0:
        raise ValueError("protocol repeats must be a positive integer")
    if not isinstance(protocol.get("seed"), int):
        raise ValueError("protocol seed must be an integer")

    case_ids = set()
    for case in protocol.get("cases", []):
        if not isinstance(case.get("id"), str) or case["id"] in case_ids:
            raise ValueError("protocol case ids must be unique strings")
        if not isinstance(case.get("command"), list) or len(case["command"]) != 3:
            raise ValueError(f"protocol command must contain three values: {case}")
        case["command"] = [float(value) for value in case["command"]]
        case_ids.add(case["id"])
    if not case_ids:
        raise ValueError("protocol must contain at least one case")
    for key in ("warmup_steps", "command_steps", "recovery_steps"):
        if not isinstance(protocol.get("timing", {}).get(key), int) or protocol["timing"][key] <= 0:
            raise ValueError(f"protocol timing.{key} must be a positive integer")
    return protocol


def protocol_phase(step: int, timing: dict) -> str:
    if step < timing["warmup_steps"]:
        return "warmup"
    if step < timing["warmup_steps"] + timing["command_steps"]:
        return "command"
    return "recovery"


def _value(total: torch.Tensor, count: torch.Tensor, index: int) -> float | None:
    denominator = float(count[index].item())
    return float(total[index].item() / denominator) if denominator > 0.0 else None


def _root(value: float | None) -> float | None:
    return math.sqrt(max(value, 0.0)) if value is not None else None


class VelocityEvaluation:
    """Apply resolved commands and accumulate one evaluation episode per environment."""

    def __init__(self, env, num_actions: int, protocol: dict):
        self.env = env
        self.protocol = protocol
        self.num_envs = len(protocol["cases"]) * protocol["repeats"]
        self.robot = env.scene["robot"]
        self.joint_names = self.robot.joint_names
        self.effort_limits = torch.zeros(len(self.joint_names), device=env.device)
        for actuator in self.robot.actuators.values():
            self.effort_limits[actuator.joint_indices] = actuator.effort_limit[0]
        if torch.any(self.effort_limits <= 0.0):
            raise ValueError("all evaluated joints must have a positive actuator effort limit")
        self.contact_sensor = env.scene.sensors["contact_forces"]
        self.robot_foot_ids, _ = self.robot.find_bodies(".*_foot_link")
        self.sensor_foot_ids, _ = self.contact_sensor.find_bodies(".*_foot_link")
        self.joint_limits = self.robot.data.soft_joint_pos_limits
        self.command_term = env.command_manager.get_term("base_velocity")
        case_commands = torch.tensor([case["command"] for case in protocol["cases"]], device=env.device)
        self.target_commands = case_commands.repeat_interleave(protocol["repeats"], dim=0)
        self.zero_commands = torch.zeros_like(self.target_commands)
        self.active = torch.ones(self.num_envs, dtype=torch.bool, device=env.device)
        self.previous_actions = torch.zeros((self.num_envs, num_actions), device=env.device)
        self.steps_alive = torch.zeros(self.num_envs, device=env.device)
        self.sample_count = torch.zeros(self.num_envs, device=env.device)
        self.target_count = torch.zeros(self.num_envs, device=env.device)
        self.recovery_count = torch.zeros(self.num_envs, device=env.device)
        self.track_abs = torch.zeros((self.num_envs, 3), device=env.device)
        self.track_sq = torch.zeros_like(self.track_abs)
        self.track_signed = torch.zeros_like(self.track_abs)
        self.recovery_abs = torch.zeros_like(self.track_abs)
        self.tilt_sq = torch.zeros(self.num_envs, device=env.device)
        self.tilt_max = torch.zeros(self.num_envs, device=env.device)
        self.height_sum = torch.zeros(self.num_envs, device=env.device)
        self.height_sq = torch.zeros(self.num_envs, device=env.device)
        self.action_rate_sq = torch.zeros(self.num_envs, device=env.device)
        self.action_abs_max = torch.zeros(self.num_envs, device=env.device)
        self.torque_sq = torch.zeros(self.num_envs, device=env.device)
        self.torque_abs_max = torch.zeros(self.num_envs, device=env.device)
        joint_shape = (self.num_envs, len(self.joint_names))
        self.computed_torque_sq = torch.zeros(joint_shape, device=env.device)
        self.applied_torque_sq = torch.zeros(joint_shape, device=env.device)
        self.computed_torque_abs_max = torch.zeros(joint_shape, device=env.device)
        self.applied_torque_abs_max = torch.zeros(joint_shape, device=env.device)
        self.torque_clip_count = torch.zeros(joint_shape, device=env.device)
        self.torque_clip_streak = torch.zeros(joint_shape, device=env.device)
        self.torque_clip_streak_max = torch.zeros(joint_shape, device=env.device)
        self.effort_limit_count = torch.zeros(joint_shape, device=env.device)
        self.joint_margin_min = torch.full((self.num_envs,), torch.inf, device=env.device)
        self.power_sum = torch.zeros(self.num_envs, device=env.device)
        self.foot_slip_sq = torch.zeros(self.num_envs, device=env.device)
        self.foot_contact_count = torch.zeros(self.num_envs, device=env.device)
        self.foot_sample_count = torch.zeros(self.num_envs, device=env.device)
        self.termination_hits = {
            name: torch.zeros(self.num_envs, dtype=torch.bool, device=env.device)
            for name in env.termination_manager.active_terms
        }

    def apply_commands(self, step: int) -> None:
        commands = (
            self.target_commands if protocol_phase(step, self.protocol["timing"]) == "command" else self.zero_commands
        )
        self.command_term.vel_command_b[:] = commands
        self.command_term.is_standing_env[:] = False
        if hasattr(self.command_term, "is_heading_env"):
            self.command_term.is_heading_env[:] = False

    @staticmethod
    def _masked_add(target: torch.Tensor, values: torch.Tensor, mask: torch.Tensor) -> None:
        target += values * mask.to(values.dtype).view(-1, *([1] * (values.ndim - 1)))

    def record_step(self, step: int, actions: torch.Tensor, dones: torch.Tensor) -> None:
        phase = protocol_phase(step, self.protocol["timing"])
        done_mask = dones.bool()
        alive_before_step = self.active.clone()
        self.steps_alive += alive_before_step
        for name in self.termination_hits:
            self.termination_hits[name] |= alive_before_step & self.env.termination_manager.get_term(name)
        sample_mask = alive_before_step & ~done_mask
        self.sample_count += sample_mask

        measured_velocity = torch.stack(
            (
                self.robot.data.root_lin_vel_b[:, 0],
                self.robot.data.root_lin_vel_b[:, 1],
                self.robot.data.root_ang_vel_b[:, 2],
            ),
            dim=1,
        )
        if phase == "command":
            error = measured_velocity - self.target_commands
            self.target_count += sample_mask
            self._masked_add(self.track_abs, error.abs(), sample_mask)
            self._masked_add(self.track_sq, error.square(), sample_mask)
            self._masked_add(self.track_signed, error, sample_mask)
        elif phase == "recovery":
            self.recovery_count += sample_mask
            self._masked_add(self.recovery_abs, measured_velocity.abs(), sample_mask)

        tilt = torch.acos(torch.clamp(-self.robot.data.projected_gravity_b[:, 2], -1.0, 1.0))
        self.tilt_sq += tilt.square() * sample_mask
        self.tilt_max = torch.maximum(self.tilt_max, torch.where(sample_mask, tilt, torch.zeros_like(tilt)))
        height = self.robot.data.root_pos_w[:, 2]
        self.height_sum += height * sample_mask
        self.height_sq += height.square() * sample_mask
        action_rate = actions - self.previous_actions
        self.action_rate_sq += action_rate.square().mean(dim=1) * sample_mask
        self.action_abs_max = torch.maximum(
            self.action_abs_max,
            torch.where(sample_mask, actions.abs().amax(dim=1), torch.zeros_like(self.action_abs_max)),
        )
        computed_torque = self.robot.data.computed_torque
        applied_torque = self.robot.data.applied_torque
        joint_sample_mask = sample_mask[:, None]
        self.computed_torque_sq += computed_torque.square() * joint_sample_mask
        self.applied_torque_sq += applied_torque.square() * joint_sample_mask
        self.computed_torque_abs_max = torch.maximum(
            self.computed_torque_abs_max,
            torch.where(joint_sample_mask, computed_torque.abs(), torch.zeros_like(computed_torque)),
        )
        self.applied_torque_abs_max = torch.maximum(
            self.applied_torque_abs_max,
            torch.where(joint_sample_mask, applied_torque.abs(), torch.zeros_like(applied_torque)),
        )
        clipped = ~torch.isclose(computed_torque, applied_torque, rtol=1.0e-5, atol=1.0e-5)
        self.torque_clip_count += clipped * joint_sample_mask
        self.torque_clip_streak = torch.where(
            clipped & joint_sample_mask, self.torque_clip_streak + 1, torch.zeros_like(self.torque_clip_streak)
        )
        self.torque_clip_streak_max = torch.maximum(self.torque_clip_streak_max, self.torque_clip_streak)
        near_effort_limit = applied_torque.abs() >= 0.99 * self.effort_limits
        self.effort_limit_count += near_effort_limit * joint_sample_mask
        self.torque_sq += applied_torque.square().mean(dim=1) * sample_mask
        self.torque_abs_max = torch.maximum(
            self.torque_abs_max,
            torch.where(sample_mask, applied_torque.abs().amax(dim=1), torch.zeros_like(self.torque_abs_max)),
        )
        margin = torch.minimum(
            self.robot.data.joint_pos - self.joint_limits[..., 0],
            self.joint_limits[..., 1] - self.robot.data.joint_pos,
        ).amin(dim=1)
        self.joint_margin_min = torch.minimum(
            self.joint_margin_min, torch.where(sample_mask, margin, self.joint_margin_min)
        )
        self.power_sum += (applied_torque * self.robot.data.joint_vel).abs().sum(dim=1) * sample_mask

        contact = torch.linalg.vector_norm(self.contact_sensor.data.net_forces_w[:, self.sensor_foot_ids], dim=-1) > 1.0
        foot_slip = torch.linalg.vector_norm(self.robot.data.body_lin_vel_w[:, self.robot_foot_ids, :2], dim=-1)
        contact_mask = contact & sample_mask[:, None]
        self.foot_slip_sq += (foot_slip.square() * contact_mask).sum(dim=1)
        self.foot_contact_count += contact_mask.sum(dim=1)
        self.foot_sample_count += sample_mask * len(self.sensor_foot_ids)
        self.previous_actions = actions
        self.active &= ~done_mask

    def rows(self) -> list[dict]:
        rows = []
        axis_labels = ("vx", "vy", "wz")
        for env_index in range(self.num_envs):
            case = self.protocol["cases"][env_index // self.protocol["repeats"]]
            target_mae = [_value(self.track_abs[:, i], self.target_count, env_index) for i in range(3)]
            target_mse = [_value(self.track_sq[:, i], self.target_count, env_index) for i in range(3)]
            target_bias = [_value(self.track_signed[:, i], self.target_count, env_index) for i in range(3)]
            recovery_mean = [_value(self.recovery_abs[:, i], self.recovery_count, env_index) for i in range(3)]
            height_mean = _value(self.height_sum, self.sample_count, env_index)
            height_second = _value(self.height_sq, self.sample_count, env_index)
            reasons = [name for name, hits in self.termination_hits.items() if bool(hits[env_index].item())]
            row = {
                "case_id": case["id"],
                "repeat": env_index % self.protocol["repeats"],
                "command_vx": case["command"][0],
                "command_vy": case["command"][1],
                "command_wz": case["command"][2],
                "duration_s": float(self.steps_alive[env_index].item() * self.protocol["step_dt"]),
                "success": not reasons and bool(self.active[env_index].item()),
                "termination_reasons": ",".join(reasons),
            }
            for index, label in enumerate(axis_labels):
                row[f"target_{label}_mae"] = target_mae[index]
                row[f"target_{label}_rmse"] = _root(target_mse[index])
                row[f"target_{label}_bias"] = target_bias[index]
                row[f"recovery_{label}_abs_mean"] = recovery_mean[index]
            tilt_mean_sq = _value(self.tilt_sq, self.sample_count, env_index)
            action_rate_mean_sq = _value(self.action_rate_sq, self.sample_count, env_index)
            torque_mean_sq = _value(self.torque_sq, self.sample_count, env_index)
            slip_mean_sq = _value(self.foot_slip_sq, self.foot_contact_count, env_index)
            row.update(
                {
                    "tilt_rms_rad": _root(tilt_mean_sq),
                    "tilt_max_rad": float(self.tilt_max[env_index].item()),
                    "base_height_mean_m": height_mean,
                    "base_height_std_m": _root(
                        max(height_second - height_mean * height_mean, 0.0)
                        if height_mean is not None and height_second is not None
                        else None
                    ),
                    "action_rate_rms": _root(action_rate_mean_sq),
                    "action_abs_max": float(self.action_abs_max[env_index].item()),
                    "joint_torque_rms_nm": _root(torque_mean_sq),
                    "joint_torque_abs_max_nm": float(self.torque_abs_max[env_index].item()),
                    "joint_limit_margin_min_rad": (
                        float(self.joint_margin_min[env_index].item())
                        if torch.isfinite(self.joint_margin_min[env_index])
                        else None
                    ),
                    "mechanical_power_abs_mean_w": _value(self.power_sum, self.sample_count, env_index),
                    "foot_slip_rms_m_s": _root(slip_mean_sq),
                    "foot_contact_fraction": _value(self.foot_contact_count, self.foot_sample_count, env_index),
                }
            )
            rows.append(row)
        return rows

    def joint_torque_rows(self) -> list[dict]:
        """Return one torque-diagnostic row per evaluation episode and joint."""
        rows = []
        for env_index in range(self.num_envs):
            case = self.protocol["cases"][env_index // self.protocol["repeats"]]
            for joint_index, joint_name in enumerate(self.joint_names):
                rows.append(
                    {
                        "case_id": case["id"],
                        "repeat": env_index % self.protocol["repeats"],
                        "joint_name": joint_name,
                        "effort_limit_nm": float(self.effort_limits[joint_index].item()),
                        "computed_torque_rms_nm": _root(
                            _value(self.computed_torque_sq[:, joint_index], self.sample_count, env_index)
                        ),
                        "applied_torque_rms_nm": _root(
                            _value(self.applied_torque_sq[:, joint_index], self.sample_count, env_index)
                        ),
                        "computed_torque_abs_max_nm": float(
                            self.computed_torque_abs_max[env_index, joint_index].item()
                        ),
                        "applied_torque_abs_max_nm": float(
                            self.applied_torque_abs_max[env_index, joint_index].item()
                        ),
                        "torque_clip_fraction": _value(
                            self.torque_clip_count[:, joint_index], self.sample_count, env_index
                        ),
                        "torque_clip_max_contiguous_s": float(
                            self.torque_clip_streak_max[env_index, joint_index].item() * self.protocol["step_dt"]
                        ),
                        "effort_limit_fraction": _value(
                            self.effort_limit_count[:, joint_index], self.sample_count, env_index
                        ),
                    }
                )
        return rows


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def summarize(rows: list[dict], cases: list[dict]) -> dict:
    """Aggregate episode rows by resolved command case."""
    metric_names = [
        "target_vx_mae",
        "target_vy_mae",
        "target_wz_mae",
        "target_vx_rmse",
        "target_vy_rmse",
        "target_wz_rmse",
        "recovery_vx_abs_mean",
        "recovery_vy_abs_mean",
        "recovery_wz_abs_mean",
        "tilt_rms_rad",
        "tilt_max_rad",
        "base_height_mean_m",
        "base_height_std_m",
        "action_rate_rms",
        "action_abs_max",
        "joint_torque_rms_nm",
        "joint_torque_abs_max_nm",
        "joint_limit_margin_min_rad",
        "mechanical_power_abs_mean_w",
        "foot_slip_rms_m_s",
        "foot_contact_fraction",
    ]
    summary = {"episodes": len(rows), "success_rate": sum(row["success"] for row in rows) / len(rows), "cases": {}}
    for case in cases:
        case_rows = [row for row in rows if row["case_id"] == case["id"]]
        termination_counts = {}
        for row in case_rows:
            for reason in filter(None, row["termination_reasons"].split(",")):
                termination_counts[reason] = termination_counts.get(reason, 0) + 1
        metrics = {}
        for name in metric_names:
            values = [float(row[name]) for row in case_rows if row[name] is not None]
            if values:
                metrics[name] = {
                    "mean": sum(values) / len(values),
                    "p95": _percentile(values, 0.95),
                    "max": max(values),
                }
        summary["cases"][case["id"]] = {
            "command": case["command"],
            "episodes": len(case_rows),
            "success_rate": sum(row["success"] for row in case_rows) / len(case_rows),
            "termination_counts": termination_counts,
            "metrics": metrics,
        }
    return summary


def summarize_joint_torques(rows: list[dict]) -> dict:
    """Aggregate torque diagnostics by joint across all command cases."""
    metric_names = [
        "computed_torque_rms_nm",
        "applied_torque_rms_nm",
        "computed_torque_abs_max_nm",
        "applied_torque_abs_max_nm",
        "torque_clip_fraction",
        "torque_clip_max_contiguous_s",
        "effort_limit_fraction",
    ]
    summary = {}
    for joint_name in dict.fromkeys(row["joint_name"] for row in rows):
        joint_rows = [row for row in rows if row["joint_name"] == joint_name]
        summary[joint_name] = {
            "effort_limit_nm": joint_rows[0]["effort_limit_nm"],
            "metrics": {
                name: {
                    "mean": sum(float(row[name]) for row in joint_rows) / len(joint_rows),
                    "p95": _percentile([float(row[name]) for row in joint_rows], 0.95),
                    "max": max(float(row[name]) for row in joint_rows),
                }
                for name in metric_names
            },
        }
    return summary


def checkpoint_metadata(checkpoint: Path, task: str, command: list[str], device: str, num_envs: int) -> dict:
    """Record checkpoint and evaluation-code provenance."""
    digest = hashlib.sha256()
    with checkpoint.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    repo_root = Path(__file__).resolve().parents[4]
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo_root, check=True, capture_output=True, text=True
    ).stdout.strip()
    status = subprocess.run(
        ["git", "status", "--porcelain"], cwd=repo_root, check=True, capture_output=True, text=True
    ).stdout
    return {
        "task": task,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": digest.hexdigest(),
        "git": {"commit": commit, "dirty": bool(status.strip())},
        "command": command,
        "device": device,
        "num_envs": num_envs,
    }


def write_results(output_dir: Path, protocol: dict, metadata: dict, rows: list[dict], joint_rows: list[dict]) -> None:
    """Write the resolved protocol, provenance, episode rows and summary."""
    output_dir.mkdir(parents=True, exist_ok=False)
    with (output_dir / "protocol.yaml").open("w", encoding="utf-8") as file:
        yaml.safe_dump(protocol, file, sort_keys=False, allow_unicode=True)
    with (output_dir / "metadata.yaml").open("w", encoding="utf-8") as file:
        yaml.safe_dump(metadata, file, sort_keys=False, allow_unicode=True)
    with (output_dir / "episodes.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output_dir / "joint_torques.csv").open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(joint_rows[0]))
        writer.writeheader()
        writer.writerows(joint_rows)
    with (output_dir / "summary.json").open("w", encoding="utf-8") as file:
        summary = summarize(rows, protocol["cases"])
        summary["joints"] = summarize_joint_torques(joint_rows)
        json.dump(summary, file, indent=2, allow_nan=False)
        file.write("\n")
