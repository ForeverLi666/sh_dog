from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def energy(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize absolute joint mechanical power."""
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.sum(
        torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids])
        * torch.abs(asset.data.applied_torque[:, asset_cfg.joint_ids]),
        dim=-1,
    )


def joint_position_penalty(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    stand_still_scale: float,
    velocity_threshold: float,
) -> torch.Tensor:
    """Penalize deviation from the default pose, especially while standing."""
    asset: Articulation = env.scene[asset_cfg.name]
    command = torch.linalg.norm(env.command_manager.get_command("base_velocity"), dim=1)
    body_velocity = torch.linalg.norm(asset.data.root_lin_vel_b[:, :2], dim=1)
    error = torch.linalg.norm(asset.data.joint_pos - asset.data.default_joint_pos, dim=1)
    return torch.where((command > 0.0) | (body_velocity > velocity_threshold), error, stand_still_scale * error)


def air_time_variance_penalty(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize unequal air and contact times between feet."""
    sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    air_time = torch.clip(sensor.data.last_air_time[:, sensor_cfg.body_ids], max=0.5)
    contact_time = torch.clip(sensor.data.last_contact_time[:, sensor_cfg.body_ids], max=0.5)
    return torch.var(air_time, dim=1) + torch.var(contact_time, dim=1)
