from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_cmd_levels(env: ManagerBasedRLEnv, env_ids: Sequence[int]) -> torch.Tensor:
    """Expand planar velocity commands after successful tracking."""
    command = env.command_manager.get_term("base_velocity")
    reward_cfg = env.reward_manager.get_term_cfg("track_lin_vel_xy")
    reward = torch.mean(env.reward_manager._episode_sums["track_lin_vel_xy"][env_ids]) / env.max_episode_length_s

    if env.common_step_counter % env.max_episode_length == 0 and reward > reward_cfg.weight * 0.8:
        delta = torch.tensor([-0.1, 0.1], device=env.device)
        command.cfg.ranges.lin_vel_x = torch.clamp(
            torch.tensor(command.cfg.ranges.lin_vel_x, device=env.device) + delta,
            *command.cfg.limit_ranges.lin_vel_x,
        ).tolist()
        command.cfg.ranges.lin_vel_y = torch.clamp(
            torch.tensor(command.cfg.ranges.lin_vel_y, device=env.device) + delta,
            *command.cfg.limit_ranges.lin_vel_y,
        ).tolist()

    return torch.tensor(command.cfg.ranges.lin_vel_x[1], device=env.device)
