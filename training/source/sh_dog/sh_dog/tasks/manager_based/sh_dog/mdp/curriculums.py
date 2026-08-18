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


def terrain_levels_omnidirectional(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    move_up_lin_threshold: float,
    move_up_yaw_threshold: float,
    move_down_lin_threshold: float,
    move_down_yaw_threshold: float,
) -> torch.Tensor:
    """Adapt terrain difficulty without misclassifying curved or reversing trajectories."""
    terrain = env.scene.terrain
    episode_duration = env.max_episode_length_s
    lin_reward_cfg = env.reward_manager.get_term_cfg("track_lin_vel_xy")
    yaw_reward_cfg = env.reward_manager.get_term_cfg("track_ang_vel_z")
    lin_score = (
        env.reward_manager._episode_sums["track_lin_vel_xy"][env_ids]
        / lin_reward_cfg.weight
        / episode_duration
    )
    yaw_score = (
        env.reward_manager._episode_sums["track_ang_vel_z"][env_ids]
        / yaw_reward_cfg.weight
        / episode_duration
    )
    timed_out = env.termination_manager.time_outs[env_ids]
    failed = env.termination_manager.terminated[env_ids]
    move_up = (
        timed_out
        & ~failed
        & (lin_score > move_up_lin_threshold)
        & (yaw_score > move_up_yaw_threshold)
    )
    move_down = failed | (
        timed_out & ((lin_score < move_down_lin_threshold) | (yaw_score < move_down_yaw_threshold))
    )
    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())
