# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Evaluate an RSL-RL checkpoint with a resolved velocity-command protocol."""

import argparse
import sys
from datetime import datetime
from pathlib import Path

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip


parser = argparse.ArgumentParser(description="Evaluate an RSL-RL velocity policy.")
parser.add_argument(
    "--task",
    type=str,
    default="ShDog-Velocity-Flat-Play",
    choices=("ShDog-Velocity-Flat-Play", "ShDog-Velocity-Flat"),
    help="Nominal Play task or randomized training task.",
)
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="RSL-RL agent configuration entry point."
)
parser.add_argument("--protocol", type=str, default=None, help="Resolved protocol YAML to replay.")
parser.add_argument("--output-dir", type=str, default=None, help="Evaluation output directory.")
parser.add_argument("--repeats", type=int, default=None, help="Episodes per command case (default: 8).")
parser.add_argument("--warmup-s", type=float, default=None, help="Zero-command warmup (default: 2.0).")
parser.add_argument("--command-s", type=float, default=None, help="Target-command duration (default: 8.0).")
parser.add_argument("--recovery-s", type=float, default=None, help="Zero-command recovery (default: 2.0).")
parser.add_argument("--seed", type=int, default=None, help="Evaluation seed (default: 42).")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

if args_cli.checkpoint is None:
    parser.error("--checkpoint is required")
if args_cli.protocol is not None and any(
    value is not None
    for value in (args_cli.repeats, args_cli.warmup_s, args_cli.command_s, args_cli.recovery_s, args_cli.seed)
):
    parser.error("protocol timing, repeats, and seed cannot be overridden when --protocol is used")

original_argv = sys.argv.copy()
sys.argv = [sys.argv[0]] + hydra_args

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import sh_dog.tasks  # noqa: F401
import torch
from rsl_rl.runners import OnPolicyRunner
from sh_dog.evaluation import (
    VelocityEvaluation,
    checkpoint_metadata,
    generate_protocol,
    load_protocol,
    write_results,
)

from isaaclab.utils.assets import retrieve_file_path

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg, agent_cfg: RslRlBaseRunnerCfg) -> None:
    """Run the resolved evaluation protocol."""
    step_dt = float(env_cfg.sim.dt * env_cfg.decimation)
    protocol = (
        load_protocol(Path(args_cli.protocol), args_cli.task, step_dt)
        if args_cli.protocol
        else generate_protocol(
            env_cfg,
            args_cli.task,
            repeats=args_cli.repeats if args_cli.repeats is not None else 8,
            seed=args_cli.seed if args_cli.seed is not None else 42,
            warmup_s=args_cli.warmup_s if args_cli.warmup_s is not None else 2.0,
            command_s=args_cli.command_s if args_cli.command_s is not None else 8.0,
            recovery_s=args_cli.recovery_s if args_cli.recovery_s is not None else 2.0,
        )
    )
    total_steps = sum(protocol["timing"][key] for key in ("warmup_steps", "command_steps", "recovery_steps"))
    num_envs = len(protocol["cases"]) * protocol["repeats"]

    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    agent_cfg.seed = protocol["seed"]
    env_cfg.seed = protocol["seed"]
    env_cfg.scene.num_envs = num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device
    env_cfg.episode_length_s = (total_steps + 1) * protocol["step_dt"]
    env_cfg.curriculum.lin_vel_cmd_levels = None
    env_cfg.commands.base_velocity.rel_standing_envs = 0.0
    env_cfg.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)
    env_cfg.commands.base_velocity.ranges.lin_vel_x = (0.0, 0.0)
    env_cfg.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
    env_cfg.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)

    checkpoint = Path(retrieve_file_path(args_cli.checkpoint)).resolve()
    task_slug = args_cli.task.replace(":", "_").replace("/", "_")
    output_dir = (
        Path(args_cli.output_dir).resolve()
        if args_cli.output_dir
        else checkpoint.parent / "eval" / checkpoint.stem / task_slug / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    )

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(str(checkpoint))
    policy = runner.get_inference_policy(device=env.unwrapped.device)
    policy_nn = runner.alg.policy if hasattr(runner.alg, "policy") else runner.alg.actor_critic
    evaluation = VelocityEvaluation(env.unwrapped, env.num_actions, protocol)
    evaluation.apply_commands(0)
    obs = env.get_observations()

    with torch.inference_mode():
        for step in range(total_steps):
            actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            policy_nn.reset(dones)
            evaluation.record_step(step, actions, dones)
            if step + 1 < total_steps:
                evaluation.apply_commands(step + 1)
                obs = env.get_observations()

    rows = evaluation.rows()
    joint_rows = evaluation.joint_torque_rows()
    metadata = checkpoint_metadata(checkpoint, args_cli.task, original_argv, str(env.unwrapped.device), num_envs)
    write_results(output_dir, protocol, metadata, rows, joint_rows)
    print(f"[INFO] Evaluation written to: {output_dir}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
