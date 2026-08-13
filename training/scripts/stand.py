#!/usr/bin/env python3

"""Validate the ShDog crouch-to-stand position controller."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--transition-time", type=float, default=0.5, help="Crouch-to-stand duration in seconds.")
parser.add_argument("--settle-time", type=float, default=0.5, help="Crouch hold before standing in seconds.")
parser.add_argument("--hold-time", type=float, default=10.0, help="Stand hold after the transition in seconds.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

if min(args_cli.transition_time, args_cli.hold_time) <= 0.0 or args_cli.settle_time < 0.0:
    parser.error("transition-time and hold-time must be positive; settle-time must be non-negative")

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402
from sh_dog.assets import SH_DOG_STAND_CFG  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

CROUCH_Q = {".*_abad_joint": 0.0, ".*_hip_joint": 1.10, ".*_knee_joint": -2.65}


@configclass
class StandSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/Ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=1000.0),
    )
    robot: ArticulationCfg = SH_DOG_STAND_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.40),
            joint_pos=CROUCH_Q,
            joint_vel={".*": 0.0},
        ),
    )


def smoothstep(value: float) -> float:
    """Fifth-order interpolation with zero endpoint velocity and acceleration."""
    value = min(max(value, 0.0), 1.0)
    return value**3 * (10.0 + value * (-15.0 + 6.0 * value))


def run(sim: SimulationContext, scene: InteractiveScene) -> None:
    robot = scene["robot"]
    dt = sim.get_physics_dt()
    total_time = args_cli.settle_time + args_cli.transition_time + args_cli.hold_time

    crouch = robot.data.default_joint_pos.clone()
    for expression, position in CROUCH_Q.items():
        joint_ids, _ = robot.find_joints(expression)
        crouch[:, joint_ids] = position
    stand = SH_DOG_STAND_CFG.init_state.joint_pos
    stand_q = robot.data.default_joint_pos.clone()
    for expression, position in stand.items():
        joint_ids, _ = robot.find_joints(expression)
        stand_q[:, joint_ids] = position

    step = 0
    print(f"running {total_time:.2f} s stand sequence", flush=True)
    while simulation_app.is_running() and step * dt < total_time:
        elapsed = step * dt
        phase = smoothstep((elapsed - args_cli.settle_time) / args_cli.transition_time)
        target = crouch + phase * (stand_q - crouch)
        robot.set_joint_position_target(target)
        robot.set_joint_velocity_target(torch.zeros_like(target))
        scene.write_data_to_sim()
        sim.step(render=not args_cli.headless)
        scene.update(dt)
        step += 1

    print("stand sequence complete", flush=True)


def main() -> None:
    print("creating simulation", flush=True)
    sim = SimulationContext(sim_utils.SimulationCfg(dt=0.005, render_interval=4, device=args_cli.device))
    sim.set_camera_view((1.8, 1.8, 1.1), (0.0, 0.0, 0.25))
    scene = InteractiveScene(StandSceneCfg(num_envs=1, env_spacing=2.0))
    print("resetting simulation", flush=True)
    sim.reset()
    print("simulation ready", flush=True)
    run(sim, scene)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
