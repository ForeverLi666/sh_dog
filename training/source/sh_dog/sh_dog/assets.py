"""ShDog robot configuration for Isaac Lab."""

import math
import tomllib
from pathlib import Path

import isaaclab.sim as sim_utils

# from isaaclab.actuators import DCMotorCfg
from isaaclab.assets import ArticulationCfg

from sh_dog.actuators import ShDogDcMotorCfg

REPO_ROOT = Path(__file__).resolve().parents[4]
MODEL_CFG = tomllib.loads((REPO_ROOT / "assets/sh_dog/model.toml").read_text(encoding="utf-8"))
USD_PATH = REPO_ROOT / "assets/sh_dog/usd/sh_dog.usd"

JOINT_ORDER = MODEL_CFG["robot"]["joint_order"]
DEFAULT_Q = dict(zip(JOINT_ORDER, MODEL_CFG["default_state"]["joint_positions_rad"], strict=True))
ACTUATOR = MODEL_CFG["actuator"]
REDUCTION = MODEL_CFG["additional_reduction"]


def joint_side(value: float, joint_type: str, *, squared: bool = False) -> float:
    reduction = REDUCTION[joint_type]
    return value * reduction ** (2 if squared else 1)


def sh_dog_dc_motor_cfg(
    joint_names: list[str],
    joint_type: str,
    stiffness: float | dict[str, float],
    damping: float | dict[str, float],
) -> ShDogDcMotorCfg:
    """Build one RS06 actuator group from the shared motor-side facts."""
    reduction = REDUCTION[joint_type]
    torque_speed = ACTUATOR["torque_speed"]
    overload = ACTUATOR["overload"]
    torque_factor = torque_speed["torque_safety_factor"]
    max_speed_rpm = [0.0, *torque_speed["max_speed_rpm"], ACTUATOR["no_load_speed_rpm"]]
    max_torque_nm = [
        ACTUATOR["peak_torque_nm"],
        *torque_speed["max_torque_nm"],
        0.0,
    ]
    return ShDogDcMotorCfg(
        joint_names_expr=joint_names,
        effort_limit=joint_side(ACTUATOR["peak_torque_nm"] * torque_factor, joint_type),
        velocity_limit=ACTUATOR["no_load_speed_rpm"] * 2.0 * math.pi / 60.0 / reduction,
        stiffness=stiffness,
        damping=damping,
        armature=joint_side(ACTUATOR["low_speed_equivalent_inertia_kg_m2"], joint_type, squared=True),
        friction=0.0,
        max_speed_rpm=[speed / reduction for speed in max_speed_rpm],
        max_torque_nm=[joint_side(torque * torque_factor, joint_type) for torque in max_torque_nm],
        thermal_speed_rpm=[0.0, *[speed / reduction for speed in torque_speed["thermal_speed_rpm"]]],
        thermal_torque_nm=[
            joint_side(overload["stall_rated_torque_nm"] * torque_factor, joint_type),
            *[joint_side(torque * torque_factor, joint_type) for torque in torque_speed["thermal_torque_nm"]],
        ],
        rotating_torque_nm=[
            joint_side(torque * torque_factor, joint_type) for torque in overload["rotating_torque_nm"]
        ],
        rotating_time_s=[time * overload["time_safety_factor"] for time in overload["rotating_time_s"]],
        stall_torque_nm=[joint_side(torque * torque_factor, joint_type) for torque in overload["stall_torque_nm"]],
        stall_time_s=[time * overload["time_safety_factor"] for time in overload["stall_time_s"]],
        stall_continuous_torque_nm=joint_side(
            overload["stall_rated_torque_nm"] * torque_factor, joint_type
        ),
        stall_blend_speed_rpm=overload["stall_blend_speed_rpm"] / reduction,
        recovery_time_s=overload["recovery_time_s"],
        initial_budget_range=tuple(overload["initial_budget_range"]),
        release_budget=overload["release_budget"],
    )


SH_DOG_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=2,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, MODEL_CFG["default_state"]["base_height_m"]),
        joint_pos=DEFAULT_Q,
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=1.0,
    # Previous linear DCMotorCfg baseline, kept for later stair-torque comparison.
    # actuators={
    #     "abad_hip": DCMotorCfg(
    #         joint_names_expr=[".*_abad_joint", ".*_hip_joint"],
    #         effort_limit=ACTUATOR["peak_torque_nm"],
    #         saturation_effort=ACTUATOR["peak_torque_nm"],
    #         velocity_limit=ACTUATOR["no_load_speed_rpm"] * 2.0 * math.pi / 60.0,
    #         stiffness={".*_abad_joint": 25.0, ".*_hip_joint": 30.0},
    #         damping={".*_abad_joint": 1.0, ".*_hip_joint": 1.2},
    #         armature=ACTUATOR["low_speed_equivalent_inertia_kg_m2"],
    #         friction=0.0,
    #     ),
    #     "knee": DCMotorCfg(
    #         joint_names_expr=[".*_knee_joint"],
    #         effort_limit=joint_side(ACTUATOR["peak_torque_nm"], "knee"),
    #         saturation_effort=joint_side(ACTUATOR["peak_torque_nm"], "knee"),
    #         velocity_limit=ACTUATOR["no_load_speed_rpm"] * 2.0 * math.pi / 60.0 / REDUCTION["knee"],
    #         stiffness=40.0,
    #         damping=2.0,
    #         armature=joint_side(ACTUATOR["low_speed_equivalent_inertia_kg_m2"], "knee", squared=True),
    #         friction=0.0,
    #     ),
    # },
    actuators={
        "abad_hip": sh_dog_dc_motor_cfg(
            [".*_abad_joint", ".*_hip_joint"],
            "hip",
            {".*_abad_joint": 25.0, ".*_hip_joint": 30.0},
            {".*_abad_joint": 1.0, ".*_hip_joint": 1.2},
        ),
        "knee": sh_dog_dc_motor_cfg([".*_knee_joint"], "knee", 40.0, 2.0),
    },
)
"""ShDog locomotion configuration with compliant joint-side PD gains."""

SH_DOG_STAND_CFG = SH_DOG_CFG.copy()
for actuator_cfg in SH_DOG_STAND_CFG.actuators.values():
    actuator_cfg.initial_budget_range = (1.0, 1.0)
SH_DOG_STAND_CFG.actuators["abad_hip"].stiffness = {".*_abad_joint": 40.0, ".*_hip_joint": 60.0}
SH_DOG_STAND_CFG.actuators["abad_hip"].damping = {".*_abad_joint": 1.5, ".*_hip_joint": 2.0}
SH_DOG_STAND_CFG.actuators["knee"].stiffness = 80.0
SH_DOG_STAND_CFG.actuators["knee"].damping = 3.0

"""ShDog pure-position stand-up configuration."""
