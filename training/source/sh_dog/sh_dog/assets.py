"""ShDog robot configuration for Isaac Lab."""

import math
import tomllib
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets import ArticulationCfg

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
    actuators={
        "abad_hip": DCMotorCfg(
            joint_names_expr=[".*_abad_joint", ".*_hip_joint"],
            effort_limit=ACTUATOR["rated_torque_nm"],
            saturation_effort=ACTUATOR["peak_torque_nm"],
            velocity_limit=ACTUATOR["no_load_speed_rpm"] * 2.0 * math.pi / 60.0,
            stiffness=25.0,
            damping=0.5,
            armature=ACTUATOR["low_speed_equivalent_inertia_kg_m2"],
            friction=0.0,
        ),
        "knee": DCMotorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit=joint_side(ACTUATOR["rated_torque_nm"], "knee"),
            saturation_effort=joint_side(ACTUATOR["peak_torque_nm"], "knee"),
            velocity_limit=ACTUATOR["no_load_speed_rpm"] * 2.0 * math.pi / 60.0 / REDUCTION["knee"],
            stiffness=25.0,
            damping=0.5,
            armature=joint_side(ACTUATOR["low_speed_equivalent_inertia_kg_m2"], "knee", squared=True),
            friction=0.0,
        ),
    },
)
"""ShDog training configuration with joint-side actuator parameters."""

SH_DOG_STAND_CFG = SH_DOG_CFG.copy()
SH_DOG_STAND_CFG.actuators["abad_hip"].stiffness = {".*_abad_joint": 40.0, ".*_hip_joint": 60.0}
SH_DOG_STAND_CFG.actuators["abad_hip"].damping = {".*_abad_joint": 1.5, ".*_hip_joint": 2.0}
SH_DOG_STAND_CFG.actuators["knee"].stiffness = 80.0
SH_DOG_STAND_CFG.actuators["knee"].damping = 3.0
"""ShDog pure-position stand-up configuration."""
