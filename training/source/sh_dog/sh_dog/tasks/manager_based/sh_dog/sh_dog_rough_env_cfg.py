# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""ShDog rough-terrain velocity-tracking task."""

import isaaclab.sim as sim_utils
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import RayCasterCfg, patterns
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

from . import mdp
from .sh_dog_flat_env_cfg import CurriculumCfg, ObservationsCfg, SceneCfg, ShDogFlatEnvCfg

SH_DOG_ROUGH_TERRAINS_CFG = ROUGH_TERRAINS_CFG.copy()
SH_DOG_ROUGH_TERRAINS_CFG.sub_terrains["pyramid_stairs"].step_height_range = (0.05, 0.16)
SH_DOG_ROUGH_TERRAINS_CFG.sub_terrains["pyramid_stairs_inv"].step_height_range = (0.05, 0.16)
SH_DOG_ROUGH_TERRAINS_CFG.sub_terrains["boxes"].grid_height_range = (0.05, 0.10)
SH_DOG_ROUGH_TERRAINS_CFG.sub_terrains["random_rough"].noise_range = (0.02, 0.05)
SH_DOG_ROUGH_TERRAINS_CFG.sub_terrains["random_rough"].noise_step = 0.01
SH_DOG_ROUGH_TERRAINS_CFG.sub_terrains["hf_pyramid_slope"].slope_range = (0.0, 0.25)
SH_DOG_ROUGH_TERRAINS_CFG.sub_terrains["hf_pyramid_slope_inv"].slope_range = (0.0, 0.25)


@configclass
class RoughSceneCfg(SceneCfg):
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=SH_DOG_ROUGH_TERRAINS_CFG,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        debug_vis=False,
    )
    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=(1.6, 1.0)),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )


@configclass
class RoughObservationsCfg(ObservationsCfg):
    @configclass
    class PolicyCfg(ObservationsCfg.PolicyCfg):
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            noise=Unoise(n_min=-0.1, n_max=0.1),
            clip=(-1.0, 1.0),
        )

    @configclass
    class CriticCfg(ObservationsCfg.CriticCfg):
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class RoughCurriculumCfg(CurriculumCfg):
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)


@configclass
class ShDogRoughEnvCfg(ShDogFlatEnvCfg):
    scene: RoughSceneCfg = RoughSceneCfg(num_envs=4096, env_spacing=2.5)
    observations: RoughObservationsCfg = RoughObservationsCfg()
    curriculum: RoughCurriculumCfg = RoughCurriculumCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        self.sim.physx.gpu_max_rigid_patch_count = 10 * 2**15
        self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        self.scene.terrain.terrain_generator.curriculum = self.curriculum.terrain_levels is not None


@configclass
class ShDogRoughEnvCfg_PLAY(ShDogRoughEnvCfg):
    def __post_init__(self) -> None:
        super().__post_init__()

        self.scene.num_envs = 1
        self.scene.terrain.max_init_terrain_level = 0
        self.scene.terrain.terrain_generator.curriculum = False
        self.observations.policy.enable_corruption = False
        self.events.physics_material.params["static_friction_range"] = (0.8, 0.8)
        self.events.physics_material.params["dynamic_friction_range"] = (0.6, 0.6)
        self.events.physics_material.params["restitution_range"] = (0.0, 0.0)
        self.events.add_base_mass = None
        self.events.push_robot = None
        self.curriculum.lin_vel_cmd_levels = None
        self.curriculum.terrain_levels = None
        self.commands.base_velocity.rel_standing_envs = 0.0
        self.commands.base_velocity.ranges.lin_vel_x = (0.5, 0.5)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (0.0, 0.0)
        self.events.reset_joints.params["velocity_range"] = (0.0, 0.0)
        self.events.reset_base.params["pose_range"] = {
            "x": (0.0, 0.0),
            "y": (0.0, 0.0),
            "yaw": (0.0, 0.0),
        }
