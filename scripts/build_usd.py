#!/usr/bin/env python3

"""Build the local Isaac Sim USD asset from the normalized ShDog URDF."""

from pathlib import Path

from isaaclab.app import AppLauncher


REPO_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = REPO_ROOT / "assets/sh_dog/urdf/sh_dog.urdf"
USD_DIR = REPO_ROOT / "assets/sh_dog/usd"

simulation_app = AppLauncher(headless=True).app

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402


def main() -> None:
    USD_DIR.mkdir(parents=True, exist_ok=True)
    print(f"usd={USD_DIR / 'sh_dog.usd'}", flush=True)
    UrdfConverter(
        UrdfConverterCfg(
            asset_path=str(URDF_PATH),
            usd_dir=str(USD_DIR),
            usd_file_name="sh_dog.usd",
            force_usd_conversion=False,
            fix_base=False,
            merge_fixed_joints=False,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                target_type="none",
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
            ),
        )
    )


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
