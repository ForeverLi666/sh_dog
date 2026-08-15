"""RS06 actuator model for ShDog training."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import MISSING

import torch

from isaaclab.actuators import IdealPDActuator, IdealPDActuatorCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions

SH_DOG_ACTUATOR_DT = 0.005


class ShDogDcMotor(IdealPDActuator):
    """RS06 PD actuator with measured T-N limits and a shared overload budget."""

    cfg: ShDogDcMotorCfg

    def __init__(self, cfg: ShDogDcMotorCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        self._validate_cfg()
        self._joint_vel = torch.zeros_like(self.computed_effort)
        self._max_speed = torch.tensor(cfg.max_speed_rpm, device=self._device) * (2.0 * torch.pi / 60.0)
        self._max_torque = torch.tensor(cfg.max_torque_nm, device=self._device)
        self._thermal_speed = torch.tensor(cfg.thermal_speed_rpm, device=self._device) * (2.0 * torch.pi / 60.0)
        self._thermal_torque = torch.tensor(cfg.thermal_torque_nm, device=self._device)
        self._rotating_torque = torch.tensor(cfg.rotating_torque_nm, device=self._device)
        self._rotating_time = torch.tensor(cfg.rotating_time_s, device=self._device)
        self._stall_torque = torch.tensor(cfg.stall_torque_nm, device=self._device)
        self._stall_time = torch.tensor(cfg.stall_time_s, device=self._device)
        self._stall_continuous_torque = cfg.stall_continuous_torque_nm
        self._stall_blend_speed = cfg.stall_blend_speed_rpm * (2.0 * torch.pi / 60.0)
        self._overload_budget = torch.empty_like(self.computed_effort)
        self._derated = torch.empty_like(self.computed_effort, dtype=torch.bool)
        self.reset(slice(None))

    @property
    def overload_budget(self) -> torch.Tensor:
        """Remaining normalized overload budget for each environment and joint."""
        return self._overload_budget

    @property
    def is_derated(self) -> torch.Tensor:
        """Whether each actuator is latched to its continuous torque boundary."""
        return self._derated

    def reset(self, env_ids: Sequence[int]):
        budget = torch.empty_like(self._overload_budget[env_ids]).uniform_(*self.cfg.initial_budget_range)
        self._overload_budget[env_ids] = budget
        self._derated[env_ids] = budget <= self.cfg.release_budget

    def compute(
        self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor
    ) -> ArticulationActions:
        self._joint_vel[:] = joint_vel
        return super().compute(control_action, joint_pos, joint_vel)

    def _clip_effort(self, effort: torch.Tensor) -> torch.Tensor:
        speed = torch.abs(self._joint_vel)
        max_torque = self._interpolate(speed, self._max_speed, self._max_torque)
        thermal_torque = torch.minimum(
            self._interpolate(speed, self._thermal_speed, self._thermal_torque), max_torque
        )
        torque_limit = torch.where(self._derated, thermal_torque, max_torque)
        applied = torch.clamp(effort, min=-torque_limit, max=torque_limit)

        torque = torch.abs(applied)
        stall_rate = self._overload_rate(
            torque,
            torch.full_like(torque, self._stall_continuous_torque),
            self._stall_torque,
            self._stall_time,
        )
        rotating_rate = self._overload_rate(torque, thermal_torque, self._rotating_torque, self._rotating_time)
        blend = torch.clamp(speed / self._stall_blend_speed, max=1.0)
        overload_rate = stall_rate + blend * (rotating_rate - stall_rate)
        torque_ratio = torque / torch.clamp(thermal_torque, min=torch.finfo(thermal_torque.dtype).eps)
        recovery_rate = torch.clamp(1.0 - torque_ratio, min=0.0) / self.cfg.recovery_time_s
        self._overload_budget.add_(
            self.cfg.step_dt * torch.where(torque > thermal_torque, -overload_rate, recovery_rate)
        ).clamp_(0.0, 1.0)

        self._derated |= self._overload_budget <= 0.0
        self._derated &= self._overload_budget < self.cfg.release_budget
        return applied

    @staticmethod
    def _interpolate(x: torch.Tensor, points: torch.Tensor, values: torch.Tensor) -> torch.Tensor:
        result = torch.full_like(x, values[0])
        for index in range(1, len(points)):
            fraction = torch.clamp((x - points[index - 1]) / (points[index] - points[index - 1]), 0.0, 1.0)
            interpolated = values[index - 1] + fraction * (values[index] - values[index - 1])
            result = torch.where(x >= points[index - 1], interpolated, result)
        return result

    @staticmethod
    def _overload_rate(
        torque: torch.Tensor,
        continuous_torque: torch.Tensor,
        torque_points: torch.Tensor,
        time_points: torch.Tensor,
    ) -> torch.Tensor:
        rate = torch.zeros_like(torque)
        lower_torque = continuous_torque
        lower_rate = torch.zeros_like(torque)
        for upper_torque, upper_time in zip(torque_points, time_points, strict=True):
            upper_rate = 1.0 / upper_time
            fraction = (torque - lower_torque) / (upper_torque - lower_torque)
            rate = torch.where(
                (torque > lower_torque) & (torque <= upper_torque),
                lower_rate + fraction * (upper_rate - lower_rate),
                rate,
            )
            lower_torque = upper_torque
            lower_rate = upper_rate
        return torch.where(torque > lower_torque, lower_rate, rate)

    def _validate_cfg(self) -> None:
        tables = (
            ("max torque-speed", self.cfg.max_speed_rpm, self.cfg.max_torque_nm),
            ("thermal torque-speed", self.cfg.thermal_speed_rpm, self.cfg.thermal_torque_nm),
            ("rotating overload", self.cfg.rotating_torque_nm, self.cfg.rotating_time_s),
            ("stall overload", self.cfg.stall_torque_nm, self.cfg.stall_time_s),
        )
        for name, x_values, y_values in tables:
            if len(x_values) < 2 or len(x_values) != len(y_values):
                raise ValueError(f"{name} table must contain matching lists with at least two points")
            if any(current <= previous for previous, current in zip(x_values, x_values[1:])):
                raise ValueError(f"{name} input points must be strictly increasing")
            if any(value <= 0.0 for value in y_values[:-1]) or y_values[-1] < 0.0:
                raise ValueError(f"{name} values must be positive except for an optional final zero")
        if self.cfg.step_dt <= 0.0 or self.cfg.recovery_time_s <= 0.0 or self.cfg.stall_blend_speed_rpm <= 0.0:
            raise ValueError("actuator time and speed parameters must be positive")
        if not 0.0 < self.cfg.stall_continuous_torque_nm < self.cfg.stall_torque_nm[0]:
            raise ValueError("stall_continuous_torque_nm must be below the first stall overload point")
        if not 0.0 <= self.cfg.initial_budget_range[0] <= self.cfg.initial_budget_range[1] <= 1.0:
            raise ValueError("initial_budget_range must lie within [0, 1]")
        if not 0.0 < self.cfg.release_budget < 1.0:
            raise ValueError("release_budget must lie within (0, 1)")


@configclass
class ShDogDcMotorCfg(IdealPDActuatorCfg):
    """Configuration for :class:`ShDogDcMotor` in joint-side units."""

    class_type: type = ShDogDcMotor
    step_dt: float = SH_DOG_ACTUATOR_DT
    max_speed_rpm: list[float] = MISSING
    max_torque_nm: list[float] = MISSING
    thermal_speed_rpm: list[float] = MISSING
    thermal_torque_nm: list[float] = MISSING
    rotating_torque_nm: list[float] = MISSING
    rotating_time_s: list[float] = MISSING
    stall_torque_nm: list[float] = MISSING
    stall_time_s: list[float] = MISSING
    stall_continuous_torque_nm: float = MISSING
    stall_blend_speed_rpm: float = MISSING
    recovery_time_s: float = MISSING
    initial_budget_range: tuple[float, float] = MISSING
    release_budget: float = MISSING
