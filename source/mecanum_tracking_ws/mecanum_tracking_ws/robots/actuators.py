"""Wheel motor model of the comparison plant: the XM430-W210-T driven in torque (current) mode.

Isaac Lab's :class:`~isaaclab.actuators.DCMotor` clips the command into the four-quadrant band of the linear
torque-speed curve, [tau_s (-1 - w/w0), tau_s (1 - w/w0)] and |tau| <= effort_limit. Above the no-load speed w0 that
band lies entirely on the braking side, so the motor applies up to -effort_limit whatever the command; and wherever
the curve is active it acts as an explicit damper of slope tau_s / w0 = 0.37 Nm s/rad. With our wheel inertia
(J1 = 1.85e-5 kg m^2, no armature) and 1/360 s physics steps that is ~56x the explicit stability limit
(d dt / J < 2): a wheel that once passes w0 (e.g. in the air at the reset, 2026-10-23: torques ~0.08 Nm) ends in a
+-160 rad/s bang-bang between +-1.95 Nm.

:class:`XM430Motor` keeps the curve only as a LIMIT of the commanded torque: tau = clip(command, lo, hi) with
hi = clip(tau_s (1 - w/w0), 0, effort_limit) and lo = clip(tau_s (-1 - w/w0), -effort_limit, 0). Inside the working
range (|w| < w0, torque toward the motion below the curve) it is the same as DCMotor; it never reverses the command
(no regenerative braking beyond the command), so its output is always between 0 and the command and cannot run away.
"""

from __future__ import annotations

import torch

from isaaclab.actuators import DCMotor, DCMotorCfg
from isaaclab.utils import configclass


class XM430Motor(DCMotor):
    """DC motor whose torque-speed curve limits, but never reverses, the commanded torque (see the module)."""

    def _clip_effort(self, effort: torch.Tensor) -> torch.Tensor:
        w = self._joint_vel / self.velocity_limit
        max_effort = torch.clip(self._saturation_effort * (1.0 - w), min=0.0)
        min_effort = torch.clip(self._saturation_effort * (-1.0 - w), max=0.0)
        max_effort = torch.minimum(max_effort, self.effort_limit)
        min_effort = torch.maximum(min_effort, -self.effort_limit)
        return torch.clip(effort, min=min_effort, max=max_effort)


@configclass
class XM430MotorCfg(DCMotorCfg):
    """Configuration of :class:`XM430Motor` (same fields as DCMotorCfg)."""

    class_type: type = XM430Motor
