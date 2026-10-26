"""Wheel-torque action: the policy commands the four wheel torques, the same interface as the MPC
(docs/OSSZEHANGOLAS.md, 3.2).

torque = clip(scale * action, -u_max, u_max) [Nm], held for the whole policy step (zero-order hold, 50 ms), applied as
the effort command of the torque-driven wheel motors (stiffness = damping = 0), which clip it once more with the
XM430 torque-speed curve. The term also keeps the torques of the previous step: the torque-rate penalty (the RL
counterpart of the MPC's S) and the ``last_wheel_torque`` observation (the MPC's u_applied) use the CLIPPED torques,
not the raw policy output.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp.actions.actions_cfg import JointEffortActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointEffortAction
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class WheelTorqueAction(JointEffortAction):
    """Joint effort action that remembers the previous step's (clipped) torques; see :class:`WheelTorqueActionCfg`."""

    cfg: WheelTorqueActionCfg

    def __init__(self, cfg: WheelTorqueActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._prev_processed_actions = torch.zeros_like(self._processed_actions)

    @property
    def prev_processed_actions(self) -> torch.Tensor:
        """(num_envs, 4): the torques of the previous policy step [Nm] (0 after a reset)."""
        return self._prev_processed_actions

    def process_actions(self, actions: torch.Tensor):
        self._prev_processed_actions[:] = self._processed_actions
        super().process_actions(actions)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        ids = slice(None) if env_ids is None else env_ids
        # the robot starts at rest: u_applied = 0, as for the MPC
        self._processed_actions[ids] = 0.0
        self._prev_processed_actions[ids] = 0.0


@configclass
class WheelTorqueActionCfg(JointEffortActionCfg):
    """Configuration of :class:`WheelTorqueAction`. Set ``scale`` [Nm per policy unit] and ``clip`` (+-u_max)."""

    class_type: type = WheelTorqueAction
