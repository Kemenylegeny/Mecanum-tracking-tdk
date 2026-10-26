"""Termination terms for mecanum trajectory tracking (node 5)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from .traj_utils import X_COL, Y_COL

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .trajectory_command import TrajectoryCommand


def tracking_deviation(
    env: ManagerBasedRLEnv, command_name: str, max_error: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """True where the position error exceeds ``max_error`` [m]. A real failure, NOT a time-out (time_out=False).

    Keep max_error well above the initial typical error and above the coarse kernel std (e.g. 1 m = 2 * 0.5 m),
    otherwise episodes end before the policy can learn to come back. Shape (num_envs,), bool.
    """
    term: TrajectoryCommand = env.command_manager.get_term(command_name)
    robot: Articulation = env.scene[asset_cfg.name]
    # after the physics step, before the clock advances: compare with ref_next (see TrajectoryCommand)
    err = torch.linalg.norm(term.ref_next[:, [X_COL, Y_COL]] - robot.data.root_pos_w[:, :2], dim=1)
    return err > max_error
