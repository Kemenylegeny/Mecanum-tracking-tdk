"""Speed curriculum for mecanum trajectory tracking (node 6).

Same pattern as your navigation task's ``terrain_levels_goal``: called BEFORE the reset of the finished envs,
per-env levels move up on success and down on failure (Rudin et al. 2021, Sec. 3.1, game-inspired curriculum).
Here the "level" is the reference time scale s: v_ref * s, lateral acceleration * s^2.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .trajectory_command import TrajectoryCommand


def speed_levels(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str,
    up_error: float,
    scale_range: tuple[float, float] = (0.5, 2.0),
    step: float = 0.1,
    failure_term: str = "deviation",
) -> torch.Tensor:
    """Per-env reference speed curriculum.

    * move up (s += step): the episode was NOT ended by ``failure_term`` and its mean position error < ``up_error``,
    * move down (s -= step): the episode was ended by ``failure_term`` (the robot lost the trajectory),
    * otherwise: stay.
    Without ``failure_term`` (a task that never terminates a lost trajectory) no episode counts as failed, so the
    levels only move up.
    Returns the mean time scale (logged by the curriculum manager).
    """
    term: TrajectoryCommand = env.command_manager.get_term(command_name)
    env_ids = torch.as_tensor(env_ids, device=term.device, dtype=torch.long)
    if env_ids.numel() == 0:
        return torch.mean(term.time_scale)
    if failure_term in env.termination_manager.active_terms:
        failed = env.termination_manager.get_term(failure_term)[env_ids]
    else:
        failed = torch.zeros(env_ids.numel(), dtype=torch.bool, device=term.device)
    # ep_steps == 0: no episode behind the env yet (the initial env.reset()), its zero error is no success
    played = term.ep_steps[env_ids] > 0
    good = played & ~failed & (term.episode_mean_error[env_ids] < up_error)
    s = term.time_scale[env_ids]
    s = s + step * good.float() - step * failed.float()
    term.time_scale[env_ids] = s.clamp(*scale_range)
    return torch.mean(term.time_scale)
