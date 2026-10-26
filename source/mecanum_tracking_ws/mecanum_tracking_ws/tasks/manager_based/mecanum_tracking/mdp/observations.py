"""Observation terms for mecanum trajectory tracking (node 3).

Every term returns (num_envs, D) -- the ObservationManager concatenates them into the policy input.
See MAGYARAZAT.md, section 5, for the reasoning.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from .traj_utils import preview_to_body, yaw_from_quat_wxyz

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .trajectory_command import TrajectoryCommand


def ref_preview_b(
    env: ManagerBasedRLEnv,
    command_name: str,
    include_current: bool = True,
    include_heading: bool = True,
    n_extra: int = 0,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reference preview relative to the robot. Shape (num_envs, M * (7 + n_extra)), or M * (5 + n_extra)
    without the heading columns.

    Per point: dx_b, dy_b, sin(dtheta), cos(dtheta), vx_ref_b, vy_ref_b, omega_ref [, tau_ref_1..4].
    M = K + 1 with ``include_current`` (current reference point + K look-ahead points), else K.
    ``include_heading=False`` drops sin/cos(dtheta): use it for the heading-FREE task (files with a constant
    theta = 0 column), where there is no heading reference to track. The velocities stay correct either way.
    ``n_extra=4`` passes the reference wheel torques through (if the MPC side provides them in the files).
    """
    term: TrajectoryCommand = env.command_manager.get_term(command_name)
    robot: Articulation = env.scene[asset_cfg.name]
    preview = term.preview_w if include_current else term.preview_w[:, 1:]
    obs = preview_to_body(
        preview, robot.data.root_pos_w[:, :2], yaw_from_quat_wxyz(robot.data.root_quat_w), n_extra=n_extra
    )
    if not include_heading:
        obs = torch.cat([obs[..., :2], obs[..., 4:]], dim=-1)
    return obs.flatten(1)


def base_planar_vel_b(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """(num_envs, 3): the velocity part of the MPC state, (X_dot, Y_dot, psi_dot) of the base_link origin (point C of
    the MPC model, not the CoM), with (X_dot, Y_dot) rotated into the robot's heading frame. No vz / roll / pitch rates:
    the MPC state is planar."""
    robot: Articulation = env.scene[asset_cfg.name]
    yaw = yaw_from_quat_wxyz(robot.data.root_link_quat_w)
    v = robot.data.root_link_lin_vel_w
    c, s = torch.cos(yaw), torch.sin(yaw)
    return torch.stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1], robot.data.root_link_ang_vel_w[:, 2]], 1)


def last_wheel_torque(env: ManagerBasedRLEnv, action_name: str = "wheel_torque", scale: float = 1.0) -> torch.Tensor:
    """(num_envs, 4): the wheel torques commanded in the previous step (after the clip) times ``scale``: the MPC's
    u_applied. Unlike ``last_action`` it is not the raw (unclipped) policy output."""
    return env.action_manager.get_term(action_name).processed_actions * scale
