"""Reward terms for mecanum trajectory tracking (node 5).

Every term returns (num_envs,); the RewardManager multiplies it by weight * dt and sums the terms.
All tracking terms are bounded, positive exp kernels; the penalties must stay small next to them, so that the total
per-step reward stays positive (otherwise early termination pays off). See MAGYARAZAT.md, section 7.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

from .traj_utils import OM_COL, TH_COL, tracking_errors, wrap_to_pi, yaw_from_quat_wxyz

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

    from .trajectory_command import TrajectoryCommand


def _errors(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg):
    term: TrajectoryCommand = env.command_manager.get_term(command_name)
    robot: Articulation = env.scene[asset_cfg.name]
    # rewards run after the physics step, before the command clock advances: the robot state is one policy step past
    # the clock, so it is compared with ref_next (the reference at the same instant)
    return tracking_errors(term.ref_next, robot.data.root_pos_w[:, :2], robot.data.root_lin_vel_w[:, :2])


def track_pos_exp(
    env: ManagerBasedRLEnv, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """exp(-||p - p_ref(t)||^2 / std^2). Use it twice: a coarse std (come back) and a fine std (precision)."""
    pos_err_sq, _ = _errors(env, command_name, asset_cfg)
    return torch.exp(-pos_err_sq / std**2)


def track_vel_exp(
    env: ManagerBasedRLEnv, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """exp(-||v - v_ref(t)||^2 / std^2), world-frame linear velocity (damping + an immediate learning signal)."""
    _, vel_err_sq = _errors(env, command_name, asset_cfg)
    return torch.exp(-vel_err_sq / std**2)


def yaw_rate_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """omega_z^2 -- with a heading-free task this discourages pointless spinning. Use a NEGATIVE weight."""
    robot: Articulation = env.scene[asset_cfg.name]
    return robot.data.root_ang_vel_w[:, 2] ** 2


def track_heading_exp(
    env: ManagerBasedRLEnv, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """exp(-wrap(psi - psi_ref(t))^2 / std^2): heading tracking (the MPC tracks psi_r with its Q_psi weight)."""
    term: TrajectoryCommand = env.command_manager.get_term(command_name)
    robot: Articulation = env.scene[asset_cfg.name]
    err = wrap_to_pi(yaw_from_quat_wxyz(robot.data.root_quat_w) - term.ref_next[:, TH_COL])
    return torch.exp(-(err**2) / std**2)


def yaw_rate_error_l2(
    env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """(psi_dot - psi_dot_ref)^2 [(rad/s)^2]. With heading tracking the robot has to turn with a tangent heading,
    so the plain yaw_rate_l2 would fight the task. Use a NEGATIVE weight."""
    term: TrajectoryCommand = env.command_manager.get_term(command_name)
    robot: Articulation = env.scene[asset_cfg.name]
    return (robot.data.root_ang_vel_w[:, 2] - term.ref_next[:, OM_COL]) ** 2


def torque_rate_l2(env: ManagerBasedRLEnv, action_name: str = "wheel_torque") -> torch.Tensor:
    """||tau_k - tau_k-1||^2 [Nm^2] of the clipped wheel torques: the RL counterpart of the MPC's torque-change
    penalty S (the first step after a reset compares with 0, like the MPC's u_applied). Use a NEGATIVE weight."""
    term = env.action_manager.get_term(action_name)
    return torch.sum((term.processed_actions - term.prev_processed_actions) ** 2, dim=1)


def torque_l2(env: ManagerBasedRLEnv, action_name: str = "wheel_torque") -> torch.Tensor:
    """||tau_k||^2 [Nm^2]: the RL counterpart of the MPC's R (which weights tau - tau_ref; tau_ref <= 0.01 Nm here).
    Use a NEGATIVE weight."""
    return torch.sum(env.action_manager.get_term(action_name).processed_actions ** 2, dim=1)


def wheel_speed_excess_l2(
    env: ManagerBasedRLEnv, omega_max: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """sum_i max(|w_i| - omega_max, 0)^2 [(rad/s)^2] over the wheel joints of ``asset_cfg``: the RL counterpart of the
    MPC's soft wheel-speed limit (slack penalty). Use a NEGATIVE weight."""
    robot: Articulation = env.scene[asset_cfg.name]
    excess = torch.clamp(torch.abs(robot.data.joint_vel[:, asset_cfg.joint_ids]) - omega_max, min=0.0)
    return torch.sum(excess**2, dim=1)
