"""Isaac Lab-independent helpers for the tabulated reference trajectories.

Everything here is pure PyTorch, so it can be tested without Isaac Lab
(see ``tests/test_traj_utils.py``).

Column layout of a trajectory table (C = 7, optionally + 4 torques):
    0: t [s]   1: X [m]   2: Y [m]   3: theta [rad]   4: vx   5: vy   6: omega   (7..10: tau_1..tau_4)
The time column is used only to check the sampling time; queries use ``traj_dt``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
import torch

# column indices
T_COL, X_COL, Y_COL, TH_COL, VX_COL, VY_COL, OM_COL = range(7)


def wrap_to_pi(a: torch.Tensor) -> torch.Tensor:
    """Wraps an angle to [-pi, pi)."""
    return torch.remainder(a + math.pi, 2.0 * math.pi) - math.pi


def load_trajectories(files: Sequence[str], device: str | torch.device) -> tuple[torch.Tensor, torch.Tensor, float]:
    """Loads CSV (header row) / NPY trajectory files into one padded GPU tensor.

    Returns:
        traj: (N_traj, T_max, C). Shorter trajectories are padded with their last row.
        traj_len: (N_traj,) long, number of valid samples per trajectory.
        traj_dt: common sampling time [s] (must be identical for all files).
    """
    tables, file_dts = [], []
    for f in files:
        arr = np.load(f) if f.endswith(".npy") else np.loadtxt(f, delimiter=",", skiprows=1)
        arr = np.atleast_2d(arr).astype(np.float64)
        # checked in float64: in float32 the time column of a long trajectory (> ~100 s) is too coarse for it
        dts = np.diff(arr[:, T_COL])
        if not np.allclose(dts, dts[0], rtol=1e-3, atol=1e-6):
            raise ValueError(f"{f}: non-uniform sampling time")
        # dt also from float64 (the float32 0.01 is 0.0099999998: a 1e-8 relative clock error at every query)
        file_dts.append((arr[-1, T_COL] - arr[0, T_COL]) / (arr.shape[0] - 1))
        tables.append(arr.astype(np.float32))
    dt = float(file_dts[0])
    for f, f_dt in zip(files, file_dts):
        if not math.isclose(float(f_dt), dt, rel_tol=1e-3):
            raise ValueError(f"{f}: sampling time differs from {files[0]}")
    n_cols = {a.shape[1] for a in tables}
    if len(n_cols) != 1:
        raise ValueError(f"all trajectory files must have the same columns, got {n_cols}")
    t_max = max(a.shape[0] for a in tables)
    padded = [np.concatenate([a, np.repeat(a[-1:], t_max - a.shape[0], axis=0)]) for a in tables]
    traj = torch.tensor(np.stack(padded), device=device)
    traj_len = torch.tensor([a.shape[0] for a in tables], device=device, dtype=torch.long)
    return traj, traj_len, dt


def sample_table(
    traj: torch.Tensor,
    traj_len: torch.Tensor,
    traj_dt: float,
    traj_idx: torch.Tensor,
    tq: torch.Tensor,
    loop: bool,
) -> torch.Tensor:
    """Linearly interpolates the trajectory table at arbitrary query times, for every env at once.

    Args:
        traj: (N_traj, T_max, C) table.
        traj_len: (N_traj,) valid samples per trajectory.
        traj_dt: sampling time of the table [s].
        traj_idx: (num_envs,) which trajectory each env follows.
        tq: (num_envs, M) query times [s] (e.g. t + i * preview_dt). The sample index is computed in tq's dtype:
            pass float64 times for long trajectories (float32 resolves only ~3e-5 s at 300 s).
        loop: closed trajectory -> wrap around (modulo); otherwise hold the last sample.

    Returns:
        (num_envs, M, C) interpolated rows (in the trajectory's own frame), in the table's dtype.
    """
    n = traj_len[traj_idx].unsqueeze(1)  # (E, 1)
    s = tq / traj_dt
    if loop:
        # the last sample of a closed curve equals the first one -> period is (n - 1) samples
        period = (n - 1).to(s.dtype)
        s = torch.remainder(s, period)
    else:
        s = torch.minimum(s.clamp(min=0.0), (n - 1).to(s.dtype))
    i0 = torch.floor(s).long()
    i0 = torch.minimum(i0, n - 1)
    i1 = torch.minimum(i0 + 1, n - 1)
    w = (s - i0.to(s.dtype)).to(traj.dtype).unsqueeze(-1)  # (E, M, 1)

    rows0 = traj[traj_idx.unsqueeze(1), i0]  # (E, M, C)
    rows1 = traj[traj_idx.unsqueeze(1), i1]
    out = rows0 + w * (rows1 - rows0)
    # the heading must not be lerped directly: interpolate the wrapped difference
    dth = wrap_to_pi(rows1[..., TH_COL] - rows0[..., TH_COL])
    out[..., TH_COL] = wrap_to_pi(rows0[..., TH_COL] + w.squeeze(-1) * dth)
    out[..., T_COL] = tq.to(traj.dtype)
    return out


def anchor_to_world(rows: torch.Tensor, anchor: torch.Tensor, rotate_velocity: bool) -> torch.Tensor:
    """Moves trajectory rows from the trajectory's own frame (starting at the origin) to the world.

    Args:
        rows: (E, M, C) rows from :func:`sample_table`.
        anchor: (E, 3) world pose (x, y, yaw) where the trajectory frame is placed.
        rotate_velocity: True if (vx, vy) in the file are given in the trajectory (world-like) frame,
            False if they are body-frame velocities (then a rotation of the whole path does not change them).
    """
    out = rows.clone()
    c = torch.cos(anchor[:, 2]).unsqueeze(1)
    s = torch.sin(anchor[:, 2]).unsqueeze(1)
    x, y = rows[..., X_COL], rows[..., Y_COL]
    out[..., X_COL] = anchor[:, 0:1] + c * x - s * y
    out[..., Y_COL] = anchor[:, 1:2] + s * x + c * y
    out[..., TH_COL] = wrap_to_pi(rows[..., TH_COL] + anchor[:, 2:3])
    if rotate_velocity:
        vx, vy = rows[..., VX_COL], rows[..., VY_COL]
        out[..., VX_COL] = c * vx - s * vy
        out[..., VY_COL] = s * vx + c * vy
    return out


def yaw_from_quat_wxyz(q: torch.Tensor) -> torch.Tensor:
    """Yaw of a (w, x, y, z) quaternion (Isaac Lab's ordering), in [-pi, pi]."""
    w, x, y, z = q.unbind(-1)
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def preview_to_body(
    preview_w: torch.Tensor, robot_xy: torch.Tensor, robot_yaw: torch.Tensor, n_extra: int = 0
) -> torch.Tensor:
    """Expresses the world-frame reference preview relative to the robot, for the policy.

    Args:
        preview_w: (E, M, C) reference rows in the world frame (from TrajectoryCommand.preview_w).
            (vx, vy, omega) are body-frame velocities of the REFERENCE pose.
        robot_xy: (E, 2) robot world position.  robot_yaw: (E,) robot world yaw.
        n_extra: number of extra columns after omega to pass through unchanged (e.g. 4 for tau_ref).

    Returns:
        (E, M, 7 + n_extra) per point: dx_b, dy_b (position error in the robot's yaw frame),
        sin(dtheta), cos(dtheta), vx_ref_b, vy_ref_b (reference velocity rotated into the robot frame),
        omega_ref [, extras].
    """
    c = torch.cos(robot_yaw).unsqueeze(1)
    s = torch.sin(robot_yaw).unsqueeze(1)
    dx = preview_w[..., X_COL] - robot_xy[:, 0:1]
    dy = preview_w[..., Y_COL] - robot_xy[:, 1:2]
    dx_b = c * dx + s * dy  # R(-yaw) @ (dx, dy)
    dy_b = -s * dx + c * dy
    dth = wrap_to_pi(preview_w[..., TH_COL] - robot_yaw.unsqueeze(1))
    # reference body velocity -> world (rotate by theta_ref) -> robot body (rotate by -yaw) = rotate by dtheta
    cd, sd = torch.cos(dth), torch.sin(dth)
    vx_r, vy_r = preview_w[..., VX_COL], preview_w[..., VY_COL]
    vx_b = cd * vx_r - sd * vy_r
    vy_b = sd * vx_r + cd * vy_r
    cols = [dx_b, dy_b, torch.sin(dth), torch.cos(dth), vx_b, vy_b, preview_w[..., OM_COL]]
    out = torch.stack(cols, dim=-1)
    if n_extra > 0:
        out = torch.cat([out, preview_w[..., OM_COL + 1 : OM_COL + 1 + n_extra]], dim=-1)
    return out


def tracking_errors(
    ref_w: torch.Tensor, robot_xy: torch.Tensor, robot_vel_w: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """Position and velocity tracking errors, both in the WORLD frame (heading-free: no heading error).

    Args:
        ref_w: (E, C) reference row in the world frame at the time of the robot state (TrajectoryCommand.ref_next
            in the rewards).
            (vx, vy) are body-frame velocities of the reference pose, whose world heading is ref_w[:, TH_COL].
        robot_xy: (E, 2) robot world position.  robot_vel_w: (E, 2) robot world linear velocity (x, y).

    Returns:
        pos_err_sq: (E,) squared position error [m^2].  vel_err_sq: (E,) squared velocity error [(m/s)^2].
    """
    pos_err_sq = torch.sum((ref_w[:, [X_COL, Y_COL]] - robot_xy) ** 2, dim=1)
    c, s = torch.cos(ref_w[:, TH_COL]), torch.sin(ref_w[:, TH_COL])
    vx_w = c * ref_w[:, VX_COL] - s * ref_w[:, VY_COL]
    vy_w = s * ref_w[:, VX_COL] + c * ref_w[:, VY_COL]
    vel_err_sq = (vx_w - robot_vel_w[:, 0]) ** 2 + (vy_w - robot_vel_w[:, 1]) ** 2
    return pos_err_sq, vel_err_sq


def scale_velocities(rows: torch.Tensor, scale: torch.Tensor) -> torch.Tensor:
    """Reference rows sampled with time scale s: the velocities (vx, vy, omega) are s times the stored ones.

    rows: (E, M, C); scale: (E, 1). NOTE: reference torques (if present) do NOT scale linearly with s (the
    accelerations scale with s^2 and friction does not scale) -- do not use tau_ref together with time scaling.
    """
    out = rows.clone()
    out[..., VX_COL:OM_COL + 1] = rows[..., VX_COL:OM_COL + 1] * scale.unsqueeze(-1)
    return out
