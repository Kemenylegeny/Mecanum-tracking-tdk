"""Reference-trajectory command term for mecanum trajectory tracking.

Every env follows one tabulated reference trajectory (the same files the MPC uses), advanced in time every
env step. The command is the current reference row plus ``preview_steps`` look-ahead rows, spaced by
``preview_dt`` (= the MPC's horizon N and sampling time T), all in the WORLD frame. The body-frame
transformation for the policy happens in the observation term (node 3), not here.

See MAGYARAZAT.md for the reasoning behind every design choice.

Timing (verified in ManagerBasedRLEnv.step): physics -> terminations -> rewards -> resets -> command update ->
observations. The clock ``t`` is the reference time of the robot state the policy OBSERVES: the observation shows
``preview_w`` (current point + look-ahead), and after the next physics step the robot has to be at ``ref_next``
(one policy step later), which is what the rewards and the termination compare against. Envs reset in a step are
anchored at ``t0`` to their new state, so their clock does not advance in that step.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm, CommandTermCfg
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils import configclass

from .traj_utils import (
    TH_COL,
    X_COL,
    Y_COL,
    anchor_to_world,
    load_trajectories,
    scale_velocities,
    sample_table,
    wrap_to_pi,
    yaw_from_quat_wxyz,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class TrajectoryCommand(CommandTerm):
    """Follows tabulated reference trajectories; see :class:`TrajectoryCommandCfg`."""

    cfg: TrajectoryCommandCfg

    def __init__(self, cfg: TrajectoryCommandCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self.robot: Articulation = env.scene[cfg.asset_name]

        # (N_traj, T_max, C) table on the GPU + valid length per trajectory + sampling time
        self.traj, self.traj_len, self.traj_dt = load_trajectories(cfg.traj_files, self.device)
        self.num_cols = self.traj.shape[-1]
        # duration of each trajectory [s]
        self.traj_duration = (self.traj_len - 1).double() * self.traj_dt

        # per-env state
        self.traj_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        # reference clock in float64: in float32 the sum t += step_dt * s drifts by up to ~8 ms over a 200 s lap, so
        # the queries slide off the table's samples (the reference shifts along the path). The query times stay
        # float64 down to the sample index (sample_table); the rows themselves are float32.
        self.t = torch.zeros(self.num_envs, dtype=torch.float64, device=self.device)
        self.anchor = torch.zeros(self.num_envs, 3, device=self.device)  # world (x, y, yaw) of the traj frame
        # query offsets: 0 (current reference) + K look-ahead points, and one policy step (ref_next, last column)
        self.preview_offsets = cfg.preview_dt * torch.arange(
            cfg.preview_steps + 1, dtype=torch.float64, device=self.device
        )
        self.query_offsets = torch.cat([self.preview_offsets, self.preview_offsets.new_tensor([env.step_dt])])
        self.preview_w = torch.zeros(self.num_envs, cfg.preview_steps + 1, self.num_cols, device=self.device)
        self.ref_next_w = torch.zeros(self.num_envs, self.num_cols, device=self.device)
        # time scale s per env: the reference clock runs s times faster than sim time -> v_ref * s, a_lat * s^2.
        # Set by the speed curriculum (curriculums.py); 1.0 = the speed stored in the files.
        self.time_scale = torch.full((self.num_envs,), cfg.initial_time_scale, device=self.device)
        # running episode statistics for the curriculum (read BEFORE the reset, cleared in _resample_command)
        self.ep_err_sum = torch.zeros(self.num_envs, device=self.device)
        self.ep_steps = torch.zeros(self.num_envs, device=self.device)

        # logged to TensorBoard via the command manager (value of the last observed state of the finished episodes)
        self.metrics["pos_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["episode_pos_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["heading_error"] = torch.zeros(self.num_envs, device=self.device)
        self.metrics["time_scale"] = self.time_scale.clone()
        # the full-path markers only change on reset (new trajectory / anchor): redrawn only then
        self._path_dirty = True

    def __str__(self) -> str:
        return (
            f"TrajectoryCommand: {self.traj.shape[0]} trajectories, dt={self.traj_dt:.4f} s, "
            f"preview {self.cfg.preview_steps} x {self.cfg.preview_dt:.3f} s, loop={self.cfg.loop}"
        )

    """
    Properties.
    """

    @property
    def command(self) -> torch.Tensor:
        """(num_envs, (K+1) * C): current + look-ahead reference rows, world frame, flattened."""
        return self.preview_w.flatten(1)

    @property
    def episode_mean_error(self) -> torch.Tensor:
        """(num_envs,): mean position error of the current (or just finished) episode [m]."""
        return self.ep_err_sum / self.ep_steps.clamp(min=1.0)

    @property
    def ref_now(self) -> torch.Tensor:
        """(num_envs, C): the reference row at the clock t = at the time of the observed robot state."""
        return self.preview_w[:, 0]

    @property
    def ref_next(self) -> torch.Tensor:
        """(num_envs, C): the reference row one policy step after the clock, i.e. where the robot has to be after the
        next physics step. Rewards and terminations use it: they run after that step, before the clock advances."""
        return self.ref_next_w

    """
    Implementation specific functions.
    """

    def _resample_command(self, env_ids: Sequence[int]):
        # called on reset (and when time_left runs out, which never happens with resampling_time_range=(1e9, 1e9))
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        if env_ids.numel() == 0:
            return
        # 1) pick a trajectory per env (evaluation: env i always follows trajectory i % N -> deterministic test set)
        if self.cfg.fixed_assignment:
            self.traj_idx[env_ids] = env_ids % self.traj.shape[0]
        else:
            self.traj_idx[env_ids] = torch.randint(0, self.traj.shape[0], (env_ids.numel(),), device=self.device)
        # 2) start time: 0, or random along the trajectory (more varied training data)
        if self.cfg.random_start:
            dur = self.traj_duration[self.traj_idx[env_ids]]
            self.t[env_ids] = torch.rand(env_ids.numel(), dtype=torch.float64, device=self.device) * dur
        else:
            self.t[env_ids] = 0.0
        # 3) anchor: place the trajectory so that its point at the start time lies exactly on the (already
        #    reset) robot -> zero initial error. Verified: ManagerBasedRLEnv._reset_idx resets the commands AFTER the
        #    reset events, and write_root_pose_to_sim updates robot.data right away.
        #    With a start_offset (dx, dy, dpsi) the robot is that far from the start pose instead (in the start pose's
        #    frame), as in the MPC side's Reference.anchor: start pose = robot pose (-) offset. Training adds a random
        #    offset on top (start_offset_prob / start_offset_max).
        start = sample_table(
            self.traj, self.traj_len, self.traj_dt, self.traj_idx[env_ids], self.t[env_ids].unsqueeze(1), self.cfg.loop
        )[:, 0]  # (n, C) in the trajectory frame
        robot_xy = self.robot.data.root_pos_w[env_ids, :2]
        robot_yaw = yaw_from_quat_wxyz(self.robot.data.root_quat_w[env_ids])
        dx, dy, dpsi = self._start_offsets(env_ids.numel(), robot_xy.dtype)
        start_yaw = robot_yaw - dpsi
        cs, ss = torch.cos(start_yaw), torch.sin(start_yaw)
        start_xy = torch.stack([robot_xy[:, 0] - (cs * dx - ss * dy), robot_xy[:, 1] - (ss * dx + cs * dy)], dim=1)
        anchor_yaw = wrap_to_pi(start_yaw - start[:, TH_COL])
        c, s = torch.cos(anchor_yaw), torch.sin(anchor_yaw)
        px, py = start[:, X_COL], start[:, Y_COL]
        self.anchor[env_ids, 0] = start_xy[:, 0] - (c * px - s * py)
        self.anchor[env_ids, 1] = start_xy[:, 1] - (s * px + c * py)
        self.anchor[env_ids, 2] = anchor_yaw
        # 4) clear the episode statistics (the curriculum has already read them)
        self.ep_err_sum[env_ids] = 0.0
        self.ep_steps[env_ids] = 0.0
        # 5) fill the preview right away, so observations right after the reset are valid
        self._compute_preview(env_ids)
        self._path_dirty = True

    def _start_offsets(self, n: int, dtype: torch.dtype) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Initial error (dx, dy, dpsi) of n resets: the fixed start_offset plus, independently, a random position
        offset (probability start_offset_prob[0]: distance uniform in [0, start_offset_max[0]], uniform direction) and
        a random heading offset (probability start_offset_prob[1]: uniform in +-start_offset_max[1])."""
        dx, dy, dpsi = (torch.full((n,), v, dtype=dtype, device=self.device) for v in self.cfg.start_offset)
        (p_pos, p_yaw), (r_max, yaw_max) = self.cfg.start_offset_prob, self.cfg.start_offset_max
        if p_pos > 0.0:
            r = r_max * torch.rand(n, dtype=dtype, device=self.device) * (torch.rand(n, device=self.device) < p_pos)
            ang = 2 * math.pi * torch.rand(n, dtype=dtype, device=self.device)
            dx, dy = dx + r * torch.cos(ang), dy + r * torch.sin(ang)
        if p_yaw > 0.0:
            on = torch.rand(n, device=self.device) < p_yaw
            dpsi = dpsi + yaw_max * (2 * torch.rand(n, dtype=dtype, device=self.device) - 1) * on
        return dx, dy, dpsi

    def _update_command(self):
        # called every env step, for every env, AFTER the rewards and the resets: advance the clock by one policy step
        # (the reference clock runs time_scale times faster than sim time), then re-sample the preview. Envs reset in
        # this very step (episode_length_buf == 0) keep their t0: their new state is the one the trajectory was
        # anchored to. (After the initial env.reset() no update runs before the first physics step, so there the
        # clock does advance in the first step.)
        stepped = (self._env.episode_length_buf > 0).double()
        self.t += self._env.step_dt * self.time_scale.double() * stepped  # step_dt = sim.dt * decimation
        self._compute_preview(slice(None))
        # the clock and the robot state now refer to the same instant
        ref = self.ref_now
        pos = self.robot.data.root_pos_w[:, :2]
        self.metrics["pos_error"] = torch.linalg.norm(ref[:, [X_COL, Y_COL]] - pos, dim=1)
        yaw = yaw_from_quat_wxyz(self.robot.data.root_quat_w)
        self.metrics["heading_error"] = torch.abs(wrap_to_pi(ref[:, TH_COL] - yaw))
        self.metrics["time_scale"] = self.time_scale.clone()
        self.ep_err_sum += self.metrics["pos_error"]
        self.ep_steps += 1.0
        self.metrics["episode_pos_error"] = self.episode_mean_error

    def _update_metrics(self):
        # runs BEFORE _update_command, when the clock still lags the robot state by one step: the metrics are
        # updated at the end of _update_command instead
        pass

    def _compute_preview(self, env_ids):
        scale = self.time_scale[env_ids].unsqueeze(1)  # (n, 1)
        # look-ahead points are T apart in REAL time -> s * T apart in reference time; the last query is ref_next
        tq = self.t[env_ids].unsqueeze(1) + scale * self.query_offsets  # (n, K+2)
        rows = sample_table(self.traj, self.traj_len, self.traj_dt, self.traj_idx[env_ids], tq, self.cfg.loop)
        rows = scale_velocities(rows, scale)
        rows = anchor_to_world(rows, self.anchor[env_ids], self.cfg.velocity_in_traj_frame)
        self.preview_w[env_ids] = rows[:, :-1]
        self.ref_next_w[env_ids] = rows[:, -1]

    """
    Debug visualization: a big sphere on the current reference point, small ones on the look-ahead points, and
    the whole trajectory of the episode as a dotted line (first ``path_vis_num_envs`` envs).
    """

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            if not hasattr(self, "ref_visualizer"):
                self.ref_visualizer = VisualizationMarkers(self.cfg.ref_visualizer_cfg)
            self.ref_visualizer.set_visibility(True)
            if self.cfg.path_vis_num_envs > 0:
                if not hasattr(self, "path_visualizer"):
                    self.path_visualizer = VisualizationMarkers(self.cfg.path_visualizer_cfg)
                    self._path_dirty = True
                self.path_visualizer.set_visibility(True)
        else:
            if hasattr(self, "ref_visualizer"):
                self.ref_visualizer.set_visibility(False)
            if hasattr(self, "path_visualizer"):
                self.path_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        if not self.robot.is_initialized:
            return
        k1 = self.preview_w.shape[1]
        pts = torch.zeros(self.num_envs, k1, 3, device=self.device)
        pts[..., 0] = self.preview_w[..., X_COL]
        pts[..., 1] = self.preview_w[..., Y_COL]
        pts[..., 2] = self.robot.data.root_pos_w[:, 2:3] + self.cfg.marker_height
        idx = torch.ones(self.num_envs, k1, dtype=torch.int32, device=self.device)
        idx[:, 0] = 0  # marker 0: current reference, marker 1: look-ahead
        self.ref_visualizer.visualize(translations=pts.reshape(-1, 3), marker_indices=idx.reshape(-1))
        if hasattr(self, "path_visualizer") and self._path_dirty:
            self._draw_path()

    def _draw_path(self):
        """Whole trajectory (one lap) of the first ``path_vis_num_envs`` envs, in the world frame."""
        ids = torch.arange(min(self.cfg.path_vis_num_envs, self.num_envs), device=self.device)
        tid = self.traj_idx[ids]
        frac = torch.linspace(0.0, 1.0, self.cfg.path_vis_points, device=self.device)
        tq = frac.unsqueeze(0) * self.traj_duration[tid].unsqueeze(1)  # (n, P), in the file's own time
        rows = sample_table(self.traj, self.traj_len, self.traj_dt, tid, tq, self.cfg.loop)
        rows = anchor_to_world(rows, self.anchor[ids], self.cfg.velocity_in_traj_frame)
        pts = torch.zeros(ids.numel(), frac.numel(), 3, device=self.device)
        pts[..., 0] = rows[..., X_COL]
        pts[..., 1] = rows[..., Y_COL]
        pts[..., 2] = self._env.scene.env_origins[ids, 2:3] + 0.01  # on the ground
        self.path_visualizer.visualize(translations=pts.reshape(-1, 3))
        self._path_dirty = False


@configclass
class TrajectoryCommandCfg(CommandTermCfg):
    """Configuration for :class:`TrajectoryCommand`."""

    class_type: type = TrajectoryCommand

    asset_name: str = MISSING
    """Name of the robot asset in the scene."""

    traj_files: list[str] = MISSING
    """Reference trajectory files (CSV with header or NPY), columns: t, X, Y, theta, vx, vy, omega [, tau_1..4]
    (tau: step-averaged reference wheel torques over [t, t + T], MPC wheel order; mpc/tables.py).
    All files must have the same sampling time and columns. Each trajectory starts in its own frame; it is
    anchored to the robot's pose at reset."""

    preview_steps: int = 20
    """Number of look-ahead points K. Set it to the MPC horizon N."""

    preview_dt: float = 0.05
    """Spacing of the look-ahead points [s]. Set it to the MPC sampling time T (so K * dt = N * T)."""

    loop: bool = True
    """Closed trajectory: wrap around after the end. The file must then repeat its first sample as the last one.
    False: hold the last sample (the robot should stop there)."""

    random_start: bool = False
    """Start every episode at a random time along the trajectory (training). Keep False for evaluation."""

    fixed_assignment: bool = False
    """Evaluation: env i follows trajectory i % N (set num_envs = number of test files). Training: False (random)."""

    start_offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Initial error (dx [m], dy [m], dpsi [rad]): the robot's pose relative to the trajectory's start pose, in the
    start pose's frame (the MPC side's ``Reference.anchor(start_offset=...)``). Training: 0; evaluation scenario 2:
    (0.3, 0.0, 0.52), 30 cm / 30 deg."""

    start_offset_prob: tuple[float, float] = (0.0, 0.0)
    """Training: probability of a random position offset and of a random heading offset at a reset, independent of
    each other (so a start can be on the reference, off it, turned, or both), added to ``start_offset``. 0: off."""

    start_offset_max: tuple[float, float] = (0.0, 0.0)
    """Largest random offset: distance [m] (uniform in [0, max], uniform direction) and heading [rad] (uniform in
    [-max, max])."""

    initial_time_scale: float = 1.0
    """Initial reference time scale s (the speed curriculum changes it per env). Evaluation: 1.0, no curriculum."""

    velocity_in_traj_frame: bool = False
    """True if (vx, vy) in the files are given in the trajectory (world-like) frame and must be rotated with the
    anchor; False if they are body-frame velocities (as in the MPC's u = (vx, vy, omega)). ASK THE MPC SIDE."""

    marker_height: float = 0.1
    """Height of the debug markers above the robot base [m]."""

    path_vis_num_envs: int = 16
    """Debug vis: draw the whole trajectory for this many envs (from env 0, the one the camera follows). 0 = off."""

    path_vis_points: int = 300
    """Debug vis: number of dots along the drawn trajectory."""

    path_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/trajectory_path",
        markers={
            "path": sim_utils.CylinderCfg(
                radius=0.02,
                height=0.005,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.75, 0.3)),
            ),
        },
    )

    ref_visualizer_cfg: VisualizationMarkersCfg = VisualizationMarkersCfg(
        prim_path="/Visuals/Command/trajectory_ref",
        markers={
            "current": sim_utils.SphereCfg(
                radius=0.06, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1))
            ),
            "preview": sim_utils.SphereCfg(
                radius=0.025, visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.4, 0.9))
            ),
        },
    )
