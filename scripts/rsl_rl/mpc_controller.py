"""The LTV-MPC as a controller of the Isaac Lab env, shared by eval_tracking.py and play_shapes.py, so the measurement and
the videos run exactly the same controller (docs/OSSZEHANGOLAS.md, 2.1).

One LinearMPCv2 per env (mecanum_tracking_ws/mpc/lmpc.py = Mecanum-Robot-MPC-TDK's LinearMPCv2 + DARE terminal weight),
CPU. Since 2026-10-28 the default is the plain LTV-MPC: Q, R, the DARE terminal weight and the hard torque box, without
the torque-change penalty S and the soft wheel-speed limit (logs/eval/mpc_plain_corners_table.md); the runs before used
--mpc_S 100 --mpc_wheel_limit. Since 2026-10-29 its state is z = [X, Y, psi, w1, w2, w3] (mpc/wheel_dynamics.py: the
pose and the speeds of the front-left, front-right and rear-left wheels); the runs before used --mpc_state world,
x = [X, Y, psi, X_dot, Y_dot, psi_dot].

Its inputs come from the env: the pose of the base_link origin (ground truth, world frame); the wheel speeds from the
ground-truth base velocity through the rolling constraints (--mpc_wheel_meas kinematic, the default: the information the
policy observes, base_vel), or from the four wheel encoders projected onto the rolling constraints (encoder,
wheel_dynamics.from_encoders: unbiased, but the roller contacts make them noisy, +-3.5 mm/s at 0.1 m/s, and the MPC
tracks ~0.6 mm worse, logs/eval/mpc_wheel_state_table.md); world: the base velocity itself; x_r,0..N and u_r,0..N-1
from the command's preview rows (the same rows the policy observes; the tau columns of the files are the MPC side's
step-averaged reference torques), u_applied = its previous output.

Imported before the app is launched (add_mpc_args), so the Isaac Lab / package imports stay inside the functions.
"""

import argparse

import numpy as np

MPC_EXTRA = ["qp_ms", "status", "slack_max", "dare_iters"]
"""Controller-specific columns of the common log (tools/metrics.py)."""


def add_mpc_args(parser: argparse.ArgumentParser):
    parser.add_argument("--mpc_terminal", choices=["dare", "scaled"], default="dare",
                        help="terminal weight: DARE solution (default) or the MPC side's P = 5 Q")
    parser.add_argument("--mpc_S", type=float, default=0.0,
                        help="torque-change penalty S = s * I (default 0: off; until 2026-10-27: 100)")
    parser.add_argument("--mpc_wheel_limit", action="store_true",
                        help="soft wheel-speed limit |w| <= 8.06 rad/s (default off; until 2026-10-27: on)")
    parser.add_argument("--mpc_Q_path", type=float, nargs=2, default=None, metavar=("Q_ALONG", "Q_CROSS"),
                        help="position weight in the path frame (along-track, cross-track) instead of X/Y (off: Q's 40000)")
    parser.add_argument("--mpc_state", choices=["wheel", "world"], default="wheel",
                        help="velocity states: three wheel speeds (default since 2026-10-29) or the world-frame base "
                        "velocity (until 2026-10-28)")
    parser.add_argument("--mpc_wheel_meas", choices=["kinematic", "encoder"], default="kinematic",
                        help="--mpc_state wheel: wheel speeds from the ground-truth base velocity through the rolling "
                        "constraints (default: the policy's information) or from the four encoders (projected onto them)")


def robot_state(robot) -> np.ndarray:
    """The MPC state of every env (n, 6): [X, Y, psi, X_dot, Y_dot, psi_dot] of the base_link origin, world frame."""
    import torch

    from mecanum_tracking_ws.tasks.manager_based.mecanum_tracking.mdp.traj_utils import yaw_from_quat_wxyz

    d = robot.data
    return torch.cat([d.root_link_pos_w[:, :2], yaw_from_quat_wxyz(d.root_link_quat_w)[:, None],
                      d.root_link_lin_vel_w[:, :2], d.root_link_ang_vel_w[:, 2:3]], 1).double().cpu().numpy()


class MPCController:
    """One LinearMPCv2 per env, fed from the env's state and the command's preview rows."""

    def __init__(self, n_envs, robot_cfg_module, env_cfg, step_dt, args):
        from mecanum_tracking_ws.mpc import params
        from mecanum_tracking_ws.mpc.lmpc import LinearMPCv2

        cmd = env_cfg.commands.ref_traj
        self.N, self.T = cmd.preview_steps, cmd.preview_dt
        if abs(self.T - step_dt) > 1e-9:
            raise ValueError(f"the MPC's T = {self.T} must equal the control period {step_dt}")
        self.p = params.from_urdf(str(robot_cfg_module.URDF_PATH))
        S = None if args.mpc_S <= 0 else [args.mpc_S] * 4
        self.state, self.wheel_meas = args.mpc_state, args.mpc_wheel_meas
        self.ctrls = [LinearMPCv2(self.p, self.T, self.N, u_max=robot_cfg_module.WHEEL_EFFORT_LIMIT,
                                  omega_max=robot_cfg_module.MOTOR_NO_LOAD_SPEED,
                                  wheel_speed_limit=args.mpc_wheel_limit,
                                  terminal=args.mpc_terminal, S=S, Q_path=args.mpc_Q_path, state=args.mpc_state)
                      for _ in range(n_envs)]
        self.u_applied = np.zeros((n_envs, 4))
        state = "[X, Y, psi, X_dot, Y_dot, psi_dot]" if self.state == "world" else \
            f"[X, Y, psi, w1, w2, w3], wheel speeds from {self.wheel_meas}"
        print(f"[mpc] N = {self.N}, T = {self.T} s, terminal {args.mpc_terminal}, S = {S}, "
              f"Q_path {args.mpc_Q_path}, wheel-speed limit {args.mpc_wheel_limit}; state {state}; "
              f"model m_e {self.p.m_e:.4f} kg, J_e {self.p.J_e:.5f} kg m^2")

    def __call__(self, x, preview_rows, w=None):
        """x: robot_state (n, 6); preview_rows: the command's preview (n, N + 1, C); w: the encoders' wheel speeds
        (n, 4), MPC order and signs (needed for --mpc_state wheel --mpc_wheel_meas encoder)."""
        from mecanum_tracking_ws.mpc import wheel_dynamics
        from mecanum_tracking_ws.mpc.tables import horizon_from_rows

        if self.state == "wheel":
            x = wheel_dynamics.from_encoders(x, w, self.p) if self.wheel_meas == "encoder" else \
                wheel_dynamics.from_world(x, self.p)
        u = np.zeros_like(self.u_applied)
        infos = []
        for i, ctrl in enumerate(self.ctrls):
            x_r, u_r = horizon_from_rows(preview_rows[i], self.N)
            if self.state == "wheel":
                x_r = wheel_dynamics.from_world(x_r, self.p)
            u[i], info = ctrl.step(x[i], x_r, u_r, self.u_applied[i])
            infos.append(info)
        self.u_applied = u.copy()
        return u, infos

    def extra(self, infos):
        """The MPC_EXTRA columns of every env."""
        return [[info["qp_ms"], info["status"], info["slack_max"], self.ctrls[i].dare_iterations]
                for i, info in enumerate(infos)]
