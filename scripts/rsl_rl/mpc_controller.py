"""The LTV-MPC as a controller of the Isaac Lab env, shared by eval_tracking.py and play_shapes.py, so the measurement and
the videos run exactly the same controller (docs/OSSZEHANGOLAS.md, 2.1).

One LinearMPCv2 per env (mecanum_tracking_ws/mpc/lmpc.py = Mecanum-Robot-MPC-TDK's LinearMPCv2 + DARE terminal weight),
CPU. Since 2026-10-28 the default is the plain LTV-MPC: Q, R, the DARE terminal weight and the hard torque box, without
the torque-change penalty S and the soft wheel-speed limit (logs/eval/mpc_plain_corners_table.md); the runs before used
--mpc_S 100 --mpc_wheel_limit. Since 2026-10-29 its state is z = [X, Y, psi, w1, w2, w3] (mpc/wheel_dynamics.py: the
pose and the speeds of the front-left, front-right and rear-left wheels); the runs before used --mpc_state world,
x = [X, Y, psi, X_dot, Y_dot, psi_dot]. Since 2026-10-30 its weights are those of the Bayesian optimisation (Q_BO, r =
400) and it has no torque-rate limit (mpc/lmpc.py); the runs before pin the Bryson weights (--mpc_Q 40000 40000 3283
2500 2500 100).

Its inputs come from the env: the pose of the base_link origin (ground truth, world frame); the wheel speeds from the
ground-truth base velocity through the rolling constraints (--mpc_wheel_meas kinematic, the default: the information the
policy observes, base_vel), or from the four wheel encoders projected onto the rolling constraints (encoder,
wheel_dynamics.from_encoders: unbiased, but the roller contacts make them noisy, +-3.5 mm/s at 0.1 m/s, and the MPC
tracks ~0.6 mm worse, logs/eval/mpc_wheel_state_table.md); world: the base velocity itself; x_r,0..N and u_r,0..N-1
from the command's preview rows (the same rows the policy observes; the tau columns of the files are the MPC side's
step-averaged reference torques), u_applied = its previous output.

--mpc_workers K steps the envs' controllers in K worker processes (this file run as a script: the mpc package without
Isaac Lab, one BLAS thread each) instead of one after the other; the same code, so the same torques. For many envs, e.g.
the weight tuning (tools/tune_mpc_bo.py: eval_tracking.py --mpc_trials).

Imported before the app is launched (add_mpc_args), so the Isaac Lab / package imports stay inside the functions.
"""

import argparse
import os
import pickle
import subprocess
import sys

import numpy as np

MPC_EXTRA = ["qp_ms", "status", "slack_max", "dare_iters"]
"""Controller-specific columns of the common log (tools/metrics.py)."""

Q_BRYSON = (40000.0, 40000.0, 3283.0, 2500.0, 2500.0, 100.0)
"""The MPC side's weights (Bryson's rule; LinearMPCv2's default), the --mpc_Q default until 2026-10-29."""
Q_BO = (1e7, 1e7, 1e6, 17.5888, 17.5888, 15.22)
"""The --mpc_Q default since 2026-10-30, with r = 400 and no S: the best of the Bayesian optimisation in the closed loop
(tools/tune_mpc_bo.py, logs/eval/mpc_bo_table.md). ~9x smaller position error than Q_BRYSON on the test sets, but the
velocity / yaw-rate errors grow (q_v, q_omega near the bottom of the search range), 40 cm / 45 deg start errors drive
the torque into its box, and with the encoders' noise the torques get ~9x noisier (logs/eval/mpc_bo_*_table.md)."""

_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "source", "mecanum_tracking_ws",
                    "mecanum_tracking_ws")


def add_mpc_args(parser: argparse.ArgumentParser):
    parser.add_argument("--mpc_terminal", choices=["dare", "scaled"], default="dare",
                        help="terminal weight: DARE solution (default) or the MPC side's P = 5 Q")
    parser.add_argument("--mpc_Q", type=float, nargs=6, default=list(Q_BO),
                        metavar=("Q_X", "Q_Y", "Q_PSI", "Q_XDOT", "Q_YDOT", "Q_PSIDOT"),
                        help="state weights (default since 2026-10-30: the Bayesian optimisation's, Q_BO; the Bryson "
                        "weights before: 40000 40000 3283 2500 2500 100); with --mpc_state wheel the velocity weights act "
                        "through the rolling constraints (wheel_dynamics.velocity_weight)")
    parser.add_argument("--mpc_R", type=float, default=None, help="torque weight r, R = r I (default 400)")
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
    parser.add_argument("--mpc_workers", type=int, default=0,
                        help="worker processes stepping the controllers (default 0: all in this process)")


def robot_state(robot) -> np.ndarray:
    """The MPC state of every env (n, 6): [X, Y, psi, X_dot, Y_dot, psi_dot] of the base_link origin, world frame."""
    import torch

    from mecanum_tracking_ws.tasks.manager_based.mecanum_tracking.mdp.traj_utils import yaw_from_quat_wxyz

    d = robot.data
    return torch.cat([d.root_link_pos_w[:, :2], yaw_from_quat_wxyz(d.root_link_quat_w)[:, None],
                      d.root_link_lin_vel_w[:, :2], d.root_link_ang_vel_w[:, 2:3]], 1).double().cpu().numpy()


def _controllers(urdf, configs):
    from mecanum_tracking_ws.mpc import params
    from mecanum_tracking_ws.mpc.lmpc import LinearMPCv2

    p = params.from_urdf(urdf)
    return p, [LinearMPCv2(p, **kw) for kw in configs]


def _step(ctrls, p, state, wheel_meas, x, preview_rows, w, u_applied):
    """One control step of the controllers ctrls (their envs' rows of x, preview_rows, w, u_applied) -> (u, infos)."""
    from mecanum_tracking_ws.mpc import wheel_dynamics
    from mecanum_tracking_ws.mpc.tables import horizon_from_rows

    if state == "wheel":
        x = wheel_dynamics.from_encoders(x, w, p) if wheel_meas == "encoder" else wheel_dynamics.from_world(x, p)
    u = np.zeros_like(u_applied)
    infos = []
    for i, ctrl in enumerate(ctrls):
        x_r, u_r = horizon_from_rows(preview_rows[i], ctrl.N)
        if state == "wheel":
            x_r = wheel_dynamics.from_world(x_r, p)
        u[i], info = ctrl.step(x[i], x_r, u_r, u_applied[i])
        info["dare_iters"] = ctrl.dare_iterations
        infos.append(info)
    return u, infos


class MPCController:
    """One LinearMPCv2 per env, fed from the env's state and the command's preview rows.

    weights: per env, a dict overriding --mpc_Q (Q, 6 values), --mpc_R (R) and --mpc_S (S) (eval_tracking.py
    --mpc_trials)."""

    def __init__(self, n_envs, robot_cfg_module, env_cfg, step_dt, args, weights=None):
        from mecanum_tracking_ws.mpc import params

        cmd = env_cfg.commands.ref_traj
        self.N, self.T = cmd.preview_steps, cmd.preview_dt
        if abs(self.T - step_dt) > 1e-9:
            raise ValueError(f"the MPC's T = {self.T} must equal the control period {step_dt}")
        self.urdf = str(robot_cfg_module.URDF_PATH)
        self.p = params.from_urdf(self.urdf)
        self.state, self.wheel_meas = args.mpc_state, args.mpc_wheel_meas
        base = dict(Q=args.mpc_Q, R=args.mpc_R, S=args.mpc_S)
        self.configs = []
        for wt in weights or [{}] * n_envs:
            wt = {**base, **wt}
            kw = dict(T=self.T, N=self.N, u_max=robot_cfg_module.WHEEL_EFFORT_LIMIT,
                      omega_max=robot_cfg_module.MOTOR_NO_LOAD_SPEED, wheel_speed_limit=args.mpc_wheel_limit,
                      terminal=args.mpc_terminal, S=None if not wt["S"] or wt["S"] <= 0 else [wt["S"]] * 4,
                      Q_path=args.mpc_Q_path, state=args.mpc_state)
            if wt["Q"] is not None:
                kw["Q"] = tuple(wt["Q"])
            if wt["R"] is not None:
                kw["R"] = (wt["R"],) * 4
            self.configs.append(kw)
        self.u_applied = np.zeros((n_envs, 4))
        self.workers = []
        if args.mpc_workers > 0:
            self._start_workers(min(args.mpc_workers, n_envs))
        else:
            _, self.ctrls = _controllers(self.urdf, self.configs)
        c = self.configs[0]
        state = "[X, Y, psi, X_dot, Y_dot, psi_dot]" if self.state == "world" else \
            f"[X, Y, psi, w1, w2, w3], wheel speeds from {self.wheel_meas}"
        weights = f"{len(weights)} per-env weight sets" if weights else \
            f"Q {c.get('Q', 'default')}, R {c.get('R', 'default')}, S = {c['S']}"
        print(f"[mpc] N = {self.N}, T = {self.T} s, terminal {args.mpc_terminal}, {weights}, "
              f"Q_path {args.mpc_Q_path}, wheel-speed limit {args.mpc_wheel_limit}; state {state}; "
              f"model m_e {self.p.m_e:.4f} kg, J_e {self.p.J_e:.5f} kg m^2; workers {len(self.workers)}")

    def _start_workers(self, k):
        env = dict(os.environ, OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1")
        for ids in np.array_split(np.arange(len(self.configs)), k):
            proc = subprocess.Popen([sys.executable, os.path.abspath(__file__)], stdin=subprocess.PIPE,
                                    stdout=subprocess.PIPE, env=env)
            pickle.dump(dict(urdf=self.urdf, configs=[self.configs[i] for i in ids], state=self.state,
                             wheel_meas=self.wheel_meas), proc.stdin)
            proc.stdin.flush()
            self.workers.append((ids, proc))

    def __call__(self, x, preview_rows, w=None):
        """x: robot_state (n, 6); preview_rows: the command's preview (n, N + 1, C); w: the encoders' wheel speeds
        (n, 4), MPC order and signs (needed for --mpc_state wheel --mpc_wheel_meas encoder)."""
        if not self.workers:
            u, infos = _step(self.ctrls, self.p, self.state, self.wheel_meas, x, preview_rows, w, self.u_applied)
        else:
            for ids, proc in self.workers:
                pickle.dump((x[ids], preview_rows[ids], None if w is None else w[ids], self.u_applied[ids]), proc.stdin)
                proc.stdin.flush()
            u, infos = np.zeros_like(self.u_applied), [None] * len(self.configs)
            for ids, proc in self.workers:
                u_k, infos_k = pickle.load(proc.stdout)
                u[ids] = u_k
                for i, info in zip(ids, infos_k):
                    infos[i] = info
        self.u_applied = u.copy()
        return u, infos

    def extra(self, infos):
        """The MPC_EXTRA columns of every env."""
        return [[info["qp_ms"], info["status"], info["slack_max"], info["dare_iters"]] for info in infos]


def _worker():
    """A worker of --mpc_workers: the controllers of a block of envs; reads (x, preview_rows, w, u_applied) from stdin,
    writes (u, the infos' log fields) to stdout. The mpc package is imported without mecanum_tracking_ws/__init__.py
    (that one registers the Isaac Lab tasks)."""
    import types

    pkg = types.ModuleType("mecanum_tracking_ws")
    pkg.__path__ = [os.path.abspath(_PKG)]
    sys.modules["mecanum_tracking_ws"] = pkg
    inp, out = sys.stdin.buffer, sys.stdout.buffer
    cfg = pickle.load(inp)
    p, ctrls = _controllers(cfg["urdf"], cfg["configs"])
    while True:
        try:
            x, preview_rows, w, u_applied = pickle.load(inp)
        except EOFError:
            return
        u, infos = _step(ctrls, p, cfg["state"], cfg["wheel_meas"], x, preview_rows, w, u_applied)
        pickle.dump((u, [{k: info[k] for k in ("qp_ms", "status", "slack_max", "dare_iters", "step_ms")}
                         for info in infos]), out)
        out.flush()


if __name__ == "__main__":
    _worker()
