"""Reference tables shared by the RL policy and the MPC (docs/OSSZEHANGOLAS.md, 3.7).

Table columns (the RL's CSV format, mdp/traj_utils.py):
    t, X, Y, theta, vx, vy, omega, tau_1, tau_2, tau_3, tau_4
    theta      reference heading psi_r, continuous (not wrapped)
    vx, vy     reference velocity in the reference's BODY frame (rotated by theta); world: R(theta) (vx, vy)
    omega      psi_r_dot
    tau_j      step-averaged reference torque of wheel j (MPC numbering: front-left, front-right, rear-left,
               rear-right) over [t, t + T]: exactly the u_r,0 the MPC uses for the step starting at t (decision D3)

Two sources:
    reference_table   the MPC side's analytic Reference (circle, ellipse, figure8, square, cosine ramps); the
                      torques come from its own step_torques, so they are bit-for-bit the MPC's u_r
    path_table        any sampled path (the random training / unseen curves); the same inverse dynamics as
                      reference.py on the samples (finite differences): w_r = [m_e R(psi)^T (a - kappa psi_dot S v),
                      J_e psi_ddot], u_r = G+ w_r, step averages of the running integral
"""
import numpy as np

from .params import RobotParams
from .reference import S_MAT

HEADER = "t,X,Y,theta,vx,vy,omega,tau_1,tau_2,tau_3,tau_4"


def _body(theta, v_w):
    c, s = np.cos(theta), np.sin(theta)
    return np.c_[c * v_w[:, 0] + s * v_w[:, 1], -s * v_w[:, 0] + c * v_w[:, 1]]


def reference_table(ref, dt, T):
    """Table of an (anchored or not) Reference over [0, T_end] on a dt grid; T = the MPC sampling time."""
    if ref.loop:
        raise ValueError("export one lap: Reference(loop=False)")
    if ref.anchor_pose is None:
        ref.anchor((0.0, 0.0, 0.0))
    n = int(round(ref.T_end / dt))
    t = np.arange(n + 1) * dt
    x = ref.states(t)
    tau = ref.step_torques_at(t, T)
    return np.c_[t, x[:, 0:2], x[:, 2], _body(x[:, 2], x[:, 3:5]), x[:, 5], tau]


def _diff(y, dt, periodic):
    """d/dt of samples y (n, ...) by central differences; periodic: y[-1] == y[0] closes the curve."""
    if not periodic:
        return np.gradient(y, dt, axis=0)
    q = y[:-1]
    d = (np.roll(q, -1, axis=0) - np.roll(q, 1, axis=0)) / (2.0 * dt)
    return np.concatenate([d, d[:1]])


def step_average(t, U, T, periodic):
    """(U(t + T) - U(t)) / T for every sample time; U (n, m) is a running integral on the grid t.
    Periodic: U continues by its gain per lap; otherwise U is held after the end (the reference is at rest)."""
    tq = t + T
    if periodic:
        period = t[-1] - t[0]
        k = np.floor((tq - t[0]) / period)
        Uq = np.stack([np.interp(tq - k * period, t, U[:, j]) for j in range(U.shape[1])], 1) + k[:, None] * (U[-1] - U[0])
    else:
        Uq = np.stack([np.interp(np.minimum(tq, t[-1]), t, U[:, j]) for j in range(U.shape[1])], 1)
    return (Uq - U) / T


def heading_rate(t, theta, periodic):
    """psi_dot of a sampled continuous heading. Periodic: a closed curve whose heading may gain whole turns per lap
    (tangent heading on a loop): the linear part is removed before differencing across the closing point."""
    dt = t[1] - t[0]
    th = np.unwrap(theta)
    if not periodic:
        return _diff(th, dt, False)
    gain = th[-1] - th[0]
    return _diff(th - np.linspace(0.0, gain, len(th)), dt, True) + gain / (t[-1] - t[0])


def path_torques(t, xy, theta, p: RobotParams, T, periodic):
    """Step-averaged reference torques (n, 4) of a sampled path: positions xy (n, 2), heading theta (n,)."""
    dt = t[1] - t[0]
    v = _diff(xy, dt, periodic)
    a = _diff(v, dt, periodic)
    psid = heading_rate(t, theta, periodic)
    rhs = a - p.kappa * psid[:, None] * (v @ S_MAT.T)
    F = p.m_e * _body(theta, rhs)
    # running integral of the force part (trapezoid); the torque part integrates exactly: J_e (psi_dot - psi_dot_0)
    UF = np.r_[np.zeros((1, 2)), np.cumsum((F[1:] + F[:-1]) / 2 * dt, axis=0)]
    U = np.c_[UF, p.J_e * (psid - psid[0])] @ p.G_pinv.T
    return step_average(t, U, T, periodic)


def path_table(t, xy, theta, p: RobotParams, T, periodic):
    """Table of a sampled path (the velocities by central differences, periodic for closed curves)."""
    theta = np.unwrap(theta)
    v_w = _diff(xy, t[1] - t[0], periodic)
    tau = path_torques(t, xy, theta, p, T, periodic)
    return np.c_[t, xy, theta, _body(theta, v_w), heading_rate(t, theta, periodic), tau]


def table_wheel_speeds(table, p: RobotParams):
    """Reference wheel speeds (n, 4) [rad/s] of a table: W (vx, vy, omega)."""
    return table[:, 4:7] @ p.W.T


def save_table(path, table):
    np.savetxt(path, table, delimiter=",", header=HEADER, comments="", fmt="%.6f")


def horizon_from_rows(rows, N):
    """The MPC's horizon from N+1 table rows at t, t + T, ..., t + NT (world frame, as TrajectoryCommand.preview_w):
    x_r (N+1, 6) = [X, Y, psi, X_dot, Y_dot, psi_dot] with the heading unwrapped along the horizon (the rows' theta
    may be wrapped; a 2 pi jump inside the horizon would look like a full turn to the QP), u_r (N, 4) = tau columns."""
    rows = np.asarray(rows, dtype=float)
    th = np.unwrap(rows[:, 3])
    c, s = np.cos(th), np.sin(th)
    x_r = np.c_[rows[:, 1:3], th, c * rows[:, 4] - s * rows[:, 5], s * rows[:, 4] + c * rows[:, 5], rows[:, 6]]
    return x_r, rows[:N, 7:11]
