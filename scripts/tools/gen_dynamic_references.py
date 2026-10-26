"""Dynamic test set: fast references near the wheel-speed limit, with large accelerations, so that the dynamics (inertia,
torque) matter, unlike the 0.1 m/s test sets where the reference torques stay below the roller rolling resistance.

Speed limit of the robot (XM430 no-load speed 8.06 rad/s, rolling constraints W of mpc/params.py): the path speed at
which the fastest wheel reaches omega_max for the current direction of travel,
    v_max = omega_max / max_i |W_i (cos phi_b, sin phi_b, dpsi/ds)|        (phi_b: direction of travel in the body frame)
= 0.244 m/s along the body axes, 0.173 m/s diagonally with a fixed heading. The curves are driven at --alpha * v_max
(forward-backward pass, |dv/dt| <= --a_max), starting and ending at rest.

Paths (all start at the origin; the anchor places them on the robot):
    circle_small   circle, R = 0.15 m, two laps (constant curvature 6.7 1/m)
    slalom         y = A sin(2 pi x / lambda), A = 0.1 m, lambda = 0.4 m, three periods (alternating curvature, up to
                   25 1/m)
    rose           three-petal rose r = a cos 3 theta, a = 0.28 m, from the centre (hairpins at the tips, 36 1/m)
    sharp_square   square, side 0.6 m, sharp 90 deg corners driven at constant speed (velocity step at each corner)
    stop_go        straight line: forward / backward / forward sprints at alpha * 0.244 m/s, each started and stopped
                   with a harder acceleration: 1 m/s^2, 3 m/s^2, a step (within one table sample)
Heading fixed (default) or tangent; tangent lowers v_max on curves (the yaw rate also turns the wheels).

Columns: t, X, Y, theta, vx, vy, omega, tau_1..tau_4 (mpc/tables.py path_table: inverse dynamics on the samples,
step-averaged over the MPC's T).

    python scripts/tools/gen_dynamic_references.py            # -> refs/test_dynamic/
"""

import argparse
import os
import sys

import numpy as np

_PKG = os.path.join(os.path.dirname(__file__), "..", "..", "source", "mecanum_tracking_ws", "mecanum_tracking_ws")
sys.path.insert(0, _PKG)
from mpc import motor, params, tables  # noqa: E402

REF_DIR = os.path.join(_PKG, "tasks", "manager_based", "mecanum_tracking", "refs")
URDF = os.path.join(os.path.dirname(__file__), "..", "..", "assets", "robots", "own_mecanum", "robot_measured.urdf")
SHAPES = ("circle_small", "slalom", "rose", "sharp_square", "stop_go")
REST = 0.1
"""Rest at the end of every table [s]."""


def dense_path(shape: str, n: int = 200001) -> np.ndarray:
    """Densely sampled geometric path (n, 2), starting at the origin."""
    if shape == "circle_small":
        r, th = 0.15, np.linspace(0.0, 4 * np.pi, n)
        return np.c_[r * np.sin(th), r * (1 - np.cos(th))]
    if shape == "slalom":
        a, lam, periods = 0.1, 0.4, 3.0
        x = np.linspace(0.0, periods * lam, n)
        return np.c_[x, a * np.sin(2 * np.pi * x / lam)]
    if shape == "rose":
        a = 0.28
        th = np.linspace(np.pi / 6, np.pi / 6 + np.pi, n)  # r = 0 at both ends: starts and ends in the centre
        r = a * np.cos(3 * th)
        return np.c_[r * np.cos(th), r * np.sin(th)]
    if shape == "sharp_square":
        side = 0.6
        h = side / 2
        corners = np.array([[0, 0], [h, 0], [h, side], [-h, side], [-h, 0], [0, 0]], dtype=float)
        seg = np.linalg.norm(np.diff(corners, axis=0), axis=1)
        s = np.r_[0.0, np.cumsum(seg)]
        sq = np.unique(np.r_[np.linspace(0.0, s[-1], n), s])  # the corners exactly on the grid
        return np.c_[np.interp(sq, s, corners[:, 0]), np.interp(sq, s, corners[:, 1])]
    raise ValueError(shape)


def arc_length(path: np.ndarray) -> np.ndarray:
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]


def directions(path: np.ndarray) -> np.ndarray:
    """Direction of travel of every sample: forward differences, so a sharp corner already points along the new side."""
    d = np.diff(path, axis=0)
    phi = np.arctan2(d[:, 1], d[:, 0])
    return np.unwrap(np.r_[phi, phi[-1]])


def wheel_per_speed(phi_b: np.ndarray, dpsi_ds: np.ndarray, p) -> np.ndarray:
    """Fastest wheel speed per unit path speed [rad/m]: max_i |W_i (cos phi_b, sin phi_b, dpsi/ds)|."""
    return np.abs(np.c_[np.cos(phi_b), np.sin(phi_b), dpsi_ds] @ p.W.T).max(axis=1)


def forward_backward(s: np.ndarray, v_lim: np.ndarray, a_max: float) -> np.ndarray:
    """Fastest speed profile v(s) <= v_lim with |dv/dt| <= a_max, at rest at both ends (v^2 changes by <= 2 a ds)."""
    v = v_lim.copy()
    v[0] = v[-1] = 0.0
    ds = np.diff(s)
    for i in range(1, len(v)):
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a_max * ds[i - 1]))
    for i in range(len(v) - 2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_max * ds[i]))
    return v


def sample_times(t_end: float, dt: float) -> np.ndarray:
    """t = 0, dt, ... past t_end by REST: the last rows are at rest, so their (differenced) velocity is exactly 0 and
    the command, which holds the last row after the end (loop=False), holds a standing reference."""
    return np.arange(int(np.ceil(t_end / dt)) + 1 + int(round(REST / dt))) * dt


def time_law(s: np.ndarray, v: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Sample times and the arc length s(t) of a speed profile v(s) (constant acceleration per cell)."""
    t_s = np.r_[0.0, np.cumsum(2 * np.diff(s) / np.maximum(v[1:] + v[:-1], 1e-12))]
    t = sample_times(t_s[-1], dt)
    return t, np.interp(t, t_s, s)


def curve_table(shape, p, alpha, a_max, heading, dt, T):
    path = dense_path(shape)
    s = arc_length(path)
    phi = directions(path)
    if heading == "tangent":
        psi = phi - phi[0]
        dpsi_ds = np.gradient(psi, s)
    else:
        psi, dpsi_ds = np.zeros_like(s), np.zeros_like(s)
    v_lim = alpha * motor.OMEGA_MAX / wheel_per_speed(phi - psi, dpsi_ds, p)
    t, sq = time_law(s, forward_backward(s, v_lim, a_max), dt)
    xy = np.c_[np.interp(sq, s, path[:, 0]), np.interp(sq, s, path[:, 1])]
    return tables.path_table(t, xy, np.interp(sq, s, psi), p, T, periodic=False)


def stop_go_table(p, alpha, dt, T, length=0.7, pause=0.5, accels=(1.0, 3.0, None)):
    """Straight sprints along x (forward, backward, forward, ...) at alpha * v_max, trapezoidal speed with the given
    accelerations (None: a speed step within one sample), a pause at rest between them."""
    v = alpha * motor.OMEGA_MAX * p.R  # body axis: one wheel speed per unit speed = 1 / R
    t_parts, x_parts, x0, t0 = [], [], 0.0, 0.0
    for k, acc in enumerate(accels):
        a = acc if acc is not None else v / (dt / 4)  # step: the ramp falls between two table samples
        t_ramp = v / a
        t_cruise = (length - v * t_ramp) / v
        dur = 2 * t_ramp + t_cruise
        tk = np.arange(0.0, dur, dt / 20)
        d = np.where(tk < t_ramp, a * tk ** 2 / 2,
                     np.where(tk < t_ramp + t_cruise, v * t_ramp / 2 + v * (tk - t_ramp),
                              length - a * np.maximum(dur - tk, 0.0) ** 2 / 2))
        sign = 1.0 if k % 2 == 0 else -1.0
        t_parts += [t0 + tk, [t0 + dur, t0 + dur + pause]]
        x_parts += [x0 + sign * d, [x0 + sign * length] * 2]
        x0, t0 = x0 + sign * length, t0 + dur + pause
    tf, xf = np.concatenate(t_parts), np.concatenate(x_parts)
    t = sample_times(t0 - pause, dt)
    xy = np.c_[np.interp(t, tf, xf), np.zeros_like(t)]
    return tables.path_table(t, xy, np.zeros_like(t), p, T, periodic=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--alpha", type=float, default=0.9, help="share of the direction-dependent speed limit v_max")
    ap.add_argument("--a_max", type=float, default=1.5, help="tangential acceleration limit on the curves [m/s^2]")
    ap.add_argument("--heading", choices=["fixed", "tangent"], default="fixed")
    ap.add_argument("--shapes", nargs="+", default=list(SHAPES), choices=list(SHAPES))
    ap.add_argument("--dt", type=float, default=0.01, help="table sampling time [s]")
    ap.add_argument("--T", type=float, default=0.05, help="MPC sampling time [s] of the step-averaged torques")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--out", default=os.path.join(REF_DIR, "test_dynamic"))
    args = ap.parse_args()

    p = params.from_urdf(args.urdf)
    os.makedirs(args.out, exist_ok=True)
    for shape in args.shapes:
        if shape == "stop_go":
            table = stop_go_table(p, args.alpha, args.dt, args.T)
        else:
            table = curve_table(shape, p, args.alpha, args.a_max, args.heading, args.dt, args.T)
        tables.save_table(os.path.join(args.out, f"{shape}.csv"), table)
        speed = np.hypot(table[:, 4], table[:, 5])
        acc = np.linalg.norm(np.gradient(np.c_[np.cos(table[:, 3]) * table[:, 4] - np.sin(table[:, 3]) * table[:, 5],
                                               np.sin(table[:, 3]) * table[:, 4] + np.cos(table[:, 3]) * table[:, 5]],
                                         args.dt, axis=0), axis=1)
        wheel = np.abs(tables.table_wheel_speeds(table, p)).max()
        print(f"{shape}: {table[-1, 0]:.2f} s, length {np.sum(np.linalg.norm(np.diff(table[:, 1:3], axis=0), axis=1)):.2f}"
              f" m, v max {speed.max():.3f} m/s, |a| max {acc.max():.2f} m/s^2 (99th pct {np.percentile(acc, 99):.2f}),"
              f" max |u_r| {np.abs(table[:, 7:11]).max():.4f} N m, wheel max {wheel:.2f} rad/s "
              f"(limit {motor.OMEGA_MAX:.2f})")


if __name__ == "__main__":
    main()
