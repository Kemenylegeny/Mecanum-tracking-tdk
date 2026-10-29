"""Corner test set: sharp corners vs. a smooth path, for the MPC ablation without the torque-change penalty S and the
soft wheel-speed limit (the plain LTV-MPC of the literature: Q, R, terminal weight, hard torque box). Both terms can
only matter where the reference torque or the wheel speeds change fast, so the set pairs sudden direction changes with
a low-curvature path driven at the same speed:

    sharp_square    square, side 0.6 m (= test_dynamic/sharp_square), four sharp 90 deg corners
    straight_turn   1.0 m straight, then a sharp 135 deg turn (zero radius), 0.5 m straight: steady state before the
                    corner, and a larger direction change than the square's
    smooth          y = A (1 - cos(2 pi x / lambda)), A = 0.15 m, lambda = 1.5 m, 1.5 periods: curvature <= 2.6 1/m
                    (radius >= 0.38 m), centripetal acceleration <= 0.15 m/s^2 even at the fast speed

The corners are driven without slowing down for the turn (the reference velocity turns between two table samples);
only the speed magnitude is limited, |dv/dt| <= --a_max (start, stop, and where the direction-dependent limit drops,
e.g. into the diagonal after the 135 deg turn). Fixed heading.

Speed: --speed v (constant path speed, e.g. 0.1 m/s = the comparison's speed) or --alpha * v_max(direction) (the
fastest wheel at alpha * 8.06 rad/s, as gen_dynamic_references.py).

Columns: t, X, Y, theta, vx, vy, omega, tau_1..tau_4 (mpc/tables.py path_table).

    python scripts/tools/gen_corner_references.py --speed 0.1 --out $R/test_corners_slow
    python scripts/tools/gen_corner_references.py --alpha 0.9 --out $R/test_corners_fast
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from gen_dynamic_references import (REF_DIR, URDF, arc_length, dense_path, directions, forward_backward,  # noqa: E402
                                    time_law, wheel_per_speed)
from mpc import motor, params, tables  # noqa: E402  (on sys.path through gen_dynamic_references)

SHAPES = ("sharp_square", "straight_turn", "smooth")


def polyline(corners, n: int) -> np.ndarray:
    """Densely sampled polyline (n, 2) with the corners exactly on the grid."""
    corners = np.asarray(corners, dtype=float)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(corners, axis=0), axis=1))]
    sq = np.unique(np.r_[np.linspace(0.0, s[-1], n), s])
    return np.c_[np.interp(sq, s, corners[:, 0]), np.interp(sq, s, corners[:, 1])]


def corner_path(shape: str, n: int = 200001) -> np.ndarray:
    """Densely sampled geometric path (n, 2), starting at the origin along +x."""
    if shape == "sharp_square":
        return dense_path("sharp_square", n)
    if shape == "straight_turn":
        turn = np.radians(135.0)
        return polyline([[0.0, 0.0], [1.0, 0.0], [1.0 + 0.5 * np.cos(turn), 0.5 * np.sin(turn)]], n)
    if shape == "smooth":
        a, lam, periods = 0.15, 1.5, 1.5
        x = np.linspace(0.0, periods * lam, n)
        return np.c_[x, a * (1 - np.cos(2 * np.pi * x / lam))]
    raise ValueError(shape)


def corner_table(shape, p, speed, alpha, a_max, dt, T):
    path = corner_path(shape)
    s = arc_length(path)
    phi = directions(path)
    if speed is not None:
        v_lim = np.full_like(s, speed)
    else:
        v_lim = alpha * motor.OMEGA_MAX / wheel_per_speed(phi, np.zeros_like(s), p)
    t, sq = time_law(s, forward_backward(s, v_lim, a_max), dt)
    xy = np.c_[np.interp(sq, s, path[:, 0]), np.interp(sq, s, path[:, 1])]
    return tables.path_table(t, xy, np.zeros_like(t), p, T, periodic=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=None, help="constant path speed [m/s] (default: --alpha * v_max)")
    ap.add_argument("--alpha", type=float, default=0.9, help="share of the direction-dependent speed limit v_max")
    ap.add_argument("--a_max", type=float, default=0.3, help="limit on the change of the speed magnitude [m/s^2]")
    ap.add_argument("--shapes", nargs="+", default=list(SHAPES), choices=list(SHAPES))
    ap.add_argument("--dt", type=float, default=0.01, help="table sampling time [s]")
    ap.add_argument("--T", type=float, default=0.05, help="MPC sampling time [s] of the step-averaged torques")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    p = params.from_urdf(args.urdf)
    os.makedirs(args.out, exist_ok=True)
    for shape in args.shapes:
        table = corner_table(shape, p, args.speed, args.alpha, args.a_max, args.dt, args.T)
        tables.save_table(os.path.join(args.out, f"{shape}.csv"), table)
        v_w = np.c_[np.cos(table[:, 3]) * table[:, 4] - np.sin(table[:, 3]) * table[:, 5],
                    np.sin(table[:, 3]) * table[:, 4] + np.cos(table[:, 3]) * table[:, 5]]
        acc = np.linalg.norm(np.gradient(v_w, args.dt, axis=0), axis=1)
        wheel = np.abs(tables.table_wheel_speeds(table, p)).max()
        print(f"{shape}: {table[-1, 0]:.2f} s, length {np.sum(np.linalg.norm(np.diff(table[:, 1:3], axis=0), axis=1)):.2f}"
              f" m, v max {np.hypot(*v_w.T).max():.3f} m/s, |a| max {acc.max():.2f} m/s^2 (99th pct "
              f"{np.percentile(acc, 99):.2f}), max |u_r| {np.abs(table[:, 7:11]).max():.4f} N m, max |du_r| "
              f"{np.abs(np.diff(table[::int(round(args.T / args.dt)), 7:11], axis=0)).max():.4f} N m/step, "
              f"wheel max {wheel:.2f} rad/s (limit {motor.OMEGA_MAX:.2f})")


if __name__ == "__main__":
    main()
