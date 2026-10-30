"""Generates random TRAINING reference trajectories (CSV), kept separate from the test set.

Each trajectory is a random smooth closed curve (low-order Fourier series), time-parametrized with a speed profile
that respects the dynamic limits:
    * lateral:      v <= sqrt(a_lat_max / |kappa|)          (a_lat = v^2 * kappa)
    * tangential:   |dv/dt| <= a_tan_max                   (forward/backward pass on v^2, as in time scaling
                                                            along a path, Lynch & Park ch. 9 / Bobrow et al. 1985)
    * wheel speed (--omega_max, own robot): every reference wheel speed <= omega_max, i.e.
                    v <= omega_max R / sqrt(2)              fixed heading (worst direction: |vx_b| + |vy_b| <= sqrt(2) v)
                    v <= omega_max R / (1 + L |kappa|)      tangent heading (psi_dot = kappa v)
                    Use omega_max = (motor no-load speed) / (top of the speed curriculum), so that the fastest
                    training reference is still feasible.
Path families (--kind):
    smooth   random smooth closed curve (Fourier series, order 3), tightest turn radius --min_radius (0.2 m): the
             original training set
    polygon  random closed polygon, 3-7 vertices, turns up to 160 deg. Fixed heading: sharp corners driven at constant
             speed (the reference velocity turns between two table samples, as in test_shapes / test_corners /
             test_dynamic sharp_square). Tangent heading: every corner rounded with a small arc (--fillet_radius, shrunk
             where the sides are short), since a sharp corner would need a yaw step; the wheel-speed limit slows the
             reference down in the arcs.
    tight    random closed curve with more high-frequency content (order 4, amplitudes ~ 1/k), tightest turn radius
             between --tight_radius (0.025 .. 0.1 m, curvature 10 .. 40 1/m; test_dynamic: slalom 25, rose 36 1/m)
    sharp    polygon and tight alternately, in pairs (so each with both headings when --heading mixed)
The test shapes (circle, figure-eight, squares, slalom, rose) are NOT generated here -> no test leakage (a random
polygon is a square only by chance).

Heading (--heading): fixed (theta = 0, the robot keeps its start yaw and strafes), tangent (theta = direction of
travel), or mixed (even index fixed, odd index tangent; the file name says which).
--ramp: the speed starts and ends at 0 (one lap, not closed in time: for the evaluation sets, loop=False).
--urdf: also writes the step-averaged reference torques tau_1..4 of the MPC's inverse dynamics (mpc/tables.py).

Columns: t, X, Y, theta, vx, vy, omega [, tau_1..tau_4]; (vx, vy) in the reference's body frame.

Usage:
    # FUJI tasks (the original set, refs/train): 0.5 m/s, heading-free
    python gen_training_trajectories.py --out refs_train --n 200 --seed 0
    # comparison with the MPC (own robot, XM430, 0.1 m/s; refs/train_own and refs/test_unseen_own)
    python gen_training_trajectories.py --own --out .../refs/train_own --n 200 --seed 0
    # its sharp corners and tight turns (refs/train_own_sharp, trained on together with train_own since 2026-10-28)
    python gen_training_trajectories.py --own --kind sharp --prefix sharp --out .../refs/train_own_sharp --n 200 --seed 1
    python gen_training_trajectories.py --own --ramp --prefix unseen --out .../refs/test_unseen_own --n 20 --seed 999
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

from gen_reference_trajectories import build_table, resample_polyline

_PKG = os.path.join(os.path.dirname(__file__), "..", "..", "source", "mecanum_tracking_ws", "mecanum_tracking_ws")
URDF_OWN = os.path.join(os.path.dirname(__file__), "..", "..", "assets", "robots", "own_mecanum", "robot_measured.urdf")
CURRICULUM_TOP = 1.2
"""Top of the speed curriculum of the comparison task (tracking_env_cfg.py); --own limits the wheel speeds to
omega_max / CURRICULUM_TOP."""
KINDS = ("smooth", "polygon", "tight", "sharp")
TIGHT_CURVE = {"size": 0.4, "order": 4, "decay": 1.0}
"""random_closed_curve of the tight family: ~2.5 % of the draws have their tightest turn in 0.025 .. 0.1 m (the rest
are redrawn), 12 % of their length is tighter than 0.2 m (the smooth family's limit), ~7 m long (median)."""


def random_closed_curve(
    rng: np.random.Generator, n: int = 4000, size: float = 1.5, order: int = 3, decay: float = 2.0
) -> np.ndarray:
    """Random closed curve: x(u), y(u) = sum_k a_k cos(k u) + b_k sin(k u), k = 1..order, amplitudes ~ 1/k^decay.

    Starts at the origin (shifted). Self-intersections are allowed (like a figure-eight), they are harmless for a
    time-indexed reference."""
    u = np.linspace(0.0, 2.0 * np.pi, n)
    xy = np.zeros((n, 2))
    for k in range(1, order + 1):
        amp = size / k**decay
        coef = rng.normal(0.0, amp, size=(2, 2))
        xy[:, 0] += coef[0, 0] * np.cos(k * u) + coef[0, 1] * np.sin(k * u)
        xy[:, 1] += coef[1, 0] * np.cos(k * u) + coef[1, 1] * np.sin(k * u)
    xy[-1] = xy[0]
    return xy - xy[0]


def turn_angles(sides: np.ndarray) -> np.ndarray:
    """Signed turn [rad] at every vertex of a closed polygon, vertex i between side i - 1 and side i."""
    a = np.arctan2(sides[:, 1], sides[:, 0])
    return (a - np.roll(a, 1) + np.pi) % (2.0 * np.pi) - np.pi


def random_polygon(
    rng: np.random.Generator, n_vertices=(3, 7), box: float = 1.2, min_side: float = 0.25, max_turn: float = 160.0
) -> np.ndarray:
    """Random closed polygon (m + 1, 2), last vertex = first = origin: m vertices uniform in a box x box square, joined
    in random order (self-intersections allowed). Redrawn while a side is shorter than ``min_side`` or a turn is
    sharper than ``max_turn`` [deg] (a near reversal). Turns: median 130 deg, 10 % below 60 deg."""
    while True:
        v = rng.uniform(0.0, box, size=(rng.integers(n_vertices[0], n_vertices[1] + 1), 2))
        sides = np.diff(np.vstack([v, v[:1]]), axis=0)
        if np.linalg.norm(sides, axis=1).min() >= min_side and np.abs(turn_angles(sides)).max() <= np.radians(max_turn):
            return np.vstack([v, v[:1]]) - v[0]


def fillet_polygon(corners: np.ndarray, radius: float, chord: float = 2e-5) -> np.ndarray:
    """The closed polygon with every corner replaced by a circular arc tangent to both sides, of the given radius, or
    smaller where a side is short (each arc takes at most 45 % of a side). Starts and ends in the middle of the first
    side, so the closed curve is smooth at its seam too; shifted to the origin.
    chord: arc sampling [m], well below the arc-length step of make_trajectory (~0.2 mm): with chords as long as that
    step, curvature() overestimates the arcs' curvature up to 2x (checked numerically)."""
    sides = np.diff(corners, axis=0)
    lengths = np.linalg.norm(sides, axis=1)
    unit = sides / lengths[:, None]
    turn = turn_angles(sides)
    pts = [corners[0] + 0.5 * sides[0]]
    for j in range(1, len(sides) + 1):
        i = j % len(sides)  # the corner between side i - 1 and side i
        half = np.tan(abs(turn[i]) / 2.0)
        r = min(radius, 0.45 * min(lengths[i - 1], lengths[i]) / max(half, 1e-9))
        t1 = corners[i] - unit[i - 1] * r * half  # where the arc leaves side i - 1
        centre = t1 + r * np.sign(turn[i]) * np.array([-unit[i - 1, 1], unit[i - 1, 0]])
        n_arc = int(np.ceil(r * abs(turn[i]) / chord)) + 2
        phi = np.arctan2(t1[1] - centre[1], t1[0] - centre[0]) + np.linspace(0.0, turn[i], n_arc)
        pts += list(centre + r * np.c_[np.cos(phi), np.sin(phi)])
    pts.append(pts[0])
    path = np.array(pts)
    return path - path[0]


def curvature(p: np.ndarray, ds: float) -> np.ndarray:
    """Curvature of an arc-length-resampled closed polyline (periodic central differences)."""
    q = p[:-1]
    d1 = (np.roll(q, -1, axis=0) - np.roll(q, 1, axis=0)) / (2.0 * ds)
    d2 = (np.roll(q, -1, axis=0) - 2.0 * q + np.roll(q, 1, axis=0)) / ds**2
    k = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(np.linalg.norm(d1, axis=1) ** 3, 1e-9)
    return np.append(k, k[0])


def speed_profile(
    kappa: np.ndarray, ds: float, v_max: float, a_lat: float, a_tan: float, v_cap: np.ndarray | None = None,
    ramp: bool = False,
) -> np.ndarray:
    """v(s) = min(v_max, sqrt(a_lat / |kappa|), v_cap), then limited to |dv/dt| <= a_tan by a forward and a backward
    pass on v^2 (v_{i+1}^2 <= v_i^2 + 2 a_tan ds). Periodic: the passes are run twice around the loop.
    ramp: v = 0 at both ends instead, one forward and one backward pass (not periodic)."""
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-9)))
    if v_cap is not None:
        v = np.minimum(v, v_cap)
    n = len(v)
    if ramp:
        v[0] = v[-1] = 0.0
        for i in range(1, n):
            v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2.0 * a_tan * ds))
        for i in range(n - 2, -1, -1):
            v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2.0 * a_tan * ds))
        return v
    for _ in range(2):
        for i in range(1, 2 * n):
            a, b = (i - 1) % n, i % n
            v[b] = min(v[b], np.sqrt(v[a] ** 2 + 2.0 * a_tan * ds))
        for i in range(2 * n - 2, -1, -1):
            a, b = (i + 1) % n, i % n
            v[b] = min(v[b], np.sqrt(v[a] ** 2 + 2.0 * a_tan * ds))
    return v


def time_parametrize_profile(p: np.ndarray, v: np.ndarray, ds: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Arc-length samples with speed v -> uniform time samples (dt). The last sample closes the curve."""
    seg_t = ds / (0.5 * (v[1:] + v[:-1]))
    t_s = np.concatenate([[0.0], np.cumsum(seg_t)])
    n = max(int(round(t_s[-1] / dt)), 2)
    t = np.arange(n + 1) * dt
    tq = np.linspace(0.0, t_s[-1], n + 1)  # equal steps, closing step included
    q = np.stack([np.interp(tq, t_s, p[:, 0]), np.interp(tq, t_s, p[:, 1])], axis=1)
    q[-1] = p[-1]
    return t, q


def tangent_heading(p: np.ndarray, t: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Direction of travel at the time samples q, continuous, from the GEOMETRIC tangent of the arc-length path p
    (defined also where the speed ramps to 0)."""
    d = np.gradient(p, axis=0)
    psi_s = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
    sq = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
    return np.interp(sq * s[-1] / sq[-1], s, psi_s)


def make_trajectory(rng, dt, v_max, a_lat, a_tan, min_radius=0.2, n_arc=12000, heading="fixed", wheel=None,
                    ramp=False, model=None, T=0.05, kind="smooth", tight_radius=(0.025, 0.1),
                    fillet_radius=(0.03, 0.15), max_length=None):
    """n_arc: arc-length resolution; 3000 overshoots the acceleration limits by ~25% after time resampling,
    12000 stays within ~4% (checked numerically). Smooth curves with a tighter turn than ``min_radius`` (near-cusps,
    where the speed would drop to ~0) are redrawn; tight ones unless their tightest turn is within ``tight_radius``.
    kind polygon: sharp corners (fixed heading) or arcs of a random radius in ``fillet_radius`` (tangent heading).
    wheel = (omega_max, R, L): wheel-speed limit; model: RobotParams -> table with the reference torques.
    max_length: only the first max_length metres of the path (an open path, so ramp; the curvature is the closed
    curve's)."""
    while True:
        if kind == "polygon":
            path = random_polygon(rng)
            if heading == "tangent":
                path = fillet_polygon(path, rng.uniform(*fillet_radius))
        else:
            path = random_closed_curve(rng, **(TIGHT_CURVE if kind == "tight" else {}))
        length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
        p = resample_polyline(path, n_arc)
        ds = length / (n_arc - 1)
        kappa = curvature(p, ds)
        k_max = np.max(np.abs(kappa))
        if kind == "polygon" or (kind == "tight" and 1.0 / tight_radius[1] <= k_max <= 1.0 / tight_radius[0]) or (
            kind == "smooth" and k_max <= 1.0 / min_radius
        ):
            break
    if max_length is not None and length > max_length:
        m = int(max_length / ds) + 1
        p, kappa = p[:m], kappa[:m]
    if kind == "polygon" and heading == "fixed":
        kappa = np.zeros_like(kappa)  # the corners at constant speed: no lateral-acceleration limit there
    v_cap = None
    if wheel is not None:
        omega_max, R, L = wheel
        v_cap = omega_max * R / (1.0 + L * np.abs(kappa)) if heading == "tangent" else np.full_like(kappa, omega_max * R / np.sqrt(2.0))
    v = speed_profile(kappa, ds, v_max, a_lat, a_tan, v_cap, ramp)
    t, q = time_parametrize_profile(p, v, ds, dt)
    if model is None:
        return build_table(t, q, "tangent" if heading == "tangent" else "constant")
    from mpc import tables

    theta = tangent_heading(p, t, q) if heading == "tangent" else np.zeros(len(t))
    return tables.path_table(t, q, theta, model, T, periodic=not ramp)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="refs_train")
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--prefix", default="train")
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--v_max", type=float, default=0.5, help="nominal speed at time scale s = 1 [m/s]")
    ap.add_argument("--a_lat", type=float, default=0.5, help="max lateral acceleration at s = 1 [m/s^2]")
    ap.add_argument("--a_tan", type=float, default=0.5, help="max tangential acceleration at s = 1 [m/s^2]")
    ap.add_argument("--min_radius", type=float, default=0.2, help="tightest allowed turn radius [m]")
    ap.add_argument("--heading", choices=["fixed", "tangent", "mixed"], default="fixed")
    ap.add_argument("--kind", choices=KINDS, default="smooth", help="path family (see above)")
    ap.add_argument("--tight_radius", type=float, nargs=2, default=(0.025, 0.1),
                    help="kind tight: range of the tightest turn radius of a curve [m]")
    ap.add_argument("--fillet_radius", type=float, nargs=2, default=(0.03, 0.15),
                    help="kind polygon, tangent heading: range of the corner arc radius [m]")
    ap.add_argument("--omega_max", type=float, default=None, help="wheel-speed limit of the reference [rad/s]")
    ap.add_argument("--ramp", action="store_true", help="speed 0 at both ends (evaluation sets, loop=False)")
    ap.add_argument("--max_length", type=float, default=None,
                    help="keep the first MAX_LENGTH metres of each path (short open paths; needs --ramp)")
    ap.add_argument("--urdf", default=None, help="write the reference torques too (model of this URDF)")
    ap.add_argument("--T", type=float, default=0.05, help="MPC sampling time of the step-averaged torques [s]")
    ap.add_argument("--own", action="store_true",
                    help="comparison preset: v_max 0.1, a_tan 0.1, mixed heading, omega_max 8.06 / "
                         f"{CURRICULUM_TOP}, torques of robot_measured.urdf")
    args = ap.parse_args()
    if args.max_length is not None and not args.ramp:
        ap.error("--max_length cuts the closed path open: needs --ramp")
    model, wheel = None, None
    if args.own:
        args.v_max, args.a_tan, args.heading = 0.1, 0.1, "mixed"
        args.urdf = args.urdf or URDF_OWN
    if args.urdf:
        sys.path.insert(0, _PKG)
        from mpc import motor, params

        model = params.from_urdf(args.urdf)
        if args.own and args.omega_max is None:
            args.omega_max = motor.OMEGA_MAX / CURRICULUM_TOP
        if args.omega_max is not None:
            wheel = (args.omega_max, model.R, model.L)
    elif args.omega_max is not None:
        ap.error("--omega_max needs --urdf (wheel radius and L)")
    os.makedirs(args.out, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    for i in range(args.n):
        heading = args.heading if args.heading != "mixed" else ("fixed", "tangent")[i % 2]
        kind = args.kind if args.kind != "sharp" else ("polygon", "tight")[(i // 2) % 2]
        table = make_trajectory(rng, args.dt, args.v_max, args.a_lat, args.a_tan, args.min_radius, heading=heading,
                                wheel=wheel, ramp=args.ramp, model=model, T=args.T, kind=kind,
                                tight_radius=args.tight_radius, fillet_radius=args.fillet_radius,
                                max_length=args.max_length)
        name = (f"{args.prefix}_{i:03d}" + (f"_{kind}" if args.kind == "sharp" else "")
                + (f"_{heading}" if args.heading == "mixed" else ""))
        header = "t,X,Y,theta,vx,vy,omega" + (",tau_1,tau_2,tau_3,tau_4" if model is not None else "")
        np.savetxt(os.path.join(args.out, f"{name}.csv"), table, delimiter=",", header=header, comments="", fmt="%.6f")
    print(f"{args.n} {args.kind} trajectories -> {args.out}  (limits at s=1: v<={args.v_max}, a_lat<={args.a_lat}, "
          f"a_tan<={args.a_tan}, wheel speed<={args.omega_max}; heading {args.heading}; ramp {args.ramp}; "
          f"torques {model is not None}; with time scale s the accelerations scale with s^2)")


if __name__ == "__main__":
    main()
