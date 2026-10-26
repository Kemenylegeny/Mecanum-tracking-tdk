"""Evaluation metrics for the RL vs. MPC comparison (node 7). Pure numpy, no Isaac Lab -- the SAME script runs on the
RL logs and on the MPC logs, so the metrics are computed identically for both (comparison principle).

Common per-run log format (CSV with header, one row per control step; scripts/rsl_rl/eval_tracking.py writes it for
both controllers):
    t                   time [s]
    x, y, yaw           robot pose (world, relative to the env origin) [m, m, rad]
    vx, vy, wz          robot velocity: (vx, vy) in the WORLD frame [m/s], yaw rate [rad/s]
    x_ref, y_ref        reference position [m]
    yaw_ref             reference heading [rad]                                   (optional, older logs lack it)
    vx_ref, vy_ref      reference velocity (WORLD frame) [m/s]
    wz_ref              reference yaw rate [rad/s]                                (optional)
    u_0 .. u_{m-1}      controller output: the commanded wheel torques [Nm] (MPC wheel order and signs)
    tau_0 .. tau_3      wheel torques the motor model applied [Nm] (the torque-speed curve may cut u)
    w_0 .. w_3          wheel speeds [rad/s]                                      (optional)
    compute_ms          controller computation time for this step [ms] (CPU, one env, same machine for both)
    failed              1 from the step where the robot lost the trajectory (deviation termination), else 0
    qp_ms, status, ...  controller-specific extras (MPC), ignored here except the solver status

Window (as the MPC side's evaluate.py): from --skip seconds to the end of the reference motion (the last row with a
nonzero reference velocity); the rows after a failure are dropped.
    pos           distance to where the reference is at that time
    cross_track   distance to the reference path (polyline of the logged reference points)
    contour, lag  the position error split normal / along the reference velocity
    heading       wrap(yaw - yaw_ref)
    omega_exceed  wheel speed above omega_max (8.06 rad/s, XM430): largest excess and share of the steps
    du            ||u_k - u_k-1|| (smoothness; the MPC's S and the RL's torque-rate penalty act on it)

Usage:
    python metrics.py run1.csv [run2.csv ...]                      -> metrics per run
    python metrics.py --group RL rl_seed*/circle.csv --group MPC mpc/circle.csv   -> mean +- std per group
    python metrics.py --table logs/eval/mpc/test_fixed/*.csv       -> one line per run (mm, deg)
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

OMEGA_MAX = 8.06
"""Wheel speed limit [rad/s]: XM430-W210-T no-load speed at 12 V (mpc/motor.py)."""
FAILED_STATUS = (1, 2)
"""MPC step status codes of a failed solve (mpc/conventions.py: fallback, stopped)."""


def load_log(path: str) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    return {name: np.atleast_1d(np.asarray(data[name], dtype=float)) for name in data.dtype.names}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def contour_lag(log: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Splits the position error into a component orthogonal to the path (contouring) and one along it (lag),
    using the reference velocity direction as the path tangent (Liniger et al. 2015, Eq. 11; Lam et al. 2010).

    lag > 0: the robot is BEHIND the reference point. contour: signed, left of the path positive."""
    ex = log["x"] - log["x_ref"]
    ey = log["y"] - log["y_ref"]
    phi = np.arctan2(log["vy_ref"], log["vx_ref"])
    contour = -np.sin(phi) * ex + np.cos(phi) * ey
    lag = -(np.cos(phi) * ex + np.sin(phi) * ey)
    return contour, lag


def dist_to_polyline(q: np.ndarray, P: np.ndarray) -> np.ndarray:
    """Distance from each point of q (n x 2) to the polyline through P (m x 2) (the MPC side's evaluate.py)."""
    A, B = P[:-1], P[1:]
    AB = B - A
    L2 = np.maximum((AB**2).sum(1), 1e-18)
    out = np.empty(len(q))
    for i, pt in enumerate(q):
        s = np.clip(((pt - A) * AB).sum(1) / L2, 0.0, 1.0)
        out[i] = np.min(np.hypot(*(A + s[:, None] * AB - pt).T))
    return out


def window(log: dict[str, np.ndarray], skip: float = 0.0) -> np.ndarray:
    """Rows from ``skip`` to the end of the reference motion, before any failure."""
    vel = [log[k] for k in ("vx_ref", "vy_ref", "wz_ref") if k in log]
    moving = np.abs(np.stack(vel, 1)).max(axis=1) > 1e-9
    t_end = log["t"][moving].max() if moving.any() else log["t"][-1]
    ok = log["failed"] < 0.5 if "failed" in log else np.ones_like(log["t"], dtype=bool)
    return (log["t"] >= skip) & (log["t"] <= t_end) & ok


def _cols(log, prefix):
    keys = sorted((k for k in log if k.startswith(prefix) and k[len(prefix):].isdigit()), key=lambda k: int(k[len(prefix):]))
    return np.stack([log[k] for k in keys], axis=1) if keys else None


def run_metrics(log: dict[str, np.ndarray], skip: float = 0.0, omega_max: float = OMEGA_MAX) -> dict[str, float]:
    m_ = window(log, skip)
    e = np.hypot(log["x"] - log["x_ref"], log["y"] - log["y_ref"])[m_]
    contour, lag = contour_lag(log)
    contour, lag = contour[m_], lag[m_]
    ref_path = np.c_[log["x_ref"], log["y_ref"]][log["t"] <= log["t"][m_].max()]
    ct = dist_to_polyline(np.c_[log["x"], log["y"]][m_], ref_path) if len(ref_path) > 1 else np.zeros(1)
    u = _cols(log, "u_")[m_]
    du = np.linalg.norm(np.diff(u, axis=0), axis=1)
    # how the heading is used: rotation and strafing (body-frame lateral speed share)
    c, s = np.cos(log["yaw"]), np.sin(log["yaw"])
    v_body_y = (-s * log["vx"] + c * log["vy"])[m_]
    speed = np.hypot(log["vx"], log["vy"])[m_]
    m = {
        "pos_rmse_m": float(np.sqrt(np.mean(e**2))),
        "pos_max_m": float(np.max(e)),
        "cross_track_rmse_m": float(np.sqrt(np.mean(ct**2))),
        "cross_track_max_m": float(np.max(ct)),
        "contour_rmse_m": float(np.sqrt(np.mean(contour**2))),
        "contour_max_m": float(np.max(np.abs(contour))),
        "lag_mean_m": float(np.mean(lag)),
        "lag_rmse_m": float(np.sqrt(np.mean(lag**2))),
        "lag_max_m": float(np.max(np.abs(lag))),
        "action_rate_mean": float(np.mean(du)) if len(du) else 0.0,
        "action_rate_max": float(np.max(du)) if len(du) else 0.0,
        "yaw_rate_abs_mean": float(np.mean(np.abs(log["wz"][m_]))),
        "strafe_share": float(np.sum(np.abs(v_body_y)) / max(np.sum(speed), 1e-9)),
        "failed": float(np.any(log["failed"] > 0.5)) if "failed" in log else 0.0,
    }
    if "yaw_ref" in log:
        de = wrap(log["yaw"] - log["yaw_ref"])[m_]
        m["heading_rmse_rad"] = float(np.sqrt(np.mean(de**2)))
        m["heading_max_rad"] = float(np.max(np.abs(de)))
    tau = _cols(log, "tau_")
    if tau is not None:
        tau = tau[m_]
        m["torque_rms_Nm"] = float(np.sqrt(np.mean(tau**2)))
        m["torque_max_Nm"] = float(np.max(np.abs(tau)))
    w = _cols(log, "w_")
    if w is not None:
        w = np.abs(w[m_])
        m["wheel_speed_max"] = float(np.max(w))
        m["omega_exceed_max"] = float(max(np.max(w) - omega_max, 0.0))
        m["omega_exceed_share"] = float(np.mean(np.any(w > omega_max, axis=1)))
    if "compute_ms" in log:
        m["compute_ms_mean"] = float(np.mean(log["compute_ms"][m_]))
        m["compute_ms_p95"] = float(np.percentile(log["compute_ms"][m_], 95))
        m["compute_ms_p99"] = float(np.percentile(log["compute_ms"][m_], 99))
    if "status" in log:
        m["solver_failures"] = float(np.isin(log["status"][m_], FAILED_STATUS).sum())
    return m


def settling_time(log: dict[str, np.ndarray], pos_tol: float = 0.01, yaw_tol: float = np.radians(2.0),
                  hold: float = 2.0) -> tuple[float, float, float]:
    """Recovery from an initial error: the first time from which the position error stays below ``pos_tol`` [m] and
    the heading error below ``yaw_tol`` [rad] for ``hold`` seconds (a short excursion later, e.g. a sharp corner, does
    not undo it). Returns (settling time [s], RMS and max of the position error from then on [m]); NaNs if never."""
    m_ = window(log)
    t = log["t"][m_]
    e = np.hypot(log["x"] - log["x_ref"], log["y"] - log["y_ref"])[m_]
    ok = (e < pos_tol) & (np.abs(wrap(log["yaw"] - log["yaw_ref"]))[m_] < yaw_tol)
    n = max(int(round(hold / (t[1] - t[0]))), 1)
    for i in range(len(ok) - n + 1):
        if ok[i : i + n].all():
            return float(t[i]), float(np.sqrt(np.mean(e[i:] ** 2))), float(e[i:].max())
    return np.nan, np.nan, np.nan


def bootstrap_ci(x: np.ndarray, n_boot: int = 10000, alpha: float = 0.05, seed: int = 0) -> tuple[float, float]:
    """Percentile bootstrap CI of the mean over runs (e.g. RL training seeds). With few runs (3-5) the interval is
    WIDE -- report it anyway (Agarwal et al. 2021 recommend interval estimates; ~10 runs for reliable ones)."""
    rng = np.random.default_rng(seed)
    means = rng.choice(x, size=(n_boot, len(x)), replace=True).mean(axis=1)
    return float(np.percentile(means, 100 * alpha / 2)), float(np.percentile(means, 100 * (1 - alpha / 2)))


def table_line(path: str, m: dict[str, float]) -> str:
    deg = np.degrees
    return (f"{os.path.basename(path)[:28]:28} | {1e3 * m['pos_rmse_m']:7.2f} / {1e3 * m['pos_max_m']:7.2f} | "
            f"{1e3 * m['cross_track_rmse_m']:7.2f} / {1e3 * m['cross_track_max_m']:7.2f} | "
            f"{deg(m.get('heading_rmse_rad', np.nan)):5.2f} / {deg(m.get('heading_max_rad', np.nan)):6.2f} | "
            f"{m.get('torque_rms_Nm', np.nan):.4f} / {m.get('torque_max_Nm', np.nan):.4f} | "
            f"{m['action_rate_max']:.4f} | {m.get('wheel_speed_max', np.nan):5.2f} {100 * m.get('omega_exceed_share', 0):4.1f}% | "
            f"{m.get('compute_ms_mean', np.nan):6.2f} / {m.get('compute_ms_p95', np.nan):6.2f} | {int(m['failed'])}")


TABLE_HEADER = (f"{'run':28} | pos RMS/max mm  | cross-track mm  | psi RMS/max deg | torque RMS/max Nm | du max | "
                f"wheel max, >omega | compute ms mean/p95 | failed")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*")
    ap.add_argument("--group", nargs="+", action="append", metavar=("NAME", "LOG"))
    ap.add_argument("--table", action="store_true", help="one line per log (mm, deg)")
    ap.add_argument("--skip", type=float, default=0.0, help="ignore the first seconds")
    ap.add_argument("--omega_max", type=float, default=OMEGA_MAX)
    args = ap.parse_args(argv)
    if args.group:
        for name, *files in args.group:
            ms = [run_metrics(load_log(f), args.skip, args.omega_max) for f in files]
            print(f"== {name} ({len(ms)} runs)")
            for k in ms[0]:
                x = np.array([m[k] for m in ms])
                line = f"  {k:20s} {x.mean():10.4f} +- {x.std(ddof=1) if len(x) > 1 else 0.0:.4f}"
                if len(x) > 1:
                    lo, hi = bootstrap_ci(x)
                    line += f"   95% CI [{lo:.4f}, {hi:.4f}]"
                print(line)
    if args.table and args.logs:
        print(TABLE_HEADER)
        for f in args.logs:
            print(table_line(f, run_metrics(load_log(f), args.skip, args.omega_max)))
        return
    for f in args.logs:
        print(f"== {f}")
        for k, v in run_metrics(load_log(f), args.skip, args.omega_max).items():
            print(f"  {k:20s} {v:10.4f}")


if __name__ == "__main__":
    sys.exit(main())
