"""Generates test reference trajectories (CSV) until the MPC side delivers its own files.

Shapes: circle, figure-eight (lemniscate of Gerono), rounded square, sharp square.
Columns: t, X, Y, theta, vx, vy, omega   (vx, vy in the BODY frame, as the MPC's u = (vx, vy, omega)).
Every trajectory starts at (0, 0) and is closed: the last sample repeats the first one (needed for loop=True).

Usage:
    python gen_reference_trajectories.py --out refs/ --dt 0.01 --speed 0.5 --heading constant
"""

from __future__ import annotations

import argparse
import math
import os

import numpy as np


def circle(radius: float, n: int) -> np.ndarray:
    u = np.linspace(0.0, 2.0 * math.pi, n)
    return np.stack([radius * np.sin(u), radius * (1.0 - np.cos(u))], axis=1)  # starts at (0, 0)


def figure_eight(size: float, n: int) -> np.ndarray:
    u = np.linspace(0.0, 2.0 * math.pi, n)
    return np.stack([size * np.sin(u), 0.5 * size * np.sin(2.0 * u)], axis=1)


def rounded_square(side: float, corner_radius: float, n: int) -> np.ndarray:
    """Square with circular-arc corners, built from straight + arc pieces, starting at the middle of the bottom
    edge and running counterclockwise."""
    r = corner_radius
    h = side / 2.0
    pts = []
    centers = [(h - r, -h + r), (h - r, h - r), (-h + r, h - r), (-h + r, -h + r)]
    start_angles = [-math.pi / 2, 0.0, math.pi / 2, math.pi]
    for (cx, cy), a0 in zip(centers, start_angles):
        # straight edge leading into this corner
        p0 = np.array([cx + r * math.cos(a0), cy + r * math.sin(a0)])
        pts.append(p0)
        # arc
        for a in np.linspace(a0, a0 + math.pi / 2, 50):
            pts.append(np.array([cx + r * math.cos(a), cy + r * math.sin(a)]))
    pts = np.array(pts)
    # close at the start point (middle of the bottom edge) and shift it to the origin
    start = np.array([0.0, -h])
    poly = np.vstack([start, pts, start])
    return resample_polyline(poly, n) - start


def sharp_square(side: float, n: int) -> np.ndarray:
    h = side / 2.0
    poly = np.array([[0.0, -h], [h, -h], [h, h], [-h, h], [-h, -h], [0.0, -h]])
    return resample_polyline(poly, n) - poly[0]


def resample_polyline(poly: np.ndarray, n: int) -> np.ndarray:
    """Resamples a polyline by arc length (n points, equally spaced)."""
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    sq = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(sq, s, poly[:, 0]), np.interp(sq, s, poly[:, 1])], axis=1)


def time_parametrize(path: np.ndarray, speed: float, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """Constant path speed: resample the geometric path to time samples t = 0, dt, 2dt, ...

    The number of steps is rounded so that the closing step has the same length as all the others; the
    actual speed therefore differs from ``speed`` by less than half a step over the whole path."""
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(int(round(s[-1] / (speed * dt))), 2)
    t = np.arange(n + 1) * dt
    sq = np.linspace(0.0, s[-1], n + 1)
    p = np.stack([np.interp(sq, s, path[:, 0]), np.interp(sq, s, path[:, 1])], axis=1)
    p[-1] = path[-1]  # exactly closed
    return t, p


def build_table(t: np.ndarray, p: np.ndarray, heading: str) -> np.ndarray:
    dt = t[1] - t[0]
    # world velocity by central differences (periodic: the path is closed)
    v_w = (np.roll(p[:-1], -1, axis=0) - np.roll(p[:-1], 1, axis=0)) / (2.0 * dt)
    v_w = np.vstack([v_w, v_w[:1]])
    if heading == "tangent":
        theta = np.unwrap(np.arctan2(v_w[:, 1], v_w[:, 0]))
        theta -= theta[0]  # start with heading 0
    else:  # constant heading: the mecanum strafes, it does not turn
        theta = np.zeros(len(t))
    omega = np.gradient(theta, dt)
    c, s = np.cos(theta), np.sin(theta)
    vx_b = c * v_w[:, 0] + s * v_w[:, 1]
    vy_b = -s * v_w[:, 0] + c * v_w[:, 1]
    theta_wrapped = (theta + math.pi) % (2.0 * math.pi) - math.pi
    return np.stack([t, p[:, 0], p[:, 1], theta_wrapped, vx_b, vy_b, omega], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="refs")
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--heading", choices=["constant", "tangent"], default="constant")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    shapes = {
        "circle": circle(1.0, 2000),
        "figure_eight": figure_eight(1.5, 2000),
        "rounded_square": rounded_square(2.0, 0.4, 2000),
        "sharp_square": sharp_square(2.0, 2000),
    }
    for name, path in shapes.items():
        t, p = time_parametrize(path, args.speed, args.dt)
        table = build_table(t, p, args.heading)
        fn = os.path.join(args.out, f"{name}.csv")
        np.savetxt(fn, table, delimiter=",", header="t,X,Y,theta,vx,vy,omega", comments="", fmt="%.6f")
        print(f"{fn}: {len(t)} samples, {t[-1]:.2f} s")


if __name__ == "__main__":
    main()
