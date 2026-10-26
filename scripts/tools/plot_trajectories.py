"""Draws the reference trajectories (small multiples): one panel per file, all panels on the same scale.

Every panel: the path in its own frame (the env anchors it to the robot's pose at reset, so in the sim it is
shifted and rotated), the start point (dot) and the direction of travel (arrow), with the length and the lap time
at time scale s = 1 in the title. Training picks a random file + random start point every episode.

Usage:
    python plot_trajectories.py                       # train, test, test_unseen -> outputs/trajectories/*.png
    python plot_trajectories.py --sets train --cols 20
"""

from __future__ import annotations

import argparse
import glob
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
REF_DIR = os.path.join(
    ROOT, "source", "mecanum_tracking_ws", "mecanum_tracking_ws", "tasks", "manager_based", "mecanum_tracking", "refs"
)
PATH_COLOR = "#2a78d6"
START_COLOR = "#0b0b0b"
MUTED = "#52514e"


def load(path: str) -> np.ndarray:
    return np.loadtxt(path, delimiter=",", skiprows=1)


def plot_set(name: str, out_dir: str, cols: int) -> str:
    files = sorted(glob.glob(os.path.join(REF_DIR, name, "*.csv")))
    if not files:
        raise FileNotFoundError(f"no reference files in {os.path.join(REF_DIR, name)}")
    tables = [load(f) for f in files]
    # common half-width around each curve's centre, so sizes are comparable between panels
    half = max(0.5 * np.ptp(t[:, 1:3], axis=0).max() for t in tables) * 1.08
    cols = min(cols, len(files))
    rows = int(np.ceil(len(files) / cols))
    size = 1.6 if len(files) > 20 else 2.6
    fig, axes = plt.subplots(rows, cols, figsize=(cols * size, rows * size + 0.6), squeeze=False)
    for ax in axes.flat:
        ax.set_axis_off()
    for ax, f, t in zip(axes.flat, files, tables):
        x, y = t[:, 1], t[:, 2]
        cx, cy = 0.5 * (x.min() + x.max()), 0.5 * (y.min() + y.max())
        ax.plot(x, y, color=PATH_COLOR, lw=1.2)
        ax.plot(x[0], y[0], "o", color=START_COLOR, ms=3.5)
        k = max(len(x) // 12, 1)  # arrow a bit after the start: direction of travel
        ax.annotate("", xy=(x[k], y[k]), xytext=(x[k // 2], y[k // 2]),
                    arrowprops=dict(arrowstyle="-|>", color=START_COLOR, lw=0.8, mutation_scale=8))
        ax.set_xlim(cx - half, cx + half)
        ax.set_ylim(cy - half, cy + half)
        ax.set_aspect("equal")
        length = np.sum(np.linalg.norm(np.diff(t[:, 1:3], axis=0), axis=1))
        ax.set_title(f"{os.path.splitext(os.path.basename(f))[0]}\n{length:.1f} m, {t[-1, 0]:.0f} s",
                     fontsize=6.5 if len(files) > 20 else 9, color=MUTED, pad=2)
    fig.suptitle(f"refs/{name}: {len(files)} trajectories (same scale, panel width {2 * half:.1f} m; "
                 f"dot = start, arrow = direction, time at s = 1)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f"{name}.png")
    fig.savefig(out, dpi=110, facecolor="white")
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sets", nargs="+", default=["train", "test", "test_unseen"])
    ap.add_argument("--out", default=os.path.join(ROOT, "outputs", "trajectories"))
    ap.add_argument("--cols", type=int, default=20)
    args = ap.parse_args()
    for name in args.sets:
        print(plot_set(name, args.out, args.cols))


if __name__ == "__main__":
    main()
