"""Table and figures of the MPC ablation on refs/test_corners_{slow,fast} (logs/runs/eval_mpc_plain_corners.sh):
the MPC with S = 100 I + soft wheel-speed limit (the default until 2026-10-27) vs. the plain one (both off,
the default since 2026-10-28), and each term off alone.

Per run: the metrics of tools/metrics.py, plus the largest position error within 1 s after a sharp corner. Per path:
the two runs compared step by step (same plant, same start, so any difference is the controller's),
over the whole window and away from the corners (each corner's -0.5 .. +1.5 s left out). Figures: position error,
largest |u_i| and largest |omega_i| over time, S + soft limit vs. plain, the corners marked.

    python scripts/tools/report_mpc_ablation.py --out logs/eval/mpc_plain_corners_table.md --fig_dir logs/eval/mpc_plain
"""

import argparse
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
import metrics  # noqa: E402

SETS = (("test_corners_slow", "0.1 m/s"), ("test_corners_fast", "0.9 · v_max (≤ 0.22 m/s)"))
PATHS = (("sharp_square", "sarkos négyzet"), ("straight_turn", "egyenes + 135°-os kanyar"),
         ("smooth", "sima, kis görbületű pálya"))
VARIANTS = (("mpc", "S + puha ω (régi alap)"), ("mpc_plain", "S és puha ω nélkül (új alap)"), ("mpc_noS", "csak S nélkül"),
            ("mpc_nosoft", "csak puha ω nélkül"))
# dataviz reference palette, light mode: categorical slots 1-2 and the chart chrome
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
C_CUR, C_PLAIN = "#2a78d6", "#eb6834"


def corner_times(log):
    """Times where the reference direction of travel jumps by > 30 deg within one step (the sharp corners); the central
    differences spread a corner over two rows, so neighbours within 0.2 s are merged."""
    phi = np.unwrap(np.arctan2(log["vy_ref"], log["vx_ref"]))
    moving = np.hypot(log["vx_ref"], log["vy_ref"]) > 1e-3
    idx = np.where((np.abs(np.diff(phi)) > np.radians(30)) & moving[1:] & moving[:-1])[0]
    out = []
    for i in idx:
        if not out or log["t"][i] - out[-1] > 0.2:
            out.append(log["t"][i])
    return np.array(out)


def pos_error(log):
    return np.hypot(log["x"] - log["x_ref"], log["y"] - log["y_ref"])


def run_rows(eval_dir, set_name):
    rows = ["| pálya | MPC | pozíció RMS / max [mm] | lag átlag [mm] | contour RMS / max [mm] | sarok utáni max [mm] | "
            "nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | ω max [rad/s] (> 8.06) | QP / lépés [ms] | elveszett |",
            "|---|---|---|---|---|---|---|---|---|---|---|"]
    for path, _ in PATHS:
        for var, label in VARIANTS:
            lg = metrics.load_log(os.path.join(eval_dir, var, set_name, f"{path}.csv"))
            m, w = metrics.run_metrics(lg), metrics.window(lg)
            near = np.zeros_like(w)
            for t in corner_times(lg):
                near |= (lg["t"] >= t) & (lg["t"] <= t + 1.0)
            corner_max = f"{1e3 * pos_error(lg)[w & near].max():.1f}" if near.any() else "–"
            rows.append(
                f"| {path} | {label} | {1e3 * m['pos_rmse_m']:.2f} / {1e3 * m['pos_max_m']:.1f} | "
                f"{1e3 * m['lag_mean_m']:+.2f} | {1e3 * m['contour_rmse_m']:.2f} / {1e3 * m['contour_max_m']:.1f} | "
                f"{corner_max} | {m['torque_rms_Nm']:.4f} / {m['torque_max_Nm']:.4f} | {m['action_rate_max']:.4f} | "
                f"{m['wheel_speed_max']:.2f} ({100 * m['omega_exceed_share']:.1f}%) | "
                f"{np.mean(lg['qp_ms'][w]):.2f} / {m['compute_ms_mean']:.1f} | {int(m['failed'])} |")
    return rows


def diff_rows(eval_dir):
    rows = ["| készlet | pálya | ‖p_régi − p_új‖ max [mm] | … a sarkoktól távol [mm] | "
            "max \\|u_régi − u_új\\| [Nm] | … a sarkoktól távol [Nm] | ‖Δu‖ max régi / új [Nm] |",
            "|---|---|---|---|---|---|---|"]
    for set_name, _ in SETS:
        for path, _ in PATHS:
            a = metrics.load_log(os.path.join(eval_dir, "mpc", set_name, f"{path}.csv"))
            b = metrics.load_log(os.path.join(eval_dir, "mpc_plain", set_name, f"{path}.csv"))
            n = min(len(a["t"]), len(b["t"]))
            a, b = {k: v[:n] for k, v in a.items()}, {k: v[:n] for k, v in b.items()}
            w = metrics.window(a)
            far = w.copy()
            for t in corner_times(a):
                far &= ~((a["t"] >= t - 0.5) & (a["t"] <= t + 1.5))
            dp = np.hypot(a["x"] - b["x"], a["y"] - b["y"])
            du = np.abs(metrics._cols(a, "u_") - metrics._cols(b, "u_")).max(axis=1)
            rows.append(f"| {set_name.replace('test_corners_', '')} | {path} | {1e3 * dp[w].max():.2f} | "
                        f"{1e3 * dp[far].max():.2f} | {du[w].max():.4f} | {du[far].max():.4f} | "
                        f"{metrics.run_metrics(a)['action_rate_max']:.4f} / "
                        f"{metrics.run_metrics(b)['action_rate_max']:.4f} |")
    return rows


def figure(eval_dir, set_name, set_label, out):
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.titlecolor": INK, "axes.facecolor": SURF, "figure.facecolor": SURF,
                         "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
                         "grid.linewidth": 0.6, "legend.frameon": False})
    fig, axes = plt.subplots(3, 3, figsize=(13, 8.2), sharex="row")
    for r, (path, path_label) in enumerate(PATHS):
        cur = metrics.load_log(os.path.join(eval_dir, "mpc", set_name, f"{path}.csv"))
        for var, color, label in (("mpc", C_CUR, "S = 100·I + puha ω-korlát (régi alap)"),
                                  ("mpc_plain", C_PLAIN, "S és puha ω-korlát nélkül (új alap)")):
            lg = metrics.load_log(os.path.join(eval_dir, var, set_name, f"{path}.csv"))
            w = metrics.window(lg)
            axes[r, 0].plot(lg["t"][w], 1e3 * pos_error(lg)[w], color=color, lw=1.4, label=label)
            axes[r, 1].plot(lg["t"][w], np.abs(metrics._cols(lg, "u_")[w]).max(axis=1), color=color, lw=1.4)
            axes[r, 2].plot(lg["t"][w], np.abs(metrics._cols(lg, "w_")[w]).max(axis=1), color=color, lw=1.4)
        axes[r, 2].axhline(metrics.OMEGA_MAX, color=INK2, lw=1, ls="--")
        axes[r, 2].text(axes[r, 2].get_xlim()[0], metrics.OMEGA_MAX, " 8.06 rad/s", color=INK2, va="bottom", fontsize=8)
        for ax in axes[r]:
            for t in corner_times(cur):
                ax.axvline(t, color=MUTED, lw=0.8, ls=":")
            ax.set_ylim(bottom=0)
        axes[r, 0].set_ylabel(f"{path_label}\n\npozícióhiba [mm]")
        axes[r, 1].set_ylabel("max |u_i| [Nm]")
        axes[r, 2].set_ylabel("max |ω_i| [rad/s]")
    for ax in axes[-1]:
        ax.set_xlabel("t [s]")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, bbox_to_anchor=(0.5, 0.955))
    fig.suptitle(f"LTV-MPC ablation, {set_label} (pontozott függőleges: sarkok)", color=INK, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval_dir", default="logs/eval")
    ap.add_argument("--out", default=None, help="markdown file of the tables (default: print only)")
    ap.add_argument("--fig_dir", default=None, help="folder of the figures (default: none)")
    args = ap.parse_args()

    lines = []
    for set_name, set_label in SETS:
        lines += [f"\n### {set_name} ({set_label})\n"] + run_rows(args.eval_dir, set_name)
    lines += ["\n### S + puha ω vs. S és puha ω nélkül, lépésenként\n"] + diff_rows(args.eval_dir)
    lines.append("\nA sarkoktól távol: a referencia mozgásának ablakából kivéve minden sarok −0.5 … +1.5 s környezetét "
                 "(a sima pályán nincs sarok, ott a teljes ablak).")
    text = "\n".join(lines) + "\n"
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)
    if args.fig_dir:
        os.makedirs(args.fig_dir, exist_ok=True)
        for set_name, set_label in SETS:
            out = os.path.join(args.fig_dir, f"{set_name}.png")
            figure(args.eval_dir, set_name, set_label, out)
            print(out)


if __name__ == "__main__":
    main()
