"""Scenario video: the RL policy (left) and the LTV-MPC (right) on the same path, side by side (raw Isaac frames of
play_shapes.py --raw_dir), with live plots of both runs below: position and heading error, the current values and the
running RMS / peak (tools/metrics.py window), or the settling time for an initial-error scenario. One clip per
scenario, each exactly --duration / (number of scenarios) long: a title card, then the run, fast-forwarded uniformly
(frames dropped, the overlay shows sim time and the factor) when it is longer than the clip, held on its last frame
when shorter. Pure numpy / matplotlib / imageio, no Isaac Lab.

Frame j of <raw>/<who>/<scenario>/<shape>.mp4 shows the state of row j of the .csv next to it (eval_tracking.py's log
format, the very run of eval_tracking.py). The settling times come from the full eval logs (<eval_root>/<rl_eval|
mpc_eval>/<set>/<shape>.csv, the same runs, not cut to the clip).

    python scripts/tools/compose_scenario_video.py --raw logs/videos/raw/sharp --eval_root logs/eval \
        --rl_eval rl_sharp_seed0 --mpc_eval mpc_plain --out logs/videos/sharp_rl_vs_mpc_30s.mp4 --duration 30
"""

import argparse
import os
import sys

import imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
import metrics  # noqa: E402
from compose_offset_video import (AXIS, C_MPC, C_RL, GRID, INK, INK2, LEGEND, MUTED, SURF, errors,  # noqa: E402
                                  hex_rgb, overlay, title_card)

SCENARIOS = {  # raw / eval subfolder, shape, title, description (logs/runs/video_sharp.sh, eval_sharp_policy.sh)
    "far": dict(set="test_shapes_y40_psi45", shape="circle", settle=True,
                title="Messziről, elfordítva indul",
                desc="40 cm oldalra és 45°-kal elfordítva a pálya elejétől · kör (R = 75 cm), 0.1 m/s, fix irány"),
    "rose": dict(set="test_dynamic_slow", shape="rose", settle=False,
                 title="Nagy görbület: háromszirmú rózsa",
                 desc="r = 28 cm · cos 3θ, hajtűkanyarok (görbület ≤ 36 1/m, sugár ≥ 2.8 cm) · 0.1 m/s, fix irány"),
    "square": dict(set="test_corners_slow", shape="sharp_square", settle=False,
                   title="Éles sarkok: négyzet",
                   desc="oldal 60 cm, a 90°-os sarkokon lassítás nélkül (a ref. sebessége ugrik) · 0.1 m/s, fix irány"),
}


class Panel:
    """The two error plots of one scenario; draw(j) shows rows 0..j of both runs."""

    def __init__(self, logs, settle, width, height, head):
        plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "axes.edgecolor": AXIS,
                             "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
                             "axes.facecolor": SURF, "figure.facecolor": SURF, "legend.frameon": False,
                             "legend.labelcolor": INK2})
        self.t = logs[0]["t"]
        self.err = [errors(log) for log in logs]  # [(e cm, de deg) RL, (e, de) MPC]
        self.win = [metrics.window(log) for log in logs]
        self.settle = settle
        self.fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
        self.fig.text(0.035, 0.95, head, color=INK, fontsize=22, fontweight="bold", va="top")
        self.t_txt = self.fig.text(0.965, 0.95, "", color=INK, fontsize=22, fontweight="bold", va="top", ha="right")
        self.note = self.fig.text(0.035, 0.865, "", color=MUTED, fontsize=12, va="top")
        self.ax, self.head, self.sub, self.lines = [], [], [], []
        self.specs = [("Pozícióhiba", "mm", 10.0, 1), ("Irányhiba", "°", 1.0, 2)]  # name, unit, from cm/deg, decimals
        for k, (name, unit, f, _) in enumerate(self.specs):
            x0 = 0.06 + k * 0.49
            ax = self.fig.add_axes([x0, 0.12, 0.42, 0.5])
            ax.set_xlim(0, self.t[-1])
            top = max(f * err[k].max() for err in self.err)
            ax.set_ylim(0, 1.12 * max(top, 1e-3))
            ax.grid(True, color=GRID, lw=0.8)
            ax.tick_params(length=0)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.set_xlabel("idő [s]")
            ax.set_ylabel(unit if unit != "°" else "fok")
            ln = [ax.plot([], [], color=C_RL, lw=2, label="RL")[0], ax.plot([], [], color=C_MPC, lw=2, label="MPC")[0]]
            ax.legend(handles=ln, ncol=2, **LEGEND)
            self.head.append(self.fig.text(x0, 0.79, "", color=INK, fontsize=16, fontweight="bold", va="top"))
            self.sub.append(self.fig.text(x0, 0.735, "", color=INK2, fontsize=13, va="top"))
            self.ax.append(ax)
            self.lines.append(ln)
        if settle is not None:  # settling moment: a dot on each run's position curve once reached (2px surface ring)
            self.dots = [self.ax[0].plot([], [], "o", ms=9, color=c, mec=SURF, mew=2, zorder=5)[0] for c in (C_RL, C_MPC)]
        self.cursor = [ax.axvline(0.0, color=MUTED, lw=1) for ax in self.ax]

    def draw(self, j, speed):
        t = self.t[: j + 1]
        self.t_txt.set_text(f"t = {self.t[j]:5.2f} s" + ("" if speed <= 1.0 else f"   ({speed:.1f}× gyorsítva)"))
        for k, (name, unit, f, dec) in enumerate(self.specs):
            for i in range(2):
                self.lines[k][i].set_data(t, f * self.err[i][k][: j + 1])
            sep = "" if unit == "°" else " "
            self.head[k].set_text(f"{name}:  RL {f * self.err[0][k][j]:.{dec}f}{sep}{unit}  ·  "
                                  f"MPC {f * self.err[1][k][j]:.{dec}f}{sep}{unit}")
            stats = []
            for i, who in enumerate(("RL", "MPC")):
                e = f * self.err[i][k][: j + 1][self.win[i][: j + 1]]
                stats.append(f"{who} {np.sqrt(np.mean(e ** 2)):.{dec}f} / {e.max():.{dec}f}" if len(e) else f"{who} —")
            self.sub[k].set_text(f"RMS / csúcs eddig ({unit}):  " + "  ·  ".join(stats))
            self.cursor[k].set_xdata([self.t[j]])
        if self.settle is not None:
            txt = "  ·  ".join(f"{who} {self.settle[i]:.2f} s" if self.t[j] >= self.settle[i] else f"{who} —"
                               for i, who in enumerate(("RL", "MPC")))
            self.note.set_text(f"beállás (a hiba < 1 cm és < 2° marad legalább 2 s-ig, a teljes futásból):  {txt}")
            for i in range(2):
                js = int(round(self.settle[i] / (self.t[1] - self.t[0]))) if np.isfinite(self.settle[i]) else None
                if js is not None and js <= j:
                    self.dots[i].set_data([self.t[js]], [10.0 * self.err[i][0][js]])
        else:
            self.note.set_text("RMS / csúcs: a referencia mozgásának végéig (metrics.py ablak)")
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3].copy()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", required=True, help="folder with rl/<scenario>/ and mpc/<scenario>/")
    ap.add_argument("--eval_root", default="logs/eval")
    ap.add_argument("--rl_eval", required=True, help="eval folder of the RL runs under --eval_root")
    ap.add_argument("--mpc_eval", required=True, help="eval folder of the MPC runs under --eval_root")
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS), choices=list(SCENARIOS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=30.0, help="total length [s], split evenly between the scenarios")
    ap.add_argument("--rl_who", default="éles pályákon is tanítva, model_276")
    ap.add_argument("--mpc_who", default="DARE-végsúly, S és puha ω-korlát nélkül")
    ap.add_argument("--title_s", type=float, default=0.8)
    ap.add_argument("--panel_height", type=int, default=480)
    args = ap.parse_args()

    writer, n_out = None, 0
    n_sc = len(args.scenarios)
    whos = [("RL", args.rl_who), ("LTV-MPC", args.mpc_who)]
    for k, sc in enumerate(args.scenarios, 1):
        spec = SCENARIOS[sc]
        logs, frames = [], []
        for who in ("rl", "mpc"):
            logs.append(metrics.load_log(os.path.join(args.raw, who, sc, f"{spec['shape']}.csv")))
            frames.append([f for f in imageio.get_reader(os.path.join(args.raw, who, sc, f"{spec['shape']}.mp4"))])
        n = min(len(frames[0]), len(frames[1]), len(logs[0]["t"]), len(logs[1]["t"]))
        logs = [{key: v[:n] for key, v in log.items()} for log in logs]
        dt = logs[0]["t"][1] - logs[0]["t"][0]
        fps = 1.0 / dt
        if writer is None:
            os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
            writer = imageio.get_writer(args.out, fps=fps, codec="libx264", quality=8, macro_block_size=8)
        settle = None
        if spec["settle"]:
            full = [metrics.load_log(os.path.join(args.eval_root, d, spec["set"], f"{spec['shape']}.csv"))
                    for d in (args.rl_eval, args.mpc_eval)]
            settle = [metrics.settling_time(log)[0] for log in full]
        n_clip = round(args.duration / n_sc * fps)
        n_title = round(args.title_s * fps)
        n_run = n_clip - n_title
        speed = max(1.0, n / n_run)
        rows = np.round(np.linspace(0, n - 1, n_run)).astype(int) if speed > 1.0 else np.arange(n)
        width = frames[0][0].shape[1] * 2
        head = f"{k}/{n_sc} · {spec['title']}"
        panel = Panel(logs, settle, width, args.panel_height, head)
        errs = [errors(log) for log in logs]
        for m, j in enumerate(rows):
            top = np.concatenate([overlay(frames[i][j], whos[i], hex_rgb((C_RL, C_MPC)[i]),
                                          (errs[i][0][j], errs[i][1][j]), legend=(i == 0), mm=True)
                                  for i in range(2)], axis=1)
            frame = np.concatenate([top, panel.draw(j, speed)], axis=0)
            if m == 0:
                card = title_card(frame, [(f"{k}/{n_sc}  {spec['title']}", 0, (255, 255, 255)),
                                          (spec["desc"], 1, (225, 225, 220)),
                                          (f"bal: RL ({args.rl_who}) · jobb: LTV-MPC ({args.mpc_who})", 1,
                                           (225, 225, 220)),
                                          ("valós idő" if speed <= 1.0 else f"{speed:.1f}× gyorsítva", 1,
                                           (225, 225, 220))])
                for _ in range(n_title):
                    writer.append_data(card)
            writer.append_data(frame)
        for _ in range(n_run - len(rows)):  # a run shorter than the clip: hold its last frame
            writer.append_data(frame)
        n_out += n_clip
        plt.close(panel.fig)
        w = [metrics.window(log) for log in logs]
        e = [10.0 * err[0] for err in errs]
        print(f"[compose] {sc}: {n} rows, x{speed:.2f}; pos RMS RL {np.sqrt(np.mean(e[0][w[0]] ** 2)):.2f} mm, "
              f"MPC {np.sqrt(np.mean(e[1][w[1]] ** 2)):.2f} mm" + (f"; settling RL {settle[0]:.2f} s, MPC {settle[1]:.2f} s"
                                                                 if settle else ""), flush=True)
    writer.close()
    print(f"[compose] {args.out}: {n_out} frames, {n_out / fps:.2f} s at {fps:.0f} fps")


if __name__ == "__main__":
    main()
