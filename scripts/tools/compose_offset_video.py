"""Initial-error video: the RL policy (left) and the LTV-MPC (right) recovering from the same initial error, side by side
(raw Isaac frames of play_shapes.py --raw_dir --start_offset), with live plots of both runs below: position and heading
error, the settling threshold, and the settling time (tools/metrics.py settling_time) once reached. One clip per
scenario, then a summary card. Pure numpy / matplotlib / imageio, no Isaac Lab.

Frame j of <raw>/<scenario>/<shape>.mp4 shows the state of row j of <raw>/<scenario>/<shape>.csv (eval_tracking.py's
log format, the very run of eval_tracking.py). The settling times come from the full eval logs (--rl_eval / --mpc_eval,
the same runs, not cut to the clip), so they equal the table of logs/eval/offset_grid_table.md.

    python scripts/tools/compose_offset_video.py --rl_raw logs/videos/raw/offsets/rl --mpc_raw logs/videos/raw/offsets/mpc \
        --rl_eval logs/eval/rl_offset_seed0/offsets --mpc_eval logs/eval/mpc/offsets \
        --scenarios y10_psi0 y20_psi0 y30_psi0 --out logs/videos/offsets_rl_vs_mpc.mp4
"""

import argparse
import os
import re
import sys

import imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
import metrics  # noqa: E402

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
SHAPE_LABELS = {"circle": "kör", "figure8": "nyolcas", "rounded_square": "lekerekített négyzet",
                "sharp_square": "éles négyzet"}

# dark chart tokens and categorical slots 1-2 of the dataviz reference palette (dark steps, validated as a pair),
# the same tokens as compose_dynamic_video.py: RL = slot 1 (blue), MPC = slot 2 (orange)
SURF, INK, INK2, MUTED, GRID, AXIS = "#1a1a19", "#f0efec", "#c3c2b7", "#898781", "#2c2c2a", "#383835"
C_RL, C_MPC = "#3987e5", "#d95926"
LEGEND = dict(loc="lower right", bbox_to_anchor=(1.0, 1.0), fontsize=12, borderaxespad=0.3, handlelength=1.6,
              columnspacing=1.2)


def scenario_label(name: str) -> str:
    """y30_psi50 -> '30 cm oldalra + 50°'."""
    cm, deg = map(int, re.fullmatch(r"y(\d+)_psi(\d+)", name).groups())
    parts = ([f"{cm} cm oldalra"] if cm else []) + ([f"{deg}° elfordítva"] if deg else [])
    return " + ".join(parts) or "kezdeti hiba nélkül"


def errors(log):
    """Position error [cm] and |heading error| [deg] per row."""
    e = 100 * np.hypot(log["x"] - log["x_ref"], log["y"] - log["y_ref"])
    de = np.degrees(np.abs(metrics.wrap(log["yaw"] - log["yaw_ref"])))
    return e, de


def hex_rgb(c):
    return tuple(int(c[i : i + 2], 16) for i in (1, 3, 5))


class Panel:
    """The two error plots of one scenario; draw(j) shows rows 0..j of both runs."""

    def __init__(self, logs, settle, width, height, t_total, head):
        plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "axes.edgecolor": AXIS,
                             "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
                             "axes.facecolor": SURF, "figure.facecolor": SURF, "legend.frameon": False,
                             "legend.labelcolor": INK2})
        self.t = logs[0]["t"]
        self.err = [errors(log) for log in logs]  # [(e, de) RL, (e, de) MPC]
        self.settle = settle
        self.fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
        self.title = self.fig.text(0.035, 0.95, head, color=INK, fontsize=22, fontweight="bold", va="top")
        self.t_txt = self.fig.text(0.965, 0.95, "", color=INK, fontsize=22, fontweight="bold", va="top", ha="right")
        self.fig.text(0.035, 0.865, "beállás: a hiba < 1 cm és < 2° marad legalább 2 s-ig (a teljes kiértékelési "
                      "futásból)", color=MUTED, fontsize=12, va="top")
        self.ax, self.head, self.sub, self.lines = [], [], [], []
        specs = [("Pozícióhiba", " cm", "cm", 1.0, "1 cm"), ("Irányhiba", "°", "fok", 2.0, "2°")]
        for k, (name, unit, axis_unit, tol, tol_lbl) in enumerate(specs):
            x0 = 0.06 + k * 0.49
            ax = self.fig.add_axes([x0, 0.12, 0.42, 0.5])
            ax.set_xlim(0, t_total)
            top = max(err[k].max() for err in self.err)
            ax.set_ylim(0, 1.12 * max(top, 3 * tol))
            ax.grid(True, color=GRID, lw=0.8)
            ax.tick_params(length=0)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.set_xlabel("idő [s]")
            ax.set_ylabel(axis_unit)
            ax.axhline(tol, color=MUTED, lw=1.2, ls=(0, (4, 3)))
            ax.text(t_total, tol, f" {tol_lbl}", color=MUTED, fontsize=11, va="bottom", ha="right")
            ln = [ax.plot([], [], color=C_RL, lw=2, label="RL")[0], ax.plot([], [], color=C_MPC, lw=2, label="MPC")[0]]
            ax.legend(handles=ln, ncol=2, **LEGEND)
            # settling moment: a dot on each run's curve once reached (2px surface ring, series color = identity)
            self.dots = getattr(self, "dots", []) + [[ax.plot([], [], "o", ms=9, color=c, mec=SURF, mew=2, zorder=5)[0]
                                                    for c in (C_RL, C_MPC)]]
            self.head.append(self.fig.text(x0, 0.79, "", color=INK, fontsize=16, fontweight="bold", va="top"))
            self.sub.append(self.fig.text(x0, 0.735, "", color=INK2, fontsize=13, va="top"))
            self.ax.append(ax)
            self.lines.append(ln)
        self.cursor = [ax.axvline(0.0, color=MUTED, lw=1) for ax in self.ax]
        self.specs = specs

    def draw(self, j):
        t = self.t[: j + 1]
        self.t_txt.set_text(f"t = {self.t[j]:4.2f} s")
        for k, (name, unit, _, _, _) in enumerate(self.specs):
            dec = 2 if k == 0 else 1
            for i in range(2):
                self.lines[k][i].set_data(t, self.err[i][k][: j + 1])
            self.head[k].set_text(f"{name}:  RL {self.err[0][k][j]:.{dec}f}{unit}  ·  MPC {self.err[1][k][j]:.{dec}f}{unit}")
            self.cursor[k].set_xdata([self.t[j]])
            for i in range(2):
                js = int(round(self.settle[i] / (self.t[1] - self.t[0]))) if np.isfinite(self.settle[i]) else None
                if js is not None and js <= j:
                    self.dots[k][i].set_data([self.t[js]], [self.err[i][k][js]])
        txt = "  ·  ".join(f"{who} {self.settle[i]:.2f} s" if self.t[j] >= self.settle[i] else f"{who} —"
                           for i, who in enumerate(("RL", "MPC")))
        self.sub[0].set_text(f"beállás:  {txt}")
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3].copy()


def boxed_text(img, xy, lines, fonts, pad, fill=(0, 0, 0, 150), swatch=None):
    """Lines of (text, font index, color) in a rounded dark box at xy (top-left); swatch: color of a square before the
    first line (the series identity; the text itself stays in the ink colors)."""
    draw = ImageDraw.Draw(img, "RGBA")
    sw = fonts[lines[0][1]].size if swatch else 0
    w = max(draw.textlength(s, font=fonts[f]) for s, f, _ in lines) + 2 * pad + (sw + pad // 2 if swatch else 0)
    h = sum(fonts[f].size + pad // 2 for _, f, _ in lines) + 2 * pad - pad // 2
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + w, xy[1] + h), radius=pad, fill=fill)
    x, y = xy[0] + pad, xy[1] + pad
    if swatch:
        draw.rounded_rectangle((x, y + sw // 8, x + sw, y + sw + sw // 8), radius=sw // 6, fill=swatch)
        x += sw + pad // 2
    for s, f, c in lines:
        draw.text((x, y), s, font=fonts[f], fill=c)
        y += fonts[f].size + pad // 2
    return w, h


def overlay(frame, who, color, err, legend, mm=False):
    """err = (position error [cm], heading error [deg]); mm: show the position error in mm."""
    img = Image.fromarray(frame)
    h = img.height
    fonts = [ImageFont.truetype(FONT_BOLD, h // 22), ImageFont.truetype(FONT, h // 36)]
    pad = h // 60
    white, grey = (255, 255, 255), (225, 225, 220)
    pos = f"{10 * err[0]:.1f} mm" if mm else f"{err[0]:.2f} cm"
    boxed_text(img, (pad, pad), [(who[0], 0, white), (who[1], 1, grey),
                                 (f"hiba: {pos} · {err[1]:.1f}°", 1, white)], fonts, pad, swatch=color)
    if legend:
        leg = [((30, 215, 65), "referencia pálya"), ((255, 90, 0), "megtett út"), ((230, 25, 25), "aktuális ref. pont"),
               ((30, 100, 230), "előretekintés (10 × 0.05 s)")]
        draw = ImageDraw.Draw(img, "RGBA")
        f = fonts[1]
        lh = f.size + pad // 2
        y0 = h - pad - len(leg) * lh - pad
        wl = 3 * pad + f.size + max(draw.textlength(t, font=f) for _, t in leg)
        draw.rounded_rectangle((pad, y0 - pad // 2, pad + wl, h - pad), radius=pad, fill=(0, 0, 0, 140))
        y = y0
        for c, text in leg:
            draw.rectangle((2 * pad, y + f.size // 4, 2 * pad + f.size, y + 3 * f.size // 4), fill=c)
            draw.text((3 * pad + f.size, y), text, font=f, fill=grey)
            y += lh
    return np.asarray(img)


def title_card(frame, lines):
    img = Image.fromarray(frame)
    h = img.height
    fonts = [ImageFont.truetype(FONT_BOLD, h // 20), ImageFont.truetype(FONT, h // 48)]
    pad = h // 50
    draw = ImageDraw.Draw(img)
    w = max(draw.textlength(s, font=fonts[f]) for s, f, _ in lines) + 2 * pad
    boxed_text(img, ((img.width - w) // 2, h // 5), lines, fonts, pad, fill=(0, 0, 0, 190))
    return np.asarray(img)


def summary_card(rows, width, height, shape):
    """Table of all scenarios: settling time and the error after it, RL vs MPC."""
    fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
    fig.patch.set_facecolor(SURF)
    fig.text(0.05, 0.94, "Összefoglaló: visszatérés a kezdeti hibából", color=INK, fontsize=30, fontweight="bold",
             va="top")
    fig.text(0.05, 0.875, f"{shape} pálya, 0.1 m/s, fix irány · beállás: < 1 cm és < 2°, 2 s-ig · utána: a pozícióhiba "
             "RMS-e a mozgás végéig (teljes kiértékelési futás)", color=MUTED, fontsize=14, va="top")
    cols = ["kezdeti hiba", "beállás RL [s]", "beállás MPC [s]", "utána RL [mm]", "utána MPC [mm]"]
    xs = [0.05, 0.42, 0.56, 0.72, 0.86]
    y = 0.79
    for x, c in zip(xs, cols):
        fig.text(x, y, c, color=INK2, fontsize=15, fontweight="bold", va="top")
    fig.add_artist(plt.Line2D([0.05, 0.96], [y - 0.045, y - 0.045], color=AXIS, lw=1.5))
    dy = min(0.06, 0.66 / max(len(rows), 1))
    for k, r in enumerate(rows):
        yy = y - 0.065 - k * dy
        vals = [r["label"], f"{r['ts'][0]:.2f}", f"{r['ts'][1]:.2f}", f"{r['rms'][0]:.2f}", f"{r['rms'][1]:.2f}"]
        for x, v in zip(xs, vals):
            fig.text(x, yy, v, color=INK, fontsize=15, va="top")
    fig.canvas.draw()
    out = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rl_raw", required=True)
    ap.add_argument("--mpc_raw", required=True)
    ap.add_argument("--rl_eval", required=True)
    ap.add_argument("--mpc_eval", required=True)
    ap.add_argument("--scenarios", nargs="+", required=True)
    ap.add_argument("--shape", default="circle")
    ap.add_argument("--out", required=True)
    ap.add_argument("--rl_who", default="Offset-tanítás, model_311")
    ap.add_argument("--mpc_who", default="DARE-végsúly, S = 100·I")
    ap.add_argument("--title_s", type=float, default=1.2)
    ap.add_argument("--summary_s", type=float, default=6.0)
    ap.add_argument("--panel_height", type=int, default=480)
    args = ap.parse_args()

    writer, fps, rows = None, None, []
    n_sc = len(args.scenarios)
    for k, sc in enumerate(args.scenarios, 1):
        logs, frames = [], []
        for raw in (args.rl_raw, args.mpc_raw):
            logs.append(metrics.load_log(os.path.join(raw, sc, f"{args.shape}.csv")))
            frames.append([f for f in imageio.get_reader(os.path.join(raw, sc, f"{args.shape}.mp4"))])
        n = min(len(frames[0]), len(frames[1]), len(logs[0]["t"]), len(logs[1]["t"]))
        full = [metrics.load_log(os.path.join(d, sc, f"{args.shape}.csv")) for d in (args.rl_eval, args.mpc_eval)]
        st = [metrics.settling_time(log) for log in full]
        rows.append(dict(label=scenario_label(sc), ts=[s[0] for s in st], rms=[1000 * s[1] for s in st]))
        dt = logs[0]["t"][1] - logs[0]["t"][0]
        if writer is None:
            fps = 1.0 / dt
            os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
            writer = imageio.get_writer(args.out, fps=fps, codec="libx264", quality=8, macro_block_size=8)
        width = frames[0][0].shape[1] * 2
        head = f"{k}/{n_sc} · Kezdeti hiba: {scenario_label(sc)}"
        panel = Panel(logs, [s[0] for s in st], width, args.panel_height, (n - 1) * dt, head)
        errs = [errors(log) for log in logs]
        whos = [("RL", args.rl_who), ("LTV-MPC", args.mpc_who)]
        for j in range(n):
            top = np.concatenate([overlay(frames[i][j], whos[i], hex_rgb((C_RL, C_MPC)[i]),
                                          (errs[i][0][j], errs[i][1][j]), legend=(i == 0)) for i in range(2)], axis=1)
            frame = np.concatenate([top, panel.draw(j)], axis=0)
            if j == 0:
                card = title_card(frame, [(f"{k}/{n_sc}  Kezdeti hiba: {scenario_label(sc)}", 0, (255, 255, 255)),
                                          (f"bal: RL ({args.rl_who}) · jobb: LTV-MPC ({args.mpc_who})", 1,
                                           (225, 225, 220)),
                                          (f"{SHAPE_LABELS.get(args.shape, args.shape)} pálya, 0.1 m/s, fix irány, "
                                           "valós idő", 1, (225, 225, 220))])
                for _ in range(round(args.title_s * fps)):
                    writer.append_data(card)
            writer.append_data(frame)
        plt.close(panel.fig)
        print(f"[compose] {sc}: {n} frames; settling RL {st[0][0]:.2f} s, MPC {st[1][0]:.2f} s", flush=True)
    card = summary_card(rows, frame.shape[1], frame.shape[0], SHAPE_LABELS.get(args.shape, args.shape))
    for _ in range(round(args.summary_s * fps)):
        writer.append_data(card)
    writer.close()
    print(f"[compose] {args.out}")


if __name__ == "__main__":
    main()
