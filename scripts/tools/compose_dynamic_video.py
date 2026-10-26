"""Analysis video of a controller on refs/test_dynamic: the raw Isaac frames of play_shapes.py --raw_dir (left) next to
live plots of the very same run's log (right): position error (with its cross-track / lag split), speed against the
direction-dependent speed limit, acceleration, and the wheel torques the motors delivered, with the current values and
the running RMS / maxima. Pure numpy / matplotlib / imageio, no Isaac Lab.

Frame j of <raw_dir>/<shape>.mp4 shows the state of row j of <raw_dir>/<shape>.csv (eval_tracking.py's log format).
The statistics use the window of tools/metrics.py (to the end of the reference motion), so the final values equal
`metrics.py --table` of the same logs. Real time: one frame per control step (20 fps).

    python scripts/tools/compose_dynamic_video.py --raw_dir logs/videos/raw/mpc_test_dynamic \
        --out logs/videos/mpc_test_dynamic_60s.mp4 --duration 60
"""

import argparse
import os
import sys

import imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

sys.path.insert(0, os.path.dirname(__file__))
_PKG = os.path.join(os.path.dirname(__file__), "..", "..", "source", "mecanum_tracking_ws", "mecanum_tracking_ws")
sys.path.insert(0, _PKG)
import metrics  # noqa: E402
from mpc import motor, params  # noqa: E402

URDF = os.path.join(os.path.dirname(__file__), "..", "..", "assets", "robots", "own_mecanum", "robot_measured.urdf")
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

SHAPES = {  # title, description lines (refs/test_dynamic, tools/gen_dynamic_references.py)
    "circle_small": ("Kis kör", ["R = 15 cm, két kör, görbület 6.7 1/m",
                                 "v_ref = 0.9 · v_max(irány): 0.155–0.22 m/s"]),
    "slalom": ("Szlalom", ["A = 10 cm, λ = 40 cm, három periódus, görbület ≤ 25 1/m",
                           "v_ref = 0.9 · v_max(irány), váltakozó oldalgyorsulás"]),
    "rose": ("Háromszirmú rózsa", ["r = 28 cm · cos 3θ, hajtűkanyarok, görbület ≤ 36 1/m",
                                   "v_ref = 0.9 · v_max(irány)"]),
    "sharp_square": ("Sarkos négyzet", ["oldal 60 cm, a 90°-os sarkokon is 0.22 m/s (= 0.9 · v_max)",
                                        "a sarokban a ref. sebességvektora ugrik: követhetetlen"]),
    "stop_go": ("Egyenes: indulás és megállás", ["3 × 70 cm oda-vissza, 0.22 m/s (= 0.9 · v_max)",
                                                 "indulás / megállás: 1 m/s², 3 m/s², majd ugrásszerűen"]),
}
WHEELS = ["BE", "JE", "BH", "JH"]  # MPC order FL, FR, RL, RR: bal/jobb első/hátsó

# dark chart tokens and categorical slots (dataviz reference palette, dark steps): robot = slot 1, reference = slot 2,
# the four wheels = slots 3-6 (adjacent slots, validated as an ordered set)
SURF, INK, INK2, MUTED, GRID, AXIS = "#1a1a19", "#f0efec", "#c3c2b7", "#898781", "#2c2c2a", "#383835"
C_ROBOT, C_REF = "#3987e5", "#d95926"
C_WHEELS = ["#199e70", "#c98500", "#d55181", "#008300"]
LEGEND = dict(loc="lower right", bbox_to_anchor=(1.0, 0.98), fontsize=11, borderaxespad=0.0, handlelength=1.6,
              columnspacing=1.2)  # above the plot, under its subtitle: never over the data


def series(log, p):
    """Everything the panel shows, per row."""
    dt = log["t"][1] - log["t"][0]
    e = np.hypot(log["x"] - log["x_ref"], log["y"] - log["y_ref"])
    contour, lag = metrics.contour_lag(log)
    v_w = np.c_[log["vx"], log["vy"]]
    vr_w = np.c_[log["vx_ref"], log["vy_ref"]]
    v, v_ref = np.linalg.norm(v_w, axis=1), np.linalg.norm(vr_w, axis=1)
    # speed limit of the reference's motion pattern: the path speed at which its fastest wheel reaches omega_max
    c, s = np.cos(log["yaw_ref"]), np.sin(log["yaw_ref"])
    nu = np.c_[c * vr_w[:, 0] + s * vr_w[:, 1], -s * vr_w[:, 0] + c * vr_w[:, 1], log["wz_ref"]]
    wheel_ref = np.abs(nu @ p.W.T).max(axis=1)
    moving = v_ref > 1e-3
    v_max = np.full_like(v, np.nan)
    v_max[moving] = motor.OMEGA_MAX * v_ref[moving] / wheel_ref[moving]
    a_w = np.gradient(v_w, dt, axis=0)
    ar_w = np.gradient(vr_w, dt, axis=0)
    u = v_w / np.maximum(v, 1e-6)[:, None]
    a_t = (a_w * u).sum(1)
    a_n = np.abs(a_w[:, 0] * u[:, 1] - a_w[:, 1] * u[:, 0])
    tau = np.stack([log[f"tau_{j}"] for j in range(4)], 1)
    w = np.stack([log[f"w_{j}"] for j in range(4)], 1)
    return dict(t=log["t"], e=e, contour=contour, lag=lag, v=v, v_ref=v_ref, v_max=v_max,
                a=np.linalg.norm(a_w, axis=1), a_ref=np.linalg.norm(ar_w, axis=1), a_t=a_t, a_n=a_n, tau=tau,
                w_max=np.abs(w).max(1), win=metrics.window(log))


class Panel:
    """The plots of one shape; draw(j) shows rows 0..j."""

    def __init__(self, d, width, height, t_total):
        self.d = d
        plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 13, "axes.edgecolor": AXIS,
                             "axes.labelcolor": MUTED, "xtick.color": MUTED, "ytick.color": MUTED,
                             "axes.facecolor": SURF, "figure.facecolor": SURF, "legend.frameon": False,
                             "legend.labelcolor": INK2})
        self.fig = plt.figure(figsize=(width / 100, height / 100), dpi=100)
        self.t_txt = self.fig.text(0.06, 0.965, "", color=INK, fontsize=24, fontweight="bold", va="top")
        self.fig.text(0.06, 0.918, "mért értékek: aktuális · RMS / csúcs a mozgás végéig (metrics.py ablak)",
                      color=MUTED, fontsize=12, va="top")
        tops = [0.865, 0.655, 0.445, 0.235]
        self.ax, self.head, self.sub = [], [], []
        for top in tops:
            ax = self.fig.add_axes([0.11, top - 0.165, 0.85, 0.1])
            ax.set_xlim(0, t_total)
            ax.grid(True, color=GRID, lw=0.8)
            ax.tick_params(length=0)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            self.head.append(self.fig.text(0.06, top, "", color=INK, fontsize=16, fontweight="bold", va="top"))
            self.sub.append(self.fig.text(0.06, top - 0.027, "", color=INK2, fontsize=12, va="top"))
            self.ax.append(ax)
        for ax in self.ax[:-1]:
            ax.set_xticklabels([])
        self.ax[-1].set_xlabel("idő [s]")
        t = d["t"]
        lw = 1.6
        self.l_e = self.ax[0].plot([], [], color=C_ROBOT, lw=lw)[0]
        self.ax[0].set_ylabel("mm")
        self.ax[0].set_ylim(0, 1.15 * 1000 * d["e"].max())
        self.l_vmax = self.ax[1].plot([], [], color=MUTED, lw=1.2, ls=(0, (4, 3)), label="v_max (irány)")[0]
        self.l_vref = self.ax[1].plot([], [], color=C_REF, lw=lw, label="referencia")[0]
        self.l_v = self.ax[1].plot([], [], color=C_ROBOT, lw=lw, label="robot")[0]
        self.ax[1].set_ylabel("m/s")
        self.ax[1].set_ylim(0, 0.27)
        self.ax[1].axhline(motor.OMEGA_MAX * PARAMS.R, color=AXIS, lw=1)
        self.ax[1].legend(handles=[self.l_v, self.l_vref, self.l_vmax], ncol=3, **LEGEND)
        self.l_aref = self.ax[2].plot([], [], color=C_REF, lw=lw, label="referencia")[0]
        self.l_a = self.ax[2].plot([], [], color=C_ROBOT, lw=lw, label="robot")[0]
        self.ax[2].set_ylabel("m/s²")
        self.ax[2].set_ylim(0, 1.1 * max(d["a"].max(), d["a_ref"].max()))
        self.ax[2].legend(handles=[self.l_a, self.l_aref], ncol=2, **LEGEND)
        self.l_tau = [self.ax[3].plot([], [], color=c, lw=1.4, label=n)[0] for c, n in zip(C_WHEELS, WHEELS)]
        self.ax[3].set_ylabel("Nm")
        m = 1.1 * np.abs(d["tau"]).max()
        self.ax[3].set_ylim(-m, m)
        self.ax[3].axhline(0.0, color=AXIS, lw=1)
        self.ax[3].legend(ncol=4, **LEGEND)
        self.cursor = [ax.axvline(0.0, color=MUTED, lw=1) for ax in self.ax]

    def draw(self, j):
        d = self.d
        t = d["t"][: j + 1]
        win = d["win"][: j + 1]
        self.t_txt.set_text(f"t = {d['t'][j]:5.2f} s")
        e_mm = 1000 * d["e"]
        self.l_e.set_data(t, e_mm[: j + 1])
        if win.any():
            rms = np.sqrt(np.mean(e_mm[: j + 1][win] ** 2))
            c_rms = 1000 * np.sqrt(np.mean(d["contour"][: j + 1][win] ** 2))
            lag = 1000 * np.mean(d["lag"][: j + 1][win])
            self.sub[0].set_text(f"RMS {rms:.1f} mm · csúcs {e_mm[: j + 1][win].max():.1f} mm · "
                                 f"keresztirányú RMS {c_rms:.1f} mm · lemaradás átlag {lag:+.1f} mm")
        self.head[0].set_text(f"Pozícióhiba: {e_mm[j]:.1f} mm")
        self.l_v.set_data(t, d["v"][: j + 1])
        self.l_vref.set_data(t, d["v_ref"][: j + 1])
        self.l_vmax.set_data(t, d["v_max"][: j + 1])
        vmax = d["v_max"][j]
        vmax_txt = "—" if np.isnan(vmax) else f"{vmax:.3f} m/s (v = {100 * d['v'][j] / vmax:.0f}%)"
        self.head[1].set_text(f"Sebesség: {d['v'][j]:.3f} m/s   v_max: {vmax_txt}")
        self.sub[1].set_text(f"csúcs {d['v'][: j + 1].max():.3f} m/s (előre/oldalra v_max = "
                             f"{motor.OMEGA_MAX * PARAMS.R:.3f}) · kerék {d['w_max'][j]:.1f} rad/s = "
                             f"{100 * d['w_max'][j] / motor.OMEGA_MAX:.0f}% · csúcs "
                             f"{100 * d['w_max'][: j + 1].max() / motor.OMEGA_MAX:.0f}%")
        self.l_a.set_data(t, d["a"][: j + 1])
        self.l_aref.set_data(t, d["a_ref"][: j + 1])
        self.head[2].set_text(f"Gyorsulás: {d['a'][j]:.2f} m/s²")
        self.sub[2].set_text(f"érintő {d['a_t'][j]:+.2f} · normál {d['a_n'][j]:.2f} m/s² · csúcs "
                             f"{d['a'][: j + 1].max():.2f} (referencia {d['a_ref'][: j + 1].max():.2f}) m/s²")
        for k, line in enumerate(self.l_tau):
            line.set_data(t, d["tau"][: j + 1, k])
        self.head[3].set_text("Nyomaték: " + "  ".join(f"{n} {1000 * d['tau'][j, k]:+5.1f}"
                                                         for k, n in enumerate(WHEELS)) + " mNm")
        self.sub[3].set_text(f"leadott · csúcs |τ| {np.abs(d['tau'][: j + 1]).max():.3f} Nm (korlát {motor.U_MAX}) · "
                             f"fix iránynál BE ≈ JH, JE ≈ BH")
        for c in self.cursor:
            c.set_xdata([d["t"][j]])
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[..., :3].copy()


def boxed_text(img, xy, lines, fonts, pad, fill=(0, 0, 0, 150)):
    """Lines of (text, font index, color) in a rounded dark box at xy (top-left)."""
    draw = ImageDraw.Draw(img, "RGBA")
    w = max(draw.textlength(s, font=fonts[f]) for s, f, _ in lines) + 2 * pad
    h = sum(fonts[f].size + pad // 2 for _, f, _ in lines) + 2 * pad - pad // 2
    draw.rounded_rectangle((xy[0], xy[1], xy[0] + w, xy[1] + h), radius=pad, fill=fill)
    y = xy[1] + pad
    for s, f, c in lines:
        draw.text((xy[0] + pad, y), s, font=fonts[f], fill=c)
        y += fonts[f].size + pad // 2
    return w, h


def render_overlay(frame, k, n_shapes, name, who, title_card):
    img = Image.fromarray(frame)
    h = img.height
    fonts = [ImageFont.truetype(FONT_BOLD, h // 24), ImageFont.truetype(FONT, h // 46),
             ImageFont.truetype(FONT_BOLD, h // 14)]
    pad = h // 70
    title, desc = SHAPES.get(name, (name, []))
    white, grey = (255, 255, 255), (225, 225, 220)
    if title_card:
        lines = [(f"{k}/{n_shapes}  {title}", 2, white)] + [(s, 1, grey) for s in desc] + [(who, 1, grey)]
        draw = ImageDraw.Draw(img)
        w = max(draw.textlength(s, font=fonts[f]) for s, f, _ in lines) + 2 * pad
        boxed_text(img, ((img.width - w) // 2, h // 3), lines, fonts, pad, fill=(0, 0, 0, 175))
    else:
        boxed_text(img, (pad, pad), [(f"{k}/{n_shapes}  {title}", 0, white)] + [(s, 1, grey) for s in desc]
                   + [(who, 1, grey)], fonts, pad)
    leg = [((30, 215, 65), "referencia pálya"), ((255, 90, 0), "megtett út"), ((230, 25, 25), "aktuális ref. pont"),
           ((30, 100, 230), "előretekintés (10 × 0.05 s)")]
    draw = ImageDraw.Draw(img, "RGBA")
    f = fonts[1]
    lh = f.size + pad // 2
    y0 = h - pad - len(leg) * lh - pad
    wl = 3 * pad + f.size + max(draw.textlength(t, font=f) for _, t in leg)
    draw.rounded_rectangle((pad, y0 - pad // 2, pad + wl, h - pad), radius=pad, fill=(0, 0, 0, 140))
    y = y0
    for color, text in leg:
        draw.rectangle((2 * pad, y + f.size // 4, 2 * pad + f.size, y + 3 * f.size // 4), fill=color)
        draw.text((3 * pad + f.size, y), text, font=f, fill=grey)
        y += lh
    return np.asarray(img)


PARAMS = params.from_urdf(URDF)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw_dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--shapes", nargs="+", default=list(SHAPES))
    ap.add_argument("--who", default="LTV-MPC (DARE-végsúly, S = 100·I, Q_path nélkül) · fix irány · valós idő")
    ap.add_argument("--title_s", type=float, default=0.5, help="still title card before each shape [s]")
    ap.add_argument("--duration", type=float, default=None, help="pad the end (last frame) to exactly this long [s]")
    ap.add_argument("--panel_width", type=int, default=840)
    ap.add_argument("--max_frames", type=int, default=None, help="debug: frames per shape")
    args = ap.parse_args()

    writer, fps, n_out = None, None, 0
    for k, name in enumerate(args.shapes, 1):
        log = metrics.load_log(os.path.join(args.raw_dir, f"{name}.csv"))
        reader = imageio.get_reader(os.path.join(args.raw_dir, f"{name}.mp4"))
        frames = [f for f in reader]
        n = min(len(frames), len(log["t"]))
        if args.max_frames:
            n = min(n, args.max_frames)
        dt = log["t"][1] - log["t"][0]
        if writer is None:
            fps = 1.0 / dt
            os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
            writer = imageio.get_writer(args.out, fps=fps, codec="libx264", quality=8, macro_block_size=8)
        d = series(log, PARAMS)
        h = frames[0].shape[0]
        panel = Panel(d, args.panel_width, h, (len(log["t"]) - 1) * dt)
        for j in range(n):
            left = render_overlay(frames[j], k, len(args.shapes), name, args.who, title_card=False)
            if j == 0:
                card = np.concatenate([render_overlay(frames[0], k, len(args.shapes), name, args.who, True),
                                       panel.draw(0)], axis=1)
                for _ in range(round(args.title_s * fps)):
                    writer.append_data(card)
                    n_out += 1
            last = np.concatenate([left, panel.draw(j)], axis=1)
            writer.append_data(last)
            n_out += 1
        plt.close(panel.fig)
        w = d["win"]
        print(f"[compose] {name}: {n} frames; pos RMS {1000 * np.sqrt(np.mean(d['e'][w] ** 2)):.2f} mm, "
              f"max {1000 * d['e'][w].max():.2f} mm; v max {d['v'].max():.3f} m/s; |a| max {d['a'].max():.2f} m/s^2 "
              f"(ref {d['a_ref'].max():.2f}); |tau| max {np.abs(d['tau']).max():.4f} Nm", flush=True)
    if args.duration:
        target = round(args.duration * fps)
        if n_out > target:
            print(f"[compose] WARNING: {n_out / fps:.2f} s > --duration {args.duration} s")
        while n_out < target:
            writer.append_data(last)
            n_out += 1
    writer.close()
    print(f"[compose] {args.out}: {n_out} frames, {n_out / fps:.2f} s at {fps:.0f} fps")


if __name__ == "__main__":
    main()
