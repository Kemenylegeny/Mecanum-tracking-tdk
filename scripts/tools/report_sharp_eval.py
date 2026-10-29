"""Tables of logs/runs/eval_sharp_policy.sh: the RL trained on sharp corners and tight turns too (rl_sharp_seed0), the
same config on the smooth references only (rl_plain_seed0) and the plain LTV-MPC (mpc_plain), at 0.1 m/s.

Per path and controller: the metrics of tools/metrics.py (window: to the end of the reference motion), the largest
position error within 1 s after a sharp corner (report_mpc_ablation.corner_times), and for the initial-error runs the
settling time (< 1 cm and < 2 deg for 2 s) and the position error after it.

    python scripts/tools/report_sharp_eval.py --out logs/eval/sharp_policy_table.md
    # the MPC with wheel-speed states (logs/runs/eval_mpc_wheel_state.sh)
    python scripts/tools/report_sharp_eval.py --controllers "mpc_plain=LTV-MPC, világ-sebesség" \
        "mpc_wheel=LTV-MPC, kerék (enkóder)" "mpc_wheel_kin=LTV-MPC, kerék (alapsebességből)" \
        --sets test_shapes test_dynamic_slow test_corners_slow test_dynamic --out logs/eval/mpc_wheel_state_table.md
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
import metrics  # noqa: E402
from report_mpc_ablation import corner_times, pos_error  # noqa: E402

CONTROLLERS = (("rl_sharp_seed0", "RL, éles pályákon is (model_276)"), ("rl_plain_seed0", "RL, csak sima pályákon (model_282)"),
               ("mpc_plain", "LTV-MPC (S és puha ω nélkül)"))
SETS = (("test_shapes", "kezdeti hiba nélkül"), ("test_dynamic_slow", "nagy görbület, 0.1 m/s"),
        ("test_corners_slow", "éles sarkok, 0.1 m/s"), ("test_dynamic", "nagy görbület és sarkok, 0.9 · v_max"))
OFFSET_SET = ("test_shapes_y40_psi45", "40 cm oldalra + 45° kezdeti hiba")


def corner_max(log, w):
    near = np.zeros_like(w)
    for t in corner_times(log):
        near |= (log["t"] >= t) & (log["t"] <= t + 1.0)
    return f"{1e3 * pos_error(log)[w & near].max():.1f}" if near.any() else "–"


def paths(eval_dir, set_name, controllers):
    return sorted(os.path.splitext(f)[0] for f in os.listdir(os.path.join(eval_dir, controllers[0][0], set_name))
                  if f.endswith(".csv"))


def run_rows(eval_dir, set_name, controllers):
    rows = ["| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | "
            "sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for path in paths(eval_dir, set_name, controllers):
        for ctl, label in controllers:
            lg = metrics.load_log(os.path.join(eval_dir, ctl, set_name, f"{path}.csv"))
            m, w = metrics.run_metrics(lg), metrics.window(lg)
            rows.append(
                f"| {path} | {label} | {1e3 * m['pos_rmse_m']:.2f} / {1e3 * m['pos_max_m']:.1f} | "
                f"{1e3 * m['contour_rmse_m']:.2f} / {1e3 * m['contour_max_m']:.1f} | {1e3 * m['lag_mean_m']:+.2f} | "
                f"{corner_max(lg, w)} | {np.degrees(m['heading_rmse_rad']):.2f} / {np.degrees(m['heading_max_rad']):.2f} | "
                f"{m['torque_rms_Nm']:.4f} / {m['torque_max_Nm']:.4f} | {m['action_rate_max']:.4f} | {int(m['failed'])} |")
    return rows


def offset_rows(eval_dir, controllers):
    rows = ["| pálya | irányító | beállás [s] | utána pozíció RMS / max [mm] | parancs \\|u\\| max [Nm] | "
            "kerék max [rad/s] | lépések > 8.06 rad/s | robot v max [m/s] | elveszett |",
            "|---|---|---|---|---|---|---|---|---|"]
    for path in paths(eval_dir, OFFSET_SET[0], controllers):
        for ctl, label in controllers:
            lg = metrics.load_log(os.path.join(eval_dir, ctl, OFFSET_SET[0], f"{path}.csv"))
            m = metrics.run_metrics(lg)
            ts, rms, mx = metrics.settling_time(lg)
            st = f"{ts:.2f}" if np.isfinite(ts) else "nem áll be"
            after = f"{1e3 * rms:.2f} / {1e3 * mx:.1f}" if np.isfinite(ts) else "–"
            n_over = int(np.any(np.abs(metrics._cols(lg, "w_")) > metrics.OMEGA_MAX, axis=1).sum())
            rows.append(f"| {path} | {label} | {st} | {after} | {np.abs(metrics._cols(lg, 'u_')).max():.3f} | "
                        f"{np.abs(metrics._cols(lg, 'w_')).max():.1f} | {n_over} | "
                        f"{np.hypot(lg['vx'], lg['vy']).max():.3f} | {int(m['failed'])} |")
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--eval_dir", default="logs/eval")
    ap.add_argument("--out", default=None, help="markdown file of the tables (default: print only)")
    ap.add_argument("--controllers", nargs="+", default=None, metavar="DIR=LABEL",
                    help="eval folders and their labels (default: the RL vs. MPC comparison)")
    ap.add_argument("--sets", nargs="+", default=[name for name, _ in SETS[:3]], choices=[name for name, _ in SETS])
    args = ap.parse_args()
    controllers = CONTROLLERS if args.controllers is None else [tuple(c.split("=", 1)) for c in args.controllers]

    lines = [f"\n### {OFFSET_SET[0]} ({OFFSET_SET[1]})\n"] + offset_rows(args.eval_dir, controllers)
    lines.append("\nBeállás: az első pillanat, ahonnan a pozícióhiba < 1 cm és az irányhiba < 2° marad legalább 2 s-ig; "
                 "utána: a pozícióhiba onnantól a mozgás végéig. A kezdeti hiba 400 mm és 45°. Kerék / lépések: a teljes "
                 "futás (a motor üresjárati fordulatszáma 8.06 rad/s, a robot csúcssebessége ezzel 0.244 m/s; a "
                 "tapadási határ kerekenként ~0.14 Nm).")
    for set_name, set_label in SETS:
        if set_name in args.sets:
            lines += [f"\n### {set_name} ({set_label})\n"] + run_rows(args.eval_dir, set_name, controllers)
    lines.append("\nAblak: a referencia mozgásának végéig (metrics.py). Sarok utáni max: a pozícióhiba legnagyobb "
                 "értéke a > 30°-os irányugrások utáni 1 s-ban.")
    text = "\n".join(lines) + "\n"
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)


if __name__ == "__main__":
    main()
