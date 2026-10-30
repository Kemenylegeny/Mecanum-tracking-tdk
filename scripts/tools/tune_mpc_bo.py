"""Bayesian optimisation of the LTV-MPC's weights in the closed loop: every trial runs in Isaac Lab through
scripts/rsl_rl/eval_tracking.py (the evaluation protocol of the comparison), on the tuning set refs/tune_mpc: paths of
the training generator (tools/gen_training_trajectories.py, own seeds), smooth curves, polygons with sharp corners and
tight curves; never the test sets (docs/OSSZEHANGOLAS.md: tune on training paths).

Parameters (5, searched in log10): q_pos (= Q_X = Q_Y), q_psi, q_v (= Q_Xdot = Q_Ydot), q_omega (= Q_psidot) and s (the
torque-change penalty S = s I; its lower bound means off). r = 400 stays fixed: the MPC's cost is invariant to a common
scale of Q, R and S (no soft constraint is on), so r is the unit.

Objective (lower is better), mean over the paths:
    J = RMSE_pos [mm] + L_PSI * RMSE_psi [mm]             (L_PSI = 0.1 m: 1 deg of heading error counts 1.75 mm)
        * max(1, du_rms / DU_CAP) ** 2                   (torque activity above the cap is penalised)
A path the robot loses (deviation termination) counts LOST_MM. The metrics of every trial are stored, so J can be
recomputed with other constants.

Batches of --batch trials run in one Isaac process (eval_tracking.py --mpc_trials, the controllers in --workers
processes). The first batch: the current weights + Sobol points, the second: Sobol points; then a Gaussian process
(Matern 5/2, ARD, hyperparameters by maximum likelihood) on log J, expected improvement, the batch by kriging believer.

    python scripts/tools/tune_mpc_bo.py --out logs/tune_mpc/bo --batches 14      # resumes from OUT/trials.jsonl
"""

import argparse
import glob
import json
import os
import subprocess
import sys

import numpy as np
from scipy.optimize import minimize
from scipy.stats import norm, qmc

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
REFS = os.path.join(ROOT, "source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs")
PY = sys.executable
NAMES = ["q_pos", "q_psi", "q_v", "q_omega", "s"]
LO = np.array([3.0, 1.5, 1.0, -1.0, -1.0])
HI = np.array([7.0, 6.0, 5.5, 4.0, 4.0])
DEFAULT = np.array([np.log10(40000.0), np.log10(3283.0), np.log10(2500.0), 2.0, LO[4]])
R_FIXED = 400.0
L_PSI = 0.1
LOST_MM = 100.0
OMEGA_MAX = 8.06


def weights(x):
    """log10 parameters -> the eval_tracking.py --mpc_trials entry."""
    q_pos, q_psi, q_v, q_om, s = 10.0 ** np.asarray(x, dtype=float)
    return dict(Q=[q_pos, q_pos, q_psi, q_v, q_v, q_om], R=R_FIXED, S=0.0 if x[4] <= LO[4] + 1e-9 else float(s))


def path_metrics(csv, t_end):
    a = np.genfromtxt(csv, delimiter=",", names=True)
    e = np.hypot(a["x"] - a["x_ref"], a["y"] - a["y_ref"]) * 1e3
    e_psi = (a["yaw"] - a["yaw_ref"] + np.pi) % (2 * np.pi) - np.pi
    u = np.stack([a[f"u_{j}"] for j in range(4)], 1)
    w = np.stack([a[f"w_{j}"] for j in range(4)], 1)
    lost = bool(a["failed"].max() > 0 or a["t"][-1] < t_end - 0.05 - 1e-6)  # the log stops at the last step <= t_end
    return dict(rmse_mm=float(np.sqrt(np.mean(e ** 2))), max_mm=float(e.max()),
                rmse_psi_deg=float(np.degrees(np.sqrt(np.mean(e_psi ** 2)))),
                du_rms_mnm=float(1e3 * np.sqrt(np.mean(np.diff(u, axis=0) ** 2))),
                u_rms_mnm=float(1e3 * np.sqrt(np.mean(u ** 2))),
                overspeed_steps=int(np.sum(np.abs(w).max(1) > OMEGA_MAX)), lost=lost)


def objective(paths, du_cap):
    track = [LOST_MM if m["lost"] else m["rmse_mm"] + L_PSI * 1e3 * np.radians(m["rmse_psi_deg"]) for m in paths]
    du = np.mean([m["du_rms_mnm"] for m in paths])
    return float(np.mean(track) * max(1.0, du / du_cap) ** 2)


def run_batch(xs, out, tune_dir, workers, hold):
    """One Isaac process: every weight set on every tuning path. Returns the per-path metrics of each set."""
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "trials.json"), "w") as f:
        json.dump([weights(x) for x in xs], f, indent=1)
    cmd = [PY, "-u", "scripts/rsl_rl/eval_tracking.py", "--controller", "mpc", "--mpc_trials",
           os.path.join(out, "trials.json"), "--mpc_workers", str(workers), "--traj_dir", tune_dir, "--out", out,
           "--hold", str(hold), "--headless"]
    for attempt in range(2):
        with open(out + ".log", "w") as log:
            subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        text = open(out + ".log").read()
        if "[eval] mpc" in text and not any(k in text for k in ("Traceback", "PhysX error", "out of memory")):
            break
        print(f"[bo] {out}: the Isaac run failed (attempt {attempt + 1}), see {out}.log", flush=True)
    else:
        raise SystemExit("two failed Isaac runs in a row")
    files = sorted(glob.glob(os.path.join(tune_dir, "*.csv")))
    ends = {os.path.basename(f)[:-4]: np.loadtxt(f, delimiter=",", skiprows=1, usecols=0)[-1] for f in files}
    return [{n: path_metrics(os.path.join(out, f"trial_{k:03d}", n + ".csv"), e + hold) for n, e in ends.items()}
            for k in range(len(xs))]


class GP:
    """Zero-mean GP on standardised targets, Matern 5/2 with one length scale per input, inputs in [0, 1]^d."""

    def __init__(self, X, y):
        self.X, self.mu, self.sd = X, y.mean(), y.std() + 1e-12
        self.y = (y - self.mu) / self.sd
        d = X.shape[1]
        bounds = [(np.log(0.03), np.log(3.0))] * d + [(np.log(0.05), np.log(20.0)), (np.log(1e-6), np.log(0.5))]
        rng = np.random.default_rng(0)
        best = None
        for start in [np.r_[np.full(d, np.log(0.3)), 0.0, np.log(1e-2)]] + \
                [np.array([rng.uniform(*b) for b in bounds]) for _ in range(5)]:
            r = minimize(self._nll, start, method="L-BFGS-B", bounds=bounds)
            if best is None or r.fun < best.fun:
                best = r
        self.theta = best.x
        self._factor()

    @staticmethod
    def _k(A, B, ls, sf2):
        r = np.sqrt(np.maximum(np.sum(((A[:, None, :] - B[None, :, :]) / ls) ** 2, -1), 0.0)) * np.sqrt(5.0)
        return sf2 * (1.0 + r + r ** 2 / 3.0) * np.exp(-r)

    def _parts(self, theta):
        d = self.X.shape[1]
        return np.exp(theta[:d]), np.exp(theta[d]), np.exp(theta[d + 1])

    def _nll(self, theta):
        ls, sf2, sn2 = self._parts(theta)
        K = self._k(self.X, self.X, ls, sf2) + (sn2 + 1e-9) * np.eye(len(self.X))
        try:
            L = np.linalg.cholesky(K)
        except np.linalg.LinAlgError:
            return 1e10
        a = np.linalg.solve(L.T, np.linalg.solve(L, self.y))
        return 0.5 * self.y @ a + np.sum(np.log(np.diag(L)))

    def _factor(self):
        ls, sf2, sn2 = self._parts(self.theta)
        K = self._k(self.X, self.X, ls, sf2) + (sn2 + 1e-9) * np.eye(len(self.X))
        self.L = np.linalg.cholesky(K)
        self.alpha = np.linalg.solve(self.L.T, np.linalg.solve(self.L, self.y))

    def add(self, x, y):
        """Kriging believer: x with the target y, the hyperparameters kept."""
        self.X = np.vstack([self.X, x])
        self.y = np.r_[self.y, (y - self.mu) / self.sd]
        self._factor()

    def predict(self, Xs):
        ls, sf2, _ = self._parts(self.theta)
        Ks = self._k(Xs, self.X, ls, sf2)
        v = np.linalg.solve(self.L, Ks.T)
        var = np.maximum(sf2 - np.sum(v ** 2, 0), 1e-12)
        return self.mu + self.sd * Ks @ self.alpha, self.sd * np.sqrt(var)

    def lengthscales(self):
        return self._parts(self.theta)[0]


def expected_improvement(gp, Xs, best):
    m, s = gp.predict(Xs)
    z = (best - m) / s
    return (best - m) * norm.cdf(z) + s * norm.pdf(z)


def propose(X, y, n, seed):
    """n points in [0, 1]^d: EI of a GP on y = log J, maximised over Sobol points + local refinement, one at a time,
    the GP told its own mean at each chosen point (kriging believer)."""
    gp = GP(X, y)
    print("[bo] GP length scales (unit cube): " + ", ".join(f"{a} {b:.2f}" for a, b in zip(NAMES, gp.lengthscales())),
          flush=True)
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        best = gp.y.min() * gp.sd + gp.mu
        cand = qmc.Sobol(X.shape[1], seed=rng).random(4096)
        top = gp.X[np.argsort(gp.y)[:5]]
        cand = np.vstack([cand, np.clip(top[rng.integers(0, len(top), 2048)] + rng.normal(0, 0.05, (2048, X.shape[1])),
                                        0, 1)])
        ei = expected_improvement(gp, cand, best)
        x_best, ei_best = cand[np.argmax(ei)], ei.max()
        for x0 in cand[np.argsort(ei)[-5:]]:
            r = minimize(lambda x: -expected_improvement(gp, x[None], best)[0], x0, method="L-BFGS-B",
                         bounds=[(0, 1)] * X.shape[1])
            if -r.fun > ei_best:
                x_best, ei_best = r.x, -r.fun
        out.append(x_best)
        gp.add(x_best, gp.predict(x_best[None])[0][0])
    return np.array(out)


def family(path):
    return path.split("_")[1]  # tune_<family>_<i>_<heading>


def report(trials, du_cap, md):
    """Markdown tables of the trials: the current weights vs. the best, the best few, the trade-off with the torque
    activity (non-dominated trials) and the GP's length scales (which parameters matter)."""
    J = np.array([objective(list(t["paths"].values()), du_cap) for t in trials])
    track = np.array([objective(list(t["paths"].values()), np.inf) for t in trials])
    du = np.array([t["du_rms_mnm"] for t in trials])
    order = np.argsort(J)
    k0, kb = 0, int(order[0])
    fams = sorted({family(p) for p in trials[0]["paths"]})

    def wcells(t):
        q = t["weights"]["Q"]
        return f"{q[0]:.3g} | {q[2]:.3g} | {q[3]:.3g} | {q[5]:.3g} | {t['weights']['S']:.3g}"

    def fam(t, f, key):
        return np.mean([m[key] for p, m in t["paths"].items() if family(p) == f])

    lines = [f"# Az MPC súlyainak Bayes-optimalizálása ({len(trials)} próba)", "",
             "`tools/tune_mpc_bo.py`, `logs/runs/tune_mpc_bo.sh`: zárt hurok Isaacben (eval_tracking.py), a "
             "`refs/tune_mpc` 9 pályáján (a tanítópálya-generátorból: 3 sima, 3 éles sarkú sokszög, 3 szűk görbe; "
             "nem a tesztpályák). r = 400 rögzített.", "",
             f"Célfüggvény: a pályák átlagában RMSE_pos + 0.1 m · RMSE_ψ [mm], szorozva max(1, Δu_RMS / {du_cap:.2f} mNm)²-tel "
             "(a sapka a jelenlegi súlyok nyomatékváltozásának háromszorosa); elvesztett pálya: 100 mm.", "",
             "## Jelenlegi és legjobb súlyok", "",
             "| | q_pos | q_ψ | q_v | q_ω | s | J | " + " | ".join(f"RMSE {f} [mm]" for f in fams)
             + " | max [mm] | RMSE ψ [°] | Δu RMS [mNm] |", "|" + "---|" * (10 + len(fams))]
    for label, k in (("jelenlegi", k0), (f"legjobb (#{kb})", kb)):
        t = trials[k]
        ps = list(t["paths"].values())
        lines.append(f"| {label} | {wcells(t)} | {J[k]:.2f} | " + " | ".join(f"{fam(t, f, 'rmse_mm'):.2f}" for f in fams)
                     + f" | {max(m['max_mm'] for m in ps):.2f} | {np.mean([m['rmse_psi_deg'] for m in ps]):.3f} | "
                     f"{du[k]:.2f} |")
    lines += ["", "## A legjobb 8 próba", "", "| # | batch | q_pos | q_ψ | q_v | q_ω | s | J | követési hiba [mm] | "
              "Δu RMS [mNm] |", "|---|---|---|---|---|---|---|---|---|---|"]
    for k in order[:8]:
        lines.append(f"| {k} | {trials[k]['batch']} | {wcells(trials[k])} | {J[k]:.2f} | {track[k]:.2f} | {du[k]:.2f} |")
    nd = [k for k in np.argsort(du) if not np.any((du < du[k]) & (track <= track[k]) | (du <= du[k]) & (track < track[k]))]
    lines += ["", "## Kompromisszum: követési hiba és nyomatékaktivitás (nem dominált próbák)", "",
              "| # | q_pos | q_ψ | q_v | q_ω | s | követési hiba [mm] | Δu RMS [mNm] |", "|---|---|---|---|---|---|---|---|"]
    for k in nd:
        lines.append(f"| {k} | {wcells(trials[k])} | {track[k]:.2f} | {du[k]:.2f} |")
    gp = GP(np.array([t["u"] for t in trials]), np.log(J))
    lines += ["", "## Melyik paraméter számít (a GP hosszskálái, egységkockában: kicsi = érzékeny)", "",
              "| " + " | ".join(NAMES) + " |", "|" + "---|" * len(NAMES),
              "| " + " | ".join(f"{v:.2f}" for v in gp.lengthscales()) + " |", ""]
    t = trials[kb]["weights"]
    lines += ["A legjobb súlyok az eval_tracking.py-nak:", "", "```",
              f"--mpc_Q {' '.join(f'{v:.6g}' for v in t['Q'])} --mpc_R {t['R']:g} --mpc_S {t['S']:.6g}", "```", ""]
    with open(md, "w") as f:
        f.write("\n".join(lines))
    print("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="logs/tune_mpc/bo")
    ap.add_argument("--tune_dir", default=os.path.join(REFS, "tune_mpc"))
    ap.add_argument("--batches", type=int, default=14, help="total number of batches (resumes from OUT/trials.jsonl)")
    ap.add_argument("--batch", type=int, default=8, help="trials per batch (one Isaac process)")
    ap.add_argument("--workers", type=int, default=10, help="MPC worker processes")
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--du_cap", type=float, default=None,
                    help="torque-change RMS cap [mNm per step] (default: 3x that of the current weights, batch 0)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--report", default=None, help="only write the tables of OUT/trials.jsonl to this markdown file")
    args = ap.parse_args()
    out = os.path.join(ROOT, args.out)
    os.makedirs(out, exist_ok=True)
    db = os.path.join(out, "trials.jsonl")
    trials = [json.loads(line) for line in open(db)] if os.path.exists(db) else []
    if args.report:
        return report(trials, args.du_cap or 3.0 * trials[0]["du_rms_mnm"], args.report)
    sobol = qmc.Sobol(5, seed=args.seed).random(2 * args.batch)
    for b in range(len(trials) // args.batch, args.batches):
        if b == 0:
            U = np.vstack([(DEFAULT - LO) / (HI - LO), sobol[:args.batch - 1]])
        elif b == 1:
            U = sobol[args.batch - 1:2 * args.batch - 1]
        else:
            du_cap = args.du_cap or 3.0 * trials[0]["du_rms_mnm"]
            X = np.array([t["u"] for t in trials])
            y = np.log([objective(list(t["paths"].values()), du_cap) for t in trials])
            U = propose(X, y, args.batch, args.seed + b)
        xs = LO + U * (HI - LO)
        res = run_batch(xs, os.path.join(out, f"batch_{b:02d}"), args.tune_dir, args.workers, args.hold)
        with open(db, "a") as f:
            for u, x, paths in zip(U, xs, res):
                t = dict(batch=b, u=u.tolist(), log10=x.tolist(), weights=weights(x), paths=paths,
                         du_rms_mnm=float(np.mean([m["du_rms_mnm"] for m in paths.values()])))
                trials.append(t)
                f.write(json.dumps(t) + "\n")
        du_cap = args.du_cap or 3.0 * trials[0]["du_rms_mnm"]
        J = [objective(list(t["paths"].values()), du_cap) for t in trials]
        k = int(np.argmin(J))
        print(f"[bo] batch {b}: {len(trials)} trials; J of this batch "
              + " ".join(f"{j:.2f}" for j in J[-args.batch:]) + f"; best J {J[k]:.3f} (trial {k}: "
              + ", ".join(f"{n} {10 ** v:.4g}" for n, v in zip(NAMES, trials[k]["log10"]))
              + f"); current weights J {J[0]:.3f}; du cap {du_cap:.2f} mNm", flush=True)


if __name__ == "__main__":
    main()
