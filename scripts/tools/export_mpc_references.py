"""Exports the MPC side's test paths into the RL's reference-table format, so both controllers follow exactly the
same references (docs/OSSZEHANGOLAS.md, 3.7).

Source: mpc/reference.py (= Mecanum-Robot-MPC-TDK @ fe01f88, mecanum_mpc_common/reference.py): circle, ellipse,
figure8, square (rounded, corner radius 15 % of the side), cosine speed ramps (2 s) at both ends, one lap
(loop=False), heading fixed or tangent (raw, continuous), the step-averaged reference torques u_r of its inverse
dynamics on the model of robot_measured.urdf. Every path is checked against the XM430 limits (Reference.check_limits)
and refused when it is not feasible.

Columns: t, X, Y, theta, vx, vy, omega, tau_1..tau_4 (mpc/tables.py).

    python scripts/tools/export_mpc_references.py                         # 0.1 m/s, fixed + tangent (the defaults)
    python scripts/tools/export_mpc_references.py --speed 0.15 --heading fixed --suffix _v015
"""

import argparse
import os
import sys

_PKG = os.path.join(os.path.dirname(__file__), "..", "..", "source", "mecanum_tracking_ws", "mecanum_tracking_ws")
sys.path.insert(0, _PKG)
from mpc import motor, params, tables  # noqa: E402
from mpc.reference import SHAPES, Reference  # noqa: E402

REF_DIR = os.path.join(_PKG, "tasks", "manager_based", "mecanum_tracking", "refs")
URDF = os.path.join(os.path.dirname(__file__), "..", "..", "assets", "robots", "own_mecanum", "robot_measured.urdf")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.1, help="path speed [m/s] (MPC side's default, decided)")
    ap.add_argument("--heading", nargs="+", default=["fixed", "tangent"], choices=["fixed", "tangent"])
    ap.add_argument("--shapes", nargs="+", default=list(SHAPES), choices=list(SHAPES))
    ap.add_argument("--dt", type=float, default=0.01, help="table sampling time [s]")
    ap.add_argument("--T", type=float, default=0.05, help="MPC sampling time [s] of the step-averaged torques")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--out", default=REF_DIR, help="parent folder; files go to <out>/test_<heading><suffix>/")
    ap.add_argument("--suffix", default="")
    args = ap.parse_args()

    p = params.from_urdf(args.urdf)
    for heading in args.heading:
        out = os.path.join(args.out, f"test_{heading}{args.suffix}")
        os.makedirs(out, exist_ok=True)
        for shape in args.shapes:
            ref = Reference(p, shape, None, args.speed, heading)
            maxima, bad = ref.check_limits(args.T, motor.U_MAX, motor.DU_MAX, motor.OMEGA_MAX)
            if bad:
                print(f"SKIPPED {shape} {heading} {args.speed} m/s: " + "; ".join(bad))
                continue
            table = tables.reference_table(ref, args.dt, args.T)
            tables.save_table(os.path.join(out, f"{shape}.csv"), table)
            print(f"{out}/{shape}.csv: {ref.T_end:.1f} s, max |u_r| {maxima['u']:.4f} N m, "
                  f"max wheel speed {maxima['omega']:.2f} rad/s (limit {motor.OMEGA_MAX:.2f})")


if __name__ == "__main__":
    main()
