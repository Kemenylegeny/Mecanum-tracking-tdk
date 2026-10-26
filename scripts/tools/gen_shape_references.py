"""Shape test set: circle, figure eight, square with rounded and with sharp corners, all from the MPC side's Reference
(mpc/reference.py), so both controllers follow exactly the MPC side's conventions: 0.1 m/s, cosine speed ramps (2 s) at
both ends, one lap (loop=False), fixed heading, step-averaged reference torques u_r of its inverse dynamics.

    circle          = test_fixed/circle.csv   (radius 0.75 m)
    figure8         = test_fixed/figure8.csv  (lemniscate of Gerono, 2 x 1 m)
    rounded_square  = test_fixed/square.csv   (side 1.5 m, corner radius 0.225 m = 15 % of the side)
    sharp_square    side 1.5 m, corner radius 1 mm: the 90 deg turn takes 16 ms at 0.1 m/s, less than one table sample
                    (10 ms grid -> the reference velocity turns between two rows) and a third of a control step, so the
                    velocity steps at each corner like a sharp square; the impulse m_e * dv is still integrated exactly
                    into the step-averaged u_r (Reference's 2 ms grid resolves the arc). Fixed heading only: a tangent
                    heading would need a 90 deg yaw step at the corners.

Columns: t, X, Y, theta, vx, vy, omega, tau_1..tau_4 (mpc/tables.py).

    python scripts/tools/gen_shape_references.py            # -> refs/test_shapes/
"""

import argparse
import os
import sys

_PKG = os.path.join(os.path.dirname(__file__), "..", "..", "source", "mecanum_tracking_ws", "mecanum_tracking_ws")
sys.path.insert(0, _PKG)
from mpc import motor, params, tables  # noqa: E402
from mpc.reference import Reference  # noqa: E402

REF_DIR = os.path.join(_PKG, "tasks", "manager_based", "mecanum_tracking", "refs")
URDF = os.path.join(os.path.dirname(__file__), "..", "..", "assets", "robots", "own_mecanum", "robot_measured.urdf")
SHAPES = {  # file name: (Reference shape, corner radius; None = the MPC side's default)
    "circle": ("circle", None),
    "figure8": ("figure8", None),
    "rounded_square": ("square", None),
    "sharp_square": ("square", 0.001),
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.1, help="path speed [m/s] (MPC side's default)")
    ap.add_argument("--dt", type=float, default=0.01, help="table sampling time [s]")
    ap.add_argument("--T", type=float, default=0.05, help="MPC sampling time [s] of the step-averaged torques")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--out", default=os.path.join(REF_DIR, "test_shapes"))
    args = ap.parse_args()

    p = params.from_urdf(args.urdf)
    os.makedirs(args.out, exist_ok=True)
    for name, (shape, rc) in SHAPES.items():
        ref = Reference(p, shape, None, args.speed, "fixed", corner_radius=rc)
        maxima, bad = ref.check_limits(args.T, motor.U_MAX, motor.DU_MAX, motor.OMEGA_MAX)
        if bad:
            print(f"SKIPPED {name}: " + "; ".join(bad))
            continue
        tables.save_table(os.path.join(args.out, f"{name}.csv"), tables.reference_table(ref, args.dt, args.T))
        print(f"{args.out}/{name}.csv: {ref.T_end:.1f} s, max |u_r| {maxima['u']:.4f} N m, "
              f"max |du_r| {maxima['du']:.4f} N m/step, max wheel speed {maxima['omega']:.2f} rad/s")


if __name__ == "__main__":
    main()
