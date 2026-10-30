#!/bin/bash
# The MPC with three wheel speeds as velocity states (mpc/wheel_dynamics.py, the default since 2026-10-29) vs. the
# world-frame velocity states (--mpc_state world, until 2026-10-28), all plain (no S, no soft wheel-speed limit):
#   mpc_wheel       wheel speeds from the four encoders, projected onto the rolling constraints
#   mpc_wheel_kin   wheel speeds from the ground-truth base velocity (no slip): the same information as mpc_plain and
#                   the policy (the default)
#   mpc_plain       world-frame states (logs/runs/eval_sharp_policy.sh, eval_mpc_plain_corners.sh); first re-run here on
#                   test_corners_slow to check that --mpc_state world still reproduces it bit for bit
# on the sets of eval_sharp_policy.sh (0.1 m/s) and refs/test_dynamic (0.9 v_max: roller slip, where the encoders and
# the base velocity disagree most). One run at a time (shared GPU; a failed PhysX allocation leaves the robots frozen).
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
OFF="--start_offset 0 0.4 0.785398"
ev() {
    name=$1; shift
    $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_Q 40000 40000 3283 2500 2500 100 "$@" --headless > logs/runs/eval_$name.log 2>&1
    grep -qE "Traceback|PhysX error|out of memory" logs/runs/eval_$name.log && echo "FAILED $name"
}
ev mpc_plain_check_test_corners_slow --mpc_state world --out logs/eval/mpc_plain_check/test_corners_slow --traj_dir $R/test_corners_slow
$PY - <<'EOF'  # every column but the timings (compute_ms, qp_ms differ from run to run)
import glob, numpy as np
for f in sorted(glob.glob("logs/eval/mpc_plain/test_corners_slow/*.csv")):
    head = open(f).readline().strip().split(",")
    a, b = (np.loadtxt(g, delimiter=",", skiprows=1) for g in (f, f.replace("mpc_plain/", "mpc_plain_check/")))
    keep = [k for k, h in enumerate(head) if h not in ("compute_ms", "qp_ms")]
    print(f, "identical" if a.shape == b.shape and np.array_equal(a[:, keep], b[:, keep]) else "DIFFERS (world state)")
EOF
for v in "mpc_wheel --mpc_wheel_meas encoder" "mpc_wheel_kin --mpc_wheel_meas kinematic"; do
    set -- $v; who=$1; shift
    ev ${who}_test_shapes "$@" --out logs/eval/$who/test_shapes --traj_dir $R/test_shapes
    ev ${who}_test_shapes_y40_psi45 "$@" --out logs/eval/$who/test_shapes_y40_psi45 --traj_dir $R/test_shapes $OFF
    ev ${who}_test_dynamic_slow "$@" --out logs/eval/$who/test_dynamic_slow --traj_dir $R/test_dynamic_slow
    ev ${who}_test_corners_slow "$@" --out logs/eval/$who/test_corners_slow --traj_dir $R/test_corners_slow
    ev ${who}_test_dynamic "$@" --out logs/eval/$who/test_dynamic --traj_dir $R/test_dynamic
done
ev mpc_plain_test_dynamic --mpc_state world --out logs/eval/mpc_plain/test_dynamic --traj_dir $R/test_dynamic
echo DONE
