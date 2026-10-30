#!/bin/bash
# Initial-error grid on refs/test_shapes: lateral offset dy = 10 / 20 / 30 cm (left of the start pose), heading offset
# 10 / 30 / 50 deg, each alone and all 9 combinations; the offset-trained RL (model_311.pt) and the MPC.
# 5 processes at a time (the compute_ms column of these logs is NOT comparable: shared CPU). Isaac exits with 0
# even after an exception, so success = the "[eval] ... trajectories" line of the log.
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs/test_shapes
CKPT=logs/rsl_rl/mecanum_tracking_torque/2026-10-26_13-39-49_torque_offset_seed0/model_311.pt
jobs() {
    for spec in 10:0 20:0 30:0 0:10 0:30 0:50 10:10 10:30 10:50 20:10 20:30 20:50 30:10 30:30 30:50; do
        cm=${spec%%:*}; deg=${spec##*:}
        dy=$(python3 -c "print($cm / 100)"); dpsi=$(python3 -c "import math; print(math.radians($deg))")
        name=y${cm}_psi${deg}
        echo "--task Mecanum-Tracking-Torque-Offset-Eval-v0 --controller policy --checkpoint $CKPT --out logs/eval/rl_offset_seed0/offsets/$name --log logs/runs/offsets/rl_$name.log --start_offset 0 $dy $dpsi"
        echo "--controller mpc --mpc_Q 40000 40000 3283 2500 2500 100 --mpc_state world --mpc_S 100 --mpc_wheel_limit --out logs/eval/mpc/offsets/$name --log logs/runs/offsets/mpc_$name.log --start_offset 0 $dy $dpsi"
    done
}
mkdir -p logs/runs/offsets
jobs | OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 xargs -P 5 -L 1 bash -c '
    log=$(echo "$@" | sed -E "s/.*--log ([^ ]+).*/\1/"); args=$(echo "$@" | sed -E "s/--log [^ ]+ //")
    '"$PY"' -u scripts/rsl_rl/eval_tracking.py $args --traj_dir '"$R"' --headless > $log 2>&1; grep -q "\[eval\] .* trajectories" $log || echo "FAILED $log"' _
echo DONE
