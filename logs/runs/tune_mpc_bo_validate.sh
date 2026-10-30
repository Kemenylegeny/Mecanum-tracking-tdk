#!/bin/bash
# After logs/runs/tune_mpc_bo.sh, unattended (start it detached: setsid nohup bash logs/runs/tune_mpc_bo_validate.sh &):
# waits for the running Bayesian optimisation, resumes it if it stopped before its last batch, then
#   logs/eval/mpc_bo_table.md          the trials (tools/tune_mpc_bo.py --report), with the best weights
#   logs/eval/mpc_bo_test_table.md     the best weights (mpc_bo) vs. the current ones (mpc_wheel_kin) on the TEST sets
#   logs/eval/mpc_bo_encoder_table.md  the same with the wheel speeds from the encoders (mpc_bo_enc vs. mpc_wheel)
# One Isaac process at a time (shared GPU).
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
OFF="--start_offset 0 0.4 0.785398"
N_TRIALS=$((${BATCHES:-14} * 8))
while pgrep -f "scripts/tools/tune_mpc_bo.py --out logs/tune_mpc/bo" > /dev/null; do sleep 30; done
for attempt in 1 2 3; do
    [ "$(cat logs/tune_mpc/bo/trials.jsonl 2> /dev/null | wc -l)" -ge $N_TRIALS ] && break
    echo "[validate] the optimisation stopped early, resuming (attempt $attempt)"
    pkill -f "eval_tracking.py --controller mpc --mpc_trials"; sleep 10   # a batch left behind
    bash logs/runs/tune_mpc_bo.sh >> logs/runs/tune_mpc_bo.log 2>&1
done
$PY scripts/tools/tune_mpc_bo.py --out logs/tune_mpc/bo --report logs/eval/mpc_bo_table.md > /dev/null
BEST=$(grep -- "^--mpc_Q" logs/eval/mpc_bo_table.md)
echo "[validate] best weights: $BEST"
ev() {
    name=$1; shift
    $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc "$@" --headless > logs/runs/eval_$name.log 2>&1
    grep -qE "Traceback|PhysX error|out of memory" logs/runs/eval_$name.log && echo "FAILED $name"
}
for v in "mpc_bo --mpc_wheel_meas kinematic" "mpc_bo_enc --mpc_wheel_meas encoder"; do
    set -- $v; who=$1; shift
    ev ${who}_test_shapes $BEST "$@" --out logs/eval/$who/test_shapes --traj_dir $R/test_shapes
    ev ${who}_test_shapes_y40_psi45 $BEST "$@" --out logs/eval/$who/test_shapes_y40_psi45 --traj_dir $R/test_shapes $OFF
    ev ${who}_test_dynamic_slow $BEST "$@" --out logs/eval/$who/test_dynamic_slow --traj_dir $R/test_dynamic_slow
done
ev mpc_bo_test_corners_slow $BEST --out logs/eval/mpc_bo/test_corners_slow --traj_dir $R/test_corners_slow
ev mpc_bo_test_dynamic $BEST --out logs/eval/mpc_bo/test_dynamic --traj_dir $R/test_dynamic
$PY scripts/tools/report_sharp_eval.py --controllers "mpc_wheel_kin=LTV-MPC, jelenlegi súlyok" "mpc_bo=LTV-MPC, BO-súlyok" \
    --sets test_shapes test_dynamic_slow test_corners_slow test_dynamic --out logs/eval/mpc_bo_test_table.md > /dev/null
$PY scripts/tools/report_sharp_eval.py --controllers "mpc_wheel=jelenlegi súlyok, enkóder" "mpc_bo_enc=BO-súlyok, enkóder" \
    --sets test_shapes test_dynamic_slow --out logs/eval/mpc_bo_encoder_table.md > /dev/null
echo "[validate] DONE"
