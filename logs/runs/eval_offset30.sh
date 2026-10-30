#!/bin/bash
# 30 cm sideways + 30 deg initial error on refs/test_shapes (0.1 m/s): the plain LTV-MPC with the Bryson weights
# (mpc_wheel_kin), with the weights of the Bayesian optimisation (mpc_bo, logs/eval/mpc_bo_table.md) and the RL trained
# on sharp corners and tight turns too (rl_sharp_seed0, model_276), all without the torque-rate limit (removed
# 2026-10-30). One run at a time (shared GPU).
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
OFF="--start_offset 0 0.3 0.523599"
SET=test_shapes_y30_psi30
SHARP=logs/rsl_rl/mecanum_tracking_torque/2026-10-28_22-03-21_torque_sharp_seed0/model_276.pt
BEST=$(grep -- "^--mpc_Q" logs/eval/mpc_bo_table.md)
ev() {
    name=$1; shift
    $PY -u scripts/rsl_rl/eval_tracking.py "$@" --traj_dir $R/test_shapes $OFF --headless > logs/runs/eval_${name}_$SET.log 2>&1
    grep -qE "Traceback|PhysX error|out of memory" logs/runs/eval_${name}_$SET.log && echo "FAILED $name"
}
ev mpc_wheel_kin --controller mpc --mpc_Q 40000 40000 3283 2500 2500 100 --out logs/eval/mpc_wheel_kin/$SET
ev mpc_bo --controller mpc $BEST --out logs/eval/mpc_bo/$SET
ev rl_sharp_seed0 --controller policy --checkpoint $SHARP --out logs/eval/rl_sharp_seed0/$SET
echo DONE
