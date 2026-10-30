#!/bin/bash
# Bayesian optimisation of the plain LTV-MPC's weights (q_pos, q_psi, q_v, q_omega, s; r = 400) in the closed loop,
# tools/tune_mpc_bo.py: 14 batches x 8 trials, each batch one Isaac process (eval_tracking.py --mpc_trials, the
# controllers in 10 worker processes), on refs/tune_mpc (training-generator paths, not the test sets):
#   R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
#   G=scripts/tools/gen_training_trajectories.py
#   python $G --own --ramp --kind smooth --max_length 3 --prefix tune_smooth --n 3 --seed 31 --out $R/tune_mpc
#   python $G --own --ramp --kind polygon --prefix tune_polygon --n 3 --seed 32 --out $R/tune_mpc
#   python $G --own --ramp --kind tight --prefix tune_tight --n 3 --seed 33 --out $R/tune_mpc
# Resumes from logs/tune_mpc/bo/trials.jsonl. One Isaac process at a time (shared GPU).
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
$PY -u scripts/tools/tune_mpc_bo.py --out logs/tune_mpc/bo --batches ${BATCHES:-14} --batch 8 --workers ${WORKERS:-10}
