# MPC ablation on refs/test_corners_{slow,fast} (scripts/tools/gen_corner_references.py): sharp square, straight + 135 deg
# turn, smooth low-curvature path; 0.1 m/s and 0.9 v_max, fixed heading. All variants: DARE terminal, no Q_path.
#   mpc         S = 100 I + soft wheel-speed limit (the default until 2026-10-27)
#   mpc_plain   without S and without the soft wheel-speed limit (the plain LTV-MPC: Q, R, P, hard torque box; the
#               default since 2026-10-28)
#   mpc_noS     only S off;  mpc_nosoft  only the soft wheel-speed limit off
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
$PY scripts/tools/gen_corner_references.py --speed 0.1 --out $R/test_corners_slow
$PY scripts/tools/gen_corner_references.py --alpha 0.9 --out $R/test_corners_fast
for SET in test_corners_slow test_corners_fast; do
  $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_S 100 --mpc_wheel_limit --out logs/eval/mpc/$SET --traj_dir $R/$SET --headless > logs/runs/eval_mpc_$SET.log 2>&1
  $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --out logs/eval/mpc_plain/$SET --traj_dir $R/$SET --headless > logs/runs/eval_mpc_plain_$SET.log 2>&1
  $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_wheel_limit --out logs/eval/mpc_noS/$SET --traj_dir $R/$SET --headless > logs/runs/eval_mpc_noS_$SET.log 2>&1
  $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_S 100 --out logs/eval/mpc_nosoft/$SET --traj_dir $R/$SET --headless > logs/runs/eval_mpc_nosoft_$SET.log 2>&1
done
$PY scripts/tools/report_mpc_ablation.py --fig_dir logs/eval/mpc_plain
echo DONE
