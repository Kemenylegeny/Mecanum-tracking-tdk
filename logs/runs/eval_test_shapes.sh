# RL (torque_seed0, model_600.pt) vs. MPC on refs/test_shapes (circle, figure8, rounded and sharp square; 0.1 m/s, fixed heading)
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
CKPT=logs/rsl_rl/mecanum_tracking_torque/2026-10-24_17-38-40_torque_seed0/model_600.pt
$PY -u scripts/rsl_rl/eval_tracking.py --task Mecanum-Tracking-Torque-Offset-Eval-v0 --controller policy --checkpoint $CKPT --out logs/eval/rl_seed0/test_shapes --traj_dir $R/test_shapes --headless > logs/runs/eval_rl_seed0_test_shapes.log 2>&1
$PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_Q 40000 40000 3283 2500 2500 100 --mpc_state world --mpc_S 100 --mpc_wheel_limit --out logs/eval/mpc/test_shapes --traj_dir $R/test_shapes --headless > logs/runs/eval_mpc_test_shapes.log 2>&1
$PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_Q 40000 40000 3283 2500 2500 100 --mpc_state world --mpc_S 100 --mpc_wheel_limit --mpc_Q_path 2560000 160000 --out logs/eval/mpc_qpath/qa2560000_qc160000/test_shapes --traj_dir $R/test_shapes --headless > logs/runs/eval_mpc_qpath_qa2560000_qc160000_test_shapes.log 2>&1
echo DONE
