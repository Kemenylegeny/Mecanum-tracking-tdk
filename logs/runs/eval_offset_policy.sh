# RL trained from random initial errors (torque_offset_seed0, model_311.pt) vs. the old RL (torque_seed0, model_600.pt) and the MPC:
# refs/test_shapes without / with the 30 cm / 30 deg initial error, refs/test_dynamic
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
NEW=logs/rsl_rl/mecanum_tracking_torque/2026-10-26_13-39-49_torque_offset_seed0/model_311.pt
OLD=logs/rsl_rl/mecanum_tracking_torque/2026-10-24_17-38-40_torque_seed0/model_600.pt
OFF="--start_offset 0.3 0.0 0.52"
ev() { $PY -u scripts/rsl_rl/eval_tracking.py "$@" --headless; }
ev --task Mecanum-Tracking-Torque-Offset-Eval-v0 --controller policy --checkpoint $NEW --out logs/eval/rl_offset_seed0/test_shapes --traj_dir $R/test_shapes > logs/runs/eval_rl_offset_seed0_test_shapes.log 2>&1
ev --task Mecanum-Tracking-Torque-Offset-Eval-v0 --controller policy --checkpoint $NEW --out logs/eval/rl_offset_seed0/test_shapes_offset --traj_dir $R/test_shapes $OFF > logs/runs/eval_rl_offset_seed0_test_shapes_offset.log 2>&1
ev --task Mecanum-Tracking-Torque-Offset-Eval-v0 --controller policy --checkpoint $OLD --out logs/eval/rl_seed0/test_shapes_offset --traj_dir $R/test_shapes $OFF > logs/runs/eval_rl_seed0_test_shapes_offset.log 2>&1
ev --controller mpc --mpc_S 100 --mpc_wheel_limit --out logs/eval/mpc/test_shapes_offset --traj_dir $R/test_shapes $OFF > logs/runs/eval_mpc_test_shapes_offset.log 2>&1
ev --task Mecanum-Tracking-Torque-Offset-Eval-v0 --controller policy --checkpoint $NEW --out logs/eval/rl_offset_seed0/test_dynamic --traj_dir $R/test_dynamic > logs/runs/eval_rl_offset_seed0_test_dynamic.log 2>&1
ev --task Mecanum-Tracking-Torque-Offset-Eval-v0 --controller policy --checkpoint $OLD --out logs/eval/rl_seed0/test_dynamic --traj_dir $R/test_dynamic > logs/runs/eval_rl_seed0_test_dynamic.log 2>&1
echo DONE
