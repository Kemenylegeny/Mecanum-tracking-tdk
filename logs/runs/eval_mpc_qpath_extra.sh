# extra Q_path runs: usage bash ... "<QA QC>" <SET> [offset]
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
QA_QC=$1; SET=$2; NAME=qa${QA_QC% *}_qc${QA_QC#* }; OUT=$SET; EXTRA=""
if [ "$3" = offset ]; then OUT=${SET}_offset; EXTRA="--start_offset 0.3 0.0 0.52"; fi
$PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_state world --mpc_S 100 --mpc_wheel_limit --mpc_Q_path $QA_QC $EXTRA --out logs/eval/mpc_qpath/$NAME/$OUT \
  --traj_dir $R/$SET --headless > logs/runs/eval_mpc_qpath_${NAME}_$OUT.log 2>&1
echo "$NAME $OUT done $(date +%H:%M:%S) exit $?"
