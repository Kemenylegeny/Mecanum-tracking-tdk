# Q_path sweep (along-track qa, cross-track qc); Bryson baseline = 40000 40000. Usage: bash ... <SET>
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
SET=$1
for QA_QC in "40000 40000" "160000 40000" "640000 40000" "2560000 40000" "10000 40000" "640000 10000" "640000 160000"; do
  NAME=qa${QA_QC% *}_qc${QA_QC#* }
  $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --mpc_Q_path $QA_QC --out logs/eval/mpc_qpath/$NAME/$SET \
    --traj_dir $R/$SET --headless > logs/runs/eval_mpc_qpath_${NAME}_$SET.log 2>&1
  echo "$NAME $SET done $(date +%H:%M:%S) exit $?"
done
