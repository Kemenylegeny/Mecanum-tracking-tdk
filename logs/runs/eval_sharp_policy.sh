#!/bin/bash
# The RL trained on sharp corners and tight turns too (torque_sharp_seed0, model_276.pt) vs. the same config trained on
# the smooth references only (torque_plain_seed0, model_282.pt) and the plain LTV-MPC (no S, no soft wheel-speed limit),
# all at 0.1 m/s: refs/test_shapes without / with a 40 cm sideways, 45 deg initial error (the edge of the training's
# random initial errors), refs/test_dynamic_slow (the tight curves of test_dynamic at 0.1 m/s, inside the RL's training
# speeds), refs/test_corners_slow. One run at a time: the GPU is shared, and 3 in parallel ran out of memory (Isaac then
# still exits with 0, and a failed PhysX allocation even leaves the robots frozen: a run counts as failed by its log).
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
SHARP=logs/rsl_rl/mecanum_tracking_torque/2026-10-28_22-03-21_torque_sharp_seed0/model_276.pt
PLAIN=logs/rsl_rl/mecanum_tracking_torque/2026-10-28_20-51-05_torque_plain_seed0/model_282.pt
OFF="--start_offset 0 0.4 0.785398"
$PY scripts/tools/gen_dynamic_references.py --speed 0.1 --a_max 0.3 --shapes circle_small slalom rose \
    --out $R/test_dynamic_slow > /dev/null
ev() {
    name=$1; shift
    $PY -u scripts/rsl_rl/eval_tracking.py "$@" --headless > logs/runs/eval_$name.log 2>&1
    grep -qE "Traceback|PhysX error|out of memory" logs/runs/eval_$name.log && echo "FAILED $name"
}
run() {
    who=$1; shift
    ev ${who}_test_shapes "$@" --out logs/eval/$who/test_shapes --traj_dir $R/test_shapes
    ev ${who}_test_shapes_y40_psi45 "$@" --out logs/eval/$who/test_shapes_y40_psi45 --traj_dir $R/test_shapes $OFF
    ev ${who}_test_dynamic_slow "$@" --out logs/eval/$who/test_dynamic_slow --traj_dir $R/test_dynamic_slow
    ev ${who}_test_corners_slow "$@" --out logs/eval/$who/test_corners_slow --traj_dir $R/test_corners_slow
}
run rl_sharp_seed0 --controller policy --checkpoint $SHARP
run rl_plain_seed0 --controller policy --checkpoint $PLAIN
run mpc_plain --controller mpc --mpc_Q 40000 40000 3283 2500 2500 100 --mpc_state world
echo DONE
