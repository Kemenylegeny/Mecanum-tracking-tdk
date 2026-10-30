#!/bin/bash
# 30 s video: the RL trained on sharp corners and tight turns too (torque_sharp_seed0, model_276.pt, left) vs. the plain
# LTV-MPC (right) in three scenarios, 0.1 m/s, fixed heading: a 40 cm sideways / 45 deg initial error on the circle of
# refs/test_shapes, the rose of refs/test_dynamic_slow (curvature <= 36 1/m), the sharp square of refs/test_corners_slow.
# Raw frames + logs per scenario and controller (play_shapes.py --raw_dir, one shape per process, so every clip is the
# very run of eval_tracking.py in logs/runs/eval_sharp_policy.sh), one at a time (P=2: two; camera renders need ~3-4 GB GPU
# memory each, the GPU is shared, and a failed PhysX allocation leaves the robot frozen); then the composed video
# (tools/compose_scenario_video.py).
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
CKPT=logs/rsl_rl/mecanum_tracking_torque/2026-10-28_22-03-21_torque_sharp_seed0/model_276.pt
RAW=logs/videos/raw/sharp
LOOK="--width 960 --height 600 --cam_elev 60 --marker_scale 0.35 --line_radius 0.008"
jobs() {
    for who in rl mpc; do
        if [ $who = rl ]; then ctl="--controller policy --checkpoint $CKPT"; else ctl="--controller mpc --mpc_Q 40000 40000 3283 2500 2500 100 --mpc_state world"; fi
        echo "$who far circle $ctl --traj_dir $R/test_shapes --start_offset 0 0.4 0.785398 --max_steps 184 --cam_window 9 --cam_zoom 1.2 --extra_s 0"
        echo "$who rose rose $ctl --traj_dir $R/test_dynamic_slow --cam_zoom ${ROSE_ZOOM:-2.4} --extra_s 0.4"
        echo "$who square sharp_square $ctl --traj_dir $R/test_corners_slow --cam_zoom ${SQUARE_ZOOM:-1.9} --extra_s 0.4"
    done
}
mkdir -p $RAW
jobs | grep -E "^(${WHO:-rl|mpc}) (${SCEN:-far|rose|square}) " | xargs -P ${P:-1} -L 1 bash -c '
    who=$1; name=$2; shape=$3; shift 3
    '"$PY"' -u scripts/rsl_rl/play_shapes.py --task Mecanum-Tracking-Torque-Eval-v0 "$@" --shapes $shape \
        --raw_dir '"$RAW"'/$who/$name '"$LOOK"' --headless --enable_cameras > '"$RAW"'/$who.$name.log 2>&1
    grep -q "\[shapes\] raw videos" '"$RAW"'/$who.$name.log && ! grep -qE "PhysX error|out of memory" '"$RAW"'/$who.$name.log || echo "FAILED $who $name"' _
[ -n "$NO_COMPOSE" ] || $PY scripts/tools/compose_scenario_video.py --raw $RAW --eval_root logs/eval --rl_eval rl_sharp_seed0 \
    --mpc_eval mpc_plain --out logs/videos/sharp_rl_vs_mpc_30s.mp4 --duration 30
