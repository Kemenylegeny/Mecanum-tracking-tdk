#!/bin/bash
# Initial-error video, RL (offset-trained, model_311.pt) vs. MPC on the circle of refs/test_shapes: raw frames + logs per
# scenario and controller (play_shapes.py --raw_dir, one process each, so every clip is the very run of
# eval_tracking.py), 8 s (160 steps) from the reset, camera on the start of the path; then the composed video.
# 2 processes at a time (GPU memory: the camera renders need ~3-4 GB each). WHO=rl / WHO=mpc records only that controller.
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs/test_shapes
CKPT=logs/rsl_rl/mecanum_tracking_torque/2026-10-26_13-39-49_torque_offset_seed0/model_311.pt
RAW=logs/videos/raw/offsets
SCEN="y10_psi0 y20_psi0 y30_psi0 y0_psi10 y0_psi30 y0_psi50 y10_psi10 y20_psi30 y30_psi50"
CAM="--max_steps 160 --cam_window 8.5 --width 960 --height 600 --cam_zoom 1.2 --cam_elev 60 --marker_scale 0.35 --line_radius 0.008 --extra_s 0"
jobs() {
    for name in $SCEN; do
        cm=$(echo $name | sed -E 's/y([0-9]+)_psi([0-9]+)/\1/'); deg=$(echo $name | sed -E 's/y([0-9]+)_psi([0-9]+)/\2/')
        off="0 $(python3 -c "print($cm / 100)") $(python3 -c "import math; print(math.radians($deg))")"
        [ "${WHO:-all}" != mpc ] && echo "rl $name --controller policy --checkpoint $CKPT --start_offset $off"
        [ "${WHO:-all}" != rl ] && echo "mpc $name --controller mpc --start_offset $off"
    done
}
mkdir -p $RAW
jobs | xargs -P 2 -L 1 bash -c '
    who=$1; name=$2; shift 2
    '"$PY"' -u scripts/rsl_rl/play_shapes.py --task Mecanum-Tracking-Torque-Eval-v0 "$@" --traj_dir '"$R"' --shapes circle \
        --raw_dir '"$RAW"'/$who/$name '"$CAM"' --headless --enable_cameras > '"$RAW"'/$who.$name.log 2>&1
    grep -q "\[shapes\] raw videos" '"$RAW"'/$who.$name.log || echo "FAILED $who $name"' _
$PY scripts/tools/compose_offset_video.py --rl_raw $RAW/rl --mpc_raw $RAW/mpc --rl_eval logs/eval/rl_offset_seed0/offsets \
    --mpc_eval logs/eval/mpc/offsets --scenarios $SCEN --out logs/videos/offsets_rl_vs_mpc.mp4
