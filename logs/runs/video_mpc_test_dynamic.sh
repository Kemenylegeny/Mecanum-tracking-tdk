#!/bin/bash
# Raw frames + logs of the MPC on refs/test_dynamic, one process per shape (each is then the very run of
# eval_tracking.py, bit for bit), then the 60 s analysis video (tools/compose_dynamic_video.py).
cd "$(dirname "$0")/../.."
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
OUT=logs/videos/raw/mpc_test_dynamic
for spec in circle_small:2.2 slalom:1.75 rose:2.4 sharp_square:2.3 stop_go:1.9; do
    shape=${spec%%:*}; zoom=${spec##*:}
    $PY -u scripts/rsl_rl/play_shapes.py --task Mecanum-Tracking-Torque-Eval-v0 --controller mpc \
        --traj_dir $R/test_dynamic --shapes $shape --raw_dir $OUT --extra_s 0.4 --width 1080 --height 1080 \
        --cam_zoom $zoom --cam_elev 65 --marker_scale 0.35 --line_radius 0.012 --headless --enable_cameras \
        > $OUT.$shape.log 2>&1 || exit 1
done
$PY scripts/tools/compose_dynamic_video.py --raw_dir $OUT --out logs/videos/mpc_test_dynamic_60s.mp4 --duration 60
