# Generated reference sets (not in git)

`train/`, `train_own/`, `test_unseen/` and `test_unseen_own/` are too large for git (train_own alone is 374 MB). They are
deterministic: these commands, run from the project root, regenerate them bit for bit (verified 2026-10-26):

```bash
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
python scripts/tools/gen_training_trajectories.py --out $R/train --n 200 --seed 0
python scripts/tools/gen_training_trajectories.py --out $R/test_unseen --n 20 --seed 999
python scripts/tools/gen_training_trajectories.py --own --out $R/train_own --n 200 --seed 0
python scripts/tools/gen_training_trajectories.py --own --ramp --prefix unseen --out $R/test_unseen_own --n 20 --seed 999
(cd $R && sha256sum -c --quiet generated.sha256)   # no output = identical
```

The small sets (`test`, `test_fixed`, `test_tangent`, `test_shapes`, `test_dynamic`) are in git; their generators are
`tools/gen_reference_trajectories.py`, `tools/export_mpc_references.py`, `tools/gen_shape_references.py` and
`tools/gen_dynamic_references.py`.
