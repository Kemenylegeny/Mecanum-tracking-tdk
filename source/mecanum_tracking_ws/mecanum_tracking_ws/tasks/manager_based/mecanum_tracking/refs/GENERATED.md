# Generated reference sets (not in git)

`train/`, `train_own/`, `train_own_sharp/`, `test_unseen/` and `test_unseen_own/` are too large for git (train_own alone
is 374 MB, train_own_sharp 102 MB). They are deterministic: these commands, run from the project root, regenerate them
bit for bit (verified 2026-10-26; train_own_sharp 2026-10-28):

```bash
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
python scripts/tools/gen_training_trajectories.py --out $R/train --n 200 --seed 0
python scripts/tools/gen_training_trajectories.py --out $R/test_unseen --n 20 --seed 999
python scripts/tools/gen_training_trajectories.py --own --out $R/train_own --n 200 --seed 0
python scripts/tools/gen_training_trajectories.py --own --kind sharp --prefix sharp --out $R/train_own_sharp --n 200 --seed 1
python scripts/tools/gen_training_trajectories.py --own --ramp --prefix unseen --out $R/test_unseen_own --n 20 --seed 999
(cd $R && sha256sum -c --quiet generated.sha256)   # no output = identical
```

The small sets (`test`, `test_fixed`, `test_tangent`, `test_shapes`, `test_dynamic`, `test_dynamic_slow`,
`test_corners_slow`, `test_corners_fast`) are in git; their generators are `tools/gen_reference_trajectories.py`,
`tools/export_mpc_references.py`, `tools/gen_shape_references.py`, `tools/gen_dynamic_references.py` and
`tools/gen_corner_references.py`.

`tune_mpc` (in git, 3.7 MB) is the tuning set of the MPC's Bayesian optimisation (`tools/tune_mpc_bo.py`, 2026-10-30):
paths of the training generator with their own seeds, open (one lap with speed ramps), not the test shapes:

```bash
G=scripts/tools/gen_training_trajectories.py
python $G --own --ramp --kind smooth --max_length 3 --prefix tune_smooth --n 3 --seed 31 --out $R/tune_mpc
python $G --own --ramp --kind polygon --prefix tune_polygon --n 3 --seed 32 --out $R/tune_mpc
python $G --own --ramp --kind tight --prefix tune_tight --n 3 --seed 33 --out $R/tune_mpc
```
