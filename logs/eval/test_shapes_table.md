# RL (torque_seed0, model_600.pt) vs. MPC on refs/test_shapes (0.1 m/s, fixed heading), mm unless noted

lag > 0: behind the reference point; contour: normal to the reference velocity; pályától: distance to the reference path (polyline). Run: logs/runs/eval_test_shapes.sh

| alakzat | szabályozó | pozíció RMS / max | lag átlag / RMS / max | contour RMS / max | pályától RMS / max | nyomaték RMS [Nm] | elveszett |
|---|---|---|---|---|---|---|---|
| circle | RL | 5.95 / 8.6 | -2.35 / 4.46 / 8.2 | 3.95 / 7.4 | 3.94 / 7.4 | 0.0109 | 0 |
| circle | MPC | 4.23 / 5.9 | +3.90 / 4.12 / 5.9 | 0.98 / 2.6 | 0.98 / 2.6 | 0.0051 | 0 |
| circle | MPC+Q_path | 1.00 / 1.9 | +0.78 / 0.83 / 1.4 | 0.56 / 1.6 | 0.56 / 1.6 | 0.0052 | 0 |
| figure8 | RL | 5.82 / 9.2 | -2.70 / 4.55 / 8.4 | 3.63 / 8.4 | 3.63 / 8.3 | 0.0108 | 0 |
| figure8 | MPC | 4.33 / 6.0 | +4.03 / 4.23 / 6.0 | 0.95 / 3.2 | 0.95 / 3.2 | 0.0051 | 0 |
| figure8 | MPC+Q_path | 1.00 / 2.1 | +0.81 / 0.86 / 1.4 | 0.52 / 1.8 | 0.52 / 1.8 | 0.0055 | 0 |
| rounded_square | RL | 5.73 / 8.6 | -2.36 / 3.97 / 7.9 | 4.13 / 7.1 | 4.13 / 7.1 | 0.0110 | 0 |
| rounded_square | MPC | 4.37 / 6.1 | +4.08 / 4.34 / 6.1 | 0.57 / 2.7 | 0.56 / 2.6 | 0.0052 | 0 |
| rounded_square | MPC+Q_path | 0.93 / 1.8 | +0.82 / 0.88 / 1.5 | 0.29 / 1.5 | 0.29 / 1.5 | 0.0055 | 0 |
| sharp_square | RL | 5.78 / 17.6 | -2.30 / 3.98 / 14.9 | 4.20 / 15.1 | 4.16 / 12.1 | 0.0110 | 0 |
| sharp_square | MPC | 4.37 / 6.5 | +4.02 / 4.33 / 6.1 | 0.56 / 6.5 | 0.49 / 5.5 | 0.0057 | 0 |
| sharp_square | MPC+Q_path | 0.92 / 2.7 | +0.84 / 0.90 / 1.4 | 0.19 / 2.7 | 0.16 / 1.7 | 0.0061 | 0 |
