# Offset-trained RL (torque_offset_seed0, model_311.pt) vs. old RL (torque_seed0, model_600.pt) vs. MPC, mm unless noted

Run: logs/runs/eval_offset_policy.sh (+ logs/runs/eval_test_shapes.sh). lag > 0: behind; eltérés = contour (normal to the reference velocity). test_dynamic is above the training speeds of both RL policies (curriculum top 1.2 x 0.1 m/s).

### test_shapes

| alakzat | szabályozó | pozíció RMS / max | lag átlag / RMS / max | eltérés RMS / max | nyomaték RMS [Nm] | ‖Δu‖ max | elveszett |
|---|---|---|---|---|---|---|---|
| circle | RL régi | 5.95 / 8.6 | -2.35 / 4.46 / 8.2 | 3.95 / 7.4 | 0.0109 | 0.0035 | 0 |
| circle | RL offset | 2.18 / 6.2 | -0.65 / 1.60 / 5.9 | 1.49 / 4.7 | 0.0099 | 0.0055 | 0 |
| circle | MPC | 4.23 / 5.9 | +3.90 / 4.12 / 5.9 | 0.98 / 2.6 | 0.0051 | 0.0101 | 0 |
| figure8 | RL régi | 5.82 / 9.2 | -2.70 / 4.55 / 8.4 | 3.63 / 8.4 | 0.0108 | 0.0036 | 0 |
| figure8 | RL offset | 2.16 / 6.1 | -0.68 / 1.55 / 6.1 | 1.51 / 4.2 | 0.0096 | 0.0053 | 0 |
| figure8 | MPC | 4.33 / 6.0 | +4.03 / 4.23 / 6.0 | 0.95 / 3.2 | 0.0051 | 0.0092 | 0 |
| rounded_square | RL régi | 5.73 / 8.6 | -2.36 / 3.97 / 7.9 | 4.13 / 7.1 | 0.0110 | 0.0039 | 0 |
| rounded_square | RL offset | 1.90 / 5.7 | -0.39 / 1.35 / 5.7 | 1.33 / 4.1 | 0.0100 | 0.0054 | 0 |
| rounded_square | MPC | 4.37 / 6.1 | +4.08 / 4.34 / 6.1 | 0.57 / 2.7 | 0.0052 | 0.0097 | 0 |
| sharp_square | RL régi | 5.78 / 17.6 | -2.30 / 3.98 / 14.9 | 4.20 / 15.1 | 0.0110 | 0.0065 | 0 |
| sharp_square | RL offset | 2.41 / 18.0 | -0.38 / 1.76 / 14.3 | 1.65 / 14.0 | 0.0100 | 0.0082 | 0 |
| sharp_square | MPC | 4.37 / 6.5 | +4.02 / 4.33 / 6.1 | 0.56 / 6.5 | 0.0057 | 0.0873 | 0 |

### test_shapes_offset (30 cm / 30°)

| alakzat | szabályozó | beállás [s] (< 1 cm, < 2°, 2 s-ig) | utána RMS / max [mm] | heading max [°] | ω max [rad/s] (> 8.06) | nyomaték max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|
| circle | RL régi | 1.70 | 6.07 / 9.5 | 29.8 | 11.2 (2.7%) | 0.069 | 0 |
| circle | RL offset | 1.95 | 2.25 / 9.6 | 29.8 | 10.5 (2.2%) | 0.086 | 0 |
| circle | MPC | 1.90 | 4.28 / 9.0 | 30.3 | 9.1 (1.0%) | 0.082 | 0 |
| figure8 | RL régi | 1.75 | 6.02 / 9.1 | 29.8 | 13.9 (2.0%) | 0.120 | 0 |
| figure8 | RL offset | 1.95 | 2.22 / 8.9 | 29.8 | 10.9 (1.6%) | 0.086 | 0 |
| figure8 | MPC | 1.90 | 4.31 / 8.6 | 30.2 | 9.6 (1.0%) | 0.082 | 0 |
| rounded_square | RL régi | 1.75 | 5.91 / 8.5 | 29.8 | 18.7 (2.3%) | 0.137 | 0 |
| rounded_square | RL offset | 1.95 | 2.03 / 9.2 | 29.8 | 10.7 (1.8%) | 0.086 | 0 |
| rounded_square | MPC | 1.90 | 4.37 / 8.8 | 30.3 | 10.7 (0.9%) | 0.082 | 0 |
| sharp_square | RL régi | 1.70 | 5.96 / 17.7 | 29.8 | 11.3 (2.1%) | 0.065 | 0 |
| sharp_square | RL offset | 1.95 | 2.46 / 17.7 | 29.8 | 10.6 (1.9%) | 0.086 | 0 |
| sharp_square | MPC | 1.90 | 4.32 / 8.1 | 30.3 | 11.5 (1.0%) | 0.082 | 0 |

### test_dynamic

| pálya | szabályozó | pozíció RMS / max | lag átlag | eltérés RMS / max | ω max (> 8.06) | nyomaték max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|
| circle_small | RL régi | 8.5 / 17.6 | -3.7 | 6.0 / 11.1 | 8.7 (8.1%) | 0.034 | 0 |
| circle_small | RL offset | 7.8 / 15.8 | -0.2 | 6.7 / 11.3 | 9.0 (4.5%) | 0.038 | 0 |
| circle_small | MPC | 8.6 / 11.3 | +7.4 | 3.8 / 7.0 | 8.1 (0.5%) | 0.034 | 0 |
| slalom | RL régi | 6.3 / 12.1 | -0.7 | 5.9 / 10.5 | 9.0 (6.3%) | 0.033 | 0 |
| slalom | RL offset | 6.8 / 15.3 | +1.7 | 5.0 / 11.8 | 9.0 (10.0%) | 0.033 | 0 |
| slalom | MPC | 7.2 / 10.9 | +5.9 | 3.5 / 8.8 | 8.3 (1.4%) | 0.044 | 0 |
| rose | RL régi | 8.7 / 22.7 | -3.4 | 6.8 / 22.3 | 9.3 (5.9%) | 0.044 | 0 |
| rose | RL offset | 9.2 / 26.6 | -0.1 | 7.8 / 25.3 | 8.9 (5.5%) | 0.036 | 0 |
| rose | MPC | 9.2 / 16.3 | +7.4 | 3.9 / 13.1 | 8.2 (0.5%) | 0.045 | 0 |
| sharp_square | RL régi | 9.3 / 28.0 | -1.7 | 7.0 / 24.1 | 9.2 (18.9%) | 0.040 | 0 |
| sharp_square | RL offset | 11.2 / 36.0 | +4.4 | 7.1 / 29.3 | 9.1 (18.5%) | 0.044 | 0 |
| sharp_square | MPC | 22.3 / 40.9 | +16.5 | 11.2 / 38.0 | 13.1 (3.6%) | 0.128 | 0 |
| stop_go | RL régi | 8.0 / 27.4 | -2.6 | 3.1 / 6.1 | 9.3 (10.6%) | 0.027 | 0 |
| stop_go | RL offset | 9.3 / 29.5 | +0.3 | 4.0 / 6.2 | 9.6 (6.4%) | 0.040 | 0 |
| stop_go | MPC | 5.2 / 8.1 | +4.7 | 0.1 / 0.3 | 8.9 (0.5%) | 0.068 | 0 |
