
### test_shapes_y40_psi45 (40 cm oldalra + 45° kezdeti hiba)

| pálya | irányító | beállás [s] | utána pozíció RMS / max [mm] | parancs \|u\| max [Nm] | kerék max [rad/s] | lépések > 8.06 rad/s | robot v max [m/s] | elveszett |
|---|---|---|---|---|---|---|---|---|
| circle | LTV-MPC, jelenlegi súlyok | 2.20 | 4.06 / 8.7 | 0.540 | 26.8 | 35 | 0.321 | 0 |
| circle | LTV-MPC, BO-súlyok | 1.30 | 0.46 / 2.7 | 1.950 | 58.1 | 27 | 0.444 | 0 |
| figure8 | LTV-MPC, jelenlegi súlyok | 2.20 | 4.14 / 9.2 | 0.540 | 26.9 | 35 | 0.315 | 0 |
| figure8 | LTV-MPC, BO-súlyok | 1.35 | 0.47 / 3.7 | 1.950 | 57.9 | 27 | 0.478 | 0 |
| rounded_square | LTV-MPC, jelenlegi súlyok | 2.20 | 4.18 / 9.6 | 0.540 | 27.9 | 35 | 0.337 | 0 |
| rounded_square | LTV-MPC, BO-súlyok | 1.35 | 0.48 / 3.6 | 1.950 | 57.9 | 28 | 0.448 | 0 |
| sharp_square | LTV-MPC, jelenlegi súlyok | 2.20 | 4.17 / 8.2 | 0.540 | 28.3 | 35 | 0.340 | 0 |
| sharp_square | LTV-MPC, BO-súlyok | 1.40 | 0.48 / 2.5 | 1.950 | 57.9 | 28 | 0.398 | 0 |

Beállás: az első pillanat, ahonnan a pozícióhiba < 1 cm és az irányhiba < 2° marad legalább 2 s-ig; utána: a pozícióhiba onnantól a mozgás végéig. A kezdeti hiba 400 mm és 45°. Kerék / lépések: a teljes futás (a motor üresjárati fordulatszáma 8.06 rad/s, a robot csúcssebessége ezzel 0.244 m/s; a tapadási határ kerekenként ~0.14 Nm).

### test_shapes (kezdeti hiba nélkül)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | sebesség RMS / max [mm/s] | ψ̇ RMS / max [°/s] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|---|
| circle | LTV-MPC, jelenlegi súlyok | 4.10 / 5.7 | 0.96 / 2.5 | +3.78 | – | 0.10 / 0.38 | 4.1 / 10 | 1.58 / 5.2 | 0.0051 / 0.0115 | 0.0101 | 0 |
| circle | LTV-MPC, BO-súlyok | 0.46 / 0.9 | 0.14 / 0.5 | +0.41 | – | 0.05 / 0.17 | 6.2 / 18 | 4.05 / 16.3 | 0.0068 / 0.0243 | 0.0488 | 0 |
| figure8 | LTV-MPC, jelenlegi súlyok | 4.26 / 5.8 | 0.96 / 3.0 | +3.95 | – | 0.13 / 0.72 | 4.1 / 12 | 1.67 / 6.1 | 0.0052 / 0.0126 | 0.0103 | 0 |
| figure8 | LTV-MPC, BO-súlyok | 0.47 / 0.9 | 0.11 / 0.4 | +0.43 | – | 0.05 / 0.17 | 6.2 / 20 | 4.47 / 17.3 | 0.0070 / 0.0279 | 0.0512 | 0 |
| rounded_square | LTV-MPC, jelenlegi súlyok | 4.25 / 5.8 | 0.56 / 2.7 | +3.96 | – | 0.08 / 0.39 | 4.3 / 13 | 1.57 / 4.9 | 0.0053 / 0.0122 | 0.0103 | 0 |
| rounded_square | LTV-MPC, BO-súlyok | 0.48 / 0.9 | 0.08 / 0.4 | +0.44 | – | 0.05 / 0.14 | 6.5 / 19 | 7.36 / 20.0 | 0.0083 / 0.0254 | 0.0566 | 0 |
| sharp_square | LTV-MPC, jelenlegi súlyok | 4.30 / 6.4 | 0.53 / 6.4 | +3.95 | 6.4 | 0.07 / 0.19 | 4.5 / 28 | 1.71 / 5.0 | 0.0058 / 0.0648 | 0.0966 | 0 |
| sharp_square | LTV-MPC, BO-súlyok | 0.48 / 1.0 | 0.06 / 1.0 | +0.44 | 1.0 | 0.04 / 0.13 | 6.8 / 45 | 7.16 / 18.9 | 0.0086 / 0.0663 | 0.1242 | 0 |

### test_dynamic_slow (nagy görbület, 0.1 m/s)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | sebesség RMS / max [mm/s] | ψ̇ RMS / max [°/s] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|---|
| circle_small | LTV-MPC, jelenlegi súlyok | 4.10 / 5.5 | 1.21 / 2.6 | +3.77 | – | 0.08 / 0.36 | 5.3 / 14 | 1.30 / 5.0 | 0.0053 / 0.0124 | 0.0105 | 0 |
| circle_small | LTV-MPC, BO-súlyok | 0.47 / 0.9 | 0.13 / 0.4 | +0.43 | – | 0.04 / 0.14 | 6.6 / 18 | 3.61 / 12.5 | 0.0070 / 0.0223 | 0.0398 | 0 |
| rose | LTV-MPC, jelenlegi súlyok | 4.04 / 5.6 | 1.32 / 3.6 | +3.65 | – | 0.11 / 0.50 | 6.0 / 16 | 1.27 / 3.9 | 0.0055 / 0.0153 | 0.0123 | 0 |
| rose | LTV-MPC, BO-súlyok | 0.47 / 0.8 | 0.13 / 0.4 | +0.43 | – | 0.05 / 0.16 | 6.2 / 15 | 4.64 / 18.4 | 0.0073 / 0.0231 | 0.0534 | 0 |
| slalom | LTV-MPC, jelenlegi súlyok | 4.02 / 5.4 | 1.60 / 3.1 | +3.56 | – | 0.18 / 0.76 | 6.7 / 16 | 1.41 / 4.9 | 0.0055 / 0.0151 | 0.0115 | 0 |
| slalom | LTV-MPC, BO-súlyok | 0.48 / 0.8 | 0.16 / 0.4 | +0.44 | – | 0.05 / 0.17 | 6.3 / 17 | 2.38 / 10.9 | 0.0068 / 0.0218 | 0.0408 | 0 |

### test_corners_slow (éles sarkok, 0.1 m/s)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | sebesség RMS / max [mm/s] | ψ̇ RMS / max [°/s] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sharp_square | LTV-MPC, jelenlegi súlyok | 4.19 / 6.0 | 0.69 / 5.2 | +3.84 | 6.0 | 0.05 / 0.15 | 5.7 / 27 | 1.29 / 4.6 | 0.0067 / 0.0642 | 0.0961 | 0 |
| sharp_square | LTV-MPC, BO-súlyok | 0.49 / 0.8 | 0.06 / 0.3 | +0.45 | 0.8 | 0.05 / 0.16 | 6.8 / 24 | 9.27 / 20.7 | 0.0101 / 0.0755 | 0.0950 | 0 |
| smooth | LTV-MPC, jelenlegi súlyok | 3.22 / 3.8 | 0.94 / 1.5 | +3.05 | – | 0.07 / 0.31 | 3.8 / 11 | 1.52 / 5.5 | 0.0041 / 0.0107 | 0.0112 | 0 |
| smooth | LTV-MPC, BO-súlyok | 0.37 / 0.6 | 0.13 / 0.4 | +0.34 | – | 0.04 / 0.14 | 5.9 / 17 | 3.03 / 13.4 | 0.0057 / 0.0154 | 0.0374 | 0 |
| straight_turn | LTV-MPC, jelenlegi súlyok | 3.18 / 4.3 | 0.59 / 1.7 | +3.02 | 3.9 | 0.05 / 0.13 | 4.2 / 18 | 0.78 / 2.8 | 0.0053 / 0.0782 | 0.1198 | 0 |
| straight_turn | LTV-MPC, BO-súlyok | 0.36 / 0.6 | 0.06 / 0.2 | +0.34 | 0.5 | 0.03 / 0.12 | 7.1 / 20 | 6.27 / 17.3 | 0.0080 / 0.0893 | 0.1263 | 0 |

### test_dynamic (nagy görbület és sarkok, 0.9 · v_max)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | sebesség RMS / max [mm/s] | ψ̇ RMS / max [°/s] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|---|
| circle_small | LTV-MPC, jelenlegi súlyok | 6.71 / 9.4 | 2.77 / 5.1 | +5.85 | – | 0.14 / 0.42 | 9.6 / 22 | 1.28 / 3.8 | 0.0098 / 0.0338 | 0.0405 | 0 |
| circle_small | LTV-MPC, BO-súlyok | 0.81 / 1.4 | 0.20 / 0.4 | +0.75 | – | 0.05 / 0.13 | 9.4 / 23 | 4.16 / 10.2 | 0.0114 / 0.0449 | 0.0547 | 0 |
| rose | LTV-MPC, jelenlegi súlyok | 6.55 / 10.8 | 2.54 / 6.6 | +5.62 | – | 0.14 / 0.45 | 10.7 / 29 | 1.32 / 4.6 | 0.0107 / 0.0461 | 0.0640 | 0 |
| rose | LTV-MPC, BO-súlyok | 0.80 / 1.3 | 0.19 / 0.5 | +0.74 | – | 0.04 / 0.17 | 10.1 / 25 | 4.41 / 12.9 | 0.0119 / 0.0637 | 0.0904 | 0 |
| sharp_square | LTV-MPC, jelenlegi súlyok | 8.73 / 12.6 | 2.11 / 11.5 | +7.71 | 12.6 | 0.19 / 0.94 | 13.5 / 87 | 2.00 / 13.8 | 0.0159 / 0.1385 | 0.2058 | 0 |
| sharp_square | LTV-MPC, BO-súlyok | 1.15 / 3.0 | 0.25 / 1.7 | +1.03 | 3.0 | 0.11 / 0.88 | 17.6 / 145 | 5.22 / 20.3 | 0.0183 / 0.1550 | 0.2335 | 0 |
| slalom | LTV-MPC, jelenlegi súlyok | 5.87 / 7.5 | 2.67 / 5.8 | +4.96 | – | 0.19 / 0.59 | 11.7 / 27 | 1.38 / 6.3 | 0.0098 / 0.0452 | 0.0495 | 0 |
| slalom | LTV-MPC, BO-súlyok | 0.75 / 1.1 | 0.23 / 0.5 | +0.70 | – | 0.05 / 0.18 | 10.2 / 22 | 3.40 / 8.2 | 0.0117 / 0.0613 | 0.0839 | 0 |
| stop_go | LTV-MPC, jelenlegi súlyok | 5.00 / 8.0 | 0.12 / 0.3 | +4.56 | – | 0.08 / 0.33 | 5.5 / 34 | 0.91 / 8.4 | 0.0099 / 0.0735 | 0.1442 | 0 |
| stop_go | LTV-MPC, BO-súlyok | 0.61 / 1.6 | 0.06 / 0.4 | +0.57 | – | 0.03 / 0.23 | 10.2 / 41 | 3.48 / 9.5 | 0.0122 / 0.0790 | 0.1259 | 0 |

Ablak: a referencia mozgásának végéig (metrics.py). Sarok utáni max: a pozícióhiba legnagyobb értéke a > 30°-os irányugrások utáni 1 s-ban.
