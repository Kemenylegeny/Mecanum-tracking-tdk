
### test_shapes_y40_psi45 (40 cm oldalra + 45° kezdeti hiba)

| pálya | irányító | beállás [s] | utána pozíció RMS / max [mm] | parancs \|u\| max [Nm] | kerék max [rad/s] | lépések > 8.06 rad/s | robot v max [m/s] | elveszett |
|---|---|---|---|---|---|---|---|---|
| circle | LTV-MPC, világ-sebesség | 2.05 | 4.11 / 8.7 | 0.540 | 28.4 | 30 | 0.341 | 0 |
| circle | LTV-MPC, kerék (enkóder) | 2.20 | 4.71 / 8.9 | 0.540 | 26.8 | 35 | 0.319 | 0 |
| circle | LTV-MPC, kerék (alapsebességből) | 2.20 | 4.06 / 8.7 | 0.540 | 26.8 | 35 | 0.321 | 0 |
| figure8 | LTV-MPC, világ-sebesség | 2.05 | 4.17 / 8.6 | 0.540 | 28.4 | 31 | 0.348 | 0 |
| figure8 | LTV-MPC, kerék (enkóder) | 2.20 | 4.84 / 8.1 | 0.540 | 26.9 | 35 | 0.331 | 0 |
| figure8 | LTV-MPC, kerék (alapsebességből) | 2.20 | 4.14 / 9.2 | 0.540 | 26.9 | 35 | 0.315 | 0 |
| rounded_square | LTV-MPC, világ-sebesség | 2.05 | 4.16 / 8.8 | 0.540 | 28.3 | 31 | 0.340 | 0 |
| rounded_square | LTV-MPC, kerék (enkóder) | 2.25 | 4.84 / 8.9 | 0.540 | 27.9 | 36 | 0.309 | 0 |
| rounded_square | LTV-MPC, kerék (alapsebességből) | 2.20 | 4.18 / 9.6 | 0.540 | 27.9 | 35 | 0.337 | 0 |
| sharp_square | LTV-MPC, világ-sebesség | 2.05 | 4.14 / 8.9 | 0.540 | 28.1 | 32 | 0.327 | 0 |
| sharp_square | LTV-MPC, kerék (enkóder) | 2.20 | 4.82 / 8.4 | 0.540 | 28.3 | 34 | 0.345 | 0 |
| sharp_square | LTV-MPC, kerék (alapsebességből) | 2.20 | 4.17 / 8.2 | 0.540 | 28.3 | 35 | 0.340 | 0 |

Beállás: az első pillanat, ahonnan a pozícióhiba < 1 cm és az irányhiba < 2° marad legalább 2 s-ig; utána: a pozícióhiba onnantól a mozgás végéig. A kezdeti hiba 400 mm és 45°. Kerék / lépések: a teljes futás (a motor üresjárati fordulatszáma 8.06 rad/s, a robot csúcssebessége ezzel 0.244 m/s; a tapadási határ kerekenként ~0.14 Nm).

### test_shapes (kezdeti hiba nélkül)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|
| circle | LTV-MPC, világ-sebesség | 4.11 / 5.8 | 0.97 / 2.5 | +3.80 | – | 0.11 / 0.43 | 0.0051 / 0.0118 | 0.0103 | 0 |
| circle | LTV-MPC, kerék (enkóder) | 4.73 / 7.1 | 1.31 / 3.8 | +4.09 | – | 0.17 / 0.71 | 0.0053 / 0.0171 | 0.0260 | 0 |
| circle | LTV-MPC, kerék (alapsebességből) | 4.10 / 5.7 | 0.96 / 2.5 | +3.78 | – | 0.10 / 0.38 | 0.0051 / 0.0115 | 0.0101 | 0 |
| figure8 | LTV-MPC, világ-sebesség | 4.24 / 5.9 | 0.94 / 2.7 | +3.94 | – | 0.13 / 0.75 | 0.0052 / 0.0125 | 0.0106 | 0 |
| figure8 | LTV-MPC, kerék (enkóder) | 4.83 / 7.2 | 1.23 / 3.9 | +4.13 | – | 0.14 / 0.63 | 0.0054 / 0.0213 | 0.0318 | 0 |
| figure8 | LTV-MPC, kerék (alapsebességből) | 4.26 / 5.8 | 0.96 / 3.0 | +3.95 | – | 0.13 / 0.72 | 0.0052 / 0.0126 | 0.0103 | 0 |
| rounded_square | LTV-MPC, világ-sebesség | 4.23 / 5.9 | 0.59 / 2.7 | +3.94 | – | 0.09 / 0.44 | 0.0052 / 0.0123 | 0.0112 | 0 |
| rounded_square | LTV-MPC, kerék (enkóder) | 4.87 / 7.7 | 0.77 / 4.2 | +4.08 | – | 0.12 / 0.67 | 0.0055 / 0.0155 | 0.0246 | 0 |
| rounded_square | LTV-MPC, kerék (alapsebességből) | 4.25 / 5.8 | 0.56 / 2.7 | +3.96 | – | 0.08 / 0.39 | 0.0053 / 0.0122 | 0.0103 | 0 |
| sharp_square | LTV-MPC, világ-sebesség | 4.27 / 6.4 | 0.55 / 6.4 | +3.93 | 6.4 | 0.07 / 0.26 | 0.0058 / 0.0652 | 0.0976 | 0 |
| sharp_square | LTV-MPC, kerék (enkóder) | 4.84 / 7.7 | 0.72 / 7.7 | +3.91 | 7.7 | 0.08 / 0.23 | 0.0061 / 0.0642 | 0.1019 | 0 |
| sharp_square | LTV-MPC, kerék (alapsebességből) | 4.30 / 6.4 | 0.53 / 6.4 | +3.95 | 6.4 | 0.07 / 0.19 | 0.0058 / 0.0648 | 0.0966 | 0 |

### test_dynamic_slow (nagy görbület, 0.1 m/s)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|
| circle_small | LTV-MPC, világ-sebesség | 4.13 / 5.6 | 1.23 / 2.7 | +3.79 | – | 0.08 / 0.26 | 0.0054 / 0.0125 | 0.0108 | 0 |
| circle_small | LTV-MPC, kerék (enkóder) | 4.74 / 7.1 | 1.57 / 4.0 | +4.09 | – | 0.11 / 0.39 | 0.0057 / 0.0160 | 0.0290 | 0 |
| circle_small | LTV-MPC, kerék (alapsebességből) | 4.10 / 5.5 | 1.21 / 2.6 | +3.77 | – | 0.08 / 0.36 | 0.0053 / 0.0124 | 0.0105 | 0 |
| rose | LTV-MPC, világ-sebesség | 4.04 / 5.6 | 1.35 / 3.6 | +3.63 | – | 0.09 / 0.36 | 0.0055 / 0.0158 | 0.0125 | 0 |
| rose | LTV-MPC, kerék (enkóder) | 4.68 / 7.0 | 1.78 / 4.7 | +3.91 | – | 0.18 / 0.82 | 0.0057 / 0.0161 | 0.0187 | 0 |
| rose | LTV-MPC, kerék (alapsebességből) | 4.04 / 5.6 | 1.32 / 3.6 | +3.65 | – | 0.11 / 0.50 | 0.0055 / 0.0153 | 0.0123 | 0 |
| slalom | LTV-MPC, világ-sebesség | 4.04 / 5.3 | 1.65 / 3.2 | +3.55 | – | 0.20 / 0.74 | 0.0055 / 0.0151 | 0.0103 | 0 |
| slalom | LTV-MPC, kerék (enkóder) | 4.74 / 6.7 | 2.18 / 4.8 | +3.94 | – | 0.21 / 0.65 | 0.0058 / 0.0154 | 0.0194 | 0 |
| slalom | LTV-MPC, kerék (alapsebességből) | 4.02 / 5.4 | 1.60 / 3.1 | +3.56 | – | 0.18 / 0.76 | 0.0055 / 0.0151 | 0.0115 | 0 |

### test_corners_slow (éles sarkok, 0.1 m/s)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|
| sharp_square | LTV-MPC, világ-sebesség | 4.18 / 5.9 | 0.62 / 4.6 | +3.85 | 5.9 | 0.07 / 0.25 | 0.0067 / 0.0646 | 0.0969 | 0 |
| sharp_square | LTV-MPC, kerék (enkóder) | 4.84 / 7.7 | 1.08 / 6.8 | +3.83 | 7.5 | 0.10 / 0.71 | 0.0073 / 0.0672 | 0.1119 | 0 |
| sharp_square | LTV-MPC, kerék (alapsebességből) | 4.19 / 6.0 | 0.69 / 5.2 | +3.84 | 6.0 | 0.05 / 0.15 | 0.0067 / 0.0642 | 0.0961 | 0 |
| smooth | LTV-MPC, világ-sebesség | 3.24 / 3.8 | 0.96 / 1.6 | +3.05 | – | 0.07 / 0.23 | 0.0042 / 0.0107 | 0.0118 | 0 |
| smooth | LTV-MPC, kerék (enkóder) | 2.96 / 4.2 | 1.06 / 2.3 | +2.63 | – | 0.09 / 0.37 | 0.0045 / 0.0131 | 0.0173 | 0 |
| smooth | LTV-MPC, kerék (alapsebességből) | 3.22 / 3.8 | 0.94 / 1.5 | +3.05 | – | 0.07 / 0.31 | 0.0041 / 0.0107 | 0.0112 | 0 |
| straight_turn | LTV-MPC, világ-sebesség | 3.15 / 4.3 | 0.59 / 1.7 | +3.00 | 3.9 | 0.04 / 0.16 | 0.0053 / 0.0780 | 0.1194 | 0 |
| straight_turn | LTV-MPC, kerék (enkóder) | 2.88 / 5.6 | 0.99 / 2.9 | +2.22 | 4.3 | 0.21 / 0.82 | 0.0057 / 0.0789 | 0.1228 | 0 |
| straight_turn | LTV-MPC, kerék (alapsebességből) | 3.18 / 4.3 | 0.59 / 1.7 | +3.02 | 3.9 | 0.05 / 0.13 | 0.0053 / 0.0782 | 0.1198 | 0 |

### test_dynamic (nagy görbület és sarkok, 0.9 · v_max)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|
| circle_small | LTV-MPC, világ-sebesség | 6.69 / 9.4 | 2.80 / 5.2 | +5.81 | – | 0.11 / 0.32 | 0.0098 / 0.0338 | 0.0398 | 0 |
| circle_small | LTV-MPC, kerék (enkóder) | 7.41 / 11.5 | 3.26 / 5.9 | +6.07 | – | 0.17 / 0.55 | 0.0106 / 0.0429 | 0.0472 | 0 |
| circle_small | LTV-MPC, kerék (alapsebességből) | 6.71 / 9.4 | 2.77 / 5.1 | +5.85 | – | 0.14 / 0.42 | 0.0098 / 0.0338 | 0.0405 | 0 |
| rose | LTV-MPC, világ-sebesség | 6.54 / 10.8 | 2.57 / 6.6 | +5.59 | – | 0.13 / 0.60 | 0.0106 / 0.0458 | 0.0588 | 0 |
| rose | LTV-MPC, kerék (enkóder) | 7.10 / 13.1 | 3.18 / 8.3 | +5.48 | – | 0.17 / 0.70 | 0.0117 / 0.0636 | 0.0795 | 0 |
| rose | LTV-MPC, kerék (alapsebességből) | 6.55 / 10.8 | 2.54 / 6.6 | +5.62 | – | 0.14 / 0.45 | 0.0107 / 0.0461 | 0.0640 | 0 |
| sharp_square | LTV-MPC, világ-sebesség | 8.69 / 12.8 | 2.12 / 11.3 | +7.65 | 12.8 | 0.18 / 0.77 | 0.0160 / 0.1375 | 0.2141 | 0 |
| sharp_square | LTV-MPC, kerék (enkóder) | 10.34 / 16.1 | 2.34 / 14.4 | +8.20 | 15.9 | 0.31 / 1.12 | 0.0174 / 0.1384 | 0.2516 | 0 |
| sharp_square | LTV-MPC, kerék (alapsebességből) | 8.73 / 12.6 | 2.11 / 11.5 | +7.71 | 12.6 | 0.19 / 0.94 | 0.0159 / 0.1385 | 0.2058 | 0 |
| slalom | LTV-MPC, világ-sebesség | 5.84 / 7.6 | 2.69 / 5.7 | +4.91 | – | 0.16 / 0.47 | 0.0099 / 0.0452 | 0.0501 | 0 |
| slalom | LTV-MPC, kerék (enkóder) | 6.30 / 8.9 | 3.36 / 6.8 | +4.69 | – | 0.20 / 0.61 | 0.0109 / 0.0547 | 0.0777 | 0 |
| slalom | LTV-MPC, kerék (alapsebességből) | 5.87 / 7.5 | 2.67 / 5.8 | +4.96 | – | 0.19 / 0.59 | 0.0098 / 0.0452 | 0.0495 | 0 |
| stop_go | LTV-MPC, világ-sebesség | 5.01 / 8.2 | 0.14 / 0.3 | +4.57 | – | 0.06 / 0.21 | 0.0096 / 0.0732 | 0.1438 | 0 |
| stop_go | LTV-MPC, kerék (enkóder) | 2.54 / 6.4 | 0.34 / 1.3 | +2.10 | – | 0.08 / 0.27 | 0.0112 / 0.0777 | 0.1486 | 0 |
| stop_go | LTV-MPC, kerék (alapsebességből) | 5.00 / 8.0 | 0.12 / 0.3 | +4.56 | – | 0.08 / 0.33 | 0.0099 / 0.0735 | 0.1442 | 0 |

Ablak: a referencia mozgásának végéig (metrics.py). Sarok utáni max: a pozícióhiba legnagyobb értéke a > 30°-os irányugrások utáni 1 s-ban.
