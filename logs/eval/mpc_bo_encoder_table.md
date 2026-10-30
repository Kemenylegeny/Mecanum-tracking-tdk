
### test_shapes_y40_psi45 (40 cm oldalra + 45° kezdeti hiba)

| pálya | irányító | beállás [s] | utána pozíció RMS / max [mm] | parancs \|u\| max [Nm] | kerék max [rad/s] | lépések > 8.06 rad/s | robot v max [m/s] | elveszett |
|---|---|---|---|---|---|---|---|---|
| circle | jelenlegi súlyok, enkóder | 2.20 | 4.71 / 8.9 | 0.540 | 26.8 | 35 | 0.319 | 0 |
| circle | BO-súlyok, enkóder | 1.30 | 2.14 / 9.5 | 1.950 | 61.5 | 223 | 0.435 | 0 |
| figure8 | jelenlegi súlyok, enkóder | 2.20 | 4.84 / 8.1 | 0.540 | 26.9 | 35 | 0.331 | 0 |
| figure8 | BO-súlyok, enkóder | 1.40 | 2.42 / 14.1 | 1.950 | 57.9 | 324 | 0.393 | 0 |
| rounded_square | jelenlegi súlyok, enkóder | 2.25 | 4.84 / 8.9 | 0.540 | 27.9 | 36 | 0.309 | 0 |
| rounded_square | BO-súlyok, enkóder | 1.40 | 2.19 / 14.6 | 1.950 | 57.9 | 237 | 0.376 | 0 |
| sharp_square | jelenlegi súlyok, enkóder | 2.20 | 4.82 / 8.4 | 0.540 | 28.3 | 34 | 0.345 | 0 |
| sharp_square | BO-súlyok, enkóder | 1.30 | 1.99 / 10.7 | 1.950 | 57.9 | 214 | 0.498 | 0 |

Beállás: az első pillanat, ahonnan a pozícióhiba < 1 cm és az irányhiba < 2° marad legalább 2 s-ig; utána: a pozícióhiba onnantól a mozgás végéig. A kezdeti hiba 400 mm és 45°. Kerék / lépések: a teljes futás (a motor üresjárati fordulatszáma 8.06 rad/s, a robot csúcssebessége ezzel 0.244 m/s; a tapadási határ kerekenként ~0.14 Nm).

### test_shapes (kezdeti hiba nélkül)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | sebesség RMS / max [mm/s] | ψ̇ RMS / max [°/s] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|---|
| circle | jelenlegi súlyok, enkóder | 4.73 / 7.1 | 1.31 / 3.8 | +4.09 | – | 0.17 / 0.71 | 5.3 / 17 | 1.75 / 5.9 | 0.0053 / 0.0171 | 0.0260 | 0 |
| circle | BO-súlyok, enkóder | 2.17 / 8.8 | 1.67 / 8.6 | +0.10 | – | 0.38 / 1.93 | 72.3 / 191 | 12.85 / 51.6 | 0.0456 / 0.2857 | 0.6314 | 0 |
| figure8 | jelenlegi súlyok, enkóder | 4.83 / 7.2 | 1.23 / 3.9 | +4.13 | – | 0.14 / 0.63 | 5.5 / 19 | 1.71 / 6.4 | 0.0054 / 0.0213 | 0.0318 | 0 |
| figure8 | BO-súlyok, enkóder | 2.01 / 8.9 | 1.44 / 7.7 | +0.17 | – | 0.39 / 2.72 | 71.6 / 183 | 12.70 / 49.4 | 0.0447 / 0.3246 | 0.6944 | 0 |
| rounded_square | jelenlegi súlyok, enkóder | 4.87 / 7.7 | 0.77 / 4.2 | +4.08 | – | 0.12 / 0.67 | 5.8 / 23 | 2.02 / 5.9 | 0.0055 / 0.0155 | 0.0246 | 0 |
| rounded_square | BO-súlyok, enkóder | 1.84 / 7.3 | 1.25 / 6.2 | +0.22 | – | 0.35 / 2.84 | 65.2 / 184 | 11.76 / 48.4 | 0.0429 / 0.3140 | 0.8667 | 0 |
| sharp_square | jelenlegi súlyok, enkóder | 4.84 / 7.7 | 0.72 / 7.7 | +3.91 | 7.7 | 0.08 / 0.23 | 6.9 / 29 | 1.61 / 8.1 | 0.0061 / 0.0642 | 0.1019 | 0 |
| sharp_square | BO-súlyok, enkóder | 1.93 / 8.0 | 1.36 / 7.3 | +0.19 | 4.2 | 0.40 / 2.70 | 71.4 / 175 | 12.96 / 63.6 | 0.0455 / 0.2448 | 0.7076 | 0 |

### test_dynamic_slow (nagy görbület, 0.1 m/s)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | sebesség RMS / max [mm/s] | ψ̇ RMS / max [°/s] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|---|
| circle_small | jelenlegi súlyok, enkóder | 4.74 / 7.1 | 1.57 / 4.0 | +4.09 | – | 0.11 / 0.39 | 7.3 / 20 | 1.20 / 4.7 | 0.0057 / 0.0160 | 0.0290 | 0 |
| circle_small | BO-súlyok, enkóder | 2.26 / 9.3 | 1.64 / 5.4 | +0.02 | – | 0.48 / 3.66 | 74.9 / 213 | 12.33 / 71.0 | 0.0471 / 0.2324 | 0.6713 | 0 |
| rose | jelenlegi súlyok, enkóder | 4.68 / 7.0 | 1.78 / 4.7 | +3.91 | – | 0.18 / 0.82 | 7.4 / 17 | 1.53 / 4.3 | 0.0057 / 0.0161 | 0.0187 | 0 |
| rose | BO-súlyok, enkóder | 1.77 / 5.9 | 1.28 / 5.4 | +0.21 | – | 0.38 / 2.39 | 61.9 / 184 | 11.75 / 39.4 | 0.0392 / 0.2218 | 0.5605 | 0 |
| slalom | jelenlegi súlyok, enkóder | 4.74 / 6.7 | 2.18 / 4.8 | +3.94 | – | 0.21 / 0.65 | 8.3 / 21 | 1.56 / 4.7 | 0.0058 / 0.0154 | 0.0194 | 0 |
| slalom | BO-súlyok, enkóder | 2.03 / 7.9 | 1.42 / 4.8 | +0.21 | – | 0.34 / 1.68 | 70.6 / 158 | 11.68 / 54.0 | 0.0451 / 0.2543 | 0.9593 | 0 |

Ablak: a referencia mozgásának végéig (metrics.py). Sarok utáni max: a pozícióhiba legnagyobb értéke a > 30°-os irányugrások utáni 1 s-ban.
