
### test_shapes_y40_psi45 (40 cm oldalra + 45° kezdeti hiba)

| pálya | irányító | beállás [s] | utána pozíció RMS / max [mm] | parancs \|u\| max [Nm] | kerék max [rad/s] | lépések > 8.06 rad/s | robot v max [m/s] | elveszett |
|---|---|---|---|---|---|---|---|---|
| circle | RL, éles pályákon is (model_276) | 1.75 | 2.02 / 7.6 | 0.439 | 24.1 | 33 | 0.347 | 0 |
| circle | RL, csak sima pályákon (model_282) | 1.90 | 3.59 / 7.2 | 0.181 | 19.8 | 32 | 0.305 | 0 |
| circle | LTV-MPC (S és puha ω nélkül) | 2.05 | 4.11 / 8.7 | 0.540 | 28.4 | 30 | 0.341 | 0 |
| figure8 | RL, éles pályákon is (model_276) | 1.75 | 2.10 / 7.9 | 0.439 | 25.7 | 33 | 0.357 | 0 |
| figure8 | RL, csak sima pályákon (model_282) | 1.85 | 3.53 / 9.2 | 0.181 | 18.2 | 33 | 0.317 | 0 |
| figure8 | LTV-MPC (S és puha ω nélkül) | 2.05 | 4.17 / 8.6 | 0.540 | 28.4 | 31 | 0.348 | 0 |
| rounded_square | RL, éles pályákon is (model_276) | 1.75 | 1.90 / 9.5 | 0.439 | 23.9 | 33 | 0.343 | 0 |
| rounded_square | RL, csak sima pályákon (model_282) | 1.90 | 3.59 / 8.2 | 0.181 | 18.0 | 33 | 0.304 | 0 |
| rounded_square | LTV-MPC (S és puha ω nélkül) | 2.05 | 4.16 / 8.8 | 0.540 | 28.3 | 31 | 0.340 | 0 |
| sharp_square | RL, éles pályákon is (model_276) | 1.75 | 1.99 / 8.4 | 0.439 | 25.1 | 32 | 0.349 | 0 |
| sharp_square | RL, csak sima pályákon (model_282) | 1.85 | 3.75 / 15.6 | 0.181 | 18.7 | 33 | 0.302 | 0 |
| sharp_square | LTV-MPC (S és puha ω nélkül) | 2.05 | 4.14 / 8.9 | 0.540 | 28.1 | 32 | 0.327 | 0 |

Beállás: az első pillanat, ahonnan a pozícióhiba < 1 cm és az irányhiba < 2° marad legalább 2 s-ig; utána: a pozícióhiba onnantól a mozgás végéig. A kezdeti hiba 400 mm és 45°. Kerék / lépések: a teljes futás (a motor üresjárati fordulatszáma 8.06 rad/s, a robot csúcssebessége ezzel 0.244 m/s; a tapadási határ kerekenként ~0.14 Nm).

### test_shapes (kezdeti hiba nélkül)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|
| circle | RL, éles pályákon is (model_276) | 1.91 / 3.3 | 1.27 / 2.9 | -1.07 | – | 0.32 / 0.78 | 0.0076 / 0.0199 | 0.0185 | 0 |
| circle | RL, csak sima pályákon (model_282) | 3.48 / 6.0 | 1.66 / 3.7 | -2.40 | – | 0.47 / 1.00 | 0.0114 / 0.0274 | 0.0214 | 0 |
| circle | LTV-MPC (S és puha ω nélkül) | 4.11 / 5.8 | 0.97 / 2.5 | +3.80 | – | 0.11 / 0.43 | 0.0051 / 0.0118 | 0.0103 | 0 |
| figure8 | RL, éles pályákon is (model_276) | 1.96 / 4.4 | 1.30 / 4.4 | -1.18 | – | 0.35 / 0.75 | 0.0077 / 0.0197 | 0.0180 | 0 |
| figure8 | RL, csak sima pályákon (model_282) | 3.38 / 6.7 | 1.66 / 3.7 | -2.24 | – | 0.47 / 0.94 | 0.0116 / 0.0274 | 0.0173 | 0 |
| figure8 | LTV-MPC (S és puha ω nélkül) | 4.24 / 5.9 | 0.94 / 2.7 | +3.94 | – | 0.13 / 0.75 | 0.0052 / 0.0125 | 0.0106 | 0 |
| rounded_square | RL, éles pályákon is (model_276) | 1.71 / 3.5 | 0.97 / 2.8 | -1.06 | – | 0.33 / 0.73 | 0.0078 / 0.0211 | 0.0180 | 0 |
| rounded_square | RL, csak sima pályákon (model_282) | 3.41 / 6.5 | 1.91 / 3.2 | -2.27 | – | 0.47 / 0.92 | 0.0115 / 0.0299 | 0.0238 | 0 |
| rounded_square | LTV-MPC (S és puha ω nélkül) | 4.23 / 5.9 | 0.59 / 2.7 | +3.94 | – | 0.09 / 0.44 | 0.0052 / 0.0123 | 0.0112 | 0 |
| sharp_square | RL, éles pályákon is (model_276) | 1.81 / 5.7 | 1.00 / 4.9 | -1.11 | 5.7 | 0.34 / 0.88 | 0.0078 / 0.0256 | 0.0357 | 0 |
| sharp_square | RL, csak sima pályákon (model_282) | 3.56 / 14.8 | 2.14 / 13.1 | -2.22 | 14.8 | 0.47 / 0.84 | 0.0115 / 0.0337 | 0.0279 | 0 |
| sharp_square | LTV-MPC (S és puha ω nélkül) | 4.27 / 6.4 | 0.55 / 6.4 | +3.93 | 6.4 | 0.07 / 0.26 | 0.0058 / 0.0652 | 0.0976 | 0 |

### test_dynamic_slow (nagy görbület, 0.1 m/s)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|
| circle_small | RL, éles pályákon is (model_276) | 1.86 / 3.4 | 1.07 / 2.8 | -1.27 | – | 0.31 / 0.68 | 0.0079 / 0.0198 | 0.0171 | 0 |
| circle_small | RL, csak sima pályákon (model_282) | 4.14 / 7.1 | 2.17 / 3.9 | -2.87 | – | 0.48 / 0.89 | 0.0119 / 0.0285 | 0.0208 | 0 |
| circle_small | LTV-MPC (S és puha ω nélkül) | 4.13 / 5.6 | 1.23 / 2.7 | +3.79 | – | 0.08 / 0.26 | 0.0054 / 0.0125 | 0.0108 | 0 |
| rose | RL, éles pályákon is (model_276) | 2.01 / 3.7 | 1.12 / 2.6 | -1.37 | – | 0.33 / 0.78 | 0.0080 / 0.0216 | 0.0176 | 0 |
| rose | RL, csak sima pályákon (model_282) | 4.31 / 10.5 | 2.60 / 9.5 | -2.69 | – | 0.47 / 0.89 | 0.0119 / 0.0326 | 0.0309 | 0 |
| rose | LTV-MPC (S és puha ω nélkül) | 4.04 / 5.6 | 1.35 / 3.6 | +3.63 | – | 0.09 / 0.36 | 0.0055 / 0.0158 | 0.0125 | 0 |
| slalom | RL, éles pályákon is (model_276) | 2.13 / 3.5 | 1.07 / 2.7 | -1.46 | – | 0.36 / 0.71 | 0.0067 / 0.0154 | 0.0139 | 0 |
| slalom | RL, csak sima pályákon (model_282) | 2.91 / 5.7 | 2.28 / 5.5 | -1.49 | – | 0.23 / 0.65 | 0.0115 / 0.0271 | 0.0139 | 0 |
| slalom | LTV-MPC (S és puha ω nélkül) | 4.04 / 5.3 | 1.65 / 3.2 | +3.55 | – | 0.20 / 0.74 | 0.0055 / 0.0151 | 0.0103 | 0 |

### test_corners_slow (éles sarkok, 0.1 m/s)

| pálya | irányító | pozíció RMS / max [mm] | keresztirányú RMS / max [mm] | lag átlag [mm] | sarok utáni max [mm] | irány RMS / max [°] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | elveszett |
|---|---|---|---|---|---|---|---|---|---|
| sharp_square | RL, éles pályákon is (model_276) | 1.92 / 5.4 | 1.06 / 4.2 | -1.14 | 4.7 | 0.34 / 0.88 | 0.0080 / 0.0284 | 0.0458 | 0 |
| sharp_square | RL, csak sima pályákon (model_282) | 3.99 / 14.5 | 2.48 / 14.2 | -2.31 | 14.5 | 0.47 / 0.94 | 0.0116 / 0.0372 | 0.0330 | 0 |
| sharp_square | LTV-MPC (S és puha ω nélkül) | 4.18 / 5.9 | 0.62 / 4.6 | +3.85 | 5.9 | 0.07 / 0.25 | 0.0067 / 0.0646 | 0.0969 | 0 |
| smooth | RL, éles pályákon is (model_276) | 1.93 / 2.8 | 1.22 / 2.2 | -1.37 | – | 0.24 / 0.49 | 0.0056 / 0.0142 | 0.0167 | 0 |
| smooth | RL, csak sima pályákon (model_282) | 1.89 / 6.0 | 1.47 / 3.0 | -0.77 | – | 0.10 / 0.35 | 0.0069 / 0.0204 | 0.0156 | 0 |
| smooth | LTV-MPC (S és puha ω nélkül) | 3.24 / 3.8 | 0.96 / 1.6 | +3.05 | – | 0.07 / 0.23 | 0.0042 / 0.0107 | 0.0118 | 0 |
| straight_turn | RL, éles pályákon is (model_276) | 2.23 / 7.3 | 1.91 / 4.0 | -0.90 | 7.3 | 0.19 / 0.61 | 0.0061 / 0.0254 | 0.0199 | 0 |
| straight_turn | RL, csak sima pályákon (model_282) | 3.64 / 14.5 | 1.47 / 7.2 | -1.78 | 14.5 | 0.30 / 0.65 | 0.0061 / 0.0201 | 0.0182 | 0 |
| straight_turn | LTV-MPC (S és puha ω nélkül) | 3.15 / 4.3 | 0.59 / 1.7 | +3.00 | 3.9 | 0.04 / 0.16 | 0.0053 / 0.0780 | 0.1194 | 0 |

Ablak: a referencia mozgásának végéig (metrics.py). Sarok utáni max: a pozícióhiba legnagyobb értéke a > 30°-os irányugrások utáni 1 s-ban.
