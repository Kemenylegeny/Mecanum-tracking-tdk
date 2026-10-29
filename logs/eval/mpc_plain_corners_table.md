# MPC ablation: S and the soft wheel-speed limit on/off, on refs/test_corners_{slow,fast}, mm unless noted

Paths (scripts/tools/gen_corner_references.py, fixed heading): sharp square (0.6 m, 90 deg corners), 1.0 m straight +
sharp 135 deg turn + 0.5 m, smooth y = 0.15 (1 - cos(2 pi x / 1.5)) (radius >= 0.38 m); corners driven without
slowing down, |dv/dt| <= 0.3 m/s^2. All variants: DARE terminal weight, no Q_path, hard torque box |u| <= 1.95 Nm.
"S és puha ω nélkül (új alap)" = the plain LTV-MPC, the default since 2026-10-28; "S + puha ω (régi alap)" =
`--mpc_S 100 --mpc_wheel_limit`. lag > 0: behind the reference point; sarok utáni max: largest position error within
1 s after a corner. Run: logs/runs/eval_mpc_plain_corners.sh; table and figures
(logs/eval/mpc_plain/test_corners_{slow,fast}.png): scripts/tools/report_mpc_ablation.py

### test_corners_slow (0.1 m/s)

| pálya | MPC | pozíció RMS / max [mm] | lag átlag [mm] | contour RMS / max [mm] | sarok utáni max [mm] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | ω max [rad/s] (> 8.06) | QP / lépés [ms] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|
| sharp_square | S + puha ω (régi alap) | 4.37 / 6.4 | +4.00 | 0.73 / 5.5 | 6.4 | 0.0065 / 0.0594 | 0.0863 | 4.55 (0.0%) | 1.18 / 9.2 | 0 |
| sharp_square | S és puha ω nélkül (új alap) | 4.18 / 5.9 | +3.85 | 0.62 / 4.6 | 5.9 | 0.0067 / 0.0646 | 0.0969 | 4.41 (0.0%) | 0.08 / 7.0 | 0 |
| sharp_square | csak S nélkül | 4.18 / 5.9 | +3.85 | 0.62 / 4.6 | 5.9 | 0.0067 / 0.0646 | 0.0969 | 4.41 (0.0%) | 1.17 / 8.7 | 0 |
| sharp_square | csak puha ω nélkül | 4.37 / 6.4 | +4.00 | 0.73 / 5.5 | 6.4 | 0.0065 / 0.0594 | 0.0863 | 4.55 (0.0%) | 0.08 / 7.1 | 0 |
| straight_turn | S + puha ω (régi alap) | 3.24 / 4.3 | +3.08 | 0.61 / 1.7 | 3.9 | 0.0051 / 0.0723 | 0.1053 | 5.35 (0.0%) | 1.34 / 10.8 | 0 |
| straight_turn | S és puha ω nélkül (új alap) | 3.15 / 4.3 | +3.00 | 0.59 / 1.7 | 3.9 | 0.0053 / 0.0780 | 0.1194 | 5.50 (0.0%) | 0.07 / 8.4 | 0 |
| straight_turn | csak S nélkül | 3.15 / 4.3 | +3.00 | 0.59 / 1.7 | 3.9 | 0.0053 / 0.0780 | 0.1194 | 5.50 (0.0%) | 1.31 / 10.5 | 0 |
| straight_turn | csak puha ω nélkül | 3.24 / 4.3 | +3.08 | 0.61 / 1.7 | 3.9 | 0.0051 / 0.0723 | 0.1053 | 5.35 (0.0%) | 0.08 / 8.5 | 0 |
| smooth | S + puha ω (régi alap) | 3.33 / 4.0 | +3.14 | 0.98 / 1.5 | – | 0.0041 / 0.0106 | 0.0106 | 5.33 (0.0%) | 1.28 / 10.1 | 0 |
| smooth | S és puha ω nélkül (új alap) | 3.24 / 3.8 | +3.05 | 0.96 / 1.6 | – | 0.0042 / 0.0107 | 0.0118 | 5.52 (0.0%) | 0.07 / 8.0 | 0 |
| smooth | csak S nélkül | 3.24 / 3.8 | +3.05 | 0.96 / 1.6 | – | 0.0042 / 0.0107 | 0.0118 | 5.52 (0.0%) | 1.24 / 9.8 | 0 |
| smooth | csak puha ω nélkül | 3.33 / 4.0 | +3.14 | 0.98 / 1.5 | – | 0.0041 / 0.0106 | 0.0106 | 5.33 (0.0%) | 0.07 / 8.0 | 0 |

### test_corners_fast (0.9 · v_max (≤ 0.22 m/s))

| pálya | MPC | pozíció RMS / max [mm] | lag átlag [mm] | contour RMS / max [mm] | sarok utáni max [mm] | nyomaték RMS / max [Nm] | ‖Δu‖ max [Nm] | ω max [rad/s] (> 8.06) | QP / lépés [ms] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|
| sharp_square | S + puha ω (régi alap) | 21.97 / 41.0 | +15.95 | 10.89 / 36.1 | 41.0 | 0.0131 / 0.1284 | 0.1842 | 8.74 (4.3%) | 1.64 / 9.4 | 0 |
| sharp_square | S és puha ω nélkül (új alap) | 8.63 / 13.4 | +7.51 | 1.88 / 9.8 | 13.4 | 0.0159 / 0.1394 | 0.2136 | 10.26 (12.0%) | 0.08 / 7.1 | 0 |
| sharp_square | csak S nélkül | 21.76 / 40.1 | +15.97 | 10.88 / 35.6 | 40.1 | 0.0139 / 0.1394 | 0.2073 | 9.29 (2.1%) | 1.58 / 9.1 | 0 |
| sharp_square | csak puha ω nélkül | 8.88 / 13.8 | +7.68 | 2.01 / 11.1 | 13.8 | 0.0142 / 0.1287 | 0.1827 | 12.60 (9.8%) | 0.08 / 7.2 | 0 |
| straight_turn | S + puha ω (régi alap) | 6.33 / 11.0 | +5.70 | 1.24 / 8.3 | 8.4 | 0.0085 / 0.0689 | 0.1005 | 8.23 (0.6%) | 1.45 / 11.3 | 0 |
| straight_turn | S és puha ω nélkül (új alap) | 5.34 / 8.0 | +4.91 | 1.21 / 8.0 | 8.0 | 0.0086 / 0.0723 | 0.1103 | 8.15 (1.8%) | 0.08 / 8.7 | 0 |
| straight_turn | csak S nélkül | 6.33 / 10.9 | +5.69 | 1.20 / 8.0 | 8.0 | 0.0086 / 0.0723 | 0.1103 | 8.75 (1.2%) | 1.38 / 10.5 | 0 |
| straight_turn | csak puha ω nélkül | 5.44 / 8.4 | +5.00 | 1.26 / 8.3 | 8.4 | 0.0085 / 0.0689 | 0.1005 | 7.93 (0.0%) | 0.07 / 8.6 | 0 |
| smooth | S + puha ω (régi alap) | 7.59 / 10.7 | +6.79 | 2.75 / 5.5 | – | 0.0064 / 0.0140 | 0.0107 | 8.44 (0.3%) | 1.43 / 10.5 | 0 |
| smooth | S és puha ω nélkül (új alap) | 5.18 / 5.9 | +4.84 | 1.60 / 2.3 | – | 0.0064 / 0.0142 | 0.0116 | 8.11 (0.3%) | 0.07 / 8.2 | 0 |
| smooth | csak S nélkül | 7.31 / 10.2 | +6.56 | 2.61 / 5.2 | – | 0.0064 / 0.0142 | 0.0116 | 8.11 (0.3%) | 1.37 / 10.0 | 0 |
| smooth | csak puha ω nélkül | 5.33 / 5.9 | +4.98 | 1.65 / 2.4 | – | 0.0064 / 0.0140 | 0.0107 | 8.03 (0.0%) | 0.07 / 8.2 | 0 |

### S + puha ω vs. S és puha ω nélkül, lépésenként

| készlet | pálya | ‖p_régi − p_új‖ max [mm] | … a sarkoktól távol [mm] | max \|u_régi − u_új\| [Nm] | … a sarkoktól távol [Nm] | ‖Δu‖ max régi / új [Nm] |
|---|---|---|---|---|---|---|
| slow | sharp_square | 0.88 | 0.84 | 0.0077 | 0.0066 | 0.0863 / 0.0969 |
| slow | straight_turn | 0.18 | 0.18 | 0.0057 | 0.0009 | 0.1053 / 0.1194 |
| slow | smooth | 0.48 | 0.48 | 0.0048 | 0.0048 | 0.0106 / 0.0118 |
| fast | sharp_square | 27.60 | 21.47 | 0.0421 | 0.0079 | 0.1842 / 0.2136 |
| fast | straight_turn | 5.02 | 5.02 | 0.0059 | 0.0059 | 0.1005 / 0.1103 |
| fast | smooth | 5.63 | 5.63 | 0.0069 | 0.0069 | 0.0107 / 0.0116 |

A sarkoktól távol: a referencia mozgásának ablakából kivéve minden sarok −0.5 … +1.5 s környezetét (a sima pályán nincs sarok, ott a teljes ablak).

