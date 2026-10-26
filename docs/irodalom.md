# RL pályakövetés — irodalom (gyűjtve 2026-10-15)

Megjegyzés: olyan cikket nem találtunk, ami mecanum robotot PPO/SAC-kal követtet pályán ÉS MPC-vel hasonlít össze → ez a dolgozatod saját hozzájárulása lehet.
Jelölés: [ellenőrizve] = forrás megnyitva/ID ellenőrizve; [snippet] = csak keresési találat szövege alapján, a részleteket a cikkben kell megnézni.

## Olvasási sorrend (legfontosabb elöl)
1. Song, Romero, Müller, Koltun, Scaramuzza — "Reaching the limit in autonomous racing: Optimal control versus reinforcement learning", Science Robotics 8(82), 2023. arXiv 2310.10943 [ellenőrizve]. A kulcscikk RL vs. optimális szabályozás témában: az RL azért nyer, mert jobb célfüggvényt optimalizálhat, nem mert ugyanazt jobban.
2. Rudin et al., "Advanced Skills by Learning Locomotion and Local Navigation End-to-End", arXiv 2209.12827 — a te mostani alapod.
3. Han et al., "Wheeled Lab", CoRL 2025, arXiv 2502.07380 [ellenőrizve] — Isaac Lab kerekes robotokra, sim2real, domain randomization.
4. Alcayaga et al., "LSTM-Enhanced Deep RL for Robust Trajectory Tracking Control of Skid-Steer Mobile Robots Under Terra-Mechanical Constraints", Robotics 14(6):74, 2025 [snippet] — PPO/DDPG/TD3/SAC ± LSTM, pályakövetés csúszással (a görgős kerékhez a legközelebbi minta).
5. "MPC-TD3 Trajectory Tracking Control for Electrically Driven Unmanned Tracked Vehicles", Electronics 2024, doi 10.3390/electronics13183747 [snippet] — reziduális RL MPC-n; MPC ~45 ms vs. TD3 ~0,8 ms.
6. "Analysis of Model-Free RL Control Schemes on self-balancing Wheeled Extendible System", arXiv 2111.08389 [snippet] — közvetlen DDPG/PPO vs. MPC összehasonlítás.
7. Kaufmann et al., "Champion-level drone racing using deep reinforcement learning", Nature 620, 2023 [emlékezetből, nincs újraellenőrizve].
8. "Path Following for Autonomous Mobile Robots with Deep Reinforcement Learning", Sensors 2024, PMC11154580 [snippet] — véletlen pályák generálása.
9. Setiadilaga, Cahyadi, Ataka, "Mecanum-Wheeled Robot Control Based on Deep Reinforcement Learning", ICITEE 2023 [snippet] — mecanum DRL, de célpontra menés.
10. "Reinforcement Learning-Based Trajectory Control for Mecanum Robot with Mass Eccentricity Considerations", J. Robotics and Control (UMY) [snippet] — online actor-critic, nem PPO.
11. Residual MPC, arXiv 2510.12717 [snippet]; Time-attenuating TD3 quadrotor tracking, arXiv 2302.06070 [snippet].

## Isaac Lab
- Beépített command termek: UniformVelocityCommand, UniformPose2dCommand, ... — pálya/trajektória command NINCS, sajátot kell írni (CommandTerm alosztály). [Ellenőrizendő a te Isaac Lab 2.3.2 forrásodban.]
