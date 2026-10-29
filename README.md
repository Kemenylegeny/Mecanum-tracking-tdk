# mecanum_tracking_ws

Isaac Lab projekt (manager-based, RSL-RL) mecanum robot **pályakövetésének** RL-tanítására, az LTV-MPC-vel való
összehasonlításhoz (TDK). Az `../isaaclab_build` kitből raktam össze (2026-10-15). A tervezési döntések indoklása
változatlanul a kit dokumentációjában van: [`docs/MAGYARAZAT.md`](docs/MAGYARAZAT.md), [`docs/dontesek.md`](docs/dontesek.md),
[`docs/irodalom.md`](docs/irodalom.md), [`docs/kesobbi_kerdesek.md`](docs/kesobbi_kerdesek.md). Az eredeti építési
útmutató: [`docs/README_EPITES.md`](docs/README_EPITES.md). Az alábbi „Eltérések a kittől” fejezet sorolja fel, mi
változott hozzá képest. A futások naplója: [`EXPERIMENTS.md`](EXPERIMENTS.md).

**2026-10-23 óta a fő feladat a `Mecanum-Tracking-Torque-*` család:** az RL és a társ LTV-MPC-je ugyanazon a növényen,
ugyanazzal az interfésszel és referenciával, ugyanabban az Isaac-envben fut. A terv: [`docs/OSSZEHANGOLAS.md`](docs/OSSZEHANGOLAS.md),
a megvalósítás a lenti „Összehasonlítás az LTV-MPC-vel” fejezetben.

## Struktúra

```
mecanum_tracking_ws/
├── assets/robots/
│   ├── mecanum/          FUJI-kerekes robot (alapértelmezett): robot.urdf, meshes/roller.stl, wheel_geometry.json
│   └── own_mecanum/      saját robot: robot.urdf, meshes/{chassis_visual,chassis_collision,roller}.stl, robot_config.json
├── source/mecanum_tracking_ws/mecanum_tracking_ws/
│   ├── robots/           mecanum.py (FUJI), own_mecanum.py (saját: mért tömegek, XM430), actuators.py (XM430Motor)
│   ├── mpc/              a társ MPC-je (Mecanum-Robot-MPC-TDK @ fe01f88): params, dynamics, reference, motor,
│   │                     wheel_speed, lmpc (+ DARE, S), tables (referencia ↔ CSV, τ_r tetszőleges pályára)
│   └── tasks/manager_based/mecanum_tracking/       ← a kit taskja
│       ├── __init__.py          gym-regisztráció (lent a táblázat)
│       ├── tracking_env_cfg.py  env-config (scene, akciók, command, obs, reward, termináció, curriculum, DR, kamera)
│       ├── agents/rsl_rl_ppo_cfg.py   PPO (a mecanum_ws navigációs PPORunnerFlatCfg-je, 24 lépés/env)
│       ├── mdp/                 trajectory_command, traj_utils, observations, rewards, terminations, curriculums,
│       │                        actions (mecanum IK, a mecanum_ws-ből), torque_actions (kerék-nyomaték)
│       └── refs/                FUJI: train/ (200 görbe, seed 0), test/ (4 alakzat), test_unseen/ (20 görbe, seed 999)
│                                összevetés: train_own/ (200, seed 0), test_fixed/, test_tangent/ (a társ 4 alakzata),
│                                test_unseen_own/ (20, seed 999); mind 0.1 m/s, τ_r oszlopokkal
├── scripts/
│   ├── rsl_rl/           train.py, play.py, cli_args.py (a mecanum_ws-ből), eval_tracking.py (a kitből, javítva)
│   ├── tools/            gen_reference_trajectories.py, gen_training_trajectories.py, export_mpc_references.py,
│   │                     plot_trajectories.py, metrics.py
│   └── list_envs.py, zero_agent.py, random_agent.py, torque_check.py (növény vs. MPC-modell)
├── tests/                test_traj_utils.py (12), test_metrics.py (3), test_mpc.py (6): Isaac Lab nélkül futnak
├── docs/                 a kit dokumentációja
└── logs/                 rsl_rl/<experiment>/<run>/ (checkpointok, videók), runs/<név>.log (a tanítások stdout-ja)
```

A projekt **önálló**: nem importál a mecanum_ws-ből. A robot-configot, a mecanum IK akciót, a navigációs task akció-configjait
és PPO-configját ide másoltam, így a mecanum_ws változásai nem hatnak rá (és fordítva). Ha a mecanum_ws-ben
újragenerálod a robotot, az URDF-et és a konstansokat ide is át kell hozni.

## Taskok

| Task | Robot | Akció | Mire |
|---|---|---|---|
| `Mecanum-Tracking-v0` | FUJI | twist (vx, vy, ωz) → mecanum IK → kerék-alapjel | tanítás (alap) |
| `Mecanum-Tracking-Play-v0` | FUJI | ugyanaz | nézegetés: 16 env, zaj / DR / curriculum nélkül, s = 1 |
| `Mecanum-Tracking-Eval-v0` | FUJI | ugyanaz | kiértékelés (`eval_tracking.py`): env *i* = tesztpálya *i*, egy kör |
| `Mecanum-Tracking-Wheel-v0` | FUJI | 4 kerék-alapjel | A4 abláció (akciótér) |
| **`Mecanum-Tracking-Torque-v0`** / `-Torque-Play-v0` / `-Torque-Eval-v0` | saját (mért tömegek, XM430) | **4 kerék-nyomaték** | **az összevetés az MPC-vel**; 20 Hz, N = 10, heading-követés |
| `Mecanum-Tracking-Own-v0` / `-Own-Play-v0` / `-Own-Eval-v0` | saját (sebességhurokkal) | twist + IK | ugyanaz a feladat, akciótér-abláció |

A FUJI-taskok 360 Hz-es fizikával és 51 Hz-es policyvel futnak (decimation 7), K = 20, heading nélkül, mint eddig
(a meglévő runok reprodukálhatók). A `-Torque-` és az `-Own-` taskok 20 Hz-esek (decimation 18 = T = 0.05 s).

## A robotok: melyik URDF került bele, és miért

| Mappa (mecanum_ws/assets/robots/…) | Bekerült? | Indok |
|---|---|---|
| `mecanum/robot.urdf` + `meshes/roller.stl` | **igen, ez az alap** | A kit ezt a robotot használja (`robot_name = "fuji"`, `mecanum.MECANUM_CFG`). Görgős, 62 link, a mecanum_ws-ben végigvalidálva (kinematika, tapadás, numerika). |
| `own_mecanum/robot.urdf` + `meshes/*.stl` | igen | A saját (valódi) robot Isaac-kész, görgős URDF-je (41 link, a CAD alvázzal). A tömegek és a motor még **becsült** értékek. A `-Own-` taskok használják. |
| `MecanumRobot_orientalt_v2_description/` | nem | ROS/Fusion-export xacro, **tömör kerék-mesh-ekkel**, görgők nélkül. PhysX-ben ez sima kerékként viselkedne, oldalazni nem tudna. Az alváz-mesh-e (base_link.stl) már benne van az `own_mecanum` alvázában. |
| `own_mecanum/stl/` | nem | A CAD-forrás (49 MB) és az elemző ábrák, csak az újrageneráláshoz kellenek (az a mecanum_ws-ben marad). |

## Eltérések a kittől

1. **Önálló projekt** a mecanum_ws-be másolás helyett (lásd fent). Az importok: `mecanum_tracking_ws.*`.
2. **Időzítési hiba a jutalomban, javítva.** Az Isaac Lab lépés-sorrendje (a telepített forrásban ellenőrizve):
   fizika → termináció → jutalom → reset → command-frissítés → megfigyelés. A kitben a jutalom a lépés *utáni*
   robotállapotot a lépés *előtti* referenciaponthoz mérte. Auto-reset után ráadásul az óra még a reset lépésében is
   lépett, a kezdeti `env.reset()` után viszont nem. Emiatt a tanító epizódok és a kiértékelő epizód (ez mindig a kezdeti
   resetből indul) között egy lépésnyi (~20 ms, 0.5 m/s-nál ~1 cm) eltolódás volt, ami pont az MPC-összehasonlítás
   cm-es skáláján torzított volna. A javítás után:
   - a megfigyelés aktuális pontja = a referencia a robot **aktuális** idejében (resetkor 0 hiba);
   - a jutalom és az eltérés-termináció a `TrajectoryCommand.ref_next`-et használja (egy policy-lépéssel később), vagyis
     a lépés utáni állapotot az ugyanahhoz az időhöz tartozó referenciához méri;
   - a reset lépésében az óra nem lép (`episode_length_buf == 0`).

   Isaac Simben ellenőriztem: reset után a hiba ~1e-6 m, lépésenként `ref_now` == az előző `ref_next`, és kényszerített
   eltérés után az új epizód a `t0`-ról indul.
3. **Curriculum:** a legelső reset (amikor még nincs lefutott epizód) „sikernek” számított (0 átlagos hiba → s 0.5 → 0.6).
   Most kimarad (`ep_steps == 0`).
4. **Új metrika:** `Metrics/ref_traj/episode_pos_error`, a lezárult epizódok átlagos pozícióhibája. A `pos_error` csak az
   utolsó megfigyelt állapoté, eltérés-terminációnál ~1 m.
5. **`eval_tracking.py`:**
   - a saját `--checkpoint` argumentuma ütközött a `cli_args`-éval (argparse-hiba lett volna);
   - RSL-RL 5-höz `handle_deprecated_rsl_rl_cfg` kell;
   - az `x_ref`, `y_ref` értékeket egy lépés után kiolvasott, közben felülírt bufferből vette (a következő lépés
     referenciája lett volna), most `.clone()`;
   - pályánként pontosan egy kört rögzít (a rövidebb pályák eddig a leghosszabb idejéig futottak tovább);
   - a kerékcsukló-neveket a robotmodulból veszi.
6. **Kamera:** a nézet és a videó az env 0 robotját követi felülről-hátulról. A kitben a világ origóját nézte, így a robotok
   kicsik és véletlenszerűek lettek volna a videón.
7. **Saját robot (`-Own-` taskok):** `apply_robot()` cseréli a robotfüggő beállításokat (a mecanum_ws navigációs taskjának
   mintájára). A sebesség-curriculum felső határa itt 1.6, mert a saját robot kerekei legfeljebb 0.91 m/s-ot tudnak, s = 2
   pedig 1.0 m/s-os referenciát jelentene.
8. A kit `ENV_SPACING`-docstringje ~3 m-es tanítógörbéket írt. Valójában a kezdőponttól medián 6.2 m-re, legfeljebb
   10.8 m-re mennek el. Ez ártalmatlan, mert a scene kiszűri az env-ek közötti ütközést (`filter_collisions`); csak a videón
   haladhat át néha egy szomszéd robot.

## A `# CHECK` pontok (MAGYARAZAT 10. fejezet)

A telepített Isaac Lab 0.54.3 és RSL-RL 5.0.1 forrásában ellenőriztem őket, a `# CHECK` jelölések kikerültek a kódból.

| # | Pont | Eredmény |
|---|---|---|
| 1 | `_reset_idx`: a command-reset a reset-eventek után fut | ✓ (`manager_based_rl_env.py`), a `write_root_pose_to_sim` azonnal frissíti a `robot.data`-t |
| 2 | `step_dt` | ✓ |
| 3 | `CommandTermCfg`: `class_type`, `resampling_time_range`, `debug_vis` | ✓ |
| 4 | `self.metrics` dict a `CommandTerm.__init__`-ben | ✓ |
| 5 | `mdp/__init__.py` exportjai | ✓ |
| 6 | `command_manager.get_term(name)` | ✓ |
| 7 | `mdp.action_rate_l2`, `mdp.joint_torques_l2` | ✓ |
| 8 | `mdp.randomize_rigid_body_mass(mass_distribution_params, operation)`, body `base_link` | ✓ (osztály lett, a paraméterek ugyanazok) |
| 9 | `termination_manager.get_term("deviation")` | ✓ |
| 10 | `eval_tracking.py`: `parse_env_cfg`, `load_cfg_from_registry`, `applied_torque`, kerékcsuklók | ✓ (és a fenti javítások) |
| 11 | `mdp.time_out`, `DoneTerm(..., time_out=True)` | ✓; az RSL-RL `ppo.py` a time-outnál bootstrapel |

## Használat

```bash
conda activate env_isaaclab            # /home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab
cd /home/peter.varadi@egroup.hu/IsaacLab/mecanum_tracking_ws
# a csomag editable módban telepítve van:  python -m pip install --no-build-isolation -e source/mecanum_tracking_ws

# tesztek (Isaac Lab nélkül)
python tests/test_traj_utils.py && python tests/test_metrics.py

# tanító pályák (már legenerálva; seed 0 ≠ a test_unseen 999-es seedje)
python scripts/tools/gen_training_trajectories.py \
    --out source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs/train --n 200 --seed 0

# az összes pálya kirajzolva (train / test / test_unseen) -> outputs/trajectories/*.png
python scripts/tools/plot_trajectories.py
# a szimulációban (play, videók) a zöld pöttyös vonal az epizód teljes pályája (első 16 env;
# TrajectoryCommandCfg.path_vis_num_envs), piros = aktuális referenciapont, kék = előretekintés

# regisztrált taskok, gyors próba
python scripts/list_envs.py
python scripts/zero_agent.py --task Mecanum-Tracking-Play-v0 --num_envs 4
```

### Tanítás wandb-vel (mint a mecanum_ws-ben)

```bash
WANDB_USERNAME=varadipeter05-budapesti-m-szaki-s-gazdas-gtudom-nyi-egyetem setsid nohup \
  /home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python -u scripts/rsl_rl/train.py \
  --task Mecanum-Tracking-v0 --headless --seed 0 --num_envs 2048 \
  --video --video_length 500 --video_interval 2400 \
  --logger wandb --log_project_name mecanum-tracking --run_name positive_only \
  env.rewards.action_rate.weight=0.0 env.rewards.yaw_rate.weight=0.0 \
  > logs/runs/positive_only_seed0.log 2>&1 < /dev/null &
```

- **Csak a 3 pozitív tag:** `track_pos_coarse` (σ = 0.5 m), `track_pos_fine` (σ = 0.1 m), `track_vel` (σ = 0.3 m/s). A két
  büntetést (`action_rate`, `yaw_rate`; a configban a kit szerinti −0.01) Hydra-override nullázza. A 0 súlyú tagot a
  reward manager meg sem hívja. A büntetéseket később ugyanígy egyenként lehet visszakapcsolni (MAGYARAZAT 7.2).
- **Wandb:** az RSL-RL beépített loggere. Az entity a `WANDB_USERNAME`, a projekt a `mecanum-tracking`, a run neve a
  log-mappa neve (`<dátum>_<run_name>`). A felvett mp4-eket az RSL-RL a `video` kulcs alá tölti fel.
- **Videó:** `--video_interval 2400` lépés = 100 iteráció (24 lépés/iteráció), `--video_length 500` lépés ≈ 10 s. A kamera
  az env 0 robotját követi: piros gömb = aktuális referenciapont, kékek = előretekintés (K = 20 pont, T = 0.05 s).
- **2048 env**, mert videóval a GPU-n ~8 GB kell (ezen a gépen ~3.4 GB-ot egy másik Isaac Sim folyamat foglal). A
  mecanum_ws tapasztalata szerint 4096 env + videó elszállhat az Omniverse renderelőben (cubric assertion). Videó nélkül
  maradhat a config 4096-os alapértéke. A tempó ~6.5 s/iteráció, 1500 iteráció ≈ 2.7 óra.
- Mindig nézd meg: `grep -c "buffer overflow" logs/runs/<név>.log` (a próbafutásban 0 volt).
- Több seed a fő eredményhez (MAGYARAZAT 9.1): `--seed 1`, `--seed 2`, …

TensorBoard / wandb alatt érdemes figyelni:
- `Episode_Reward/*`: tagonként. Tökéletes követésnél egy 20 s-os epizód összege (1 + 1 + 0.5) · 20 = 50.
- `Metrics/ref_traj/episode_pos_error`: az epizódok átlagos pozícióhibája [m].
- `Curriculum/speed`: az időskála átlaga, 0.5 → 2.0.
- `Episode_Termination/deviation`: az elején sok, aztán csökken.

### Megnézés és kiértékelés

```bash
# a legutóbbi run legutóbbi checkpointja, 16 env (vagy: --checkpoint <model.pt>; --headless --video --video_length 1000)
python scripts/rsl_rl/play.py --task Mecanum-Tracking-Play-v0 --num_envs 16

# bemutató videó: kör, nyolcas, lekerekített négyzet egymás után, egy-egy kör; zöld = a földre festett referencia,
# narancs = a megtett út, felirat: pillanatnyi / átlagos / max pozícióhiba (az eval protokolljával)
python scripts/rsl_rl/play_shapes.py --checkpoint <model.pt> --out logs/videos/shapes.mp4 --headless
# --duration 30: egyenletesen gyorsított lejátszás, hogy a videó 30 s legyen (a szimuláció s = 1 marad, csak
# képkockák maradnak ki; a gyorsítás mértéke a címkockán látszik)
# --shapes circle figure_eight sharp_square: más pályák / sorrend (refs/test; alapértelmezés: kör, nyolcas,
# lekerekített négyzet)

# (a) standard tesztpályák és (b) nem látott görbék, s = 1
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs
python scripts/rsl_rl/eval_tracking.py --task Mecanum-Tracking-Eval-v0 --checkpoint <model.pt> \
    --out logs/eval/rl_seed0/test --traj_dir $R/test --time_scale 1.0 --headless
python scripts/rsl_rl/eval_tracking.py --task Mecanum-Tracking-Eval-v0 --checkpoint <model.pt> \
    --out logs/eval/rl_seed0/unseen --traj_dir $R/test_unseen --time_scale 1.0 --headless

# metrikák (az MPC-logokra ugyanez a szkript)
python scripts/tools/metrics.py logs/eval/rl_seed0/test/*.csv
python scripts/tools/metrics.py --group RL logs/eval/rl_seed*/test/circle.csv --group MPC logs/eval/mpc/test/circle.csv
```

## Összehasonlítás az LTV-MPC-vel (2026-10-23)

A terv: [`docs/OSSZEHANGOLAS.md`](docs/OSSZEHANGOLAS.md) (a 7. fejezete sorolja fel, mi készült el és hol tér el a tervtől).
Vezérelv: **csak a szabályozó különbözhet.**

| | RL (`Mecanum-Tracking-Torque-v0`) | MPC (`eval_tracking.py --controller mpc`) |
|---|---|---|
| növény | Isaac Sim, `robot_measured.urdf` (2.134 kg), XM430 nyomatékmód, 360 Hz | **ugyanaz az env** |
| modell | — | ugyanabból az URDF-ből (`mpc.params.from_urdf`): m_e = 2.215 kg, J_e = 0.03035 kg m² |
| kimenet | 4 kerék-nyomaték, ±1.95 Nm, 20 Hz, ZOH | ugyanaz |
| bemenet | x_r,0..10 a robothoz képest, a robot sebessége (2026-10-28 előtt + u_applied) | x = [X, Y, ψ, ω₁, ω₂, ω₃] (2026-10-29 óta: póz + három kerék fordulatszáma, az alapsebességből a gördülési feltételekkel, ugyanaz az információ, mint az RL-é; előtte [X, Y, ψ, Ẋ, Ẏ, ψ̇]), x_r,0..10, u_r,0..9 (u_applied csak a sosem aktív kemény Δu-korlátban) |
| referencia | a társ `reference.py`-jából exportált CSV (0.1 m/s, rámpa, ψ_r, τ_r) | ugyanaz a CSV, ugyanabból a command-bufferből |
| kezdőállapot | a pálya eleje, ψ = ψ_r(0); 2. forgatókönyv: 30 cm / 30° (`--start_offset 0.3 0 0.52`) | ugyanaz |
| log, metrika | `eval_tracking.py` → CSV → `tools/metrics.py` | ugyanaz |

```bash
PY=/home/bence.farkas@egroup.hu/miniconda3/envs/env_isaaclab/bin/python
R=source/mecanum_tracking_ws/mecanum_tracking_ws/tasks/manager_based/mecanum_tracking/refs

# 0. tesztek (Isaac nélkül) és a növény ellenőrzése (előjelek, kerékszámozás, modell vs. Isaac)
$PY tests/test_mpc.py && $PY tests/test_metrics.py && $PY tests/test_traj_utils.py
$PY -u scripts/torque_check.py --headless

# 1. referenciák (már legenerálva)
$PY scripts/tools/export_mpc_references.py                     # refs/test_fixed, refs/test_tangent
(cd scripts/tools && $PY gen_training_trajectories.py --own --out ../../$R/train_own --n 200 --seed 0 \
  && $PY gen_training_trajectories.py --own --ramp --prefix unseen --out ../../$R/test_unseen_own --n 20 --seed 999)

# 2. tanítás (2048 env: ~11 s/iteráció, 1500 iteráció ≈ 4.5 óra)
WANDB_USERNAME=varadipeter05-budapesti-m-szaki-s-gazdas-gtudom-nyi-egyetem setsid nohup $PY -u scripts/rsl_rl/train.py \
  --task Mecanum-Tracking-Torque-v0 --headless --seed 0 --num_envs 2048 \
  --logger wandb --log_project_name mecanum-tracking --run_name torque \
  > logs/runs/torque_seed0.log 2>&1 < /dev/null &

# 3. kiértékelés: ugyanaz a szkript, ugyanazok a pályák mindkét szabályozóra
for SET in test_fixed test_tangent test_unseen_own; do
  $PY -u scripts/rsl_rl/eval_tracking.py --controller mpc --out logs/eval/mpc/$SET --traj_dir $R/$SET --headless
  $PY -u scripts/rsl_rl/eval_tracking.py --controller policy --checkpoint <model.pt> --out logs/eval/rl_seed0/$SET \
      --traj_dir $R/$SET --headless
done
# 2. forgatókönyv (kezdeti hiba): ... --start_offset 0.3 0.0 0.52 --out logs/eval/<ctrl>/test_fixed_offset
# MPC-változatok: --mpc_terminal scaled (a társ P = 5Q-ja), --mpc_S 100 --mpc_wheel_limit (a 2026-10-28 előtti alap:
# S = 100·I és puha ω-korlát), --mpc_state world (a 2026-10-29 előtti állapot: világ-sebességek a kerékfordulatszámok
# helyett), --mpc_wheel_meas encoder (a kerékfordulatszám a négy enkóderből, nem az alapsebességből);
# a 2026-10-28 előtt tanított checkpointok: --task Mecanum-Tracking-Torque-Offset-Eval-v0

# 4. metrikák (mm, fok; egy sor futásonként) és csoportok (seedek átlaga, bootstrap CI)
$PY scripts/tools/metrics.py --table logs/eval/mpc/test_fixed/*.csv logs/eval/rl_seed0/test_fixed/*.csv
$PY scripts/tools/metrics.py --group RL logs/eval/rl_seed*/test_fixed/circle.csv --group MPC logs/eval/mpc/test_fixed/circle.csv
```

A közös log oszlopai (`metrics.py` fejléce): `t, x, y, yaw, vx, vy, wz, x_ref, y_ref, yaw_ref, vx_ref, vy_ref, wz_ref,
u_0..3` (kiadott nyomaték, az MPC kerékszámozásában), `tau_0..3` (amit a motormodell ténylegesen alkalmazott), `w_0..3`
(keréksebesség), `compute_ms` (az MPC lépésideje / a policy egyetlen megfigyelésre, CPU-n, TorchScript), `failed`; az
MPC-nél még `qp_ms, status, slack_max, dare_iters`.

A kísérletmátrix (E1–E5, A1–A4), a nyitott döntések (akciótér, éles négyzetsarok, az MPC $N$ és $T$ értéke) és az MPC-s
társsal egyeztetendő pontok változatlanul a `docs/README_EPITES.md` 8–9. és a `docs/MAGYARAZAT.md` 9. fejezetében vannak.
