# Mecanum RL pályakövetés: építési útmutató (Isaac Lab)

Ebben a mappában minden megvan, ami a projekt felépítéséhez kell: a kód, a referenciapályák, a tesztek és a magyarázatok.
Összeállítva: 2026-10-15. A *miért* minden lépésnél a `docs/MAGYARAZAT.md`-ben van (a fejezetszámokra lent hivatkozom).

## 0. Mi van a mappában

```
isaaclab_build/
├── README_EPITES.md            ← ez a fájl
├── mecanum_tracking/           ← AZ ISAAC LAB TASK (ezt másolod a repóba, 1. lépés)
│   ├── __init__.py             gym-regisztráció: Mecanum-Tracking-v0 / -Play-v0 / -Eval-v0 / -Wheel-v0
│   ├── tracking_env_cfg.py     a teljes env-config (scene, command, obs, reward, termination, curriculum, DR)
│   ├── agents/rsl_rl_ppo_cfg.py   PPO-beállítások (a navigációs flat configodból)
│   ├── mdp/
│   │   ├── trajectory_command.py  referencia-generátor CommandTerm (táblázat, interpoláció, anchor, időskála)
│   │   ├── traj_utils.py          tiszta PyTorch segédfüggvények (tesztelve)
│   │   ├── observations.py        ref_preview_b: a referencia a robothoz viszonyítva
│   │   ├── rewards.py             exp-kernelek, forgás-büntetés
│   │   ├── terminations.py        eltérés-termináció
│   │   └── curriculums.py         sebesség-curriculum (időskála)
│   └── refs/
│       ├── test/                  standard tesztpályák: kör, nyolcas, lekerekített és éles négyzet
│       ├── test_unseen/           20 véletlen görbe (seed 999): EGYIK vezérlő sem látja tanítás/hangolás közben
│       └── train/                 ← ezt TE generálod (2. lépés)
├── scripts/eval_tracking.py    kiértékelés → közös CSV log-formátum
├── tools/
│   ├── gen_reference_trajectories.py   tesztpályák generátora
│   ├── gen_training_trajectories.py    tanító pályák generátora (megvalósítható sebességprofillal)
│   └── metrics.py                      metrikák (RL- és MPC-logokra ugyanaz a kód)
├── tests/                      Isaac Lab nélkül futó tesztek (14 db)
└── docs/
    ├── MAGYARAZAT.md           MINDEN, amit átbeszéltünk, és miért ilyen a kód (forrásokkal)
    ├── dontesek.md             döntésnapló: döntés → indoklás → forrás
    ├── irodalom.md             cikklista
    └── kesobbi_kerdesek.md     2 nyitott kérdés, amit még meg kell válaszolnod
```

## 1. Másolás a repóba

A repó gyökeréből (ahol a `scripts/` és a `source/` van):

```bash
B=<ennek a mappának az útvonala>
cp -r $B/mecanum_tracking        source/mecanum_ws/mecanum_ws/tasks/manager_based/
cp    $B/scripts/eval_tracking.py scripts/rsl_rl/          # a play.py és a cli_args.py mellé
cp    $B/tools/*.py               scripts/tools/
cp -r $B/tests                    tests_tracking
```

- A task **magától regisztrálódik**: a `tasks/__init__.py`-od `import_packages`-szel minden alcsomagot betölt (a `.mdp`-t kihagyja).
- A `tracking_env_cfg.py` a navigációs taskodból importálja az `ActionsCfg`-t és az `IkActionsCfg`-t, a PPO-config pedig a `PPORunnerFlatCfg`-t. A navigációs task tehát maradjon a helyén.

## 2. Tanító pályák generálása

```bash
python scripts/tools/gen_training_trajectories.py \
    --out source/mecanum_ws/mecanum_ws/tasks/manager_based/mecanum_tracking/refs/train --n 200 --seed 0
```

(Kb. 30 s. A `test/` és a `test_unseen/` már benne van. A **seed 0 ≠ 999**, így a tanító és a nem látott tesztpályák biztosan különböznek. Lásd MAGYARAZAT 8.1.)

## 3. Tesztek Isaac Lab nélkül (az Isaac Lab Python-környezetében)

```bash
python tests_tracking/test_traj_utils.py      # 12 teszt: interpoláció, θ-átfordulás, anchor, invariancia, időskála, ...
python tests_tracking/test_metrics.py         # 2 teszt: kontúr/lag felbontás, CSV
```

Ha a `tests_tracking` mappát máshova tetted, a `sys.path` sorokat igazítsd a fájlok elején.

## 4. Ellenőrizd a `# CHECK` pontokat (MAGYARAZAT 10. fejezet)

Ezeket nem tudtam ellenőrizni, mert ahol a kód készült, ott nincs Isaac Lab. Ha valamelyik nem stimmel, általában egy sor a javítás:
1. A `ManagerBasedRLEnv._reset_idx`-ben a parancsok **a reset-eventek után** sorsolódnak-e (az anchor erre épít; a navigációs kódod kommentje szerint igen).
2. `env.command_manager.get_term(name)` létezik-e. (Ha nem, akkor `env.command_manager._terms[name]`.)
3. `CommandTerm.__init__` létrehozza-e a `self.metrics` dict-et.
4. `mdp.randomize_rigid_body_mass` paraméternevei.
5. `mdp.action_rate_l2`, `mdp.time_out`, `DoneTerm(..., time_out=True)`.
6. A `scripts/rsl_rl/eval_tracking.py` `# CHECK` sorai (`parse_env_cfg`, `load_cfg_from_registry`, `applied_torque`, a kerékcsukló-nevek).

A `step_dt` igazolva van (a `play.py`-od is használja).

## 5. Gyors próba (tanítás előtt)

```bash
python scripts/zero_agent.py --task Mecanum-Tracking-Play-v0 --num_envs 4
```

Mit kell látnod: piros gömb az aktuális referenciaponton, kék kis gömbök az előretekintő pontokon. A gömbök a pályán haladnak, a robot pedig áll (zero agent), ezért a hiba nő, és kb. 1 m-nél az epizód újraindul (eltérés-termináció). Ha a gömbök nem a robotnál indulnak, a 4/1. pont (reset-sorrend) a hibás.

## 6. Tanítás

```bash
python scripts/rsl_rl/train.py --task Mecanum-Tracking-v0 --headless --seed 0
# a fő eredményhez több seed kell (MAGYARAZAT 9.1): --seed 1, --seed 2 (, 3, 4)
```

TensorBoardban figyeld:
- a **jutalomtagokat egyenként** (`Episode_Reward/...`): a büntetések maradjanak a pozitív tagok töredékei (MAGYARAZAT 7.2);
- `Metrics/ref_traj/pos_error` és `time_scale`, valamint a `Curriculum/speed`: a sebesség-szint fokozatosan nő (MAGYARAZAT 8.2);
- `Episode_Termination/deviation`: az elején sok, aztán csökken.

A jutalomsúlyok **kiinduló** értékek. A hangoláshoz a meglévő eszközeid (`tune_rewards.py`, `staged_penalties.py`, a `PILLAR_PENALTIES`-nél használt kalibráció) ugyanúgy használhatók.

## 7. Megnézés és kiértékelés

```bash
python scripts/rsl_rl/play.py --task Mecanum-Tracking-Play-v0 --num_envs 16

# (a) standard tesztpályák és (b) nem látott görbék, minden seedre:
python scripts/rsl_rl/eval_tracking.py --task Mecanum-Tracking-Eval-v0 --checkpoint <model.pt> \
    --out logs/eval/rl_seed0/test --traj_dir <...>/mecanum_tracking/refs/test --time_scale 1.0 --headless
python scripts/rsl_rl/eval_tracking.py --task Mecanum-Tracking-Eval-v0 --checkpoint <model.pt> \
    --out logs/eval/rl_seed0/unseen --traj_dir <...>/mecanum_tracking/refs/test_unseen --time_scale 1.0 --headless

# metrikák (az MPC-logokra ugyanez):
python scripts/tools/metrics.py --group RL logs/eval/rl_seed*/test/circle.csv --group MPC logs/eval/mpc/test/circle.csv
```

A kísérletmátrix (E1–E5, A1–A4) és a prioritások: MAGYARAZAT 9.4. A számítási időt ne a kiértékelő szkriptből vedd (az a GPU-n, az összes env-re együtt mér). Az exportált `policy.pt`-t mérd egyetlen bemenetre, CPU-n, ugyanazon a gépen, ahol az MPC fut (MAGYARAZAT 9.2).

## 8. Egyeztesd az MPC-s társaddal

- [ ] **Közös log-formátum**: a `tools/metrics.py` fejléce. Az MPC-futások is ilyen CSV-t írjanak, **ugyanabból az Isaac-szimből**, ugyanazzal a kezdőállapottal, súrlódással és tömeggel.
- [ ] **$N$ és $T$**: ezeket írd be a `tracking_env_cfg.py`-ba (`MPC_N`, `MPC_T`), így $K\Delta t = NT$.
- [ ] **Szabad orientáció**: a referenciában nincs $\theta_{ref}$. Az MPC-ben a $\theta$ maradjon állapot, az $\omega$ bemenet, a költségben csak a pozíció és a bemenetek legyenek. A linearizálás az előző megoldás eltolt jósolt trajektóriája körül történjen (RTI, Gros et al. 2020; MAGYARAZAT 6.2).
- [ ] **Az éles négyzetsarok**: a sarokban egyik vezérlő sem tudja pontosan követni a referenciát. Lehetőségek: lekerekítés, a sarok kizárása a metrikából, vagy megállás a sarokban (MAGYARAZAT 2/6).
- [ ] **Hangolás**: ha lehet, a tanító görbéken (`refs/train`) hangoljon, ne a teszthalmazon (MAGYARAZAT 8, dontesek.md).
- [ ] **Fizikai korlátok**: ugyanaz a nyomaték- és keréksebesség-korlát mindkét vezérlőnek (MAGYARAZAT 2).

## 9. Nyitott döntések

- **Akciótér**: twist + IK (`Mecanum-Tracking-v0`, ez az alap) vagy kerékszintű (`Mecanum-Tracking-Wheel-v0`). Ha az MPC nyomatékot ad ki, a teljesen azonos interfészhez nyomaték-akció kellene (MAGYARAZAT 2, 4. pont).
- A `docs/kesobbi_kerdesek.md` két kérdése (ezeket velem még átbeszéljük).
