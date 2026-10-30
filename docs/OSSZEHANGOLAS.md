# MPC ↔ RL összehangolás: mit kell változtatni, hogy a kettő összevethető legyen

Összeállítva: 2026-10-23 (a „sárkányharcos” lecke végén).
Kiindulás: az LTV-MPC **a társad repója szerint** (Pvaradi05/Mecanum-Robot-MPC-TDK, `fe01f88`, `mecanum_lmpc_v2/lmpc.py`), két kiegészítéssel: **DARE-végköltség** és **$S$-büntetés** a nyomatékváltozásra.
Az RL: a `isaaclab_build/` csomag (`mecanum_tracking`, Isaac Lab, PPO).

> Az 1–6. fejezet a terv, változatlanul. A **7. fejezet** írja le, mi készült el belőle 2026-10-23-án ebben a projektben
> (`mecanum_tracking_ws`), és hol tér el a megvalósítás a tervtől.

**Az egyetlen vezérelv:** *csak a szabályozó különbözhet.* Ugyanaz a növény (szimulátor, robot, motor), ugyanaz a feladat (referencia, heading, kezdőállapot), ugyanaz az interfész (mit ad ki a szabályozó, milyen gyakran, milyen korlátokkal), és ugyanaz a mérce (közös log, közös metrika-kód). Minden alábbi pont ennek egy-egy esete.

---

## 0. Röviden

| # | Eltérés most | Mi kell | Kinél |
|---|---|---|---|
| 1 | MPC Gazebóban, RL Isaac Labben | **mindkettő ugyanabban az Isaac Lab envben**; az MPC is az `eval_tracking.py`-ból fut | MPC |
| 2 | RL a nagy „fuji” robottal (`mecanum.py`, $r = 0.1025$ m, 20 Nm) | **saját robot, mért tömegekkel**: `robot_measured_B.urdf` mindkettőnek | RL |
| 3 | RL motor: tippelt JGB37 (`own_mecanum.py`) | **XM430-W210-T**: $\tau_{max} = 1.95$ Nm, $\omega_{max} = 8.06$ rad/s | RL |
| 4 | RL akció: twist → IK → kerék-sebesség alapjel | **4 kerék-nyomaték**, $\pm 1.95$ Nm | RL |
| 5 | RL 51 Hz (decimation 7), MPC 20 Hz | **mindkettő 20 Hz** ($T = 0.05$ s) | RL |
| 6 | RL preview $N = 20$ | **$N = 10$**, $T = 0.05$ → 0.5 s | RL |
| 7 | RL heading-szabad, MPC követi $\psi_r$-t | **RL is kövesse a headinget** | RL |
| 8 | RL referencia 0.5 m/s, időskála 2-ig | **≤ ~0.1–0.15 m/s** (a motor 0.24 m/s kerékkerületi sebességnél áll meg) | RL |
| 9 | RL pályák saját generátorral, rámpa nélkül | **ugyanazok a pályák**: a társad `reference.py`-jából exportálva (koszinuszos rámpa, $\psi_r$, $\tau_r$) | mindkettő |
| 10 | log-formátum különbözik | **egy log, egy `metrics.py`**, kiegészítve heading-hibával és kerék-sebességgel | mindkettő |

---

## 1. Heading-unwrap — melyik megoldás?

**Az LTV-MPC-hez a referenciás unwrap a jobb**, a $\sin/\cos$-alak inkább NMPC-be való.

- **Referenciás (ezt használd):** a mért szöget a horizont elején a referencia köré csomagolod:

$$
\psi_{mért} \leftarrow \psi_{r,0} + \mathrm{wrap}(\psi_{mért} - \psi_{r,0})
$$

Az LTV úgyis eltérésben számol ($\delta\psi$), ez így mindig kicsi, az állapot 6 dimenziós marad, és a linearizálás érvényes. Az $A$, $B$ mátrixokban $\psi$ csak $\sin$/$\cos$ alakban szerepel, tehát ott nincs ugrás.

- **$\sin/\cos$ az állapotban (ne az LTV-be):** az állapot redundáns lesz (7 változó + a $c^2 + s^2 = 1$ kényszer, amit a lineáris predikció nem tart be), lesz egy irányíthatatlan, $\lambda \approx 1$ irány (a vektor hossza), ami a DARE-t megnehezíti, és kis hibánál semmit nem nyersz:

$$
(c - c_r)^2 + (s - s_r)^2 = 4\sin^2\frac{\delta\psi}{2} \approx \delta\psi^2
$$

- **Hol jó a $\sin/\cos$ vagy az $1 - \cos(\psi - \psi_r)$ költség:** NMPC-ben, ahol a költség közvetlenül az abszolút szögből számolódik.

- **A csapda:** a wrap-ot *csak a mért szögre, csak a horizont elején* szabad csinálni (a társad is így csinálja, `lmpc.py`: `x[2] = x_r[0, 2] + wrap(x[2] - x_r[0, 2])`). **A referencia szöge legyen folytonos** (unwrapped): egy körpályán $\psi_r$ nőjön $\pi$ fölé, ne ugorjon $-\pi$-re. Különben a horizont közepén $\delta\psi_j$ $2\pi$-t ugrik, és a QP egy teljes visszafordulást „lát”.

**Az RL-ben ugyanez:** a megfigyelésbe a heading-hibát $\sin(\delta\psi)$, $\cos(\delta\psi)$ formában add (a `ref_preview_b` `include_heading=True` ezt teszi), a jutalomba pedig $\mathrm{wrap}(\psi - \psi_r)$ menjen. Az RL-nek nincs linearizálása, ott a $\sin/\cos$ a természetes.

---

## 2. Az MPC-oldal: elég-e a társad LTV-MPC-je + DARE + $S$?

**Igen, a vezérlő-algoritmus így jó.** Az affin tag, a pontos RK4-Jacobi, a puha kerék-sebességkorlát, az unwrap és a tartalékterv maradjon, mind hasznos. Amit még **kell** csinálni, az nem az algoritmusban van, hanem a környezetben:

### 2.1 Kötelező

1. **Gazebo → Isaac Lab.** Ez a legnagyobb munka, és az összevethetőség alapja. A legtisztább út: **az MPC az `eval_tracking.py`-ban fut**, ugyanabban az envben, ahol a policy, csak a `policy(obs)` hívás helyén egy `mpc.step(x_meas, x_r, u_r, u_applied)` áll. Így a növény, a kezdőállapot, a reset, a log és a termináció *biztosan* azonos. A `LinearMPCv2` tiszta numpy + `quadprog`, ROS nélkül is fut. Kevés env kell (4 tesztpálya → 4 env), CPU-n ciklusban hívva.
   - Az állapot az Isaac ground truth-ból jön: `root_pos_w`, a yaw a kvaternióból, `root_lin_vel_w` (már világkeretben, ahogy az ő modellje kéri), `root_ang_vel_w[:, 2]`.
   - Az ő `/odom`-glitch szűrése (`mpc_node.py:150`) Gazebo-specifikus, itt nem kell. Az unwrap viszont kell.
2. **Ugyanaz a URDF a modellhez és a növényhez.** A `params.from_urdf(...)` a `robot_measured_B.urdf`-et olvassa (kipróbáltam, működik: $m = 2.134$ kg, $m_e = 2.222$ kg, $J_e = 0.0334$ kg·m²). Az ő Gazebo-URDF-je kicsit más ($J_e = 0.0304$, kb. 10%), tehát **ne** azt használja.
3. **Ugyanazok a korlátok, mint a növényben:** `u_max = 1.95`, `omega_max = 8.06` (a `motor.py` már ezt tartalmazza).
4. **A nyomaték-előjelek / kerékszámozás** egyeztetése az Isaac-csuklókkal (az ő `params.py`-ja `WHEELS = (front_left, front_right, rear_left, rear_right)`, `S_V`, `S_R` előjelekkel). Egy gyorsulásteszttel ellenőrizd: mind a négy $+0.02$ Nm → előre gyorsul ~1.19 m/s²-tel (a fő leckéből, $M_B$-vel).

### 2.2 A két kiegészítés, kondenzált formában

**DARE-végköltség.** A `Qbar` utolsó blokkja $5Q$ helyett $P_{DARE}$. Riccati-iterációval, scipy nélkül, a horizont végi $(A_{N-1}, B_{N-1})$-re (a társad robotján ~50 iteráció alatt konvergál, ezt lemértem):

```python
P = Q.copy()
for _ in range(500):
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    P_new = Q + A.T @ P @ (A - B @ K)
    if np.abs(P_new - P).max() < 1e-9 * np.abs(P).max(): break
    P = P_new
```

Melegindítva az előző lépés $P$-jével ez 1–3 iteráció lépésenként. Az eredmény a pozícióra ~6.1×, a szögre ~4.5×, a sebességekre ~1.1× a $Q$ (tehát közel van az $5Q$-hoz, de a sebességeket nem súlyozza túl).

**$S$-büntetés a nyomatékváltozásra.** Az abszolút nyomaték-változás a kódban már megvan: $D\tilde u + \delta$, ahol `delta = [u_r[0] - u_applied, u_r[1:] - u_r[:-1]]` (az első sor a **ténylegesen kiadott** nyomatékhoz köt, így helyes). A költséghez:

$$
J \mathrel{+}= (D\tilde u + \delta)^T \bar S (D\tilde u + \delta)
$$

ami a QP-ben:

$$
H \mathrel{+}= 2 D^T \bar S D, \qquad f \mathrel{+}= 2 D^T \bar S \delta
$$

($\bar S = I_N \otimes S$.) A kemény rátakorlát (`du_max = u_max`) maradhat, úgysem aktív. (2026-10-30: kivettük; kezdeti hiba nélkül sosem aktív, a BO-súlyokkal a kezdeti hibás indulásnál igen; `docs/dontesek.md`.)

### 2.3 Opcionális (nem kell az egyenértékűséghez)

- **Nyomaték–fordulatszám görbe.** Ha a növényben lineáris motorgörbe van (lásd 3.3), akkor $\omega > 2.8$ rad/s fölött már nem érhető el az 1.95 Nm. Az MPC ezt nem tudja, csak a dobozt és a $\omega_{max}$-ot. 0.1 m/s-nál a referencianyomatékok $\le 0.02$ Nm (lemértem), tehát követés közben nem számít; nagy kezdeti hibánál, ahol az MPC nagy nyomatékot kér, igen. Ha mégis, akkor egy lineáris korlát kerül a QP-be: $|\tau_i| \le \tau_s (1 - |\omega_i| / \omega_0)$, a referencia mentén linearizálva.
- **Súrlódás / integrátor.** A társad modellje „tiszta”, nincs benne súrlódás vagy zavarbecslés. Isaacban a görgők csillapítása miatt maradó követési hiba jöhet. Az összevetéshez ezt **ne** javítsd előre: az „MPC modellhibával” vs. „RL, ami megtanulja” pont a kísérlet tárgya. Ha nagyon rossz, egy zavarbecslő a második körben jöhet, és a dolgozatban külön sorként jelenjen meg.

---

## 3. Az RL-oldal: mit kell átírni

Fájl: `mecanum_tracking/tracking_env_cfg.py`, ha mást nem írok.

### 3.1 Robot: a saját, mért tömegű robot (`:27`)

- `from mecanum_ws.robots import mecanum` → a **saját robot** konfigja (`own_mecanum.py`), de a `URDF_PATH` a **`robot_measured_B.urdf`**-re mutasson (ugyanaz a 41 linkes, görgős URDF, csak a mért tömegekkel; a `make_measured_urdf.py` ebből készítette).
- A `own_mecanum.py` geometriája már egyezik az MPC-vel ($r = 0.03028$, $l_x = 0.064$, $l_y = 0.1526$), ezt ellenőriztem.
- Ami **tipp** volt benne, és cserélni kell: a motor (JGB37 → XM430, lásd 3.3) és a gumi-kontakt értékek (ez maradhat, de **ugyanaz** legyen az MPC-futásoknál, ami automatikus, ha ugyanabban az envben futnak).

### 3.2 Akció: kerék-nyomaték (`:211`)

- Az `IkActionsCfg` (twist → IK → sebesség-alapjel) helyett **nyomaték-akció**: 4 dimenziós, `JointEffortActionCfg` a négy kerékcsuklóra, `scale = 1.95`, és a policy kimenete legyen $[-1, 1]$-re vágva (vagy a szabályozó oldalon `clip`). Így pontosan az lesz az interfész, amit az MPC kiad.
- A twist-akciós változat maradhat **ablációnak** („mennyit számít az akciótér?”), de a fő összevetés a nyomaték-akciós legyen.
- Tanítási tipp: a nyomaték-akció nehezebb. Az `action_rate` büntetés (`:143`) itt fontosabb lesz, és ez az RL-es megfelelője az MPC $S$-ének.

### 3.3 Aktuátor (növény): XM430, nyomatékvezérlés

A saját robot konfigjában a kerekek aktuátora:

```python
"wheels": DCMotorCfg(
    joint_names_expr=WHEEL_JOINT_NAMES,
    stiffness=0.0, damping=0.0,        # tiszta nyomaték-parancs, nincs sebesség-szabályozó
    saturation_effort=3.0,             # XM430 stall torque @12 V [Nm]
    effort_limit=1.95,                 # a teljesítménygrafikon maximuma, = az MPC u_max-a
    velocity_limit=8.06,               # terheletlen fordulat @12 V [rad/s]
    armature=0.0,                      # a fő leckében így döntöttünk (sysID később)
)
```

- `# CHECK`: hogy a `DCMotorCfg` `stiffness = damping = 0` mellett az effort-parancsot a lineáris nyomaték–fordulat görbével vágja-e (ezt a saját Isaac Lab verziódban nézd meg az `actuator_pd.py`-ban). Ha nem, akkor `IdealPDActuatorCfg` + saját clamp.
- A régi `damping=0.1` sebességhurok-erősítés és az `armature=9e-4` a sebesség-alapjeles akcióhoz kellett (explicit stabilitás). Nyomaték-akciónál mindkettő kikerül.
- Ez a **növény** — mindkét szabályozó ugyanezt a motort hajtja.

### 3.4 Vezérlési frekvencia: 20 Hz (`:220`)

- `self.decimation = 7` (51 Hz) → **`self.decimation = 18`** ($18 / 360 = 0.05$ s). A fizika marad 360 Hz.
- Így a policy is 50 ms-onként ad új nyomatékot, és közben tartja (ZOH), pont mint az MPC.

### 3.5 Előretekintés (`:40–42`)

- `MPC_N = 20` → **`MPC_N = 10`**, `MPC_T = 0.05` marad. A preview így 0.5 s, mint az MPC horizontja.

### 3.6 Heading követése (`:99`, `:114`, jutalmak)

A társad MPC-je követi a $\psi_r$-t (alapértelmezés: `heading='fixed'`, a $Q$-ban $\psi$-súly 3283). Hogy ugyanaz legyen a feladat, **az RL is kövesse a headinget**:

- `velocity_in_traj_frame` és a referenciafájlok: legyen benne a `theta` oszlop ($\psi_r$, folytonosan).
- `ref_preview_b`: `include_heading=True`.
- Új jutalomtag: `track_heading = RewTerm(func=mdp.track_heading_exp, weight=0.5, params={"command_name": "ref_traj", "std": 0.2})`, ahol a hiba $\mathrm{wrap}(\psi - \psi_r)$. (Ezt a függvényt meg kell írni a `rewards.py`-ba, a `track_pos_exp` mintájára.)
- A `yaw_rate` büntetés (`:144`) heading-követésnél zavarhat (tangens headingnél a robotnak forognia kell), ezt `weight=0`-ra vagy a $\dot\psi - \dot\psi_r$ hibára érdemes átírni.

**Miért nem fordítva?** Elvileg az MPC-ből is ki lehetne venni a headinget ($Q_\psi = 0$), de az LTV a $\psi_r$ körül linearizál, heading-szabad feladatnál ennek nincs értelme (a régi terv RTI-t javasolt, ami nagyobb átírás). Az RL-be betenni a headinget egyetlen jutalomtag.

Ez **visszavonja a 4. csomópont „heading-szabad” döntését** — írd be a `dontesek.md`-be az indokkal: „az MPC heading-követő; azonos feladat kell”.

### 3.7 Referencia: sebesség és pályák

**A sebesség a fő gond.** A motor $\omega_{max} = 8.06$ rad/s, $r = 0.0303$ m → **0.24 m/s kerékkerületi sebesség**. Mecanumnál oldalazva és forogva ennél is kevesebb jut a robotra. A mostani tesztpályák 0.5 m/s-mal, a curriculum 2× időskálával mennek: **ezek a robotnak megvalósíthatatlanok**.

- A társad alapértéke `speed=0.1` m/s („feasible on every path with the XM430 limits”). A `Reference.check_limits(...)`-szal lemértem a `robot_measured_B.urdf`-fel (a legnagyobb referencia-kerékszögsebesség, határ: 8.06 rad/s):

| sebesség | fixed heading (minden alakzat) | tangent heading |
|---|---|---|
| 0.10 m/s | 4.67 rad/s ✅ | 4.3–6.7 rad/s ✅ (nyolcas a legrosszabb) |
| 0.15 m/s | 7.01 rad/s ✅ | kör ✅, ellipszis / nyolcas / négyzet ❌ (9.2–10.1) |
| 0.20 m/s | 9.34 rad/s ❌ | ❌ |

  Fő eset tehát: **0.1 m/s**; 0.15 m/s csak fixed headinggel. A referencianyomatékok közben **kicsik** ($\le 0.02$ Nm, a robot könnyű): a szűk keresztmetszet a **kerék-sebesség**, nem a nyomaték. Ezért fontos, hogy a puha $\omega$-korlát és a növény motorgörbéje egyezzen.
- **Curriculum** (`:161`): `scale_range=(0.5, 2.0)` → a felső határ olyan legyen, hogy a leggyorsabb tanítópálya is megvalósítható maradjon: `v_max = 0.1` mellett fixed headinggel ~1.5, tangenssel ~1.2 (8.06 / 6.73).
- **Tanítópályák** (`tools/gen_training_trajectories.py`): `--v_max 0.5` → **0.1**; az `a_lat`, `a_tan` maradhat (a nyomaték bőven elég, a sebesség a szűk keresztmetszet).
- `initial_time_scale` (`:98`) maradhat.

**Ugyanazok a tesztpályák.** A legegyszerűbb: a társad `reference.py`-jából exportálod a tesztpályákat az RL CSV-formátumába (`t, X, Y, theta, vx, vy, omega, tau_1..4`):

- alakzatok: `circle`, `figure8`, `square` (lekerekített, `corner_radius` alapérték), `ellipse` — **az éles négyzet kiesik** (az ő generátora nem tud ilyet, és nyomatékkorláttal úgysem követhető);
- benne van a **koszinuszos sebesség-rámpa** az elején (a mostani RL-pályák álló helyzetből rögtön 0.5 m/s-mal indulnak — ez mindkét szabályozónak egy ugrás lenne);
- benne van a $\psi_r$ (fixed és/vagy tangent), folytonosan;
- benne van a lépésátlagolt $\tau_r$ (`step_torques`), így az RL is megkaphatja előrecsatolásként (`n_extra=4`) — ez opcionális, de az MPC-nek is van ilyen információja; ablációnak jó.
- `loop=False`, egy kör, ugyanaz az epizódhossz.

**Nem látott pályák (`refs/test_unseen`).** Ezekhez az MPC-nek is kell $\tau_r$: a CSV-ből (spline + deriválás + az ő inverz dinamikája: $w_r = M\ddot p + \dots$, $u_r = G^+ w_r$) — ez egy kis kiegészítés a `reference.py`-hoz (`Reference.from_csv`). Ha nincs idő rá, a nem látott pályákat csak az RL általánosítására használd, és a dolgozatban így szerepeljen.

### 3.8 Megfigyelés (apróságok)

- `wheel_vel` skálázás (`:122`): `1 / MAX_WHEEL_SPEED` → `1 / 8.06`.
- `base_lin_vel` testkeretben jön — ez rendben van, a policynek mindegy; az MPC-nek a világkeretes sebesség kell, de az a ground truth-ból jön.
- `last_action` maradjon (ez a $\tau_{k-1}$, az MPC is tudja: `u_applied`).

### 3.9 Kezdőállapot és termináció

- **Eval:** a robot a referencia kezdőpontján, $\psi = \psi_r(0)$-val (most `yaw = 0`; fixed headingnél ez egyezik, tangentnél állítsd be). Az MPC ugyanebből az envből indul, ez automatikus, ha az `eval_tracking.py` futtatja.
- **Második forgatókönyv:** kezdeti hiba **30 cm / 30°** (a társad ezzel mérte a slack árnyékárát) — mindkét szabályozóra ugyanazzal a seeddel/értékkel.
- A deviation-termináció (1 m, `:152`) mindkettőnél ugyanaz: a log `failed` oszlopa.

### 3.10 Domain randomization és zaj

- Tanításkor maradhat (ez az RL eszköze), **kiértékeléskor ki** (a `_PLAY`/`_EVAL` config már kikapcsolja). Az MPC modellje a névleges URDF, a növény is névleges → igazságos.
- A tömeg-randomizálás `(0.8, 1.2)` maradhat; ha az MPC-t is akarod robusztusságra tesztelni, ugyanazzal a perturbált növénnyel futtasd mindkettőt (külön kísérlet).

---

## 4. Közös kiértékelés

**Log (`tools/metrics.py` fejléc), kiegészítve:**
- `yaw_ref` → **heading-hiba** metrika: RMSE és max $|\mathrm{wrap}(\psi - \psi_r)|$;
- `w_0..w_3` (kerék-szögsebesség) → **$\omega_{max}$-túllépés**: max és időarány (az MPC puha korlátja vs. a policy);
- `u_0..u_3` = a kiadott nyomaték (mindkettőnél ugyanaz a jelentés), `tau_0..tau_3` = a szimulátor által ténylegesen alkalmazott (a motorgörbe vághat!);
- a régi „szabad heading” metrikák (oldalazás-arány) maradhatnak, de már nem fő eredmény.

**Metrikák, mindkettőre ugyanaz a kód:** pozíció RMSE / max, kontúr- és lag-hiba, heading-hiba, nyomaték-RMS, $\|\Delta\tau\|$ (simaság), $\omega$-túllépés, sikertelen futások aránya.

**Számítási idő:** az MPC `qp_ms`/`step_ms`-e (CPU), a policy egyetlen bemenetre, CPU-n, **ugyanazon a gépen** (README 7. lépés).

**Hangolás:** az MPC súlyait és az RL jutalmait a **tanítópályákon** hangold, ne a teszten (README 8).

---

## 5. Sorrend (javaslat)

1. Robot + motor + nyomaték-akció + 20 Hz az RL envben (3.1–3.4), gyorsulásteszt a zero/konstans-nyomaték ágenssel.
2. Az MPC beillesztése az `eval_tracking.py`-ba (2.1), a társad `LinearMPCv2`-jével, a `robot_measured_B.urdf`-fel; egy körön ellenőrizni, hogy követ.
3. DARE + $S$ (2.2).
4. Közös tesztpályák exportja a `reference.py`-ból (3.7), heading az RL-be (3.6).
5. RL újratanítás (több seed), aztán a közös kiértékelés (4).

## 6. Nyitott döntések — és mi van belőlük már a társad repójában

| Döntés | A társad repójában (`fe01f88`) | Mit jelent nekünk |
|---|---|---|
| **Heading: fixed vagy tangent a fő eset?** | **Mindkettő megvan** (`heading:=fixed\|tangent`). A kód alapértéke **fixed** (`mpc_node.py:78`), de a README-k minden példája **tangent**-tel fut (`shape:=square heading:=tangent`). Fő esetet nem rögzít. | **Kérdezd meg tőle.** Ha tangent: a curriculum időskála-felső határa ~1.2 (a 0.1 m/s-os nyolcas már 6.73 rad/s-ot kér a 8.06-ból). Ha fixed: ~1.5. |
| **Sebesség: 0.1 vagy 0.15 m/s is?** | **0.1 m/s, kimondott döntés.** Indok: 0.2 m/s-nál minden pálya 81–129 rpm-et kérne, a határ 77 rpm (`mecanum_lmpc_v2/README.md:56`; gyökér `README.md:20`). A 0.15-ről nem ír. | Fő eset **0.1 m/s**, az RL ehhez igazodik (3.7). A 0.15 csak opcionális kiegészítés, és csak fixed headinggel megvalósítható (a 3.7 táblázata szerint). |
| **$\tau_r$ az RL megfigyelésében?** | **Nem az ő döntése.** Az MPC-je **mindig** kap $u_r$-t (`mpc_node.py:180`: `x_r, u_r = self.ref.horizon(...)`), és nem is kapcsolható ki: erre épül a linearizálás, az előrecsatolás ($u = u_r + \tilde u$), az $R$ középpontja és az eltolt nyomatékdoboz. | **Te döntesz.** Javaslat: fő eset nélküle (az RL csak pályát lát), abláció vele (azonos előrecsatolási információ). A $u_r$ kicsi ($\le 0.02$ Nm), valószínűleg kevés különbség — ez is eredmény. Figyelem: a `tau` oszlopok az időskálás curriculummal nem kompatibilisek. |
| **MPC a nem látott pályákon?** | **Nincs meg.** A `reference.py` csak a négy analitikus alakzatot ismeri (`circle`, `ellipse`, `figure8`, `square`); CSV-ből referenciát nem tud betölteni (az `evaluate.py` CSV-olvasása a futási logokra vonatkozik). | Kell hozzá a `Reference.from_csv` (spline + inverz dinamika + $G^+$). Ha nem készül el: a nem látott pályák **csak az RL általánosítását** mérik, és a dolgozatban is így szerepeljenek. |

**Összefoglalva:** a **sebességben** már egyetértetek (0.1 m/s). Az egyetlen kérdés, amit a **társaddal** tisztázni kell, a **fő heading-mód**. A $\tau_r$-megfigyelés a **te** döntésed. A nem látott pályákhoz pedig valakinek meg kell írnia a CSV-betöltést.

---

## 7. Megvalósítás (2026-10-23)

Állapot: **minden pont elkészült**, az MPC fut az Isaac-envben (eredmények: 7.4), az RL-tanítás a rövid
próbafutásokon kívül (7.5) még hátravan. Ellenőrzés: `tests/test_mpc.py` (6), `tests/test_metrics.py` (3),
`tests/test_traj_utils.py` (12) zöld; `scripts/torque_check.py`; az MPC- és a policy-kiértékelés végigfut.

### 7.1 Pontonként (a 0. fejezet táblázata szerint)

| # | Mi készült | Hol |
|---|---|---|
| 1 | Az MPC a projektben: a társ `mecanum_mpc_common` moduljai (`params`, `dynamics`, `motor`, `wheel_speed`, `reference`, `conventions`) és a `LinearMPCv2` változatlanul átmásolva (csak az importok relatívak; alapbeállítással **bitre azonos** kimenet az eredetivel, 240 véletlen bemeneten ellenőrizve). Az `eval_tracking.py --controller mpc` futtatja, ugyanabban az envben, mint a policyt | `source/.../mpc/`, `scripts/rsl_rl/eval_tracking.py` |
| 2 | `robot_measured.urdf` = a társ Gazebo-URDF-je (mért tömegek). Az inerciákon kívül bájtra azonos a mi `robot.urdf`-ünkkel (diff), így **a növény és az MPC modellje ugyanabból a fájlból** jön | `assets/robots/own_mecanum/robot_measured.urdf`, `robots/own_mecanum.py` |
| 3 | XM430: 1.95 Nm, 8.06 rad/s, 3.0 Nm álló nyomaték, armature 0, nyomatékmód (stiffness = damping = 0) — saját `XM430Motor` modellel (7.3) | `robots/actuators.py`, `robots/own_mecanum.py` |
| 4 | Nyomaték-akció: `WheelTorqueAction` (= `JointEffortAction` + az előző lépés vágott nyomatéka), τ = clip(0.25 · a, ±1.95) | `mdp/torque_actions.py`, `TorqueActionsCfg` |
| 5 | 20 Hz: decimation 18 | `apply_comparison()` |
| 6 | K = N = 10, T = 0.05 s | `COMPARE_N`, `COMPARE_T` |
| 7 | Heading-követés: `include_heading=True`, `track_heading_exp` (σ 0.2), a `yaw_rate` helyett `yaw_rate_error_l2` (ψ̇ − ψ̇_r) | `mdp/rewards.py`, `CompareRewardsCfg` |
| 8 | 0.1 m/s; tanítópályák keréksebesség-korláttal (ω ≤ 8.06/1.2), curriculum s ∈ [0.5; 1.2] | `tools/gen_training_trajectories.py --own`, `refs/train_own` |
| 9 | A társ 4 alakzata exportálva (rámpa, ψ_r folytonosan, τ_r = az ő `step_torques`-a), fixed és tangent; a nem látott pályákhoz is van τ_r (7.2) | `tools/export_mpc_references.py`, `refs/test_fixed`, `refs/test_tangent`, `refs/test_unseen_own` |
| 10 | Közös log és `metrics.py`: heading-hiba, pályától mért távolság (a társ `evaluate.py`-ja szerint), ω-túllépés, alkalmazott vs. kiadott nyomaték, ‖Δu‖, számítási idő; ablak: a referencia mozgásának végéig | `tools/metrics.py` (`--table`, `--group`) |
| 2.2 | DARE-végköltség (Riccati-iteráció, melegindítás; 52 iteráció hidegen, 1–2 melegen; P/Q = 6.06 / 4.5 / 1.1) és S-büntetés (H += 2DᵀS̄D, f += 2DᵀS̄δ), mindkettő opció, az alapérték DARE + S = 100·I | `mpc/lmpc.py`; `--mpc_terminal`, `--mpc_S` |
| 3.9 | Kezdeti hiba: `TrajectoryCommandCfg.start_offset` (a társ `Reference.anchor` szemantikája), `--start_offset 0.3 0 0.52` | `mdp/trajectory_command.py` |

### 7.2 Eltérések a tervtől, és miért

- **`robot_measured_B.urdf` nincs meg** (sem itt, sem a társ repójában), ezért a társ mért tömegű URDF-jét használom.
  Ezzel $m_e = 2.215$ kg, $J_e = 0.03035$ kg·m² (a terv „B” változata 2.222 / 0.0334-et írt). Az elv teljesül, mert a
  modell és a növény ugyanabból a fájlból jön, és ráadásul a társ Gazebo-modelljével is azonos. Ha a B-változat
  előkerül, csak a fájlt kell kicserélni (`own_mecanum.URDF_PATH`).
- **Akcióskála 0.25 Nm/egység** a terv 1.95-je helyett, a vágás továbbra is ±1.95 Nm (ugyanaz a doboz, mint az MPC-é).
  A referencianyomaték ≤ 0.01 Nm, a tapadási határ ~0.14 Nm/kerék, 1.95-ös egységgel már a kezdeti felfedező zaj
  (σ = 0.5) is körbepörgetné a kerekeket.
- **A megfigyelésből kikerült a keréksebesség** és a 3D-s sebesség-maradék (vz, ωx, ωy): az MPC állapotában
  ($X, Y, \psi, \dot X, \dot Y, \dot\psi$) nincsenek benne. A megfigyelés pontosan az MPC `step()`-jének bemenete
  ($u_r$ nélkül, az abláció), a robot keretében kifejezve.
- **Az Isaac `DCMotor` helyett saját `XM430Motor`** (7.3).
- **Reset-magasság 5 → 1 mm** (7.3).
- **Tanítópályák: fele fixed, fele tangent heading**, mert a fő heading-mód még nyitott (6. fejezet), így a policy
  mindkettőt tudja.
- **Nem látott pályák rámpával** (`--ramp`: 0-ról indul és 0-ra áll meg), hogy az MPC-nek se legyen sebességugrás. Ezek
  0.1 m/s-mal 135–291 s hosszúak.
- **A τ_r a nem látott pályákhoz:** a `Reference.from_csv` helyett `mpc/tables.py` `path_torques`: ugyanaz az inverz
  dinamika a mintavételezett pályán (véges differenciák). Ellenőrizve: a 4 analitikus alakzaton a társ
  `step_torques`-ától ≤ 2·10⁻⁶ Nm-rel tér el (a tangent négyzeten 5·10⁻³, a ψ̇ ugrásai miatt, ott az export az eredetit
  használja). Az MPC a névleges modellen ezeken a pályákon is ≤ 0.6 mm-rel követ.
- **Jutalom:** a kernelek a 0.1 m/s / mm-es feladathoz skálázva (pozíció σ 0.25 és 0.05 m, sebesség σ 0.1 m/s),
  `torque_rate` −20 (az első próbán −1-gyel rángatott), és új tag, `wheel_speed` (−0.1): a túllépés négyzete, az MPC
  puha keréksebesség-korlátjának párja.
- **A FUJI-taskok változatlanok** (K = 20, 51 Hz, heading-szabad), hogy a meglévő runok reprodukálhatók maradjanak; az
  `-Own-` taskok mostantól az összevetés twist-akciós ablációi (sebességhurok, ehhez kell armature, tehát kicsit más
  növény — ezt a dolgozatban ki kell mondani).

### 7.3 A növény ellenőrzése és két javítás

**Előjelek, számozás, dinamika** (`scripts/torque_check.py`, 0.01 Nm/kerék, 0.2 s álló helyzetből):

| minta (MPC-sorrend FL, FR, RL, RR) | Isaac | MPC-modell | arány |
|---|---|---|---|
| előre (1, 1, 1, 1) | Δv_x = 0.091 m/s | 0.119 m/s | 76% |
| oldalra (−1, 1, 1, −1) = S_V | Δv_y = +0.077 m/s (balra) | 0.119 m/s | 65% |
| fordulás (−1, 1, −1, 1) = S_R | Δψ̇ = +0.97 rad/s | 1.89 rad/s | 51% |

Az előjelek és a kerékszámozás **helyesek** (a társ `S_V`, `S_R` konvenciója egyezik az Isaac-csuklókkal, minden
kerékelőjel +1). A nagyság eltér: ez **sebességfüggő veszteség a görgő–talaj kontaktusban** (0.1 s alatt még
89 / 83 / 59%, fél nyomatékkal 0.4 s alatt 67 / 44 / 33%; a PhysX-iterációk számától nem függ). Nagysága
~0.002 Nm/kerék, **nagyobb, mint a referencianyomaték** (≤ 0.0015 Nm a körön). Ez az MPC modellhibája (a terv 2.3
szerint nem javítottam előre).

**1. javítás: motormodell.** Az Isaac `DCMotor` a terhelés nélküli fordulat (8.06 rad/s) fölött a parancstól
függetlenül ±1.95 Nm-rel fékez, és a görbe meredeksége (0.37 Nm·s/rad) a könnyű kerékkel (J1 = 1.85·10⁻⁵ kg·m²,
armature 0) ~56× az explicit stabilitási határ fölött van. A 30 cm-es kezdeti hibánál az MPC ~0.08 Nm-et kért a
levegőben induló kerekekre: a kerekek ±160 rad/s-mal oszcilláltak, és **mind a 4 pálya elveszett**. Az `XM430Motor`
a görbét csak korlátként használja (|τ| ≤ 3.0 (1 − |ω|/8.06) a mozgás irányába, ≤ 1.95), és sosem fordítja meg a
parancsot. A javítás után egy pálya sem veszett el.

**2. javítás: reset-magasság 5 → 1 mm**, hogy reset után ne essen ~30 ms-ig a robot.

### 7.4 Az MPC az Isaac-envben (DARE, S = 100·I, puha ω-korlát; 2026-10-23)

| készlet | szabályozó | pozíció RMS / max [mm] | pályától RMS [mm] | lag átlag [mm] | heading RMS / max [°] | nyomaték RMS [Nm] | ‖Δu‖ max [Nm] | ω max [rad/s] (> 8.06) | számítás [ms] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|
| test_fixed (4) | MPC | 4.15 / 6.2 | 0.89 | +3.84 | 0.11 / 0.7 | 0.0050 | 0.0122 | 5.5 (0.0%) | 9.37 | 0 |
| test_tangent (4) | MPC | 2.69 / 5.9 | 0.25 | +2.63 | 0.71 / 2.8 | 0.0035 | 0.0228 | 7.3 (0.0%) | 9.62 | 0 |
| test_fixed + 30 cm/30° (4) | MPC | 31.82 / 300.0 | 4.23 | -1.20 | 4.09 / 30.3 | 0.0055 | 0.1019 | 11.5 (0.9%) | 8.63 | 0 |
| test_tangent + 30 cm/30° (4) | MPC | 31.63 / 300.0 | 4.19 | -2.36 | 4.14 / 30.4 | 0.0044 | 0.1019 | 12.6 (0.9%) | 9.48 | 0 |
| test_unseen_own (20) | MPC | 3.68 / 6.4 | 0.57 | +3.56 | 0.18 / 1.9 | 0.0044 | 0.0121 | 7.4 (0.0%) | 10.17 | 0 |

Alakzatonkénti átlag (max: a legrosszabb futás); `tools/metrics.py`, ablak: a referencia mozgásának végéig; a logok:
`logs/eval/mpc/<készlet>/`. A 30 cm/30°-os sorokban a max (300 mm) maga a kezdeti hiba.
- A hiba **szinte csak lemaradás** (lag +3.8 mm fixed, +2.6 mm tangent headingnél; a pályától mért távolság 0.3–0.9 mm):
  az MPC kerekenként ~0.005 Nm-et ad a 0.0014 Nm-es τ_r helyett, ez pótolja a görgőkontaktus veszteségét, de mivel nincs
  súrlódásmodellje és integrátora, csak a lemaradásból tudja előállítani. Nagyságrendben egyezik a társ Gazebo-eredményével
  (v1: 1.7–3.6 mm).
- Lépésidő ~9 ms (CPU, egy env, DARE-vel), a 20 Hz-hez bőven elég. Nem látott pályák: mind a 20 végigment.

### 7.5 Első RL-próbák (150 iteráció, seed 0 — csak a lánc ellenőrzése, nem eredmény)

| készlet | szabályozó | pozíció RMS / max [mm] | pályától RMS [mm] | lag átlag [mm] | heading RMS / max [°] | nyomaték RMS [Nm] | ‖Δu‖ max [Nm] | ω max [rad/s] (> 8.06) | számítás [ms] | elveszett |
|---|---|---|---|---|---|---|---|---|---|---|
| test_fixed (4) | MPC | 4.15 / 6.2 | 0.89 | +3.84 | 0.11 / 0.7 | 0.0050 | 0.0122 | 5.5 (0.0%) | 9.37 | 0 |
| test_fixed (4) | RL 150 it, torque_rate −1 | 2.90 / 6.2 | 1.60 | -2.04 | 1.32 / 3.3 | 0.0243 | 0.1521 | 13.3 (66.8%) | 0.09 | 0 |
| test_fixed (4) | RL 150 it, −20 + wheel_speed | 5.03 / 9.8 | 3.36 | -1.25 | 1.18 / 2.1 | 0.0063 | 0.0059 | 5.7 (0.0%) | 0.06 | 0 |
| test_tangent (4) | MPC | 2.69 / 5.9 | 0.25 | +2.63 | 0.71 / 2.8 | 0.0035 | 0.0228 | 7.3 (0.0%) | 9.62 | 0 |
| test_tangent (4) | RL 150 it, torque_rate −1 | 2.45 / 15.2 | 1.01 | -0.37 | 1.81 / 6.1 | 0.0192 | 0.1536 | 14.7 (44.3%) | 0.09 | 0 |
| test_tangent (4) | RL 150 it, −20 + wheel_speed | 3.48 / 16.5 | 2.62 | +0.88 | 1.46 / 4.5 | 0.0106 | 0.0255 | 11.1 (0.4%) | 0.07 | 0 |

Számítás: a policy TorchScript-ben, CPU-n, egyetlen megfigyelésre (a GPU-s inference-szel 4·10⁻⁸-on belül azonos kimenet).
- Az első próba (`torque_rate` −1) pozícióban már az MPC szintjén van, és nem marad le (lag < 0), de **rángat**:
  5× nyomaték-RMS, 0.15 Nm-es ugrások, a kerekek a lépések kétharmadában 8.06 rad/s fölött (csúsznak/oszcillálnak).
- `torque_rate` −20 + `wheel_speed` −0.1 után a nyomaték az MPC-éhez hasonló (‖Δu‖ max kisebb is), túllépés alig, a
  pozícióhiba 150 iteráció után még ~5 mm fixed headingnél: ehhez kell a teljes tanítás (1500 iteráció, több seed).

### 7.6 Ami hátravan

1. **Fő heading-mód** egyeztetése a társsal (a policy mindkettőt tanulja, a kiértékelés mindkettőn fut).
2. **Teljes tanítás** több seeddel (2048 env-vel ~11 s/iteráció; 1500 iteráció ≈ 4.5 óra/seed), utána a közös
   kiértékelés (README, „Összehasonlítás az LTV-MPC-vel”).
3. Abláció: τ_r a megfigyelésben (`ref_preview` `n_extra=4`, curriculum nélkül); twist-akció (`-Own-` taskok).
4. Az MPC hangolása (Q, R, S) a **tanítópályákon**, ha a társ szeretné; zavarbecslő / integrátor csak külön sorként.
5. A görgőkontaktus veszteségének azonosítása (a valódi robottal összevetve), és egy armature-érték a motorhoz
   (sysID; ekkor az MPC modelljébe is be kell tenni: J1 += armature).

