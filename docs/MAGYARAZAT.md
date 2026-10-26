# Mecanum RL pályakövetés: amit eddig átbeszéltünk, és miért ilyen a kód

Állapot: 2026-10-15. Kész: 1. node (Isaac Lab managerek), 2. node (referencia-generátor), 3. node (megfigyelés), 4. node (orientáció: szabad), 5. node (jutalom + termináció), 6. node (tanító pályák, curriculum, DR), 7. node (kiértékelés + kísérlettervezés). **Mind a 7 node kész.**
Kapcsolódó fájlok (ebben a mappában): `dontesek.md` (döntésnapló forrásokkal), `irodalom.md` (cikklista), `kesobbi_kerdesek.md` (nyitott kérdések). Az építés lépései: `../README_EPITES.md`.

---

## 0. A projekt célja és kerete

- **Feladat:** RL-alapú (PPO, RSL-RL) **trajektóriakövetés** (idő-indexelt referencia) mecanum robotra Isaac Labben, és összehasonlítás egy **LTV-MPC**-vel. Az MPC-t a társad írja, a te részed csak az RL.
- **Kimenet:** TDK-dolgozat, a határidő kb. **2026-11-01**.
- **Az MPC dinamikus modellt használ:** a referencia a pontokból és a sebességekből áll, a bemenet a 4 kerék nyomatéka.
- **Tesztpályák** (ezeken hangolják az MPC-t): négyzet, lekerekített sarkú négyzet, kör, nyolcas.
- **Saját hozzájárulás:** nem találtunk olyan cikket, amely mecanum robotot PPO-val követtet pályán **és** MPC-vel hasonlít össze (lásd `irodalom.md`).

---

## 1. Az RL-alapok, amikre építünk

Ezeket a felmérésben ellenőriztük, itt csak a dolgozatba kellő állítások és a forrásaik szerepelnek.

| Állítás | Forrás |
|---|---|
| PPO: clipped surrogate objektív, a kritikus $V(s)$ az előnybecsléshez | Schulman et al. 2017, arXiv:1707.06347; GAE: Schulman et al. 2016, arXiv:1506.02438 |
| A policy a **hozamot** maximalizálja, nem a pillanatnyi jutalmat: $G_t = \sum_k \gamma^k r_{t+k}$ | PPO / GAE (fent) |
| $\gamma = 0.99$ mellett az effektív horizont kb. $1/(1-\gamma) = 100$ lépés, azaz kb. 2 s (51 Hz-en) | standard RL-összefüggés |
| A követéshez **sűrű** (lépésenkénti) jutalom kell, nem csak a végén adott | Rudin et al. 2021 (velocity tracking reward) |

### Miért exp-kernel a jutalom?

$$
r = \exp\left(-\frac{\lVert e \rVert^2}{\sigma^2}\right) \in (0, 1]
$$

- **Korlátos és pozitív.** Minden túlélt lépés ér valamit. A $-\lVert e\rVert^2$ mindig negatív, és ha a túl nagy hiba **korai terminációt** okoz, akkor a gyors kisodródás levágja a további negatív jutalmakat, vagyis „megéri” az ágensnek véget vetni az epizódnak.
  Forrás: a korai termináció működése DeepMimic, 6.2 fejezet. Hogy negatív jutalom mellett a leállás válik optimálissá, arra jelölt forrás a DERAIL (arXiv:2012.01365, EarlyTerm feladatok) **[nincs ellenőrizve]**. Ha nem igazolható, saját ablációval mutasd meg.
- **A $\sigma$ az érzékenység.** Túl kicsi $\sigma$ esetén (pl. 0.02 m, miközben a kezdeti hiba 0.3 m) a kernel mindenhol $\approx 0$, a jutalom gyakorlatilag ritka, és a tanulás elakad. Ezért a $\sigma$ a tipikus hiba nagyságrendjébe kerül, vagy két kernel kell (egy durva és egy finom), vagy curriculum, amely menet közben szűkíti a $\sigma$-t.
- Forrás: Rudin et al. 2021, A.3 függelék, 2. táblázat: $\exp(-\lVert e\rVert^2/0.25)$, azaz $\sigma = 0.5$. DeepMimic (Peng et al. 2018) 5.3 fejezet: több exp-tag súlyozott összege.

### Miért legyen a sebesség is referencia (és jutalomtag)?

Elvben redundáns, hiszen $v_{ref} = \dot p_{ref}$. Mégis három okból kell:
1. **Csillapítás (D-tag):** csak pozícióhibával a szabályozó olyan, mint egy P-tag a pozícióra, így túllő és leng. A sebességhiba büntetése csillapít.
2. **Előrecsatolás:** a dinamikus MPC a $\tau_{ref}$-et inverz dinamikából számolja, ehhez kell a $v_{ref}$ (és a $\dot v_{ref}$). Az LTV-linearizálás is a $(x_{ref}, u_{ref})$ körül történik.
3. **Azonnali tanuló jel az RL-nek:** ha a robot a jó pozícióban áll, de rossz sebességgel, akkor a pozíciós jutalom *most* maximális, és a büntetés csak a következő lépések pozícióhibájában jelenik meg. Ezt a késleltetett hatást az RL a hozamon és a kritikuson keresztül megtanulja, de lassabban és zajosabban. Egy sebességtag *azonnal* jelez.

Forrás: DeepMimic 5.3 fejezet (sebesség-exp-tag a póztag mellett). Abláció ajánlott: sebességtaggal vs. nélküle.

---

## 2. A korrekt összehasonlítás szabályai (a kísérlettervezés)

**Az alapelv, amiből minden szabály következik:**

> **Egy összehasonlító kísérletben a két futás között CSAK a vizsgált dolog különbözhet: a vezérlő. Minden más azonos.**

Ez a kontrollált kísérlet definíciója: ha két dolog is különbözik, a mért eltérést nem tudod egyiknek sem tulajdonítani. Az alábbi szabályok mind ennek az esetei:

| Ami azonos kell legyen | Ha nem az, honnan jöhet a különbség? |
|---|---|
| referenciapályák (közös teszthalmaz) | a feladat nehézségéből |
| kezdőállapotok, seedek, domain randomization értékei | a véletlenből |
| előre ismert információ ($K\Delta t = NT$) | a több információból |
| interfész (nyomaték vs. twist) | a belső szabályzó hurokból |
| állapot-minőség (zaj) | a bemenet minőségéből |
| fizikai korlátok (nyomaték- **és** keréksebesség-korlát) | abból, hogy az egyik többet „engedhet meg” magának |

**Kiértékelés ≠ tervezés.** Az alapelv a *kísérletre* (a kész vezérlők kiértékelésére) vonatkozik. A *tervezés* lehet eltérő, mert az a vezérlő része, vagyis pont az, amit összehasonlítasz: az MPC modellje és a $Q$, $R$, $N$ hangolása, illetve az RL tanítása (jutalom, zaj, domain randomization, tanító pályák). Az RL zajjal tanítása tehát rendben van, akkor is, ha az MPC-t zajmentes szimben hangolják. Két dolgot viszont érdemes megtenni:
1. A tervezési feltételeket mindkét vezérlőnél **dokumentáld** a dolgozatban.
2. Ablációként tanítsd meg az RL-t **zaj nélkül** is: a zajos tanítás simább, óvatosabb policyt ad, ami a zajmentes teszten kicsit pontatlanabb lehet.

Lazább méltányossági szempont: hasonló hangolási erőfeszítés, és ugyanaz a szimulátor-hozzáférés mindkét oldalon.

A „mindkettő a saját körülményei között fut” (a *kiértékelésnél*) csábító, mert valósághűnek hangzik. A valóságban azonban a két vezérlő **ugyanazt** a szenzort és motort kapná, tehát a valósághű teszt is azonos körülményeket jelent.

1. **Közös, rögzített teszthalmaz:** kör, nyolcas, négyzetek, valamint a tanításból **kihagyott** pályák, azonos kezdőállapotokkal és seedekkel. Ha a tanító pályákon tesztelnél, az tesztszivárgás lenne, az eltérő seed pedig zajt vinne be.
   → kód: a `traj_files` a tanításnál és a kiértékelésnél külön lista; kiértékeléskor `random_start=False`.
2. **Ugyanazt a referenciát** követik: ugyanaz a fájl, nem „ugyanaz a képlet”.
   → kód: táblázatos referencia, fájlból betöltve (lásd 3.2).
3. **Ugyanannyi előre ismert információ.** Az MPC $N$ lépésre előre látja az $x_{ref}$, $u_{ref}$ értékeket, $T$ időközzel. Fontos különbség, hogy a jövőbeli *bemeneteket* nem látja, hanem kiszámolja: azok a döntési változói.
   → kód: `preview_steps = N`, `preview_dt = T`, így az RL **pontosan ugyanazokat az időpontokat** látja előre.
4. **Ugyanaz az interfész (akciótér):** **NYITOTT.** Mivel az MPC nyomatékot ad ki, a twist + IK akciótér nem azonos vele: az RL kapna egy belső sebesség-PD hurkot, ami az MPC-nek nincs. Opciók: (a) az RL is nyomatékot ad (effort action, a kerék-actuator stiffness/damping = 0); (b) twist + IK, és az eltérést kimondod a dolgozatban. A 4 kerekes akciótér 1 dimenziós nulltere (a kerekek egymás ellen dolgoznak) mindkét esetben felesleges szabadság.
5. **Ha az RL nyer, miért?** Több ok keveredhet, és ezeket kísérletekkel kell szétválasztani:
   - **modellhiba:** az RL ugyanabban a szimben tanult, így a görgőcsúszást is megtanulta; az MPC modelljéből ez hiányzik;
   - **az MPC hangolása:** $Q$, $R$, $N$;
   - **az RL a szimhez illeszkedett:** a valódi robotra nem biztos, hogy átvihető;
   - **jobb célfüggvény:** az RL a feladatszintű célt optimalizálja, az MPC-t pedig a tervezés–szabályozás szétbontása korlátozza (Song et al. 2023, Science Robotics, doi:10.1126/scirobotics.adg1462).

   Szétválasztó kísérletek: csúszásmentes környezet vagy identifikált modellű MPC (ha a rés eltűnik, a modell volt az ok); paramétereltolás (súrlódás vagy tömeg a domain randomization tartományán kívül) a robusztussághoz.
6. **Az éles sarkú négyzet:** a sarokban a referenciasebesség *vektora* ugrik, ehhez végtelen gyorsulás, azaz végtelen nyomaték kellene. **Egyik vezérlő sem** tudja ott pontosan követni a referenciát. A holonomitás ezen nem segít, mert az kinematikai tulajdonság, nem dinamikai. Ez a referencia hibája, nem a vezérlőé. A társaddal döntsétek el, melyik legyen: lekerekített sarkok, a sarkok kizárása a metrikából, vagy időparaméterezés megállással a sarokban.

---

## 3. Isaac Lab: hogyan illeszkedik a kód (1. node)

### 3.1 Alapigazság: minden term batch

> **MINDEN Isaac Lab term egyszerre MINDEN környezetet kezel: bemenete a teljes `env`, kimenete egy tenzor, amelynek első dimenziója `num_envs`.**

Forrás: masszívan párhuzamos GPU-s tanítás, Rudin et al. 2021, CoRL, arXiv:2109.11978. Isaac Lab: Mittal et al. 2025, arXiv:2511.04831.

- **Reward-term:** `(num_envs,)` tenzort ad vissza, környezetenként egy skalárt. (`(num_envs, 1)` **hibás**: a broadcast miatt `(num_envs, num_envs)` lenne belőle.)
- **RewardManager** (`reward_manager.py`, ellenőrizve):

  ```python
  value = term_cfg.func(self._env, **term_cfg.params) * term_cfg.weight * dt
  ```

  A súlyt **a lépésidővel is szorozza** (a Rudin-cikk 2. táblázatában is így szerepel: súly $\cdot dt$). A 0 súlyú termet meg sem hívja.
- **ObservationManager:** a termeket összefűzi, így lesz `(num_envs, obs_dim)`.

### 3.2 A CommandTerm hívási sorrendje

A `command_manager.py` szerint (ellenőrizve) minden env-lépésben:
1. `_update_metrics()`
2. `time_left -= dt`
3. ahol `time_left <= 0`: `_resample(env_ids)` → **`_resample_command(env_ids)`**, és új `time_left` a `resampling_time_range`-ből
4. **`_update_command()`**, minden env-re

Resetkor is meghívódik a `_resample(env_ids)`.

**Következmények a kódra:**
- A `t` léptetése az `_update_command`-ba kerül, mert az minden lépésben fut.
- Az új pálya sorsolása és a `t = 0` a `_resample_command`-ba kerül.
- `resampling_time_range=(1e9, 1e9)`: **csak a reset** sorsoljon, különben a pálya epizód közben ugrana. Ha pontosan az epizódhosszra állítanád, a `time_left` egy lépéssel a reset előtt lejárhatna. Ugyanez a trükk szerepel a navigációs configodban is (`navigation_env_cfg.py:158`).
- A `_resample_command(env_ids)` **csak** az `env_ids` sorait írja. Ide kerülnek a lejárt `time_left`-ű **és a resetelt** env-ek. Pályakövetésnél a leggyakoribb ok a korai termináció (túl nagy hiba).

---

## 4. A `TrajectoryCommand`: miért ilyen (2. node)

Fájlok: `mecanum_tracking/mdp/trajectory_command.py` (Isaac Lab-függő), `mecanum_tracking/mdp/traj_utils.py` (tiszta PyTorch, tesztelve: `tests/test_traj_utils.py`), `tools/gen_reference_trajectories.py` (tesztpályák, amíg a társad fájljai meg nem jönnek), `mecanum_tracking/tracking_env_cfg.py` (a config).

### 4.1 Táblázat, nem képletek

A pályák egy GPU-n lévő `(N_pálya, T_minta, C)` tenzorban vannak; minden env-nek van egy `traj_idx`-e és egy `t` ideje.

Miért:
- **Ugyanaz a fájl, mint az MPC-nél.** Így garantáltan ugyanazt a referenciát követik (2. szabály).
- **Bármilyen alakzat** működik (négyzet, spline, a társad bármilyen pályája), alakzatonkénti képlet nélkül.
- **Egyszerű vektorizálás:** a kiolvasás egy indexelés a `[traj_idx, idő_idx]` páron, alakzatonkénti `if` nélkül (3.1).

Az eltérő hosszú pályákat a `load_trajectories` az utolsó sorral **paddolja**, és a valódi hosszt a `traj_len` tárolja.

Oszlopok: `t, X, Y, theta, vx, vy, omega` (+ opcionálisan `tau_1..4`, ha a társad megadja a referencia-nyomatékot).

### 4.2 Interpoláció

A lekérdezett $t + i\Delta t$ időpont általában **két minta közé** esik. Ilyenkor nem lenne egyértelmű, mihez mérjük a hibát. Ezért `sample_table` **lineárisan interpolál**:

$$
s = \frac{t_q}{\Delta t_{tábla}}, \qquad i_0 = \lfloor s \rfloor, \qquad w = s - i_0, \qquad x = x_{i_0} + w \left(x_{i_0+1} - x_{i_0}\right)
$$

- **A $\theta$-t nem szabad közvetlenül interpolálni.** $3.1 \to -3.1$ rad között a sima átlag 0 lenne, vagyis egy fél fordulat rossz irányba. Helyette a **wrap-elt különbséget** interpoláljuk: $\theta_0 + w \cdot \mathrm{wrap}(\theta_1 - \theta_0)$, ahol $\mathrm{wrap}(a) = \mathrm{atan2}(\sin a, \cos a)$. (A teszt: `test_heading_wraps_across_pi`.)
- **A pálya vége:** `loop=True` esetén zárt pálya, modulo a perióduson. Ekkor a fájl utolsó sora meg kell egyezzen az elsővel; a generátor így írja. `loop=False` esetén az utolsó mintán marad, és a robotnak meg kell állnia.
- **Az óra float64-ben van** (`TrajectoryCommand.t`, a lekérdezési időpontok és a mintaindex). float32-ben a lépésenkénti `t += 0.05·s` kerekítési hibája összeadódik: egy 200 s-os körön +7,6 ms (mérve), vagyis a lekérdezés ~0,8 mintával lecsúszik a rácsról, és 0,1 m/s mellett a referencia ~0,8 mm-rel elcsúszik a pálya mentén. Ugyanígy a $\Delta t_{tábla}$ is a float64 időoszlopból jön (float32-ben 0,0099999998). A sorok maguk float32-ben maradnak. (A teszt: `test_long_trajectory_clock_stays_on_samples`.)

### 4.3 Előretekintés: $\Delta t = T$, $K = N$

A `command` az aktuális pont és $K$ előretekintő pont: `preview_w`, alakja `(num_envs, K+1, C)`, kilapítva `(num_envs, (K+1)·C)`.

A felvetésed az volt, hogy az RL 51 Hz-en sűrűbben látná a pályát, mint az MPC. A megoldás: **a $\Delta t$ szabad tervezési paraméter**, nem kell a szimulátor lépésidejének lennie. $\Delta t = T$ és $K = N$ mellett a két vezérlő **pontosan ugyanazokat** az időpontokat látja. Az interpoláció miatt ez bármely $t$-re kiértékelhető, akkor is, ha a $t$ nem esik az MPC rácsára.

A `command` **világkoordinátában** van. A test-koordinátába transzformálás a megfigyelés-term dolga (3. node). Így a reward-termek a világbeli referenciát használhatják, a policy pedig a robothoz viszonyított értékeket kapja.

### 4.4 Anchor: a pálya a robothoz igazodik

Minden pálya a saját koordinátarendszerében, a $(0, 0)$ pontból indul. Resetkor a `_resample_command` kiszámol egy **anchort** (világbeli $x, y, \text{yaw}$), amivel a pálya kezdőpontja **pontosan a robotra** kerül:

$$
\psi_a = \psi_{robot} - \theta_{pálya}(t_0), \qquad p_a = p_{robot} - R(\psi_a)\, p_{pálya}(t_0)
$$

Miért:
- A kezdő hiba nulla, így a policy nem egy ugrásszerű hibával kezd.
- Nem kell a reset-események sorrendjével bajlódni (a robot bárhol lehet a reset után).
- `random_start=True` esetén ($t_0$ véletlen a pályán) is működik, mert a képlet a $t_0$-beli pontot illeszti a robotra. Ez tanításnál több változatosságot ad, kiértékelésnél kikapcsolod.

A sebességek ($v_x, v_y$) **test-koordinátában** vannak, mint az MPC $u = (v_x, v_y, \omega)$ bemenete, ezért az anchor elforgatása nem változtat rajtuk (`velocity_in_traj_frame=False`). **Kérdezd meg a társadat**, hogy ő is így adja-e meg; ha világkoordinátában, állítsd `True`-ra.

### 4.5 A tesztpálya-generátor

`tools/gen_reference_trajectories.py` négy zárt tesztpályát állít elő 0.5 m/s állandó pályasebességgel: kör, nyolcas, lekerekített négyzet, éles négyzet. Az irány (heading) lehet konstans (a mecanum oldalaz) vagy érintő irányú (`--heading tangent`).

- A lépésszám kerekítése miatt a sebesség pontosan nem 0.5 m/s, de az eltérés fél lépésnyinél kisebb a teljes pályán. Így a záró lépés ugyanolyan hosszú, mint a többi, és a modulo-ugrásnál nincs sebességhiba.
- A sebességet a generátor középponti differenciával számolja. Az **éles négyzet sarkaiban** ez 0.37 m/s-ot ad: ez a sebességugrás „elkenődése” egy mintára, és pontosan a 2. fejezet 6. pontjában leírt jelenség (a sebességvektor ott nem folytonos).
- Ezek csak **tesztpályák**. A tanító pályák (véletlen görbék, a teszthalmaztól külön) a 6. node-ban jönnek.

### 4.6 Metrikák és debug-vis

- `_update_metrics`: `pos_error` (pozícióhiba-norma) és `heading_error` a TensorBoardba.
- `debug_vis`: piros gömb az aktuális referenciapontra, kék kis gömbök az előretekintő pontokra.

---

## 5. A megfigyelés: miért ilyen (3. node)

Fájlok: `mecanum_tracking/mdp/observations.py` (`ref_preview_b`), `mecanum_tracking/mdp/traj_utils.py` (`preview_to_body`, tesztelve), `mecanum_tracking/tracking_env_cfg.py` (`ObservationsCfg`, `TerminationsCfg`).

A policy csak azt tudja, ami a megfigyelés-vektorban van. A `TrajectoryCommand` világkoordinátában ad referenciát; ezt alakítjuk át a robothoz viszonyítva.

### 5.1 Mi kerül bele

| Elem (pontonként, $i = 0..K$) | Miért | Forrás |
|---|---|---|
| $\Delta x_b, \Delta y_b$: a referenciapont helye a robot test-koordinátájában | A feladat invariáns a világ eltolására és elforgatására. Test-koordinátában két ilyen helyzet **ugyanúgy néz ki**, így a policynak nem kell külön megtanulnia (`test_preview_to_body_invariance`). | Kaufmann et al. 2023 (Nature): a kapusarkok „a járműhöz képest” [részben ellenőrizve; a koordináta-rendszert a cikk nem nevezi meg] |
| $\sin\Delta\theta_i, \cos\Delta\theta_i$ | A wrap-elt $\Delta\theta$ a $\pm\pi$-nél **szakad**: 179° és −179° majdnem azonos helyzet, a szám mégis $+3.12$ és $-3.12$. A háló folytonos függvény, ezért ezt nehezen tanulja. Az egységkörön ábrázolva nincs szakadás. | Zhou et al. 2019, CVPR, arXiv:1812.07035, 3. fejezet (a 2D-s eset) [ellenőrizve] |
| $v_{ref}$ ($v_x, v_y$) a **robot** test-koordinátájába forgatva, és $\omega_{ref}$ | Előrecsatolás, ugyanaz az információ, amit az MPC ismer. A fájlban a referencia-póz saját test-koordinátájában van, ezért $\Delta\theta$-val forgatjuk. | alapelv (2. fejezet) |
| $\tau_{ref}$ (ha a társ fájljában van; `n_extra=4`) | Az MPC ismeri, tehát az RL-nek is látnia kell. | alapelv |

Plusz a robot saját állapota:

| Elem | Miért |
|---|---|
| saját sebesség ($v_x, v_y, \omega$, `base_lin_vel`, `base_ang_vel`) és keréksebességek | Dinamikus feladat: a gyorsításhoz és a fékezéshez a jelenlegi sebesség kell. Az MPC 6 állapota (póz + sebesség) így teljesen benne van: a póz a hibákban, a sebesség itt. |
| utolsó akció | simaság, illetve nyomaték-akciónál a dinamika része |

### 5.2 Mi NEM kerül bele: a hátralévő idő

A navigációs taskodban van egy `time_left`, a pályakövetésben **nincs**. A **döntő teszt**:

> **Változna-e a legjobb akció, ha a robot tudná, hogy 1 s múlva vége az epizódnak?**

- **Navigáció: igen.** Ha a cél messze van, kockáztatni kell, mert a jutalom a célba érésért jár. Ez **időkorlátos** feladat, ezért a `time_left` a megfigyelésben van, és a timeoutnál nincs bootstrap (`time_out=False`). A navigációs configodban ez **helyes**.
- **Pályakövetés: nem.** A legjobb akció mindig a követés. A 20 s-os timeout csak a tanítás technikai része, hogy a környezetek újrainduljanak. Ez **időkorlát nélküli** feladat, ezért nincs idő a megfigyelésben, és `time_out=True`, így az RSL-RL a timeoutnál a kritikus becslésével folytatja a hozamot ($r_T + \gamma V(s_{T+1})$).
- Nem zárt pályánál (`loop=False`) is időkorlát nélküli marad. A pálya végét a policy az **előretekintő pontokból** látja (a végén azok egy pontba „csúsznak össze”), nem egy órából.

**Ezt tehát nem szabad változatlanul átmásolni a navigációs taskból.**

Források:
- Pardo et al. 2018, „Time Limits in RL”, ICML, arXiv:1712.00378: a 2. fejezet szerint időkorlátos feladatnál az idő a megfigyelésbe kerül, a 3. fejezet szerint időkorlát nélkülinél bootstrap kell a timeoutnál [ellenőrizve].
- RSL-RL `ppo.py`: `rewards += gamma * values * time_outs` [ellenőrizve a forrásban]. Schwarke et al. 2025, arXiv:2509.10771.

### 5.3 Normalizálás és zaj

- **Normalizálás:** a megfigyelés elemei nagyon különböző skálájúak (m, rad, m/s, $\sin$). A futó átlaggal és szórással végzett normalizálás PPO-nál a legtöbb környezetben kritikus. Forrás: Andrychowicz et al. 2021, ICLR, arXiv:2006.05990, 3.3 fejezet [ellenőrizve]. A configodban **már be van kapcsolva** (`actor_obs_normalization=True`, `critic_obs_normalization=True`).
- **Zaj:** tanításnál be van kapcsolva (`enable_corruption=True`, `Unoise`), ez a robusztusságot segíti. A **fő kiértékelésnél ki** kell kapcsolni (`enable_corruption=False`), mert az MPC is a pontos állapotot kapja (alapelv). Ha zajjal is összehasonlítod őket, ugyanazt a zajmodellt kell a közös kiértékelő keretben **mindkét** vezérlő állapotára rátenni.

---

## 6. Az orientáció: szabad (4. node)

**Döntés (2026-10-15):** a referencia csak $X, Y$ és ezek deriváltjai; **nincs $\theta_{ref}$** egyik vezérlőnél sem. Az orientációt és a forgást mindkét vezérlő maga optimalizálja. Ez az alapelvnek megfelel, mert mindkettő ugyanazt a feladatot kapja.

### 6.1 Mit jelent ez

- A mecanum **holonom**: a haladási irány és a test orientációja független, ezért a $	heta$ külön szabadsági fok.
- $	heta_{ref}$ nélkül a feladat ebben a szabadsági fokban **alulhatározott**: bármely orientáció jó, csak a pozíció számít.
- A policy (és az MPC) a saját előnyére választhat orientációt. Például előre haladhat oldalazás helyett, mert oldalazásnál nagyobb a görgőcsúszás és a követési hiba. Forrás: Kim, Lee, Joo 2020, J. Korean Soc. Precision Eng. 37(1):35–42, doi:10.7736/JKSPE.019.082 [csak az összefoglaló ellenőrizve]. Ha az RL ezt megtanulja, az a Song et al. 2023 értelmében „jobb célfüggvény” előny. **Érdekes, riportolható eredmény.**
- Kockázat: céltalan forgás. Ellene: az MPC-ben az $\omega$ büntetése az $R$-ben, az RL-ben egy akció-simasági tag (5. node).

### 6.2 Amit közben megértettünk az LTV-MPC-ről (a dolgozat MPC-fejezetéhez is)

- **Miért QP az LTV, ha a $B_k$-ban $\sin$ és $\cos$ van?** Mert az $A_k$, $B_k$ a **referencia mentén**, $(x_{ref,k}, u_{ref,k})$-nál van kiértékelve, és ez az optimalizálás *előtt* ismert. Így előre kiszámolt **számmátrixok** lesznek. A döntési változók lineárisan szerepelnek, a költség kvadratikus, és ez a QP. A nemlineáris MPC-ben ezzel szemben a $\sin\theta_k$ az *ismeretlen* jövőbeli $\theta_k$-é, ezért nemkonvex NLP.
- **A költség ≠ a dinamika.** A $\theta$ súlyának nullázása a $Q$-ban csak a *célt* változtatja. A nemlinearitás ($\cos\theta$, $\sin\theta$ a bemenet forgatásában) a *dinamikában* marad.
- **A linearizálás csak akkor pontos, ha a valódi állapot közel marad a linearizálási ponthoz.** Ha a $\theta$-t semmi nem tartja a $\theta_{ref}$ közelében, és a referencia körül linearizálunk, a $B_k$ rossz lesz. (Példa: a robot 90°-ra fordul, az MPC $v_x$-et ad $+X$ felé, a robot pedig $+Y$ felé megy.)
- **Ha konstans $\theta$-t követnénk**, a $\cos\theta_0$, $\sin\theta_0$ konstans lenne, és a pozíció-dinamika lineáris (LTI).
- **Szabad $\theta$ esetén a helyes linearizálási pont:** az **előző lépés MPC-megoldásának eltolt (shiftelt) jósolt trajektóriája**, nem a referencia. Így a linearizálási pont követi a tényleges $\theta$-t. Ez a real-time iteration (RTI) eljárás. A referencia körüli LTV ennek speciális esete: ha a referencia a kezdeti becslés, az első SQP-lépés megegyezik a lineáris MPC megoldásával.
  Forrás: Gros, Zanon, Quirynen, Bemporad, Diehl, „From linear to nonlinear MPC: bridging the gap via the real-time iteration”, Int. J. Control 93(1):62–80, 2020, doi:10.1080/00207179.2016.1222553. A shiftelés a 3.2 fejezetben van ((19) egyenlet), a 2. és 3. algoritmus az eltolt becslésnél értékeli ki az $A$, $B$ érzékenységeket, a 4.3 fejezet pedig a lineáris MPC kiterjesztéséről szól RTI-vel. **[ellenőrizve: a szöveget kiolvastuk]**. Az eredeti RTI: Diehl, Bock, Schlöder 2005, SIAM J. Control Optim. 43(5):1714–1736, doi:10.1137/S0363012902400713 [csak bibliográfiai adat].
- A $\tau_{ref}$ előrecsatoláshoz (inverz dinamika test-koordinátában) is $\theta$ kell. Ezt is a jósolt trajektóriából kell venni.

### 6.3 Az RL-oldal (kód)

- A referenciafájlokban a **$\theta$ oszlop konstans 0** (generátor: `--heading constant`, ez az alapérték), a $v_x, v_y$ a pálya keretében van. Ezzel a meglévő kód **változtatás nélkül** helyes sebesség-referenciát ad a robot keretében, bármerre néz a robot. Teszt: `test_heading_free_reference_velocity`.
- Megfigyelés: `ref_preview_b(..., include_heading=False)`. A $\sin\Delta\theta$, $\cos\Delta\theta$ kimarad, mert nincs követendő orientáció. A robot pályához viszonyított elfordulása így is látszik az előretekintő pontok test-koordinátás helyzetéből.
- Jutalom (5. node): **nincs** szöghiba-tag.
- Metrika: csak pozícióhiba. Az orientációt *leíró* adatként riportáld (pl. mennyit forog, és mennyit oldalaz az egyik és a másik vezérlő).
- Megjegyzés: az anchor resetkor a pályát a robot kezdő yaw-jával forgatja el. Tanításnál ez csak változatosságot ad. Kiértékelésnél a két vezérlő azonos kezdőállapotból indul, így azonos pályát kap.

---

## 7. A jutalom és a termináció (5. node)

Fájlok: `mecanum_tracking/mdp/rewards.py`, `mecanum_tracking/mdp/terminations.py`, `mecanum_tracking/mdp/traj_utils.py` (`tracking_errors`, tesztelve), `mecanum_tracking/tracking_env_cfg.py` (`RewardsCfg`, `TerminationsCfg`).

### 7.1 A javasolt jutalom

$$
r = w_1 e^{-\lVert e_p\rVert^2/\sigma_1^2} + w_2 e^{-\lVert e_p\rVert^2/\sigma_2^2} + w_3 e^{-\lVert e_v\rVert^2/\sigma_v^2} - w_a \lVert a_t - a_{t-1}\rVert^2 - w_\omega \omega^2
$$

(Az Isaac Lab minden tagot még a $dt$-vel is szoroz, ez az arányokat nem változtatja.)

| Tag | Miért | Kezdő érték | Forrás |
|---|---|---|---|
| durva pozíciós kernel | „gyere vissza” jel nagy hibánál; túl kicsi $\sigma$ mellett a jutalom ritka lenne | $w_1 = 1$, $\sigma_1 = 0.5$ m | Rudin et al. 2021, 2. táblázat: $\exp(-\lVert e\rVert^2/0.25)$ [ellenőrizve] |
| finom pozíciós kernel | pontosság a pálya közelében | $w_2 = 1$, $\sigma_2 = 0.1$ m | több exp-tag súlyozott összege: DeepMimic 5.3 [ellenőrizve] |
| sebességkövetés (világkeretben) | csillapítás + azonnali jel (1. fejezet) | $w_3 = 0.5$, $\sigma_v = 0.3$ m/s | DeepMimic 5.3 [ellenőrizve] |
| akció-simaság | rángatás ellen | $w_a = 0.01$ | CAPS, Mysore et al. 2021 [ellenőrizve] |
| forgás-büntetés ($\omega^2$) | szabad orientáció mellett a céltalan forgás ellen (6. fejezet) | $w_\omega = 0.01$ | tervezési döntés |
| szöghiba | **nincs**: szabad orientáció | — | — |

Ezek **kiinduló** értékek. A hangolásra a meglévő eszközeid valók (`tune_rewards.py`, `staged_penalties.py`). Tökéletes követésnél $r \approx 2.5$.

### 7.2 A legfontosabb szabály: a teljes jutalom maradjon pozitív

Az exp-kernel csak akkor véd az „öngyilkosság” ellen, ha a **teljes** lépésenkénti jutalom pozitív. Ha a büntetések (pl. egy rángatózó kezdeti policynál) a pozitív tagok fölé nőnek, a teljes $r$ negatív lesz, és a korai termináció megint „megéri”. Ezért:
- a büntetések a tipikus állapotokban maradjanak a pozitív tagok töredékei;
- figyeld a TensorBoardban **tagonként** az epizód-összegeket;
- a büntetéseket lehet fokozatosan bevezetni. Pont ezt csinálja a `staged_penalties.py`: egy csak pozitív jutalommal tanított policyra egyenként rakja vissza a büntetéseket.

### 7.3 A termináció

| Termináció | `time_out` | Miért |
|---|---|---|
| időkorlát (20 s) | `True` → bootstrap | időkorlát nélküli feladat mesterséges csonkolása (5.2, Pardo et al. 2018) |
| eltérés: $\lVert e_p\rVert > 1$ m | `False` → nincs bootstrap | valódi kudarc: a robot elvesztette a pályát |

**Vigyázz, a `time_out` jelzés és a `time_left` megfigyelés két független beállítás.** A `time_left` azt dönti el, hogy a policy *látja-e* a hátralévő időt. A `time_out` egy adott terminációnál azt, hogy a PPO *bootstrapel-e*: a feladat valóban véget ért-e itt. A navigációs configodban a kettő együtt szerepelt (mindkettő a Pardo-féle „időkorlátos” esetből következett), a pályakövetésnél viszont két terminációd van, és mindkettőnek saját jelzése.

Miért nincs bootstrap az eltérésnél? Mert ott a robot **elveszíti** az összes jövőbeli pozitív jutalmat, és ez erős ösztönzés arra, hogy ne sodródjon el. `time_out=True` mellett a hozam $\gamma V(s_{T+1})$-gyel folytatódna, *mintha tovább követne*, így a kisodródásnak nem lenne következménye.

**A küszöb megválasztása:** legyen **jóval nagyobb** a kezdeti tipikus hibánál és a durva kernel $\sigma_1$-énél. Különben az epizódok néhány lépés után véget érnek, és a policy nem gyűjt tapasztalatot arról, hogyan jöjjön vissza; ez ugyanaz a hiba, mint a túl kicsi $\sigma$. Kezdő javaslat: 1 m $= 2\sigma_1$. Curriculummal később szűkíthető (6. node).

**Ha nyomaték lesz az akciótér:** jöhet egy $-\lVert\tau\rVert^2$ tag is (`joint_torques_l2`, nagyon kis súllyal).

---

## 8. Tanító pályák, curriculum, domain randomization (6. node)

Fájlok: `tools/gen_training_trajectories.py`, `mecanum_tracking/mdp/curriculums.py` (`speed_levels`), `mecanum_tracking/mdp/trajectory_command.py` (`time_scale`), `mecanum_tracking/mdp/traj_utils.py` (`scale_velocities`), `mecanum_tracking/tracking_env_cfg.py` (`CurriculumCfg`, `EventCfg`). Tesztek: `test_time_scale`, `test_training_trajectories_feasible`.

### 8.1 Tanító pályák: véletlen, de megvalósítható görbék

- **Nem tartalmazhatják a tesztalakzatokat** (kör, nyolcas, négyzetek), különben tesztszivárgás lenne (2. fejezet). Viszont **le kell fedniük ugyanazt a jellemző-tartományt**: görbület, sebesség, kanyarirány, egyenes szakasz, sebességváltás. Így a policy általánosít, nem magol.
- A generátor véletlen, sima, zárt görbéket készít: alacsony rendű Fourier-sor, a $k$-adik harmonikus amplitúdója $\sim 1/k^2$. Minden pálya az origóból indul, a $\theta$ oszlop 0 (szabad orientáció).
- **Megvalósíthatóság:** a centripetális gyorsulás $a_{lat} = v^2 \kappa$ (körön $v^2/R$). Ha ez nagyobb, mint amit a kerekek ki tudnak fejteni, a referencia **követhetetlen** (mint az éles sarok), és nincs mit tanulni belőle. Ezért a sebességprofil:
  - oldalirányban: $v \le \sqrt{a_{lat,max} / |\kappa|}$;
  - érintőirányban: $|dv/dt| \le a_{tan,max}$, egy előre és egy hátra menettel a $v^2$-en ($v_{i+1}^2 \le v_i^2 + 2 a_{tan} \Delta s$);
  - a $0.2$ m-nél szűkebb kanyarú görbéket (majdnem csúcs, ahol a sebesség $\approx 0$-ra esne) újrasorsolja.

  Ez a pálya menti időparaméterezés általános keretének speciális esete. Forrás: Lynch & Park, *Modern Robotics*, 9.4 fejezet (Time-Optimal Time Scaling); Bobrow, Dubowsky, Gibson 1985, IJRR 4(3):3–17, doi:10.1177/027836498500400301 [bibliográfiai adat]. Az $a = v^2\kappa$ elemi kinematika.
- **Numerikus ellenőrzés:** az ívhossz menti felbontás számít. 3000 ponttal az időbeli újramintavételezés után a gyorsulás kb. 25%-kal túllőtt, 12000 ponttal 4%-on belül marad (`test_training_trajectories_feasible`).
- Hasonló gyakorlat (véletlen tanító pályák, eltérő tesztpályák): Kiemel & Kröger 2022, IROS, arXiv:2203.01968 [csak az abstract].

### 8.2 Sebesség-curriculum: az időskála $s$

- **Mi az $s$:** a referencia órájának sebessége a szimulált időhöz képest, mint egy videó lejátszási sebessége. Lépésenként $t \mathrel{+}= s \cdot dt$. **Nem** mintavételezés. Ugyanaz a fájl, $s$-szeres sebességgel.
- **Hatása:** $v \to s v$, és $a_{lat} = (sv)^2\kappa = s^2 v^2 \kappa$, vagyis **négyzetes**. Kis változtatással nagy nehézségkülönbség állítható be.
- A kódban: az előretekintő pontok a *referencia-időben* $t + s \cdot i \cdot T$-nél vannak, így a *valós* időben továbbra is $T$-nként, ahogy az MPC látja őket. A $v_{ref}$ $s$-sel szorzódik. (A $\tau_{ref}$ **nem** skálázható így, mert a gyorsulás $s^2$-tel nő, a súrlódás viszont nem. Időskálázással ne használd.)
- **A curriculum** a te `terrain_levels_goal`-od mintájára készült: env-enként, a reset **előtt** hívódik.
  - Feljebb lép ($s \mathrel{+}= 0.1$): az epizód nem eltéréssel ért véget, és az átlagos hiba kisebb 0.1 m-nél.
  - Lejjebb lép: az epizód eltéréssel ért véget.
  - Tartomány: $s \in [0.5, 2]$, kezdetben $0.5$.
- Forrás: a curriculum learning alapötlete Bengio et al. 2009, ICML, doi:10.1145/1553374.1553380 [bibliográfiai adat]. Az env-enkénti, adaptív („game-inspired”) curriculum Rudin et al. 2021, 3.1 fejezet [ellenőrizve]: ott a robot akkor lép feljebb, ha átjut a terepe határán, és akkor lejjebb, ha a kért távolság felét sem teszi meg.
- További curriculum-lehetőségek, ha kellenek: a finom kernel $\sigma_2$-jének vagy az eltérés-küszöbnek a szűkítése, a büntetések fokozatos bevezetése (`staged_penalties.py`).

### 8.3 Domain randomization

- Súrlódás 0.5–1.1 (mint a navigációs taskban, mert a valódi görgő–padló súrlódás ismeretlen), tömeg ×0.8–1.2.
- Forrás: Tobin et al. 2017 (IROS, arXiv:1703.06907); Peng et al. 2018 (ICRA, arXiv:1710.06537) [ellenőrizve].
- **A DR és a curriculum tervezési eszköz** (a tanítás része), ezért megengedett, hogy az MPC-t névleges értékekkel hangolják (2. fejezet: kiértékelés ≠ tervezés). **A kiértékelésnél:**
  - fő eredmény: nincs DR és nincs curriculum, rögzített névleges súrlódás és tömeg, rögzített $s$ (pl. 1.0, vagy egy 0.5/1.0/1.5 sorozat) **mindkét** vezérlőre;
  - robusztusság: **a tanítási tartományon kívüli** értékek (pl. súrlódás 0.3, tömeg ×1.4), mindkét vezérlőre azonosan.

---

## 9. Kiértékelés és kísérlettervezés (7. node)

Fájlok: `tools/metrics.py` (tesztelve: `tests/test_metrics.py`), `scripts/eval_tracking.py` (Isaac Lab), `TrajectoryCommandCfg.fixed_assignment`, `Mecanum-Tracking-Eval-v0`.

### 9.1 Mit állítasz, és hányszor kell mérni

| Állítás | Mérés |
|---|---|
| „Az **RL mint módszer** jobb/rosszabb, mint az **MPC mint módszer**” (ez a tiéd) | **több tanítási seed**, átlag ± szórás és konfidencia-intervallum |
| „**Ez a betanított policy** jobb/rosszabb, mint ez az MPC” | elég egyetlen policy, de az állítás csak arra a szabályzóra érvényes |

Az RL *tanítása* véletlen folyamat (kezdeti súlyok, felfedezés, mintasorrend), ezért minden seed egy kicsit más szabályzót ad. Az MPC-nél a „seed” megfelelője a hangolás.
- Henderson et al. 2018, „Deep RL that Matters”, arXiv:1709.06560, „Random Seeds and Trials” [a tartalom ellenőrizve]: 10, csak a seedben különböző tanítást két 5-ös csoportra bontottak, és a két csoport átlaga **szignifikánsan** különbözött. Alapértelmezésük 5 seed, de konkrét számot nem ajánlanak.
- Agarwal et al. 2021, NeurIPS, arXiv:2108.13264 [ellenőrizve]: kevés futásnál intervallum-becslés kell (4.1: rétegzett bootstrap CI; 4.3: IQM). Kb. 10 futástól megbízható. **3–5 seednél a CI széles lesz, ezt őszintén le kell írni.**
- Javaslat a határidő miatt: **legalább 3, ha belefér 5 seed** a fő RL-konfigurációra; az ablációkra 1–3 seed (ezt jelöld).

### 9.2 Metrikák (ugyanaz a szkript mindkét vezérlőre)

A `tools/metrics.py` egy **közös log-formátumot** olvas (lásd a fájl fejlécét). **Ezt egyeztesd a társaddal**, hogy az MPC-futások ugyanilyen CSV-t írjanak. Így a metrikák garantáltan azonos módon számolódnak.

| Metrika | Mit mutat |
|---|---|
| `pos_rmse_m`, `pos_max_m` | az idő-indexelt pozícióhiba átlaga és legrosszabb értéke (a fő metrika) |
| `contour_rmse_m` | a pályára **merőleges** hiba: mennyire van *rajta* a vonalon |
| `lag_mean_m` | a pálya **menti** hiba (pozitív = lemarad): mennyire *tart lépést* |
| `action_rate_mean`, `torque_rms_Nm` | simaság, beavatkozási energia |
| `yaw_rate_abs_mean`, `strafe_share` | **a szabad orientáció kihasználása**: mennyit forog, és a haladás mekkora része oldalazás |
| `compute_ms_mean/p99` | számítási idő |
| `failed` | elvesztette-e a pályát |

**A kontúr- és lag-felbontás azért fontos**, mert ugyanaz az RMSE két nagyon különböző viselkedést takarhat: a lemaradás túl óvatos vezérlőre utal, az oldalirányú kilengés pontatlanra vagy rosszul csillapítottra. Forrás: Liniger, Domahidi, Morari 2015, Optim. Control Appl. Methods, arXiv:1711.07300, III-B2, (9) és (11) egyenlet; ott Lam et al. 2010 (CDC) definícióit használják [ellenőrizve].

**Számítási idő, korrekten:** a kiértékelő szkriptben mért idő a GPU-n, *összes env-re együtt* (batch) futó forward pass, ez nem összehasonlítható. A dolgozathoz az exportált `policy.pt`-t **egyetlen bemenetre, CPU-n, ugyanazon a gépen** mérd, ahol az MPC-megoldó fut (alapelv). A Song et al. 2023 cikkre itt csak a tanítási idő és a 100 Hz-es futás kapcsán hivatkozz, mert futásidő-összehasonlítást nem tartalmaz.

### 9.3 A kiértékelési protokoll (`scripts/eval_tracking.py`)

- Env-enként egy tesztpálya (`fixed_assignment=True`), egy kör.
- Nincs zaj, nincs DR (névleges súrlódás és tömeg), nincs curriculum, rögzített $s$.
- **Az MPC-futások pontosan ugyanezzel a beállítással** mennek, ugyanabban az Isaac-szimben.
- Két teszthalmaz (8. fejezet):
  - (a) standard alakzatok: kör, nyolcas, lekerekített és éles négyzet. Az MPC-t ezeken hangolták, ezt ki kell mondani. Az éles négyzetnél a sarkokat külön kezeld (2. fejezet, 6. pont).
  - (b) **egyik vezérlő által sem látott** véletlen görbék: `gen_training_trajectories.py --seed 999 --out refs/test_unseen`.

### 9.4 A kísérletmátrix (a dolgozat eredmény-fejezete)

| # | Kísérlet | Kérdés | Futások |
|---|---|---|---|
| E1 | **Fő eredmény:** RL vs. MPC, (a) és (b) teszthalmaz, $s = 1$ | ki követ jobban, és miben (kontúr vagy lag)? | RL: 3–5 seed; MPC: 1 |
| E2 | Sebesség-sorozat: $s = 0.5, 1.0, 1.5$ | hogyan romlik a követés a dinamikai határ felé ($a \sim s^2$)? | ugyanazok a modellek |
| E3 | **Robusztusság:** súrlódás és tömeg a tanítási tartományon *kívül* (pl. 0.3; ×1.4), mindkettőnek azonosan | ki robusztusabb a modellhibára? | ugyanazok |
| E4 | **Az okok szétválasztása:** csúszásmentes vagy ideálisabb szim (vagy identifikált modellű MPC) | a különbség a modellhibából jön-e (2. fejezet, 5. pont)? | ugyanazok |
| E5 | Szabad orientáció kihasználása (`yaw_rate_abs_mean`, `strafe_share`) | megtanulja-e az RL, hogy kevesebbet oldalazzon (irányfüggő csúszás, Kim et al. 2020)? | leíró, E1 adataiból |
| A1 | Abláció: RL zaj nélkül tanítva | mennyit ad vagy vesz el a zajos tanítás? | 1–3 seed |
| A2 | Abláció: sebességtag nélkül | kell-e a sebességtag (csillapítás, gyorsabb tanulás)? | 1–3 seed |
| A3 | Abláció: $-\lVert e\rVert^2$ az exp-kernel helyett (opcionális) | igazolja-e az „öngyilkossági” érvet (ehhez nincs ellenőrzött cikk, a saját kísérlet pótolja) | 1 seed |
| A4 | (Ha nyitva marad) akciótér: twist + IK vs. nyomaték | mennyit számít az interfész? | 1–3 seed |

**Prioritás a határidő miatt:** E1 → E3 → E2 → A1/A2 → E4, E5 (E5 ingyen jön az E1 adataiból) → A3, A4.

---

## 10. Amit a telepített Isaac Labedben ellenőrizned kell (`# CHECK` a kódban)

1. **`ManagerBasedRLEnv._reset_idx`:** a `command_manager.reset` a reset-eventek (`reset_root_state_uniform`) **után** fut-e. Az anchor erre épít. A navigációs kódod kommentje („already reset robot”) erre utal.
2. ~~`self._env.step_dt`~~: **igazolva**, a te `play.py`-od is használja (`env.unwrapped.step_dt`).
3. **`CommandTermCfg`:** a `class_type`, a `resampling_time_range` és a `debug_vis` mezőket a manager olvassa (ez ellenőrizve). Az `asset_name`-et mi adtuk hozzá, mint a beépített command-cfg-k.
4. **`self.metrics`:** a `CommandTerm.__init__` létrehozza-e dict-ként (a beépített `UniformPose2dCommand` így használja).
5. **`mecanum_tracking/mdp/__init__.py`:** exportálja-e a `TrajectoryCommandCfg`-t és a `ref_preview_b`-t (a mellékelt `mecanum_tracking/mdp/__init__.py` ezt teszi).
6. **`env.command_manager.get_term(name)`:** létezik-e (az `ref_preview_b` ezzel éri el a `preview_w`-t).
7. **`mdp.action_rate_l2`, `mdp.joint_torques_l2`:** a beépített `isaaclab.envs.mdp`-ből jönnek; ellenőrizd a nevüket.
8. **`mdp.randomize_rigid_body_mass`:** a paraméternevek (`mass_distribution_params`, `operation`) és a robot alvázának body-neve.
9. **`env.termination_manager.get_term("deviation")`:** a curriculum ezzel olvassa ki, mely env-ek vesztették el a pályát (a navigációs curriculumod ugyanígy használja).
10. **`scripts/eval_tracking.py`:** a `# CHECK` sorok (`parse_env_cfg`, `load_cfg_from_registry`, `applied_torque`, a kerékcsukló-nevek, egy manager kikapcsolása `None`-nal).
11. **`mdp.time_out` és `DoneTerm(..., time_out=True)`:** a navigációs configod ugyanezt a formát használja, csak `False`-szal.

**Gyors próba:** `num_envs=4`, play módban, `debug_vis=True`, a generált `circle.csv`-vel. A piros gömbnek a körön kell haladnia 0.5 m/s-mal, a kék pontoknak egyenletesen, $T$ távolságra kell követniük egymást. Korai termináció után csak az adott env kaphat új pályát.

---

## 11. Hogyan tovább (a terv)

| Node | Téma | Állapot |
|---|---|---|
| 1 | Isaac Lab managerek | kész |
| 2 | Referencia-generátor (ez a kód) | kész (tervezés + kód) |
| 3 | Megfigyelés: test-koordinátás preview, $(\sin, \cos)$, $v_{ref}$, ($\tau_{ref}$), saját állapot, nincs idő, zaj/normalizálás | kész (kód + 5. fejezet) |
| 4 | Orientáció: **szabad** (nincs $\theta_{ref}$) | kész (6. fejezet) |
| 5 | Jutalom + termináció | kész (7. fejezet) |
| 6 | Tanító pályák + sebesség-curriculum + domain randomization | kész (8. fejezet) |
| 7 | Kiértékelés + kísérlettervezés | kész (9. fejezet) |

Nyitott döntések: az **akciótér** (nyomaték vagy twist + IK), az **éles négyzetsarok** kezelése, és az MPC $N$ és $T$ értéke. (A $v_x, v_y$ kerete eldőlt: szabad orientációnál a pálya keretében, $\theta = 0$ oszloppal.)
