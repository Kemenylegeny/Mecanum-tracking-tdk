# Később megválaszolandó kérdések

## 1. A szabad θ az MPC-ben (feltéve 2026-10-15, a 4. node után)

**A kérdés (saját szavaiddal válaszolj, a rubrikát a válaszod után mutatom meg):**
1. Mi a különbség aközött, hogy a $\theta$ nincs a *referenciában*, és aközött, hogy nincs a *modellben*?
2. Milyen feltétellel fogja az MPC egyáltalán „használni” a szabad $\theta$-t? Mondj rá egy konkrét példát a mecanumnál.

**A háttér, amit hozzá elmondtam:**

- A *referencia* és az *állapot* két külön dolog. Ha nincs $\theta_{ref}$, akkor a **költség** nem bünteti a $\theta$-t, de a $\theta$ attól még **állapot**, az $\omega$ pedig **bemenet** marad. Ha kiveszed a modellből (ahogy kipróbáltad: $x = (X, Y)$, $u = (v_x, v_y)$), a szabályzó nem tud forogni, és nem is tud a forgásról.
- A helyes felállás szabad $\theta$-val:
  - állapot: $(X, Y, \theta)$ (+ sebességek a dinamikus modellnél);
  - bemenet: $(v_x, v_y, \omega)$ (vagy nyomatékok);
  - költség: csak az $X, Y$ hibája és a bemenetek (az $\omega$ az $R$-ben);
  - linearizálás: az előző megoldás eltolt jósolt trajektóriája körül (RTI, Gros et al. 2020).
- **Az MPC csak azt optimalizálja, ami a modellben, a korlátokban vagy a költségben számít.** Világkoordinátás sebesség-bemenettel a holonom kinematika lineáris integrátor ($\dot X = v_{X,w}$, $\dot Y = v_{Y,w}$), és ebben a $\theta$ közömbös.
- A $\theta$ akkor számít, ha valami **irányfüggő**:
  - **Keréksebesség-korlát:** az IK-ból $\omega = 0$ mellett $|v_x| + |v_y| \le r\,\omega_{max}$. A megengedett test-sebességek **rombuszt** alkotnak: a tengelyek mentén a maximum $r\omega_{max}$, átlósan csak $r\omega_{max}/\sqrt{2}$. Gyors pályán ezért érdemes úgy fordulni, hogy a haladás a test tengelyére essen, és egy keréksebesség-korlátos MPC ezt magától megtalálja.
  - **Irányfüggő csúszás:** csak akkor használja ki, ha benne van a modellben. Az RL viszont a szimulátorból tanul, így kihasználhatja.
- Az MPC nem csak kis hibákra való. A horizonton belül nagy döntéseket is hozhat, de korlátozza
  - a horizont ($N T$),
  - a modell,
  - és a lokalitás: az LTV/QP lokális közelítés, ezért egy „balra vagy jobbra forduljak 180°-ot?” típusú döntést nem talál meg globálisan.

## 2. Záró kérdés: mit szabad kijelenteni, ha az RL nyer? (feltéve 2026-10-15, a 7. node végén)

**A kérdés (saját szavaiddal válaszolj, a rubrikát a válaszod után mutatom meg):**
A kísérletek lefutottak, és az E1-ben (a fő összehasonlításban) az RL kisebb pozíció-RMSE-t ér el, mint az MPC.
1. Mit kell ellenőrizned, mielőtt egyáltalán azt mondod, hogy az RL jobb?
2. Milyen okokból nyerhet, és melyik kísérlettel (E1–E5, A1–A4) tudod ezeket szétválasztani?
3. Pontosan mit írnál le a dolgozat következtetésében?

**A kísérletek** (részletesen: `mecanum_tracking/MAGYARAZAT.md`, 9.4 fejezet):

| # | Kísérlet | Kérdés |
|---|---|---|
| E1 | Fő eredmény: RL vs. MPC a két teszthalmazon (standard alakzatok + nem látott véletlen görbék), $s = 1$ | ki követ jobban, és miben (kontúr vagy lag)? |
| E2 | Sebesség-sorozat: $s = 0.5, 1.0, 1.5$ | hogyan romlik a követés a dinamikai határ felé? |
| E3 | Robusztusság: súrlódás és tömeg a tanítási tartományon kívül, mindkettőnek azonosan | ki bírja jobban a modellhibát? |
| E4 | Csúszásmentes/ideálisabb szim vagy identifikált modellű MPC | a különbség a modellhibából jön-e? |
| E5 | A szabad orientáció kihasználása (forgás, oldalazás) | megtanulja-e az RL, hogy kevesebbet oldalazzon? |
| A1 | RL zaj nélkül tanítva | mennyit ad vagy vesz el a zajos tanítás? |
| A2 | RL sebességtag nélkül | kell-e a sebességtag? |
| A3 | $-\lVert e\rVert^2$ az exp-kernel helyett | igazolja-e az öngyilkossági érvet? |
| A4 | twist + IK vs. nyomaték akciótér | mennyit számít az interfész? |

Tipp: gondolj vissza a 2. fejezet alapelvére, a 9.1 fejezetre (seedek) és a felmérés első kérdésére (modellhiba).
