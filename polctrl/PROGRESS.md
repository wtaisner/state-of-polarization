# PROGRESS.md — Historia decyzji projektowych

## Faza 0 — Szkielet projektu

### Zrobiono
- Utworzono strukturę katalogów: `c/{include,src,tests}`, `python/`, `tests/`, `scripts/output/`.
- `.gitignore` (build artefakty, `__pycache__`, `*.so`, `*.o`, IDE).
- `Makefile` w `c/` budujący `libpolctrl.so` (`gcc -std=c99 -Wall -Wextra -pedantic -O2 -fPIC -shared`) oraz cel `size_check` i `test`.
- `requirements.txt`: numpy, pytest, matplotlib.
- `README.md` z opisem projektu.

### Testy
- Brak testów w tej fazie (tylko szkielet).

### Decyzje
- Struktura katalogów zgodna ze specyfikacją.
- `Makefile` zawiera cele: `all` (buduje `.so`), `size_check` (binarny do testów `sizeof`), `test` (testy C), `clean`.

---

## Faza 1 — Arytmetyka stałoprzecinkowa Q8.8

### Zrobiono
- `c/include/fixedpoint.h` / `c/src/fixedpoint.c`: pełna implementacja Q8.8 na `int16_t`.
  - `fp_from_float`, `fp_to_float` (tylko testy/init), `fp_mul`, `fp_div`, `fp_clamp`, `fp_abs`.
  - Mnożenie/dzielenie przez `int32_t` pośrednio, z clamping do `int16_t`.
  - Dzielenie przez zero: zwraca wartość graniczną (FP_MAX/FP_MIN/0).
  - `fp_abs` obsługuje przypadek brzegowy FP_MIN (-32768) → zwraca FP_MAX.
- `python/fixedpoint.py`: lustrzane odbicie C, ręczna symulacja przepełnienia `int16_t`/`int32_t`.
  - `_to_int16`/`_to_int32` symulują maskowanie i rozszerzenie znaku.
  - `_c_div` implementuje dzielenie z obcięciem w stronę zera (zgodne z C, różne od Python `//`).
- `python/constants.py`: wszystkie stałe numeryczne ze specyfikacji (jedno źródło prawdy).
- `tests/test_fixedpoint.py`: 22 testy (Python + parytet C vs Python).
- `scripts/run_all_tests.sh`: buduje C, uruchamia pytest, grep-sprawdza brak float/double/malloc.

### Konwencje zaokrąglania (ważne dla parytetu)
- `fp_from_float`: zaokrąglenie do najbliższej (round half away from zero).
- `fp_mul`: arytmetyczne przesunięcie w prawo (>> na signed int = obcięcie w stronę -inf).
  To pasuje do Python `>>` na liczbach ujemnych.
- `fp_div`: dzielenie całkowite C (obcięcie w stronę zera). Python używa `_c_div` do dopasowania.

### Testy (22/22 przechodzą)
- `TestPythonFixedPoint`: from_float, to_float, mul, div, div_by_zero, clamp, abs, overflow, voltage range.
- `TestCParity`: parytet C vs Python dla from_float, mul, div, div_by_zero, clamp, abs, edge cases.

### Decyzje
- Tolerancja błędu w testach precyzji: `< 2/256` (1 LSB margines na obcięcie). Testy porównują
  wynik z ilorazem/iloczynem wartości stałoprzecinkowych (nie oryginalnych float), aby odizolować
  błąd arytmetyki od błędu reprezentacji.
- Przypadki overflow (np. 5*30=150 > 127.99) są pomijane w teście precyzji, testowane osobno
  w `test_mul_overflow_clamp`.
- `V_STEP_Q88 = 26` (0.1V * 256 = 25.6 → zaokrąglone do 26). To jest najbliższa reprezentacja 0.1V
  w Q8.8 — błąd to 0.0015625V, akceptowalne dla kroku aktuatora.

---

## Faza 2 — Symulator fizyczny (Python)

### Zrobiono
- `python/simulator.py`: pełny model fizyczny łącza światłowodowego.
  - Reprezentacja SOP: punkt na sferze Poincarégo (znormalizowane wektory Stokesa s1,s2,s3).
  - Model aktuatora: 4 sekcje piezo jako retardery. Sekcje {0,1} obracają SOP wokół osi A=(1,0,0),
    sekcje {2,3} wokół osi B=(0,1,0). Kąt obrotu = (V/V_max) * max_rotation_rad (domyślnie 2π).
    Zastosowanie sekwencyjne: SOP_out = R3·R2·R1·R0·SOP_in (wzór Rodriguesa).
  - Beat note power: I = cos²(angle(SOP_out, SOP_ref)/2), przeskalowane do dBm względem
    konfigurowalnego sufitu kanału (channel_ceiling_dbm, domyślnie -38 dBm).
  - Dryf SOP: proces Ornsteina-Uhlenbecka na azymucie/elewacji, z konfigurowalną stałą czasową
    i amplitudą.
  - Szum pomiaru: dwa tryby — (a) biały gaussowski, (b) interferometryczny (suma sinusoid
    o losowych fazach + szum biały).
  - Metoda `step(voltages, dt_ms=1.0)` zwraca zaszumiony odczyt mocy w dBm.
  - 7 scenariuszy: stable, slow_drift, fast_drift, regime_switch, cold_start, sudden_fade,
    channel_degradation.
  - Metoda `get_optimal_voltages()` (wyszukiwanie na siatce, do testów).
- `tests/test_simulator.py`: 18 testów (sanity checks + scenariusze).

### Testy (18/18 przechodzą)
- Moc maksymalna gdy SOP_out == SOP_reference.
- Moc niska gdy SOP orthogonal do reference.
- Zakres dBm w granicach fizycznego detektora.
- Stabilność przy braku dryfu i szumu.
- Monotoniczność: ruch w stronę optimum zwiększa moc.
- Clamping napięć.
- Odtwarzalność przy ustalonym seedzie.
- Wszystkie 7 scenariuszy uruchamia się bez błędu.
- Stable: niska wariancja.
- Sudden fade: spadek mocy po zdarzeniu.
- Channel degradation: sufit mocy spada w czasie.
- Regime switch: wariancja rośnie po przełączeniu w tryb szybkiego dryfu.

### Decyzje
- `max_rotation_rad = 2π` (pełny obrót przy V_max). Daje aktuatorowi wystarczający zakres
  do osiągnięcia dowolnego SOP na sferze — fizycznie realistyczne dla 4-sekcyjnego kontrolera.
- Test `test_sudden_fade_triggers_drop` używa niestandardowego symulatora z zerowym dryfem
  i odwróceniem wektora SOP (zamiast losowego), aby odizolować efekt nagłego skoku od dryfu.
  Próg spadku: 3 dB (nie 5 dB) — aktuator ma wystarczający zakres, by częściowo kompensować
  zmianę SOP, więc spadek nie jest tak drastyczny jak przy pełnym odłączeniu sygnału.
- Scenariusz `regime_switch` implementuje przełączenie przez nadpisanie metody `step`
  (wrapper zmieniający parametry dryfu po ustalonym kroku).

---

## Faza 3 (3a-3g) — Rdzeń algorytmu w portable C

### Zrobiono

**3a: Stałe i typy (`polctrl.h`, `polctrl_internal.h`)**
- Wszystkie stałe numeryczne zdefiniowane jako `#define` w `polctrl.h`, odpowiadające `python/constants.py`.
- `PolCtrlOutput`: struktura wyjściowa (actuate flag + 4 napięcia Q8.8).
- `PolCtrlState`: pełny stan kontrolera (244 bajty), zawiera stany wszystkich podmodułów.
- Jednostka wewnętrzna: `beatnote_reading = (dBm + 65) * 256`, zakres [0, 7680].
- Napięcia w Q8.8 wprost jako wolty: 0V → 0, 60V → 15360.

**3b: Adaptacyjny baseline (`baseline.c`)**
- `BaselineState`: baseline, noise_sigma, y_fast (EMA τ=8ms), y_slow (EMA τ=75ms), initialized, warmup_counter.
- `baseline_update`: aktualizuje EMAs, baseline (szybki wzrost, powolny spadek), sigma (tylko w dead-zone).
- `baseline_zscore`: (baseline - y_slow) / max(sigma, eps). Podczas cold-start zwraca FP_MAX (wymusza SEARCH).
- **Decyzja**: sigma estymowana jako EMA z |y_raw - y_slow| (zamiast |y_fast - y_slow| z propozycji specyfikacji).
  Uzasadnienie: |y_raw - y_slow| jest znacznie lepszym estymatorem szumu (std ≈ 0.99 * sigma_true)
  w porównaniu do |y_fast - y_slow| (std ≈ 0.26 * sigma_true). Dokumentowane jako świadoma zmiana.
- **Decyzja**: cold-start inicjalizuje baseline po ustalonej liczbie iteracji (COLD_START_WARMUP=200),
  zamiast czekać na plateau z gradientu. Uproszczenie dopuszczone w specyfikacji.

**3c: Dead-zone i SEARCH (`fsm.c`, część 1)**
- `FsmState`: mode (TRACK/SEARCH/RECOVERY), consecutive_good_windows, periodic_probe_counter.
- `fsm_update`: przejścia TRACK↔SEARCH↔RECOVERY z histerezą (HYSTERESIS_WINDOWS=5).
- Nagła detekcja spadku: y_fast < 0.75 * y_slow → SEARCH (niezależne od z-score).
- `fsm_should_actuate`: 1 jeśli zscore > k1 (2.5 sigma) lub w SEARCH.
- `fsm_check_periodic_probe`: wymusza SPSA co PERIODIC_PROBE_INTERVAL (30000) próbek, nieaktywne w SEARCH.

**3d: SPSA (`spsa.c`)**
- `SpsaState`: theta[4], last_grad_estimate[4], rng, delta[4], c_k[4].
- `boundary_weight`: 1.0 w środku, liniowo maleje do 0.2 (BOUNDARY_FLOOR_WEIGHT) na brzegach.
- `forced_inward_sign`: +1 poniżej 2V, -1 powyżej 58V, 0 w środku.
- `spsa_compute_probe`: losuje delta, liczy theta_plus/theta_minus z wagami brzegowymi.
  Możliwość wyłączenia wag brzegowych (dla SEARCH).
- `spsa_apply_result`: ĝ = (y_plus - y_minus) / (2 * c_k * delta), theta += a * ĝ, clamp, snap do gridu.
  Gradient clamp ±4.0, zabezpieczenie przed przepełnieniem fp_div.
- `snap_to_voltage_grid`: zaokrągla do najbliższego 0.1V (V_STEP_Q88=26), clamp do [0, 60].

**3e: Kompletny FSM (`fsm.c`, część 2)**
- `fsm_gain_for_mode`: SEARCH → SEARCH_GAIN_PROFILE (a=8V, c=8V), TRACK/RECOVERY → bandit profile.

**3f: Kontekstowy bandyta (`bandit.c`)**
- `BanditState`: q_value[6][4], count[6][4], total_count.
- `discretize_context`: 3 poziomy dryfu × 2 poziomy szumu = 6 koszyków.
- `bandit_select_arm`: UCB1 z LUT dla sqrt(ln(N+1)) i isqrt dla sqrt(n+1).
  LUT: 128 wpisów, generowany przez `scripts/generate_lut.py`.
- `bandit_update`: EMA z alpha = 1/min(count, 255) (cap na 255 dla uniknięcia utraty precyzji).
- `ARM_PROFILES`: 4 profile (kombinacje a∈{1,4}V × c∈{1,4}V).

**3g: Publiczne API (`polctrl.c`)**
- `polctrl_init(rng_seed)`: inicjalizuje wszystkie podmoduły, theta=30V, rng=xorshift32.
- `polctrl_step(state, beatnote, &out)`: główna pętla 1ms.
  - Aktualizuje baseline, liczy z-score, aktualizuje FSM.
  - Wewnętrzny automat SPSA: IDLE → SET_PLUS → MEASURE_PLUS → SET_MINUS → MEASURE_MINUS → APPLY → IDLE.
  - Czas trwania rundy SPSA: ~456 próbek (2×225 settle + 6 przejść) ≈ 0.46s.
  - Bandit aktualizowany co BANDIT_WINDOW_ITERATIONS (50) rund SPSA.
  - Reward: mean(y_slow) - LAMBDA*boundary_fraction - MU*movement.
- HAL musi implementować tylko: timer 1ms → ADC → polctrl_step → DAC (gdy actuate==1).

### Faza 4 — Bindings ctypes (`python/bindings.py`)
- Wszystkie struktury C zdefiniowane jako `ctypes.Structure` z zgodnym layoutem.
- `polctrl_init`, `polctrl_step` oraz funkcje wszystkich podmodułów eksportowane.
- Wszystkie sizeof() struktur Python == C (test parytetu).

### Faza 5 — Testy (88/88 przechodzą)
- `test_fixedpoint.py`: 22 testy (Python + parytet C).
- `test_simulator.py`: 18 testów (sanity + scenariusze).
- `test_baseline.py`: 7 testów (init, cold-start, fast-rise, slow-fall, sigma).
- `test_fsm.py`: 11 testów (przejścia, histereza, sudden-fade, periodic probe).
- `test_spsa_core.py`: 16 testów (boundary weight, forced inward, grid, convergence, fuzz).
- `test_bandit.py`: 14 testów (discretize, UCB1 convergence, overflow, independence).
- `run_all_tests.sh`: build C + pytest + grep (float/double/malloc).

### Decyzje
- Zmiana estymatora sigma: |y_raw - y_slow| zamiast |y_fast - y_slow| (uzasadnienie wyżej).
- `snap_to_voltage_grid` dodatkowo clampuje do [V_MIN, V_MAX].
- V_MAX_Q88 (15360) nie jest wielokrotnością V_STEP_Q88 (26) — wartości na granicy mogą
  nie być dokładnie na gridu. Testy akceptują theta==V_MAX_Q88 jako wyjątek.
- Q-value update z cap 255 na effective count (powyżej 255, alpha=0, Q zbiegałe).
- Bandit LUT generowany offline w Pythonie, embedowany jako stała tablica w C.

---

## Faza 6 — Testy parytetu C vs Python (KRYTYCZNE)

### Zrobiono
- `python/reference_impl.py`: czysto-pythonowa transliteracja logiki rdzenia C,
  używająca `fixedpoint.py` do całej arytmetyki (bez float/double).
  - Implementuje: rng (xorshift32), baseline, FSM, SPSA, bandyta, polctrl_step.
  - Ten sam algorytm PRNG w obu językach (ręczna implementacja, nie `random`/`numpy.random`).
- `tests/test_parity_c_vs_python.py`: 28 testów parytetu.
  - Struct sizeof: 8 testów (ctypes vs C sizeof).
  - Stałe: 14 testów (parsowanie `#define` z `polctrl.h` vs `constants.py`).
  - RNG: 1 test (1000 wywołań, identyczna sekwencja).
  - Trajektoria: 2 testy (10000 kroków z losowymi wejściami, 2000 kroków z symulatorem).
  - Moduły: 4 testy (baseline_update, fsm_update, bandit_select, isqrt).

### Wynik
- **Wszystkie 28 testów parytetu przechodzi.**
- Trajektorie `theta` bit-dokładnie identyczne między C a Python po 10000 kroków.
- Każda rozbieżność byłaby bugiem — nie znaleziono żadnej.

### Łącznie testów: 116/116 przechodzi.

---

## Faza 7 — README_HAL.md

### Zrobiono
- Pełna dokumentacja kontraktu API dla elektronika w `README_HAL.md` (po polsku).
- Opisuje: inicjalizację, pętlę 1ms, jednostki wejściowe/wyjściowe (dokładne wzory),
  wymagania pamięciowe, kompilację pod AVR64DB, diagram przepływu, szybki test na PC.

## Faza 8 — scenario_runner.py

### Zrobiono
- `scripts/scenario_runner.py`: uruchamia pełny stack (symulator + skompilowane C)
  na wszystkich 7 scenariuszach, generuje wykresy (moc, napięcia, FSM, bandyta).
- Wyniki zapisane w `scripts/output/*.png`.

### Obserwacje jakościowe
- **stable**: mean=-39.9 dBm (blisko sufitu -38), niski ruch (124.9V), głównie TRACK.
  Dead-zone działa — kontroler nie rusza aktuatorem niepotrzebnie.
- **slow_drift**: mean=-41.8, umiarkowany ruch, trochę SEARCH — kontroler podąża za dryfem.
- **fast_drift**: mean=-42.1, duży ruch (3619V), więcej SEARCH/RECOVERY — agresywniejsze sterowanie.
- **regime_switch**: pierwsza połowa spokojna, druga agresywna — bandyta adaptuje.
- **cold_start**: SEARCH na starcie (1465 kroków), potem odzyskuje sygnał.
- **sudden_fade**: SEARCH wyzwolony po nagłym skoku, częściowe odzyskanie.
- **channel_degradation**: moc spada w czasie (mean=-51.1), baseline adaptuje się w dół.
