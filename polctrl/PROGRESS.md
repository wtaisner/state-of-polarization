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
