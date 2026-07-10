# Raport końcowy — kontroler polaryzacji SOP

## Status ogólny
Zrobione — wszystkie fazy 0-8 zaimplementowane i przetestowane.

## Co zostało zaimplementowane

- [x] **Faza 0 — Szkielet projektu**: struktura katalogów, Makefile, .gitignore, requirements.txt
- [x] **Faza 1 — Arytmetyka stałoprzecinkowa Q8.8**: C (`fixedpoint.c/h`) + Python (`fixedpoint.py`), 22 testy (w tym parytet C↔Python)
- [x] **Faza 2 — Symulator fizyczny**: model sfery Poincarégo, dryf OU, szum biały/interferometryczny, 7 scenariuszy, 18 testów
- [x] **Faza 3a — Stałe i typy**: wszystkie `#define` w `polctrl.h` zsynchronizowane z `constants.py`
- [x] **Faza 3b — Adaptacyjny baseline**: EMA fast/slow, śledzenie sufitu, estymacja sigma, cold-start, 7 testów
- [x] **Faza 3c — Dead-zone i SEARCH**: FSM TRACK/SEARCH/RECOVERY, histereza, sudden-fade detection, periodic probe, 11 testów
- [x] **Faza 3d — SPSA**: per-coordinate boundary weighting, forced inward, snap-to-grid, compute_probe/apply_result, 16 testów
- [x] **Faza 3e — Kompletny FSM**: mapowanie stanu na profile SPSA (SEARCH override)
- [x] **Faza 3f — Kontekstowy bandyta UCB1**: LUT dla sqrt/ln, 4 ramiona, 6 koszyków kontekstu, 14 testów
- [x] **Faza 3g — Publiczne API**: `polctrl_init`/`polctrl_step`, wewnętrzny automat SPSA, kontrakt HAL
- [x] **Faza 4 — Bindings ctypes**: wszystkie struktury i funkcje eksportowane, sizeof parity
- [x] **Faza 5 — Pełny zestaw testów**: `run_all_tests.sh` (build + pytest + grep float/malloc)
- [x] **Faza 6 — Testy parytetu C vs Python**: 28 testów, **bit-identical trajectories po 10000 kroków**
- [x] **Faza 7 — README_HAL.md**: pełna dokumentacja kontraktu API dla elektronika
- [x] **Faza 8 — scenario_runner.py**: wykresy dla wszystkich 7 scenariuszy

## Wyniki testów

```
scripts/run_all_tests.sh:
  [1/4] Build C:           OK
  [2/4] pytest:            116 passed
  [3/4] No float/double:   OK
  [4/4] No malloc/free:    OK
  ALL TESTS PASSED
```

Rozbicie testów:
- `test_fixedpoint.py`: 22 (arytmetyka Q8.8 + parytet C↔Python)
- `test_simulator.py`: 18 (sanity + scenariusze)
- `test_baseline.py`: 7 (baseline, sigma, cold-start)
- `test_fsm.py`: 11 (przejścia, histereza, sudden-fade, probe)
- `test_spsa_core.py`: 16 (boundary weight, grid, convergence, fuzz)
- `test_bandit.py`: 14 (discretize, UCB1, overflow, independence)
- `test_parity_c_vs_python.py`: 28 (sizeof, constants, RNG, trajectory, modules)

## Kluczowe decyzje projektowe podjęte samodzielnie

1. **Estymator sigma**: użyto `|y_raw - y_slow|` zamiast `|y_fast - y_slow|`.
   Uzasadnienie: `|y_raw - y_slow|` jest znacznie lepszym estymatorem szumu
   (std ≈ 0.99 × sigma_true vs ≈ 0.26 × sigma_true dla `|y_fast - y_slow|`).
   [PROGRESS.md, Faza 3b]

2. **Cold-start**: inicjalizacja baseline po ustalonej liczbie iteracji (200)
   zamiast czekać na plateau z gradientu. Uproszczenie dopuszczone w specyfikacji.
   [PROGRESS.md, Faza 3b]

3. **Q-value update w bandycie**: EMA z cap 255 na effective count.
   Dla count > 255, alpha = 0 (Q zbieżane). Unika utraty precyzji w Q8.8.
   [PROGRESS.md, Faza 3f]

4. **Bandit LUT**: 128 wpisów dla `sqrt(ln(i+1))`, generowany offline w Pythonie,
   embedowany jako `static const` w C. `sqrt(n+1)` liczone przez integer square root.
   [PROGRESS.md, Faza 3f]

5. **V_STEP_Q88 = 26** (0.1V × 256 = 25.6 → zaokrąglone do 26).
   V_MAX_Q88 (15360) nie jest wielokrotnością 26 — wartości na granicy mogą
   nie być dokładnie na gridu. `snap_to_voltage_grid` clampuje do [0, 15360].
   [PROGRESS.md, Faza 3d]

6. **Sudden fade detection**: y_fast < 0.75 × y_slow → SEARCH (heurystyka
   niezależna od sigma, działa nawet gdy sigma nie jest jeszcze ustabilizowane).
   [PROGRESS.md, Faza 3c]

7. **Jednostka wewnętrzna**: `(dBm + 65) × 256`, zakres [0, 7680].
   Przesunięcie +65 dBm daje nieujemne wartości, skalowanie ×256 daje Q8.8.
   [polctrl.h, README_HAL.md]

## Otwarte problemy / czego nie udało się dokończyć

Brak otwartych problemów blokujących. Wszystkie fazy ukończone.

Potencjalne ulepszenia (nie-blokujące):
1. **Dobór ARM_PROFILES** — obecne wartości (a∈{1,4}V, c∈{1,4}V) to zgadnięcia.
   Wymaga empirycznego dostrojenia na prawdziwym sprzęcie lub przez scenario_runner.
2. **SPSA settle time** — 225 próbek (3×τ_slow) na pomiar może być konserwatywne.
   Można skrócić do 2×τ_slow (150) dla szybszej reakcji kosztem precyzji.
3. **Kompilacja pod AVR64DB** — podano przykładową komendę `avr-gcc`, ale dokładna
   nazwa MCU (`avr64db28/32/48`) wymaga weryfikacji z toolchainem Microchip.
4. **Kalibracja ADC→dBm** — wzór w README_HAL.md jest przykładowy; elektronik
   musi dostarczyć rzeczywistą krzywą kalibracji detektora.
5. **Persistencja stanu** — brak zapisu/odczytu `PolCtrlState` z EEPROM.
   Dodane jako opcjonalne w OPEN_ISSUES dla elektronika.

## Sugerowane następne kroki dla Witka

1. **Przejrzeć wykresy** w `scripts/output/*.png` — ocenić jakościowo zachowanie
   kontrolera na każdym scenariuszu.
2. **Dostroić ARM_PROFILES** w `constants.py` / `polctrl.h` — zmodyfikować wartości
   a_gain/c_gain i uruchomić `scenario_runner.py` ponownie, porównać wykresy.
3. **Skonsultować z Krzyśkiem** jednostkę wejściową — potwierdzić, że odczyt ADC
   może być przeliczony na dBm (wymaga kalibracji detektora), a następnie na Q8.8.
4. **Przetestować na AVR** — skompilować `polctrl` z `avr-gcc`, napisać `hal/main.c`
   z timerem 1ms, ADC, DAC. Zweryfikować zużycie RAM/Flash/czasu CPU.
5. **Zdecydować o SPSA settle time** — czy 225ms na pomiar jest akceptowalne,
   czy trzeba skrócić (kosztem precyzji gradientu).
6. **Rozważyć persistencję stanu** — czy zapisywać `PolCtrlState` do EEPROM
   przy wyłączaniu, żeby uniknąć cold-start po restarcie.

## Jak uruchomić / zweryfikować pracę

```bash
cd polctrl

# Build biblioteki C
make -C c

# Uruchom wszystkie testy (116 testów)
bash scripts/run_all_tests.sh

# Uruchom scenario_runner (generuje wykresy)
python3 scripts/scenario_runner.py
# Wykresy: scripts/output/*.png

# Uruchom tylko testy parytetu (krytyczne)
python3 -m pytest tests/test_parity_c_vs_python.py -v

# Sprawdź brak float/double w rdzeniu
grep -nE '\b(float|double)\b' c/src/{baseline,spsa,fsm,bandit,polctrl}.c
# (powinno zwrócić puste)
```
