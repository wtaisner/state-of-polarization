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
