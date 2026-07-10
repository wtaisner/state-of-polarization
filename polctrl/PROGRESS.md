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
