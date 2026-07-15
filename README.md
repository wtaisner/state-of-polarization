# PolCtrl — Kontroler polaryzacji SOP

System sterowania 4-sekcyjnym piezoelektrycznym kontrolerem polaryzacji
w łączeniu światłowodowym do transferu częstotliwości optycznej.

## Opis

Algorytm maksymalizuje moc beat note (sygnał dudnienia między laserem
lokalnym a sygnałem przychodzącym) poprzez sterowanie 4 napięciami
piezo (0–60 V, krok 0.1 V). Wykorzystuje:

- **SPSA** (Simultaneous Perturbation Stochastic Approximation) do estymacji
  gradientu w 4-wymiarowej przestrzeni napięć,
- **Adaptacyjny baseline i sigma** — dynamiczna estymacja osiągalnego sufitu
  mocy oraz poziomu szumu pomiarowego (bez sztywnych progów w dBm),
- **FSM TRACK/SEARCH/RECOVERY** — automat stanów z dead-zone i histerezą,
- **Kontekstowy bandyta UCB1** — adaptacyjny dobór agresywności sterowania
  w zależności od tempa dryfu polaryzacji i poziomu szumu.

## Architektura

```
c/              — rdzeń algorytmu w ANSI C99 (bez FPU, bez malloc)
python/         — symulator fizyczny, referencja, bindings ctypes
tests/          — testy jednostkowe (pytest)
scripts/        — skrypty uruchamiające, generator LUT, scenario_runner
pyproject.toml  — konfiguracja projektu i zależności (uv)
```

## Szybki start

```bash
# Zainstaluj zależności (tworzy .venv)
uv sync

# Build biblioteki C + uruchom wszystkie testy (116 testów)
bash scripts/run_all_tests.sh

# Alternatywnie: tylko testy Python
uv run pytest tests/

# Wygeneruj wykresy scenariuszy
uv run python scripts/scenario_runner.py
```

## Warstwy

1. **Rdzeń C** (`c/src/`) — czysty C99, fixed-point Q8.8, bez `float`/`double`,
   bez `malloc`/`free`. Docelowo trafia na AVR64DB (brak FPU).
2. **Symulator** (`python/simulator.py`) — model fizyczny sfery Poincarégo
   z dryfem OU i szumem interferometrycznym.
3. **Bindings** (`python/bindings.py`) — wrapper ctypes na `libpolctrl.so`.
4. **Referencja** (`python/reference_impl.py`) — czysto-pythonowa
   transliteracja logiki rdzenia (fixed-point), do testów parytetu.

## Dokumentacja

- `ALGORITHM.md` — pełny opis algorytmu z diagramami Mermaid
- `README_HAL.md` — dokumentacja kontraktu API dla elektronika
- `PROGRESS.md` — historia decyzji projektowych
- `OPEN_ISSUES.md` — otwarte problemy
- `FINAL_REPORT.md` — raport końcowy
