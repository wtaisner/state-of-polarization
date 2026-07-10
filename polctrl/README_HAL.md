# README_HAL — Dokumentacja kontraktu API dla elektronika

**Dokumentacja w języku polskim.** Ten dokument opisuje kontrakt między
rdzeniem algorytmu (`libpolctrl`) a warstwą sprzętową (HAL) na mikrokontrolerze
AVR64DB.

---

## 1. Przegląd

Rdzeń algorytmu sterowania polaryzacją jest zaimplementowany w czystym C99
bez zależności sprzętowych. Elektronik musi zaimplementować tylko:

1. **Timer 1 ms** — wywołuje `polctrl_step` co milisekundę.
2. **ADC** — odczytuje moc beat note i przelicza na jednostkę wewnętrzną.
3. **DAC** — ustawia 4 napięcia piezo, gdy algorytm tego zażąda.

**Żadnej logiki decyzyjnej po stronie MCU.** Wszystkie decyzje (kiedy ruszyć
aktuatorem, jakie napięcia ustawić, tryb SEARCH/TRACK) są podejmowane przez
`polctrl_step`.

---

## 2. Kontrakt API

### 2.1 Inicjalizacja

```c
#include "polctrl.h"

PolCtrlState state = polctrl_init(rng_seed);
```

- `rng_seed` — dowolna wartość `uint32_t` różna od 0 (0 jest zamieniane na 1).
  Determinuje sekwencję losową SPSA. Ta sama wartość → to samo zachowanie.
- Wywoływać **raz** przy starcie systemu.
- `PolCtrlState` to struktura 244 bajtów — przechowywana w RAM, przekazywana
  przez wartość (jest kopiowana przy każdym wywołaniu `polctrl_step`).

### 2.2 Pętla główna (1 ms)

```c
PolCtrlOutput output;
fp_t beatnote_reading = read_and_scale_adc();  // patrz sekcja 3
state = polctrl_step(state, beatnote_reading, &output);

if (output.actuate) {
    for (int i = 0; i < NUM_SECTIONS; i++) {
        set_dac_voltage(i, output.voltages[i]);  // patrz sekcja 4
    }
}
```

- Wywoływać **co 1 ms** z timera.
- `beatnote_reading` — odczyt mocy w jednostce wewnętrznej (patrz sekcja 3).
- `output.actuate == 1` — należy ustawić nowe napięcia na DAC.
- `output.actuate == 0` — nie ruszać DAC, zachować dotychczasowe napięcia.
- `output.voltages[]` — 4 wartości napięć w Q8.8 (patrz sekcja 4).

### 2.3 Struktury danych

```c
// Typ liczbowy: Q8.8 na int16_t
typedef int16_t fp_t;

// Wyjście kontrolera
typedef struct {
    uint8_t actuate;              // 0 lub 1
    fp_t    voltages[NUM_SECTIONS]; // Napięcia w Q8.8 (wolty)
} PolCtrlOutput;

// Pełny stan kontrolera (244 bajty)
typedef struct PolCtrlState PolCtrlState;
```

---

## 3. Jednostka wejściowa: `beatnote_reading`

### Wzór przeliczenia

```
beatnote_reading = (fp_t)((dBm + 65.0) * 256)
```

- **dBm** — odczyt mocy beat note w dBm, zakres -65..-35.
- **+65** — przesunięcie, aby wartości były nieujemne.
- **×256** — konwersja do formatu Q8.8.

### Przykłady

| dBm    | beatnote_reading (Q8.8) |
|--------|-------------------------|
| -65    | 0                       |
| -50    | 3840                    |
| -38    | 6912                    |
| -35    | 7680                    |

### Implementacja na MCU

Odczyt ADC → kalibracja na dBm → przeliczenie na Q8.8:

```c
// Przykład (dostosuj do swojej kalibracji ADC):
float adc_voltage = adc_read() * VREF / ADC_MAX;
float dBm = calibrate_voltage_to_dBm(adc_voltage);
fp_t beatnote_reading = (fp_t)((dBm + 65.0f) * 256.0f);
```

**Uwaga:** Przeliczenie ADC → dBm → Q8.8 jest jedynym miejscem gdzie
dozwolone jest użycie zmiennoprzecinkowe (po stronie HAL, nie rdzenia).
Jeśli MCU nie ma FPU, można użyć stałoprzecinkowej kalibracji LUT.

### Ważne: zakres

Jeśli odczyt ADC jest poza zakresem detektora, **przycinaj** do [0, 7680]:

```c
if (beatnote_reading < 0) beatnote_reading = 0;
if (beatnote_reading > 7680) beatnote_reading = 7680;
```

---

## 4. Jednostka wyjściowa: `voltages[]`

### Format

Każda wartość `voltages[i]` jest w formacie **Q8.8 reprezentującym wolty**:

```
voltage_volts = voltages[i] / 256.0
```

- Zakres: 0..15360 (0.0V..60.0V)
- Rozdzielczość: 0.1V (grid = 26 w Q8.8 ≈ 0.1016V)

### Przeliczenie na DAC

```c
void set_dac_voltage(uint8_t channel, fp_t voltage_q88) {
    // Przeliczenie Q8.8 na wartość DAC
    // Zależy od konkretnego DAC — przykład dla 12-bit DAC (0-4095):
    // voltage_volts = voltage_q88 / 256.0
    // dac_value = voltage_volts / 60.0 * 4095
    // = voltage_q88 / 256.0 / 60.0 * 4095
    // = voltage_q88 * 4095 / (256 * 60)
    // = voltage_q88 * 4095 / 15360
    uint32_t dac_value = (uint32_t)voltage_q88 * 4095 / 15360;
    dac_write(channel, dac_value);
}
```

### Zabezpieczenia

Rdzeń **zawsze** zwraca napięcia w zakresie [0, 15360] i zaokrąglone do gridu
0.1V. Dodatkowe zabezpieczenia po stronie HAL nie są wymagane, ale zalecane:

```c
if (dac_value > 4095) dac_value = 4095;
```

---

## 5. Wymagania pamięciowe i wydajnościowe

### Pamięć

| Struktura        | Rozmiar (bajty) |
|------------------|-----------------|
| `PolCtrlState`   | 244             |
| `PolCtrlOutput`  | 10              |
| `libpolctrl.so`  | ~28 KB (skompilowana) |
| Kod (.text)      | ~8 KB           |

AVR64DB ma 64 KB RAM i 64 KB Flash — wystarczająco z dużym zapasem.

### Wydajność

- `polctrl_step` wykonuje stałą liczbę operacji (brak pętli zależnych od danych
  poza iteracjami po 4 sekcjach).
- Orientacyjnie: ~200-500 operacji całkowitoliczbowych na wywołanie.
- Przy 1 MHz CPU (AVR conservative) to < 1 ms — mieści się w budżecie.
- **Brak alokacji dynamicznej** (`malloc`/`free` zakazane).
- **Brak FPU** — cała arytmetyka w Q8.8 na `int16_t`/`int32_t`.

---

## 6. Czego NIE zaimplementowano (wymaga uwagi elektronika)

1. **Sterownik ADC** — odczyt detektora beat note, kalibracja na dBm.
2. **Sterownik DAC** — ustawianie 4 napięć piezo (0-60V).
3. **Timer 1 ms** — przerwanie okresowe wywołujące `polctrl_step`.
4. **Inicjalizacja sprzętu** — konfiguracja pinów, taktowanie, itp.
5. **Watchdog / fault recovery** — co robić przy błędzie ADC/DAC.
6. **Persistencja stanu** — zapis/odczyt `PolCtrlState` z EEPROM (opcjonalne).

Patrz też `OPEN_ISSUES.md` dla otwartych problemów algorytmicznych.

---

## 7. Kompilacja pod AVR64DB

### Toolchain

```bash
# Zainstaluj avr-gcc (np. z Atmel/Microchip toolchain):
# sudo apt install gcc-avr avr-libc

# Kompilacja:
avr-gcc -std=c99 -Wall -Wextra -pedantic -Os -mmcu=avr64db32 \
    -I c/include \
    c/src/fixedpoint.c c/src/rng.c c/src/baseline.c \
    c/src/spsa.c c/src/fsm.c c/src/bandit.c c/src/polctrl.c \
    hal/main.c -o polctrl.elf
```

### Uwagi

- `-mmcu=avr64db32` — dokładna nazwa MCU zależy od konkretnej wersji układu
  (avr64db28, avr64db32, avr64db48). Sprawdź w datasheecie.
- `-Os` — optymalizacja pod kątem rozmiaru (zalecane dla MCU).
- Plik `hal/main.c` — implementacja pętli głównej i sterowników I/O (do napisania).
- Biblioteka `libpolctrl.so` jest dla testów na PC. Na MCU kompiluj jako
  część firmware'u (linkuj pliki `.c` bezpośrednio).

### Otwarty punkt

Dokładna nazwa MCU w toolchainie avr-gcc wymaga weryfikacji. Możliwe opcje:
`avr64db28`, `avr64db32`, `avr64db48`. Sprawdź dokumentację toolchaina
Microchip/Atmel dla serii AVR DB.

---

## 8. Diagram przepływu

```
┌─────────┐     ┌──────────────────┐     ┌─────────┐
│  Timer  │────▶│  polctrl_step()  │────▶│   DAC   │
│  1 ms   │     │                  │     │  4×V    │
└─────────┘     │  ┌────────────┐  │     └─────────┘
                │  │ Baseline   │  │
                │  │ (EMA, σ)   │  │
                │  └─────┬──────┘  │
                │        │         │
                │  ┌─────▼──────┐  │     ┌─────────┐
                │  │ FSM        │  │     │   ADC   │
                │  │ TRACK/SEARCH│ ◀│─────│ beatnote│
                │  └─────┬──────┘  │     │  -65dBm │
                │        │         │     └─────────┘
                │  ┌─────▼──────┐  │
                │  │ SPSA       │  │
                │  │ (gradient) │  │
                │  └─────┬──────┘  │
                │        │         │
                │  ┌─────▼──────┐  │
                │  │ Bandit     │  │
                │  │ (UCB1)     │  │
                │  └────────────┘  │
                └──────────────────┘
```

---

## 9. Szybki test na PC

```bash
cd polctrl
make -C c
bash scripts/run_all_tests.sh
```

Wszystkie 116 testów powinno przejść (w tym testy parytetu C↔Python).
