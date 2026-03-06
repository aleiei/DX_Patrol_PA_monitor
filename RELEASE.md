# RELEASE NOTES — IU0PXK PA Monitor

## v2.0.0 — 2026-03-06

Refactoring completo del firmware originale. Di seguito l'elenco dettagliato dei miglioramenti introdotti.

---

### 1. Costanti simboliche al posto dei magic numbers

Tutti i valori numerici hardcodati nel corpo del codice sono stati estratti e dichiarati come `constexpr` con nomi autoesplicativi in cima al file sorgente.

| Costante | Valore | Descrizione |
|---|---|---|
| `TEMP_OFFSET_MV` | `464.0f` | Offset tensione sensore temperatura a 0 °C |
| `TEMP_MV_PER_DEG` | `6.25f` | Sensibilità sensore (mV/°C) |
| `PWR_ADC_OFFSET_MV` | `298.0f` | Offset canale FWD in mV |
| `SWR_ADC_OFFSET_MV` | `1288.0f` | Offset canale REF in mV |
| `RAW_PWR_NOISE_FLOOR` | `8` | Soglia ADC rumore canale potenza |
| `RAW_SWR_NOISE_FLOOR` | `4` | Soglia ADC rumore canale SWR |
| `VREF_MV` | `5000.0f` | Tensione di riferimento ADC |
| `ADC_SAMPLES` | `16` | Numero campioni per averaging |
| `LOOP_INTERVAL_MS` | `100` | Periodo aggiornamento loop (ms) |

Cambiare un parametro di calibrazione o di soglia non richiede più di cercare nel codice: basta modificare la costante in cima al file.

---

### 2. ADC averaging (riduzione rumore)

Introdotta la funzione helper `adcAverage(pin, samples)` che esegue `N` letture consecutive sullo stesso pin e restituisce la media in virgola mobile.

Con `ADC_SAMPLES = 16` il rumore di quantizzazione viene ridotto di un fattore √16 = 4, eliminando gran parte delle oscillazioni di ±1–2 count che causavano salti indesiderati sul display, specialmente a bassa potenza.

```cpp
static float adcAverage(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) sum += analogRead(pin);
    return static_cast<float>(sum) / samples;
}
```

---

### 3. Loop non bloccante con `millis()`

Il precedente `delay(100)` bloccava l'intera CPU per 100 ms ad ogni ciclo, rendendo impossibile gestire interrupt o aggiungere funzionalità future (es. pulsanti, allarmi).

Il loop è ora basato sul pattern `millis()`:

```cpp
if (now - last_update_ms < LOOP_INTERVAL_MS) return;
last_update_ms = now;
```

Il periodo di aggiornamento rimane invariato (100 ms) ma il microcontrollore è libero di eseguire altro codice tra un ciclo e l'altro.

---

### 4. Aggiornamento LCD senza flickering

Il problema originale: ogni iterazione cancellava e riscriveva tutte le righe del display, causando un flickering visibile a 10 fps.

La nuova funzione `lcdUpdateLine()` mantiene una cache delle stringhe già visualizzate e scrive sul display **solo se il contenuto è cambiato**. Usa inoltre `snprintf` con stringa a larghezza fissa per sovrascrivere in-place senza dover cancellare la riga.

---

### 5. Debug seriale condizionale (`#define DEBUG`)

In produzione l'output seriale non serve e spreca cicli CPU e memoria flash. Tutto il codice di debug è ora racchiuso in blocchi `#ifdef DEBUG`:

```cpp
// #define DEBUG    ← decommentare per abilitare
```

Quando `DEBUG` non è definito, il compilatore esclude completamente il codice seriale. Per riattivare il debug durante la calibrazione è sufficiente decommentare una riga.

---

### 6. Algoritmo di Horner per la valutazione del polinomio

La funzione `evalPoly` è stata riscritta usando lo **schema di Horner**, che valuta un polinomio di grado N con solo N moltiplicazioni e N addizioni (invece di N chiamate a `pow()`), ideale su un MCU a 8 bit senza FPU:

```cpp
// Prima (loop con pow):      y += c[i] * xi;  xi *= x;
// Dopo (schema di Horner):   y = y * x + c[i];
```

---

### 7. Uso di `constrain()` per il clamping

Le catene di `if` per limitare i valori in range sono state sostituite con la funzione Arduino `constrain(value, min, max)`, più leggibile e meno soggetta a errori di trascrizione.

---

### Compatibilità

Nessuna modifica alle dipendenze esterne. Il progetto continua a richiedere solo:

- `LiquidCrystal_I2C` (come da `platformio.ini`)
- PlatformIO con board `nanoatmega328new`

---

### Prossimi passi consigliati

- Acquisire dati reali di calibrazione ed eseguire `calibration_fit.py` per sostituire i coefficienti placeholder in `PWR_COEFFS[]`.
- Valutare se aggiungere una regressione polinomiale anche per SWR (i coefficienti sono già predisposti nel sorgente, commentati).
- Considerare l'aggiunta di un allarme visivo (backlight lampeggiante) quando SWR > 2.0 o temperatura > soglia configurabile.

---

---

# RELEASE NOTES — IU0PXK PA Monitor *(English)*

## v2.0.0 — 2026-03-06

Complete refactoring of the original firmware. Below is a detailed list of all improvements introduced.

---

### 1. Symbolic constants instead of magic numbers

All numeric literals hardcoded in the body of the code have been extracted and declared as `constexpr` with self-explanatory names at the top of the source file.

| Constant | Value | Description |
|---|---|---|
| `TEMP_OFFSET_MV` | `464.0f` | Temperature sensor voltage offset at 0 °C |
| `TEMP_MV_PER_DEG` | `6.25f` | Sensor sensitivity (mV/°C) |
| `PWR_ADC_OFFSET_MV` | `298.0f` | FWD channel offset in mV |
| `SWR_ADC_OFFSET_MV` | `1288.0f` | REF channel offset in mV |
| `RAW_PWR_NOISE_FLOOR` | `8` | ADC noise threshold for power channel |
| `RAW_SWR_NOISE_FLOOR` | `4` | ADC noise threshold for SWR channel |
| `VREF_MV` | `5000.0f` | ADC reference voltage |
| `ADC_SAMPLES` | `16` | Number of samples for averaging |
| `LOOP_INTERVAL_MS` | `100` | Loop update period (ms) |

Changing a calibration or threshold parameter no longer requires hunting through the code — just edit the constant at the top of the file.

---

### 2. ADC averaging (noise reduction)

Introduced the helper function `adcAverage(pin, samples)`, which takes `N` consecutive readings on the same pin and returns the floating-point average.

With `ADC_SAMPLES = 16`, quantisation noise is reduced by a factor of √16 = 4, eliminating most of the ±1–2 count oscillations that caused unwanted display jumps, especially at low power levels.

```cpp
static float adcAverage(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) sum += analogRead(pin);
    return static_cast<float>(sum) / samples;
}
```

---

### 3. Non-blocking loop with `millis()`

The previous `delay(100)` stalled the entire CPU for 100 ms every cycle, making it impossible to handle interrupts or add future features (e.g. buttons, alarms).

The loop is now based on the `millis()` pattern:

```cpp
if (now - last_update_ms < LOOP_INTERVAL_MS) return;
last_update_ms = now;
```

The update period remains unchanged (100 ms), but the microcontroller is free to execute other code between cycles.

---

### 4. Flicker-free LCD updates

The original problem: every iteration cleared and rewrote all display rows, causing visible flickering at 10 fps.

The new `lcdUpdateLine()` function keeps a cache of the strings already shown and writes to the display **only if the content has changed**. It also uses `snprintf` with a fixed-width string to overwrite in-place without needing to clear the row first.

---

### 5. Conditional serial debug (`#define DEBUG`)

In production, serial output is unnecessary and wastes CPU cycles and flash memory. All debug code is now enclosed in `#ifdef DEBUG` blocks:

```cpp
// #define DEBUG    ← uncomment to enable
```

When `DEBUG` is not defined, the compiler completely excludes the serial code. To re-enable debug output during calibration, simply uncomment one line.

---

### 6. Horner's method for polynomial evaluation

The `evalPoly` function has been rewritten using **Horner's scheme**, which evaluates a degree-N polynomial with only N multiplications and N additions (instead of N calls to `pow()`), ideal for an 8-bit MCU without an FPU:

```cpp
// Before (loop with pow):    y += c[i] * xi;  xi *= x;
// After  (Horner's scheme):  y = y * x + c[i];
```

---

### 7. Use of `constrain()` for clamping

The chains of `if` statements used to clamp values to a range have been replaced with the Arduino `constrain(value, min, max)` function, which is more readable and less error-prone.

---

### Compatibility

No changes to external dependencies. The project still requires only:

- `LiquidCrystal_I2C` (as per `platformio.ini`)
- PlatformIO with board `nanoatmega328new`

---

### Recommended next steps

- Collect real calibration data and run `calibration_fit.py` to replace the placeholder coefficients in `PWR_COEFFS[]`.
- Consider adding polynomial regression for SWR as well (coefficients are already prepared in the source, commented out).
- Consider adding a visual alarm (flashing backlight) when SWR > 2.0 or temperature exceeds a configurable threshold.
