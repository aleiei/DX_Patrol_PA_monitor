# DX_Patrol_PA_monitor

Progetto: PA monitor con lettura (PWR e SWR) e temperatura per Arduino Nano (ATmega328P) da utilizzare esclusivamente con il nuovo PA DX Patrol da 20W by IU0PXK.

Contenuti principali:
- `src/main.cpp` — sketch principale con lettura ADC, calibrazione polinomiale, formattazione display LCD I2C e output seriale per debug.
- `calibration_fit.py` — script Python per adattare polinomi di calibrazione a partire da dati di misura (mV → valore reale).
- `platformio.ini` — configurazione PlatformIO per `nanoatmega328new`.

## Compilazione e upload

Assicurati di avere PlatformIO CLI funzionante (consigliato installazione tramite `pipx`).

## Calibrazione

1. Raccogli coppie di dati `(mV, valore_reale)` per PWR e SWR.
2. Modifica `pwr_data` e `swr_data` in `calibration_fit.py` e imposta il grado del polinomio.
3. Esegui `python3 calibration_fit.py` per ottenere i coefficienti.
4. Copia i coefficienti negli array `pwrCoeffs` e `swrCoeffs` in `src/main.cpp`.

## Note

- Il display mostra sempre i valori in formato `NN,NN` (due cifre intere, due decimali) e i valori piccoli vengono azzerati per evitare oscillazioni dovute al rumore ADC.
- Se occorre, regolare le soglie `raw_pwr < 8` o `pwr_voltage < 0.001f` nel codice.

## Licenza

MIT.

---

---

# DX_Patrol_PA_monitor *(English)*

Project: PA monitor with PWR, SWR and temperature readout for Arduino Nano (ATmega328P), intended exclusively for use with the new DX Patrol 20W PA by IU0PXK.

Main contents:
- `src/main.cpp` — main sketch with ADC reading, polynomial calibration, I2C LCD display formatting and serial debug output.
- `calibration_fit.py` — Python script to fit calibration polynomials from measurement data (mV → real value).
- `platformio.ini` — PlatformIO configuration for `nanoatmega328new`.

## Build and upload

Make sure you have a working PlatformIO CLI installation (recommended: install via `pipx`).

## Calibration

1. Collect `(mV, actual_value)` data pairs for PWR and SWR.
2. Edit `pwr_data` and `swr_data` in `calibration_fit.py` and set the polynomial degree.
3. Run `python3 calibration_fit.py` to obtain the coefficients.
4. Copy the coefficients into the `pwrCoeffs` and `swrCoeffs` arrays in `src/main.cpp`.

## Notes

- The display always shows values in `NN,NN` format (two integer digits, two decimal places); small values are zeroed out to avoid oscillations caused by ADC noise.
- If needed, adjust the thresholds `raw_pwr < 8` or `pwr_voltage < 0.001f` in the source code.

## License

MIT.
