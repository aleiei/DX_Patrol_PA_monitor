# DX_Patrol_PA_monitor

Progetto: PA monitor con lettura (PWR e SWR) e temperatura per Arduino Nano (ATmega328P) da utilizzare esclusivamente con il nuovo PA da 20W by IU0PXK.

Contenuti principali:
- `src/IU0PXK_PA_Monitor.cpp` — sketch principale con lettura ADC, calibrazione polinomiale, formattazione display LCD I2C e output seriale per debug.
- `calibration_fit.py` — script Python per adattare polinomi di calibrazione a partire da dati di misura (mV → valore reale).
- `platformio.ini` — configurazione PlatformIO per `nanoatmega328new`.

Compilazione e upload

Assicurati di avere PlatformIO CLI funzionante (consigliato installazione tramite `pipx`).


Calibrazione

1. Raccogli coppie di dati `(mV, valore_reale)` per PWR e SWR.
2. Modifica `pwr_data` e `swr_data` in `calibration_fit.py` e imposta il grado del polinomio.
3. Esegui `python3 calibration_fit.py` per ottenere i coefficienti.
4. Copia i coefficienti negli array `pwrCoeffs` e `swrCoeffs` in `src/IU0PXK_PA_Monitor.cpp`.

Note

- Il display mostra sempre i valori in formato `NN,NN` (due cifre intere, due decimali) e i valori piccoli vengono azzerati per evitare oscillazioni dovute al rumore ADC.
- Se occorre, regolare le soglie `raw_pwr < 8` o `pwr_voltage < 0.001f` nel codice.

Licenza 

MIT.
