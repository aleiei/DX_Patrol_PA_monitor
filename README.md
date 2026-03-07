# DX Patrol PA Monitor — Documentazione
```

PA Monitor con lettura PWR, SWR e Temperatura

per Arduino Nano (ATmega328P)

Nuovo PA DX Patrol 20W — by IU0PXK

```

## 1. Panoramica del progetto
Questo repository contiene il firmware per un monitor di amplificatore di potenza (PA) da utilizzare esclusivamente con il PA DX Patrol da 20W. Il sistema misura in tempo reale la potenza in uscita (PWR), il rapporto di onde stazionarie (SWR) e la temperatura del dissipatore, visualizzando i valori su un display LCD I2C 20×4.

Il repository include tre versioni del firmware, ciascuna con funzionalità crescenti:

- v2.0.0 — Monitor base (PWR, SWR, Temperatura)

- v3.0.0-4wire — Monitor + Encoder rotativo + Controllo ventola PWM 4 fili

- v3.0.0-3wire — Monitor + Encoder rotativo + Controllo ventola 3 fili

## 2. Contenuto del repository
| File / Cartella | Descrizione |
|---------------------|-----------------------------------------------------------|
| src/main.cpp        | Firmware v2.0.0 — monitor base                            |
| main_4wire.cpp      | Firmware v3.0.0-4wire — ventola 4 fili + encoder menu     |
| main_3wire.cpp      | Firmware v3.0.0-3wire — ventola 3 fili + menù encoder     |
| calibration_fit.py  | Script Python per regressione polinomiale di calibrazione |
| platformio.ini      | Configurazione PlatformIO (board nanoatmega328new)        |
| RELEASE.md          | Note di rilascio dettagliate v2.0.0                       |

## 3. Versione base — v2.0.0

### Caratteristiche principali
- Lettura ADC con averaging su 16 campioni (riduzione rumore √16 = 4×)

- Calibrazione polinomiale per PWR tramite schema di Horner

- Calcolo SWR con metodo fisico Γ = Vref / Vfwd

- Filtro EMA sulla temperatura (α = 0.15)

- Loop non bloccante con millis() al posto di delay()

- Aggiornamento LCD differenziale (anti-flickering, cache stringhe)

- Debug seriale condizionale (#define DEBUG)

### Costanti di calibrazione
| Costante | Valore | Descrizione |
|---------------------|------------|-----------------------------------|
| TEMP_OFFSET_MV      | 464.0 mV   | Offset sensore temperatura a 0 °C |
| TEMP_MV_PER_DEG     | 6.25 mV/°C | Sensibilità sensore               |
| PWR_ADC_OFFSET_MV   | 298.0 mV   | Offset canale FWD                 |
| SWR_ADC_OFFSET_MV   | 1288.0 mV  | Offset canale REF                 |
| RAW_PWR_NOISE_FLOOR | 8 count    | Soglia rumore canale potenza      |
| RAW_SWR_NOISE_FLOOR | 4 count    | Soglia rumore canale SWR          |
| VREF_MV             | 5000.0 mV  | Tensione di riferimento ADC       |
| ADC_SAMPLES         | 16         | Campioni per averaging            |
| LOOP_INTERVAL_MS    | 100 ms     | Periodo di aggiornamento          |

### Pin hardware
| Pin Arduino | Segnale |
|-----------------|--------------------------|
| A0              | Sensore temperatura      |
| A1              | Canale SWR (REF)         |
| A2              | Canale PWR (FWD)         |
| SDA / SCL       | LCD I2C (indirizzo 0x27) |

## 4. Versione v3.0.0-4wire — Ventola 4 fili + Encoder

### Novità rispetto a v2.0.0
- Encoder rotativo su D2 (CLK/INT0), D3 (DT/INT1), D4 (SW)

- Menù impostazioni su LCD con 3 parametri regolabili

- Controllo ventola PWM hardware a ~50 kHz su D9 (OC1A / Timer1)

- Salvataggio impostazioni in EEPROM (magic byte 0xA5)

- Controllo proporzionale velocità ventola con isteresi

- Timeout automatico menù 15 s → ritorno al monitor

### Pin aggiuntivi
| Pin Arduino | Segnale |
|-----------------|------------------------------|
| D2 (INT0)       | Encoder CLK                  |
| D3 (INT1)       | Encoder DT                   |
| D4              | Encoder SW (pulsante)        |
| D9 (OC1A)       | PWM ventola 4 fili (~50 kHz) |

### Parametri menu (etichette LCD in inglese)
| Etichetta LCD | Significato | Range | Passo | Default |
|-------------------|----------------------------|-------------|-----------|-------------|
| Fan ON thresh.    | Soglia accensione ventola  | 20-85 C     | 0.5 C     | 45.0 C      |
| Fan OFF thresh.   | Soglia spegnimento ventola | 15-(ON-1) C | 0.5 C     | 40.0 C      |
| PWM minimum       | PWM minimo ventola         | 0-254       | 5         | 80 (31%)    |
| Save & Exit       | Salva in EEPROM ed esci    | -          | -        | -          |

### Schema di controllo ventola 4 fili
**Connessione diretta:** il pin D9 si collega direttamente al pin BLU della ventola (pin 4 su connettore standard PC). Non è necessario alcun MOSFET.

```
Arduino D9 (OC1A) ────────── Pin 4 BLU ventola (PWM)

Arduino GND ────────── Pin 1 NERO ventola (GND)

Pin 2 GIALLO (TACH, non usato)

+12V ────────── Pin 3 ROSSO ventola (+12V)
```

## 5. Versione v3.0.0-3wire — Ventola 3 fili + Encoder

### Differenze rispetto alla versione 4 fili
- Timer2 (8-bit) al posto di Timer1 → frequenza ~61 Hz su D11 (OC2A)

- Frequenza bassa (~61 Hz) adatta a ventole senza pin PWM dedicato

- MOSFET N-ch low-side obbligatorio sul negativo della ventola

- Soft-start configurabile: avvia al 'Kick%' per 1.2 s poi duty proporzionale

- Menù ampliato con 4 parametri (aggiunto Kick-start %)

- TACH opzionale su D5: abilita #define TACH_ENABLED per leggere RPM

- EEPROM separata (magic byte 0xB6, non compatibile con versione 4 fili)

### Parametri menu (etichette LCD in inglese)
| Etichetta LCD | Significato | Range | Passo | Default |
|-------------------|----------------------------|-------------|-----------|-------------|
| Fan ON thresh.    | Soglia accensione ventola  | 20-85 C     | 0.5 C     | 45.0 C      |
| Fan OFF thresh.   | Soglia spegnimento ventola | 15-(ON-1) C | 0.5 C     | 40.0 C      |
| Min duty          | Duty minimo ventola        | 20-95 %     | 1 %       | 40 %        |
| Kick-st           | Duty kick-start avvio      | 50-100 %    | 5 %       | 100 %       |
| Save & Exit       | Salva in EEPROM ed esci    | -          | -        | -          |

### Schema di controllo ventola 3 fili — MOSFET low-side
**Componenti:** MOSFET N-ch (es. IRLZ44N), resistenza gate 100 Ω, pull-down gate 10 kΩ.

```
+12V ─────────────────── filo ROSSO ventola (+12V)

filo NERO ventola ────── Drain MOSFET

Source MOSFET ────────── GND Arduino / GND alimentazione

Gate MOSFET ──\[100Ω\]──── Arduino D11 (OC2A)

└──\[10kΩ\]── GND (pull-down, evita floating)

filo GIALLO (TACH) ───── D5 (opzionale, #define TACH_ENABLED)
```
**Attenzione:** il GND Arduino e il GND dell'alimentazione 12V della ventola devono essere in comune. Il MOSFET deve sopportare almeno 2× la corrente nominale della ventola (margine termico).

## 6. Confronto tra le versioni
| Caratteristica | v2.0.0 | v3.0.0-4wire | v3.0.0-3W |
|----------------------|------------|------------------|-----------------|
| Monitor PWR/SWR/Temp | ✓          | ✓                | ✓               |
| Encoder + Menù       | —          | ✓                | ✓               |
| Controllo ventola    | —          | ✓                | ✓               |
| Tipo ventola         | —          | 4 fili (PWM)     | 3 fili (MOSFET) |
| Timer usato          | —          | Timer1 (D9)      | Timer2 (D11)    |
| Freq. PWM            | —          | ~50 kHz          | ~61 Hz          |
| Soft-start           | —          | —                | ✓ (1.2 s)       |
| TACH RPM             | —          | —                | Opzionale (D5)  |
| EEPROM               | —          | ✓ (0xA5)         | ✓ (0xB6)        |
| Voci menu            | —          | 3                | 4               |

## 7. Procedura di calibrazione
La calibrazione è necessaria per ottenere letture accurate di PWR. Il processo si esegue una sola volta e i coefficienti vengono salvati nel firmware.

1.  Raccogliere coppie di dati (mV_ADC, potenza_reale_W) con un wattmetro di riferimento.

2.  Modificare le liste pwr_data in calibration_fit.py con i propri dati.

3.  Eseguire lo script: python3 calibration_fit.py

4.  Copiare i coefficienti stampati nell'array PWR_COEFFS\[\] in main_4wire.cpp.

5.  Ricompilare e caricare il firmware.

**Nota:** lo script scala automaticamente i valori mV → Volt prima della regressione. I coefficienti nell'array corrispondono a: c\[0\] + c\[1\]·x + c\[2\]·x² + ... dove x è la tensione in Volt.

## 8. Compilazione e upload

### Requisiti
- PlatformIO CLI (installazione consigliata tramite pipx)

- Board: nanoatmega328new (bootloader optiboot)

- Libreria: LiquidCrystal_I2C (gestita automaticamente da platformio.ini)

### Comandi
```

# Compilazione

pio run

# Upload

pio run --target upload

# Monitor seriale (debug)

pio device monitor --baud 115200
```

## 9. Note e raccomandazioni
- Il display mostra i valori nel formato NN,NN (due cifre intere, due decimali).

- I valori piccoli vengono azzerati automaticamente per evitare oscillazioni da rumore ADC.

- Regolare le soglie RAW_PWR_NOISE_FLOOR e RAW_SWR_NOISE_FLOOR se necessario.

- La ventola rimane in stato KS (kick-start) sul display per 1.2 s all'avvio (solo versione 3 fili).

- Le impostazioni EEPROM delle due versioni v3 non sono compatibili tra loro.

### Prossimi sviluppi consigliati
- Acquisire dati reali di calibrazione ed eseguire calibration_fit.py per sostituire i coefficienti placeholder.

- Valutare l'aggiunta di regressione polinomiale anche per SWR.

- Considerare un allarme visivo (backlight lampeggiante) per SWR \> 2.0 o temperatura oltre soglia.

**Licenza:** MIT — Copyright (c) 2026 Alessandro Orlando (IU0PXK)

# DX Patrol PA Monitor — Documentation (English)
```

PA Monitor with PWR, SWR and Temperature Readout

for Arduino Nano (ATmega328P)

DX Patrol 20W PA — by IU0PXK

```

## 1. Project Overview
This repository contains firmware for a power amplifier (PA) monitor designed exclusively for the DX Patrol 20W PA. The system measures output power (PWR), standing wave ratio (SWR) and heatsink temperature in real time, displaying readings on a 20×4 I2C LCD.

The repository includes three firmware versions with progressively enhanced features:

- v2.0.0 — Basic monitor (PWR, SWR, Temperature)

- v3.0.0-4wire — Monitor + Rotary encoder + 4-wire PWM fan control

- v3.0.0-3wire — Monitor + Rotary encoder + 3-wire fan control

## 2. Repository Contents
| File / Folder | Description |
|--------------------|-----------------------------------------------------|
| src/main.cpp       | Firmware v2.0.0 — basic monitor                     |
| main_4wire.cpp     | Firmware v3.0.0-4wire — 4-wire fan + encoder menu   |
| main_3wire.cpp     | Firmware v3.0.0-3wire — 3-wire fan + encoder menu   |
| calibration_fit.py | Python script for polynomial calibration regression |
| platformio.ini     | PlatformIO configuration (board nanoatmega328new)   |
| RELEASE.md         | Detailed v2.0.0 release notes                       |

## 3. Base Version — v2.0.0

### Key Features
- ADC averaging over 16 samples (noise reduction factor √16 = 4×)

- Polynomial PWR calibration evaluated via Horner's scheme

- SWR calculated using physical method Γ = Vref / Vfwd

- Exponential moving average filter on temperature (α = 0.15)

- Non-blocking loop using millis() instead of delay()

- Differential LCD update (anti-flicker, string cache)

- Conditional serial debug output (#define DEBUG)

### Calibration Constants
| Constant | Value | Description |
|---------------------|------------|-------------------------------------------|
| TEMP_OFFSET_MV      | 464.0 mV   | Temperature sensor voltage offset at 0 °C |
| TEMP_MV_PER_DEG     | 6.25 mV/°C | Sensor sensitivity                        |
| PWR_ADC_OFFSET_MV   | 298.0 mV   | FWD channel offset                        |
| SWR_ADC_OFFSET_MV   | 1288.0 mV  | REF channel offset                        |
| RAW_PWR_NOISE_FLOOR | 8 counts   | Power channel noise threshold             |
| RAW_SWR_NOISE_FLOOR | 4 counts   | SWR channel noise threshold               |
| VREF_MV             | 5000.0 mV  | ADC reference voltage                     |
| ADC_SAMPLES         | 16         | Samples per averaging                     |
| LOOP_INTERVAL_MS    | 100 ms     | Update period                             |

### Hardware Pins
| Arduino Pin | Signal |
|-----------------|------------------------|
| A0              | Temperature sensor     |
| A1              | SWR channel (REF)      |
| A2              | PWR channel (FWD)      |
| SDA / SCL       | I2C LCD (address 0x27) |

## 4. Version v3.0.0-4wire — 4-wire Fan + Encoder

### New Features vs v2.0.0
- Rotary encoder on D2 (CLK/INT0), D3 (DT/INT1), D4 (SW)

- LCD settings menu with 3 adjustable parameters

- Hardware PWM fan control at ~50 kHz on D9 (OC1A / Timer1)

- Settings stored in EEPROM (magic byte 0xA5)

- Proportional fan speed control with hysteresis

- Automatic menu timeout after 15 s → returns to monitor

### Additional Pins
| Arduino Pin | Signal |
|-----------------|--------------------------|
| D2 (INT0)       | Encoder CLK              |
| D3 (INT1)       | Encoder DT               |
| D4              | Encoder SW (button)      |
| D9 (OC1A)       | 4-wire fan PWM (~50 kHz) |

### Menu Parameters (LCD labels)
| LCD Label | Meaning | Range | Step | Default |
|-----------------|-------------------------|-------------|----------|-------------|
| Fan ON thresh.  | Fan turn-on threshold   | 20-85 C     | 0.5 C    | 45.0 C      |
| Fan OFF thresh. | Fan turn-off threshold  | 15-(ON-1) C | 0.5 C    | 40.0 C      |
| PWM minimum     | Minimum fan PWM         | 0-254       | 5        | 80 (31%)    |
| Save & Exit     | Save to EEPROM and exit | -          | -       | -          |

### 4-wire Fan Wiring
**Direct connection:** pin D9 connects directly to the BLUE wire of the fan (pin 4 on standard PC connector). No MOSFET required.

```
Arduino D9 (OC1A) ────────── Pin 4 BLUE (PWM control)

Arduino GND ────────── Pin 1 BLACK (GND)

Pin 2 YELLOW (TACH, unused)

+12V ────────── Pin 3 RED (+12V)
```

## 5. Version v3.0.0-3wire — 3-wire Fan + Encoder

### Differences vs 4-wire Version
- Timer2 (8-bit) instead of Timer1 → frequency ~61 Hz on D11 (OC2A)

- Low frequency (~61 Hz) suitable for fans without a dedicated PWM pin

- N-channel MOSFET low-side switching required on the fan negative wire

- Configurable soft-start: runs at 'Kick%' for 1.2 s then proportional duty

- Extended menu with 4 parameters (Kick-start % added)

- Optional TACH on D5: enable #define TACH_ENABLED to read RPM

- Separate EEPROM layout (magic byte 0xB6, incompatible with 4-wire version)

### Menu Parameters (LCD labels)
| LCD Label | Meaning | Range | Step | Default |
|-----------------|-------------------------|-------------|----------|-------------|
| Fan ON thresh.  | Fan turn-on threshold   | 20-85 C     | 0.5 C    | 45.0 C      |
| Fan OFF thresh. | Fan turn-off threshold  | 15-(ON-1) C | 0.5 C    | 40.0 C      |
| Min duty        | Minimum fan duty cycle  | 20-95 %     | 1 %      | 40 %        |
| Kick-st         | Kick-start duty         | 50-100 %    | 5 %      | 100 %       |
| Save & Exit     | Save to EEPROM and exit | -          | -       | -          |

### 3-wire Fan Wiring — MOSFET Low-side
**Components:** N-ch MOSFET (e.g. IRLZ44N), 100 Ω gate resistor, 10 kΩ gate pull-down.

```
+12V ─────────────────── RED wire (+12V)

BLACK wire ───────────── MOSFET Drain

MOSFET Source ────────── Arduino GND / supply GND (common)

MOSFET Gate ──\[100Ω\]──── Arduino D11 (OC2A)

└──\[10kΩ\]── GND (pull-down, prevents floating)

YELLOW wire (TACH) ───── D5 (optional, #define TACH_ENABLED)
```
**Important:** Arduino GND and 12V supply GND must share a common ground. The MOSFET must handle at least 2× the fan's rated current for adequate thermal margin.

## 6. Version Comparison
| Feature | v2.0.0 | v3.0.0-4wire | v3.0.0-3W |
|----------------------|------------|------------------|-----------------|
| PWR/SWR/Temp monitor | ✓          | ✓                | ✓               |
| Encoder + Menu       | —          | ✓                | ✓               |
| Fan control          | —          | ✓                | ✓               |
| Fan type             | —          | 4-wire (PWM)     | 3-wire (MOSFET) |
| Timer used           | —          | Timer1 (D9)      | Timer2 (D11)    |
| PWM frequency        | —          | ~50 kHz          | ~61 Hz          |
| Soft-start           | —          | —                | ✓ (1.2 s)       |
| TACH RPM             | —          | —                | Optional (D5)   |
| EEPROM               | —          | ✓ (0xA5)         | ✓ (0xB6)        |
| Menu items           | —          | 3                | 4               |

## 7. Calibration Procedure
Calibration is required for accurate PWR readings. It is a one-time process; coefficients are then hard-coded into the firmware.

6.  Collect (mV_ADC, actual_power_W) data pairs using a reference wattmeter.

7.  Edit the pwr_data list in calibration_fit.py with your measurements.

8.  Run the script: python3 calibration_fit.py

9.  Copy the printed coefficients into the PWR_COEFFS\[\] array in main.cpp.

10. Recompile and upload the firmware.

**Note:** the script automatically scales mV → Volts before regression. Array coefficients follow: c\[0\] + c\[1\]·x + c\[2\]·x² + ... where x is voltage in Volts.

## 8. Build and Upload

### Requirements
- PlatformIO CLI (recommended: install via pipx)

- Board: nanoatmega328new (optiboot bootloader)

- Library: LiquidCrystal_I2C (handled automatically by platformio.ini)

### Commands
```

# Build

pio run

# Upload

pio run --target upload

# Serial monitor (debug)

pio device monitor --baud 115200
```

## 9. Notes and Recommendations
- The display shows values in NN,NN format (two integer digits, two decimal places).

- Small values are automatically zeroed to prevent ADC noise oscillations.

- Adjust RAW_PWR_NOISE_FLOOR and RAW_SWR_NOISE_FLOOR thresholds as needed.

- The fan shows KS (kick-start) on the display for 1.2 s at startup (3-wire version only).

- EEPROM settings of the two v3 versions are not mutually compatible.

### Recommended Next Steps
- Collect real calibration data and run calibration_fit.py to replace placeholder coefficients.

- Consider adding polynomial regression for SWR as well.

- Consider a visual alarm (flashing backlight) when SWR \> 2.0 or temperature exceeds threshold.

**License:** MIT — Copyright (c) 2026 Alessandro Orlando (IU0PXK)
