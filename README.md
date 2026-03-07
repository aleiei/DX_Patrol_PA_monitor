# DX Patrol PA Monitor

PA monitor with PWR, SWR, and temperature readout for Arduino Nano (ATmega328P), designed for the DX Patrol 20W PA (IU0PXK).

## 1. Project Overview
This repository contains firmware for a power amplifier monitor that measures:
- Output power (PWR)
- Standing wave ratio (SWR)
- Heatsink temperature

Readings are displayed on a 20x4 I2C LCD.

Available firmware variants:
- `v2.0.0`: Base monitor (PWR, SWR, Temperature)
- `v3.0.0-4wire`: Base monitor + rotary encoder menu + 4-wire PWM fan control
- `v3.0.0-3wire`: Base monitor + rotary encoder menu + 3-wire fan control

## 2. Repository Contents
| File / Folder | Description |
|---|---|
| `src/main.cpp` | Firmware `v2.0.0` (base monitor) |
| `src/main_4wire.cpp` | Firmware `v3.0.0-4wire` |
| `src/main_3wire.cpp` | Firmware `v3.0.0-3wire` |
| `calibration_fit.py` | Polynomial calibration helper script |
| `platformio.ini` | PlatformIO project configuration |
| `RELEASE.md` | Release notes |

## 3. Base Version (`v2.0.0`)
### Key Features
- ADC averaging over 16 samples (noise reduction)
- Polynomial PWR calibration (Horner evaluation)
- Physical SWR computation (`gamma = Vref / Vfwd`)
- Exponential moving average on temperature (`alpha = 0.15`)
- Non-blocking loop with `millis()`
- Differential LCD update to reduce flicker
- Optional serial debug via `#define DEBUG`

### Calibration Constants
| Constant | Value | Description |
|---|---|---|
| `TEMP_OFFSET_MV` | `464.0 mV` | Temperature sensor offset at 0 C |
| `TEMP_MV_PER_DEG` | `6.25 mV/C` | Sensor sensitivity |
| `PWR_ADC_OFFSET_MV` | `298.0 mV` | FWD channel offset |
| `SWR_ADC_OFFSET_MV` | `1288.0 mV` | REF channel offset |
| `RAW_PWR_NOISE_FLOOR` | `8` | ADC threshold for power channel |
| `RAW_SWR_NOISE_FLOOR` | `4` | ADC threshold for SWR channel |
| `VREF_MV` | `5000.0 mV` | ADC reference |
| `ADC_SAMPLES` | `16` | Samples per average |
| `LOOP_INTERVAL_MS` | `100 ms` | Monitor update period |

### Hardware Pins
| Arduino Pin | Signal |
|---|---|
| `A0` | Temperature sensor |
| `A1` | SWR channel (REF) |
| `A2` | Power channel (FWD) |
| `SDA/SCL` | I2C LCD (`0x27`) |

## 4. `v3.0.0-4wire` (4-Wire Fan + Encoder)
### Added Features
- Rotary encoder on `D2` (CLK), `D3` (DT), `D4` (SW)
- LCD settings menu (3 editable parameters)
- Hardware PWM fan control on `D9` (`OC1A`, Timer1)
- EEPROM settings persistence (`magic byte 0xA5`)
- Hysteresis-based fan control
- Menu timeout after 15 s

### Extra Pins
| Arduino Pin | Signal |
|---|---|
| `D2 (INT0)` | Encoder CLK |
| `D3 (INT1)` | Encoder DT |
| `D4` | Encoder switch |
| `D9 (OC1A)` | 4-wire fan PWM |

### 4-Wire Fan Menu Parameters
| LCD Label | Meaning | Range | Step | Default |
|---|---|---|---|---|
| `Fan ON thresh.` | Fan turn-on threshold | 20-85 C | 0.5 C | 45.0 C |
| `Fan OFF thresh.` | Fan turn-off threshold | 15-(ON-1) C | 0.5 C | 40.0 C |
| `PWM minimum` | Minimum PWM output | 0-254 | 5 | 80 |
| `Save & Exit` | Save settings and exit | - | - | - |

### 4-Wire Wiring
Direct PWM connection (no MOSFET required):

```text
Arduino D9 (OC1A)  -> Fan pin 4 (BLUE, PWM)
Arduino GND        -> Fan pin 1 (BLACK, GND)
+12V supply        -> Fan pin 3 (RED, +12V)
Fan pin 2 (YELLOW, TACH) optional / unused
```

## 5. `v3.0.0-3wire` (3-Wire Fan + Encoder)
### Differences vs 4-Wire
- Timer2 PWM on `D11` (`OC2A`) at about 61 Hz
- N-channel MOSFET low-side switching required
- Kick-start support (high duty at startup)
- Optional TACH readout on `D5`
- EEPROM layout for this variant (`magic byte 0xB6`)

### 3-Wire Fan Menu Parameters
| LCD Label | Meaning | Range | Step | Default |
|---|---|---|---|---|
| `Fan ON thresh.` | Fan turn-on threshold | 20-85 C | 0.5 C | 45.0 C |
| `Fan OFF thresh.` | Fan turn-off threshold | 15-(ON-1) C | 0.5 C | 40.0 C |
| `Min duty` | Minimum running duty | 20-95 % | 1 % | 40 % |
| `Kick-st` | Startup kick duty | 50-100 % | 5 % | 100 % |
| `Save & Exit` | Save settings and exit | - | - | - |

### 3-Wire Wiring (MOSFET low-side)
Recommended components: N-channel MOSFET (for example IRLZ44N), 100 ohm gate resistor, 10 kohm gate pull-down.

```text
+12V supply         -> Fan RED wire
Fan BLACK wire      -> MOSFET Drain
MOSFET Source       -> Common GND (Arduino + 12V supply)
Arduino D11 (OC2A)  -> 100 ohm -> MOSFET Gate
MOSFET Gate         -> 10 kohm -> GND (pull-down)
Fan YELLOW (TACH)   -> D5 (optional with TACH enabled)
```

## 6. Version Comparison
| Feature | `v2.0.0` | `v3.0.0-4wire` | `v3.0.0-3wire` |
|---|---|---|---|
| PWR/SWR/Temperature monitor | Yes | Yes | Yes |
| Encoder + menu | No | Yes | Yes |
| Fan control | No | Yes | Yes |
| Fan type | - | 4-wire PWM | 3-wire MOSFET |
| PWM source | - | Timer1 (`D9`) | Timer2 (`D11`) |
| Typical PWM frequency | - | about 50 kHz | about 61 Hz |
| Kick-start | - | No | Yes |
| TACH RPM | - | No | Optional (`D5`) |
| EEPROM settings | - | Yes (`0xA5`) | Yes (`0xB6`) |

## 7. Calibration Procedure
1. Collect `(mV_ADC, real_power_W)` pairs with a trusted wattmeter.
2. Update the data list in `calibration_fit.py`.
3. Run:
   - `python3 calibration_fit.py`
4. Copy printed coefficients into `PWR_COEFFS[]` in the target firmware file.
5. Rebuild and upload.

## 8. Build and Upload
### Requirements
- PlatformIO CLI
- Board: `nanoatmega328new`
- Library: `LiquidCrystal_I2C` (resolved by PlatformIO)

### Commands
```bash
# Build
pio run

# Upload
pio run --target upload

# Serial monitor
pio device monitor --baud 115200
```

## 9. Notes
- Display format uses comma-style decimal output (for example `12,34`).
- Small values are forced to zero to suppress ADC noise jitter.
- Tune `RAW_PWR_NOISE_FLOOR` and `RAW_SWR_NOISE_FLOOR` if needed.
- In the 3-wire variant, `KS` is shown during kick-start time.
- `v3.0.0-4wire` and `v3.0.0-3wire` EEPROM layouts are intentionally incompatible.

## License
MIT License

Copyright (c) 2026 Alessandro Orlando (IU0PXK)
