# RELEASE NOTES — IU0PXK PA Monitor

## v3.0.0-4wire — 2026-03-07

### Summary
Firmware variant for 4-wire PC-style fans with direct PWM control from Arduino Nano.

### Highlights
- Rotary encoder UI (`D2/D3/D4`) with settings menu
- Timer1 hardware PWM output on `D9 (OC1A)`
- High-frequency PWM suitable for 4-wire fan control
- Fan ON/OFF thresholds with hysteresis
- EEPROM persistence for fan settings (`magic byte 0xA5`)
- Differential LCD updates and non-blocking monitor loop

### Menu Parameters
- `Fan ON thresh.`
- `Fan OFF thresh.`
- `PWM minimum`
- `Save & Exit`

### Notes
- 4-wire control is direct PWM signal control (no MOSFET required for PWM pin).
- This EEPROM format is not compatible with the 3-wire variant.

---

## v3.0.0-3wire — 2026-03-07

### Summary
Firmware variant for 3-wire fans driven through low-side MOSFET PWM control.

### Highlights
- Rotary encoder UI (`D2/D3/D4`) with settings menu
- Timer2 PWM output on `D11 (OC2A)`
- Kick-start support for reliable fan spin-up
- Optional TACH RPM readout (`D5`, if enabled)
- EEPROM persistence for fan settings (`magic byte 0xB6`)
- Differential LCD updates and non-blocking monitor loop

### Menu Parameters
- `Fan ON thresh.`
- `Fan OFF thresh.`
- `Min duty`
- `Kick-st`
- `Save & Exit`

### Notes
- Requires external N-channel MOSFET low-side switching.
- This EEPROM format is not compatible with the 4-wire variant.

---

## v2.0.0 — 2026-03-06

### Summary
Complete refactoring of the original monitor firmware.

### Highlights
- Symbolic `constexpr` constants replaced magic numbers
- ADC averaging (`ADC_SAMPLES = 16`) for lower noise
- Non-blocking scheduler based on `millis()`
- Flicker-reduced LCD updates with line caching
- Optional serial debug guarded by `#define DEBUG`
- Efficient polynomial evaluation with Horner's method
- Cleaner range clamping with `constrain()`

### Compatibility
- `LiquidCrystal_I2C`
- PlatformIO board: `nanoatmega328new`

### Recommended Follow-up
- Replace placeholder calibration coefficients using real measurements (`calibration_fit.py`)
- Consider adding SWR polynomial regression if desired
- Consider visual alarms for high SWR / over-temperature
