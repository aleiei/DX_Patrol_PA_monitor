/**
 * main.cpp
 * PA Monitor — PWR, SWR and temperature for Arduino Nano (ATmega328P)
 * New 20W PA by IU0PXK
 *
 * Improvements over the original version:
 *  - Symbolic constants for all magic numbers
 *  - ADC averaging on N samples to reduce noise
 *  - Non-blocking loop with millis() instead of delay()
 *  - LCD update only when values change (removes flicker)
 *  - Serial debug output controlled by #define DEBUG
 *  - Polynomial coefficients for PWR and SWR in dedicated structures
 *  - Well-separated, documented helper functions
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Build-time options
// ---------------------------------------------------------------------------
// Uncomment to enable serial debug output
// #define DEBUG

// ---------------------------------------------------------------------------
// Hardware / ADC
// ---------------------------------------------------------------------------
static constexpr uint8_t  PIN_TEMP   = A0;
static constexpr uint8_t  PIN_SWR    = A1;
static constexpr uint8_t  PIN_PWR    = A2;

static constexpr uint16_t ADC_MAX    = 1023;
static constexpr float    VREF_MV    = 5000.0f;          // ADC reference voltage in mV
static constexpr float    ADC_MV_PER_STEP = VREF_MV / ADC_MAX;

static constexpr uint8_t  ADC_SAMPLES = 16;              // samples for averaging

// ---------------------------------------------------------------------------
// Temperature (LM35 or equivalent: Vout = 10 mV/°C, offset ~464 mV)
// ---------------------------------------------------------------------------
static constexpr float TEMP_OFFSET_MV   = 464.0f;        // mV at sensor 0 C
static constexpr float TEMP_MV_PER_DEG  = 6.25f;         // mV / °C
static constexpr float TEMP_FILTER_ALPHA = 0.15f;        // EMA: 0=frozen, 1=no filtering

// ---------------------------------------------------------------------------
// ADC offsets for FWD/REF channels (from calibration)
// ---------------------------------------------------------------------------
static constexpr float PWR_ADC_OFFSET_MV = 298.0f;       // FWD channel offset in mV
static constexpr float SWR_ADC_OFFSET_MV = 1288.0f;      // REF channel offset in mV

// ---------------------------------------------------------------------------
// Noise thresholds
// ---------------------------------------------------------------------------
static constexpr uint16_t RAW_PWR_NOISE_FLOOR = 8;       // raw ADC below which PWR = 0
static constexpr uint16_t RAW_SWR_NOISE_FLOOR = 4;       // raw ADC below which REF = 0
static constexpr float    PWR_MIN_WATTS        = 0.001f; // W below which value is considered zero
static constexpr float    EPS                  = 1e-6f;

// ---------------------------------------------------------------------------
// Display limits
// ---------------------------------------------------------------------------
static constexpr float PWR_MAX_DISPLAY = 99.995f;        // max W shown (format XX,XX)
static constexpr float SWR_MAX_DISPLAY = 9.995f;         // max SWR shown (format X,XX)

// ---------------------------------------------------------------------------
// Non-blocking loop timing
// ---------------------------------------------------------------------------
static constexpr uint32_t LOOP_INTERVAL_MS = 100;        // ms between updates

// ---------------------------------------------------------------------------
// Calibration polynomials
//   Form: y = c[0] + c[1]*x + c[2]*x^2 + ...
//   x = adjusted voltage in Volts
//
//   REPLACE with coefficients from calibration_fit.py
// ---------------------------------------------------------------------------
static constexpr float PWR_COEFFS[] = { 0.0f, 0.0f, 1.0f }; // quadratic placeholder
static constexpr int   PWR_DEG      = (sizeof(PWR_COEFFS) / sizeof(PWR_COEFFS[0])) - 1;

// For SWR, the physical method (gamma), but if regression is preferred
// uncomment the following block and comment out gamma calculation in loop.
// static constexpr float SWR_COEFFS[] = { 1.0f, 0.0f, 0.0f };
// static constexpr int   SWR_DEG      = (sizeof(SWR_COEFFS) / sizeof(SWR_COEFFS[0])) - 1;

// ---------------------------------------------------------------------------
// 20x4 I2C LCD object
// ---------------------------------------------------------------------------
static LiquidCrystal_I2C MyLCD(0x27, 20, 4);

// ---------------------------------------------------------------------------
// Persistent state
// ---------------------------------------------------------------------------
static float    temp_filtered_c        = 0.0f;
static bool     temp_filter_initialized = false;

// Cache previous values to update the LCD only when they change
static char     prev_temp_str[8]  = "";
static char     prev_swr_str[8]   = "";
static char     prev_pwr_str[8]   = "";

static uint32_t last_update_ms = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Run ADC averaging on @samples readings from pin @pin.
 * Returns the mean value (floating point).
 */
static float adcAverage(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
        sum += analogRead(pin);
    }
    return static_cast<float>(sum) / samples;
}

/**
 * Evaluate the polynomial c[0] + c[1]*x + ... + c[deg]*x^deg
 * using Horner's method (efficient on MCU).
 */
static float evalPoly(float x, const float *c, int deg) {
    float y = c[deg];
    for (int i = deg - 1; i >= 0; --i) {
        y = y * x + c[i];
    }
    return y;
}

/**
 * Write @text on LCD row @row only if different from previous value.
 * @prev_buf must be a buffer of at least strlen(@text)+1 bytes.
 */
static void lcdUpdateLine(uint8_t row, const char *label, const char *value_str, char *prev_buf) {
    if (strcmp(value_str, prev_buf) == 0) return;    // no change, skip
    strncpy(prev_buf, value_str, 7);
    prev_buf[7] = '\0';

    MyLCD.setCursor(0, row);
    // Print label + value + padding to clear any leftover characters
    char line[21];
    snprintf(line, sizeof(line), "%-8s%-12s", label, value_str);
    MyLCD.print(line);
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
    MyLCD.init();
    MyLCD.backlight();
    MyLCD.setCursor(0, 0);
    MyLCD.print("PA Monitor by IU0PXK");

#ifdef DEBUG
    Serial.begin(115200);
    Serial.println(F("PA Monitor DEBUG start"));
#endif
}

void loop() {
    // Non-blocking loop: run only when LOOP_INTERVAL_MS has elapsed
    uint32_t now = millis();
    if (now - last_update_ms < LOOP_INTERVAL_MS) return;
    last_update_ms = now;

    // -----------------------------------------------------------------------
    // 1. ADC read with averaging
    // -----------------------------------------------------------------------
    float raw_temp = adcAverage(PIN_TEMP, ADC_SAMPLES);
    float raw_swr  = adcAverage(PIN_SWR,  ADC_SAMPLES);
    float raw_pwr  = adcAverage(PIN_PWR,  ADC_SAMPLES);

    // -----------------------------------------------------------------------
    // 2. Temperature
    // -----------------------------------------------------------------------
    float temp_mV = raw_temp * ADC_MV_PER_STEP;
    float temp_c  = (temp_mV - TEMP_OFFSET_MV) / TEMP_MV_PER_DEG;

    if (!temp_filter_initialized) {
        temp_filtered_c = temp_c;
        temp_filter_initialized = true;
    } else {
        temp_filtered_c += TEMP_FILTER_ALPHA * (temp_c - temp_filtered_c);
    }

    // -----------------------------------------------------------------------
    // 3. Power (PWR) — polynomial regression
    // -----------------------------------------------------------------------
    float pwr_mV  = raw_pwr * ADC_MV_PER_STEP;
    float pwr_adj = pwr_mV - PWR_ADC_OFFSET_MV;
    float x_pwr   = pwr_adj / 1000.0f;              // scale to Volts for the polynomial

    float pwr_watts = evalPoly(x_pwr, PWR_COEFFS, PWR_DEG);

    // Fallback: if polynomial gives non-physical value, use empirical quadratic formula
    if (!isfinite(pwr_watts) || pwr_watts < 0.0f || pwr_watts > 200.0f) {
        pwr_watts = (pwr_adj * pwr_adj) / 1000000.0f;
    }

    // Noise threshold: below ADC floor, force zero
    if (raw_pwr < RAW_PWR_NOISE_FLOOR || pwr_watts < PWR_MIN_WATTS) {
        pwr_watts = 0.0f;
    }

    // -----------------------------------------------------------------------
    // 4. SWR — physical method (gamma = Vref / Vfwd)
    // -----------------------------------------------------------------------
    float swr_mV  = raw_swr * ADC_MV_PER_STEP;
    float swr_adj = swr_mV - SWR_ADC_OFFSET_MV;

    float swr_val = 1.0f;   // default: perfect SWR

    bool tx_active = (raw_pwr >= RAW_PWR_NOISE_FLOOR)
                  && (pwr_watts >= PWR_MIN_WATTS)
                  && (raw_swr  >= RAW_SWR_NOISE_FLOOR);

    if (tx_active) {
        float fwd = (pwr_adj > 0.0f) ? pwr_adj : 0.0f;
        float ref = (swr_adj > 0.0f) ? swr_adj : 0.0f;

        if (fwd > EPS) {
            float gamma = ref / fwd;
            gamma = constrain(gamma, 0.0f, 0.98f);
            swr_val = (1.0f + gamma) / (1.0f - gamma);
        }
    }

    if (!isfinite(swr_val) || swr_val < 1.0f) swr_val = 1.0f;
    if (swr_val > SWR_MAX_DISPLAY + 0.005f) swr_val = SWR_MAX_DISPLAY;

    // -----------------------------------------------------------------------
    // 5. Display string formatting
    // -----------------------------------------------------------------------
    char temp_str[8], swr_str[8], pwr_str[8];

    // Temperature: one decimal
    dtostrf(temp_filtered_c, 5, 1, temp_str);

    // SWR: X,XX
    {
        float sv = constrain(swr_val, 1.0f, SWR_MAX_DISPLAY);
        int isv = (int)sv;
        int dsv = (int)roundf((sv - isv) * 100.0f);
        if (dsv >= 100) { isv++; dsv = 0; }
        if (isv > 9)    { isv = 9; dsv = 99; }
        snprintf(swr_str, sizeof(swr_str), "%1d,%02d", isv, dsv);
    }

    // PWR: XX,XX
    {
        float pv = constrain(pwr_watts, 0.0f, PWR_MAX_DISPLAY);
        int ip = (int)pv;
        int dp = (int)roundf((pv - ip) * 100.0f);
        if (dp >= 100) { ip++; dp = 0; }
        if (ip > 99)   { ip = 99; dp = 99; }
        snprintf(pwr_str, sizeof(pwr_str), "%02d,%02d", ip, dp);
    }

    // -----------------------------------------------------------------------
    // 6. LCD update (only if values changed)
    // -----------------------------------------------------------------------
    lcdUpdateLine(1, "TEMP C= ", temp_str, prev_temp_str);
    lcdUpdateLine(2, "SWR   = ", swr_str,  prev_swr_str);
    lcdUpdateLine(3, "PWR W = ", pwr_str,  prev_pwr_str);

    // -----------------------------------------------------------------------
    // 7. Serial debug (only if #define DEBUG is active)
    // -----------------------------------------------------------------------
#ifdef DEBUG
    Serial.print(F("raw_pwr="));  Serial.print(raw_pwr);
    Serial.print(F(" pwr_mV="));  Serial.print(pwr_mV);
    Serial.print(F(" pwr_adj=")); Serial.print(pwr_adj);
    Serial.print(F(" pwr_W="));   Serial.print(pwr_watts);
    Serial.print(F(" | raw_swr=")); Serial.print(raw_swr);
    Serial.print(F(" swr_mV="));  Serial.print(swr_mV);
    Serial.print(F(" swr_adj=")); Serial.print(swr_adj);
    Serial.print(F(" swr="));     Serial.println(swr_val);
#endif
}
