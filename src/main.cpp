/**
 * main.cpp
 * PA Monitor — PWR, SWR e temperatura per Arduino Nano (ATmega328P)
 * Nuovo PA da 20W by IU0PXK
 *
 * Miglioramenti rispetto alla versione originale:
 *  - Costanti simboliche per tutti i magic numbers
 *  - ADC averaging su N campioni per ridurre il rumore
 *  - Loop non bloccante con millis() al posto di delay()
 *  - Aggiornamento LCD solo se il valore cambia (elimina flickering)
 *  - Output seriale di debug condizionato a #define DEBUG
 *  - Coefficienti polinomiali per PWR e SWR in strutture dedicate
 *  - Funzioni helper ben separate e documentate
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Build-time options
// ---------------------------------------------------------------------------
// Decommentare per abilitare l'output seriale di debug
// #define DEBUG

// ---------------------------------------------------------------------------
// Hardware / ADC
// ---------------------------------------------------------------------------
static constexpr uint8_t  PIN_TEMP   = A0;
static constexpr uint8_t  PIN_SWR    = A1;
static constexpr uint8_t  PIN_PWR    = A2;

static constexpr uint16_t ADC_MAX    = 1023;
static constexpr float    VREF_MV    = 5000.0f;          // tensione di riferimento ADC in mV
static constexpr float    ADC_MV_PER_STEP = VREF_MV / ADC_MAX;

static constexpr uint8_t  ADC_SAMPLES = 16;              // campioni per averaging

// ---------------------------------------------------------------------------
// Temperatura (LM35 o equivalente: Vout = 10 mV/°C, offset ~464 mV)
// ---------------------------------------------------------------------------
static constexpr float TEMP_OFFSET_MV   = 464.0f;        // mV a 0 °C del sensore
static constexpr float TEMP_MV_PER_DEG  = 6.25f;         // mV / °C
static constexpr float TEMP_FILTER_ALPHA = 0.15f;        // EMA: 0=congelato, 1=nessun filtro

// ---------------------------------------------------------------------------
// Offsets ADC per i canali FWD/REF (da calibrazione)
// ---------------------------------------------------------------------------
static constexpr float PWR_ADC_OFFSET_MV = 298.0f;       // offset canale FWD in mV
static constexpr float SWR_ADC_OFFSET_MV = 1288.0f;      // offset canale REF in mV

// ---------------------------------------------------------------------------
// Soglie di rumore
// ---------------------------------------------------------------------------
static constexpr uint16_t RAW_PWR_NOISE_FLOOR = 8;       // ADC raw sotto il quale PWR = 0
static constexpr uint16_t RAW_SWR_NOISE_FLOOR = 4;       // ADC raw sotto il quale REF = 0
static constexpr float    PWR_MIN_WATTS        = 0.001f; // W sotto i quali si considera zero
static constexpr float    EPS                  = 1e-6f;

// ---------------------------------------------------------------------------
// Limiti display
// ---------------------------------------------------------------------------
static constexpr float PWR_MAX_DISPLAY = 99.995f;        // W massimi mostrati (formato XX,XX)
static constexpr float SWR_MAX_DISPLAY = 9.995f;         // SWR massimo mostrato (formato X,XX)

// ---------------------------------------------------------------------------
// Timing loop non bloccante
// ---------------------------------------------------------------------------
static constexpr uint32_t LOOP_INTERVAL_MS = 100;        // ms tra un aggiornamento e l'altro

// ---------------------------------------------------------------------------
// Polinomi di calibrazione
//   Forma: y = c[0] + c[1]*x + c[2]*x^2 + ...
//   x = tensione adjusted in Volt
//
//   SOSTITUIRE con i coefficienti ottenuti da calibration_fit.py
// ---------------------------------------------------------------------------
static constexpr float PWR_COEFFS[] = { 0.0f, 0.0f, 1.0f }; // placeholder quadratico
static constexpr int   PWR_DEG      = (sizeof(PWR_COEFFS) / sizeof(PWR_COEFFS[0])) - 1;

// Per SWR si usa il metodo fisico (gamma), ma se si preferisce la regressione
// decommentare il blocco seguente e commentare il calcolo gamma nel loop.
// static constexpr float SWR_COEFFS[] = { 1.0f, 0.0f, 0.0f };
// static constexpr int   SWR_DEG      = (sizeof(SWR_COEFFS) / sizeof(SWR_COEFFS[0])) - 1;

// ---------------------------------------------------------------------------
// Oggetto LCD 20x4 I2C
// ---------------------------------------------------------------------------
static LiquidCrystal_I2C MyLCD(0x27, 20, 4);

// ---------------------------------------------------------------------------
// Stato persistente
// ---------------------------------------------------------------------------
static float    temp_filtered_c        = 0.0f;
static bool     temp_filter_initialized = false;

// Cache dei valori precedenti per aggiornare il LCD solo se cambiano
static char     prev_temp_str[8]  = "";
static char     prev_swr_str[8]   = "";
static char     prev_pwr_str[8]   = "";

static uint32_t last_update_ms = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/**
 * Esegue ADC averaging su @samples letture del pin @pin.
 * Restituisce il valore medio (floating point).
 */
static float adcAverage(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) {
        sum += analogRead(pin);
    }
    return static_cast<float>(sum) / samples;
}

/**
 * Valuta il polinomio c[0] + c[1]*x + ... + c[deg]*x^deg
 * usando lo schema di Horner (efficiente su MCU).
 */
static float evalPoly(float x, const float *c, int deg) {
    float y = c[deg];
    for (int i = deg - 1; i >= 0; --i) {
        y = y * x + c[i];
    }
    return y;
}

/**
 * Scrive @text sulla riga @row del LCD solo se diverso dal valore precedente.
 * @prev_buf deve essere un buffer di almeno strlen(@text)+1 byte.
 */
static void lcdUpdateLine(uint8_t row, const char *label, const char *value_str, char *prev_buf) {
    if (strcmp(value_str, prev_buf) == 0) return;    // nessun cambiamento, skip
    strncpy(prev_buf, value_str, 7);
    prev_buf[7] = '\0';

    MyLCD.setCursor(0, row);
    // Stampa label + valore + padding per pulire eventuali caratteri residui
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
    // Loop non bloccante: esegui solo se è trascorso LOOP_INTERVAL_MS
    uint32_t now = millis();
    if (now - last_update_ms < LOOP_INTERVAL_MS) return;
    last_update_ms = now;

    // -----------------------------------------------------------------------
    // 1. Lettura ADC con averaging
    // -----------------------------------------------------------------------
    float raw_temp = adcAverage(PIN_TEMP, ADC_SAMPLES);
    float raw_swr  = adcAverage(PIN_SWR,  ADC_SAMPLES);
    float raw_pwr  = adcAverage(PIN_PWR,  ADC_SAMPLES);

    // -----------------------------------------------------------------------
    // 2. Temperatura
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
    // 3. Potenza (PWR) — regressione polinomiale
    // -----------------------------------------------------------------------
    float pwr_mV  = raw_pwr * ADC_MV_PER_STEP;
    float pwr_adj = pwr_mV - PWR_ADC_OFFSET_MV;
    float x_pwr   = pwr_adj / 1000.0f;              // scala in Volt per il polinomio

    float pwr_watts = evalPoly(x_pwr, PWR_COEFFS, PWR_DEG);

    // Fallback: se il polinomio dà un valore non fisico, usa formula empirica quadratica
    if (!isfinite(pwr_watts) || pwr_watts < 0.0f || pwr_watts > 200.0f) {
        pwr_watts = (pwr_adj * pwr_adj) / 1000000.0f;
    }

    // Soglia rumore: sotto il pavimento ADC, forza zero
    if (raw_pwr < RAW_PWR_NOISE_FLOOR || pwr_watts < PWR_MIN_WATTS) {
        pwr_watts = 0.0f;
    }

    // -----------------------------------------------------------------------
    // 4. SWR — metodo fisico (gamma = Vref / Vfwd)
    // -----------------------------------------------------------------------
    float swr_mV  = raw_swr * ADC_MV_PER_STEP;
    float swr_adj = swr_mV - SWR_ADC_OFFSET_MV;

    float swr_val = 1.0f;   // default: SWR perfetto

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
    // 5. Formattazione stringhe display
    // -----------------------------------------------------------------------
    char temp_str[8], swr_str[8], pwr_str[8];

    // Temperatura: un decimale
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
    // 6. Aggiornamento LCD (solo se i valori sono cambiati)
    // -----------------------------------------------------------------------
    lcdUpdateLine(1, "TEMP C= ", temp_str, prev_temp_str);
    lcdUpdateLine(2, "SWR   = ", swr_str,  prev_swr_str);
    lcdUpdateLine(3, "PWR W = ", pwr_str,  prev_pwr_str);

    // -----------------------------------------------------------------------
    // 7. Debug seriale (solo se #define DEBUG è attivo)
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
