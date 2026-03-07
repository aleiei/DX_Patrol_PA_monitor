/**
 * main_4wire.cpp  —  v3.0.0-4wire
 * PA Monitor — PWR, SWR, Temperatura + Controllo Ventola PWM 4 FILI
 * Arduino Nano (ATmega328P) — DX Patrol 20W PA by IU0PXK
 *
 * Novità rispetto a v2.0.0:
 *  - Encoder rotativo (CLK/DT/SW) per navigazione menù
 *  - Menù impostazioni con 3 parametri regolabili (vedere sotto)
 *  - Controllo ventola PWM hardware a ~50 kHz su pin OC1A (D9)
 *  - Salvataggio impostazioni in EEPROM (magic byte 0xA5)
 *  - Controllo proporzionale velocità ventola con isteresi
 *  - Display: schermata monitor (3 righe dati + 1 stato) e schermata menù
 *  - Timeout automatico menù 15 s → ritorno al monitor
 *
 *  VENTOLA 4 FILI — Schema di controllo:
 *  ──────────────────────────────────────
 *  Le ventole a 4 fili (GND, +12V, TACH, PWM) hanno un pin PWM dedicato
 *  che accetta un segnale a ~25 kHz direttamente dal microcontrollore.
 *  Non è necessario alcun MOSFET: il pin D9 si collega direttamente al
 *  pin BLU (pin 4) della ventola.
 *
 *       Arduino D9 (OC1A) ─────  Pin 4 BLU  ventola (PWM)
 *       Arduino GND       ─────  Pin 1 NERO  ventola (GND)
 *                                Pin 2 GIALLO (TACH, non usato)
 *       +12V              ─────  Pin 3 ROSSO  ventola (+12V)
 *
 *  Frequenza PWM: ~50 kHz (Timer1, Fast PWM, TOP = ICR1 = 319)
 *    f = 16 MHz / (1 * (319+1)) = 50 kHz  → silenzioso, ideale per 4 fili
 *
 *  Encoder rotativo:
 *  ─────────────────
 *  CLK → D2 (INT0), DT → D3 (INT1), SW → D4
 *
 *  Menu settings (LCD labels in English):
 *    1. Fan ON  thresh.  (°C, step 0.5) — turn-on threshold
 *    2. Fan OFF thresh.  (°C, step 0.5) — turn-off threshold (hysteresis)
 *    3. PWM minimum      (0-254, step 5) — minimum duty when fan is running
 *    4. Save & Exit      — write settings to EEPROM and return to monitor
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Build-time options
// ---------------------------------------------------------------------------
// #define DEBUG   // decommentare per output seriale

// ---------------------------------------------------------------------------
// Pin mapping
// ---------------------------------------------------------------------------
static constexpr uint8_t PIN_TEMP       = A0;
static constexpr uint8_t PIN_SWR        = A1;
static constexpr uint8_t PIN_PWR        = A2;

static constexpr uint8_t PIN_ENC_CLK    = 2;   // interrupt INT0
static constexpr uint8_t PIN_ENC_DT     = 3;   // interrupt INT1
static constexpr uint8_t PIN_ENC_SW     = 4;   // pulsante encoder

static constexpr uint8_t PIN_FAN_PWM    = 9;   // OC1A — Timer1 ~25 kHz

// ---------------------------------------------------------------------------
// Hardware / ADC
// ---------------------------------------------------------------------------
static constexpr uint16_t ADC_MAX           = 1023;
static constexpr float    VREF_MV           = 5000.0f;
static constexpr float    ADC_MV_PER_STEP   = VREF_MV / ADC_MAX;
static constexpr uint8_t  ADC_SAMPLES       = 16;

// ---------------------------------------------------------------------------
// Temperatura
// ---------------------------------------------------------------------------
static constexpr float TEMP_OFFSET_MV      = 464.0f;
static constexpr float TEMP_MV_PER_DEG     = 6.25f;
static constexpr float TEMP_FILTER_ALPHA   = 0.15f;

// ---------------------------------------------------------------------------
// Offsets ADC canali RF
// ---------------------------------------------------------------------------
static constexpr float PWR_ADC_OFFSET_MV   = 298.0f;
static constexpr float SWR_ADC_OFFSET_MV   = 1288.0f;

// ---------------------------------------------------------------------------
// Soglie rumore ADC
// ---------------------------------------------------------------------------
static constexpr uint16_t RAW_PWR_NOISE_FLOOR = 8;
static constexpr uint16_t RAW_SWR_NOISE_FLOOR = 4;
static constexpr float    PWR_MIN_WATTS        = 0.001f;
static constexpr float    EPS                  = 1e-6f;

// ---------------------------------------------------------------------------
// Limiti display
// ---------------------------------------------------------------------------
static constexpr float PWR_MAX_DISPLAY  = 99.995f;
static constexpr float SWR_MAX_DISPLAY  = 9.995f;

// ---------------------------------------------------------------------------
// Timing loop
// ---------------------------------------------------------------------------
static constexpr uint32_t LOOP_INTERVAL_MS  = 100;
static constexpr uint32_t MENU_TIMEOUT_MS   = 15000;  // ritorno auto al monitor dopo 15 s

// ---------------------------------------------------------------------------
// Polinomi calibrazione PWR (sostituire con valori reali da calibration_fit.py)
// ---------------------------------------------------------------------------
static constexpr float PWR_COEFFS[] = { 0.0f, 0.0f, 1.0f };
static constexpr int   PWR_DEG      = (sizeof(PWR_COEFFS) / sizeof(PWR_COEFFS[0])) - 1;

// ---------------------------------------------------------------------------
// EEPROM layout
// ---------------------------------------------------------------------------
// Indirizzi EEPROM (ogni float = 4 byte)
static constexpr int EE_ADDR_MAGIC       = 0;          // 1 byte magic number
static constexpr int EE_ADDR_FAN_ON      = 1;          // float  soglia ON  (°C)
static constexpr int EE_ADDR_FAN_OFF     = 5;          // float  soglia OFF (°C)
static constexpr int EE_ADDR_FAN_PWM_MIN = 9;          // uint8  PWM minimo (0-255)
static constexpr uint8_t EE_MAGIC_VALUE  = 0xA5;

// Valori default impostazioni
static constexpr float   DEFAULT_FAN_ON      = 45.0f;  // °C
static constexpr float   DEFAULT_FAN_OFF     = 40.0f;  // °C
static constexpr uint8_t DEFAULT_FAN_PWM_MIN = 80;     // ~31% duty (ventola si muove)

// ---------------------------------------------------------------------------
// Ventola PWM — Timer1 Fast PWM, prescaler 1 → ~25 kHz (silenzioso)
// ICR1 = 319 → f = 16MHz / (1 * (319+1)) = 50 kHz; TOP=319 → OCR1A scala 0-319
// ---------------------------------------------------------------------------
static constexpr uint16_t FAN_PWM_TOP = 319;           // TOP Timer1

// ---------------------------------------------------------------------------
// LCD
// ---------------------------------------------------------------------------
static LiquidCrystal_I2C MyLCD(0x27, 20, 4);

// ---------------------------------------------------------------------------
// Stato applicazione
// ---------------------------------------------------------------------------
enum AppState { STATE_MONITOR, STATE_MENU };
enum MenuItem {
    MENU_FAN_ON = 0,
    MENU_FAN_OFF,
    MENU_FAN_PWM_MIN,
    MENU_EXIT,
    MENU_COUNT
};
static const char* const MENU_LABELS[MENU_COUNT] = {
    "Fan ON  thresh.",
    "Fan OFF thresh.",
    "PWM minimum   ",
    "  [ Save&Exit  ]"
};

static AppState appState        = STATE_MONITOR;
static uint8_t  menuIndex       = 0;
static bool     menuEditing     = false;    // true: stiamo modificando il valore
static uint32_t menuLastActivity= 0;

// Impostazioni (in RAM, sincronizzate con EEPROM al salvataggio)
static float    cfg_fan_on      = DEFAULT_FAN_ON;
static float    cfg_fan_off     = DEFAULT_FAN_OFF;
static uint8_t  cfg_fan_pwm_min = DEFAULT_FAN_PWM_MIN;

// Valori temporanei durante editing
static float    edit_fan_on;
static float    edit_fan_off;
static int16_t  edit_fan_pwm_min;

// Stato sensori
static float    temp_filtered_c        = 0.0f;
static bool     temp_filter_initialized = false;
static bool     fan_running            = false;

// Cache LCD per aggiornamento differenziale
static char prev_line[4][21];

// ---------------------------------------------------------------------------
// Encoder rotativo (gestito via interrupt)
// ---------------------------------------------------------------------------
volatile int8_t enc_delta = 0;       // accumulatore giri encoder
static uint8_t  enc_last_clk = HIGH;

// Debounce pulsante encoder
static uint32_t btn_last_press_ms = 0;
static constexpr uint32_t BTN_DEBOUNCE_MS = 50;

// ISR encoder — lettura CLK/DT
void IRAM_ATTR encoderISR() {
    uint8_t clk_now = digitalRead(PIN_ENC_CLK);
    if (clk_now != enc_last_clk) {
        enc_last_clk = clk_now;
        if (clk_now == LOW) {
            // fronte di discesa CLK → leggi DT
            if (digitalRead(PIN_ENC_DT) == HIGH) {
                enc_delta++;
            } else {
                enc_delta--;
            }
        }
    }
}

static int8_t encoderRead() {
    noInterrupts();
    int8_t d = enc_delta;
    enc_delta = 0;
    interrupts();
    return d;
}

static bool buttonPressed() {
    if (digitalRead(PIN_ENC_SW) == LOW) {
        uint32_t now = millis();
        if (now - btn_last_press_ms > BTN_DEBOUNCE_MS) {
            btn_last_press_ms = now;
            // aspetta rilascio
            while (digitalRead(PIN_ENC_SW) == LOW) delay(5);
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// EEPROM helpers
// ---------------------------------------------------------------------------
static void eepromLoadSettings() {
    uint8_t magic;
    EEPROM.get(EE_ADDR_MAGIC, magic);
    if (magic != EE_MAGIC_VALUE) {
        // Prima accensione: usa default
        cfg_fan_on      = DEFAULT_FAN_ON;
        cfg_fan_off     = DEFAULT_FAN_OFF;
        cfg_fan_pwm_min = DEFAULT_FAN_PWM_MIN;
        return;
    }
    EEPROM.get(EE_ADDR_FAN_ON,      cfg_fan_on);
    EEPROM.get(EE_ADDR_FAN_OFF,     cfg_fan_off);
    EEPROM.get(EE_ADDR_FAN_PWM_MIN, cfg_fan_pwm_min);

    // Sanity check
    if (!isfinite(cfg_fan_on)  || cfg_fan_on  < 20.0f || cfg_fan_on  > 90.0f) cfg_fan_on  = DEFAULT_FAN_ON;
    if (!isfinite(cfg_fan_off) || cfg_fan_off < 15.0f || cfg_fan_off > 85.0f) cfg_fan_off = DEFAULT_FAN_OFF;
    if (cfg_fan_off >= cfg_fan_on) cfg_fan_off = cfg_fan_on - 5.0f;
}

static void eepromSaveSettings() {
    EEPROM.put(EE_ADDR_FAN_ON,      cfg_fan_on);
    EEPROM.put(EE_ADDR_FAN_OFF,     cfg_fan_off);
    EEPROM.put(EE_ADDR_FAN_PWM_MIN, cfg_fan_pwm_min);
    EEPROM.put(EE_ADDR_MAGIC,       EE_MAGIC_VALUE);
}

// ---------------------------------------------------------------------------
// Fan PWM — Timer1 Fast PWM non-invertente su OC1A (D9)
// ---------------------------------------------------------------------------
static void fanPwmSetup() {
    // Fast PWM, TOP = ICR1, non-invertente su OC1A
    // WGM13:10 = 1110 → Fast PWM, TOP = ICR1
    // COM1A1:0 = 10   → Clear OC1A on compare match
    // CS10 = 1        → prescaler 1 → f = 16 MHz / (TOP+1) ≈ 50 kHz
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS10);
    ICR1   = FAN_PWM_TOP;
    OCR1A  = 0;    // ventola ferma all'avvio
    pinMode(PIN_FAN_PWM, OUTPUT);
}

/**
 * Imposta la velocità della ventola.
 * @param duty_255: 0 = spenta, 255 = massima velocità
 */
static void fanSetDuty(uint8_t duty_255) {
    if (duty_255 == 0) {
        OCR1A = 0;
    } else {
        // Rimappa 0-255 → cfg_fan_pwm_min..FAN_PWM_TOP
        uint16_t ocr = (uint16_t)cfg_fan_pwm_min
                     + (uint16_t)((FAN_PWM_TOP - cfg_fan_pwm_min) * (uint32_t)duty_255 / 255UL);
        OCR1A = ocr;
    }
}

// ---------------------------------------------------------------------------
// Logica controllo ventola con isteresi
// ---------------------------------------------------------------------------
static void updateFan(float temp_c) {
    if (!fan_running && temp_c >= cfg_fan_on) {
        fan_running = true;
    } else if (fan_running && temp_c <= cfg_fan_off) {
        fan_running = false;
    }

    if (!fan_running) {
        fanSetDuty(0);
    } else {
        // Modulazione proporzionale tra soglia OFF e soglia ON + 5 °C
        float range = (cfg_fan_on + 5.0f) - cfg_fan_off;
        float t_norm = (temp_c - cfg_fan_off) / max(range, 1.0f);
        t_norm = constrain(t_norm, 0.0f, 1.0f);
        uint8_t duty = (uint8_t)(t_norm * 255.0f);
        // Garantisci almeno il PWM minimo quando la ventola è accesa
        if (duty < cfg_fan_pwm_min) duty = cfg_fan_pwm_min;
        fanSetDuty(duty);
    }
}

// ---------------------------------------------------------------------------
// ADC helpers
// ---------------------------------------------------------------------------
static float adcAverage(uint8_t pin, uint8_t samples) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < samples; ++i) sum += analogRead(pin);
    return static_cast<float>(sum) / samples;
}

static float evalPoly(float x, const float *c, int deg) {
    float y = c[deg];
    for (int i = deg - 1; i >= 0; --i) y = y * x + c[i];
    return y;
}

// ---------------------------------------------------------------------------
// LCD helpers
// ---------------------------------------------------------------------------
static void lcdWriteLine(uint8_t row, const char *text) {
    if (strncmp(text, prev_line[row], 20) == 0) return;
    strncpy(prev_line[row], text, 20);
    prev_line[row][20] = '\0';
    MyLCD.setCursor(0, row);
    char buf[21];
    snprintf(buf, sizeof(buf), "%-20s", text);
    MyLCD.print(buf);
}

static void lcdClearCache() {
    for (uint8_t i = 0; i < 4; ++i) prev_line[i][0] = '\0';
}

// ---------------------------------------------------------------------------
// Schermata monitor
// ---------------------------------------------------------------------------
static void drawMonitor(float temp_c, float pwr_w, float swr_v) {
    char line[21];
    char val[9];

    // Riga 0: titolo
    lcdWriteLine(0, "PA Monitor IU0PXK   ");

    // Riga 1: temperatura + stato ventola
    dtostrf(temp_c, 5, 1, val);
    snprintf(line, sizeof(line), "TEMP:%s%cC Fan:%s",
             val,
             (char)223,   // simbolo grado se LCD lo supporta
             fan_running ? "ON " : "OFF");
    lcdWriteLine(1, line);

    // Riga 2: SWR
    {
        float sv = constrain(swr_v, 1.0f, SWR_MAX_DISPLAY);
        int isv = (int)sv;
        int dsv = (int)roundf((sv - isv) * 100.0f);
        if (dsv >= 100) { isv++; dsv = 0; }
        snprintf(line, sizeof(line), "SWR   = %1d,%02d           ", isv, dsv);
    }
    lcdWriteLine(2, line);

    // Riga 3: PWR
    {
        float pv = constrain(pwr_w, 0.0f, PWR_MAX_DISPLAY);
        int ip = (int)pv;
        int dp = (int)roundf((pv - ip) * 100.0f);
        if (dp >= 100) { ip++; dp = 0; }
        snprintf(line, sizeof(line), "PWR W = %02d,%02d          ", ip, dp);
    }
    lcdWriteLine(3, line);
}

// ---------------------------------------------------------------------------
// Schermata menù
// ---------------------------------------------------------------------------
static void drawMenu() {
    char line[21];

    // Row 0: header
    lcdWriteLine(0, " **  SETTINGS   ** ");

    for (uint8_t i = 0; i < 3 && i < MENU_COUNT - 1; ++i) {
        uint8_t row = i + 1;
        char cursor = (menuIndex == i) ? '>' : ' ';

        if (menuIndex == i && menuEditing) {
            // Show value being edited (asterisk = edit mode)
            switch (i) {
                case MENU_FAN_ON:
                    snprintf(line, sizeof(line), "%c*Fan ON : %4.1f C  ", cursor, edit_fan_on);
                    break;
                case MENU_FAN_OFF:
                    snprintf(line, sizeof(line), "%c*Fan OFF: %4.1f C  ", cursor, edit_fan_off);
                    break;
                case MENU_FAN_PWM_MIN:
                    snprintf(line, sizeof(line), "%c*PWM min: %3d/255  ", cursor, (int)edit_fan_pwm_min);
                    break;
            }
        } else {
            switch (i) {
                case MENU_FAN_ON:
                    snprintf(line, sizeof(line), "%c Fan ON : %4.1f C  ", cursor, cfg_fan_on);
                    break;
                case MENU_FAN_OFF:
                    snprintf(line, sizeof(line), "%c Fan OFF: %4.1f C  ", cursor, cfg_fan_off);
                    break;
                case MENU_FAN_PWM_MIN:
                    snprintf(line, sizeof(line), "%c PWM min: %3d/255  ", cursor, (int)cfg_fan_pwm_min);
                    break;
            }
        }
        lcdWriteLine(row, line);
    }

    // Riga EXIT (scorre se menuIndex == MENU_EXIT)
    // Usiamo la quarta riga per EXIT sempre visibile
    {
        char cursor = (menuIndex == MENU_EXIT) ? '>' : ' ';
        snprintf(line, sizeof(line), "%c   [ Save & Exit ]  ", cursor);
        lcdWriteLine(3, line);
    }
}

// ---------------------------------------------------------------------------
// Gestione input menù
// ---------------------------------------------------------------------------
static void handleMenuInput() {
    int8_t delta = encoderRead();
    bool   btn   = buttonPressed();

    if (delta != 0 || btn) menuLastActivity = millis();

    if (!menuEditing) {
        // Navigazione voci
        if (delta > 0) {
            menuIndex = (menuIndex + 1) % MENU_COUNT;
            lcdClearCache();
        } else if (delta < 0) {
            menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT;
            lcdClearCache();
        }

        if (btn) {
            if (menuIndex == MENU_EXIT) {
                // Save and return to monitor
                eepromSaveSettings();
                appState = STATE_MONITOR;
                lcdClearCache();
            } else {
                // Inizia editing del parametro selezionato
                menuEditing = true;
                edit_fan_on      = cfg_fan_on;
                edit_fan_off     = cfg_fan_off;
                edit_fan_pwm_min = cfg_fan_pwm_min;
                lcdClearCache();
            }
        }
    } else {
        // Modifica valore
        if (delta != 0) {
            switch (menuIndex) {
                case MENU_FAN_ON:
                    edit_fan_on += delta * 0.5f;
                    edit_fan_on  = constrain(edit_fan_on, edit_fan_off + 1.0f, 85.0f);
                    break;
                case MENU_FAN_OFF:
                    edit_fan_off += delta * 0.5f;
                    edit_fan_off  = constrain(edit_fan_off, 20.0f, edit_fan_on - 1.0f);
                    break;
                case MENU_FAN_PWM_MIN:
                    edit_fan_pwm_min += delta * 5;
                    edit_fan_pwm_min  = constrain((int16_t)edit_fan_pwm_min, 0, 254);
                    break;
            }
            lcdClearCache();
        }

        if (btn) {
            // Conferma valore
            cfg_fan_on      = edit_fan_on;
            cfg_fan_off     = edit_fan_off;
            cfg_fan_pwm_min = (uint8_t)edit_fan_pwm_min;
            menuEditing     = false;
            lcdClearCache();
        }
    }

    // Timeout automatico: ritorna al monitor senza salvare
    if (millis() - menuLastActivity > MENU_TIMEOUT_MS) {
        appState    = STATE_MONITOR;
        menuEditing = false;
        lcdClearCache();
    }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
    // Pin encoder
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT,  INPUT_PULLUP);
    pinMode(PIN_ENC_SW,  INPUT_PULLUP);
    enc_last_clk = digitalRead(PIN_ENC_CLK);

    // Interrupt encoder su fronte di cambio CLK e DT
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encoderISR, CHANGE);

    // Ventola PWM
    fanPwmSetup();

    // LCD
    MyLCD.init();
    MyLCD.backlight();
    lcdClearCache();

    // Splash screen
    MyLCD.setCursor(0, 0);
    MyLCD.print("PA Monitor v3.0     ");
    MyLCD.setCursor(0, 1);
    MyLCD.print("by IU0PXK           ");
    MyLCD.setCursor(0, 2);
    MyLCD.print("Press enc. = menu   ");
    delay(2000);
    lcdClearCache();

    // Carica impostazioni da EEPROM
    eepromLoadSettings();

#ifdef DEBUG
    Serial.begin(115200);
    Serial.println(F("PA Monitor v3.0 DEBUG"));
    Serial.print(F("Fan ON="));  Serial.print(cfg_fan_on);
    Serial.print(F(" OFF="));    Serial.print(cfg_fan_off);
    Serial.print(F(" PWMmin=")); Serial.println(cfg_fan_pwm_min);
#endif
}

void loop() {
    uint32_t now = millis();

    // ------------------------------------------------------------------
    // Controlla pulsante encoder per entrare nel menù (da schermata monitor)
    // ------------------------------------------------------------------
    if (appState == STATE_MONITOR && buttonPressed()) {
        appState          = STATE_MENU;
        menuIndex         = 0;
        menuEditing       = false;
        menuLastActivity  = now;
        lcdClearCache();
    }

    // ------------------------------------------------------------------
    // Gestione menù (non bloccante, gira ad ogni iterazione)
    // ------------------------------------------------------------------
    if (appState == STATE_MENU) {
        handleMenuInput();
        drawMenu();
        return;   // non legge sensori mentre è nel menù
    }

    // ------------------------------------------------------------------
    // Loop monitor — cadenza LOOP_INTERVAL_MS
    // ------------------------------------------------------------------
    if (now - last_update_ms < LOOP_INTERVAL_MS) return;
    last_update_ms = now;

    // 1. Lettura ADC
    float raw_temp = adcAverage(PIN_TEMP, ADC_SAMPLES);
    float raw_swr  = adcAverage(PIN_SWR,  ADC_SAMPLES);
    float raw_pwr  = adcAverage(PIN_PWR,  ADC_SAMPLES);

    // 2. Temperatura con EMA
    float temp_mV = raw_temp * ADC_MV_PER_STEP;
    float temp_c  = (temp_mV - TEMP_OFFSET_MV) / TEMP_MV_PER_DEG;

    if (!temp_filter_initialized) {
        temp_filtered_c = temp_c;
        temp_filter_initialized = true;
    } else {
        temp_filtered_c += TEMP_FILTER_ALPHA * (temp_c - temp_filtered_c);
    }

    // 3. Potenza
    float pwr_mV    = raw_pwr * ADC_MV_PER_STEP;
    float pwr_adj   = pwr_mV - PWR_ADC_OFFSET_MV;
    float x_pwr     = pwr_adj / 1000.0f;
    float pwr_watts = evalPoly(x_pwr, PWR_COEFFS, PWR_DEG);

    if (!isfinite(pwr_watts) || pwr_watts < 0.0f || pwr_watts > 200.0f) {
        pwr_watts = (pwr_adj * pwr_adj) / 1000000.0f;
    }
    if (raw_pwr < RAW_PWR_NOISE_FLOOR || pwr_watts < PWR_MIN_WATTS) {
        pwr_watts = 0.0f;
    }

    // 4. SWR
    float swr_mV  = raw_swr * ADC_MV_PER_STEP;
    float swr_adj = swr_mV - SWR_ADC_OFFSET_MV;
    float swr_val = 1.0f;

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

    // 5. Controllo ventola
    updateFan(temp_filtered_c);

    // 6. Aggiornamento display
    drawMonitor(temp_filtered_c, pwr_watts, swr_val);

#ifdef DEBUG
    Serial.print(F("T="));  Serial.print(temp_filtered_c);
    Serial.print(F(" P="));  Serial.print(pwr_watts);
    Serial.print(F(" SWR=")); Serial.print(swr_val);
    Serial.print(F(" Fan=")); Serial.print(fan_running ? "ON" : "OFF");
    Serial.print(F(" OCR1A=")); Serial.println(OCR1A);
#endif
}
