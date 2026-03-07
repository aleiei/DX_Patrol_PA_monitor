/**
 * main_3wire.cpp  —  v3.0.0-3wire
 * PA Monitor — PWR, SWR, Temperature + 3-WIRE Fan Control
 * Arduino Nano (ATmega328P) — DX Patrol 20W PA by IU0PXK
 *
 * Differences compared to the 4-wire version (main_4wire.cpp):
 *
 *  3-WIRE FAN — Control wiring:
 *  ─────────────────────────────────────
 *  3-wire fans (GND, +12V, TACH) do not have a dedicated PWM control pin.
 *  Speed is controlled by switching current with an N-channel MOSFET
 *  (e.g. IRLZ44N, 2N7000) on the LOW-SIDE:
 *
 *       +12V ──────────── fan RED wire
 *       Fan GND ───────── MOSFET Drain
 *       Source MOSFET ─── GND Arduino
 *       Gate MOSFET ───── D11 (OC2A) ── 100 ohm series ── Gate
 *                                    └── 10 kohm pull-down to GND
 *
 *  PWM frequency: ~61 Hz (Timer2, prescaler 1024, Fast PWM)
 *    f = 16 MHz / (1024 x 256) ≈ 61 Hz
 *    Note: some fans accept up to 100 Hz; 61 Hz is a good compromise.
 *    Lowering to ~30 Hz is not practical with Timer2 in Fast PWM mode.
 *
 *  SOFT-START:
 *  ──────────
 *  When the fan turns on, duty is forced to 100% for
 *  FAN_KICKSTART_MS milliseconds, then reduced to the temperature-based
 *  proportional value. This guarantees motor startup even with
 *  low duty cycles (many fans do not start below 30-40%).
 *
 *  TACH (optional):
 *  ─────────────────
 *  The fan YELLOW wire carries the tach signal (2 pulses/revolution).
 *  The code can read it on D5 (pin-change interrupt) and show RPM
 *  on the display when #define TACH_ENABLED is enabled.
 *
 *  Rotary encoder:
 *  ─────────────────
 *  CLK → D2 (INT0), DT → D3 (INT1), SW → D4
 *
 *  Settings menu:
 *    1. Fan ON threshold  (°C, step 0.5)
 *    2. Fan OFF threshold (°C, step 0.5)
 *    3. Minimum duty %    (%, step 1) — duty after kick-start
 *    4. Kick-start %      (%, step 5) — duty during startup (100% is
 *                                       the recommended default)
 *    5. Save & Exit
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
// #define DEBUG           // serial debug output
// #define TACH_ENABLED    // enable RPM reading from fan YELLOW wire

// ---------------------------------------------------------------------------
// Pin mapping
// ---------------------------------------------------------------------------
static constexpr uint8_t PIN_TEMP       = A0;
static constexpr uint8_t PIN_SWR        = A1;
static constexpr uint8_t PIN_PWR        = A2;

static constexpr uint8_t PIN_ENC_CLK    = 2;   // interrupt INT0
static constexpr uint8_t PIN_ENC_DT     = 3;   // interrupt INT1
static constexpr uint8_t PIN_ENC_SW     = 4;   // encoder button

// D11 = OC2A — only Timer2 pin on Nano that supports hardware Fast PWM
static constexpr uint8_t PIN_FAN_PWM    = 11;  // OC2A — Timer2 ~61 Hz

#ifdef TACH_ENABLED
static constexpr uint8_t PIN_FAN_TACH   = 5;   // PCINT, fan YELLOW wire
#endif

// ---------------------------------------------------------------------------
// Hardware / ADC
// ---------------------------------------------------------------------------
static constexpr uint16_t ADC_MAX           = 1023;
static constexpr float    VREF_MV           = 5000.0f;
static constexpr float    ADC_MV_PER_STEP   = VREF_MV / ADC_MAX;
static constexpr uint8_t  ADC_SAMPLES       = 16;

// ---------------------------------------------------------------------------
// Temperature
// ---------------------------------------------------------------------------
static constexpr float TEMP_OFFSET_MV      = 464.0f;
static constexpr float TEMP_MV_PER_DEG     = 6.25f;
static constexpr float TEMP_FILTER_ALPHA   = 0.15f;

// ---------------------------------------------------------------------------
// ADC offsets for RF channels
// ---------------------------------------------------------------------------
static constexpr float PWR_ADC_OFFSET_MV   = 298.0f;
static constexpr float SWR_ADC_OFFSET_MV   = 1288.0f;

// ---------------------------------------------------------------------------
// ADC noise thresholds
// ---------------------------------------------------------------------------
static constexpr uint16_t RAW_PWR_NOISE_FLOOR = 8;
static constexpr uint16_t RAW_SWR_NOISE_FLOOR = 4;
static constexpr float    PWR_MIN_WATTS        = 0.001f;
static constexpr float    EPS                  = 1e-6f;

// ---------------------------------------------------------------------------
// Display limits
// ---------------------------------------------------------------------------
static constexpr float PWR_MAX_DISPLAY  = 99.995f;
static constexpr float SWR_MAX_DISPLAY  = 9.995f;

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
static constexpr uint32_t LOOP_INTERVAL_MS  = 100;
static constexpr uint32_t MENU_TIMEOUT_MS   = 15000;

// ---------------------------------------------------------------------------
// 3-wire fan — soft-start
// ---------------------------------------------------------------------------
static constexpr uint32_t FAN_KICKSTART_MS  = 1200;  // full-power startup duration

// ---------------------------------------------------------------------------
// PWR calibration polynomials
// ---------------------------------------------------------------------------
static constexpr float PWR_COEFFS[] = { 0.0f, 0.0f, 1.0f };
static constexpr int   PWR_DEG      = (sizeof(PWR_COEFFS) / sizeof(PWR_COEFFS[0])) - 1;

// ---------------------------------------------------------------------------
// EEPROM layout
// ---------------------------------------------------------------------------
static constexpr int     EE_ADDR_MAGIC        = 0;   // uint8
static constexpr int     EE_ADDR_FAN_ON       = 1;   // float (4 byte)
static constexpr int     EE_ADDR_FAN_OFF      = 5;   // float (4 byte)
static constexpr int     EE_ADDR_FAN_DUTY_MIN = 9;   // uint8  min duty %
static constexpr int     EE_ADDR_FAN_KICK_PCT = 10;  // uint8  kick-start %
static constexpr uint8_t EE_MAGIC_VALUE       = 0xB6; // different from v4-wire

// Default settings
static constexpr float   DEFAULT_FAN_ON       = 45.0f;
static constexpr float   DEFAULT_FAN_OFF      = 40.0f;
static constexpr uint8_t DEFAULT_FAN_DUTY_MIN = 40;   // 40% — safe startup threshold
static constexpr uint8_t DEFAULT_FAN_KICK_PCT = 100;  // 100% kick (recommended)

// ---------------------------------------------------------------------------
// LCD
// ---------------------------------------------------------------------------
static LiquidCrystal_I2C MyLCD(0x27, 20, 4);

// ---------------------------------------------------------------------------
// Menu
// ---------------------------------------------------------------------------
enum AppState { STATE_MONITOR, STATE_MENU };
enum MenuItem {
    MENU_FAN_ON = 0,
    MENU_FAN_OFF,
    MENU_DUTY_MIN,
    MENU_KICK_PCT,
    MENU_EXIT,
    MENU_COUNT
};

static AppState appState         = STATE_MONITOR;
static uint8_t  menuIndex        = 0;
static bool     menuEditing      = false;
static uint32_t menuLastActivity = 0;

// Runtime settings
static float    cfg_fan_on        = DEFAULT_FAN_ON;
static float    cfg_fan_off       = DEFAULT_FAN_OFF;
static uint8_t  cfg_fan_duty_min  = DEFAULT_FAN_DUTY_MIN;  // % (0-100)
static uint8_t  cfg_fan_kick_pct  = DEFAULT_FAN_KICK_PCT;  // % (0-100)

// Temporary editing values
static float   edit_fan_on;
static float   edit_fan_off;
static int16_t edit_duty_min;
static int16_t edit_kick_pct;

// Sensor and fan state
static float    temp_filtered_c         = 0.0f;
static bool     temp_filter_initialized = false;
static bool     fan_running             = false;
static uint32_t fan_kick_start_ms       = 0;   // kick-start start timestamp
static bool     fan_in_kickstart        = false;

// LCD cache
static char prev_line[4][21];

// ---------------------------------------------------------------------------
// TACH (optional)
// ---------------------------------------------------------------------------
#ifdef TACH_ENABLED
volatile uint16_t tach_pulse_count = 0;
static uint16_t   fan_rpm          = 0;
static uint32_t   tach_last_calc_ms = 0;

ISR(PCINT2_vect) {
    // Only rising edges on TACH pin
    static uint8_t last_tach = HIGH;
    uint8_t now_tach = digitalRead(PIN_FAN_TACH);
    if (now_tach == HIGH && last_tach == LOW) tach_pulse_count++;
    last_tach = now_tach;
}

static void tachSetup() {
    pinMode(PIN_FAN_TACH, INPUT_PULLUP);
    // Enable Pin Change Interrupt on PCINT21 (D5 = PD5 = PCINT21)
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT21);
}

static void tachUpdate() {
    uint32_t now = millis();
    if (now - tach_last_calc_ms >= 1000) {
        noInterrupts();
        uint16_t pulses = tach_pulse_count;
        tach_pulse_count = 0;
        interrupts();
        // 2 pulses per revolution → RPM = pulses / 2 * 60
        fan_rpm = (uint16_t)(pulses * 30UL);
        tach_last_calc_ms = now;
    }
}
#endif // TACH_ENABLED

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------
volatile int8_t enc_delta    = 0;
static uint8_t  enc_last_clk = HIGH;
static uint32_t btn_last_press_ms = 0;
static constexpr uint32_t BTN_DEBOUNCE_MS = 50;

void encoderISR() {
    uint8_t clk_now = digitalRead(PIN_ENC_CLK);
    if (clk_now != enc_last_clk) {
        enc_last_clk = clk_now;
        if (clk_now == LOW) {
            enc_delta += (digitalRead(PIN_ENC_DT) == HIGH) ? 1 : -1;
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
        cfg_fan_on       = DEFAULT_FAN_ON;
        cfg_fan_off      = DEFAULT_FAN_OFF;
        cfg_fan_duty_min = DEFAULT_FAN_DUTY_MIN;
        cfg_fan_kick_pct = DEFAULT_FAN_KICK_PCT;
        return;
    }
    EEPROM.get(EE_ADDR_FAN_ON,       cfg_fan_on);
    EEPROM.get(EE_ADDR_FAN_OFF,      cfg_fan_off);
    EEPROM.get(EE_ADDR_FAN_DUTY_MIN, cfg_fan_duty_min);
    EEPROM.get(EE_ADDR_FAN_KICK_PCT, cfg_fan_kick_pct);

    if (!isfinite(cfg_fan_on)  || cfg_fan_on  < 20.0f || cfg_fan_on  > 90.0f) cfg_fan_on  = DEFAULT_FAN_ON;
    if (!isfinite(cfg_fan_off) || cfg_fan_off < 15.0f || cfg_fan_off > 85.0f) cfg_fan_off = DEFAULT_FAN_OFF;
    if (cfg_fan_off >= cfg_fan_on) cfg_fan_off = cfg_fan_on - 5.0f;
    if (cfg_fan_duty_min > 100) cfg_fan_duty_min = DEFAULT_FAN_DUTY_MIN;
    if (cfg_fan_kick_pct > 100) cfg_fan_kick_pct = DEFAULT_FAN_KICK_PCT;
}

static void eepromSaveSettings() {
    EEPROM.put(EE_ADDR_FAN_ON,       cfg_fan_on);
    EEPROM.put(EE_ADDR_FAN_OFF,      cfg_fan_off);
    EEPROM.put(EE_ADDR_FAN_DUTY_MIN, cfg_fan_duty_min);
    EEPROM.put(EE_ADDR_FAN_KICK_PCT, cfg_fan_kick_pct);
    EEPROM.put(EE_ADDR_MAGIC,        EE_MAGIC_VALUE);
}

// ---------------------------------------------------------------------------
// 3-wire fan — Timer2 Fast PWM on OC2A (D11)
//
// Timer2: 8-bit, Fast PWM, TOP = 0xFF (256 step)
// COM2A1:0 = 10  → clear OC2A on match, set at BOTTOM (non-inverting)
// WGM2  = 011    → Fast PWM, TOP=0xFF
// CS2   = 111    → prescaler 1024
//   f = 16 MHz / (1024 × 256) ≈ 61 Hz  → suitable for 3-wire fans
//
// OCR2A = 0   → fan off (MOSFET off)
// OCR2A = 255 -> fan at full speed (MOSFET always on)
// ---------------------------------------------------------------------------
static void fanPwmSetup() {
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);  // Fast PWM, non-inv
    TCCR2B = (1 << CS22)   | (1 << CS21)  | (1 << CS20);   // prescaler 1024
    OCR2A  = 0;
    pinMode(PIN_FAN_PWM, OUTPUT);
    digitalWrite(PIN_FAN_PWM, LOW);  // MOSFET off = fan off
}

/**
 * Set fan duty cycle.
 * @param duty_pct  0-100 (percentage)
 */
static void fanSetDutyPct(uint8_t duty_pct) {
    duty_pct = constrain(duty_pct, 0, 100);
    // Remap 0-100% → 0-255 (OCR2A)
    OCR2A = (uint8_t)((uint16_t)duty_pct * 255U / 100U);
}

// ---------------------------------------------------------------------------
// Fan control logic with hysteresis + soft-start
// ---------------------------------------------------------------------------
static void updateFan(float temp_c) {
    uint32_t now = millis();

    // ON/OFF hysteresis
    if (!fan_running && temp_c >= cfg_fan_on) {
        fan_running       = true;
        fan_in_kickstart  = true;
        fan_kick_start_ms = now;
    } else if (fan_running && temp_c <= cfg_fan_off) {
        fan_running      = false;
        fan_in_kickstart = false;
        fanSetDutyPct(0);
        return;
    }

    if (!fan_running) {
        fanSetDutyPct(0);
        return;
    }

    // Soft-start: hold kick-start % for FAN_KICKSTART_MS
    if (fan_in_kickstart) {
        if (now - fan_kick_start_ms < FAN_KICKSTART_MS) {
            fanSetDutyPct(cfg_fan_kick_pct);
            return;
        }
        fan_in_kickstart = false;
    }

    // Temperature-proportional duty between OFF threshold and (ON threshold + 5 °C)
    float t_lo  = cfg_fan_off;
    float t_hi  = cfg_fan_on + 5.0f;
    float range = t_hi - t_lo;
    float t_norm = (temp_c - t_lo) / max(range, 1.0f);
    t_norm = constrain(t_norm, 0.0f, 1.0f);

    // Scale between duty_min% and 100%
    uint8_t duty = (uint8_t)(cfg_fan_duty_min + t_norm * (100 - cfg_fan_duty_min));
    duty = constrain(duty, cfg_fan_duty_min, 100);
    fanSetDutyPct(duty);
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
// Monitor screen
// ---------------------------------------------------------------------------
static void drawMonitor(float temp_c, float pwr_w, float swr_v) {
    char line[21];
    char val[8];

    // Row 0: title
    lcdWriteLine(0, "PA Monitor IU0PXK   ");

    // Row 1: temperature + fan status
    // If in kick-start, show "KS" instead of "ON"
    dtostrf(temp_c, 5, 1, val);
    const char *fan_state = fan_in_kickstart ? "KS " : (fan_running ? "ON " : "OFF");
    snprintf(line, sizeof(line), "TEMP:%s%cC Fan:%s",
             val, (char)223, fan_state);
    lcdWriteLine(1, line);

    // Row 2: SWR
    {
        float sv = constrain(swr_v, 1.0f, SWR_MAX_DISPLAY);
        int isv = (int)sv;
        int dsv = (int)roundf((sv - isv) * 100.0f);
        if (dsv >= 100) { isv++; dsv = 0; }
        snprintf(line, sizeof(line), "SWR   = %1d,%02d         ", isv, dsv);
    }
    lcdWriteLine(2, line);

    // Row 3: PWR  (or RPM if TACH_ENABLED)
#ifdef TACH_ENABLED
    snprintf(line, sizeof(line), "PWR=%02d,%02dW  %4dRPM ",
             (int)constrain(pwr_w, 0.0f, PWR_MAX_DISPLAY),
             (int)roundf((constrain(pwr_w, 0.0f, PWR_MAX_DISPLAY)
                          - (int)constrain(pwr_w, 0.0f, PWR_MAX_DISPLAY)) * 100.0f),
             (int)fan_rpm);
#else
    {
        float pv = constrain(pwr_w, 0.0f, PWR_MAX_DISPLAY);
        int ip = (int)pv;
        int dp = (int)roundf((pv - ip) * 100.0f);
        if (dp >= 100) { ip++; dp = 0; }
        snprintf(line, sizeof(line), "PWR W = %02d,%02d         ", ip, dp);
    }
#endif
    lcdWriteLine(3, line);
}

// ---------------------------------------------------------------------------
// Menu screen
// ---------------------------------------------------------------------------
static void drawMenu() {
    char line[21];

    // Row 0: header — also shows current OCR2A duty for feedback
    snprintf(line, sizeof(line), "   SETTINGS    %3d%% ",
             (int)((uint16_t)OCR2A * 100U / 255U));
    lcdWriteLine(0, line);

    // Rows 1-3: show first 3 items starting from slot 0
    // (simple scrolling: always show 3 items)
    for (uint8_t slot = 0; slot < 3; ++slot) {
        uint8_t item = slot;   // menu has 5 items; scroll only when needed
        if (item >= MENU_COUNT) break;

        char cursor = (menuIndex == item) ? '>' : ' ';
        uint8_t row = slot + 1;

        if (menuEditing && menuIndex == item) {
            // Value being edited
            switch (item) {
                case MENU_FAN_ON:
                    snprintf(line, sizeof(line), "%c*Fan ON :%5.1fC    ", cursor, edit_fan_on);
                    break;
                case MENU_FAN_OFF:
                    snprintf(line, sizeof(line), "%c*Fan OFF:%5.1fC    ", cursor, edit_fan_off);
                    break;
                case MENU_DUTY_MIN:
                    snprintf(line, sizeof(line), "%c*Min duty:  %3d%%   ", cursor, (int)edit_duty_min);
                    break;
                case MENU_KICK_PCT:
                    snprintf(line, sizeof(line), "%c*Kick-st:   %3d%%   ", cursor, (int)edit_kick_pct);
                    break;
                case MENU_EXIT:
                    snprintf(line, sizeof(line), "%c [Save & Exit]     ", cursor);
                    break;
            }
        } else {
            switch (item) {
                case MENU_FAN_ON:
                    snprintf(line, sizeof(line), "%c Fan ON :%5.1fC    ", cursor, cfg_fan_on);
                    break;
                case MENU_FAN_OFF:
                    snprintf(line, sizeof(line), "%c Fan OFF:%5.1fC    ", cursor, cfg_fan_off);
                    break;
                case MENU_DUTY_MIN:
                    snprintf(line, sizeof(line), "%c Min duty:  %3d%%   ", cursor, (int)cfg_fan_duty_min);
                    break;
                case MENU_KICK_PCT:
                    snprintf(line, sizeof(line), "%c Kick-st:   %3d%%   ", cursor, (int)cfg_fan_kick_pct);
                    break;
                case MENU_EXIT:
                    snprintf(line, sizeof(line), "%c [Save & Exit]     ", cursor);
                    break;
            }
        }
        lcdWriteLine(row, line);
    }

    // If menuIndex >= 3, redraw from menuIndex-2 (manual scrolling)
    if (menuIndex >= 3) {
        lcdClearCache();
        for (uint8_t slot = 0; slot < 3; ++slot) {
            uint8_t item = (menuIndex - 2) + slot;
            if (item >= MENU_COUNT) break;
            char cursor = (menuIndex == item) ? '>' : ' ';
            uint8_t row = slot + 1;
            switch (item) {
                case MENU_DUTY_MIN:
                    snprintf(line, sizeof(line), "%c Min duty:  %3d%%   ", cursor,
                             menuEditing && menuIndex==item ? (int)edit_duty_min : (int)cfg_fan_duty_min);
                    break;
                case MENU_KICK_PCT:
                    snprintf(line, sizeof(line), "%c Kick-st:   %3d%%   ", cursor,
                             menuEditing && menuIndex==item ? (int)edit_kick_pct : (int)cfg_fan_kick_pct);
                    break;
                case MENU_EXIT:
                    snprintf(line, sizeof(line), "%c [Save & Exit]     ", cursor);
                    break;
                default:
                    snprintf(line, sizeof(line), "%-20s", "");
                    break;
            }
            lcdWriteLine(row, line);
        }
    }
}

// ---------------------------------------------------------------------------
// Menu input handling
// ---------------------------------------------------------------------------
static void handleMenuInput() {
    int8_t delta = encoderRead();
    bool   btn   = buttonPressed();

    if (delta != 0 || btn) menuLastActivity = millis();

    if (!menuEditing) {
        if (delta > 0) { menuIndex = (menuIndex + 1) % MENU_COUNT; lcdClearCache(); }
        else if (delta < 0) { menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT; lcdClearCache(); }

        if (btn) {
            if (menuIndex == MENU_EXIT) {
                eepromSaveSettings();
                appState = STATE_MONITOR;
                lcdClearCache();
            } else {
                menuEditing   = true;
                edit_fan_on   = cfg_fan_on;
                edit_fan_off  = cfg_fan_off;
                edit_duty_min = cfg_fan_duty_min;
                edit_kick_pct = cfg_fan_kick_pct;
                lcdClearCache();
            }
        }
    } else {
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
                case MENU_DUTY_MIN:
                    edit_duty_min += delta;
                    edit_duty_min  = constrain(edit_duty_min, 20, 95);  // min 20%, max 95%
                    break;
                case MENU_KICK_PCT:
                    edit_kick_pct += delta * 5;
                    edit_kick_pct  = constrain(edit_kick_pct, 50, 100); // kick at least 50%
                    break;
            }
            lcdClearCache();
        }

        if (btn) {
            cfg_fan_on       = edit_fan_on;
            cfg_fan_off      = edit_fan_off;
            cfg_fan_duty_min = (uint8_t)edit_duty_min;
            cfg_fan_kick_pct = (uint8_t)edit_kick_pct;
            menuEditing      = false;
            lcdClearCache();
        }
    }

    // Timeout -> return to monitor without saving
    if (millis() - menuLastActivity > MENU_TIMEOUT_MS) {
        appState    = STATE_MONITOR;
        menuEditing = false;
        lcdClearCache();
    }
}

// ---------------------------------------------------------------------------
// Global loop state
// ---------------------------------------------------------------------------
static uint32_t last_update_ms = 0;

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    pinMode(PIN_ENC_CLK, INPUT_PULLUP);
    pinMode(PIN_ENC_DT,  INPUT_PULLUP);
    pinMode(PIN_ENC_SW,  INPUT_PULLUP);
    enc_last_clk = digitalRead(PIN_ENC_CLK);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), encoderISR, CHANGE);

    fanPwmSetup();

#ifdef TACH_ENABLED
    tachSetup();
#endif

    MyLCD.init();
    MyLCD.backlight();
    lcdClearCache();

    // Splash screen
    MyLCD.setCursor(0, 0); MyLCD.print("PA Monitor v3.0-3W  ");
    MyLCD.setCursor(0, 1); MyLCD.print("by IU0PXK           ");
    MyLCD.setCursor(0, 2); MyLCD.print("Fan: 3-wire  61Hz   ");
    MyLCD.setCursor(0, 3); MyLCD.print("Press ENC for menu  ");
    delay(2500);
    lcdClearCache();

    eepromLoadSettings();

#ifdef DEBUG
    Serial.begin(115200);
    Serial.println(F("PA Monitor v3.0-3wire DEBUG"));
    Serial.print(F("Fan ON="));       Serial.print(cfg_fan_on);
    Serial.print(F(" OFF="));         Serial.print(cfg_fan_off);
    Serial.print(F(" DutyMin="));     Serial.print(cfg_fan_duty_min);
    Serial.print(F("% Kick="));       Serial.print(cfg_fan_kick_pct);
    Serial.println('%');
#endif
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
    uint32_t now = millis();

#ifdef TACH_ENABLED
    tachUpdate();
#endif

    // Menu access
    if (appState == STATE_MONITOR && buttonPressed()) {
        appState         = STATE_MENU;
        menuIndex        = 0;
        menuEditing      = false;
        menuLastActivity = now;
        lcdClearCache();
    }

    if (appState == STATE_MENU) {
        handleMenuInput();
        drawMenu();
        return;
    }

    // --- Monitor ---
    if (now - last_update_ms < LOOP_INTERVAL_MS) return;
    last_update_ms = now;

    // 1. ADC read
    float raw_temp = adcAverage(PIN_TEMP, ADC_SAMPLES);
    float raw_swr  = adcAverage(PIN_SWR,  ADC_SAMPLES);
    float raw_pwr  = adcAverage(PIN_PWR,  ADC_SAMPLES);

    // 2. Temperature + EMA
    float temp_mV = raw_temp * ADC_MV_PER_STEP;
    float temp_c  = (temp_mV - TEMP_OFFSET_MV) / TEMP_MV_PER_DEG;
    if (!temp_filter_initialized) {
        temp_filtered_c         = temp_c;
        temp_filter_initialized = true;
    } else {
        temp_filtered_c += TEMP_FILTER_ALPHA * (temp_c - temp_filtered_c);
    }

    // 3. Power
    float pwr_mV    = raw_pwr * ADC_MV_PER_STEP;
    float pwr_adj   = pwr_mV - PWR_ADC_OFFSET_MV;
    float pwr_watts = evalPoly(pwr_adj / 1000.0f, PWR_COEFFS, PWR_DEG);
    if (!isfinite(pwr_watts) || pwr_watts < 0.0f || pwr_watts > 200.0f)
        pwr_watts = (pwr_adj * pwr_adj) / 1000000.0f;
    if (raw_pwr < RAW_PWR_NOISE_FLOOR || pwr_watts < PWR_MIN_WATTS)
        pwr_watts = 0.0f;

    // 4. SWR
    float swr_mV  = raw_swr * ADC_MV_PER_STEP;
    float swr_adj = swr_mV - SWR_ADC_OFFSET_MV;
    float swr_val = 1.0f;
    bool  tx_active = (raw_pwr >= RAW_PWR_NOISE_FLOOR)
                   && (pwr_watts >= PWR_MIN_WATTS)
                   && (raw_swr  >= RAW_SWR_NOISE_FLOOR);
    if (tx_active) {
        float fwd = (pwr_adj > 0.0f) ? pwr_adj : 0.0f;
        float ref = (swr_adj > 0.0f) ? swr_adj : 0.0f;
        if (fwd > EPS) {
            float gamma = constrain(ref / fwd, 0.0f, 0.98f);
            swr_val = (1.0f + gamma) / (1.0f - gamma);
        }
    }
    if (!isfinite(swr_val) || swr_val < 1.0f) swr_val = 1.0f;
    if (swr_val > SWR_MAX_DISPLAY + 0.005f)   swr_val = SWR_MAX_DISPLAY;

    // 5. Fan
    updateFan(temp_filtered_c);

    // 6. Display
    drawMonitor(temp_filtered_c, pwr_watts, swr_val);

#ifdef DEBUG
    Serial.print(F("T="));      Serial.print(temp_filtered_c);
    Serial.print(F(" P="));     Serial.print(pwr_watts);
    Serial.print(F(" SWR="));   Serial.print(swr_val);
    Serial.print(F(" Fan="));   Serial.print(fan_running ? (fan_in_kickstart ? "KS" : "ON") : "OFF");
    Serial.print(F(" OCR2A=")); Serial.print(OCR2A);
    Serial.print(F(" duty="));
    Serial.print((uint16_t)OCR2A * 100U / 255U);
    Serial.print('%');
#ifdef TACH_ENABLED
    Serial.print(F(" RPM=")); Serial.print(fan_rpm);
#endif
    Serial.println();
#endif
}
