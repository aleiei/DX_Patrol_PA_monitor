#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <stdio.h>

LiquidCrystal_I2C MyLCD(0x27, 20, 4);

float temp_voltage = 0.0f;
float swr_voltage = 0.0f;
float pwr_voltage = 0.0f;

static const float ADC_MV_PER_STEP = 5000.0f / 1023.0f;
static const float EPS = 1e-6f;

void clearLcdLine(uint8_t row) {
    MyLCD.setCursor(0, row);
    for (uint8_t i = 0; i < 20; ++i) MyLCD.print(' ');
    MyLCD.setCursor(0, row);
}

// Evaluate polynomial c[0] + c[1]*x + ... + c[deg]*x^deg
static float evalPoly(float x, const float *c, int deg) {  float y = 0.0f;
    float xi = 1.0f;
    for (int i = 0; i <= deg; ++i) {
        y += c[i] * xi;
        xi *= x;
    }
    return y;
}

void setup() {
    MyLCD.init();
    MyLCD.backlight();
    MyLCD.setCursor(0, 0);
    MyLCD.print("PA Monitor by IU0PXK");
    Serial.begin(115200);
}

void loop() {
    // reset displayed values to zero before new measurements
    temp_voltage = 0.0f;
    swr_voltage  = 0.0f;
    pwr_voltage  = 0.0f;

    int raw_temp = analogRead(A0);
    int raw_swr  = analogRead(A1);
    int raw_pwr  = analogRead(A2);

    // temperature uses simple linear conversion
    float temp_mV = raw_temp * ADC_MV_PER_STEP;
    temp_voltage = (temp_mV - 464.0f) / 6.25f;

    // polynomial regression coefficients (example values)
    // these should be replaced with values obtained from calibration
    // NOTE: we evaluate the polynomials on a scaled voltage-like input
    const float pwrCoeffs[] = { 0.0f, 0.0f, 1.0f }; // placeholder: a0 + a1*x + a2*x^2 (x in volts)
    const int   pwrDeg     = sizeof(pwrCoeffs)/sizeof(pwrCoeffs[0]) - 1;
    const float swrCoeffs[] = { 0.0f, 1.0f, 0.0f };           // placeholder linear model
    const int   swrDeg     = sizeof(swrCoeffs)/sizeof(swrCoeffs[0]) - 1;

    // convert ADC to millivolts and compute adjusted values
    float swr_mV = raw_swr * ADC_MV_PER_STEP;
    float pwr_mV = raw_pwr * ADC_MV_PER_STEP;
    float swr_adj = swr_mV - 1288.0f; // original offset used previously
    float pwr_adj = pwr_mV - 298.0f;

    // scale inputs to approximately volt-range for polynomial (x in volts)
    float x_pwr = pwr_adj / 1000.0f; // volts (may be negative)
    float x_swr = swr_adj / 1000.0f;

    // evaluate polynomial using scaled inputs
    pwr_voltage = evalPoly(x_pwr, pwrCoeffs, pwrDeg);
    swr_voltage = evalPoly(x_swr, swrCoeffs, swrDeg);

    // fallback: if polynomial gives unrealistic results (too large or negative),
    // use previous empirical formula for power
    if (!isfinite(pwr_voltage) || pwr_voltage < 0.0f || pwr_voltage > 200.0f) {
        pwr_voltage = ((pwr_adj + 0.5f) * (pwr_adj + 0.5f)) / 1000000.0f;
    }

    // if the raw ADC or computed wattage is very small, treat as zero to avoid
    // noise making the display jump above 00,00
    if (raw_pwr < 8 || pwr_voltage < 0.001f) {
        pwr_voltage = 0.0f;
    }

    // remove any constant offset produced by the SWR polynomial at x=0
    float swr_baseline = evalPoly(0.0f, swrCoeffs, swrDeg);
    swr_voltage -= swr_baseline;

    // If measured power is very small or ADC reading is near zero,
    // consider SWR as 0 (no meaningful measurement)
    if (raw_pwr < 8 || pwr_voltage < 0.001f || raw_swr < 4) {
        swr_voltage = 0.0f;
    }

    // final sanity clamp
    if (!isfinite(swr_voltage)) swr_voltage = 0.0f;

    // If SWR looks unrealistic, compute alternative estimate using original empirical formula
    if (swr_voltage > 10.0f || swr_voltage < 0.0f) {
        float denom = (pwr_adj - swr_adj);
        float swr_alt = 0.0f;
        if (fabsf(denom) > EPS) {
            swr_alt = (pwr_adj + swr_adj) / denom;
        }
        if (isfinite(swr_alt) && swr_alt > 0.0f && swr_alt < 10.0f) {
            swr_voltage = swr_alt;
        } else {
            swr_voltage = 0.0f;
        }
    }

    if (swr_voltage < 0.0f) swr_voltage = 0.0f;

    clearLcdLine(1);
    MyLCD.setCursor(0, 1);
    MyLCD.print("TEMP C= ");
    MyLCD.print(temp_voltage, 1);

    clearLcdLine(2);
    MyLCD.setCursor(0, 2);
    MyLCD.print("SWR   = ");
    {
        // format swr exactly like power so it always shows two digits
        char buf[8];
        float sv = swr_voltage;
        if (sv < 0.0f) sv = 0.0f;
        // cap to 99.99 (allow rounding)
        if (sv > 99.995f) sv = 99.995f;
        int isv = (int)sv;
        int dsv = (int)roundf((sv - isv) * 100.0f);
        if (dsv >= 100) { isv++; dsv = 0; }
        if (isv > 99) { isv = 99; dsv = 99; }
        snprintf(buf, sizeof(buf), "%02d,%02d", isv, dsv);
        MyLCD.print(buf);
    }

    clearLcdLine(3);
    MyLCD.setCursor(0, 3);
    MyLCD.print("PWR W = ");
    {
        char buf[8];
        float pv = pwr_voltage;
        if (pv < 0.0f) pv = 0.0f;
        // cap to 99.99 (allow rounding)
        if (pv > 99.995f) pv = 99.995f;
        int ip = (int)pv;
        int dp = (int)roundf((pv - ip) * 100.0f);
        if (dp >= 100) { ip++; dp = 0; }
        if (ip > 99) { ip = 99; dp = 99; }
        snprintf(buf, sizeof(buf), "%02d,%02d", ip, dp);
        MyLCD.print(buf);
    }

    delay(300);

    // Serial debug output for calibration: raw ADC, mV, adjusted, and computed values
    Serial.print("raw_pwr="); Serial.print(raw_pwr);
    Serial.print(" pwr_mV="); Serial.print(pwr_mV);
    Serial.print(" pwr_adj="); Serial.print(pwr_adj);
    Serial.print(" pwr_W="); Serial.print(pwr_voltage);
    Serial.print(" | raw_swr="); Serial.print(raw_swr);
    Serial.print(" swr_mV="); Serial.print(swr_mV);
    Serial.print(" swr_adj="); Serial.print(swr_adj);
    Serial.print(" swr="); Serial.println(swr_voltage);
}
