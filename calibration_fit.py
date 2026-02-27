"""
Simple script to fit polynomial to calibration data for power and SWR sensors.
Usage:
  1. Collect measurements: pairs of (adc_mV, actual_value) from known references.
  2. Save them in lists below or load from CSV.
  3. Run the script and it will print polynomial coefficients.

The coefficients can then be copied into IU0PXK_PA_Monitor.cpp arrays.
"""

import numpy as np

# example data - replace with your calibration measurements
# each entry: (input_mV, measured_output)
pwr_data = [
    (100.0, 0.0),
    (500.0, 1.2),
    (1000.0, 4.5),
    (2000.0, 12.8),
    (5000.0, 45.0),
]
swr_data = [
    (100.0, 0.0),
    (500.0, 1.0),
    (1000.0, 1.5),
    (2000.0, 2.0),
    (5000.0, 3.0),
]

# choose polynomial degree
pwr_deg = 2
swr_deg = 2

# helper to fit

def fit_poly(data, deg):
    x = np.array([d[0] for d in data]) / 1000.0  # scale to volts
    y = np.array([d[1] for d in data])
    coeffs = np.polyfit(x, y, deg)
    # np.polyfit returns highest degree first, reverse
    return coeffs[::-1]

print("Power polynomial coefficients (a0 + a1*x + ...):")
coeffs = fit_poly(pwr_data, pwr_deg)
for i, c in enumerate(coeffs):
    print(f"  {c:.6g}")

print("\nSWR polynomial coefficients (a0 + a1*x + ...):")
coeffs = fit_poly(swr_data, swr_deg)
for i, c in enumerate(coeffs):
    print(f"  {c:.6g}")

print("\nCopy these values into the source arrays and rebuild the sketch.")
