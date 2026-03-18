import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

fs = 1000.0       # IMU Frequency
num_taps = 51     # Length of filter

# Low Pass Filter at 350 Hz
b_accel_general = signal.firwin(num_taps, 350, pass_zero=True, fs=fs)

## --- Z-Axis Filter (Target: ~277 Hz) ---
## We create a bandpass filter from 200Hz to 350Hz to capture the peak
#b_accel_z = signal.firwin(num_taps, [200, 350], pass_zero=False, fs=fs)
#
## --- X/Y-Axis Filter (Target: ~146 Hz) ---
## We create a bandpass filter from 100Hz to 200Hz
#b_accel_xy = signal.firwin(num_taps, [100, 200], pass_zero=False, fs=fs)

# Print for C++
print("General Coeffs:", ", ".join(map(str, b_accel_general)))
#print("Z Coeffs:", ", ".join(map(str, b_accel_z)))
#print("XY Coeffs:", ", ".join(map(str, b_accel_xy)))

Indices = np.arange(51)

# Create the plot
plt.figure(figsize=(8, 5))  # Optional: set figure size
plt.plot(Indices, b_accel_general, color='blue', linestyle='--', marker='o')

# Add labels
plt.title("IIR Filter Coefficients")
plt.xlabel("Index")
plt.ylabel("Coefficient Value")
plt.grid(True)

# Show it
plt.show()

