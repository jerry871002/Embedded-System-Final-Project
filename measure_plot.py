from imu import gy801, EARTH_GRAVITY_MS2
import time
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import pandas as pd
import numpy as np

# while True:
#     try:
#         sensor = gy801()
#         break
#     except OSError:
#         print('Init fail, try again...')
#         time.sleep(1)

MEASURE_DURATION = 5 # unit: s

# df = sensor.measure(MEASURE_DURATION)
df = pd.read_csv('measure.csv')
print(df)

dt = MEASURE_DURATION / len(df)

df['EARTH LINEAR ACCELERATION X'] /= EARTH_GRAVITY_MS2
df['EARTH LINEAR ACCELERATION Y'] /= EARTH_GRAVITY_MS2
df['EARTH LINEAR ACCELERATION Z'] /= EARTH_GRAVITY_MS2

# Double integrate accelerations to find positions
x = cumtrapz(cumtrapz(df['EARTH LINEAR ACCELERATION X'], dx=dt), dx=dt)
y = cumtrapz(cumtrapz(df['EARTH LINEAR ACCELERATION Y'], dx=dt), dx=dt)
z = cumtrapz(cumtrapz(df['EARTH LINEAR ACCELERATION Z'], dx=dt), dx=dt)
# Plot 3D Trajectory
fig3, ax = plt.subplots()
fig3.suptitle('3D Trajectory of IMU', fontsize=20)
ax = plt.axes(projection='3d')
ax.plot3D(x, y, z, c='red', lw=5, label='IMU trajectory')
ax.set_xlabel('X position (m)')
ax.set_ylabel('Y position (m)')
ax.set_zlabel('Z position (m)')
plt.show()

# Try to remove noise via Fourier analysis
# Discrete Fourier Transform sample frequencies
freq = np.fft.rfftfreq(df['EARTH LINEAR ACCELERATION X'].size, d=dt)
# Compute the Fast Fourier Transform (FFT) of acceleration signals
fft_x = np.fft.rfft(df['EARTH LINEAR ACCELERATION X']) 
fft_y = np.fft.rfft(df['EARTH LINEAR ACCELERATION Y']) 
fft_z = np.fft.rfft(df['EARTH LINEAR ACCELERATION Z'])
# Plot Frequency spectrum
fig4, [ax1, ax2, ax3] = plt.subplots(3, 1, sharex=True, sharey=True)
fig4.suptitle('Noise Spectrum',fontsize=20)
ax1.plot(freq,abs(fft_x),c='r',label='x noise')
ax1.legend()
ax2.plot(freq,abs(fft_y),c='b',label='y noise')
ax2.legend()
ax3.plot(freq,abs(fft_z),c='g',label='z noise')
ax3.legend()
ax3.set_xlabel('Freqeuncy (Hz)')
plt.show()
