from imu import gy801, EARTH_GRAVITY_MS2
import time
import pandas as pd
from plot import plot_trajectory, fft, plot_trajectory_attenuated

while True:
    try:
        sensor = gy801()
        break
    except OSError:
        print('Init fail, try again...')
        time.sleep(1)

MEASURE_DURATION = 5 # unit: s

df = sensor.measure_complementary(MEASURE_DURATION)
print(df)

freq = MEASURE_DURATION / len(df)

plot_trajectory(df, freq)
df = fft(df, freq)
plot_trajectory_attenuated(df, freq)