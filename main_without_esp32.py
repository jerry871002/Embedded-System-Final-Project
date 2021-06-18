from imu import gy801
import time
from plot import plot_trajectory, fft, plot_trajectory_attenuated

if __name__ == '__main__':
    sensor = gy801()
    
    MEASURE_DURATION = 5 # unit: s

    df = sensor.measure_complementary(MEASURE_DURATION)
    print(df)

    freq = MEASURE_DURATION / len(df)

    plot_trajectory(df, freq)
    df = fft(df, freq)
    plot_trajectory_attenuated(df, freq)