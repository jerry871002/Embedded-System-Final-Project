from imu import gy801
import time

sensor = gy801()

while True:
    print(f'{sensor.accel.getXg():4.2f}\t{sensor.accel.getYg():4.2f}\t{sensor.accel.getZg():4.2f}')
    time.sleep(0.5)