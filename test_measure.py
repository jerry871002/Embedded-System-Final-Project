from imu import gy801
import time

sensor = gy801()

while True:
    print(f'x = {sensor.accel.getX():5.3f},\ty = {sensor.accel.getY():5.3f},\tz = {sensor.accel.getZ():5.3f} (m/s2)')
    print(*sensor.getGravity())

    time.sleep(0.2)
