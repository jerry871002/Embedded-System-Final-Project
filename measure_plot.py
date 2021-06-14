from imu import gy801
import time
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

while True:
    try:
        sensor = gy801()
        break
    except OSError:
        print('Init fail, try again...')
        time.sleep(1)


df = sensor.measure(5)
print(df)

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
