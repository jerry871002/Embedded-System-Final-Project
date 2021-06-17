import pandas as pd
import numpy as np
from math import degrees, atan2, asin, sin, cos
import ahrs

EARTH_GRAVITY_MS2 = 9.80665 # m/s2

def madgwick(acc, gyr, mag, freq, filename='madgwick.csv'):
    df = pd.DataFrame()
    for (w, x, y, z), (aX, aY, aZ) in zip(ahrs.filters.Madgwick(acc=acc, gyr=gyr, mag=mag, frequency=freq).Q, acc):
        roll, pitch, yaw = _quaternion2euler(w, x, y, z)

        lax, lay, laz = getLinearAcc(roll, pitch, yaw, aX, aY, aZ)
        elax, elay, elaz = R_x(roll) @ R_y(pitch) @ R_z(yaw) @ np.array([lax, lay, laz])

        df = df.append({
                'ROLL': degrees(roll), 
                'PITCH': degrees(pitch),
                'YAW': degrees(yaw),
                'LINEAR ACCELERATION X': lax,
                'LINEAR ACCELERATION Y': lay,
                'LINEAR ACCELERATION Z': laz,
                'EARTH LINEAR ACCELERATION X': elax,
                'EARTH LINEAR ACCELERATION Y': elay,
                'EARTH LINEAR ACCELERATION Z': elaz,
            }, ignore_index=True)

    df.to_csv(filename)
    return df

def kalman(acc, gyr, mag, freq, filename='kalman.csv'):
    df = pd.DataFrame()
    for (w, x, y, z), (aX, aY, aZ) in zip(ahrs.filters.EKF(acc=acc, gyr=gyr, mag=mag, frequency=freq).Q, acc):
        roll, pitch, yaw = _quaternion2euler(w, x, y, z)

        lax, lay, laz = getLinearAcc(roll, pitch, yaw, aX, aY, aZ)
        elax, elay, elaz = R_x(roll) @ R_y(pitch) @ R_z(yaw) @ np.array([lax, lay, laz])

        df = df.append({
                'ROLL': degrees(roll), 
                'PITCH': degrees(pitch),
                'YAW': degrees(yaw),
                'LINEAR ACCELERATION X': lax,
                'LINEAR ACCELERATION Y': lay,
                'LINEAR ACCELERATION Z': laz,
                'EARTH LINEAR ACCELERATION X': elax,
                'EARTH LINEAR ACCELERATION Y': elay,
                'EARTH LINEAR ACCELERATION Z': elaz,
            }, ignore_index=True)

    df.to_csv(filename)
    return df

def complementary(acc, gyr, mag, freq, filename='complementary.csv'):
    df = pd.DataFrame()
    for (w, x, y, z), (aX, aY, aZ) in zip(ahrs.filters.Complementary(acc=acc, gyr=gyr, mag=mag, frequency=freq).Q, acc):
        roll, pitch, yaw = _quaternion2euler(w, x, y, z)

        lax, lay, laz = getLinearAcc(roll, pitch, yaw, aX, aY, aZ)
        elax, elay, elaz = R_x(roll) @ R_y(pitch) @ R_z(yaw) @ np.array([lax, lay, laz])

        df = df.append({
                'ROLL': degrees(roll), 
                'PITCH': degrees(pitch),
                'YAW': degrees(yaw),
                'LINEAR ACCELERATION X': lax,
                'LINEAR ACCELERATION Y': lay,
                'LINEAR ACCELERATION Z': laz,
                'EARTH LINEAR ACCELERATION X': elax,
                'EARTH LINEAR ACCELERATION Y': elay,
                'EARTH LINEAR ACCELERATION Z': elaz,
            }, ignore_index=True)

    df.to_csv(filename)
    return df

# rotation matrices
def R_x(x):
    # body frame rotation about x axis
    return np.array([[1,       0,        0],
                     [0, cos(-x), -sin(-x)],
                     [0, sin(-x),  cos(-x)]])
def R_y(y):
    # body frame rotation about y axis
    return np.array([[cos(-y), 0, -sin(-y)],
                     [      0, 1,        0],
                     [sin(-y), 0,  cos(-y)]])
def R_z(z):
    # body frame rotation about z axis
    return np.array([[cos(-z), -sin(-z), 0],
                     [sin(-z),  cos(-z), 0],
                     [      0,        0, 1]])

def getGravity(roll, pitch, yaw):
    return (R_x(roll) @ R_y(pitch) @ R_z(yaw) @ np.array([0, 0, 1])) * EARTH_GRAVITY_MS2

def getLinearAcc(roll, pitch, yaw, aX, aY, aZ):
    gravX, gravY, gravZ = getGravity(roll, pitch, yaw)
    return aX - gravX, aY - gravY, aZ - gravZ

def _quaternion2euler(w, x, y, z):
    roll = atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    pitch = asin(2*w*y - 2*x*z)
    yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))

    return roll, pitch, yaw