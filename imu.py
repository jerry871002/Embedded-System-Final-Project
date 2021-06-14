import smbus
import time
from math import sqrt, pi, degrees, radians, atan2, asin, sin, cos
import ahrs
import numpy as np
import pandas as pd

EARTH_GRAVITY_MS2 = 9.80665 # m/s2
bus = smbus.SMBus(1) # 0 for R-Pi Rev. 1, 1 for Rev. 2

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

class IMU(object):
    def write_byte(self, adr, value):
        bus.write_byte_data(self.ADDRESS, adr, value)
    
    def read_byte(self, adr):
        return bus.read_byte_data(self.ADDRESS, adr)

    def read_word(self, adr, rf=1):
        # rf = 1 Little Endian Format, rf = 0 Big Endian Format
        if rf == 1:
            low = self.read_byte(adr)
            high = self.read_byte(adr+1)
        else:
            high = self.read_byte(adr)
            low = self.read_byte(adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr, rf=1):
        val = self.read_word(adr, rf)
        if(val & (1 << 16 - 1)):
            return val - (1<<16)
        else:
            return val


ADXL345_ADDRESS          = 0x53 # I2C address
ADXL345_BW_RATE          = 0x2C # data rate and power mode control
ADXL345_POWER_CTL        = 0x2D # power-saving features control 
ADXL345_DATA_FORMAT      = 0x31 # data format control 
ADXL345_DATAX0           = 0x32
ADXL345_DATAY0           = 0x34
ADXL345_DATAZ0           = 0x36
ADXL345_BW_RATE_100HZ    = 0x0A # 0A = 0000 1111
ADXL345_MEASURE          = 0x08 # 08 = 0000 1000
ADXL345_SCALE_MULTIPLIER = 0.00390625 # G/LSP. 1/256 = 0.00390625

class ADXL345(IMU):
    ADDRESS = ADXL345_ADDRESS
    
    def __init__(self, X_OFFSET, Y_OFFSET, Z_OFFSET) :
        # class properties
        self.Xoffset = X_OFFSET # unit: G
        self.Yoffset = Y_OFFSET # unit: G
        self.Zoffset = Z_OFFSET # unit: G
        self.Xraw = 0.0
        self.Yraw = 0.0
        self.Zraw = 0.0
        self.Xg = 0.0
        self.Yg = 0.0
        self.Zg = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0

        self.df_value = 0b00001000 # Self test disabled, 4-wire interface, Full resolution, Range = +/-2g
        
        self.Xcalibr = ADXL345_SCALE_MULTIPLIER
        self.Ycalibr = ADXL345_SCALE_MULTIPLIER
        self.Zcalibr = ADXL345_SCALE_MULTIPLIER

        # Register 0x2C: BW_RATE
        self.write_byte(ADXL345_BW_RATE, ADXL345_BW_RATE_100HZ)    
        # write value= 0x0A = 00001111
        # D3-D0: The default value is 0x0A, 
        # which translates to a 100 Hz output data rate.

        # Register 0x2D: POWER_CTL 
        self.write_byte(ADXL345_POWER_CTL, ADXL345_MEASURE)    
        # write value: 0x08 = 00001000
        # D3=1: set 1 for measurement mode.

        # Register 0x31: DATA_FORMAT 
        self.write_byte(ADXL345_DATA_FORMAT, self.df_value)
        # write value=00001000
        # D3 = 1: the device is in full resolution mode, 
        # where the output resolution increases with the g range 
        # set by the range bits to maintain a 4 mg/LSB scale factor. 
        # D1 D0 = range. 00 = +-2g 
    
    # RAW readings in LPS
    def getRawX(self) :
        self.Xraw = self.read_word_2c(ADXL345_DATAX0)
        return self.Xraw

    def getRawY(self) :
        self.Yraw = self.read_word_2c(ADXL345_DATAY0)
        return self.Yraw
    
    def getRawZ(self) :
        self.Zraw = self.read_word_2c(ADXL345_DATAZ0)
        return self.Zraw

    # G related readings in g
    # similar to filter, combine current value with previous one
    # plf = 1 means it only uses "current reading"
    def getXg(self, plf=1.0) :
        self.Xg = (self.getRawX() * self.Xcalibr + self.Xoffset) * plf + (1.0 - plf) * self.Xg
        return self.Xg

    def getYg(self, plf=1.0) :
        self.Yg = (self.getRawY() * self.Ycalibr + self.Yoffset) * plf + (1.0 - plf) * self.Yg
        return self.Yg

    def getZg(self, plf=1.0) :
        self.Zg = (self.getRawZ() * self.Zcalibr + self.Zoffset) * plf + (1.0 - plf) * self.Zg
        return self.Zg
    
    # unit: m/s2
    def getX(self, plf=1.0) :
        self.X = self.getXg(plf) * EARTH_GRAVITY_MS2
        return self.X
    
    def getY(self, plf=1.0) :
        self.Y = self.getYg(plf) * EARTH_GRAVITY_MS2
        return self.Y
    
    def getZ(self, plf=1.0) :
        self.Z = self.getZg(plf) * EARTH_GRAVITY_MS2
        return self.Z

    def getPitch(self) :
        aX = self.getXg()
        aY = self.getYg()
        aZ = self.getZg()
        self.pitch = degrees(atan2(-aX, sqrt(aY * aY + aZ * aZ)))
        return self.pitch 

    def getRoll(self) :
        aX = self.getXg()
        aY = self.getYg()
        aZ = self.getZg()
        self.roll = degrees(atan2(-aY, sqrt(aX * aX + aZ * aZ)))
        return self.roll


L3G4200D_ADDRESS   = 0x69
L3G4200D_CTRL_REG1 = 0x20
L3G4200D_CTRL_REG4 = 0x23
L3G4200D_OUT_X_L   = 0x28
L3G4200D_OUT_Y_L   = 0x2A
L3G4200D_OUT_Z_L   = 0x2C

class L3G4200D(IMU):
    ADDRESS = L3G4200D_ADDRESS

    def __init__(self) :
        # class properties
        self.Xraw = 0.0
        self.Yraw = 0.0
        self.Zraw = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.Xangle = 0.0
        self.Yangle = 0.0
        self.Zangle = 0.0
        self.t0x = None
        self.t0y = None
        self.t0z = None

        self.gain_std = 0.00875 # dps/digit
        
        self.write_byte(L3G4200D_CTRL_REG1, 0x0F) # Normal mode, X, Y, Z-Axis enabled 0xB0
        self.write_byte(L3G4200D_CTRL_REG4, 0x80)

        self.setCalibration()

    def setCalibration(self) :
        gyr_r = self.read_byte(L3G4200D_CTRL_REG4)
        
        self.gain = 2 ** (gyr_r & 48 >> 4) * self.gain_std

    def getRawX(self):
        self.Xraw = self.read_word_2c(L3G4200D_OUT_X_L)
        return self.Xraw

    def getRawY(self):
        self.Yraw = self.read_word_2c(L3G4200D_OUT_Y_L)
        return self.Yraw

    def getRawZ(self):
        self.Zraw = self.read_word_2c(L3G4200D_OUT_Z_L)
        return self.Zraw

    # unit: dps
    def getX(self, plf=1.0):
        self.X = (self.getRawX() * self.gain) * plf + (1.0 - plf) * self.X
        return self.X

    def getY(self, plf=1.0):
        self.Y = (self.getRawY() * self.gain) * plf + (1.0 - plf) * self.Y
        return self.Y

    def getZ(self, plf=1.0):
        self.Z = (self.getRawZ() * self.gain) * plf + (1.0 - plf) * self.Z
        return self.Z
    
    def getXangle(self, plf=1.0) :
        if self.t0x is None: 
            self.t0x = time.time()

        t1x = time.time()
        LP = t1x - self.t0x
        self.t0x = t1x
        self.Xangle = self.getX(plf) * LP
        return self.Xangle
    
    def getYangle(self, plf=1.0) :
        if self.t0y is None: 
            self.t0y = time.time()

        t1y = time.time()
        LP = t1y - self.t0y
        self.t0y = t1y
        self.Yangle = self.getY(plf) * LP
        return self.Yangle
    
    def getZangle(self, plf=1.0) :
        if self.t0z is None: 
            self.t0z = time.time()

        t1z = time.time()
        LP = t1z - self.t0z
        self.t0z = t1z
        self.Zangle = self.getZ(plf) * LP
        return self.Zangle


HMC5883L_ADDRESS = 0x1E
HMC5883L_CRA     = 0x00
HMC5883L_CRB     = 0x01
HMC5883L_MR      = 0x02
HMC5883L_DO_X_H	 = 0x03
HMC5883L_DO_Z_H	 = 0x05
HMC5883L_DO_Y_H	 = 0x07

class HMC5883L(IMU):
    ADDRESS = HMC5883L_ADDRESS

    def __init__(self, X_OFFSET, Y_OFFSET, Z_OFFSET, ANGLE_OFFSET) :
        # class properties
        self.X = None
        self.Y = None
        self.Z = None
        self.angle = None
        self.Xoffset = X_OFFSET
        self.Yoffset = Y_OFFSET
        self.Zoffset = Z_OFFSET
        
        # Declination Angle
        DEG, MIN = ANGLE_OFFSET
        self.angle_offset = radians(-1 * (DEG + (MIN / 60))) # unit: radians
        # Formula: (deg + (min / 60.0)) / (180 / M_PI)
        # ex: Hsinchu = Magnetic Declination: -4 deg, 32 min
        # ex: Taichung = Magnetic Declination: -4 deg, 29 min
        # http://www.magnetic-declination.com/
        
        self.scale = 0.92 # convert bit value (LSB) to gauss

        self.write_byte(HMC5883L_CRA, 0b01110000) # configuration register A, set to 8 samples @ 15Hz
        self.write_byte(HMC5883L_CRB, 0b00100000) # configuration register B, 1.3 gain LSb / Gauss 1090 (default)
        self.write_byte(HMC5883L_MR, 0b00000000)  # mode register, continuous sampling

    # degree: gauss
    def getX(self):
        self.X = (self.read_word_2c(HMC5883L_DO_X_H) - self.Xoffset) * self.scale
        return self.X

    def getY(self):
        self.Y = (self.read_word_2c(HMC5883L_DO_Y_H) - self.Yoffset) * self.scale
        return self.Y

    def getZ(self):
        self.Z = (self.read_word_2c(HMC5883L_DO_Z_H) - self.Zoffset) * self.scale
        return self.Z


class gy801(object):
    def __init__(self):
        # accelorator caliberation
        ACC_X_OFFSET = 0.047
        ACC_Y_OFFSET = -0.004
        ACC_Z_OFFSET = 0.099

        # compass caliberation
        COM_X_OFFSET = 326.5
        COM_Y_OFFSET = 49.0
        COM_Z_OFFSET = -58.0
        ANGLE_OFFSET = (4, 29) # (4, 29) for Taichung, (4, 32) for Hsinchu

        self.accel = ADXL345(ACC_X_OFFSET, ACC_Y_OFFSET, ACC_Z_OFFSET)
        self.gyro = L3G4200D()
        self.mag = HMC5883L(COM_X_OFFSET, COM_Y_OFFSET, COM_Z_OFFSET, ANGLE_OFFSET)

    def getRoll(self):
        roll_pre = self.accel.getRoll()
        self.gyro.getXangle()
        return (roll_pre + self.gyro.getXangle()) * 0.9 + self.accel.getRoll() * 0.1
    
    def getPitch(self):
        pitch_pre = self.accel.getPitch()       
        self.gyro.getYangle()
        return (pitch_pre + self.gyro.getYangle()) * 0.9 + self.accel.getPitch() * 0.1    

    def getYaw(self):
        magx = self.mag.getX()
        magy = self.mag.getY()
        magz = self.mag.getZ()

        pitch = radians(self.getPitch())
        roll = radians(self.getRoll())

        compx = magx * cos(pitch) + magz * sin(pitch)
        compy = magx * sin(roll) * sin(pitch) + \
                magy * cos(roll) - \
                magz * sin(roll) * cos(pitch)

        return degrees(atan2(compy, compx) + self.mag.angle_offset)

    def getGravity(self):
        roll = radians(self.getRoll())
        pitch = radians(self.getPitch())
        yaw = radians(self.getYaw())

        return (R_x(roll) @ R_y(pitch) @ R_z(yaw) @ np.array([0, 0, -1])) * EARTH_GRAVITY_MS2

    def getLinearAcc(self):
        gravX, gravY, gravZ = self.getGravity()

        aX = self.accel.getX()
        aY = self.accel.getY()
        aZ = self.accel.getZ()

        return aX - gravX, aY - gravY, aZ - gravZ

    # algorithm for sensor fusion
    def ahrs(self):
        acc = []
        gyr = []
        mag = []

        for i in range(10):
            acc.append([self.accel.getX(), self.accel.getY(), self.accel.getZ()])
            gyr.append([self.gyro.getX(), self.gyro.getY(), self.gyro.getZ()])
            mag.append([self.gyro.getX(), self.gyro.getY(), self.gyro.getZ()])
            time.sleep(0.01)

        acc = np.array(acc) # unit: m/s2
        gyr = np.array(gyr) # unit: dps
        gyr = np.radians(gyr) # unit: rad/s
        mag = np.array(mag) # unit: gauss
        mag = 0.1 * mag # unit: mT

        acc = np.reshape(np.mean(acc, axis=0), (1, 3))
        gyr = np.reshape(np.mean(gyr, axis=0), (1, 3))
        mag = np.reshape(np.mean(mag, axis=0), (1, 3))
        
        x, y, z, w = ahrs.filters.Madgwick(acc=acc, gyr=gyr, mag=mag).Q[0]
        roll = degrees(atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z))
        pitch = degrees(asin(2*x*y + 2*z*w))
        yaw = degrees(atan2(2*x*w + 2*y*z, 1 - 2*x*x - 2*z*z))

        return roll, pitch, yaw

    # measure data for a duration, unit is second
    def measure(self, duration, filename='measure.csv'):
        df = pd.DataFrame()

        start_time = time.time()
        while time.time() - start_time <= duration:
            roll = radians(self.getRoll())
            pitch = radians(self.getPitch())
            yaw = radians(self.getYaw())

            lax, lay, laz = self.getLinearAcc()
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
            time.sleep(0.1)

        df.to_csv(filename)
        return df


