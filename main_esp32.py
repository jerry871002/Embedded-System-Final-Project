from machine import I2C,Pin
import utime
import ujson
from math import sqrt, pi, degrees, radians, atan2, asin, sin, cos
EARTH_GRAVITY_MS2 = 9.80665 # m/s2

from umqtt.simple import MQTTClient
import machine,utime,time,network

class SMBus(I2C):

    """ Provides an 'SMBus' module which supports some of the py-smbus
        i2c methods, as well as being a subclass of machine.I2C
        Hopefully this will allow you to run code that was targeted at
        py-smbus unmodified on micropython.
        Use it like you would the machine.I2C class:
            import usmbus.SMBus
            bus = SMBus(1, pins=('G15','G10'), baudrate=100000)
            bus.read_byte_data(addr, register)
            ... etc
    """

    def read_byte_data(self, addr, register):
        """ Read a single byte from register of device at addr
            Returns a single byte """
        return self.readfrom_mem(addr, register, 1)[0]

    def read_i2c_block_data(self, addr, register, length):
        """ Read a block of length from register of device at addr
            Returns a bytes object filled with whatever was read """
        return self.readfrom_mem(addr, register, length)

    def write_byte_data(self, addr, register, data):
        """ Write a single byte from buffer `data` to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(data, int):
            data = bytes([data])
        return self.writeto_mem(addr, register, data)

    def write_i2c_block_data(self, addr, register, data):
        """ Write multiple bytes of data to register of device at addr
            Returns None """
        # writeto_mem() expects something it can treat as a buffer
        if isinstance(data, int):
            data = bytes([data])
        return self.writeto_mem(addr, register, data)


bus = SMBus(1,scl=Pin(22), sda=Pin(21), freq=100000)
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

#G-sensor
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

#Gyro
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
    
#Magnetometer
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
        ACC_X_OFFSET = -0.03
        ACC_Y_OFFSET = 0.04
        ACC_Z_OFFSET = 0.059

        # compass caliberation
        COM_X_OFFSET = -205.2
        COM_Y_OFFSET = -152.2
        COM_Z_OFFSET = 56.5
        ANGLE_OFFSET = (4, 32) # (4, 29) for Taichung, (4, 32) for Hsinchu

        self.accel = ADXL345(ACC_X_OFFSET, ACC_Y_OFFSET, ACC_Z_OFFSET)
        self.gyro = L3G4200D()
        self.mag = HMC5883L(COM_X_OFFSET, COM_Y_OFFSET, COM_Z_OFFSET, ANGLE_OFFSET)

        
sensors = gy801()
acc = sensors.accel
mag = sensors.mag
gyro = sensors.gyro

mq_server = '192.168.1.8' # server IP
mq_id = 'esp00001' 
mq_topic = 'data'
mq_user ='IMU'
mq_pass ='0101xx'


wifi = network.WLAN(network.STA_IF)
wifi.active(True)
    

try:
    wifi.connect('Jerry_TOTOLINK_A950RG','fRog1615')
    print('start to connect wifi')
    for i in range(20):
        print('try to connect wifi in {}s'.format(i))
        utime.sleep(1)
        if wifi.isconnected():
            break          
    if wifi.isconnected():
        print('WiFi connection OK!')
        print('Network Config=',wifi.ifconfig())
    else:
        print('WiFi connection Error')

    mqClient0 = MQTTClient(mq_id, mq_server, user=mq_user, password=mq_pass)
    mqClient0.connect()
    i = 0
    while True:
        data = {
            'acc_X': acc.getX(),
            'acc_Y': acc.getY(),
            'acc_Z': acc.getZ(),
            'mag_X': mag.getX(),
            'mag_Y': mag.getY(),
            'mag_Z': mag.getZ(),
            'gyro_X': gyro.getX(),
            'gyro_Y': gyro.getY(),
            'gyro_Z': gyro.getZ(),
            'timestamp':utime.ticks_us(),
        }
        mq_message = ujson.dumps(data)
        mqClient0.publish(mq_topic, mq_message)
        i = i + 1
        print("message publish {}".format(i))
except Exception as e: print(e)