from micropython import const
from adafruit_register.i2c_struct import Struct, ROUnaryStruct
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
import adafruit_bus_device.i2c_device as i2cdevice
import time

_REG_PARTID = const(0x00)
_REG_REVISION = const(0x01)

_REG_CTRL1 = const(0x02)  # Serial interface and sensor enable
_REG_CTRL2 = const(0x03)  # Accelerometer settings
_REG_CTRL3 = const(0x04)  # Gyroscope settings
_REG_CTRL4 = const(0x05)  # Magnetomer settings (support not implemented in this driver yet)
_REG_CTRL5 = const(0x06)  # Sensor data processing settings
_REG_CTRL6 = const(0x07)  # Attitude Engine ODR and Motion on Demand
_REG_CTRL7 = const(0x08)  # Enable Sensors and Configure Data Reads

_REG_TEMP = const(0x33)  # Temperature sensor.

_REG_AX_L = const(0x35)  # Read accelerometer
# _REG_AX_H = const(0x36)
# _REG_AY_L = const(0x37)
# _REG_AY_H = const(0x38)
# _REG_AZ_L = const(0x39)
# _REG_AZ_H = const(0x3A)
_REG_GX_L = const(0x3B)  # read gyro
# _REG_GX_H = const(0x3C)
# _REG_GY_L = const(0x3D)
# _REG_GY_H = const(0x3E)
# _REG_GZ_L = const(0x3F)
# _REG_GZ_H = const(0x40)

_QMI8658_I2CADDR_DEFAULT = const(0X6B)

class SerialSensorEnable:  # _REG_CTRL1
    SIM    = 0b10000000
    SPI_AI = 0b01000000
    SPI_BE = 0b00100000
    DISBLE = 0b00000001

class AccelScale:  # Accelerometer scale, use with _REG_CTRL2
    Range_2g  = 0b000
    Range_4g  = 0b001
    Range_8g  = 0b010
    Range_16g = 0b011

class ODR:  # Accelerometer or Gyro output data rate, use with _REG_CTRL2 and _REG_CTRL3
    DR_8000HZ  = 0b0000
    DR_4000HZ  = 0b0001
    DR_2000HZ  = 0b0010
    DR_1000HZ  = 0b0011
    DR_500HZ   = 0b0100
    DR_250HZ   = 0b0101
    DR_125HZ   = 0b0110
    DR_62_5HZ  = 0b0111
    DR_31_25HZ = 0b1000
    DR_128HZ   = 0b1100  # Not valid for Gyro
    DR_21HZ    = 0b1101  # Not valid for Gyro
    DR_11HZ    = 0b1110  # Not valid for Gyro
    DR_3HZ     = 0b1111  # Not valid for Gyro

class MODR:  # External magnetometer output data rate, use with _REG_CTRL4
    DR_1000HZ  = 0b000
    DR_500HZ   = 0b001
    DR_250HZ   = 0b010
    DR_125HZ   = 0b011
    DR_62_5HZ  = 0b100
    DR_31_25HZ = 0b101

class GyroScale:  # Gyroscope scale, use with _REG_CTRL3
    Range_16dps   = 0b000
    Range_32dps   = 0b001
    Range_64dps   = 0b010
    Range_128dps  = 0b011
    Range_256dps  = 0b100
    Range_512dps  = 0b101
    Range_1024dps = 0b110
    Range_2048dps = 0b111

class LPFMode:  # Gyro and accelerometer low-pass filter Mode, use with _REG_CTRL5
    ODR_2_62_PCT = 0b00
    ODR_3_59_PCT = 0b01
    ODR_5_32_PCT = 0b10
    ODR_14_PCT   = 0b11

class AttitudeODR:  # Attitude engine output data rate, use with _REG_CTRL6
    DR_1HZ     = 0b000
    DR_2HZ     = 0b001
    DR_4HZ     = 0b010
    DR_8HZ     = 0b011
    DR_16HZ    = 0b100
    DR_32HZ    = 0b101
    DR_64HZ    = 0b110

class QMI8658(object):
    """Driver for the QMI8658 Accelerometer.
    :param ~busio.I2C i2c_bus: The I2C bus the MSA is connected to.
    """
    _part_id = ROUnaryStruct(_REG_PARTID, "<B")
    _revision = ROUnaryStruct(_REG_REVISION, "<B")
    _temperature = ROUnaryStruct(_REG_TEMP, "<H")
    _xyz_raw = Struct(_REG_AX_L, "<hhh")
    _gyro_raw = Struct(_REG_GX_L, "<hhh")

    # Basic device config
    _3w_spi_enable = RWBit(_REG_CTRL1, 7)
    _spi_auto_increment = RWBit(_REG_CTRL1, 6)
    _spi_big_endian = RWBit(_REG_CTRL1, 5)
    _sensor_disable = RWBit(_REG_CTRL1, 0)
    
    # Accelerometer config
    _accel_self_test = RWBit(_REG_CTRL2, 7)
    _accel_full_scale = RWBits(3, _REG_CTRL2, 4)
    _accel_output_data_rate = RWBits(4, _REG_CTRL2, 0)
    _accel_lpf_enable = RWBit(_REG_CTRL5, 0)
    _accel_lpf_mode = RWBits(2, _REG_CTRL5, 1)
    _accel_enable = RWBit(_REG_CTRL7, 0)

    # Gyro config
    _gyro_self_test = RWBit(_REG_CTRL3, 7)
    _gyro_full_scale = RWBits(3, _REG_CTRL3, 4)
    _gyro_output_data_rate = RWBits(4, _REG_CTRL3, 0)
    _gyro_lpf_enable = RWBit(_REG_CTRL5, 4)
    _gyro_lpf_mode = RWBits(2, _REG_CTRL5, 5)
    _gyro_enable = RWBit(_REG_CTRL7, 1)

    # External magnetometer config
    _magnetometer_device = RWBits(4, _REG_CTRL4, 3)
    _magnetometer_odr = RWBits(3, _REG_CTRL4, 0)

    # Motion on demand and attitude engine
    _motion_on_demand_enable = RWBit(_REG_CTRL6, 7)
    _attitude_engine_output_data_rate = RWBits(3, _REG_CTRL6, 0)

    def __init__(self, i2c_bus):
        self.i2c_device = i2cdevice.I2CDevice(i2c_bus, _QMI8658_I2CADDR_DEFAULT)

        if self._part_id != 0x05:
            raise AttributeError("Cannot find a QMI8658")

        # Not sure why, but the sample code from Waveshare sets these bits even though we're using I2C
        self._spi_auto_increment = True
        self._spi_big_endian = True

        # Configure the accelerometer
        self._accel_self_test = False
        self._accel_enable = True
        self._accel_full_scale = AccelScale.Range_8g
        self._accel_output_data_rate = ODR.DR_1000HZ
        self._accel_lpf_enable = True
        self._accel_lpf_mode = LPFMode.ODR_2_62_PCT

        # Configure the gyro
        self._gyro_self_test = False
        self._gyro_enable = True
        self._gyro_full_scale = GyroScale.Range_512dps
        self._gyro_output_data_rate = ODR.DR_1000HZ
        self._gyro_lpf_enable = True
        self._gyro_lpf_mode = LPFMode.ODR_2_62_PCT

        # We are not using these features by default
        self._magnetometer_device = False
        self._magnetometer_odr = False
        self._motion_on_demand_enable = False
        self._attitude_engine_output_data_rate = False

        time.sleep(.1)

        super().__init__()

    @property
    def temperature(self):
        return self._temperature / 256.0

    @property
    def acceleration(self):
        """The x, y, z acceleration values returned in a
        3-tuple in :math:`m / s ^ 2`"""

        if self._accel_full_scale == AccelScale.Range_2g:
            acc_lsb_div = 1 << 14
        elif self._accel_full_scale == AccelScale.Range_4g:
            acc_lsb_div = 1 << 13
        elif self._accel_full_scale == AccelScale.Range_8g:
            acc_lsb_div = 1 << 12
        elif self._accel_full_scale == AccelScale.Range_16g:
            acc_lsb_div = 1 << 11

        x, y, z = [a/acc_lsb_div for a in self._xyz_raw]
        return (x, y, z)

    @property
    def gyro(self):
        """The x, y, z gyroscope values returned in a
        3-tuple in :math:`d / s`"""

        if self._gyro_full_scale == GyroScale.Range_16dps:
            gyro_lsb_div = 2048
        if self._gyro_full_scale == GyroScale.Range_32dps:
            gyro_lsb_div = 1024
        if self._gyro_full_scale == GyroScale.Range_64dps:
            gyro_lsb_div = 512
        if self._gyro_full_scale == GyroScale.Range_128dps:
            gyro_lsb_div = 256
        if self._gyro_full_scale == GyroScale.Range_256dps:
            gyro_lsb_div = 128
        if self._gyro_full_scale == GyroScale.Range_512dps:
            gyro_lsb_div = 64
        if self._gyro_full_scale == GyroScale.Range_1024dps:
            gyro_lsb_div = 32
        if self._gyro_full_scale == GyroScale.Range_2048dps:
            gyro_lsb_div = 16

        xyz = [g/gyro_lsb_div for g in self._gyro_raw]
        return xyz
