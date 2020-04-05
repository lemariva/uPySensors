'''
'''
# Power Modes
from machine import I2C, Pin
from micropython import const
from ustruct import unpack as unp
import utime 

epoch_offset = 946684800

NORMAL = const(0)

# BME680 Temperature Registers
BME680_REG_DIG_T1 = const(0xE9)
BME680_REG_DIG_T2 = const(0x8A)
BME680_REG_DIG_T3 = const(0x8C)
# BME680 Pressure Registers
BME680_REG_DIG_P1 = const(0x8E)
BME680_REG_DIG_P2 = const(0x90)
BME680_REG_DIG_P3 = const(0x92)
BME680_REG_DIG_P4 = const(0x94)
BME680_REG_DIG_P5 = const(0x96)
BME680_REG_DIG_P6 = const(0x99)
BME680_REG_DIG_P7 = const(0x98)
BME680_REG_DIG_P8 = const(0x9C)
BME680_REG_DIG_P9 = const(0x9E)
BME680_REG_DIG_P10 = const(0xA0)
# BME680 Humidity Registers
BME680_REG_DIG_H1_LSB = const(0xE1)
BME680_REG_DIG_H1_MSB = const(0xE3)
BME680_REG_DIG_H2_LSB = const(0xE2)
BME680_REG_DIG_H2_MSB = const(0xE1)
BME680_REG_DIG_H3 = const(0xE4)
BME680_REG_DIG_H4 = const(0xE5)
BME680_REG_DIG_H5 = const(0xE6)
BME680_REG_DIG_H6 = const(0xE7)
BME680_REG_DIG_H7 = const(0xE8)
# BME680 Gas Sensor
BME680_REG_DIG_G1 = const(0xED)
BME680_REG_DIG_G2 = const(0xE7)
BME680_REG_DIG_G3 = const(0xEE)

BME680_REG_ID = const(0xD0)
BME680_NEW_DATA_MSK = const(0x80)
BME680_REG_RESET = const(0xE0)
BME680_RES_HEAT_0 = const(0x5A)
BME680_GAS_WAIT_0 = const(0x64)
BME680_HEAT_STAB_MSK = const(0x10)

BME680_GAS_INDEX_MSK = const(0x0F)
BME680_GAS_RANGE_MSK = const(0x0F)
BME680_GASM_VALID_MSK = const(0x20)

BME680_REG_CTRL_GAS = const(0x71)
BME680_REG_CTRL_HUM = const(0x72)
BME680_REG_STATUS = const(0xF3)
BME680_REG_CTRL_MEAS = const(0x74)
BME680_REG_CONFIG = const(0x75)  # IIR filter config

BME680_REG_MEAS_STATUS = const(0x1D)
BME680_REG_PDATA = const(0x1F)
BME680_REG_TDATA = const(0x22)
BME680_REG_HDATA = const(0x25)
BME680_MAX_OVERFLOW_VAL =const(0x40000000)

BMP680_I2C_ADDR = const(0x77)

BME680_SAMPLERATES = (0, 1, 2, 4, 8, 16)
BME680_FILTERSIZES = (0, 1, 3, 7, 15, 31, 63, 127)
BME680_RUNGAS = const(0x10)

_LOOKUP_TABLE_1 = (
    2147483647.0,
    2147483647.0,
    2147483647.0,
    2147483647.0,
    2147483647.0,
    2126008810.0,
    2147483647.0,
    2130303777.0,
    2147483647.0,
    2147483647.0,
    2143188679.0,
    2136746228.0,
    2147483647.0,
    2126008810.0,
    2147483647.0,
    2147483647.0,
)

_LOOKUP_TABLE_2 = (
    4096000000.0,
    2048000000.0,
    1024000000.0,
    512000000.0,
    255744255.0,
    127110228.0,
    64000000.0,
    32258064.0,
    16016016.0,
    8000000.0,
    4000000.0,
    2000000.0,
    1000000.0,
    500000.0,
    250000.0,
    125000.0,
)

class MPUException(OSError):
    '''
    Exception for MPU devices
    '''
    pass

class GasSettings:
    def __init__(self):
        # Variable to store nb conversion
        self.nb_conv = None
        # Variable to store heater control
        self.heatr_ctrl = None
        # Run gas enable value
        self.run_gas = None
        # Pointer to store heater temperature
        self.heatr_temp = None
        # Pointer to store duration profile
        self.heatr_dur = None

class TPHSettings:
    def __init__(self):
        # Humidity oversampling
        self.os_hum = None
        # Temperature oversampling
        self.os_temp = None
        # Pressure oversampling
        self.os_pres = None
        # Filter coefficient
        self.filter = None

class BME680():
    _I2Cerror = "I2C failure when communicating with the BMP/E"
    # BME680 = 0x61
    _chip_id = 0x61

    _i2c_addr = BMP680_I2C_ADDR

    def __init__(self, i2c, pins):
        
        self._pins = pins
        
        self._buf1 = bytearray(1)
        self._buf2 = bytearray(2)

        scl_pin = Pin(self._pins["scl"], Pin.OUT)
        sda_pin = Pin(self._pins["sda"], Pin.IN)

        self._i2c = I2C(i2c, scl=scl_pin, sda=sda_pin)
        
        self.chip_id

        self._load_calibration()

        self.power_on()

        # Sensor settings
        self.tph_settings = TPHSettings()
        # Gas Sensor settings
        self.gas_settings = GasSettings()
        
        # Default oversampling and filter register values.
        self.tph_settings.os_pres = 0b011
        self.tph_settings.os_temp = 0b100
        self.tph_settings.os_hum = 0b010
        self.tph_settings.filter = 0b010

        # gas measurements enabled
        self._write(BME680_REG_CTRL_GAS, BME680_RUNGAS)

        # RAW measurements
        self._p_raw = 0
        self._t_raw = 0
        self._h_raw = 0
        self._g_raw = 0

        # Calibrated measurements
        self._t_fine = 0
        self._t = 0
        self._h = 0
        self._p = 0
        self._g = 0

        self._g_range = 0
        self._g_stable = 0

        self._read_wait_ms = 100 
        self._new_read_ms = 200 
        self._last_read_ts = 0

    def _read(self, memaddr, size=1):
        data = self._i2c.readfrom_mem(self._i2c_addr, memaddr, size)
        return data
        
    def _write(self, addr, b_arr):
        if not type(b_arr) is bytearray:
            b_arr = bytearray([b_arr])
        return self._i2c.writeto_mem(self._i2c_addr, addr, b_arr)

    def _load_calibration(self):
        # read calibration data
        # < little-endian
        # H unsigned short
        # h signed short
        self._T1 = unp('<H', self._read(BME680_REG_DIG_T1, 2))[0]
        self._T2 = unp('<h', self._read(BME680_REG_DIG_T2, 2))[0]
        self._T3 = unp('<b', self._read(BME680_REG_DIG_T3, 1))[0]

        self._P1 = unp('<H', self._read(BME680_REG_DIG_P1, 2))[0]
        self._P2 = unp('<h', self._read(BME680_REG_DIG_P2, 2))[0]
        self._P3 = unp('<b', self._read(BME680_REG_DIG_P3, 1))[0]
        self._P4 = unp('<h', self._read(BME680_REG_DIG_P4, 2))[0]
        self._P5 = unp('<h', self._read(BME680_REG_DIG_P5, 2))[0]
        self._P6 = unp('<h', self._read(BME680_REG_DIG_P6, 2))[0]
        self._P7 = unp('<b', self._read(BME680_REG_DIG_P7, 2))[0]
        self._P8 = unp('<h', self._read(BME680_REG_DIG_P8, 2))[0]
        self._P9 = unp('<h', self._read(BME680_REG_DIG_P9, 2))[0]
        self._P10 = unp('<b', self._read(BME680_REG_DIG_P10, 1))[0]

        self._H1 = ((unp('<b', self._read(BME680_REG_DIG_H1_MSB, 1))[0] << 4)
			| (unp('<b', self._read(BME680_REG_DIG_H1_LSB, 1))[0] & 0x0F))
        self._H2 = ((unp('<b', self._read(BME680_REG_DIG_H2_MSB, 1))[0] << 4)
			| (unp('<b', self._read(BME680_REG_DIG_H2_LSB, 1))[0] & 0x0F))
        self._H3 = unp('<b', self._read(BME680_REG_DIG_H3, 1))[0]
        self._H4 = unp('<b', self._read(BME680_REG_DIG_H4, 1))[0]
        self._H5 = unp('<b', self._read(BME680_REG_DIG_H5, 1))[0]
        self._H6 = unp('<b', self._read(BME680_REG_DIG_H6, 1))[0]
        self._H7 = unp('<b', self._read(BME680_REG_DIG_H7, 1))[0]
        
        self._G1 = unp('<b', self._read(BME680_REG_DIG_G1, 1))[0]
        self._G2 = unp('<h', self._read(BME680_REG_DIG_G2, 2))[0]
        self._G3 = unp('<b', self._read(BME680_REG_DIG_G3, 1))[0]

        self._heat_range = (unp('<b', self._read(0x02, 1))[0] & 0x30) / 16
        self._heat_val = unp('<b', self._read(0x00, 1))[0]
        self._sw_err = (unp('<b', self._read(0x04, 1))[0] & 0xF0) / 16
        
    def print_calibration(self):
        print("T1: {} {}".format(self._T1, type(self._T1)))
        print("T2: {} {}".format(self._T2, type(self._T2)))
        print("T3: {} {}".format(self._T3, type(self._T3)))
        print("P1: {} {}".format(self._P1, type(self._P1)))
        print("P2: {} {}".format(self._P2, type(self._P2)))
        print("P3: {} {}".format(self._P3, type(self._P3)))
        print("P4: {} {}".format(self._P4, type(self._P4)))
        print("P5: {} {}".format(self._P5, type(self._P5)))
        print("P6: {} {}".format(self._P6, type(self._P6)))
        print("P7: {} {}".format(self._P7, type(self._P7)))
        print("P8: {} {}".format(self._P8, type(self._P8)))
        print("P9: {} {}".format(self._P9, type(self._P9)))
        print("P10: {} {}".format(self._P10, type(self._P10)))
        print("H1: {} {}".format(self._H1, type(self._H1)))
        print("H2: {} {}".format(self._H2, type(self._H2)))
        print("H3: {} {}".format(self._H3, type(self._H3)))
        print("H4: {} {}".format(self._H4, type(self._H4)))
        print("H5: {} {}".format(self._H5, type(self._H5)))
        print("H6: {} {}".format(self._H6, type(self._H6)))
        print("G1: {} {}".format(self._G1, type(self._G1)))
        print("G2: {} {}".format(self._G2, type(self._G2)))
        print("G3: {} {}".format(self._G3, type(self._G3)))
        print("heater_range: {} {}".format(self._heat_range, type(self._heat_range)))
        print("heat_val: {} {}".format(self._heat_val, type(self._heat_val)))
        print("sw_err: {} {}".format(self._sw_err, type(self._sw_err)))

    def power_off(self):
        self._write(0xF4, 0)

    # normal mode
    def power_on(self):
        self._write(0xF4, 0x2F)

    def _gauge(self):
        """Perform a single-shot reading from the sensor and fill internal data structure for
           calculations"""
        now = utime.ticks_ms()
        if utime.ticks_diff(now, self._last_read_ts) > self._new_read_ms:
            # set filter
            self._write(BME680_REG_CONFIG, self.tph_settings.filter << 2)
            # turn on temp oversample & pressure oversample
            self._write(
                BME680_REG_CTRL_MEAS,
                (self.tph_settings.os_temp << 5) | (self.tph_settings.os_pres << 2),
            )
            utime.sleep_ms(100)

            # turn on humidity oversample
            self._write(BME680_REG_CTRL_HUM, self.tph_settings.os_hum)

            ctrl = unp('<b', self._read(BME680_REG_CTRL_MEAS, 1))[0]
            ctrl = (ctrl & 0xFC) | 0x01  # enable single shot!
            self._write(BME680_REG_CTRL_MEAS, ctrl)
            
            data_status = False
            while not data_status:
                regs = self._read(BME680_REG_MEAS_STATUS, 15)
                data_status = regs[0] & BME680_NEW_DATA_MSK != 0
                utime.sleep_ms(5)
            
            self._last_read_ts = utime.ticks_ms()

            self._p_raw  = (regs[2] << 12) | (regs[3] << 4) | (regs[4] >> 4)
            self._t_raw  = (regs[5] << 12) | (regs[6] << 4) | (regs[7] >> 4)
            self._h_raw  = (regs[8] << 8) | regs[9]

            self._g_raw  = (regs[13] << 2) | (regs[14] >> 6)
            self._g_range  = regs[14] & BME680_GAS_RANGE_MSK
            self._g_stable = (data_status & BME680_HEAT_STAB_MSK) > 0

            self._t_fine = 0
            self._t = 0
            self._g = 0
            self._h = 0
            self._p = 0

    def _calc_t_fine(self):
        # From datasheet page 22
        self._gauge()
        if self._t_fine == 0:
            var1 = (((self._t_raw / 16384.0) - (self._T1 / 1024.0)) * self._T2)
            var2 = ((((self._t_raw / 131072.0) - (self._T1 / 8192.0)) *
                ((self._t_raw / 131072.0) - (self._T1 / 8192.0))) *
                (self._T3 * 16.0))
            self._t_fine = var1 + var2
        
        self._t = ((self._t_fine * 5) + 128) / 256 / 100


    def set_gas_heater_profile(self, temperature, duration, nb_profile=0):

        if nb_profile > 9 or nb_profile < 0:
            raise ValueError("Profile '{}' should be between {} and {}".format(nb_profile, 0, 9)) 

        # set temperature
        self.gas_settings.heatr_temp = temperature
        temp = int(self._calc_heater_resistance(temperature))
        self._write(BME680_RES_HEAT_0+ nb_profile, temp)

        # set duration
        self.gas_settings.heatr_dur = duration
        temp = self._calc_heater_duration(duration)
        self._write(BME680_GAS_WAIT_0 + nb_profile, temp)


    def _calc_heater_resistance(self, temperature):
        temperature = min(max(temperature,200),400)

        var1 = ((self._t * self._G3) / 1000) * 256
        var2 = (self._G1 + 784) * (((((self._G2 + 154009) * temperature * 5) / 100) + 3276800) / 10)
        var3 = var1 + (var2 / 2)
        var4 = (var3 / (self._g_range + 4))
        var5 = (131 * self._heat_val) + 65536
        heatr_res_x100 = (((var4 / var5) - 250) * 34)
        heatr_res = ((heatr_res_x100 + 50) / 100)

        return heatr_res

    def _calc_heater_duration(self, duration):
        if duration < 0xfc0:
            factor = 0

            while duration > 0x3f:
                duration /= 4
                factor += 1

            return int(duration + (factor * 64))

        return 0xff

    @property
    def gas(self):
        """The gas resistance in ohms"""
        self._calc_t_fine()
        if self._g == 0:
            var1 = int(
                (1340 + (5 * self._sw_err)) * (_LOOKUP_TABLE_1[self._g_range])
            ) >> 16
            var2 = ((self._g_raw << 15) - 16777216) + var1
            var3 = int(_LOOKUP_TABLE_2[self._g_range] * var1) >> 9
            self._g = (var3 + (var2 / 2)) / var2
        
        return self._g

    @property
    def filter_size(self):
        """The filter size for the built in IIR filter"""
        return BME680_FILTERSIZES[self._filter]

    @property
    def humidity(self):
        """The relative humidity in RH %"""
        self._calc_t_fine()
        if self._h == 0:
            temp_scaled = ((self._t_fine * 5) + 128) / 256
            var1 = (self._h_raw - (self._H1 * 16)) - (
                (temp_scaled * self._H3) / 200
            )
            var2 = (
                self._H2
                * (
                    ((temp_scaled * self._H4) / 100)
                    + (
                        (
                            (
                                temp_scaled
                                * ((temp_scaled * self._H5) / 100)
                            )
                            / 64
                        )
                        / 100
                    )
                    + 16384
                )
            ) / 1024
            var3 = var1 * var2
            var4 = self._H6 * 128
            var4 = (var4 + ((temp_scaled * self._H7) / 100)) / 16
            var5 = ((var3 / 16384) * (var3 / 16384)) / 1024
            var6 = (var4 * var5) / 2
            calc_hum = (((var3 + var6) / 1024) * 1000) / 4096
            self._h = calc_hum / 1000  # get back to RH
        return self._h

    @property
    def temperature(self):
        self._calc_t_fine()
        self._t = ((self._t_fine * 5) + 128) / 256 / 100
        return self._t 

    @property
    def pressure(self):
        self._calc_t_fine()
        if self._p == 0:
            var1 = (int(self._t_fine) >> 1) - 64000
            var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * self._P6) >> 2
            var2 = var2 + ((var1 * self._P5) << 1)
            var2 = (var2 >> 2) + (self._P4 << 16)
            var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * \
                    (self._P3 << 5)) >> 3) + ((self._P2 * var1) >> 1)
            var1 = var1 >> 18
            var1 = ((32768 + var1) * self._P1) >> 15
            pressure_comp = 1048576 - self._p_raw
            pressure_comp = int((pressure_comp - (var2 >> 12)) * 3125)
            
            if pressure_comp >= BME680_MAX_OVERFLOW_VAL:
                pressure_comp = (int(pressure_comp / var1) << 1)
            else:
                pressure_comp = (pressure_comp << 1) / var1
            

            var1 = (self._P9 * ((pressure_comp >> 3) * \
                (pressure_comp >> 3)) >> 13) >> 12
            var2 = ((pressure_comp >> 2) * self._P8) >> 13 
            var3 = ((pressure_comp >> 8) * (pressure_comp >> 8) * \
		        (pressure_comp >> 8) * self._P10) >> 17

        self._p = (pressure_comp + ((var1 + var2 + var3 + (self._P7 << 7)) >> 4)) / 100
        
        return self._p

    @property
    def measurements(self):
        d = {}
        d['time'] = utime.time() + epoch_offset
        d['temp'] = self.temperature
        d['hum'] = self.humidity
        d['press'] = self.pressure
        d['gas'] = self.gas
        return d
        
    @property
    def chip_id(self):
        '''
        Returns Chip ID
        '''
        try:
            chip_id = unp('<b',self._read(const(0xD0), 1))[0]
        except OSError:
            raise MPUException(self._I2Cerror)
        if chip_id != self._chip_id:
            raise ValueError('Bad chip ID ({0}!={1}) retrieved: MPU communication failure'.format(chip_id, self._chip_id))
        return chip_id