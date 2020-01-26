#!/usr/bin/env python3

import time
from collections import namedtuple
try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus


class RogySensor:

    SensorData = namedtuple('SensorData', ['name', 'val', 'units'])

    def __init__(self, sensor_type=None, history=5, samples_per_read=1):
        self.sensor_type = sensor_type
        self.active = False
        self.vals = []
        self.vals_ts = []
        self.history = history
        self.samples_per_read = samples_per_read

    def __str__(self):

        return ['At {0}: {1}={2}{3}'.format(self.vals_ts[i], self.vals[i].name, self.vals[i].val,
                                            self.vals[i].units) for i in range(0, len(self.vals))]

    def read(self, return_value=True, pretty_format=False):
        # Override this
        return [self.SensorData(name='na', val=-1, units='na')]

    def post_conversion(self, in_value):
        # Override this
        return in_value

    def _free_local(self):
        # Override this
        pass

    def free(self):
        if self.active is True:
            self._free_local()
        self.vals.clear()
        self.vals_ts.clear()


class RogySensorI2C(RogySensor):

    I2C_SENSORS = {
        'BMP280': {'chipid': 0x58}
    }

    i2c_bus = SMBus(1)

    def __init__(self, scl_pin=22, sda_pin=21, device='Unknown', history=5, samples_per_read=1):
        super().__init__(sensor_type='i2c', history=history, samples_per_read=samples_per_read)
        self._scl_pin = scl_pin
        self._sda_pin = sda_pin

    def _read_i2c(self):
        # Override this
        return [self.SensorData(name='na', val=-1, units='na')]

    def read_data_val(self, sensordata_name):
        self.read()
        for _sval in self.vals[0]:
            if _sval.name == sensordata_name:
                return _sval.val, '{0}{1} at {2}'.format(_sval.val, _sval.units, time.asctime(self.vals_ts[0]))

        else:
            return None, 'Offline'

    def read(self, return_value=True, pretty_format=False):
        if self.active is not True:
            return None

        self.vals.insert(0, self._read_i2c())

        if len(self.vals) > self.history:
            del self.vals[self.history]

        self.vals_ts.insert(0, time.localtime())
        if len(self.vals_ts) > self.history:
            del self.vals_ts[self.history]

        if return_value is True:
            if pretty_format is True:
                return ['{0}={1}{2}'.format(sd.name, sd.val, sd.units) for sd in self.vals[0]]

            else:
                return self.post_conversion(self.vals[0])
        else:
            return None


class RogyBMP280(RogySensorI2C):

    def __init__(self, scl_pin=22, sda_pin=21, history=5, samples_per_read=1):
        super().__init__(scl_pin=scl_pin, sda_pin=sda_pin, device='BMP280', history=history, samples_per_read=samples_per_read)

        # self.rs_i2c_device = MP_BMP280_I2C(i2c=self.rs_i2c, address=118)
        self._read_i2c = self._read_bmp280

        try:
            import bmp280
            # self.rs_i2c = machine.I2C(scl=machine.Pin(scl_pin), sda=machine.Pin(sda_pin))
            self._sensor = bmp280.BMP280(i2c_dev=self.i2c_bus)

        except ImportError as err:
            print('Cant start i2c sensor:', err)
            self.active = False
            return

        self.active = True

    def _read_bmp280(self):
        # return self.rs_i2c_device.temperature
        _st1 = 0
        _st2 = 0
        _st3 = 0
        for i in range(0, self.samples_per_read):
            _st1 += self._sensor.get_temperature()
            _st2 += self._sensor.get_pressure()
            _st3 += self._sensor.get_altitude()
            time.sleep(.1)

        # I'm American...convert to F
        _st1 = '{:.1f}'.format(((_st1 / self.samples_per_read) * 9/5) + 32)
        _st2 = '{:.2f}'.format(_st2 / self.samples_per_read)
        _st3 = '{:.2f}'.format(_st3 / self.samples_per_read)

        return [self.SensorData(name='temp', val=_st1, units='F'),
                self.SensorData(name='bar_pres', val=_st2, units='hPa'),
                self.SensorData(name='altitude', val=_st3, units='ft')
                ]


class RogyINA260(RogySensorI2C):

    def __init__(self, scl_pin=22, sda_pin=21, history=5, samples_per_read=1):
        super().__init__(scl_pin=scl_pin, sda_pin=sda_pin, device='INA260', history=history,
                         samples_per_read=samples_per_read)

        # self.rs_i2c_device = MP_BMP280_I2C(i2c=self.rs_i2c, address=118)
        self._read_i2c = self._read_ina260

        try:
            import board
            import busio
            import adafruit_ina260
            self._sensor = adafruit_ina260.INA260(busio.I2C(board.SCL, board.SDA))

        except ImportError as err:
            print('Cant start i2c sensor:', err)
            self.active = False
            return

        self.active = True

    def _read_ina260(self):
        _st1 = 0
        _st2 = 0
        _st3 = 0
        for i in range(0, self.samples_per_read):
            _st1 += self._sensor.current
            _st2 += self._sensor.voltage
            _st3 += self._sensor.power
            time.sleep(.1)

        # Convert to Amps, V, Watts
        _st1 = '{:.2f}'.format((_st1 / self.samples_per_read) / 1000)
        _st2 = '{:.2f}'.format(_st2 / self.samples_per_read)
        _st3 = '{:.2f}'.format((_st3 / self.samples_per_read) / 1000)

        return [self.SensorData(name='current', val=_st1, units='A'),
                self.SensorData(name='voltage', val=_st2, units='V'),
                self.SensorData(name='power', val=_st3, units='W')
                ]


def main():

    # vbat = machine.ADC(36)
    # vbat.atten(vbat.ATTN_11DB)
    # VBAT = Pin 35

    bmp280 = RogyBMP280()
    ina260 = RogyINA260()

    while True:
        print(bmp280.read(pretty_format=True))
        print(ina260.read(pretty_format=True))
        time.sleep(1)


if __name__ == '__main__':
    main()



