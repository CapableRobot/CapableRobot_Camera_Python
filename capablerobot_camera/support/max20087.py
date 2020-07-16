# The MIT License (MIT)
#
# Copyright (c) 2019 Chris Osterwood for Capable Robot Components
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import time

from adafruit_bus_device.i2c_device import I2CDevice
from ..util import *

_REG_MASK       = 0x00
_REG_CONFIG     = 0x01
_REG_ID         = 0x02
_REG_STAT1      = 0x03
_REG_STAT2      = 0x04
_REG_STAT3      = 0x05
_REG_ADC1       = 0x06
_REG_ADC2       = 0x07
_REG_ADC3       = 0x08
_REG_ADC4       = 0x09

_ADC_MODE_CURRENT = 0x00
_ADC_MODE_VOLTAGE = 0x01
_ADC_MODE_RAILS   = 0x02

_ADC_DELAY = 0.01

class MAX20087:

    def __init__(self, i2c_bus, addr=0x2c):
        self.i2c_device = I2CDevice(i2c_bus, addr)
        
        ## Querry current output state to allow setup of the _config structure
        outputs = self._read_register(_REG_CONFIG)
        
        self._config = dict(
            adc_mode = _ADC_MODE_CURRENT,
            enc = True, ## This is not the POR value, but we do want continuous ADC reading
            clr = True,
            outputs = [get_bit(outputs, idx) for idx in [0,1,2,3]]
        )

    def _write_register(self, reg, value):
        seq = bytearray([reg, value & 0xFF])
        with self.i2c_device as i2c:
            i2c.write(seq)

    def _read_register(self, reg):
        outbuf = bytearray(1)
        outbuf[0] = reg

        inbuf  = bytearray(1)

        with self.i2c_device as i2c:
            i2c.write_then_readinto(outbuf, inbuf, stop=False)

        return inbuf[0]

    @property
    def id(self):
        data = self._read_register(_REG_ID)
        part = (data & 0b00110000) >> 4
        rev  =  data & 0b00001111
        
        if part == 0:
            part = "MAX20089"
        elif part == 1:
            part = "MAX20088"
        elif part == 2:
            part = "MAX20087"
        elif part == 3:
            part = "MAX20086"

        return part, rev

    @property
    def currents(self):
        if self._config['adc_mode'] != _ADC_MODE_CURRENT:
            self.set_config(adc_mode=_ADC_MODE_CURRENT)
            time.sleep(_ADC_DELAY)

        return [value * 3 for value in self.get_adc_readings()]

    @property
    def voltages(self):
        if self._config['adc_mode'] != _ADC_MODE_VOLTAGE:
            self.set_config(adc_mode=_ADC_MODE_VOLTAGE)
            time.sleep(_ADC_DELAY)

        return [float(value) * 0.070 for value in self.get_adc_readings()]

    @property
    def rails(self):
        if self._config['adc_mode'] != _ADC_MODE_RAILS:
            self.set_config(adc_mode=_ADC_MODE_RAILS)
            time.sleep(_ADC_DELAY)

        out   = self.get_adc_readings()
        vin   = float(out[0]) * 0.070
        vdd   = float(out[1]) * 0.025
        viset = float(out[2]) * 0.005

        return [vin, vdd, viset]

    def get_adc_readings(self):
        adc1 = self._read_register(_REG_ADC1)
        adc2 = self._read_register(_REG_ADC2)
        adc3 = self._read_register(_REG_ADC3)
        adc4 = self._read_register(_REG_ADC4)

        return adc1, adc2, adc3, adc4


    @property
    def errors(self):
        out = []

        data = self._read_register(_REG_STAT1)

        ## Skip bit 4 as ADC convertsion complete is not an error
        if data & 0b00100000:
            out.append('ISET')
        if data & 0b00001000:
            out.append("OVIN")
        if data & 0b00000100:
            out.append("UVIN")
        if data & 0b00000010:
            out.append("OVDD")
        if data & 0b00000001:
            out.append("UVDD")

        data = self._read_register(_REG_STAT2)

        if data & 0b10000000:
            out.append("TS2")
        if data & 0b01000000:
            out.append("OC2")
        if data & 0b00100000:
            out.append("OV2")
        if data & 0b00010000 and self._config['outputs'][1]:
            out.append("UV2")

        if data & 0b00001000:
            out.append("TS1")
        if data & 0b00000100:
            out.append("OC1")
        if data & 0b00000010:
            out.append("OV1")
        if data & 0b00000001 and self._config['outputs'][0]:
            out.append("UV1")

        data = self._read_register(_REG_STAT3)

        if data & 0b10000000:
            out.append("TS4")
        if data & 0b01000000:
            out.append("OC4")
        if data & 0b00100000:
            out.append("OV4")
        if data & 0b00010000 and self._config['outputs'][3]:
            out.append("UV4")

        if data & 0b00001000:
            out.append("TS3")
        if data & 0b00000100:
            out.append("OC3")
        if data & 0b00000010:
            out.append("OV3")
        if data & 0b00000001 and self._config['outputs'][2]:
            out.append("UV3")

        return out
        
    
    @property
    def config(self):
        return self._read_register(_REG_CONFIG)

    @property
    def outputs(self):
        return self._config['outputs']

    def set_config(self, **params):
        
        for key in ['adc_mode', 'enc', 'clr', 'outputs']:
            if key in params:
                self._config[key] = params[key]

        value = self._config['adc_mode'] << 6 | \
                self._config['enc'] << 5 | \
                self._config['clr'] << 4 | \
                self._config['outputs'][3] << 3 | \
                self._config['outputs'][2] << 2 | \
                self._config['outputs'][1] << 1 | \
                self._config['outputs'][0]

        self._write_register(_REG_CONFIG, value)


    