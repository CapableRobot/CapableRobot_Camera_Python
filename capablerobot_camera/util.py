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
import busio

class DirectI2C(busio.I2C):
    ### Replacement / extension of the busio object

    ## This directly tells the underlying i2c object the bus number, instead of
    ## extracting that information from from the board defintion pin mappings.
    ## Currently, the TX2 definition does not define I2C2, but it does exist in linux.
    ## So, direct acess to an I2C Bus is provided here
    
    def __init__(self, scl=None, sda=None, frequency=400000, bus=None):
        self.init(scl, sda, frequency, bus)

    def init(self, scl, sda, frequency, bus=None):
        from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as _I2C
        self._i2c = _I2C(bus, mode=_I2C.MASTER, baudrate=frequency)


### Redefine __init__ method to not have write checks -- the failure takes too long on Linux ###
def I2CDeviceInit(self, i2c, device_address, *, debug=False, probe=True):

    self.i2c = i2c
    self._has_write_read = hasattr(self.i2c, "writeto_then_readfrom")
    self.device_address = device_address

    if probe:
        while not i2c.try_lock():
            pass
            
        # some OS's don't like writing an empty bytesting...
        # Retry by reading a byte
        try:
            result = bytearray(1)
            i2c.readfrom_into(device_address, result)
        except OSError:
            raise ValueError("No I2C device at address: %x" % device_address)

        finally:
            i2c.unlock()

    self._debug = debug

from adafruit_bus_device.i2c_device import I2CDevice
I2CDevice.__init__ = I2CDeviceInit

def bytearry_to_int(b):
    x = 0
    for c in b:
        x <<= 8
        x |= c
    return x
    
def int_to_bytearray(val):
    return [(val >> 8) & 0xFF, val & 0xFF]

def set_bit(value, bit):
    return value | (1<<bit)

def clear_bit(value, bit):
    return value & ~(1<<bit)

def get_bit(value, bit):
    return (value & (1<<bit)) > 0 

def set_bit_to(value, bit, state):
    if state:
        return value | (1<<bit)
        
    return value & ~(1<<bit)
