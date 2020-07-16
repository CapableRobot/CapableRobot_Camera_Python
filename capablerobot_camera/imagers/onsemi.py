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
import math
import logging

from adafruit_bus_device.i2c_device import I2CDevice

from ..util import *


_REG_GAIN_GREEN1 = 0x3056
_REG_GAIN_BLUE   = 0x3058
_REG_GAIN_RED    = 0x305A
_REG_GAIN_GREEN2 = 0x305A
_REG_GAIN_GLOBAL = 0x305E

_REG_TEST_PATTERN_MODE   = 0x3070
_REG_TEST_PATTERN_RED    = 0x3072
_REG_TEST_PATTERN_GREEN1 = 0x3074
_REG_TEST_PATTERN_BLUE   = 0x3076
_REG_TEST_PATTERN_GREEN2 = 0x3078

_REG_DIGITAL_TEST = 0x30B0

class OnSemi:

    def __init__(self, i2c_bus, addr=0x10):
        self.i2c_device = I2CDevice(i2c_bus, addr, probe=False)

        self._name = None
        self._temp_cal = None
        self._temp_slope = None
        self._temp_zero  = None

    def _read_register(self, reg, length=1, max_attempts=5):
        outbuf = bytearray(2)
        outbuf[0] = (reg >> 8) & 0xFF 
        outbuf[1] = reg & 0xFF

        inbuf  = bytearray(length)
        
        attempts = 0
        while attempts < max_attempts:
            attempts += 1
            try:
                with self.i2c_device as i2c:
                    i2c.write(outbuf)
                    i2c.readinto(inbuf)
                break
            except OSError:
                if attempts >= max_attempts:
                    return None
        return inbuf


    ## Values must be a list
    def _write_register(self, reg, values):
        seq = bytearray(2)
        seq[0] = (reg >> 8) & 0xFF 
        seq[1] = reg & 0xFF

        try:
            with self.i2c_device as i2c:
                i2c.write(seq + bytearray(values))
        except OSError:
            return None 

        return True

    @property
    def id(self):
        chip     = bytearry_to_int(self._read_register(0x3000, length=2))
        revision = bytearry_to_int(self._read_register(0x300E, length=1))

        if chip == 0x2406:
            chip = 'AR0134'
        elif chip == 0x0051:
            chip = 'AR0141'
        elif chip == 0x0754:
            chip = 'AR0135'

        return chip, revision

    @property
    def name(self):
        if self._name is None:
            self._name = list(self.id)[0]

        return self._name
    

    @property
    def temperature(self):
        if self._temp_slope == None:
            ## Get calibration information and store necessary values
            c55 = bytearry_to_int(self._read_register(0x30C8, length=2)) & 0x1FF
            c70 = bytearry_to_int(self._read_register(0x30C6, length=2)) & 0x1FF
            self._temp_cal = (c55, c70)

            self._temp_slope = 15.0 / max(1e-19,float(c70-c55))
            self._temp_zero  = 55.0 - self._temp_slope * c55

            logging.debug("Temperature calibration (55C, 70C) {}".format(self._temp_cal))
            logging.debug("Temperature slope {} zero {}".format(self._temp_slope, self._temp_zero))

        ## Turn the temperature sensor on by settings bit 0 
        ## Start a temperature reading by setting bit 4 to 1
        # value = self._read_register(0x30B4)[0]
        value = 0
        value |= (1<<4)
        value |= (1)
        status = self._write_register(0x30B4, [0, value])

        if status == None:
            return None

        time.sleep(0.01)
        value = self._read_register(0x30B2, length=2)
        
        if value == None:
            return None
        
        value = bytearry_to_int(value) & 0x1FF

        if value == 0:
            return None

        return float(value) * self._temp_slope + self._temp_zero

    @property
    def frame_count(self):
        value = self._read_register(0x303A, length=2)
        
        if value == None:
            return -1

        return bytearry_to_int(value)

    def dump_registers(self, filepath=None):
        addr = 0x3000
        data = {}
        handle = None
        max_attempts = 5

        if filepath != None:
            handle = open(filepath, 'w')

        while addr <= 0x31FC:
            value = None
            attempt_count = 0
            while value == None and attempt_count < max_attempts:
                time.sleep(0.02)
                value = self._read_register(addr, length=2)
                attempt_count += 1
            if value == None:
                value = 'ReadFailed'
            else:
                value = bytearry_to_int(value)
                
            logging.debug("{} : {}".format(hex(addr), value))
            data[hex(addr)] = value

            if handle != None:
                handle.write("{} {}\r\n".format(hex(addr), value))

            addr += 2

        handle.close()

        return data


    def roi(self, size=None):
        if size == None:
            size = bytearry_to_int(self._read_register(0x3002, length=8))
            x1 = (size      ) & 0xFFFF
            y1 = (size >> 16) & 0xFFFF
            x0 = (size >> 32) & 0xFFFF
            y0 = (size >> 48) & 0xFFFF

            size = (x1-x0, y1-y0)

        ## TODO : support setting of ROI

        return size

    def config(self, stream=False, trigger=False, parallel=True):

        buf = self._read_register(0x301A, length=2)
        value = bytearry_to_int(buf)
        logging.debug("0x301A current {} {}".format(bytearry_to_int(buf), buf))

        if trigger and stream:
            logging.warn("Trigger and Streaming both enabled. Pick one.")

        if trigger:
            buf[0] = set_bit(buf[0], 0)   # Enable GPI input buffers
            buf[0] = set_bit(buf[0], 3)   # Force PLL to always be on
        else:
            buf[0] = clear_bit(buf[0], 0) # Disable GPI input buffers
            buf[0] = clear_bit(buf[0], 3) # Allows PLLs to be powered down in standby

        if stream:
            buf[1] = set_bit(buf[1], 2)
        else:
            buf[1] = clear_bit(buf[1], 2)

        if parallel:
            buf[1] = set_bit(buf[1], 7)     # Enable parallel data pins
            buf[1] = set_bit(buf[1], 6)     # Enable parallel data pins
            buf[0] = set_bit(buf[0], 4)     # Disable the serial interface
        else:
            buf[1] = clear_bit(buf[1], 7)   # Disable parallel data pins
            buf[1] = clear_bit(buf[1], 6)   # Disable parallel data pins
            buf[0] = clear_bit(buf[0], 4)   # Enable the serial interface

        ## Configuration has changed, send to the device
        if bytearry_to_int(buf) != value:
            logging.debug("0x301A update {} {}".format(bytearry_to_int(buf), buf))
            self._write_register(0x301A, buf)
    
        return buf

    def config_pll(self, disable=False):

        buf = self._read_register(_REG_DIGITAL_TEST, length=2)
        logging.debug("0x30B0 current {} {}".format(bytearry_to_int(buf), buf))

        if disable:
            buf[1] = set_bit(buf[1], 6)
            logging.debug("0x30B0 update {} {}".format(bytearry_to_int(buf), buf))
            self._write_register(_REG_DIGITAL_TEST, buf)

        return buf

    def test_pattern(self, mode='normal', color=None):
        modes = dict(
            normal=0,
            color=1,
            colorbar=2,
            gray=3,
            walking=256
        )

        if mode not in modes.keys():
            raise ValueError("test_pattern mode '{}' not understood".format(mode))

        if color is not None:
            for i, addr in enumerate([_REG_TEST_PATTERN_RED, _REG_TEST_PATTERN_GREEN1, _REG_TEST_PATTERN_BLUE, _REG_TEST_PATTERN_GREEN2]):
                self._write_register(addr, int_to_bytearray(color[i]))

        self._write_register(_REG_TEST_PATTERN_MODE, int_to_bytearray(modes[mode]))

    def column_correction(self):
        self.config(stream=False)
        time.sleep(0.01)

        ## Set all gains to 1
        self._write_register(_REG_GAIN_GLOBAL, [0, 1])

        ## Enable Column Corretion
        self._write_register(0x30D4, [0xE0, 0x07])        

        ## Enable streaming and wait 8 frames
        self.config(stream=True)
        time.sleep(0.3)
        self.config(stream=False)


