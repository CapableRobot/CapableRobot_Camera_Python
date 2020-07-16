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
import logging

from adafruit_bus_device.i2c_device import I2CDevice

from ..util import *

_DELAY_MS = 0.001

_REG_CONTROL      = 0x04
_REG_ENC_CONFIG   = 0x07
_REG_GPIO_CONFIG  = 0x0E
_REG_GPIO_SET     = 0x0F
_REG_GPIO_GET     = 0x10
_REG_INPUT_STATUS = 0x15
_REG_ID           = 0x1E
_REG_REV          = 0x1F
_REG_TRIG         = 0x43
_REG_CXTP         = 0x4D
_REG_PRBS_TYPE    = 0x66
_REG_DBL_ALIGN    = 0x67

class MAX967xx:

    def __init__(self, i2c_bus, addr=0x40):
        ## Device will not respond to inital I2C check as it's likely asleep right now
        self.i2c_device = I2CDevice(i2c_bus, addr, probe=False)
        self._name = "MAX967xx"

    def _write_register(self, reg, value, max_attempts=5):
        seq = bytearray([reg, value & 0xFF])

        attempts = 0
        while attempts < max_attempts:
            attempts += 1
            try:
                with self.i2c_device as i2c:
                    i2c.write(seq)
                break
            except OSError:
                if attempts >= max_attempts:
                    raise OSError('Remote I/O error')

    def _read_register(self, reg, max_attempts=5):
        outbuf = bytearray(1)
        outbuf[0] = reg

        inbuf = bytearray(1)
        attempts = 0

        while attempts < max_attempts:
            attempts += 1
            try:
                with self.i2c_device as i2c:
                    i2c.write_then_readinto(outbuf, inbuf, stop=False)
                break
            except OSError:
                if attempts >= max_attempts:
                    return None 

        return inbuf[0]

    @property
    def name(self):
        return self._name
        
    @property
    def id(self):
        reg_val = self._read_register(_REG_ID)
        rev = self._read_register(_REG_REV)

        if reg_val == 0b01000001:
            name = "MAX96705"
        elif reg_val == 0b01000011:
            name = "MAX96711"
        else:
            raise ValueError("Device '%s' / '%s' is not recognized" % (reg_val, rev) )

        self._name = name

        
        return name, rev

    def wakeup(self):

        ## Device is likely asleep.  No expectation that this call will suceed.
        try:
            reg = self._read_register(_REG_CONTROL)
        except OSError:
            pass

        time.sleep(_DELAY_MS*5)

        try:
            ## Turn on configure channel, disable sleep, and setup I2C interface
            self._write_register(_REG_CONTROL, 0b01000011)
            # self._write_register(_REG_CONTROL, 0b11000011)
        except OSError:
            logging.warning("No ACK after disable sleep in control register")
            pass

        time.sleep(_DELAY_MS*5)

    def configure_i2c(self, auto_clink=False, double_align_mode=0b111):

        ## auto_clink : This allows system to automatically
        ##              enable configuration link when SEREN = 1 and PCLKDET = 0

        value = 0b11 << 6 | \
                auto_clink << 5 | \
                (double_align_mode & 0b111)

        logging.info("REG DBL_ALIGN {}".format(hex(value)))
        self._write_register(_REG_DBL_ALIGN, value)
        time.sleep(_DELAY_MS*5)

    def configure_encoding(self, **kwargs):

        current = self._read_register(_REG_ENC_CONFIG)
        desired = current
        result = {}

        names = [
            'crc_type', 
            None, 
            'hsvs_encoding', 
            None, 
            'edge_select', 
            'bus_width',
            'high_bandwidth',
            'double_mode'
        ]
        
        for i,name in enumerate(names):
            if name is not None:
                result[name] = get_bit(current,i)

        for name, value in kwargs.items():
            if name not in names:
                raise ValueError("configure_encoding doesn't understand key '{}'".format(name))

            idx = names.index(name)

            if value:
                desired = set_bit(desired, idx)
                result[name] = True
            else:
                desired = clear_bit(desired, idx)
                result[name] = False

        if desired != current:
            logging.debug("REG ENC_CONFIG change {} -> {}".format(hex(current), hex(desired)))
            self._write_register(_REG_ENC_CONFIG, desired)

        return result

    def reset(self):
        logging.info("Reseting {}".format(self._name))

        wake = self.wakeup()
        logging.debug("{} WAKE {}".format(self._name, wake))

        ctrl = self.control(serialize=False, configure=True)
        logging.debug("{} CTRL {}".format(self._name, ctrl))

        gpio = self.setup_gpio()
        logging.debug("{} GPIO {}".format(self._name, gpio))

    def reset_imager(self):
        ## Send reset pulse to imager
        ## with no time delay added, pulse is ~4ms
        self.gpio(1, False)
        time.sleep(0.01)
        self.gpio(1, True)
        time.sleep(0.01)


    def control(self, serialize=None, configure=None, prbs_test=None, \
                sleep=None, reverse_control=None, foward_control=None):

        current = self._read_register(_REG_CONTROL)
        desired = current

        if serialize != None:
            if serialize:
                desired = set_bit(desired, 7)
            else:
                desired = clear_bit(desired, 7)

        if configure != None:
            if configure:
                desired = set_bit(desired, 6)
            else:
                desired = clear_bit(desired, 6)

        if prbs_test != None:
            if prbs_test:
                desired = set_bit(desired, 5)
            else:
                desired = clear_bit(desired, 5)

        if sleep != None:
            if sleep:
                desired = set_bit(desired, 4)
            else:
                desired = clear_bit(desired, 4)

        if reverse_control != None:
            if reverse_control:
                desired = set_bit(desired, 1)
            else:
                desired = clear_bit(desired, 1)

        if foward_control != None:
            if foward_control:
                desired = set_bit(desired, 0)
            else:
                desired = clear_bit(desired, 0)

        if desired != current:
            self._write_register(_REG_CONTROL, desired)
            time.sleep(_DELAY_MS*5)
            return desired

    def get_control_status(self):
        value = self._read_register(_REG_CONTROL)
        
        if value is None:
            return {}

        names = ['foward_control','reverse_control','interface_type0','interface_type1','sleep','prbs_test','configure','serialize']
        result = {}

        for i,name in enumerate(names):
            result[name] = get_bit(value,i)

        return result

    def setup_gpio(self, gpios=[1]):
        current = self._read_register(_REG_GPIO_CONFIG)
        desired = current

        for idx in gpios:
            desired = set_bit(desired, idx)

        ## Default state (in hardware) is for GPIO1 to be enabled.  
        ## If it's not in the list above, we need to explicitly disable it.
        if 1 not in gpios:
            desired = clear_bit(desired, 0)

        if desired != current:
            logging.debug("REG GPIO_CONFIG change {} -> {}".format(hex(current), hex(desired)))
            self._write_register(_REG_GPIO_CONFIG, desired)
        
        return desired

    def gpio(self, pin, state=None):
        if state == None:
            value = self._read_register(_REG_GPIO_GET)
            return (value >> pin) & 0x01

        current = self._read_register(_REG_GPIO_SET)
        
        if state:
            desired = set_bit(current, pin)
        else:
            desired = clear_bit(current, pin)

        if desired != current:
            self._write_register(_REG_GPIO_SET, desired)
            return desired

        return current

    def status(self):
        value = self._read_register(_REG_INPUT_STATUS)

        if value is None:
            return {}

        names = ['pclk_detect','output_enabled',None,None,None,'lccen',None,'cxtp']
        result = {}

        for i,name in enumerate(names):
            if name is not None:
                result[name] = get_bit(value,i)

        return result

    def configure_cxtp(self, him=False, coax=True, vsync_invert=False, hsync_invert=False, de_invert=False):

        value = him  << 7 | \
                coax << 6 | \
                vsync_invert << 3 | \
                hsync_invert << 2 | \
                de_invert << 1

        logging.info("REG CXTP {}".format(hex(value)))
        self._write_register(_REG_CXTP, value)

    def enable_him(self):
        self.configure_cxtp(him=True)

    def configure_prbs_type(self, pbrs_type=True, rev_fast=False, de_enable=False, disable_remote_wakeup=False, coax_select=True):

        value = 1 << 6 | \
                pbrs_type << 5 | \
                rev_fast << 4 | \
                de_enable << 3 | \
                disable_remote_wakeup << 2 | \
                coax_select

        logging.info("REG PRBS_TYPE {}".format(hex(value)))
        self._write_register(_REG_PRBS_TYPE, value)

    def enable_reverse_channel_fast_mode(self):
        self.configure_prbs_type(rev_fast=True)
