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

from ..support import MAX20087
from ..util import *

_DELAY = 0.01

_REG_LINKEN         = 0x00
_REG_GPIO_CFG       = 0x01
_REG_CTRLCNEN       = 0x0A
_REG_SYNC           = 0x0C
_REG_FLT_MON        = 0x0D
_REG_ERROR_GPIO     = 0x0F
_REG_CSI            = 0x12
_REG_CSI_REMAP      = 0x14
_REG_CSI_CHANNEL    = 0x15
_REG_HIM            = 0x1C
_REG_ID             = 0x1E
_REG_REV            = 0x1F
_REG_STAT_HLOCKED   = 0x21
_REG_STAT_LINK      = 0x22
_REG_STAT_PRBS_ERR  = 0x23
_REG_STAT_DET_ERR   = 0x28
_REG_STAT_COR_ERR   = 0x2C
_REG_GPIO           = 0x27
_REG_REV_FAST       = 0x3F



class MAX9286:

    def __init__(self, i2c_bus, addr=0x48):
        """
        Construct a new MAX9286 instance 

        :param i2c_bus busio.I2C: I2C Bus object that communication should take place over
        :param addr int: I2C Address (7 bit) of the MAX9286
        """
        self.i2c_device = I2CDevice(i2c_bus, addr)

        self.power = MAX20087(i2c_bus, addr=0x2c)

        self._links = []
        self._errors = []

        ## Turn off all line fault monitoring (required if resistors are not in place)
        self.configure_line_fault_monitor(links=[])

    def _write_register(self, reg, value):
        seq = bytearray([reg, value & 0xFF])
        
        try:
            with self.i2c_device as i2c:
                i2c.write(seq)
        except OSError:
            self._errors.append("I2C_WRITE")
            return None

        return True
            

    def _read_register(self, reg):
        outbuf = bytearray(1)
        outbuf[0] = reg

        inbuf  = bytearray(1)

        try:
            with self.i2c_device as i2c:
                i2c.write_then_readinto(outbuf, inbuf, stop=False)
        except OSError:
            self._errors.append("I2C_READ")
            return None

        if inbuf == None:
            return None
            
        return inbuf[0]


    @property
    def id(self):
        """
        Get part SKU and silicon revision.

        :return: (string, int) | Part SKU, silicon revision
        """

        if self._read_register(_REG_ID) != 0x40:
            raise ValueError("Device is not a MAX9286")

        rev = self._read_register(_REG_REV)
        return "MAX9286", rev

    def poc(self, outputs, delay=0.5):
        self.power.set_config(outputs=outputs)
        time.sleep(delay)

    def enable_links(self, links=[0], clock=0b111, internal_vsync=True):
        """
        Enables one ore more GMSL links and their forward / reverse control channels.

        If link 0 is enabled and this method is later called with `links=[1]`, then link 0 will become disabled.

        :param links Array[int]: List of active links.  Valid range 0 thru 3.
        :param clock int: CSI clock source.  Value of 0b1xx will enable auto-detection.
        :param internal_vsync bool: When false, VSYNC comes from the camera.  
        True is only valid when FSYNCMODE is not set to 0b11 (e.g. external frame sync supplied by MCU).
        :return: (int, int) | Contents of LINK_ENABLE and CONTROL_CHANNEL registers after writes completed.
        """

        self._links = links
        ## Clock can be set to channel number, or 0b111 means to auto-select
        ## When internal_vsync = True, chip will generate VS when FSYNCMODE not set to 11

        link3 = 3 in links
        link2 = 2 in links
        link1 = 1 in links
        link0 = 0 in links

        linken = clock << 5          | \
                 internal_vsync << 4 | \
                 link3 << 3          | \
                 link2 << 2          | \
                 link1 << 1          | \
                 link0
        self._write_register(_REG_LINKEN, linken)

        ## Upper nibble : forward control channel from serializer (receiving)
        ## Lower nibble : reverse control channel to serializer (sending)
        ctrlen = link3 << 7 | \
                 link2 << 6 | \
                 link1 << 5 | \
                 link0 << 4 | \
                 link3 << 3 | \
                 link2 << 2 | \
                 link1 << 1 | \
                 link0

        self._write_register(_REG_CTRLCNEN, ctrlen)

        return linken, ctrlen

    def enable_link(self, link=0):
        """
        Calls `enable_links(links=[link])`, waits, and checks to see if link is locked.
        Method will log error messages if link does not lock within 50 ms.

        :param link int: Link to enable and check for lock on.
        """
        self.enable_links(self._links + [link])

        ## wait 10ms for link to lock
        time.sleep(_DELAY) 

        ## Check link lock and wait for it
        idx = 0
        while not self.locked:
            time.sleep(_DELAY*5)
            idx += 1

            if idx > 10:
                logging.warn("Link not locked")
                break

    @property
    def locked(self):
        """
        Deserializer GMSL lock state.

        :return: bool | Deserializer GMSL lock state
        """

        value = self._read_register(_REG_GPIO)
        if value == None:
            return None
        return (value >> 7) == True

    def configure_line_fault_monitor(self, links=[0], hsync_track=False, glitch_filter=True):
        """
        Configure line fault monitor settings.

        :param links Array[int]: List of links that are active and should have LMN enabled on.
        :param hsync_track bool: HSYNC / line valid tracking.
        :param glitch_filter bool: HSYNC & VSYNC glitch filtering.  Default when BWS net is low on boot.
        """
        link3 = 3 in links
        link2 = 2 in links
        link1 = 1 in links
        link0 = 0 in links

        value = link3 << 7 | \
                link2 << 6 | \
                link1 << 5 | \
                link0 << 4 | \
                hsync_track   << 2 | \
                glitch_filter << 1 | \
                glitch_filter 

        ## HSYNC / line valid tracking is disabled by default
        ## Bit 1 is for HSYNC. Default of 0 when BWS = open, 1 otherwise.
        ## Bit 0 is for VSYNC. Default of 0 when BWS = open, 1 otherwise.

        self._write_register(_REG_FLT_MON, value)

    def configure_csi(self, lanes=1, double_load=False, pixel_type="YUV422_10"):
        """
        Configure CSI-2 bus parameters.

        :param lanes int: Number of lanes to emit data over.  Valid range is 1 thru 4.
        :param double_load bool: Enable double input mode. 

        Single-mode operation is compatible with all GMSL devices and serializers, yielding one parallel word for each serial word. Double mode serializes two half-width parallel words for each serial word, resulting in a 2x increase in the parallel word rate range (compared to single mode) 
        :param pixel_type str: Imager pixel encoding.  

        There are 12 valid values: 
        - `RGB888 RGB565 RGB666`
        - `YUV422_8 YUV422_10`
        - `RAW8/16 RAW10/20 RAW11/12 RAW14` 
        - `USER24 USER12 USER8`
        """
        
        valid_types = [
            "RGB888", "RGB565", "RGB666",
            "YUV422_8", "YUV422_10",
            "RAW8/16", "RAW10/20", "RAW11/12", "RAW14",
            "USER24", "USER12", "USER8"
        ]

        if not pixel_type in valid_types:
            raise ValueError("Unkown pixel type. Valid choices: [{}]".format(" ".join(valid_types)))

        value = (lanes-1)   << 6 | \
                double_load << 5 | \
                double_load << 4 | \
                valid_types.index(pixel_type) 

        self._write_register(_REG_CSI, value)

    def remap_csi_lanes(self, order=[0,1,2,3]):
        """
        Remap physical and logical CSI-2 lanes.  
        This functionality allows PCB layout to be optimized by swapping lanes.

        :param order [int,int,int,int]: Logical order to apply to the physical lanes.
        """

        if len(order) != 4:
            raise ValueError("Expect mapping for all 4 CSI lanes, found {}.".format(len(order)))

        for lane in [0,1,2,3]:
            if lane not in order:
                raise ValueError("No assignment for lane {}".format(lane))

        value = order[3] << 6 | \
                order[2] << 4 | \
                order[1] << 2 | \
                order[0]

        self._write_register(_REG_CSI_REMAP, value)

    def enable_csi(self, virtual_channel=0, enable=True):
        """
        Turn on the CSI-2 output bus.

        :param virtual_channel int: Virtual channel to emit data on. Valid options include 'auto' and 0 thru 3.
        :param enable bool: Flag to enable / disable the CSI-2 output bus.
        """
        vc_type = 0

        if virtual_channel == 'auto':
            vc_type = 1
            virtual_channel = 0

        value = virtual_channel << 5 | \
                vc_type << 4 | \
                enable  << 3 | \
                0b011 ## reserved value in datasheet

        ## Bits 0, 1, 2, & 7 are reserved
        ## When vc_type == 1, virtual channel is set according to the link number
        ## Otherwise, channel is set via bits 5 & 6

        self._write_register(_REG_CSI_CHANNEL, value)

    def configure_trig(self, ext_trig=True):
        if ext_trig:
            logging.debug("Enabling external trigger")
            self._write_register(_REG_GPIO_CFG, 226)
        else:
            logging.debug("Disabling external trigger")
            self._write_register(_REG_GPIO_CFG, 34)

    def configure_error_gpio(self, auto_error_reset=False, gpio0=True, gpio1=True):      
        current = self._read_register(_REG_ERROR_GPIO)
        desired = current

        if auto_error_reset:
            desired = set_bit(desired, 5)
        else:
            desired = clear_bit(desired, 5)

        if gpio1:
            desired = set_bit(desired, 1)
        else:
            desired = clear_bit(desired, 1)

        if gpio0:
            desired = set_bit(desired, 0)
        else:
            desired = clear_bit(desired, 0)

        if desired != current:
            logging.debug("REG ERROR_GPIO change {} -> {}".format(hex(current), hex(desired)))
            self._write_register(_REG_ERROR_GPIO, desired)

    def enable_reverse_channel_fast_mode(self):
        current = self._read_register(_REG_REV_FAST)
        desired = set_bit(current, 7)

        if desired != current:
            logging.debug("REG REV_FAST {}".format(hex(value)))
            self._write_register(_REG_REV_FAST, desired)

    def disable_reverse_channel_fast_mode(self):
        current = self._read_register(_REG_REV_FAST)
        desired = clear_bit(current, 7)

        if desired != current:
            logging.debug("REG REV_FAST {}".format(hex(value)))
            self._write_register(_REG_REV_FAST, desired)      

    def errors(self, prbs=False):
        """
        Fetch any errors within various MAX9286 error registers.

        :param prbs bool: Query for PRBS errors.  Only valid to do during PRBS testing.
        :return: Array[string] | Short, textual representation of various error states.   
        """
        out = []

        value = self._read_register(_REG_STAT_HLOCKED)
        if value != None:
            for link in self._links:
                if value != None and (value >> link) & 0x01:
                    out.append("LINK{}_LINE_TRACKING_UNLOCK".format(link))

        value = self._read_register(_REG_STAT_LINK)
        if value != None:
            for link in self._links:
                if (value >> (link + 4)) & 0x01:
                    out.append("LINK{}_LINE_BUFFER_OVERLFOW".format(link))

                if (value >>link) & 0x01:
                    out.append("LINK{}_LINE_ERROR".format(link))

        if prbs:
            for link in self._links:
                value = self._read_register(_REG_STAT_PRBS_ERR + link)
                if value != None and value > 0:
                    out.append("LINK{}_PRBS_ERROR {}".format(link, value))

        for link in self._links:
            value = self._read_register(_REG_STAT_DET_ERR + link)
            if value != None and value > 0:
                out.append("LINK{}_DETECTED_ERRORS {}".format(link, value))

            value = self._read_register(_REG_STAT_COR_ERR + link)
            if value != None and value > 0:
                out.append("LINK{}_CORRECTED_ERRORS {}".format(link, value))

        for error in self._errors:
            out.append(error)
        self._errors = []
        
        return out

    def enable_him(self):
        value = self._read_register(_REG_HIM)

        link3 = 3 in self._links
        link2 = 2 in self._links
        link1 = 1 in self._links
        link0 = 0 in self._links

        if link0:
            value = set_bit(value, 4)

        if link1:
            value = set_bit(value, 5)

        if link2:
            value = set_bit(value, 6)

        if link3:
            value = set_bit(value, 7)

        logging.info("REG HIM {}".format(hex(value)))
        self._write_register(_REG_HIM, value)

    


