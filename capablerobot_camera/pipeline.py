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

from .deserializers import MAX9286
from .serializers import MAX967xx
from .imagers import OnSemi

class Pipeline:

    def __init__(self, i2c_bus, 
        deserializer='max9286', 
        serializer='max967xx', 
        imager='onsemi',
        channels=[0]):

        self.i2c_bus = i2c_bus
        self.channels = channels

        if deserializer == 'max9286':
            self.deser = MAX9286(i2c_bus)
        else:
            logging.error("Invalid deserializer {} specified".format(deserializer))

        if serializer == 'max967xx':
            self.ser = MAX967xx(i2c_bus)
        else:
            logging.error("Invalid serializer {} specified".format(serializer))

        if imager == 'onsemi':
            self.imager = OnSemi(i2c_bus)
        else:
            logging.error("Invalid imager {} specified".format(imager))

        self.trigger = False

    def reset_imager(self):
        self.ser.reset_imager()
        time.sleep(0.1)

        self.imager.config(stream=False, trigger=False)
        time.sleep(0.1)

        stream = not self.trigger
        self.imager.config(stream=stream, trigger=self.trigger)
        time.sleep(0.1)

    def configure_deser(self, poc_off_time=0.5):

        if "UVIN" in self.deser.power.errors:
            logging.error("12v input rail is off")
            sys.exit(0)

        ## Turn off PoC outputs
        logging.info("Turning off PoC for {:.1f} seconds".format(poc_off_time))
        self.deser.poc(outputs=[False,False,False,False], delay=poc_off_time)

        ## Use the GPIO to reset the chip so that BWS and HIM strapping pins can be configured
        # self.deser.reset(bws=False, him=False)

        ## Tell the power switch IC to only deliver power to PoC circuits specified
        poc_enable = [False, False, False, False]
        for channel in self.channels:
            poc_enable[channel] = True
        print(poc_enable)
        self.deser.poc(outputs=poc_enable)

        for channel in self.channels:
            self.deser.enable_link(link=channel)

        self.deser.configure_csi(lanes=2, pixel_type="RAW11/12")
        self.deser.configure_trig(ext_trig=self.trigger)
        self.deser.configure_error_gpio(auto_error_reset=True)

    def init(self, poc_off_time=0.5):
        self.configure_deser(poc_off_time=poc_off_time)

        self.ser.wakeup()
        self.ser.reset()

        logging.info("SER ID {}".format(self.ser.id))
        time.sleep(0.25)

        self.reset_imager()

        logging.info("Imager ID {}".format(self.imager.id))
        logging.info("Imager Window {}".format(self.imager.roi()))

        logging.info("SER EN {}".format(self.ser.control(serialize=True, configure=True)))
        logging.info("STATUS {}".format(self.deser.errors()))

        t0 = time.time()

        while True:
            if self.deser.locked == True:
                logging.info("Locked after {:.2f} seconds".format(time.time() - t0))
                break

            time.sleep(0.1)

            if time.time() - t0 > 4:
                logging.error("Timeout waiting for lock failed")
                break

    def poc_info(self):
        print()
        print("PoC mA out ", self.deser.power.currents)
        print("PoC V  out ", *["%.2f" % f for f in self.deser.power.voltages])
        print("PoC V rails", *["%.2f" % v for v in self.deser.power.rails])
        print("PoC errors ", self.deser.power.errors)
        print()

    def loop_status(self):
        start_time = time.time()
        t0 = time.time()

        last_frame_count = 0

        while True:
    
            state = self.deser.locked

            if state == True:
                state_str = "LOCK"
                poc_off_time = 0.5
            elif state == None:
                state_str = "UNKNOWN"
            else:
                state_str = "UNLOCK"

            frame_count = self.imager.frame_count
            t1 = time.time()
            fps = (frame_count-last_frame_count) / (t1 - t0)

            print(int(time.time() - start_time), state_str, "FPS {:.1f} Count {}".format(fps, frame_count))

            t0 = t1
            last_frame_count = frame_count

            if state_str == "UNLOCK":
                poc_off_time = poc_off_time * 1.5
                self.init(poc_off_time)
        
            time.sleep(1)



