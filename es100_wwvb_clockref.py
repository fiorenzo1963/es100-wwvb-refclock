#!/usr/bin/python -u

#
# Simple python code to test ES100 WWVB receiver.
# Ths code tries to update NTP SHM segment, if any.
#
# This code has been developed and tested using Raspberry PI 3. Your mileage may vary.
#
# Copyright (C) 2019 Fio Cattaneo <fio@cattaneo.us>, All Rights Reserved.
#
# This code is released under dual GPL Version 2 / FreeBSD license, at your option.
#
# The test code runs forever and keeps receiving data from WWVB.
# It starts with user supplied antenna configuration (1, 2), then it keeps using the same
# antenna for as long as RX is successful. Upon RX timeout or RX error it switches to the
# other antenna. The receive timestamp is taken with PPS api when GPIO_IRQ goes low, thus
# its accuracy does not depend on the I2C bus's baud rate.
#
# TODO: general code cleanup
#

import os
import sys
import time
from es100_wwvb import es100_wwvb

def main():
        #
        # do one time init before entering RX loop
        #
        wwvb = es100_wwvb(allow_tracking_mode = True)
        #
        # RX loop
        #
        while True:
                #
                # get wwvb timestamp
                #
                rx_ret = wwvb.get_timestamp_from_wwvb_device()

main()
