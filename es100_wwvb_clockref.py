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
        rx_params = 0
        if len(sys.argv) > 1 and sys.argv[1] == '1': 
                rx_params = es100_wwvb.ES100_CONTROL_START_RX_ANT1
        if len(sys.argv) > 1 and sys.argv[1] == '2': 
                rx_params = es100_wwvb.ES100_CONTROL_START_RX_ANT2
        if rx_params == 0:
                rx_params = es100_wwvb.ES100_CONTROL_START_RX_ANT1
        # stats
        # FIXME: move stats into class
        #
        rx_stats = [ 0 ] * (es100_wwvb.RX_STATUS_WWVB_MAX_STATUS + 1)
        rx_loop = 0
        #
        # do one time init before entering RX loop
        #
        # wwvb = es100_wwvb(allow_tracking_mode = True, force_tracking_mode = True)
        wwvb = es100_wwvb(allow_tracking_mode = True)
        # wwvb = es100_wwvb()
        #
        # RX loop
        #
        while True:
                #
                # get wwvb timestamp
                #
                t0 = time.time()
                rx_ret = wwvb.get_timestamp_from_wwvb_device(rx_params)
                t1 = time.time()
                #
                # emit stats
                #
                print "main: rx_loop = " + str(rx_loop) + " complete, rx_ret = " + str(rx_ret) + ", elapsed = " + str(t1-t0)
                rx_loop = rx_loop + 1
                rx_stats[rx_ret] = rx_stats[rx_ret] + 1
                wwvb.wwvb_emit_rx_stats(rx_loop, rx_stats)
                #
                # select rx antenna for next receive
                #
                rx_params = wwvb.rx_wwvb_select_antenna(rx_ret, rx_params)

main()
