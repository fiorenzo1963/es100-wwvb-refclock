#!/usr/bin/python -u

import time
import es100_wwvb

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
        wwvb = es100_wwvb()
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
