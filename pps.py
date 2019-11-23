#!/usr/bin/python -u
#vim: tabstop=8 expandtab shiftwidth=8 softtabstop=8

#
# Python interface to PPS.
#
# This code has been developed and tested using Raspberry PI 3. Your mileage may vary.
#
# Copyright (C) 2019 Fio Cattaneo <fio@cattaneo.us>, All Rights Reserved.
#
# This code is released under dual GPL Version 2 / FreeBSD license, at your option.
#

#
# TODO: actually use ppsapi via ctypes
#

#import time
#import sys
#import os

#
# FIXME: the code currently uses python floating point to store timespec values.
# most (all?) python implementations use IEEE754 double precision, which gives plenty of precision to
# handle unix epoch times with nanosecond precision. printing a unix epoch as timespec "{0.09f}" format
# yields the desired precision when using IEEE754 double precision.
# code rewrite will be technically needed if this code is ever used with single precision arithmetic,
# although it's possible that given the jitter of the WWVB signal, there would be no practical consequences.
#

class pps:
        def __init__(self, pps_devname, pps_edge):
                self.pps_devname = pps_devname
                self.pps_edge = pps_edge
                print "pps.__init__: " + self.pps_devname + ", " + self.pps_edge
                self.pps_edge = pps_edge
        def time_pps_fetch(self):
                pps_dev = "/sys/devices/virtual/pps/" + self.pps_devname + "/" + self.pps_edge
                pps_fd = open(pps_dev, "r")
                pps_data = pps_fd.readline().replace('\n', '')
                if pps_data == '':
                        return [ 0.0, 0 ]
                pps_data_a = pps_data.split('#')
                pps_stamp = [ 0.0, 0 ]
                pps_stamp[0] = float(pps_data_a[0])
                pps_stamp[1] = int(pps_data_a[1])
                return pps_stamp
