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

class ppsapi:
        PPS_CAPTUREASSERT = 0x01
        PPS_CAPTURECLEAR  = 0x02
        def __init__(self, pps_devname, pps_mode):
                self.pps_devname = pps_devname
                self.pps_mode = pps_mode
                print "ppsapi.__init__: " + self.pps_devname + ", " + "{0:0x}".format(self.pps_mode)
        def __time_pps_fetch(self, mode):
                if mode == ppsapi.PPS_CAPTUREASSERT:
                        pps_dev = "/sys/devices/virtual/pps/" + self.pps_devname + "/assert"
                else:
                        pps_dev = "/sys/devices/virtual/pps/" + self.pps_devname + "/clear"
                pps_fd = open(pps_dev, "r")
                pps_data = pps_fd.readline().replace('\n', '')
                if pps_data == '':
                        return [ 0.0, 0 ]
                pps_data_a = pps_data.split('#')
                pps_stamp = [ 0.0, 0 ]
                pps_stamp[0] = float(pps_data_a[0])
                pps_stamp[1] = int(pps_data_a[1])
                return pps_stamp
        def time_pps_fetch(self):
                pps_data = { }
                pps_data['mode'] = self.pps_mode
                if self.pps_mode & ppsapi.PPS_CAPTUREASSERT:
                        pps_data['assert'] = self.__time_pps_fetch(ppsapi.PPS_CAPTUREASSERT)
                if self.pps_mode & ppsapi.PPS_CAPTURECLEAR:
                        pps_data['clear'] = self.__time_pps_fetch(ppsapi.PPS_CAPTURECLEAR)
                return pps_data
