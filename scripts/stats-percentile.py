#!/usr/bin/python

import os
import sys
import time

#
# XXX: time_constant is a bad name for it, as TAU is not fixed for WWVB rx,
# although it can be very closely approximated to 1 sample = 1 minute when RX is okay
#

if len(sys.argv) != 2:
        print sys.argv[0] + ": usage: " + sys.argv[0] + " logfile"
        exit(1)
logfile = sys.argv[1]

print "         logfile = " + logfile

def read_data():
        raw_data = [ ]
        fd = open(logfile, "r")
        for line in fd.readlines():
                line = line.replace('\n', '')
                prefix = "RX_WWVB_CLOCKSTATS,"
                # print line[0:len(prefix)]
                if line[0:len(prefix)] != prefix:
                        continue
                # print line
                tokens = line.split(',')
                if len(tokens[11]) != 0:
                        # print tokens[5] + " --> " + tokens[11]
                        raw_data.append([ float(tokens[5]), float(tokens[11]) ])
                else:
                        # print tokens[5]
                        raw_data.append([ float(tokens[5]), None ])
        return raw_data

all_data = read_data()
valid_data = [ ]
for e in all_data:
        if e[1] is not None:
                valid_data.append(e)

def get_samples(data, i_start, i_end):
        samples = [ ]
        while i_start <= i_end:
                samples.append(data[i_start][1])
                i_start = i_start + 1
        return samples

def do_average(s):
        return sum(s, 0.0) / len(s)

def do_median(s):
        s1 = s
        s1.sort()
        if len(s1) == 1:
                return s1[0]
        if len(s1) % 2 == 0:
                return avg(s1[len(s1) / 2], s1[len(s1) / 2 + 1])
        else:
                return s1[len(s1) / 2]

def make_timespec_s(timestamp):
        return "{0:09.09f}".format(timestamp)

def make_timefrac_s(timestamp):
        if timestamp < 0:
            return "{0:0.09f}".format(timestamp)
        else:
            return "+{0:0.09f}".format(timestamp)

#
# call api to calculate this correctly
# {"results":{"sunrise":"2019-11-29T14:03:00+00:00","sunset":"2019-11-29T23:34:36+00:00","solar_noon":"2019-11-29T18:48:48+00:00","day_length":34296,"civil_twilight_begin":"2019-11-29T13:32:54+00:00","civil_twilight_end":"2019-11-30T00:04:43+00:00","nautical_twilight_begin":"2019-11-29T12:59:04+00:00","nautical_twilight_end":"2019-11-30T00:38:32+00:00","astronomical_twilight_begin":"2019-11-29T12:26:13+00:00","astronomical_twilight_end":"2019-11-30T01:11:24+00:00"},"status":"OK"}
#
def is_daytime(timestamp):
        sunrise_secs = 14 * 3600 + 3 * 60
        sunset_secs = 23 * 3600 + 34 * 60
        day_secs = int(timestamp) % 86400
        if day_secs >= sunrise_secs and day_secs <= sunset_secs:
                return True
        else:
                return False

samples = get_samples(valid_data, 0, len(valid_data) - 1)
samples.sort()

print ""
print "              logfile = " + logfile
print "total samples         = " + str(len(all_data))
print "total valid samples   = " + str(len(valid_data))
print "      valid samples   = " + "{0:.2f}%".format(100.0 * len(valid_data) / len(all_data))
print ""

def get_percentile(samples, pct):
        index = (len(samples) * pct) / 100.0
        index = int(index)
        #print "get_percentile: len(samples) = " + str(len(samples)) + ", index=" + str(index)
        if index > len(samples) - 1:
                index = len(samples) - 1
        return samples[index]

print "P00            = " + make_timefrac_s(get_percentile(samples, 0.0))
print "P01            = " + make_timefrac_s(get_percentile(samples, 1.0))
print "P05            = " + make_timefrac_s(get_percentile(samples, 5.0))
print "P10            = " + make_timefrac_s(get_percentile(samples, 10.0))
print "P20            = " + make_timefrac_s(get_percentile(samples, 20.0))
print "P25            = " + make_timefrac_s(get_percentile(samples, 25.0))
print "P30            = " + make_timefrac_s(get_percentile(samples, 30.0))
print "P40            = " + make_timefrac_s(get_percentile(samples, 40.0))
print "P50            = " + make_timefrac_s(get_percentile(samples, 50.0))
print "P60            = " + make_timefrac_s(get_percentile(samples, 60.0))
print "P70            = " + make_timefrac_s(get_percentile(samples, 70.0))
print "P75            = " + make_timefrac_s(get_percentile(samples, 75.0))
print "P80            = " + make_timefrac_s(get_percentile(samples, 80.0))
print "P90            = " + make_timefrac_s(get_percentile(samples, 90.0))
print "P95            = " + make_timefrac_s(get_percentile(samples, 95.0))
print "P99            = " + make_timefrac_s(get_percentile(samples, 99.0))
print "P100           = " + make_timefrac_s(get_percentile(samples, 100.0))
