#!/usr/bin/python

import os
import sys
import time

if len(sys.argv) != 4:
        print sys.argv[0] + ": usage: " + sys.argv[0] + " logfile time_constant median_threshold"
        exit(1)
logfile = sys.argv[1]
TIME_CONSTANT = int(sys.argv[2])
median_threshold = float(sys.argv[3])

print "         logfile = " + logfile
print "   TIME_CONSTANT = " + str(TIME_CONSTANT)
print "median_threshold = " + str(median_threshold) + " # NTP loop filter = reject (1 - median_threshold) samples"

if median_threshold < 0.05 or median_threshold > 1.0:
        print "median threshold needs to be between 0.05 and 1.0"
        exit(1)

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

def get_samples(data, curr_index, TIME_CONSTANT):
        samples = [ ]
        i = curr_index
        curr_timestamp = data[i][0]
        while i >= 0 and data[i][0] + TIME_CONSTANT >= curr_timestamp:
                samples.append(data[i][1])
                i = i - 1
        oldest_timestamp = data[i][0]
        if curr_timestamp - oldest_timestamp >= TIME_CONSTANT:
                samples.sort()
                return samples
        return None

def do_average(s):
        return sum(s, 0.0) / len(s)

def do_median_filter(s):
        s1 = s
        s1.sort()
        curr_len = len(s1)
        threshold_len = len(s1) * median_threshold
        while len(s1) > threshold_len and len(s1) > 2:
                if abs(s1[0]) > abs(s1[-1]):
                        del s1[0]
                else:
                        del s1[-1]
        if len(s1) == 1:
                return s1[0]
        #if len(s1) % 2 == 0:
        #        return avg(s1[len(s1) / 2], s1[len(s1) / 2 + 1])
        #else:
        #        return s1[len(s1) / 2]
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

min_sample = 86400.0
max_sample = -86400.0
min_avg = 86400.0
max_avg = -86400.0
min_median = 86400.0
max_median = -86400.0
median_values = [ ]
median_values_min_sample_size = 1000000000
median_values_max_sample_size = 0
median_values_sum_sample_size = 0
for i in range(0, len(valid_data) - 1):
        samples = get_samples(valid_data, i, TIME_CONSTANT)
        if samples == None:
                continue
        avg = do_average(samples)
        median = do_median_filter(samples)
        spaces = "      "
        s = "#" + str(i) + spaces
        s = s + make_timespec_s(valid_data[i][0]) + spaces
        s = s + make_timefrac_s(valid_data[i][1])
        #####if is_daytime(valid_data[i][0]):
        #####        s = s + spaces + "  DAY"
        #####else:
        #####        s = s + spaces + "NIGHT"
        s = s + spaces + "{0:4d}".format(len(samples))
        if valid_data[i][1] < min_sample:
                min_sample = valid_data[i][1]
        if valid_data[i][1] > max_sample:
                max_sample = valid_data[i][1]
        if avg is not None and median is not None:
                s = s + spaces
                s = s + make_timefrac_s(avg) + spaces
                s = s + make_timefrac_s(median)
                if avg < min_avg:
                        min_avg = avg
                if avg > max_avg:
                        max_avg = avg
                if median < min_median:
                        min_median = median
                if median > max_median:
                        max_median = median
                median_values.append(median)
                if len(samples) < median_values_min_sample_size:
                        median_values_min_sample_size = len(samples)
                if len(samples) > median_values_max_sample_size:
                        median_values_max_sample_size = len(samples)
                median_values_sum_sample_size = median_values_sum_sample_size + float(len(samples)) + 0.0
                s = s + spaces + "{0:4d}".format(len(samples))
        print s

print ""
print "         logfile = " + logfile
print "   TIME_CONSTANT = " + str(TIME_CONSTANT)
print ""
print ""
print "total samples         = " + str(len(all_data))
print "total valid samples   = " + str(len(valid_data))
print "      valid samples   = " + "{0:.2f}%".format(100.0 * len(valid_data) / len(all_data))
print "   filtered samples   = " + str(len(median_values))
print "   filtered samples   = " + "{0:.2f}%".format(100.0 * len(median_values) / len(all_data))
timestamp_range = all_data[len(all_data) - 1][0] - all_data[0][0]
print " first to last sample = " + str(timestamp_range) + " seconds"
print "       average period = " + str(timestamp_range / len(all_data)) + " seconds"
print ""
print ""
print "general stats:"
print "         min_sample   = " + make_timefrac_s(min_sample)
print "         max_sample   = " + make_timefrac_s(max_sample)
print "            min_avg   = " + make_timefrac_s(min_avg)
print "            max_avg   = " + make_timefrac_s(max_avg)
print "         min_median   = " + make_timefrac_s(min_median)
print "         max_median   = " + make_timefrac_s(max_median)
print ""
print ""

median_values.sort()
print "median stats:"
print "       median samples = " + str(len(median_values))
print "     min samples size = " + str(median_values_min_sample_size)
print "     max samples size = " + str(median_values_max_sample_size)
print "     avg samples size = " + str(median_values_sum_sample_size / len(median_values))
if len(median_values) > 10:
        print "           P05 median = " + make_timefrac_s(median_values[int(len(median_values) * 0.05)])
        print "           P10 median = " + make_timefrac_s(median_values[int(len(median_values) * 0.10)])
        print "           P25 median = " + make_timefrac_s(median_values[int(len(median_values) * 0.25)])
        print "           P50 median = " + make_timefrac_s(median_values[int(len(median_values) * 0.50)])
        print "           P75 median = " + make_timefrac_s(median_values[int(len(median_values) * 0.75)])
        print "           P90 median = " + make_timefrac_s(median_values[int(len(median_values) * 0.90)])
        print "           P95 median = " + make_timefrac_s(median_values[int(len(median_values) * 0.95)])
