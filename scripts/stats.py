#!/usr/bin/python

import os
import sys
import time

logfile = sys.argv[1]
N = int(sys.argv[2])

print "logfile = " + logfile
print "N = " + str(N)

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

print "total samples         = " + str(len(all_data))
print "total valid samples   = " + str(len(valid_data))
print "      valid samples   = " + str(100.0 * len(valid_data) / len(all_data)) + "%"
print ""

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
        if len(s1) % 2 == 0:
                return s1[len(s1) / 2]
        else:
                return s1[len(s1) / 2]

def make_timespec_s(timestamp):
        return "{0:09.09f}".format(timestamp)

def make_timefrac_s(timestamp):
        if timestamp < 0:
            return "{0:0.09f}".format(timestamp)
        else:
            return "+{0:0.09f}".format(timestamp)

for i in range(0, len(valid_data) - 1):
        if i >= N - 1:
                samples = get_samples(valid_data, i - N, i)
                avg = do_average(samples)
                median = do_median(samples)
        spaces = "      "
        s = "#" + str(i) + spaces
        s = s + make_timespec_s(valid_data[i][0]) + spaces
        s = s + make_timefrac_s(valid_data[i][1])
        if i >= N - 1:
                s = s + spaces
                s = s + make_timefrac_s(avg) + spaces
                s = s + make_timefrac_s(median)
        print s
